using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;
using System.Threading;
using UnityEngine;
using NetTopologySuite.Geometries;
using PathPlanning;
using Map;
using CentralControl.OrderAssignment;
using CentralControl.PickerControl;

namespace CentralControl.RobotControl
{
    public enum RobotState
    {
        Moving, 
        Picking
    }
    
    public class RobotController : MonoBehaviour
    {
        #region Inspector Fields
        public Transform targetIndicator;
        public SearchAlgorithm selectedAlgorithm = SearchAlgorithm.Astar_Basic;
        public SearchMethod selectedSearchMethod = SearchMethod.Manhattan_Distance;
        public float baseDetectionRadius = 0.7f;
        public float emergencyStopDistance = 0.2f;
        public float defaultSpeed = 2.0f;
        public float maxAcceleration = 2.0f;
        public float maxDeceleration = 5.0f;
        public int MaxOrder = 20;
        #endregion

        #region Public Properties
        public int Id { get; private set; }
        public Graph graph { get; private set; }
        public IndoorSpace indoorSpace { get; private set; }
        public MapGrid mapGrid { get; private set; }
        public bool lastReportedAvailability { get; set; }
        public bool lastReportedFreeStatus { get; set; }
        public int lastReportedOrderCount { get; set; } = -1;
        #endregion

        #region Private Fields
        private DynamicOccupancyLayer personalOccupancyLayer;
        private DynamicOccupancyLayer globalOccupancyLayer;
        private PathPlanner pathPlanner;
        private Rigidbody rb;
        private ConcurrentQueue<Order> ordersQueue = new ConcurrentQueue<Order>();
        private Order currentOrder;
        private Coroutine currentMovementCoroutine;
        private HashSet<RobotState> currentStates = new HashSet<RobotState>();
        private DWAPlanner dwaPlanner;
        private float detectionRadius;
        private bool isWaitingForPicking = false;
        private bool isProcessingOrdersRunning = false;
        private int isProcessingOrder = 0;
        private List<ConnectionPoint> currentPath = new List<ConnectionPoint>();
        private List<PathSegment> currentMovement = new List<PathSegment>();
        private int currentPathIndex = 0;
        private float lastReplanTime = 0f;
        private float replanInterval = 5.0f;
        private bool pickingCompleted = false;
        private string currentPlanId;
        private LayerMask obstacleLayer;
        #endregion

        #region State Properties
        public bool IsMoving => currentStates.Contains(RobotState.Moving);
        public bool IsPicking => currentStates.Contains(RobotState.Picking);
        public bool IsFree => currentStates.Count == 0;
        public bool IsOnTask => !IsFree;
        public bool IsAvailable => ordersQueue.Count < MaxOrder;
        public bool IsProcessingOrders => isProcessingOrdersRunning;
        public int GetRobotOrdersQueueCount => ordersQueue.Count;
        public List<string> listOrdersQueue => new List<string>(ordersQueue.Select(order => order.Id));
        #endregion

        #region Navigation System
        private enum NavigationPhase
        {
            FollowingPath,       // 沿着连接点(CellBoundaries)导航
            ApproachingTarget    // 最终接近目标点(CellSpace内的拣货点)
        }
        private NavigationPhase currentNavigationPhase = NavigationPhase.FollowingPath;
        private Vector3 finalDestination;
        private float pathPointAcceptanceRadius = 0.8f;
        private float finalDestinationAcceptanceRadius = 0.3f;
        private bool hasReportedArrival = false;
        #endregion

        #region DWA Parameters
        private bool isDWAEnabled = false;
        private float lastDWAStateChangeTime = 0f;
        private const float DWA_STATE_COOLDOWN = 2.0f;
        private IntersectionAreaManager intersectionManager;
        private bool isIntersectionControlled = false;
        private string currentIntersectionId;
        #endregion

        #region Unity Lifecycle Methods
        private void Awake()
        {
            rb = GetComponent<Rigidbody>();
            if (rb == null)
            {
                Debug.LogWarning($"Robot {Id}: No Rigidbody found, adding one automatically");
                rb = gameObject.AddComponent<Rigidbody>();
                
                // 设置基本属性
                rb.useGravity = false;  // 通常机器人不需要重力
                rb.isKinematic = false; // 允许物理引擎移动物体
                rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
                rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
                rb.interpolation = RigidbodyInterpolation.Interpolate;
                rb.mass = 1.0f;
                rb.drag = 0.5f;
            }

            detectionRadius = baseDetectionRadius;
            obstacleLayer = LayerMask.GetMask("RobotLayer", "ObstacleLayer");

            if (GetComponent<Collider>() == null)
            {
                CapsuleCollider collider = gameObject.AddComponent<CapsuleCollider>();
                collider.radius = 0.5f;
                collider.height = 1.5f;
                collider.center = new Vector3(0, 0.75f, 0);
            }
        }

        private void Update()
        {
            if (!isInitialized) return;

            UpdateRobotStatus();
            UpdateDynamicObstaclePerception();
            
            if (IsMoving && !isWaitingForPicking)
            {
                UpdateMovement();
            }
        }
        #endregion

        #region Initialization
        public void InitializeRobot(int id, IndoorSpace indoorSpace, Graph graph, MapGrid mapGrid, 
                                    DynamicOccupancyLayer globalOccupancyLayer,
                                    IntersectionAreaManager intersectionManager)
        {
            this.Id = id;
            this.indoorSpace = indoorSpace;
            this.graph = graph;
            this.mapGrid = mapGrid;
            this.globalOccupancyLayer = globalOccupancyLayer;
            this.intersectionManager = intersectionManager;

            personalOccupancyLayer = new DynamicOccupancyLayer(indoorSpace, 300f, Time.time);
            
            pathPlanner = new PathPlanner(graph, indoorSpace, selectedAlgorithm, selectedSearchMethod, 
                                          id.ToString(), maxAcceleration, maxDeceleration);
            
            dwaPlanner = new DWAPlanner();
            dwaPlanner.MaxSpeed = defaultSpeed;
            dwaPlanner.ObstacleLayer = obstacleLayer;
            
            isInitialized = true;
            Debug.Log($"Robot {Id} initialized at {transform.position}");
        }

        public void UpdateGlobalLayer(DynamicOccupancyLayer newGlobalLayer)
        {
            globalOccupancyLayer = newGlobalLayer;
        }
        #endregion

        #region Order Processing
        public void ReceiveOrder(Order order)
        {
            if (order == null)
            {
                Debug.LogError($"Robot {Id} received null order");
                return;
            }

            ordersQueue.Enqueue(order);
            Debug.Log($"Robot {Id} received order {order.Id}, queue size: {ordersQueue.Count}");
            
            if (!isProcessingOrdersRunning)
            {
                StartCoroutine(ProcessOrders());
            }
        }

        private IEnumerator ProcessOrders()
        {
            if (Interlocked.Exchange(ref isProcessingOrder, 1) == 1)
            {
                yield break;
            }
            
            isProcessingOrdersRunning = true;
            
            while (ordersQueue.Count > 0)
            {
                // Get next order
                if (!ordersQueue.TryDequeue(out currentOrder))
                {
                    Debug.LogError($"Robot {Id} failed to dequeue order");
                    break;
                }
                
                Debug.Log($"Robot {Id} processing order {currentOrder.Id}, destination: {currentOrder.Destination}");
                
                // Execute order
                yield return StartCoroutine(ExecuteOrderCoroutine(currentOrder));
                
                // Order completed
                Debug.Log($"Robot {Id} completed order {currentOrder.Id}");
                currentOrder = null;
            }
            
            isProcessingOrdersRunning = false;
            Interlocked.Exchange(ref isProcessingOrder, 0);
            
            if (IsFree && !lastReportedFreeStatus)
            {
                Debug.Log($"Robot {Id} has no more orders and is now free");
                CentralController.NotifyRobotFree(this);
                lastReportedFreeStatus = true;
            }
        }

        private IEnumerator ExecuteOrderCoroutine(Order order)
        {
            // Plan path to destination
            Debug.Log($"Robot {Id} planning path to {order.Destination}");
            SetState(RobotState.Moving, true);
            
            
            Vector3 targetPosition = GetTargetPosition(order.PickingPoint);
            var result = PlanPath(targetPosition);
            if (result.path == null || result.path.Count == 0)
            {
                Debug.LogError($"Robot {Id} failed to plan path to {order.Destination}");
                SetState(RobotState.Moving, false);
                yield break;
            }
            
            currentPlanId = result.planId;
            UpdateCurrentMovementInfo(result.path, result.times, result.speeds);
            
            // Wait until destination reached or timeout
            float timeout = 300f;  // 5 minutes timeout
            float startTime = Time.time;
            
            while (!hasReportedArrival && Time.time - startTime < timeout)
            {
                yield return null;
            }
            
            if (!hasReportedArrival)
            {
                Debug.LogWarning($"Robot {Id} timed out while trying to reach destination");
                SetState(RobotState.Moving, false);
                yield break;
            }
            
            // Start picking process
            Debug.Log($"Robot {Id} waiting for picking at {transform.position}");
            SetState(RobotState.Moving, false);
            SetState(RobotState.Picking, true);
            
            pickingCompleted = false;
            yield return StartCoroutine(WaitForPicking(order, transform.position));
            
            // Picking completed, order executed
            SetState(RobotState.Picking, false);
            
            Debug.Log($"Robot {Id} completed picking for order {order.Id}");
        }

        private IEnumerator WaitForPicking(Order order, Vector3 location)
        {
            isWaitingForPicking = true;
            rb.velocity = Vector3.zero; // Stop moving
            
            // Notify picker
            EventManager.Instance.TriggerRobotReadyForPicking(Id.ToString(), order.PickingPoint, location);
            
            // Wait until picking is completed
            float timeout = 180f;  // 3 minutes timeout
            float startTime = Time.time;
            
            while (!pickingCompleted && Time.time - startTime < timeout)
            {
                yield return null;
            }
            
            if (!pickingCompleted)
            {
                Debug.LogWarning($"Robot {Id} timed out waiting for picking");
            }
            
            isWaitingForPicking = false;
        }

        public void NotifyPickingCompleted()
        {
            pickingCompleted = true;
            Debug.Log($"Robot {Id} notified that picking was completed");
        }
        #endregion

        #region Movement System
        private Vector3 GetTargetPosition(PickingPoint pickingPoint)
        {
            CellSpace cellSpace = indoorSpace.GetCellSpaceFromId(pickingPoint.Id);
            Point point = (Point)cellSpace.Node;
            return new Vector3((float)point.X, mapGrid.height, (float)point.Y);
        }

        private (List<ConnectionPoint> path, List<float> times, List<float> speeds, string planId) PlanPath(Vector3 targetPosition, string currentPlanId = null)
        {
            currentPath.Clear();
            currentMovement.Clear();
            finalDestination = targetPosition;
            currentNavigationPhase = NavigationPhase.FollowingPath;
            hasReportedArrival = false;

            RoutePoint startPoint = graph.GetRoutePointFromCoordinate(transform.position, true);
            RoutePoint endPoint = graph.GetRoutePointFromCoordinate(targetPosition, false);
            Debug.Log($"Robot {Id}: Planning path from {transform.position} to {targetPosition}");

            if (startPoint == null || endPoint == null)
            {
                Debug.LogError($"Robot {Id} cannot find valid route points");
                return (null, null, null, null);
            }
            
            var result = pathPlanner.FindPathWithDynamicObstacles(
                personalOccupancyLayer,
                globalOccupancyLayer,
                startPoint,
                endPoint,
                Id.ToString(),
                currentPlanId,
                Time.time,
                defaultSpeed
            );
            
            return result;
        }

        private void UpdateMovement()
        {
            if (currentPath == null || currentPath.Count == 0 || isWaitingForPicking)
                return;

            // 根据当前导航阶段选择目标点和行为
            Vector3 currentTarget;
            float acceptanceRadius;

            switch (currentNavigationPhase)
            {
                case NavigationPhase.FollowingPath:
                    // 确保索引有效
                    if (currentPathIndex >= currentPath.Count)
                    {
                        SwitchToApproachingTarget();
                        return;
                    }

                    // 目标是下一个连接点
                    currentTarget = GetPointVector3((Point)currentPath[currentPathIndex].Point);
                    acceptanceRadius = pathPointAcceptanceRadius;

                    // 检查是否到达当前连接点
                    if (Vector3.Distance(transform.position, currentTarget) < acceptanceRadius)
                    {
                        // 前往下一个连接点
                        currentPathIndex++;
                        
                        // 如果到达最后一个连接点，切换到接近目标阶段
                        if (currentPathIndex >= currentPath.Count)
                        {
                            SwitchToApproachingTarget();
                            return;
                        }
                    }
                    break;
                    
                case NavigationPhase.ApproachingTarget:
                    // 目标是实际的拣货点
                    currentTarget = finalDestination;
                    
                    // 使用较小的接受半径用于最终目的地
                    acceptanceRadius = finalDestinationAcceptanceRadius;
                    
                    // 检查是否到达最终目标
                    if (HasReachedFinalDestination())
                    {
                        HandleDestinationReached();
                        return;
                    }
                    break;
                    
                default:
                    return;
            }

            // 根据导航阶段执行移动
            MoveTowards(currentTarget);
            CheckIntersectionStatus();
            
            // 检查是否偏离路径
            if (IsOffPath() && currentNavigationPhase == NavigationPhase.FollowingPath)
            {
                CorrectPathDeviation();
            }
        }

        private void SwitchToApproachingTarget()
        {
            currentNavigationPhase = NavigationPhase.ApproachingTarget;
            Debug.Log($"Robot {Id}: Switching to approaching target phase. Target: {finalDestination}");
        }

        private bool HasReachedFinalDestination()
        {
            if (currentOrder == null)
                return false;
                
            CellSpace cellSpace = indoorSpace.GetCellSpaceFromId(currentOrder.PickingPoint.Id);
            if (cellSpace == null) 
                return false;
            
            // 检查是否在包含拣货点的单元格内
            CellSpace currentCell = indoorSpace.GetCellSpaceFromCoordinates(transform.position);
            if (currentCell != null && currentCell.Id == cellSpace.Id)
            {
                // 在正确的单元格内，现在检查与拣货点中心的距离
                float distanceToPickingPoint = Vector3.Distance(transform.position, finalDestination);
                return distanceToPickingPoint < finalDestinationAcceptanceRadius;
            }
            
            return false;
        }

        private void HandleDestinationReached()
        {
            if (!hasReportedArrival && currentOrder != null)
            {
                Debug.Log($"Robot {Id}: Reached picking point {currentOrder.PickingPoint.Id}");
                isWaitingForPicking = true;
                hasReportedArrival = true;
                
                // 停止移动
                rb.velocity = Vector3.zero;
            }
        }

        private void MoveTowards(Vector3 target)
        {
            // 根据导航阶段使用不同的移动参数
            float dwaActivationDistance;
            float lookAheadDistance;
            
            if (currentNavigationPhase == NavigationPhase.FollowingPath)
            {
                // 路径跟随阶段的宽松参数
                dwaActivationDistance = 1.8f;
                lookAheadDistance = 2.5f;
            }
            else
            {
                // 最终接近阶段的精确参数
                dwaActivationDistance = 1.0f;
                lookAheadDistance = 1.2f;
            }
            
            // 只在需要时激活DWA
            bool shouldUseDWA = ShouldActivateDWA(dwaActivationDistance);
            
            if (shouldUseDWA)
            {
                // 配置当前阶段的DWA
                dwaPlanner.PredictionTime = currentNavigationPhase == NavigationPhase.FollowingPath ? 2.0f : 1.2f;
                dwaPlanner.HeadingWeight = currentNavigationPhase == NavigationPhase.FollowingPath ? 0.6f : 0.5f;
                dwaPlanner.VelocityWeight = currentNavigationPhase == NavigationPhase.FollowingPath ? 0.2f : 0.15f;
                
                var (velocity, needReplan) = dwaPlanner.CalculateVelocity(transform.position, rb.velocity, target);
                
                // 平滑应用速度
                Vector3 targetVelocity = velocity.normalized * 
                    Mathf.Min(velocity.magnitude, currentNavigationPhase == NavigationPhase.FollowingPath ? defaultSpeed : defaultSpeed * 0.7f);
                
                rb.velocity = Vector3.Lerp(rb.velocity, targetVelocity, 0.3f);
                
                // 如果DWA报告需要重新规划路径
                if (needReplan && Time.time - lastReplanTime > replanInterval)
                {
                    lastReplanTime = Time.time;
                    StartCoroutine(ReplanPath());
                }
            }
            else
            {
                // 直接路径跟随，带有航向修正
                DirectPathFollow(target, lookAheadDistance);
            }
        }

        private void DirectPathFollow(Vector3 target, float lookAheadDistance)
        {
            // 计算到目标的方向
            Vector3 directionToTarget = (target - transform.position).normalized;
            
            // 获取当前单元格
            CellSpace currentCell = indoorSpace.GetCellSpaceFromCoordinates(transform.position);
            
            // 计算适当的速度
            float speedMultiplier = 1.0f;
            
            // 如果接近转弯点，减速
            if (currentPathIndex < currentPath.Count - 1 && currentNavigationPhase == NavigationPhase.FollowingPath)
            {
                Vector3 nextPoint = currentPathIndex + 1 < currentPath.Count ? 
                    GetPointVector3((Point)currentPath[currentPathIndex + 1].Point) : target;
                
                Vector3 currentDirection = directionToTarget;
                Vector3 nextDirection = (nextPoint - target).normalized;
                
                float turnAngle = Vector3.Angle(currentDirection, nextDirection);
                if (turnAngle > 30f)
                {
                    speedMultiplier = Mathf.Lerp(1.0f, 0.5f, turnAngle / 90f);
                }
            }
            
            // 如果在狭窄空间或接近单元边界，减速
            if (IsNearCellBoundary(currentCell))
            {
                speedMultiplier *= 0.8f;
            }
            
            // 计算目标速度
            Vector3 targetVelocity = directionToTarget * defaultSpeed * speedMultiplier;
            
            // 平滑应用速度变化
            rb.velocity = Vector3.Lerp(rb.velocity, targetVelocity, 0.2f);
            
            // 确保机器人朝向移动方向
            if (rb.velocity.magnitude > 0.1f)
            {
                Quaternion targetRotation = Quaternion.LookRotation(rb.velocity.normalized);
                transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, 0.1f);
            }
        }

        private bool ShouldActivateDWA(float activationDistance)
        {
            // Check for nearby obstacles
            bool nearObstacle = Physics.CheckSphere(transform.position, activationDistance, obstacleLayer);
            
            // Check if near intersection
            bool nearIntersection = false;
            if (intersectionManager != null)
            {
                nearIntersection = intersectionManager.IsNearIntersection(transform.position, activationDistance);
            }
            
            // Cooldown DWA activation
            if (isDWAEnabled && Time.time - lastDWAStateChangeTime < DWA_STATE_COOLDOWN)
                return true;
                
            bool shouldActivate = nearObstacle || nearIntersection;

            if (shouldActivate)
            {
                // If activate DWA due to nearby obstacle, log
                if (nearObstacle && !isDWAEnabled)
                    Debug.Log($"Robot {Id}: Activating DWA due to nearby obstacle");
            }
            else
            {
                // Reset obstacle weight
                dwaPlanner.ObstacleWeight = 0.15f;
            }
            
            // Record DWA state change and time
            if (shouldActivate != isDWAEnabled)
            {
                lastDWAStateChangeTime = Time.time;
                isDWAEnabled = shouldActivate;
            }
            
            return shouldActivate;
        }

        private bool IsOffPath()
        {
            if (currentPath == null || currentPath.Count == 0)
                return false;
                
            // Find closest point on path
            Vector3 currentPos = transform.position;
            Vector3 closestPoint = FindClosestPointOnPath(currentPos);
            
            float distanceFromPath = Vector3.Distance(currentPos, closestPoint);
            
            // Allow larger deviation in open cells, smaller near boundaries
            CellSpace currentCell = indoorSpace.GetCellSpaceFromCoordinates(currentPos);
            float deviationThreshold = IsNearCellBoundary(currentCell) ? 0.5f : 1.0f;
            
            return distanceFromPath > deviationThreshold;
        }

        private Vector3 FindClosestPointOnPath(Vector3 position)
        {
            Vector3 closestPoint = Vector3.zero;
            float closestDistance = float.MaxValue;
            
            // Check current and next few path segments for closest point
            int startIdx = Mathf.Max(0, currentPathIndex - 1);
            int endIdx = Mathf.Min(currentPath.Count - 1, currentPathIndex + 2);
            
            for (int i = startIdx; i < endIdx; i++)
            {
                if (i + 1 >= currentPath.Count) continue;
                
                Vector3 segmentStart = GetPointVector3((Point)currentPath[i].Point);
                Vector3 segmentEnd = GetPointVector3((Point)currentPath[i + 1].Point);
                
                Vector3 pointOnSegment = FindClosestPointOnSegment(position, segmentStart, segmentEnd);
                float distance = Vector3.Distance(position, pointOnSegment);
                
                if (distance < closestDistance)
                {
                    closestDistance = distance;
                    closestPoint = pointOnSegment;
                }
            }
            
            return closestPoint;
        }

        private Vector3 FindClosestPointOnSegment(Vector3 point, Vector3 lineStart, Vector3 lineEnd)
        {
            Vector3 line = lineEnd - lineStart;
            float lineLength = line.magnitude;
            line.Normalize();

            Vector3 v = point - lineStart;
            float d = Vector3.Dot(v, line);
            d = Mathf.Clamp(d, 0f, lineLength);
            
            return lineStart + line * d;
        }

        private bool IsNearCellBoundary(CellSpace cellSpace)
        {
            if (cellSpace == null) return false;
            
            foreach (var boundary in indoorSpace.CellBoundaries)
            {
                if (boundary.Source == cellSpace.Id || boundary.Target == cellSpace.Id)
                {
                    Vector3 boundaryPos = new Vector3(
                        (float)boundary.Boundary.Centroid.X,
                        transform.position.y,
                        (float)boundary.Boundary.Centroid.Y
                    );
                    
                    if (Vector3.Distance(transform.position, boundaryPos) < 1.0f)
                        return true;
                }
            }
            
            return false;
        }

        private void CorrectPathDeviation()
        {
            // Find closest point on path
            Vector3 currentPos = transform.position;
            Vector3 closestPathPoint = FindClosestPointOnPath(currentPos);
            
            // Adjust velocity directly towards path
            Vector3 correctionDirection = (closestPathPoint - currentPos).normalized;
            
            Vector3 correctionVelocity = correctionDirection * defaultSpeed;
            rb.velocity = Vector3.Lerp(rb.velocity, correctionVelocity, 0.3f);
            
            Debug.Log($"Robot {Id}: Correcting path deviation. Distance from path: {Vector3.Distance(currentPos, closestPathPoint)}");
            
            // Replan path if deviation is too large
            if (Vector3.Distance(currentPos, closestPathPoint) > 2.0f && Time.time - lastReplanTime > replanInterval)
            {
                lastReplanTime = Time.time;
                StartCoroutine(ReplanPath());
            }
        }

        private IEnumerator ReplanPath()
        {
            Debug.Log($"Robot {Id}: Replanning path to destination");
            
            if (currentOrder == null)
            {
                Debug.LogWarning($"Robot {Id}: No current order to replan path for");
                yield break;
            }
            
            var result = PlanPath(currentOrder.Destination, currentPlanId);
            if (result.path == null || result.path.Count == 0)
            {
                Debug.LogError($"Robot {Id}: Failed to replan path");
                yield break;
            }
            
            currentPlanId = result.planId;
            UpdateCurrentMovementInfo(result.path, result.times, result.speeds);
            
            Debug.Log($"Robot {Id}: Path replanned successfully");
        }

        private Vector3 GetPointVector3(Point point)
        {
            return new Vector3((float)point.X, transform.position.y, (float)point.Y);
        }

        private void UpdateCurrentMovementInfo(List<ConnectionPoint> path, List<float> times, List<float> speeds)
        {
            if (path == null || times == null || speeds == null)
            {
                Debug.LogError($"Robot {Id}: Null path or timing information");
                return;
            }

            if (path.Count != times.Count || path.Count != speeds.Count)
            {
                Debug.LogError($"Robot {Id}: Mismatched array lengths - Path: {path.Count}, Times: {times.Count}, Speeds: {speeds.Count}");
                return;
            }

            currentPathIndex = 0;
            currentPath.Clear();
            currentPath.AddRange(path);
            currentMovement.Clear();

            for (int i = 0; i < path.Count; i++)
            {
                currentMovement.Add(new PathSegment(path[i], times[i], speeds[i]));
            }

            dwaPlanner.MaxSpeed = speeds.Max();
        }
        #endregion

        #region Status Management
        private void UpdateRobotStatus()
        {
            bool previousFreeStatus = IsFree;
            
            if (previousFreeStatus != IsFree || lastReportedOrderCount != ordersQueue.Count)
            {
                lastReportedOrderCount = ordersQueue.Count;
                lastReportedFreeStatus = IsFree;
            }

            if (ordersQueue.Count > 0 && !isProcessingOrdersRunning)
            {
                StartCoroutine(ProcessOrders());
            }
        }

        private void SetState(RobotState state, bool value)
        {
            if (value)
            {
                currentStates.Add(state);
            }
            else
            {
                currentStates.Remove(state);
            }
        }

        public void ResetRobot()
        {
            StopAllCoroutines();
            SetState(RobotState.Moving, false);
            SetState(RobotState.Picking, false);
            isProcessingOrdersRunning = false;
            Interlocked.Exchange(ref isProcessingOrder, 0);
            
            currentPath.Clear();
            currentMovement.Clear();
            
            while (ordersQueue.TryDequeue(out _)) { }
            
            currentOrder = null;
            rb.velocity = Vector3.zero;
            
            Debug.Log($"Robot {Id} has been reset");
        }
        #endregion

        #region Occupancy Layer
        private void UpdateDynamicObstaclePerception()
        {
            float currentTime = Time.time;
            
            Collider[] nearbyColliders = Physics.OverlapSphere(transform.position, detectionRadius, obstacleLayer);
            
            foreach (var collider in nearbyColliders)
            {
                if (collider.gameObject == this.gameObject)
                    continue;
                    
                CellSpace obstacleCellSpace = indoorSpace.GetCellSpaceFromCoordinates(collider.transform.position);
                if (obstacleCellSpace == null) continue;
                
                if (personalOccupancyLayer != null)
                {
                    string obstaclerId = "unknown";
                    RobotController otherRobot = collider.GetComponent<RobotController>();
                    if (otherRobot != null)
                        obstaclerId = otherRobot.Id.ToString();
                        
                    personalOccupancyLayer.SetDynamicOccupancy(
                        obstacleCellSpace.Id,
                        currentTime,
                        currentTime + 0.5f, 
                        obstaclerId
                    );
                }
            }
        }
    
        public DynamicOccupancyLayer GetPersonalOccupancyLayer()
        {
            return personalOccupancyLayer;
        }
        
        public void UpdatePersonalLayer()
        {
            CellSpace currentCellSpace = indoorSpace.GetCellSpaceFromCoordinates(transform.position);
            if (currentCellSpace == null)
                return;

            float currentTime = Time.time;
            
            if (lastCellSpace == null)
            {
                lastCellSpace = currentCellSpace;
                return;
            }

            // Check if robot has transitioned to a new cell space
            if (currentCellSpace.Id != lastCellSpace.Id)
            {
                Debug.Log($"Robot {Id}: Transitioning from CellSpace {lastCellSpace.Id} to {currentCellSpace.Id}");
                
                // Update occupancy for last cell space
                if (cellSpaceEntryTimes.TryGetValue(lastCellSpace.Id, out float oldEntryTime))
                {
                    personalOccupancyLayer.SetPlannedOccupancy(
                        lastCellSpace.Id,
                        oldEntryTime,
                        currentTime,
                        Id.ToString(),
                        "RealTimeMovement"
                    );
                    cellSpaceEntryTimes.Remove(lastCellSpace.Id);
                }
                
                // Record entry time for new cell space
                cellSpaceEntryTimes[currentCellSpace.Id] = currentTime;
                lastCellSpace = currentCellSpace;
                return;
            }

            // Update occupancy for current cell space
            if (cellSpaceEntryTimes.TryGetValue(currentCellSpace.Id, out float entryTime))
            {
                personalOccupancyLayer.SetPlannedOccupancy(
                    currentCellSpace.Id,
                    entryTime,
                    currentTime,
                    Id.ToString(),
                    "RealTimeMovement"
                );
            }
        }
        
        public void ClearPathReservations()
        {
            if (!string.IsNullOrEmpty(currentPlanId))
            {
                foreach (var cellSpace in indoorSpace.CellSpaces)
                {
                    personalOccupancyLayer.ClearPlannedOccupancy(
                        cellSpace.Id,
                        Id.ToString(),
                        currentPlanId,
                        Time.time
                    );
                }
            }

            cellSpaceEntryTimes.Clear();
            lastCellSpace = null;
            currentPlanId = null;
        }
        #endregion

        #region Intersection Management
        private void EnterIntersection(IntersectionArea intersection)
        {
            if (intersection == null) return;
            
            isIntersectionControlled = true;
            currentIntersectionId = intersection.Id;
            
            // Get entry and suggested exit directions
            var (entryDirection, suggestedExitDirection) = intersection.CalculateTraversalDirections(
                transform.position, rb.velocity);
            
            // Adjust DWA parameters for intersection traversal
            dwaPlanner.HeadingWeight = 0.7f;
            dwaPlanner.MaxSpeed = defaultSpeed * 0.8f;
            
            Debug.Log($"Robot {Id} entered intersection {intersection.Id}, entry direction: {entryDirection}, " +
                    $"suggested exit: {suggestedExitDirection}");
        }

        private void ExitIntersection()
        {
            if (currentIntersectionId == null) return;
            
            isIntersectionControlled = false;
            
            // Reset DWA parameters
            dwaPlanner.HeadingWeight = 0.6f;
            dwaPlanner.MaxSpeed = defaultSpeed;
            
            Debug.Log($"Robot {Id} exited intersection {currentIntersectionId}");
            currentIntersectionId = null;
        }

        private void CheckIntersectionStatus()
        {
            if (intersectionManager == null) return;
            
            IntersectionArea intersection = intersectionManager.GetIntersectionAtPosition(transform.position);
            
            // If entered a new intersection
            if (intersection != null && (currentIntersectionId == null || intersection.Id != currentIntersectionId))
            {
                // If exiting a previous intersection
                if (currentIntersectionId != null)
                {
                    ExitIntersection();
                }
                
                // Enter new intersection
                EnterIntersection(intersection);
            }
            // If exited an intersection
            else if (intersection == null && currentIntersectionId != null)
            {
                ExitIntersection();
            }
        }
        #endregion

        #region Collision Detection
        private void CheckSurroundings()
        {
            // 检测周围环境的障碍物
            Collider[] hitColliders = Physics.OverlapSphere(transform.position, detectionRadius, obstacleLayer);
            bool emergencySituation = false;
            
            foreach (var hitCollider in hitColliders)
            {
                // 忽略自身碰撞体
                if (hitCollider.gameObject == gameObject)
                    continue;
                    
                // 获取另一个机器人
                RobotController otherRobot = hitCollider.GetComponent<RobotController>();
                if (otherRobot != null)
                {
                    float distance = Vector3.Distance(transform.position, otherRobot.transform.position);
                    
                    // 如果非常接近，可能需要紧急制动
                    if (distance < emergencyStopDistance)
                    {
                        emergencySituation = true;
                        
                        // 执行紧急避险操作，如减速或停止
                        EmergencyStop();
                        break;
                    }
                }
            }
            
            // 如果没有紧急情况但启用了紧急制动，则恢复正常运行
            if (!emergencySituation && isEmergencyStopped)
            {
                ResumeFromEmergencyStop();
            }
        }
        
        private bool isEmergencyStopped = false;
        
        private void EmergencyStop()
        {
            if (!isEmergencyStopped)
            {
                isEmergencyStopped = true;
                rb.velocity = Vector3.zero;
                Debug.Log($"Robot {Id} executing emergency stop");
            }
        }
        
        private void ResumeFromEmergencyStop()
        {
            isEmergencyStopped = false;
            Debug.Log($"Robot {Id} resuming from emergency stop");
        }
        #endregion

        #region Visualization
        private void OnDrawGizmos()
        {
            if (!Application.isPlaying) return;
            
            // 可视化探测半径
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(transform.position, detectionRadius);
            
            // 可视化探测范围
            Gizmos.color = new Color(1, 1, 0, 0.1f); 
            Gizmos.DrawSphere(transform.position, detectionRadius);
            
            // 显示机器人ID和状态
            UnityEditor.Handles.Label(transform.position + Vector3.up * 2, 
                $"Robot {Id}\nState: {(IsMoving ? "Moving" : IsPicking ? "Picking" : "Idle")}\nOrders: {ordersQueue.Count}");
            
            // 可视化当前速度
            if (rb != null)
            {
                Gizmos.color = Color.blue;
                Gizmos.DrawLine(transform.position, transform.position + rb.velocity * 0.5f);
            }
            
            // 可视化当前目标方向
            if (currentOrder != null && finalDestination != Vector3.zero)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawRay(transform.position, (finalDestination - transform.position).normalized);
            }
            
            // 可视化路径
            if (currentPath != null && currentPath.Count > 0)
            {
                Gizmos.color = Color.cyan;
                
                // 绘制整个路径
                for (int i = 0; i < currentPath.Count - 1; i++)
                {
                    Vector3 start = GetPointVector3((Point)currentPath[i].Point);
                    Vector3 end = GetPointVector3((Point)currentPath[i + 1].Point);
                    
                    Gizmos.DrawLine(start, end);
                }
                
                // 高亮显示当前路径段
                if (currentPathIndex < currentPath.Count)
                {
                    Vector3 currentPos = transform.position;
                    Vector3 targetPos = GetPointVector3((Point)currentPath[currentPathIndex].Point);
                    
                    Gizmos.color = Color.green;
                    Gizmos.DrawLine(currentPos, targetPos);
                    Gizmos.DrawSphere(targetPos, 0.2f);
                }
                
                // 显示最终目标
                if (finalDestination != Vector3.zero)
                {
                    Gizmos.color = Color.red;
                    Gizmos.DrawSphere(finalDestination, 0.3f);
                }
            }
        }
        
        public void UpdateTargetIndicator()
        {
            if (targetIndicator != null)
            {
                if (currentOrder != null && finalDestination != Vector3.zero)
                {
                    targetIndicator.gameObject.SetActive(true);
                    targetIndicator.position = finalDestination;
                }
                else
                {
                    targetIndicator.gameObject.SetActive(false);
                }
            }
        }
        #endregion
        
        #region Helper Methods
        private bool isInitialized = false;
        private CellSpace lastCellSpace = null;
        private Dictionary<string, float> cellSpaceEntryTimes = new Dictionary<string, float>();
        private Vector3 currentPosition;
        
        public struct PathSegment
        {
            public ConnectionPoint Point;
            public float Time;
            public float Speed;

            public PathSegment(ConnectionPoint point, float time, float speed)
            {
                Point = point;
                Time = time;
                Speed = speed;
            }
        }
        #endregion
    }
}