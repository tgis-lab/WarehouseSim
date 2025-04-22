using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;
using System.Threading;
using UnityEngine;
using UnityEngine.AI;
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
        # region Inspector Fields
        public Transform targetIndicator;
        public SearchAlgorithm selectedAlgorithm = SearchAlgorithm.Astar_Basic;
        public SearchMethod selectedSearchMethod = SearchMethod.Manhattan_Distance;
        public float baseDetectionRadius = 0.7f;
        public float emergencyStopDistance = 0.2f;
        public float defaultSpeed = 2.0f;
        public float maxAcceleration = 2.0f;
        public float maxDeceleration = 5.0f;
        public int MaxOrder = 20;
        # endregion

        # region Public Properties
        public int Id { get; private set; }
        public Graph graph { get; private set; }
        public IndoorSpace indoorSpace { get; private set; }
        public MapGrid mapGrid { get; private set; }
        public bool lastReportedAvailability { get; set; } = false;
        public bool lastReportedFreeStatus { get; set; } = false;
        public int lastReportedOrderCount { get; set; } = -1;
        # endregion

        # region Private Fields
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
        # endregion

        # region State Properties
        public bool IsMoving => currentStates.Contains(RobotState.Moving);
        public bool IsPicking => currentStates.Contains(RobotState.Picking);
        public bool IsFree => currentStates.Count == 0;
        public bool IsOnTask => !IsFree;
        public bool IsAvailable => ordersQueue.Count < MaxOrder;
        public bool IsProcessingOrders => isProcessingOrdersRunning;
        public Order CurrentOrder => currentOrder;
        public int GetRobotOrdersQueueCount => ordersQueue.Count;
        public List<string> listOrdersQueue => new List<string>(ordersQueue.Select(order => order.Id));
        private Vector3 lastPosition;
        private float lastPositionChangeTime;
        # endregion

        # region Navigation System
        private enum NavigationPhase
        {
            FollowingPath,
            ApproachingTarget
        }
        private NavigationPhase currentNavigationPhase = NavigationPhase.FollowingPath;
        private Vector3 finalDestination;
        private Vector3 targetPosition;
        private float pathPointAcceptanceRadius = 0.8f;
        private float finalDestinationAcceptanceRadius = 0.3f;
        private bool hasReportedArrival = false;
        private bool navigationEnabled = true;
        private bool isEmergencyStopped = false;
        private float lastEmergencyStopTime = 0f;
        private const float EMERGENCY_COOLDOWN = 3.0f;
        public LayerMask robotCollisionLayer;
        public LayerMask pickerCollisionLayer;
        # endregion

        # region DWA Parameters
        private bool isDWAEnabled = false;
        private float lastDWAStateChangeTime = 0f;
        private const float DWA_STATE_COOLDOWN = 2.0f;
        private IntersectionAreaManager intersectionManager;
        private bool isIntersectionControlled = false;
        private string currentIntersectionId;
        # endregion

        # region Path Smoothing
        private List<ConnectionPoint> originalPath = new List<ConnectionPoint>();
        private List<Vector3> smoothPositions = new List<Vector3>();
        private List<float> smoothTimes = new List<float>();
        private List<float> smoothSpeeds = new List<float>();
        private List<bool> isAuxiliaryPoint = new List<bool>();
        private int smoothPathIndex = 0;
        # endregion

        # region Path Correction
        private bool needsPathRecovery = false;
        private int pathRecoveryAttempts = 0;
        private float lastPathRecoveryTime = 0f;
        private const int MAX_RECOVERY_ATTEMPTS = 3;
        private const float RECOVERY_INTERVAL = 2.0f;
        # endregion

        # region Conflict Resolution
        private Coroutine waitForExecutionCoroutine;
        private bool isWaitingForExecution = false;
        private List<float> pathTimes = new List<float>();
        # endregion

        # region Collision Avoidance
        private List<GameObject> recentCollisions = new List<GameObject>();
        private float collisionClearInterval = 5.0f;
        private float lastCollisionClearTime = 0f;
        # endregion

        # region Event
        public event EventHandler<RobotPathEventArgs> OnPathPlanned;
        protected virtual void RaisePathPlannedEvent(List<ConnectionPoint> path, List<float> times, string planId)
        {
            OnPathPlanned?.Invoke(this, new RobotPathEventArgs(Id.ToString(), planId, path, times));
        }
        # endregion

        # region Unity Lifecycle Methods
        private void Awake()
        {
            rb = GetComponent<Rigidbody>();
            if (rb == null)
            {
                rb = gameObject.AddComponent<Rigidbody>();
            }
            
            // Configure Rigidbody
            rb.useGravity = false;
            rb.drag = 2.0f;
            rb.angularDrag = 5.0f;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
            rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
            rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
            
            if (GetComponent<Collider>() == null)
            {
                BoxCollider collider = gameObject.AddComponent<BoxCollider>();

                collider.size = new Vector3(2.5f, 0.8f, 5.4f); // AGV size
                collider.center = new Vector3(0, 0.1f, 0.05f); // Adjust collider position
            }
            
            detectionRadius = baseDetectionRadius;
            obstacleLayer = LayerMask.GetMask("RobotLayer", "PickerLayer", "CollisionLayer", "BoundaryWall");
            
            recentCollisions = new List<GameObject>();
            lastCollisionClearTime = Time.time;
        }

        private void Update()
        {
            if (!isInitialized) return;

            // Update personal occupancy layer
            UpdatePersonalLayer();
            
            // Update robot status
            UpdateRobotStatus();
            
            // Collision Detection
            CheckSurroundings();

            // Update movement info
            lastPosition = transform.position;
            lastPositionChangeTime = Time.time;
            
            // Not moving if emergency stopped
            if (navigationEnabled && !isEmergencyStopped && IsMoving && !isWaitingForPicking)
            {
                UpdateMovement();
            }
            
            // Update visualization
            UpdateTargetIndicator();
            
            if (isEmergencyStopped || needsPathRecovery)
            {
                VisualizeEmergencyState();
            }
        }

        private void VisualizeEmergencyState()
        {
            // 将在Editor模式下运行的可视化
            // 在游戏中可以通过材质变色或闪烁效果表示紧急状态
            Debug.DrawLine(transform.position, transform.position + Vector3.up * 2, Color.red);
            Debug.DrawLine(transform.position, transform.position + transform.forward * 1.5f, Color.red);
        }

        private void OnDestroy()
        {
            if (EventManager.Instance != null)
            {
                EventManager.Instance.OnRobotPathModified -= HandlePathModified;
                EventManager.Instance.OnRobotPathAccepted -= HandlePathAccepted;
                EventManager.Instance.OnPickingCompleted -= HandlePickingCompletedEvent;
            }
        }
        # endregion

        # region Initialization
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

            ConfigureNavMeshObstacle();

            CreateInteractionTrigger();

            StartCoroutine(ManageObstacleCarving());

            personalOccupancyLayer = new DynamicOccupancyLayer(indoorSpace, 300f, Time.time);
            
            pathPlanner = new PathPlanner(graph, indoorSpace, selectedAlgorithm, selectedSearchMethod, 
                                          id.ToString(), maxAcceleration, maxDeceleration);
            
            dwaPlanner = new DWAPlanner();
            dwaPlanner.MaxSpeed = defaultSpeed;
            dwaPlanner.ObstacleLayer = obstacleLayer;

            if (EventManager.Instance != null)
            {
                EventManager.Instance.OnRobotPathModified += HandlePathModified;
                EventManager.Instance.OnRobotPathAccepted += HandlePathAccepted;
                EventManager.Instance.OnPickingCompleted += HandlePickingCompletedEvent;
            }
            else
            {
                Debug.LogError($"Robot {Id}: EventManager not found, path modification events will not be received");
            }
            
            isInitialized = true;
            Debug.Log($"Robot {Id} initialized at {transform.position}");
        }

        private void ConfigureNavMeshObstacle()
        {
            NavMeshObstacle obstacle = GetComponent<NavMeshObstacle>();
            if (obstacle == null)
            {
                obstacle = gameObject.AddComponent<NavMeshObstacle>();
            }
            
            obstacle.carving = true;
            obstacle.carveOnlyStationary = false;
            
            obstacle.shape = NavMeshObstacleShape.Box;
            obstacle.size = new Vector3(1.5f, 0.8f, 2.0f);
            obstacle.center = new Vector3(0, 0.1f, 0.05f);
            
            obstacle.carvingMoveThreshold = 0.5f;
            obstacle.carvingTimeToStationary = 0.1f;
        }

        public void UpdateGlobalLayer(DynamicOccupancyLayer newGlobalLayer)
        {
            globalOccupancyLayer = newGlobalLayer;
        }
        # endregion

        # region Order Processing
        public void ReceiveOrder(Order order)
        {
            if (order == null)
            {
                Debug.LogError($"Robot {Id} received null order");
                return;
            }

            if (ordersQueue.Count >= MaxOrder)
            {
                Debug.LogError($"Robot {Id} is full ({ordersQueue.Count}/{MaxOrder}) and cannot accept more orders");
                throw new Exception($"Robot {Id} is full and cannot accept more orders");
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
            RaisePathPlannedEvent(result.path, result.times, currentPlanId);
            personalOccupancyLayer.RecordPathOccupancy(indoorSpace, result.path, result.times, this.Id.ToString(), currentPlanId);
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
        # endregion

        # region Movement System
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
            
            // Check if in the cell containing the picking point
            CellSpace currentCell = indoorSpace.GetCellSpaceFromCoordinates(transform.position);
            if (currentCell != null && currentCell.Id == cellSpace.Id)
            {
                // Check distance to the center of the picking point
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
                
                // Stop
                rb.velocity = Vector3.zero;
            }
        }

        private void MoveTowards(Vector3 target)
        {
            // Different movement parameters based on navigation phase
            float dwaActivationDistance = currentNavigationPhase == NavigationPhase.FollowingPath ? 2.5f : 1.5f;
            
            // Active DWA only when needed
            bool shouldUseDWA = ShouldActivateDWA(dwaActivationDistance);
            
            if (shouldUseDWA)
            {
                // Adjust DWA parameters based on navigation phase
                if (currentNavigationPhase == NavigationPhase.FollowingPath)
                {
                    dwaPlanner.PredictionTime = 2.0f;
                    
                    // Check if the robot is on a smooth path or an auxiliary point
                    if (smoothPathIndex < smoothPositions.Count && isAuxiliaryPoint[smoothPathIndex])
                    {
                        dwaPlanner.ObstacleWeight = 0.6f;
                        dwaPlanner.HeadingWeight = 0.3f;
                        
                        // Check if the point is a corner point
                        if (IsCornerPoint(smoothPathIndex))
                        {
                            dwaPlanner.PredictionTime = 2.5f;
                        }
                    }
                    else
                    {
                        // Normal path following
                        dwaPlanner.ObstacleWeight = 0.4f;
                        dwaPlanner.HeadingWeight = 0.4f;
                    }
                }
                else // ApproachingTarget
                {
                    dwaPlanner.PredictionTime = 1.2f;
                    dwaPlanner.ObstacleWeight = 0.7f;
                    dwaPlanner.HeadingWeight = 0.2f;
                }
                
                if (rb == null)
                {
                    Debug.LogError($"Robot {Id}: Rigidbody component missing");
                    return;
                }
                
                bool obstacleNearby = recentCollisions.Count > 0 || isEmergencyStopped;

                ConfigureDWAParameters(obstacleNearby);

                var (velocity, needReplan) = dwaPlanner.CalculateVelocity(
                    transform.position, 
                    rb.velocity, 
                    target);
                
                // 失败保护：如果DWA无法找到有效速度但距离目标还远，使用基础移动
                if (velocity.magnitude < 0.1f && Vector3.Distance(transform.position, target) > 1.0f)
                {
                    Debug.LogWarning($"Robot {Id}: DWA failed to find valid velocity, using basic movement");
                    velocity = (target - transform.position).normalized * defaultSpeed * 0.3f;
                }
                
                // 限制最大速度
                float maxAllowedSpeed = currentNavigationPhase == NavigationPhase.FollowingPath ? 
                                    defaultSpeed : defaultSpeed * 0.7f;
                
                Vector3 targetVelocity = Vector3.ClampMagnitude(velocity, maxAllowedSpeed);
                
                // Smooth velocity
                rb.velocity = Vector3.Lerp(rb.velocity, targetVelocity, 0.3f);
                
                // Update rotation to face movement direction
                if (rb.velocity.magnitude > 0.1f)
                {
                    Quaternion targetRotation = Quaternion.LookRotation(rb.velocity.normalized);
                    transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, Time.deltaTime * 5f);
                }
                
                // If DWA reports need to replan path
                if (needReplan && Time.time - lastReplanTime > replanInterval)
                {
                    lastReplanTime = Time.time;
                    StartCoroutine(ReplanPath());
                }
            }
            else
            {
                // Path following with heading correction
                DirectPathFollow(target);
            }
        }

        private void DirectPathFollow(Vector3 target)
        {
            if (rb == null)
            {
                Debug.LogError($"Robot {Id}: Rigidbody component missing");
                return;
            }

            // Calculate direction
            Vector3 directionToTarget = (target - transform.position).normalized;

            CellSpace currentCell = indoorSpace.GetCellSpaceFromCoordinates(transform.position);
            
            //  Calcualte appropriate speed
            float speedMultiplier = 1.0f;
            
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
            
            // Slow down if near cell boundary
            if (IsNearCellBoundary(currentCell))
            {
                speedMultiplier *= 0.8f;
            }
            
            // Add speed adjustment based on distance to target
            float distanceToTarget = Vector3.Distance(transform.position, target);
            if (distanceToTarget < 2.0f)
            {
                // Slow down when close to target
                speedMultiplier *= Mathf.Lerp(0.3f, 1.0f, distanceToTarget / 2.0f);
            }
            
            // Calculate target velocity
            Vector3 targetVelocity = directionToTarget * defaultSpeed * speedMultiplier;
            
            rb.velocity = Vector3.Lerp(rb.velocity, targetVelocity, 0.15f);
            
            // Make sure robot is facing movement direction
            if (rb.velocity.magnitude > 0.1f)
            {
                Quaternion targetRotation = Quaternion.LookRotation(rb.velocity.normalized);
                transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, 0.07f);
            }
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
        # endregion

        # region Path Following
        private void UpdateMovement()
        {
            // If navigation is disabled or emergency stopped, do not move
            if (!navigationEnabled || isEmergencyStopped) return;
            if (currentPath == null || currentPath.Count == 0 || isWaitingForPicking) return;

            try
            {
                // Selection of target point and behavior based on current navigation phase
                Vector3 currentTarget;
                float acceptanceRadius;

                switch (currentNavigationPhase)
                {
                    case NavigationPhase.FollowingPath:
                        // 检查是否到达路径终点
                        if (currentPathIndex >= currentPath.Count)
                        {
                            SwitchToApproachingTarget();
                            return;
                        }

                        // 使用平滑路径或原始路径点
                        if (smoothPathIndex < smoothPositions.Count)
                        {
                            // 使用平滑路径点
                            currentTarget = smoothPositions[smoothPathIndex];
                            
                            // 根据点类型设置接受半径
                            acceptanceRadius = isAuxiliaryPoint[smoothPathIndex] ? 
                                (pathPointAcceptanceRadius * 0.6f) : pathPointAcceptanceRadius;
                            
                            // 检查是否到达当前点
                            if (Vector3.Distance(transform.position, currentTarget) < acceptanceRadius)
                            {
                                // 移动到下一个平滑点
                                smoothPathIndex++;
                                
                                // 如果到达了原始点，也更新原始路径索引
                                if (smoothPathIndex < smoothPositions.Count && !isAuxiliaryPoint[smoothPathIndex])
                                {
                                    // 查找对应的原始点
                                    UpdateOriginalPathIndex();
                                    
                                    // 检查是否完成了全部原始点
                                    if (currentPathIndex >= currentPath.Count)
                                    {
                                        SwitchToApproachingTarget();
                                        return;
                                    }
                                }
                                
                                // 如果所有平滑点都用完了
                                if (smoothPathIndex >= smoothPositions.Count)
                                {
                                    // 检查原始路径是否也完成
                                    if (currentPathIndex >= currentPath.Count)
                                    {
                                        SwitchToApproachingTarget();
                                        return;
                                    }
                                    else
                                    {
                                        // 继续使用原始路径点
                                        currentTarget = GetPointVector3((Point)currentPath[currentPathIndex].Point);
                                    }
                                }
                                
                                return; // 已经处理完这一帧的移动
                            }
                        }
                        else
                        {
                            // 如果没有平滑路径或平滑路径用完，使用原始路径点
                            if (currentPathIndex < currentPath.Count)
                            {
                                currentTarget = GetPointVector3((Point)currentPath[currentPathIndex].Point);
                                acceptanceRadius = pathPointAcceptanceRadius;
                                
                                // 检查是否到达当前连接点
                                if (Vector3.Distance(transform.position, currentTarget) < acceptanceRadius)
                                {
                                    // 移动到下一个点
                                    currentPathIndex++;
                                    
                                    // 检查是否到了路径终点
                                    if (currentPathIndex >= currentPath.Count)
                                    {
                                        SwitchToApproachingTarget();
                                        return;
                                    }
                                    
                                    return; // 已经处理完这一帧的移动
                                }
                            }
                            else
                            {
                                SwitchToApproachingTarget();
                                return;
                            }
                        }
                        break;
                        
                    case NavigationPhase.ApproachingTarget:
                        // Target is the actual picking point
                        currentTarget = finalDestination;
                        
                        // Use smaller acceptance radius for final destination
                        acceptanceRadius = finalDestinationAcceptanceRadius;
                        
                        // Check if reached final destination
                        if (HasReachedFinalDestination())
                        {
                            HandleDestinationReached();
                            return;
                        }
                        break;

                    default:
                        return;
                }

                // Move according to current navigation phase
                if (isDWAEnabled)
                {
                    MoveTowards(currentTarget);
                }
                else
                {
                    DirectPathFollow(currentTarget);
                }
                
                // Path deviation check
                if (IsOffPath() && currentNavigationPhase == NavigationPhase.FollowingPath)
                {
                    CorrectPathDeviation();
                }
            }
            catch (Exception ex)
            {
                // Log error and attempt recovery
                Debug.LogError($"Robot {Id}: UpdateMovement error - {ex.Message}\n{ex.StackTrace}");
                needsPathRecovery = true;
            }
        }

        private void UpdateOriginalPathIndex()
        {
            if (smoothPathIndex >= smoothPositions.Count) return;
            
            // 如果是辅助点，不更新原始索引
            if (isAuxiliaryPoint[smoothPathIndex])
                return;
                
            // 获取当前平滑点
            Vector3 currentSmoothPos = smoothPositions[smoothPathIndex];
            
            // 在原始路径中找到匹配点
            for (int i = currentPathIndex; i < currentPath.Count; i++)
            {
                Vector3 origPos = GetPointVector3((Point)currentPath[i].Point);
                
                // 如果距离很近，认为是同一个点
                if (Vector3.Distance(currentSmoothPos, origPos) < 0.1f)
                {
                    currentPathIndex = i;
                    return;
                }
            }
        }

        private bool IsCornerPoint(int index)
        {
            if (index <= 0 || index >= smoothPositions.Count - 1)
                return false;
                
            Vector3 prev = smoothPositions[index - 1];
            Vector3 current = smoothPositions[index];
            Vector3 next = smoothPositions[index + 1];
            
            Vector3 dir1 = (current - prev).normalized;
            Vector3 dir2 = (next - current).normalized;
            
            float angle = Vector3.Angle(dir1, dir2);
            return angle > 30f;
        }

        private bool IsOffPath()
        {
            if (currentPath == null || currentPath.Count == 0)
                return false;
                
            // Path index safety check
            int safeIndex = Mathf.Min(currentPathIndex, currentPath.Count - 1);
            
            Vector3 currentPos = transform.position;
            Vector3 targetPos = GetPointVector3((Point)currentPath[safeIndex].Point);
            
            // Calculate distance to target
            float distanceToTarget = Vector3.Distance(currentPos, targetPos);
            
            // Non-deviation if very close to target
            if (distanceToTarget < pathPointAcceptanceRadius * 1.5f)
                return false;
            
            // Find closest point on path
            Vector3 closestPoint = FindClosestPointOnPath(currentPos);
            float distanceFromPath = Vector3.Distance(currentPos, closestPoint);
            
            // Dynamic deviation threshold
            float maxAllowedDeviation = 1.5f;
            
            // Larger deviation allowed at intersections
            CellSpace currentCell = indoorSpace.GetCellSpaceFromCoordinates(currentPos);
            if (currentCell != null && IsNearCellBoundary(currentCell))
            {
                maxAllowedDeviation = 2.0f;
            }
            
            // Con 1: Distance from path exceeds threshold
            bool tooFarFromPath = distanceFromPath > maxAllowedDeviation;
            
            // Con 2: Direction off path (if velocity is high enough)
            bool directionOffPath = false;
            if (rb != null && rb.velocity.magnitude > 0.1f)
            {
                Vector3 pathDirection = (targetPos - currentPos).normalized;
                Vector3 movementDirection = rb.velocity.normalized;
                
                // Check if robot movement direction deviates too much from path direction
                float angleDeviation = Vector3.Angle(pathDirection, movementDirection);
                directionOffPath = angleDeviation > 45f;
            }
            
            return tooFarFromPath || directionOffPath;
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

        private void CorrectPathDeviation()
        {
            if (!needsPathRecovery)
            {
                // Recovery from general path deviation
                Vector3 currentPos = transform.position;
                Vector3 closestPathPoint = FindClosestPointOnPath(currentPos);
                
                // Calculate correction direction
                Vector3 correctionDirection = (closestPathPoint - currentPos).normalized;
                
                // Add correction force, apply smooth transition
                float correctionStrength = defaultSpeed * 0.8f;
                Vector3 correctionVelocity = correctionDirection * correctionStrength;
                
                if (rb != null)
                {
                    rb.velocity = Vector3.Lerp(rb.velocity, correctionVelocity, 0.3f);
                }
                
                Debug.Log($"Robot {Id}: Recover from general path deviation. Distance from route: {Vector3.Distance(currentPos, closestPathPoint):F2}m");
            }
            else
            {
                // Recovery from severe path deviation
                if (Time.time - lastPathRecoveryTime > RECOVERY_INTERVAL && pathRecoveryAttempts < MAX_RECOVERY_ATTEMPTS)
                {
                    pathRecoveryAttempts++;
                    lastPathRecoveryTime = Time.time;
                    
                    // Stop
                    StopMoving();
                    
                    // Log
                    Debug.LogWarning($"Robot {Id}: Recover path - Attempt {pathRecoveryAttempts}/{MAX_RECOVERY_ATTEMPTS}");
                    
                    // Replan path
                    StartCoroutine(ReplanPath());
                }
                else if (pathRecoveryAttempts >= MAX_RECOVERY_ATTEMPTS)
                {
                    // Recovery failed, perform emergency handling
                    Debug.LogError($"Robot {Id}: Recovery failed, perform emergency handling");
                    StartCoroutine(EmergencyRecovery());
                    needsPathRecovery = false;
                }
            }
        }

        private void StopMoving()
        {
            if (rb != null)
            {
                rb.velocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
            }
        }
        # endregion

        # region DWA Control
        private bool ShouldActivateDWA(float activationDistance)
        {
            if (isEmergencyStopped || needsPathRecovery)
                return true;
            
            // Check if obstacle detected
            bool obstacleDetected = IsAnyObstacleDetected(activationDistance);
            
            // Check if in special area
            bool inSpecialArea = IsInIntersection();
            
            // DWA activation logic
            bool shouldActivate = obstacleDetected || inSpecialArea;
            
            // Cooldown period after state change
            if (isDWAEnabled && Time.time - lastDWAStateChangeTime < DWA_STATE_COOLDOWN)
                return true;
            
            // Update DWA state
            if (shouldActivate != isDWAEnabled)
            {
                lastDWAStateChangeTime = Time.time;
                isDWAEnabled = shouldActivate;
                
                // Adjust DWA parameters
                if (shouldActivate)
                {
                    ConfigureDWAParameters(obstacleDetected);
                }
            }
            
            return shouldActivate;
        }

        private bool IsAnyObstacleDetected(float distance)
        {
            // 扇形区域检测 - 覆盖前方和侧面
            float detectionAngle = 120f; // ±60度范围
            int numberOfRays = 9; // 发射更多射线
            
            // 1. 射线检测 - 远距离检测
            for (int i = 0; i < numberOfRays; i++)
            {
                float angle = -detectionAngle/2 + detectionAngle * i / (numberOfRays - 1);
                Quaternion rayRotation = Quaternion.Euler(0, angle, 0);
                Vector3 rayDirection = rayRotation * transform.forward;
                
                // 绘制检测射线
                Debug.DrawRay(transform.position, rayDirection * distance, Color.yellow);
                
                RaycastHit hit;
                if (Physics.Raycast(transform.position, rayDirection, out hit, distance, obstacleLayer))
                {
                    // 检测到障碍物
                    Debug.DrawLine(transform.position, hit.point, Color.red);
                    Debug.Log($"Robot {Id}: 射线检测到障碍物 {hit.collider.name}，距离 {hit.distance}");
                    return true;
                }
            }
            
            // 2. 盒体检测 - 更接近实际形状的检测
            Vector3 boxExtents = new Vector3(0.4f, 0.4f, 0.6f); // 根据机器人实际形状调整
            Vector3 boxCenter = transform.position + transform.forward * boxExtents.z;
            
            if (Physics.BoxCast(boxCenter, boxExtents, transform.forward, out RaycastHit boxHit, 
                            transform.rotation, distance, obstacleLayer))
            {
                Debug.DrawLine(transform.position, boxHit.point, Color.red);
                Debug.Log($"Robot {Id}: 盒体检测到障碍物 {boxHit.collider.name}，距离 {boxHit.distance}");
                return true;
            }
            
            // 3. 球体检测 - 周围环境检测
            Collider[] nearbyObjects = Physics.OverlapSphere(transform.position, distance * 0.7f, obstacleLayer);
            
            if (nearbyObjects.Length > 0)
            {
                foreach (var obj in nearbyObjects)
                {
                    // 忽略自身
                    if (obj.gameObject == this.gameObject) continue;
                    
                    Vector3 dirToObj = (obj.transform.position - transform.position).normalized;
                    float angleToForward = Vector3.Angle(transform.forward, dirToObj);
                    
                    // 只考虑前方180度范围内的物体
                    if (angleToForward <= 90f)
                    {
                        Debug.DrawLine(transform.position, obj.transform.position, Color.red);
                        Debug.Log($"Robot {Id}: 周围检测到障碍物 {obj.name}，角度 {angleToForward}°");
                        return true;
                    }
                }
            }
            
            return false;
        }

        private bool IsInIntersection()
        {
            if (intersectionManager != null)
            {
                return intersectionManager.IsRobotInIntersection(Id);
            }
            return false;
        }

        private void ConfigureDWAParameters(bool obstacleDetected=false)
        {
            if (obstacleDetected)
            {
                dwaPlanner.ObstacleWeight = 0.6f;
                dwaPlanner.HeadingWeight = 0.3f;
                dwaPlanner.DistanceWeight = 0.1f;
                dwaPlanner.MaxSpeed = defaultSpeed * 0.7f;
                dwaPlanner.PredictionTime = 2.0f;
                
                dwaPlanner.VelocitySamples = 9;
                dwaPlanner.AngleSamples = 20;
            }
            else
            {
                dwaPlanner.HeadingWeight = 0.5f;
                dwaPlanner.DistanceWeight = 0.2f;
                dwaPlanner.VelocityWeight = 0.2f;
                dwaPlanner.ObstacleWeight = 0.3f;
                dwaPlanner.MaxSpeed = defaultSpeed * 0.8f;
                dwaPlanner.PredictionTime = 1.5f;
                dwaPlanner.VelocitySamples = 7;
                dwaPlanner.AngleSamples = 15;
            }
        }
        # endregion

        #region Emergency Recovery
        private IEnumerator EmergencyRecovery()
        {
            // Stop
            StopMoving();
            
            // Log error and notify central controller
            Debug.LogError($"Robot {Id}: Emergency recovery initiated");
            
            // Failed Order
            if (currentOrder != null)
            {
                Debug.LogWarning($"Robot {Id}: Failed order {currentOrder.Id}");
                // 这里可以通知中央控制器订单失败
            }
            
            // Reset robot state
            ResetRobot();
            
            yield return new WaitForSeconds(2.0f);
            
            // Attempt to recover to safe position
            Vector3 safePosition = FindSafePosition();
            yield return StartCoroutine(MoveToSafePosition(safePosition));
        }

        private Vector3 FindSafePosition()
        {
            // Attempt to find safe position nearby
            Vector3 currentPos = transform.position;
            CellSpace currentCell = indoorSpace.GetCellSpaceFromCoordinates(currentPos);
            
            // Stay at current position if safe
            if (currentCell != null && !Physics.CheckSphere(currentPos, 1.0f, obstacleLayer))
                return currentPos;
            
            // Search for safe position nearby
            for (float radius = 2.0f; radius <= 10.0f; radius += 2.0f)
            {
                for (int angle = 0; angle < 360; angle += 45)
                {
                    float radians = angle * Mathf.Deg2Rad;
                    Vector3 testPos = currentPos + new Vector3(
                        Mathf.Cos(radians) * radius,
                        0,
                        Mathf.Sin(radians) * radius
                    );
                    
                    CellSpace testCell = indoorSpace.GetCellSpaceFromCoordinates(testPos);
                    if (testCell != null && testCell.IsNavigable() && !Physics.CheckSphere(testPos, 1.0f, obstacleLayer))
                    {
                        return testPos;
                    }
                }
            }
            
            // Return to origin/predefined safe position if no safe position found
            return Vector3.zero;
        }

        private IEnumerator MoveToSafePosition(Vector3 safePosition)
        {
            Debug.Log($"Robot {Id}: Moving to a safe position {safePosition}");
            
            // Replan path to safe position
            var result = PlanPath(safePosition);
            if (result.path != null && result.path.Count > 0)
            {
                currentPlanId = result.planId;
                RaisePathPlannedEvent(result.path, result.times, currentPlanId);
                personalOccupancyLayer.RecordPathOccupancy(indoorSpace, result.path, result.times, this.Id.ToString(), currentPlanId);
                UpdateCurrentMovementInfo(result.path, result.times, result.speeds);
                
                // Force DWA activation
                isDWAEnabled = true;
                lastDWAStateChangeTime = Time.time;
                
                float startTime = Time.time;
                float timeout = 30f;
                
                while (Vector3.Distance(transform.position, safePosition) > 0.5f)
                {
                    if (Time.time - startTime > timeout)
                    {
                        Debug.LogError($"Robot {Id}: Timeout while moving to safe position");
                        break;
                    }
                    
                    yield return null;
                }
                
                Debug.Log($"Robot {Id}: Reached safe position {safePosition}");
            }
            else
            {
                Debug.LogError($"Robot {Id}: Failed to plan path to safe position");
            }
            
            // Reset recovery state
            needsPathRecovery = false;
            pathRecoveryAttempts = 0;
            isDWAEnabled = false;
            isEmergencyStopped = false;
            
            // Clear recent collisions
            recentCollisions.Clear();
        }
        private IEnumerator ReplanPath()
        {
            Debug.Log($"Robot {Id}: Replanning path to destination");
            
            if (currentOrder == null)
            {
                Debug.LogWarning($"Robot {Id}: No current order to replan path for");
                yield break;
            }
            
            // Save current position and final target
            Vector3 currentPos = transform.position;
            Vector3 finalTarget = GetTargetPosition(currentOrder.PickingPoint);
            
            // Disabling navigation to prevent interference
            bool wasNavigationEnabled = navigationEnabled;
            navigationEnabled = false;
            
            if (isDWAEnabled)
            {
                dwaPlanner.MaxSpeed = defaultSpeed * 0.5f;
            }
            
            // Wait for stabilization
            yield return new WaitForSeconds(0.5f);
            
            // Replan path
            var result = PlanPath(finalTarget, currentPlanId);
            if (result.path != null && result.path.Count > 0)
            {
                currentPlanId = result.planId;
                RaisePathPlannedEvent(result.path, result.times, currentPlanId);
                personalOccupancyLayer.RecordPathOccupancy(indoorSpace, result.path, result.times, this.Id.ToString(), currentPlanId);
                UpdateCurrentMovementInfo(result.path, result.times, result.speeds);
                currentPathIndex = 0;
                
                // Reset navigation phase
                currentNavigationPhase = NavigationPhase.FollowingPath;
                
                Debug.Log($"Robot {Id}: Replanning path succeeded.");
            }
            else
            {
                Debug.LogError($"Robot {Id}: Replanning path failed.");
                
                // Attempt to intermediate target
                Vector3 intermediateTarget = Vector3.Lerp(currentPos, finalTarget, 0.5f);
                CellSpace intermediateCell = indoorSpace.GetCellSpaceFromCoordinates(intermediateTarget);
                
                if (intermediateCell != null && intermediateCell.IsNavigable())
                {
                    Debug.Log($"Robot {Id}: Attempt to intermediate target {intermediateTarget}");
                    result = PlanPath(intermediateTarget);
                    
                    if (result.path != null && result.path.Count > 0)
                    {
                        currentPlanId = result.planId;
                        RaisePathPlannedEvent(result.path, result.times, currentPlanId);
                        personalOccupancyLayer.RecordPathOccupancy(indoorSpace, result.path, result.times, this.Id.ToString(), currentPlanId);
                        UpdateCurrentMovementInfo(result.path, result.times, result.speeds);
                        currentPathIndex = 0;
                        finalDestination = intermediateTarget;
                        
                        Debug.Log($"Robot {Id}: Replanning to intermediate target succeeded.");
                    }
                }
            }
            
            // Re-enable navigation
            navigationEnabled = wasNavigationEnabled;
        }
        # endregion

        # region Conflict Resolution
        // 1. Time shift modification
        private void ApplyTimeShiftModification(List<ConnectionPoint> path, List<float> newTimes, string planId, float shift)
        {
            // Update path and times
            currentPath = path;
            pathTimes = newTimes;
            currentPlanId = planId;
            
            Debug.Log($"Robot {Id}: Utilize time shift modification: {shift}s, new path length: {path.Count}");
            
            // Save path to personal layer
            SavePathToPersonalLayer(path, newTimes, planId);
            
            // Adjust waiting time
            float newStartTime = newTimes[0];
            float currentTime = Time.time;
            float waitTime = Math.Max(0, newStartTime - currentTime);
            
            isWaitingForExecution = true;
            
            // If there's an existing waiting coroutine, stop it
            if (waitForExecutionCoroutine != null)
            {
                StopCoroutine(waitForExecutionCoroutine);
            }
            
            // Execute new waiting time
            waitForExecutionCoroutine = StartCoroutine(WaitForExecution(waitTime));
        }

        // 2. Speed adjustment modification
        private void ApplySpeedAdjustmentModification(
            List<ConnectionPoint> path, 
            List<float> newTimes, 
            List<float> newSpeeds, 
            string planId, 
            float avgSpeedChange)
        {
            // 更新路径和时间
            currentPath = path;
            pathTimes = newTimes;
            currentPlanId = planId;
            
            Debug.Log($"机器人 {Id}: 应用速度调整修改: 平均速度变化 {avgSpeedChange:F2}倍, 新路径长度: {path.Count}");
            
            // 更新路径段并应用新的速度
            UpdatePathSegmentsWithSpeed(path, newTimes, newSpeeds);
            
            // 保存路径到个人地图层
            SavePathToPersonalLayer(path, newTimes, planId);
            
            // 调整等待时间
            float newStartTime = newTimes[0];
            float currentTime = Time.time;
            float waitTime = Math.Max(0, newStartTime - currentTime);
            
            isWaitingForExecution = true;
            
            // 如果有现有等待，取消它
            if (waitForExecutionCoroutine != null)
            {
                StopCoroutine(waitForExecutionCoroutine);
            }
            
            // 启动新的等待
            waitForExecutionCoroutine = StartCoroutine(WaitForExecution(waitTime));
            
            // 通知状态改变
            SetState(RobotState.Moving, true);
        }

        private void UpdatePathSegmentsWithSpeed(List<ConnectionPoint> path, List<float> times, List<float> speeds)
        {
            // 清空当前的移动段
            currentMovement.Clear();
            
            // 确保有效的输入
            if (path == null || times == null || speeds == null || 
                path.Count != times.Count || path.Count != speeds.Count)
            {
                Debug.LogError($"机器人 {Id}: 更新路径段失败 - 数据不一致");
                return;
            }
            
            // 创建新的路径段
            for (int i = 0; i < path.Count; i++)
            {
                currentMovement.Add(new PathSegment(path[i], times[i], speeds[i]));
            }
            
            // 更新DWA规划器的最大速度，使用路径中的最大速度
            if (speeds.Count > 0)
            {
                float maxSpeed = speeds.Max();
                dwaPlanner.MaxSpeed = maxSpeed;
                Debug.Log($"机器人 {Id}: 更新最大速度为 {maxSpeed}");
            }
        }

        private IEnumerator WaitForExecution(float waitTime)
        {
            Debug.Log($"机器人 {Id}: 等待 {waitTime}s 后开始执行路径");
            
            yield return new WaitForSeconds(waitTime);
            
            BeginPathExecution();
        }

        private void BeginPathExecution()
        {
            if (currentPath == null || pathTimes == null || currentPath.Count == 0)
            {
                Debug.LogWarning($"Robot {Id}: Attempt to execute an empty path");
                return;
            }
            
            Debug.Log($"Robot {Id}: Start to execute naivgation, path length: {currentPath.Count}");
            
            isWaitingForExecution = false;
            SetState(RobotState.Moving, true);
            
            // Set navigation target
            SetNavigationTarget(currentPath[0]);
            
            // Update path tracking index
            currentPathIndex = 0;
        }

        private void SetNavigationTarget(ConnectionPoint target)
        {
            if (target == null || target.Point == null)
            {
                Debug.LogWarning($"Robot {Id}: Attempt to set an invalid target point");
                return;
            }
            
            Point point = target.Point as Point;
            if (point == null)
            {
                Debug.LogWarning($"Robot {Id}: Attempt to set an invalid target point type");
                return;
            }
            
            targetPosition = new Vector3((float)point.X, transform.position.y, (float)point.Y);
            Debug.Log($"Robot {Id}: set target to: {targetPosition}");
        }
        # endregion

        # region Status Management
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

        public bool IsStuck
        {
            get
            {
                if (!IsMoving || currentPath.Count != 0)
                    return false;
                    
                float timeSinceLastMove = Time.time - lastPositionChangeTime;
                float distanceMoved = Vector3.Distance(lastPosition, transform.position);
                
                return timeSinceLastMove > 5.0f && distanceMoved < 0.1f;
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

        public void ResetPath()
        {
            Debug.Log($"Robot {Id}: Resetting path state");
            
            // 重置状态变量
            needsPathRecovery = true;
            pathRecoveryAttempts = 0;
            lastPathRecoveryTime = Time.time;
            
            // 取消当前移动 - 使用现有方法
            if (IsMoving)
            {
                StopMoving();
                SetState(RobotState.Moving, false);
            }
            
            // 短暂延迟后进行适当的恢复
            StartCoroutine(RecoverAfterDelay(1.0f));
        }

        private IEnumerator RecoverAfterDelay(float delay)
        {
            yield return new WaitForSeconds(delay);
            
            // 重置状态
            needsPathRecovery = false;
            
            // 根据当前状态决定如何恢复
            if (currentOrder != null)
            {
                // 有订单，重新规划路径
                Debug.Log($"Robot {Id}: Recovering with replanning path");
                StartCoroutine(ReplanPath());
            }
            else if (finalDestination != Vector3.zero) // 使用已有的finalDestination变量
            {
                // 没有订单但有目标，恢复导航
                Debug.Log($"Robot {Id}: Recovering by resuming navigation to existing destination");
                navigationEnabled = true;
                
                UpdateTargetIndicator();
            }
            else
            {
                // 没有订单也没有目标，简单恢复导航能力
                Debug.Log($"Robot {Id}: Recovering navigation capability only");
                navigationEnabled = true;
            }
        }
        # endregion

        # region Occupancy Layer
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
                // Determine if the last cell space was a start cell
                bool wasStartCell = (lastCellSpace == firstCellSpace);
                
                // Update occupancy for last cell space
                if (cellSpaceEntryTimes.TryGetValue(lastCellSpace.Id, out float oldEntryTime))
                {
                    personalOccupancyLayer.SetDynamicOccupancy(
                        lastCellSpace.Id,
                        oldEntryTime,
                        currentTime,
                        Id.ToString(), 
                        currentPlanId,
                        wasStartCell,
                        false // Not an end cell since we're moving to a new cell
                    );
                    cellSpaceEntryTimes.Remove(lastCellSpace.Id);
                }
                
                // Record entry time for new cell space
                cellSpaceEntryTimes[currentCellSpace.Id] = currentTime;
                
                // If this is our first cell transition, record the first cell space
                if (firstCellSpace == null)
                {
                    firstCellSpace = lastCellSpace;
                }
                
                lastCellSpace = currentCellSpace;
                return;
            }

            // Update occupancy for current cell space
            if (cellSpaceEntryTimes.TryGetValue(currentCellSpace.Id, out float entryTime))
            {
                // Determine if this is a start or end cell
                bool isStartCell = (currentCellSpace == firstCellSpace);
                bool isEndCell = (currentOrder != null && HasReachedFinalDestination());
                
                personalOccupancyLayer.SetDynamicOccupancy(
                    currentCellSpace.Id,
                    entryTime,
                    currentTime,
                    Id.ToString(), 
                    currentPlanId,
                    isStartCell,
                    isEndCell
                );
            }
        }

        private void SavePathToPersonalLayer(List<ConnectionPoint> path, List<float> times, string planId)
        {
            // 创建一个占用标识符
            var occupier = new OccupancyIdentifier(Id.ToString(), planId, OccupancyType.Planned);
            
            // 记录路径到个人地图层
            personalOccupancyLayer.RecordPathOccupancy(indoorSpace, path, times, Id.ToString(), planId);
        }

        public void ClearPathReservations()
        {
            if (!string.IsNullOrEmpty(currentPlanId))
            {
                foreach (var cellSpace in indoorSpace.CellSpaces)
                {
                    personalOccupancyLayer.ClearPlannedOccupancy(
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
        # endregion

        # region Intersection Management
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
        # endregion

        #region Collision Detection
        private void CheckSurroundings()
        {
            // Clean up recent collisions list
            if (Time.time - lastCollisionClearTime > collisionClearInterval)
            {
                recentCollisions.Clear();
                lastCollisionClearTime = Time.time;
            }

            // Check for nearby obstacles
            Collider[] hitColliders = Physics.OverlapSphere(transform.position, detectionRadius, obstacleLayer);
            bool emergencySituation = false;
            
            foreach (var hitCollider in hitColliders)
            {
                if (hitCollider.gameObject == gameObject)
                    continue;
                    
                RobotController otherRobot = hitCollider.GetComponent<RobotController>();
                if (otherRobot != null)
                {
                    float distance = Vector3.Distance(transform.position, otherRobot.transform.position);
                    
                    // Emergency stop if too close
                    if (distance < emergencyStopDistance)
                    {
                        emergencySituation = true;
                        
                        EmergencyStop();
                        
                        // Record this collision
                        if (!recentCollisions.Contains(otherRobot.gameObject))
                        {
                            recentCollisions.Add(otherRobot.gameObject);
                        }
                        
                        break;
                    }
                    
                    // Determine if potential collision risk
                    if (distance < detectionRadius * 0.7f && !isEmergencyStopped)
                    {
                        // Calculate relative velocity and approach speed
                        Vector3 relativeVelocity = Vector3.zero;
                        if (otherRobot.rb != null && rb != null)
                        {
                            relativeVelocity = otherRobot.rb.velocity - rb.velocity;
                        }
                        
                        Vector3 directionToOther = (otherRobot.transform.position - transform.position).normalized;
                        float approachSpeed = -Vector3.Dot(relativeVelocity, directionToOther);
                        
                        // Force DWA activation if approaching too fast
                        if (approachSpeed > 0.5f)
                        {
                            isDWAEnabled = true;
                            lastDWAStateChangeTime = Time.time;
                            dwaPlanner.ObstacleWeight = 0.4f;
                            
                            Debug.Log($"Robot {Id}: Detected risk of collision, DWA activated. Approach speed: {approachSpeed:F2}m/s");
                        }
                    }
                }
                else
                {
                    // Other types of obstacles
                    float distance = Vector3.Distance(transform.position, hitCollider.transform.position);
                    if (distance < emergencyStopDistance * 1.5f)
                    {
                        emergencySituation = true;
                        EmergencyStop();
                        
                        if (!recentCollisions.Contains(hitCollider.gameObject))
                        {
                            recentCollisions.Add(hitCollider.gameObject);
                        }
                        
                        break;
                    }
                }
            }
            
            // Check if emergency stop can be resumed, if no emergency situation but emergency stopped
            if (!emergencySituation && isEmergencyStopped)
            {
                // Cooldown before resuming
                if (Time.time - lastEmergencyStopTime > EMERGENCY_COOLDOWN)
                {
                    ResumeFromEmergencyStop();
                }
            }
        }

        private void EmergencyStop()
        {
            if (!isEmergencyStopped)
            {
                isEmergencyStopped = true;
                lastEmergencyStopTime = Time.time;
                
                // Stop
                if (rb != null)
                {
                    rb.velocity = Vector3.zero;
                    rb.angularVelocity = Vector3.zero;
                }
                
                // Force DWA activation
                isDWAEnabled = true;
                lastDWAStateChangeTime = Time.time;
                
                dwaPlanner.MaxSpeed = defaultSpeed * 0.3f;
                dwaPlanner.ObstacleWeight = 0.5f;
                dwaPlanner.HeadingWeight = 0.3f;
                
                Debug.Log($"Robot {Id}: Emergency stop activated");
                
                // Mark for path recovery
                needsPathRecovery = true;
                pathRecoveryAttempts = 0;
            }
        }

        private void ResumeFromEmergencyStop()
        {
            if (isEmergencyStopped)
            {
                isEmergencyStopped = false;
                
                // Recover DWA parameters
                dwaPlanner.MaxSpeed = defaultSpeed * 0.7f;
                dwaPlanner.ObstacleWeight = 0.3f;
                dwaPlanner.HeadingWeight = 0.5f;
                
                // Maintain DWA for a while
                lastDWAStateChangeTime = Time.time;
                
                Debug.Log($"Robot {Id}: Resuming from emergency stop");
                
                // Replan path if needed
                StartCoroutine(ReplanPath());
            }
        }

        private void OnCollisionEnter(Collision collision)
        {
            if (collision.gameObject.layer == LayerMask.NameToLayer("GroundLayer"))
                return;
                
            Debug.LogWarning($"Robot {Id}: Detected collision - {collision.gameObject.name}");
            
            if (!recentCollisions.Contains(collision.gameObject))
            {
                recentCollisions.Add(collision.gameObject);
            }
            
            EmergencyStop();
            
            // Wait for situation to stabilize
            StartCoroutine(TemporaryDisableNavigation());
        }

        private IEnumerator TemporaryDisableNavigation()
        {
            navigationEnabled = false;
            yield return new WaitForSeconds(1.0f);
            navigationEnabled = true;
            
            // Replan path
            yield return StartCoroutine(ReplanPath());
        }
        #endregion

        #region Events Management
        private void HandlePathModified(object sender, RobotPathModificationEventArgs e)
        {
            // 检查是否是给当前机器人的修改
            if (e.RobotId != Id.ToString())
                return;
            
            Debug.Log($"机器人 {Id}: 收到路径修改通知: 类型 {e.ModificationType}, 值 {e.ModificationValue}");
            
            // 根据修改类型处理
            switch (e.ModificationType)
            {
                case "time_shift":
                    // 处理时间偏移修改
                    ApplyTimeShiftModification(e.Path, e.NewTimes, e.PlanId, e.ModificationValue);
                    break;
                case "speed_adjustment":
                    // 处理速度调整修改
                    ApplySpeedAdjustmentModification(e.Path, e.NewTimes, e.NewSpeeds, e.PlanId, e.ModificationValue);
                    break;
                // 未来可以添加其他修改类型的处理
                default:
                    Debug.LogWarning($"机器人 {Id}: 未知的路径修改类型: {e.ModificationType}");
                    break;
            }
        }

        private void HandlePathAccepted(object sender, string robotId)
        {
            // 检查是否是给当前机器人的通知
            if (robotId != Id.ToString())
                return;
            
            Debug.Log($"机器人 {Id}: 路径已被中央接受");
            
            // 执行响应逻辑，例如开始执行路径
            BeginPathExecution();
        }

        private void HandlePickingCompletedEvent(object sender, PickingEventArgs e)
        {
            // 确认事件是否针对此机器人
            if (e.RobotId != Id.ToString())
                return;
            
            Debug.Log($"Robot {Id}: 收到拣货完成事件，PickingPoint: {e.PickingPoint.Id}");
            
            // 确认是当前订单的拣货点
            if (currentOrder != null && currentOrder.PickingPoint.Id == e.PickingPoint.Id)
            {
                NotifyPickingCompleted();
            }
            else
            {
                Debug.LogWarning($"Robot {Id}: 收到拣货完成事件，但当前没有匹配的订单或拣货点不匹配");
            }
        }
        #endregion

        # region Interaction with Picker
        // 添加创建交互触发器的方法
        private void CreateInteractionTrigger()
        {
            // 检查是否已存在交互触发器
            Transform existingTrigger = transform.Find("InteractionTrigger");
            if (existingTrigger != null)
            {
                return; // 已存在，不需要重复创建
            }
            
            // 创建触发器对象
            GameObject triggerObj = new GameObject("InteractionTrigger");
            triggerObj.transform.SetParent(transform);
            triggerObj.transform.localPosition = Vector3.zero;
            
            // 添加球形触发器，范围大于NavMeshObstacle
            SphereCollider interactionTrigger = triggerObj.AddComponent<SphereCollider>();
            interactionTrigger.radius = 2.0f; // 大范围，便于拣货员检测
            interactionTrigger.center = new Vector3(0, 0.5f, 0);
            interactionTrigger.isTrigger = true; // 设置为触发器
            
            // 添加交互处理组件
            RobotInteractionZone interactionZone = triggerObj.AddComponent<RobotInteractionZone>();
            interactionZone.robotController = this;
            
            Debug.Log($"Robot {Id}: 创建了交互触发区域");
        }

        // 添加动态管理NavMeshObstacle的协程
        private IEnumerator ManageObstacleCarving()
        {
            NavMeshObstacle obstacle = GetComponent<NavMeshObstacle>();
            if (obstacle == null) yield break;
            
            Vector3 normalSize = obstacle.size;
            Vector3 movingSize = new Vector3(
                normalSize.x * 0.8f, 
                normalSize.y, 
                normalSize.z * 0.8f
            );
            
            while (true)
            {
                // 获取当前速度
                float currentSpeed = 0;
                if (rb != null)
                {
                    currentSpeed = rb.velocity.magnitude;
                }
                
                // 当机器人快速移动时，缩小障碍物
                if (currentSpeed > 0.5f)
                {
                    obstacle.size = movingSize;
                }
                else
                {
                    // 静止或慢速移动时使用正常尺寸
                    obstacle.size = normalSize;
                }
                
                yield return new WaitForSeconds(0.5f);
            }
        }
        # endregion

        # region Visualization
        private void OnDrawGizmos()
        {
            if (!Application.isPlaying) return;

            // // Visualize detection radius
            // Gizmos.color = Color.yellow;
            // Gizmos.DrawWireSphere(transform.position, detectionRadius);
            
            // // Visualize detection coverage
            // Gizmos.color = new Color(1, 1, 0, 0.1f); 
            // Gizmos.DrawSphere(transform.position, detectionRadius);
            
            // // Display robot ID and state
            // UnityEditor.Handles.Label(transform.position + Vector3.up * 2, 
            //     $"Robot {Id}\nState: {(IsMoving ? "Moving" : IsPicking ? "Picking" : "Idle")}\nOrders: {ordersQueue.Count}");
            
            // Visualize velocity
            if (rb != null)
            {
                Gizmos.color = Color.blue;
                Gizmos.DrawLine(transform.position, transform.position + rb.velocity * 0.5f);
            }
            
            // // Visualize target indicator
            // if (currentOrder != null && finalDestination != Vector3.zero)
            // {
            //     Gizmos.color = Color.red;
            //     Gizmos.DrawRay(transform.position, (finalDestination - transform.position).normalized);
            // }
            
            // Visualize path
            if (currentPath != null && currentPath.Count > 0)
            {
                Gizmos.color = Color.cyan;
                
                // Draw the entire path
                for (int i = 0; i < currentPath.Count - 1; i++)
                {
                    Vector3 start = GetPointVector3((Point)currentPath[i].Point);
                    Vector3 end = GetPointVector3((Point)currentPath[i + 1].Point);
                    
                    Gizmos.DrawLine(start, end);
                }
                
                // // Highligh current path segment
                // if (currentPathIndex < currentPath.Count)
                // {
                //     Vector3 currentPos = transform.position;
                //     Vector3 targetPos = GetPointVector3((Point)currentPath[currentPathIndex].Point);
                    
                //     Gizmos.color = Color.green;
                //     Gizmos.DrawLine(currentPos, targetPos);
                //     Gizmos.DrawSphere(targetPos, 0.2f);
                // }
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
        # endregion
        
        # region Helper Methods
        private bool isInitialized = false;
        private CellSpace lastCellSpace = null;
        private CellSpace firstCellSpace = null; // Track the first cell space in a path
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
        # endregion
    }
}
