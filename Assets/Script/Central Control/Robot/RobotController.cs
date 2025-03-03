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
        public bool lastReportedAvailability { get; set; }
        public bool lastReportedFreeStatus { get; set; }
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
        public int GetRobotOrdersQueueCount => ordersQueue.Count;
        public List<string> listOrdersQueue => new List<string>(ordersQueue.Select(order => order.Id));
        # endregion

        # region Navigation System
        private enum NavigationPhase
        {
            FollowingPath,
            ApproachingTarget
        }
        private NavigationPhase currentNavigationPhase = NavigationPhase.FollowingPath;
        private Vector3 finalDestination;
        private float pathPointAcceptanceRadius = 0.8f;
        private float finalDestinationAcceptanceRadius = 0.3f;
        private bool hasReportedArrival = false;
        private bool navigationEnabled = true;
        private bool isEmergencyStopped = false;
        private float lastEmergencyStopTime = 0f;
        private const float EMERGENCY_COOLDOWN = 3.0f;
        # endregion

        # region DWA Parameters
        private bool isDWAEnabled = false;
        private float lastDWAStateChangeTime = 0f;
        private const float DWA_STATE_COOLDOWN = 2.0f;
        private IntersectionAreaManager intersectionManager;
        private bool isIntersectionControlled = false;
        private string currentIntersectionId;
        # endregion

        # region Path Correction
        private bool needsPathRecovery = false;
        private int pathRecoveryAttempts = 0;
        private float lastPathRecoveryTime = 0f;
        private const int MAX_RECOVERY_ATTEMPTS = 3;
        private const float RECOVERY_INTERVAL = 2.0f;
        # endregion

        # region Collision Avoidance
        private List<GameObject> recentCollisions = new List<GameObject>();
        private float collisionClearInterval = 5.0f;
        private float lastCollisionClearTime = 0f;
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
                CapsuleCollider collider = gameObject.AddComponent<CapsuleCollider>();
                collider.radius = 0.5f;
                collider.height = 1.5f;
                collider.center = new Vector3(0, 0.75f, 0);
            }
            
            detectionRadius = baseDetectionRadius;
            obstacleLayer = LayerMask.GetMask("RobotLayer", "ObstacleLayer");
            
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
        # endregion

        # region Order Processing
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
                
                // Stop
                rb.velocity = Vector3.zero;
            }
        }

        private void MoveTowards(Vector3 target)
        {
            // Different movement parameters based on navigation phase
            float dwaActivationDistance;
            float lookAheadDistance;
            
            if (currentNavigationPhase == NavigationPhase.FollowingPath)
            {
                // Parameters for path following phase
                dwaActivationDistance = 1.8f;
                lookAheadDistance = 2.5f;
            }
            else
            {
                // Params for final approaching phase
                dwaActivationDistance = 1.0f;
                lookAheadDistance = 1.2f;
            }
            
            // Active DWA only when needed
            bool shouldUseDWA = ShouldActivateDWA(dwaActivationDistance);
            
            if (shouldUseDWA)
            {
                dwaPlanner.PredictionTime = currentNavigationPhase == NavigationPhase.FollowingPath ? 2.0f : 1.2f;
                
                if (rb == null)
                {
                    Debug.LogError($"Robot {Id}: Rigidbody component missing");
                    return;
                }
                
                var (velocity, needReplan) = dwaPlanner.CalculateVelocity(transform.position, rb.velocity, target);
                
                Vector3 targetVelocity = velocity.normalized * 
                    Mathf.Min(velocity.magnitude, currentNavigationPhase == NavigationPhase.FollowingPath ? defaultSpeed : defaultSpeed * 0.7f);
                
                if (targetVelocity.magnitude < 0.1f && Vector3.Distance(transform.position, target) > 1.0f)
                {
                    Debug.LogWarning($"Robot {Id}: DWA failed to find valid velocity, using basic movement");
                    
                    targetVelocity = (target - transform.position).normalized * defaultSpeed * 0.3f;
                }
                
                rb.velocity = Vector3.Lerp(rb.velocity, targetVelocity, 0.3f);
                
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
                DirectPathFollow(target, lookAheadDistance);
            }
        }

        private void DirectPathFollow(Vector3 target, float lookAheadDistance)
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
            
            if (currentPath == null || currentPath.Count == 0 || isWaitingForPicking)
                return;

            try
            {
                // Selection of target point and behavior based on current navigation phase
                Vector3 currentTarget;
                float acceptanceRadius;

                switch (currentNavigationPhase)
                {
                    case NavigationPhase.FollowingPath:
                        if (currentPathIndex >= currentPath.Count)
                        {
                            SwitchToApproachingTarget();
                            return;
                        }

                        // Next connectionPoint
                        currentTarget = GetPointVector3((Point)currentPath[currentPathIndex].Point);
                        acceptanceRadius = pathPointAcceptanceRadius;

                        // Check if reached current connectionPoint
                        if (Vector3.Distance(transform.position, currentTarget) < acceptanceRadius)
                        {
                            // Move to next point
                            currentPathIndex++;
                            
                            // Switch to approaching target phase
                            if (currentPathIndex >= currentPath.Count)
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
                    DirectPathFollow(currentTarget, 2.0f);
                }
                
                // Path deviation
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
                    if (rb != null)
                    {
                        rb.velocity = Vector3.zero;
                    }
                    
                    // Lod
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
        # endregion

        # region DWA Control
        private bool ShouldActivateDWA(float activationDistance)
        {
            // Force DWA if in emergency state
            if (isEmergencyStopped || needsPathRecovery)
                return true;
            
            // Check if obstacle ahead
            bool obstacleAhead = IsObstacleAhead(activationDistance);
            
            // Check if near intersection
            bool nearIntersection = false;
            if (intersectionManager != null)
            {
                nearIntersection = intersectionManager.IsNearIntersection(transform.position, activationDistance);
            }
            
            
            // Check if near recently collided object
            bool nearRecentCollision = false;
            foreach (var collisionObj in recentCollisions)
            {
                if (collisionObj != null && Vector3.Distance(transform.position, collisionObj.transform.position) < activationDistance * 1.5f)
                {
                    nearRecentCollision = true;
                    break;
                }
            }
            
            bool shouldActivate = obstacleAhead || nearIntersection || nearRecentCollision;
            
            // Cool down
            if (isDWAEnabled && Time.time - lastDWAStateChangeTime < DWA_STATE_COOLDOWN)
                return true;
            
            // Record time if state changed
            if (shouldActivate != isDWAEnabled)
            {
                lastDWAStateChangeTime = Time.time;
                isDWAEnabled = shouldActivate;
                
                if (shouldActivate)
                {
                    if (obstacleAhead)
                    {
                        float obstacleDistance = GetForwardObstacleDistance();
                        float distanceFactor = Mathf.Clamp01(obstacleDistance / (activationDistance * 1.5f));
                        
                        dwaPlanner.HeadingWeight = Mathf.Lerp(0.2f, 0.4f, distanceFactor);
                        dwaPlanner.ObstacleWeight = Mathf.Lerp(0.5f, 0.3f, distanceFactor);
                        dwaPlanner.MaxSpeed = defaultSpeed * Mathf.Lerp(0.5f, 0.8f, distanceFactor);
                    }
                    else if (nearRecentCollision)
                    {
                        dwaPlanner.HeadingWeight = 0.3f;
                        dwaPlanner.ObstacleWeight = 0.4f;
                        dwaPlanner.MaxSpeed = defaultSpeed * 0.7f;
                    }
                    else
                    {
                        dwaPlanner.HeadingWeight = 0.5f;
                        dwaPlanner.ObstacleWeight = 0.25f;
                        dwaPlanner.MaxSpeed = defaultSpeed * 0.9f;
                    }
                }
                else
                {
                    dwaPlanner.HeadingWeight = 0.6f;
                    dwaPlanner.ObstacleWeight = 0.15f;
                    dwaPlanner.MaxSpeed = defaultSpeed;
                }
            }
            
            return shouldActivate;
        }

        private bool IsObstacleAhead(float distance)
        {
            // Get forward direction
            Vector3 forwardDirection = rb.velocity.magnitude > 0.1f ? 
                rb.velocity.normalized : transform.forward;
            
            // Detect obstacles in front
            bool obstacleDetected = false;
            float fanAngle = 90f;
            int rayCount = 5;
            
            for (int i = 0; i < rayCount; i++)
            {
                float angle = (i - (rayCount - 1) / 2f) * (fanAngle / (rayCount - 1));
                Vector3 rayDirection = Quaternion.Euler(0, angle, 0) * forwardDirection;
                
                // 绘制调试射线
                Debug.DrawRay(transform.position, rayDirection * distance, Color.yellow, 0.1f);
                
                if (Physics.Raycast(transform.position, rayDirection, out RaycastHit hit, distance, obstacleLayer))
                {
                    if (hit.collider.gameObject == this.gameObject)
                        continue;
                        
                    Debug.DrawRay(transform.position, rayDirection * hit.distance, Color.red, 0.1f);
                    obstacleDetected = true;
                }
            }
            
            return obstacleDetected;
        }

        private float GetForwardObstacleDistance()
        {
            Vector3 forwardDirection = rb.velocity.magnitude > 0.1f ? 
                rb.velocity.normalized : transform.forward;
            
            float minDistance = float.MaxValue;
            float fanAngle = 90f;
            int rayCount = 7;
            
            for (int i = 0; i < rayCount; i++)
            {
                float angle = (i - (rayCount - 1) / 2f) * (fanAngle / (rayCount - 1));
                Vector3 rayDirection = Quaternion.Euler(0, angle, 0) * forwardDirection;
                
                if (Physics.Raycast(transform.position, rayDirection, out RaycastHit hit, detectionRadius * 2, obstacleLayer))
                {
                    if (hit.collider.gameObject == this.gameObject)
                        continue;
                        
                    minDistance = Mathf.Min(minDistance, hit.distance);
                }
            }
            
            return minDistance < float.MaxValue ? minDistance : detectionRadius * 2;
        }
        # endregion

        #region Emergency Recovery
        private IEnumerator EmergencyRecovery()
        {
            // Stop
            if (rb != null)
            {
                rb.velocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
            }
            
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
                
                // Update occupancy for last cell space
                if (cellSpaceEntryTimes.TryGetValue(lastCellSpace.Id, out float oldEntryTime))
                {
                    personalOccupancyLayer.SetPlannedOccupancy(
                        lastCellSpace.Id,
                        oldEntryTime,
                        currentTime,
                        Id.ToString(), 
                        currentPlanId
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
                    currentPlanId
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

        # region Visualization
        private void OnDrawGizmos()
        {
            if (!Application.isPlaying) return;
            
            // Visualize detection radius
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(transform.position, detectionRadius);
            
            // Visualize detection coverage
            Gizmos.color = new Color(1, 1, 0, 0.1f); 
            Gizmos.DrawSphere(transform.position, detectionRadius);
            
            // Display robot ID and state
            UnityEditor.Handles.Label(transform.position + Vector3.up * 2, 
                $"Robot {Id}\nState: {(IsMoving ? "Moving" : IsPicking ? "Picking" : "Idle")}\nOrders: {ordersQueue.Count}");
            
            // Visualize velocity
            if (rb != null)
            {
                Gizmos.color = Color.blue;
                Gizmos.DrawLine(transform.position, transform.position + rb.velocity * 0.5f);
            }
            
            // Visualize target indicator
            if (currentOrder != null && finalDestination != Vector3.zero)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawRay(transform.position, (finalDestination - transform.position).normalized);
            }
            
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
                
                // Highligh current path segment
                if (currentPathIndex < currentPath.Count)
                {
                    Vector3 currentPos = transform.position;
                    Vector3 targetPos = GetPointVector3((Point)currentPath[currentPathIndex].Point);
                    
                    Gizmos.color = Color.green;
                    Gizmos.DrawLine(currentPos, targetPos);
                    Gizmos.DrawSphere(targetPos, 0.2f);
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
        # endregion
        
        # region Helper Methods
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
        # endregion
    }
}