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
using CentralControl.PickingQueueControl;


namespace CentralControl.RobotControl
{
    public enum RobotBehavior
    {
        Idle,
        Moving,
        Picking,
        Queuing,
        AdvancingInQueue,
        PickingPending,
        EmergencyStopped
    }
    
    public enum VerticalTransportType
    {
        None,
        Elevator,
        Stairs,
        Escalator
    }
    
    public class RobotController : MonoBehaviour
    {
        #region Inspector Fields
        public Transform targetIndicator;
        public SearchAlgorithm selectedAlgorithm = SearchAlgorithm.AnytimeSIPP;
        public SearchMethod selectedSearchMethod = SearchMethod.Manhattan_Distance;
        public float baseDetectionRadius = 1.0f;
        public float emergencyStopDistance = 0.5f;
        public float defaultSpeed = 2.0f;
        public float maxAcceleration = 2.0f;
        public float maxDeceleration = 5.0f;
        public float maxJerk = 8.0f;
        public int MaxOrder = 20;
        public float floorHeight = 3.0f;
        #endregion

        #region Public Properties
        public int Id { get; private set; }
        public Graph graph { get; private set; }
        public IndoorSpace indoorSpace { get; private set; }
        public MapGrid mapGrid { get; private set; }
        public bool lastReportedAvailability { get; set; } = false;
        public bool lastReportedFreeStatus { get; set; } = false;
        public int lastReportedOrderCount { get; set; } = -1;
        public string CurrentFloor { get; private set; } = "1F";
        
        public RobotBehavior Behavior => currentBehavior;
        public VerticalTransportType VerticalTransport => currentVerticalTransport;
        public bool IsMoving => currentBehavior == RobotBehavior.Moving || currentBehavior == RobotBehavior.PickingPending || currentBehavior == RobotBehavior.AdvancingInQueue;
        public bool IsPicking => currentBehavior == RobotBehavior.Picking;
        public bool IsQueuing => currentBehavior == RobotBehavior.Queuing;
        public bool IsAdvancingInQueue => currentBehavior == RobotBehavior.AdvancingInQueue;
        public bool IsPickingPending => currentBehavior == RobotBehavior.PickingPending;
        public bool IsEmergencyStopped => currentBehavior == RobotBehavior.EmergencyStopped;
        public bool IsUsingElevator => currentVerticalTransport == VerticalTransportType.Elevator;
        public bool IsUsingStairs => currentVerticalTransport == VerticalTransportType.Stairs;
        public bool IsVerticallyMoving => currentVerticalTransport != VerticalTransportType.None;
        public bool IsFree => currentBehavior == RobotBehavior.Idle;
        public bool IsOnTask => currentBehavior != RobotBehavior.Idle;
        public bool IsAvailable => ordersQueue.Count < MaxOrder;
        public bool IsProcessingOrders => isProcessingOrdersRunning;
        public Order CurrentOrder => currentOrder;
        public int GetRobotOrdersQueueCount => ordersQueue.Count;
        public List<string> listOrdersQueue => new List<string>(ordersQueue.Select(order => order.Id));
        public bool IsBackingUp => isBackingUp;
        public Vector3 CurrentVelocity => rb != null ? rb.velocity : Vector3.zero;
        public bool IsActuallyMoving => rb != null && rb.velocity.magnitude > 0.1f;
        public Vector3 TargetPosition => targetPosition;
        public List<ConnectionPoint> CurrentPath => currentPath;
        public int CurrentPathIndex => currentPathIndex;

        public Vector3 GetLastOrderDestination()
        {
            if (ordersQueue.IsEmpty && currentOrder == null)
            {
                return transform.position;
            }
            
            if (ordersQueue.IsEmpty && currentOrder != null)
            {
                 return currentOrder.Destination;
            }

            Order lastOrder = ordersQueue.ToArray().LastOrDefault();
            if (lastOrder != null)
            {
                return lastOrder.Destination;
            }
            
            return currentOrder != null ? currentOrder.Destination : transform.position;
        }
        #endregion

        #region Private Fields
        private DynamicOccupancyLayer personalOccupancyLayer;
        private DynamicOccupancyLayer globalOccupancyLayer;
        private PathPlanner pathPlanner;
        private Rigidbody rb;
        private ConcurrentQueue<Order> ordersQueue = new ConcurrentQueue<Order>();
        private Order currentOrder;
        private Coroutine currentMovementCoroutine;
        private RobotBehavior currentBehavior = RobotBehavior.Idle;
        private VerticalTransportType currentVerticalTransport = VerticalTransportType.None;
        private float detectionRadius;
        private bool isWaitingForPicking = false;
        private bool isProcessingOrdersRunning = false;
        private int isProcessingOrder = 0;
        private List<ConnectionPoint> currentPath = new List<ConnectionPoint>();
        private List<PathSegment> currentMovement = new List<PathSegment>();
        private int currentPathIndex = 0;
        private bool pickingCompleted = false;
        private string currentPlanId;
        private LayerMask obstacleLayer;
        private IntersectionAreaManager intersectionManager;
        private string currentIntersectionId;
        private bool isInitialized = false;
        private Vector3 lastPosition;
        private float lastPositionChangeTime;
        private Vector3 finalDestination;
        private Vector3 targetPosition;
        private bool navigationEnabled = true;
        private float lastEmergencyStopTime = 0f;
        private const float EMERGENCY_RECOVERY_DELAY = 0.5f;
        private LayerMask robotCollisionLayer;
        private LayerMask pickerCollisionLayer;
        private CellSpace lastCellSpace = null;
        private CellSpace firstCellSpace = null;
        private Dictionary<string, float> cellSpaceEntryTimes = new Dictionary<string, float>();
        
        // Floor transition related fields
        private float baseHeight = 0.05f;
        
        // Path recovery
        private bool isReplanning = false;
        private bool needsPathRecovery = false;
        private float lastPathRecoveryTime = 0f;
        private const int MAX_RECOVERY_ATTEMPTS = 5;
        private const float RECOVERY_INTERVAL = 1.0f;
        
        // Conflict resolution
        private Coroutine waitForExecutionCoroutine;
        private List<float> pathTimes = new List<float>();
        private float currentSpeed = 0f;
        private float currentAcceleration = 0f;
        
        // Collision
        private List<GameObject> recentCollisions = new List<GameObject>();
        private float collisionClearInterval = 2.0f;
        private float lastCollisionClearTime = 0f;
        private float pathPointAcceptanceRadius = 0.5f;
        private float finalDestinationAcceptanceRadius = 1.0f;
        private bool hasReportedArrival = false;
        
        // Collision detection optimization
        private Coroutine collisionCheckCoroutine;
        private Coroutine obstacleCarvingCoroutine;
        private const float COLLISION_CHECK_INTERVAL = 0.05f; // 20Hz detection frequency
        private bool isBackingUp = false;
        private RobotBehavior behaviorBeforeBackup = RobotBehavior.Idle;
        
        // Path history for backup recovery
        private List<Vector3> pathHistory = new List<Vector3>();
        private const int PATH_HISTORY_SIZE = 10;
        
        // Local collision recovery manager
        private CollisionRecoveryManager collisionRecoveryManager;
        #endregion

        #region Event
        public event EventHandler<RobotPathEventArgs> OnPathPlanned;
        protected virtual void RaisePathPlannedEvent(string planId, List<ConnectionPoint> path, List<float> times, List<float> speeds)
        {
            OnPathPlanned?.Invoke(this, new RobotPathEventArgs(Id.ToString(), planId, path, times, speeds));
        }
        #endregion

        #region Unity Lifecycle Methods
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

        private void Start()
        {
            // Register for events
            if (EventManager.Instance != null)
            {
                EventManager.Instance.OnRobotPathModified += HandlePathModified;
                EventManager.Instance.OnRobotPathAccepted += HandlePathAccepted;
                EventManager.Instance.OnPickingCompleted += HandlePickingCompletedEvent;
                EventManager.Instance.OnGlobalOccupancyLayerUpdated += HandleGlobalOccupancyLayerUpdated;
                
                PickingQueueManager.Instance.OnRobotAllowedToPick += HandleRobotAllowedToPick;
                PickingQueueManager.Instance.OnQueueAdvance += HandleQueueAdvance;
            }

            // Initialize target indicator
            if (targetIndicator != null)
            {
                targetIndicator.gameObject.SetActive(false);
            }
        }

        private void Update()
        {
            if (!isInitialized) return;

            UpdatePersonalLayer();
            UpdateRobotStatus();
            RecordPathPosition();

            lastPosition = transform.position;
            lastPositionChangeTime = Time.time;
            
            if (navigationEnabled && !IsEmergencyStopped && !isWaitingForPicking && IsMoving)
            {
                UpdateMovement();
            }
            
            UpdateTargetIndicator();
            
            if (IsEmergencyStopped || needsPathRecovery)
            {
                VisualizeEmergencyState();
            }
            
            if (intersectionManager != null)
            {
                CheckIntersectionStatus();
            }
        }

        private void VisualizeEmergencyState()
        {
            // Visualization for emergency state in the editor
            Debug.DrawLine(transform.position, transform.position + Vector3.up * 2, Color.red);
            Debug.DrawLine(transform.position, transform.position + transform.forward * 1.5f, Color.red);
        }

        #endregion

        #region Initialization
        public void InitializeRobot(int id, IndoorSpace indoorSpace, Graph graph, MapGrid mapGrid,
                                    DynamicOccupancyLayer globalOccupancyLayer,
                                    IntersectionAreaManager intersectionManager)
        {
            // Validate critical dependencies before initialization
            if (indoorSpace == null)
            {
                Debug.LogError($"Robot {id}: InitializeRobot called with null indoorSpace - initialization aborted");
                return;
            }
            if (graph == null)
            {
                Debug.LogError($"Robot {id}: InitializeRobot called with null graph - initialization aborted");
                return;
            }

            this.Id = id;
            this.indoorSpace = indoorSpace;
            this.graph = graph;
            this.mapGrid = mapGrid;
            this.globalOccupancyLayer = globalOccupancyLayer;
            this.intersectionManager = intersectionManager;

            // Determine current floor level from position
            CurrentFloor = GetFloorLevelFromPosition(transform.position);
            Debug.Log($"Robot {Id} initialized at {transform.position} on floor {CurrentFloor}");

            ConfigureNavMeshObstacle();
            CreateInteractionTrigger();
            obstacleCarvingCoroutine = StartCoroutine(ManageObstacleCarving());

            personalOccupancyLayer = new DynamicOccupancyLayer(indoorSpace, 300f, Time.time);
            
            pathPlanner = new PathPlanner(graph, indoorSpace, selectedAlgorithm, selectedSearchMethod, 
                                          id.ToString(), maxAcceleration, maxDeceleration);

            collisionRecoveryManager = new CollisionRecoveryManager(this);

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
            
            StartCollisionDetection();
            isInitialized = true;
        }

        private void OnDestroy()
        {
            // Stop coroutines to prevent MissingReferenceException
            if (obstacleCarvingCoroutine != null)
            {
                StopCoroutine(obstacleCarvingCoroutine);
                obstacleCarvingCoroutine = null;
            }
            if (collisionCheckCoroutine != null)
            {
                StopCoroutine(collisionCheckCoroutine);
                collisionCheckCoroutine = null;
            }

            collisionRecoveryManager?.Cleanup();

            if (EventManager.Instance != null)
            {
                EventManager.Instance.OnRobotPathModified -= HandlePathModified;
                EventManager.Instance.OnRobotPathAccepted -= HandlePathAccepted;
                EventManager.Instance.OnPickingCompleted -= HandlePickingCompletedEvent;
                EventManager.Instance.OnGlobalOccupancyLayerUpdated -= HandleGlobalOccupancyLayerUpdated;
            }

            PickingQueueManager.Instance.OnRobotAllowedToPick -= HandleRobotAllowedToPick;
            PickingQueueManager.Instance.OnQueueAdvance -= HandleQueueAdvance;
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
        #endregion

        #region Order Processing
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
            
            EventManager.Instance?.TriggerOrderAssigned(order.Id, Id.ToString());
            
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
                
                Debug.Log($"Robot {Id} processing order {currentOrder.Id}, destination: {currentOrder.Destination}, floor: {currentOrder.FloorLevel}");
                
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
            Debug.Log($"Robot {Id} planning path to {order.Destination}");
            SetBehavior(RobotBehavior.Moving);
            
            Vector3 targetPosition = GetTargetPosition(order.PickingPoint);
            string targetFloorLevel = order.PickingPoint.FloorLevel;
            targetPosition.y = GetFloorHeight(targetFloorLevel);
            
            Debug.Log($"Robot {Id} planning path from floor {CurrentFloor} to floor {targetFloorLevel}");
            
            var result = PlanPath(targetPosition, targetFloorLevel);
            if (result.path == null || result.path.Count == 0)
            {
                Debug.LogError($"Robot {Id} failed to plan path to {order.Destination}");
                SetBehavior(RobotBehavior.Idle);
                ReportOrderFailed(order.Id, "PathPlanningFailed");
                yield break;
            }

            currentPlanId = result.planId;
            RaisePathPlannedEvent(currentPlanId, result.path, result.times, result.speeds);
            personalOccupancyLayer.RecordPathOccupancy(result.path, result.times, this.Id.ToString(), currentPlanId);
            UpdateCurrentMovementInfo(result.path, result.times, result.speeds);
            
            float timeout = 300f;
            float startTime = Time.time;
            
            while (!hasReportedArrival && Time.time - startTime < timeout)
            {
                if (!IsMoving && !IsVerticallyMoving && !IsQueuing && !IsEmergencyStopped)
                {
                    Debug.LogWarning($"Robot {Id} stopped moving before reaching destination. State: {currentBehavior}, Vertical: {currentVerticalTransport}");
                    break;
                }
                
                yield return null;
            }
            
            if (!hasReportedArrival)
            {
                Debug.LogWarning($"Robot {Id} timed out while trying to reach destination");
                SetBehavior(RobotBehavior.Idle);
                ReportOrderFailed(order.Id, "NavigationTimeout");
                yield break;
            }
            
            Debug.Log($"Robot {Id} waiting for picking at {transform.position}");
            SetBehavior(RobotBehavior.Picking);
            
            pickingCompleted = false;
            yield return StartCoroutine(WaitForPicking(order, transform.position));
            
            SetBehavior(RobotBehavior.Idle);
            
            ReportOrderCompleted(order.Id);
            Debug.Log($"Robot {Id} completed picking for order {order.Id}");
        }

        private void ReportOrderCompleted(string orderId)
        {
            if (EventManager.Instance != null)
            {
                EventManager.Instance.TriggerOrderCompleted(orderId, Id.ToString());
            }
        }

        private void ReportOrderFailed(string orderId, string reason)
        {
            if (EventManager.Instance != null)
            {
                EventManager.Instance.TriggerOrderFailed(orderId, Id.ToString(), reason);
            }
        }

        private IEnumerator WaitForPicking(Order order, Vector3 location)
        {
            isWaitingForPicking = true;
            rb.velocity = Vector3.zero; // Stop moving
            
            // Notify picker
            EventManager.Instance.TriggerRobotReadyForPicking(Id.ToString(), order.PickingPoint, location);
            
            // Wait until picking is completed
            float timeout = 10f;  // 1 minutes timeout
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
            NotifyPickingCompleted();
        }

        public void NotifyPickingCompleted()
        {
            if (currentOrder != null && currentOrder.PickingPoint != null)
            {
                PickingQueueManager.Instance.PickingCompleted(currentOrder.PickingPoint.Id, Id.ToString());
            }

            pickingCompleted = true;
            Debug.Log($"Robot {Id} notified that picking was completed");
        }
        #endregion

        #region Path Planning and Movement
        private Vector3 GetTargetPosition(PickingPoint pickingPoint)
        {
            // Defensive null checks
            if (indoorSpace == null || pickingPoint == null)
            {
                Debug.LogWarning($"Robot {Id}: GetTargetPosition called with null dependencies");
                return Vector3.zero;
            }

            CellSpace cellSpace = indoorSpace.GetCellSpaceFromId(pickingPoint.Id);
            if (cellSpace == null)
            {
                Debug.LogWarning($"Robot {Id}: Could not find CellSpace for picking point {pickingPoint.Id}");
                return Vector3.zero;
            }

            Point point = (Point)cellSpace.Node;
            float y = GetFloorHeight(pickingPoint.FloorLevel);
            return new Vector3((float)point.X, y, (float)point.Y);
        }

        private (List<ConnectionPoint> path, List<float> times, List<float> speeds, string planId, PathPlanningMetrics metrics) PlanPath(Vector3 targetPosition, string targetFloorLevel = null)
        {
            currentPath.Clear();
            currentMovement.Clear();
            finalDestination = targetPosition;
            hasReportedArrival = false;

            // Clear old planned occupancy before planning new path
            // This prevents conflicts with our own old reservations
            if (!string.IsNullOrEmpty(currentPlanId))
            {
                personalOccupancyLayer.ClearPlannedOccupancy(Id.ToString(), currentPlanId, Time.time);
                Debug.Log($"Robot {Id}: Cleared old planned occupancy for plan {currentPlanId}");
            }

            RoutePoint startPoint = graph.GetRoutePointFromCoordinate(transform.position, true);
            RoutePoint endPoint = graph.GetRoutePointFromCoordinate(targetPosition, false);

            Debug.Log($"Robot {Id}: Planning path from {transform.position} (Floor {CurrentFloor}) to {targetPosition} (Floor {targetFloorLevel})");

            if (startPoint == null || endPoint == null)
            {
                // Detailed diagnostic logging to identify which point failed
                if (startPoint == null)
                {
                    Debug.LogError($"Robot {Id} cannot find start RoutePoint at position {transform.position} (Floor {CurrentFloor}). " +
                        $"Check if robot is spawned inside a valid navigable CellSpace.");
                }
                if (endPoint == null)
                {
                    Debug.LogError($"Robot {Id} cannot find end RoutePoint at position {targetPosition} (Floor {targetFloorLevel}). " +
                        $"Check if target PickingPoint has valid CellBoundary connections.");
                }
                return (null, null, null, null, null);
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

        private bool HasReachedFinalDestination()
        {
            if (currentOrder == null)
                return false;

            // Defensive null check
            if (indoorSpace == null)
                return false;

            CellSpace cellSpace = indoorSpace.GetCellSpaceFromId(currentOrder.PickingPoint.Id);
            if (cellSpace == null) 
                return false;
            
            // Check if in the cell containing the picking point
            CellSpace currentCell = indoorSpace.GetCellSpaceFromCoordinates(transform.position, GetFloorLevelFromPosition(transform.position));
            if (currentCell != null)
            {
                // Check distance to the center of the picking point
                float distanceToPickingPoint = Vector3.Distance(transform.position, finalDestination);
                return distanceToPickingPoint < finalDestinationAcceptanceRadius;
            }
            
            return false;
        }

        private void HandleDestinationReached()
        {
            if (currentOrder == null) return;
            
            rb.velocity = Vector3.zero;
            
            if (IsPickingPending)
            {
                Debug.Log($"Robot {Id}: PickingPending robot reached picking point {currentOrder.PickingPoint.Id} - Starting Picking");
                isWaitingForPicking = true;
                hasReportedArrival = true;
                return;
            }
            
            if (!hasReportedArrival)
            {
                bool canPick = PickingQueueManager.Instance.RegisterArrival(currentOrder.PickingPoint.Id, Id.ToString());
                
                if (canPick)
                {
                    Debug.Log($"Robot {Id}: Reached picking point {currentOrder.PickingPoint.Id} - Starting Picking");
                    isWaitingForPicking = true;
                    hasReportedArrival = true;
                }
                else
                {
                    if (!IsQueuing)
                    {
                        Debug.Log($"Robot {Id}: Reached picking point {currentOrder.PickingPoint.Id} - Queuing");
                        SetBehavior(RobotBehavior.Queuing);
                    }
                }
            }
        }

        private void HandleRobotAllowedToPick(string robotId)
        {
            if (robotId == Id.ToString() && (IsQueuing || IsAdvancingInQueue) && currentOrder != null)
            {
                Debug.Log($"Robot {Id}: Allowed to start picking from queue, entering PickingPending state");
                SetBehavior(RobotBehavior.PickingPending);

                if (HasReachedFinalDestination())
                {
                    Debug.Log($"Robot {Id}: Already at picking point, starting picking immediately");
                    hasReportedArrival = true;
                    isWaitingForPicking = true;
                }
            }
        }

        /// <summary>
        /// Handle notification to advance in queue when a front robot moves forward
        /// </summary>
        private void HandleQueueAdvance(string pickingPointId, string robotId)
        {
            if (robotId != Id.ToString()) return;
            if (currentOrder?.PickingPoint?.Id != pickingPointId) return;

            if (IsQueuing)
            {
                Debug.Log($"Robot {Id}: Received queue advance notification, entering AdvancingInQueue state");
                SetBehavior(RobotBehavior.AdvancingInQueue);
            }
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
        }
        #endregion

        #region Movement Control
        private void UpdateMovement()
        {
            if (!IsMoving)
                return;
            
            if (IsPickingPending && (currentPath == null || currentPath.Count == 0 || currentPathIndex >= currentPath.Count))
            {
                if (finalDestination != Vector3.zero && !HasReachedFinalDestination())
                {
                    MoveDirectlyToDestination(finalDestination);
                    return;
                }
                else if (HasReachedFinalDestination())
                {
                    HandleDestinationReached();
                    return;
                }
            }
            
            if (IsAdvancingInQueue && (currentPath == null || currentPath.Count == 0 || currentPathIndex >= currentPath.Count))
            {
                if (finalDestination != Vector3.zero && !HasReachedFinalDestination())
                {
                    MoveDirectlyToDestination(finalDestination);
                    return;
                }
            }
            
            if (currentPath == null || currentPath.Count == 0 || currentPathIndex >= currentPath.Count)
                return;
            
            ConnectionPoint targetPoint = currentPath[currentPathIndex];
            
            Vector3 targetPos = GetConnectionPointPosition(targetPoint);
            
            targetPosition = targetPos;
            
            float distanceToTarget = Vector3.Distance(transform.position, targetPos);
            
            if (distanceToTarget < pathPointAcceptanceRadius)
            {
                currentPathIndex++;

                if (currentPathIndex >= currentPath.Count)
                {
                    Debug.Log($"Robot {Id}: Path connection points completed");
                    
                    if (currentOrder != null && finalDestination == Vector3.zero && currentPath.Count > 0)
                    {
                        ConnectionPoint lastPoint = currentPath[currentPath.Count - 1];
                        
                        MoveFromConnectionPointToTargetCellSpace(lastPoint);
                        
                        currentPathIndex = 0;
                        currentPath = new List<ConnectionPoint> { lastPoint };
                    }
                    else
                    {
                        if (HasReachedFinalDestination())
                        {
                            HandleDestinationReached();
                        }
                        else
                        {
                            Debug.Log($"Robot {Id}: Path completed but not at final destination yet.");
                        }
                        
                        SetBehavior(RobotBehavior.Idle);
                        
                        rb.velocity = Vector3.zero;
                    }
                    
                    currentSpeed = 0f;
                    currentAcceleration = 0f;
                    return;
                }
                
                return;
            }
            
            float targetSpeed = currentPathIndex < currentMovement.Count ? 
                currentMovement[currentPathIndex].Speed : defaultSpeed;
            
            Vector3 direction = (targetPos - transform.position).normalized;
            
            float dt = Time.deltaTime;
            currentAcceleration = Mathf.MoveTowards(currentAcceleration, 0f, maxJerk * dt);
            float desiredAcceleration = (targetSpeed - currentSpeed) / Mathf.Max(dt, 0.001f);
            desiredAcceleration = Mathf.Clamp(desiredAcceleration, -maxDeceleration, maxAcceleration);
            currentAcceleration = Mathf.MoveTowards(currentAcceleration, desiredAcceleration, maxJerk * dt);
            currentSpeed = Mathf.Max(0f, currentSpeed + currentAcceleration * dt);
            currentSpeed = Mathf.Min(currentSpeed, targetSpeed);
            
            Vector3 targetVelocity = direction * currentSpeed;
            rb.velocity = targetVelocity;
            
            Vector3 horizontalDirection = new Vector3(direction.x, 0, direction.z).normalized;
            if (horizontalDirection.magnitude > 0.2f)
            {
                Quaternion targetRotation = Quaternion.LookRotation(horizontalDirection);
                transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, Time.deltaTime * 10f);
            }
        }

        private void MoveFromConnectionPointToTargetCellSpace(ConnectionPoint targetPoint)
        {
            if (targetPoint == null)
                return;

            // Defensive null check
            if (indoorSpace == null)
                return;

            string targetCellSapceId = targetPoint.CellBoundary.Target;
            CellSpace targetCellSpace = indoorSpace.GetCellSpaceFromId(targetCellSapceId);
            
            Point point = (Point)targetCellSpace.Node;
            float y = GetFloorHeight(targetCellSpace.GetLocatedFloor());
            
            Vector3 targetPos = new Vector3((float)point.X, y, (float)point.Y);
            
            finalDestination = targetPos;
        }
        
        private void MoveDirectlyToDestination(Vector3 destination)
        {
            Vector3 direction = (destination - transform.position).normalized;
            float distance = Vector3.Distance(transform.position, destination);
            
            if (distance < finalDestinationAcceptanceRadius)
            {
                rb.velocity = Vector3.zero;
                HandleDestinationReached();
                return;
            }
            
            float targetSpeed = defaultSpeed * 0.5f;
            float dt = Time.deltaTime;
            
            currentAcceleration = Mathf.MoveTowards(currentAcceleration, 0f, maxJerk * dt);
            float desiredAcceleration = (targetSpeed - currentSpeed) / Mathf.Max(dt, 0.001f);
            desiredAcceleration = Mathf.Clamp(desiredAcceleration, -maxDeceleration, maxAcceleration);
            currentAcceleration = Mathf.MoveTowards(currentAcceleration, desiredAcceleration, maxJerk * dt);
            currentSpeed = Mathf.Max(0f, currentSpeed + currentAcceleration * dt);
            currentSpeed = Mathf.Min(currentSpeed, targetSpeed);
            
            Vector3 targetVelocity = direction * currentSpeed;
            rb.velocity = targetVelocity;
            
            if (direction != Vector3.zero)
            {
                Quaternion targetRotation = Quaternion.LookRotation(direction);
                transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, Time.deltaTime * 5f);
            }
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

        private void SetBehavior(RobotBehavior behavior, VerticalTransportType verticalTransport = VerticalTransportType.None)
        {
            RobotBehavior previousBehavior = currentBehavior;
            VerticalTransportType previousVertical = currentVerticalTransport;
            
            currentBehavior = behavior;
            currentVerticalTransport = (behavior == RobotBehavior.Moving) ? verticalTransport : VerticalTransportType.None;
            
            if (previousBehavior != behavior || previousVertical != currentVerticalTransport)
            {
                string stateInfo = currentVerticalTransport != VerticalTransportType.None 
                    ? $"{behavior} ({currentVerticalTransport})" 
                    : behavior.ToString();
                Debug.Log($"Robot {Id}: Behavior changed to {stateInfo}");
                EventManager.Instance?.TriggerRobotStateChanged(Id.ToString(), behavior.ToString(), true);
            }
        }

        public bool IsStuck
        {
            get
            {
                if (!IsMoving || currentPath.Count == 0)
                    return false;
                    
                float timeSinceLastMove = Time.time - lastPositionChangeTime;
                float distanceMoved = Vector3.Distance(lastPosition, transform.position);
                
                return timeSinceLastMove > 5.0f && distanceMoved < 0.1f;
            }
        }

        public void ResetRobot()
        {
            StopAllCoroutines();
            SetBehavior(RobotBehavior.Idle);
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
            
            needsPathRecovery = true;
            lastPathRecoveryTime = Time.time;
            
            if (IsMoving || IsVerticallyMoving)
            {
                StopMoving();
                SetBehavior(RobotBehavior.Idle);
            }
            
            StartCoroutine(RecoverAfterDelay(1.0f));
        }

        private void StopMoving()
        {
            if (rb != null)
            {
                rb.velocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
            }
        }

        private IEnumerator RecoverAfterDelay(float delay)
        {
            yield return new WaitForSeconds(delay);
            
            // Reset status
            needsPathRecovery = false;
            
            // Re-enable navigation based on current state
            navigationEnabled = true;
            
            // If there's a current order, attempt to replan
            if (currentOrder != null)
            {
                // Replan path to destination
                Debug.Log($"Robot {Id}: Recover with replan to current order destination");
                StartCoroutine(ReplanPath());
            }
            
            UpdateTargetIndicator();
        }

        private IEnumerator ReplanPath()
        {
            isReplanning = true;
            Debug.Log($"Robot {Id}: Replanning path to destination at time {Time.time}");
            
            if (currentOrder == null)
            {
                Debug.LogWarning($"Robot {Id}: No current order to replan path for");
                isReplanning = false;
                yield break;
            }
            
            bool wasNavigationEnabled = navigationEnabled;
            navigationEnabled = false;
            
            yield return new WaitForSeconds(0.5f);
            
            if (currentOrder == null || currentOrder.PickingPoint == null)
            {
                Debug.LogWarning($"Robot {Id}: Order became null during replan wait");
                isReplanning = false;
                navigationEnabled = wasNavigationEnabled;
                yield break;
            }
            
            Vector3 finalTarget = GetTargetPosition(currentOrder.PickingPoint);
            string targetFloorLevel = currentOrder.PickingPoint.FloorLevel;
            
            needsPathRecovery = false;
            
            int attempts = 0;
            lastPathRecoveryTime = Time.time;
            bool pathReplanned = false;

            while (attempts < MAX_RECOVERY_ATTEMPTS && !pathReplanned)
            {
                Debug.Log($"Robot {Id}: Attempting to replan path to {finalTarget} (Attempt {attempts + 1})");
                
                var result = PlanPath(finalTarget, targetFloorLevel);
                
                if (result.path != null && result.path.Count > 0)
                {
                    currentPlanId = result.planId;
                    RaisePathPlannedEvent(currentPlanId, result.path, result.times, result.speeds);
                    
                    personalOccupancyLayer.RecordPathOccupancy(result.path, result.times, this.Id.ToString(), currentPlanId);
                    
                    UpdateCurrentMovementInfo(result.path, result.times, result.speeds);
                    currentPathIndex = 0;
                    
                    Debug.Log($"Robot {Id}: Replanning path succeeded on attempt {attempts + 1}");
                    
                    SetBehavior(RobotBehavior.Moving);
                    pathReplanned = true;
                    
                    float startDelay = 0.5f;
                    if (waitForExecutionCoroutine != null)
                    {
                        StopCoroutine(waitForExecutionCoroutine);
                    }
                    waitForExecutionCoroutine = StartCoroutine(WaitForExecution(startDelay));
                    
                    break;
                }
                
                attempts++;
                yield return new WaitForSeconds(RECOVERY_INTERVAL);
            }
            
            navigationEnabled = wasNavigationEnabled;
            
            if (!pathReplanned)
            {
                Debug.LogError($"Robot {Id}: Failed to replan path after {attempts} attempts");
            }
            
            isReplanning = false;
        }
        #endregion

        #region Occupancy Layer
        public DynamicOccupancyLayer GetPersonalOccupancyLayer()
        {
            return personalOccupancyLayer;
        }

        public void UpdatePersonalLayer()
        {
            // Defensive null check - indoorSpace might be null during scene transitions or if initialization failed
            if (indoorSpace == null)
            {
                return;
            }

            CellSpace currentCellSpace = indoorSpace.GetCellSpaceFromCoordinates(transform.position, GetFloorLevelFromPosition(transform.position));
            if (currentCellSpace == null)
                return;

            float currentTime = Time.time;
            
            if (lastCellSpace == null)
            {
                lastCellSpace = currentCellSpace;
                cellSpaceEntryTimes[currentCellSpace.Id] = currentTime;
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
                    
                    // Mark this robot's layer as dirty for incremental merge
                    if (CentralController.Instance != null)
                    {
                        CentralController.Instance.MarkRobotLayerDirty(Id);
                    }
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
                
                // Mark this robot's layer as dirty for incremental merge
                if (CentralController.Instance != null)
                {
                    CentralController.Instance.MarkRobotLayerDirty(Id);
                }
            }
        }

        private void SavePathToPersonalLayer(List<ConnectionPoint> path, List<float> times, string planId)
        {
            // Create an occupancy identifier
            var occupier = new OccupancyIdentifier(Id.ToString(), planId, OccupancyType.Planned);
            
            // Record path to personal occupancy layer
            personalOccupancyLayer.RecordPathOccupancy(path, times, Id.ToString(), planId);
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
            firstCellSpace = null;
            currentPlanId = null;
        }
        #endregion

        #region Intersection Management
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

        private void EnterIntersection(IntersectionArea intersection)
        {
            if (intersection == null) return;
            
            currentIntersectionId = intersection.Id;
            
            // Get entry and suggested exit directions
            var (entryDirection, suggestedExitDirection) = intersection.CalculateTraversalDirections(
                transform.position, rb.velocity);
        }

        private void ExitIntersection()
        {
            if (currentIntersectionId == null) return;
            
            currentIntersectionId = null;
        }
        #endregion

        #region Collision Detection Optimization
        private void StartCollisionDetection()
        {
            if (collisionCheckCoroutine != null)
            {
                StopCoroutine(collisionCheckCoroutine);
            }
            collisionCheckCoroutine = StartCoroutine(CollisionCheckRoutine());
        }

        private IEnumerator CollisionCheckRoutine()
        {
            WaitForSeconds wait = new WaitForSeconds(COLLISION_CHECK_INTERVAL);
            while (true)
            {
                if ((IsMoving || isBackingUp) && !isWaitingForPicking)
                {
                    CheckSurroundings();
                }
                yield return wait;
            }
        }

        private void RecordPathPosition()
        {
            if (!IsMoving && !isBackingUp) return;
            
            if (pathHistory.Count == 0 || 
                Vector3.Distance(pathHistory[pathHistory.Count - 1], transform.position) > 0.5f)
            {
                pathHistory.Add(transform.position);
                
                while (pathHistory.Count > PATH_HISTORY_SIZE)
                {
                    pathHistory.RemoveAt(0);
                }
            }
        }

        public Vector3? GetPreviousPathNode()
        {
            if (pathHistory.Count >= 2)
            {
                return pathHistory[pathHistory.Count - 2];
            }
            
            if (currentPath != null && currentPathIndex > 0)
            {
                var prevPoint = currentPath[currentPathIndex - 1];
                return new Vector3(
                    (float)prevPoint.Point.Coordinate.X,
                    transform.position.y,
                    (float)prevPoint.Point.Coordinate.Y
                );
            }
            
            return null;
        }

        /// <summary>
        /// Gets recent path history positions for track-aligned backup.
        /// </summary>
        /// <param name="maxCount">Maximum number of positions to return</param>
        /// <returns>List of recent positions, most recent first</returns>
        public List<Vector3> GetRecentPathHistory(int maxCount = 3)
        {
            var result = new List<Vector3>();
            
            // Add from pathHistory (newest to oldest)
            for (int i = pathHistory.Count - 2; i >= 0 && result.Count < maxCount; i--)
            {
                result.Add(pathHistory[i]);
            }
            
            // If not enough, try to get from planned path
            if (result.Count < maxCount && currentPath != null && currentPathIndex > 0)
            {
                for (int i = currentPathIndex - 1; i >= 0 && result.Count < maxCount; i--)
                {
                    var point = currentPath[i];
                    var pos = new Vector3(
                        (float)point.Point.Coordinate.X,
                        transform.position.y,
                        (float)point.Point.Coordinate.Y
                    );
                    
                    // Avoid duplicates
                    if (result.Count == 0 || Vector3.Distance(result[result.Count - 1], pos) > 0.3f)
                    {
                        result.Add(pos);
                    }
                }
            }
            
            return result;
        }

        /// <summary>
        /// Checks if a backup target position is safe to move to.
        /// Validates: no obstacles, within navigable area, no lane violations.
        /// </summary>
        /// <param name="target">Target position to check</param>
        /// <returns>True if safe, false otherwise</returns>
        public bool IsSafeBackupTarget(Vector3 target, bool allowLateralWithinCell = false)
        {
            float robotRadius = 1.5f;
            var robotCollider = GetComponent<Collider>();
            if (robotCollider != null)
            {
                var extents = robotCollider.bounds.extents;
                robotRadius = Mathf.Max(extents.x, extents.z) * 1.1f;
            }
            
            // 1. Check if path from current position to target is clear
            Vector3 direction = (target - transform.position).normalized;
            float distance = Vector3.Distance(transform.position, target);
            
            if (distance < 0.1f)
                return false; // Too close, not useful
            
            // Check for obstacles along the path
            if (Physics.SphereCast(transform.position, robotRadius * 0.8f, direction, out RaycastHit hit, distance, obstacleLayer))
            {
                // There's an obstacle in the way
                if (hit.collider.gameObject != gameObject)
                {
                    Debug.Log($"Robot {Id}: Backup target {target} blocked by {hit.collider.name}");
                    return false;
                }
            }
            
            // 2. Check if target position has no obstacles
            Collider[] overlaps = Physics.OverlapSphere(target, robotRadius * 0.6f, obstacleLayer);
            foreach (var collider in overlaps)
            {
                if (collider.gameObject != gameObject)
                {
                    Debug.Log($"Robot {Id}: Backup target {target} occupied by {collider.name}");
                    return false;
                }
            }
            
            // 3. Check if target is within a valid navigable cell
            if (indoorSpace != null)
            {
                var point = new NetTopologySuite.Geometries.Point(target.x, target.z);
                CellSpace cell = null;
                
                foreach (var cellSpace in indoorSpace.CellSpaces)
                {
                    if (cellSpace.Space.Contains(point))
                    {
                        cell = cellSpace;
                        break;
                    }
                }
                
                if (cell == null)
                {
                    Debug.Log($"Robot {Id}: Backup target {target} not in any navigable cell");
                    return false;
                }
                
                // 4. Check for one-way lane violations (if applicable)
                // The cell's navigability doesn't inherently encode direction,
                // but we can check if we're moving against traffic flow
                // by comparing backup direction with planned path direction
                if (!allowLateralWithinCell && currentPath != null && currentPath.Count > 1 && currentPathIndex < currentPath.Count)
                {
                    // Get the planned direction at current position
                    var currentTarget = currentPath[currentPathIndex];
                    Vector3 plannedDirection = new Vector3(
                        (float)currentTarget.Point.Coordinate.X - transform.position.x,
                        0,
                        (float)currentTarget.Point.Coordinate.Y - transform.position.z
                    ).normalized;
                    
                    Vector3 backupDirection = (target - transform.position).normalized;
                    backupDirection.y = 0;
                    
                    // Check if backup direction is too perpendicular to aisle
                    // (which would indicate leaving the lane)
                    float dotWithPlanned = Vector3.Dot(plannedDirection, backupDirection);
                    
                    // Allow backup (opposite direction) but not sideways movement
                    // Dot product: -1 = opposite (OK), 0 = perpendicular (bad), 1 = same (OK)
                    if (Mathf.Abs(dotWithPlanned) < 0.3f)
                    {
                        // Sideways movement - might leave lane
                        Debug.Log($"Robot {Id}: Backup direction {backupDirection} too perpendicular to lane");
                        return false;
                    }
                }
            }
            
            return true;
        }

        /// <summary>
        /// Gets a track-aligned backup position along the current aisle/corridor.
        /// </summary>
        /// <returns>Backup position aligned with current movement direction, or null if not available</returns>
        public Vector3? GetTrackAlignedBackupPosition()
        {
            const float backupDistance = 1.5f;
            
            // If we have a current path, backup along it
            if (currentPath != null && currentPathIndex > 0 && currentPathIndex < currentPath.Count)
            {
                // Get direction from previous path point to current position
                var prevPoint = currentPath[currentPathIndex - 1];
                Vector3 prevPos = new Vector3(
                    (float)prevPoint.Point.Coordinate.X,
                    transform.position.y,
                    (float)prevPoint.Point.Coordinate.Y
                );
                
                Vector3 trackDirection = (transform.position - prevPos).normalized;
                trackDirection.y = 0;
                
                if (trackDirection.magnitude > 0.1f)
                {
                    // Backup in opposite direction (back along track)
                    Vector3 backupTarget = transform.position - trackDirection * backupDistance;
                    return backupTarget;
                }
            }
            
            // Fallback: use current forward direction
            Vector3 fallbackTarget = transform.position - transform.forward * backupDistance;
            return fallbackTarget;
        }

        /// <summary>
        /// Calculates a safe backup target using priority: track history > track-aligned > stay in place.
        /// </summary>
        /// <param name="obstaclePosition">Optional obstacle position to move away from</param>
        /// <returns>Safe backup target position</returns>
        public Vector3 CalculateSafeBackupTarget(Vector3? obstaclePosition = null)
        {
            // Priority 1: Try recent path history positions (track-aligned backup)
            var historyPositions = GetRecentPathHistory(3);
            foreach (var pos in historyPositions)
            {
                if (IsSafeBackupTarget(pos, allowLateralWithinCell: true))
                {
                    Debug.Log($"Robot {Id}: Using path history position for backup: {pos}");
                    return pos;
                }
            }
            
            // Priority 2: Try track-aligned backup
            var trackAligned = GetTrackAlignedBackupPosition();
            if (trackAligned.HasValue && IsSafeBackupTarget(trackAligned.Value))
            {
                Debug.Log($"Robot {Id}: Using track-aligned position for backup: {trackAligned.Value}");
                return trackAligned.Value;
            }
            
            // Priority 3: Try backing away from obstacle (with shorter distance)
            if (obstaclePosition.HasValue)
            {
                Vector3 awayFromObstacle = (transform.position - obstaclePosition.Value).normalized;
                awayFromObstacle.y = 0;
                
                if (awayFromObstacle.magnitude > 0.1f)
                {
                    // Try progressively shorter distances
                    float[] distances = { 1.0f, 0.7f, 0.5f, 0.3f };
                    foreach (float dist in distances)
                    {
                        Vector3 candidate = transform.position + awayFromObstacle * dist;
                        if (IsSafeBackupTarget(candidate))
                        {
                            Debug.Log($"Robot {Id}: Using obstacle-away position for backup: {candidate} (dist={dist})");
                            return candidate;
                        }
                    }
                }
            }
            
            // Priority 4: No safe backup found - stay in place
            Debug.LogWarning($"Robot {Id}: No safe backup position found, staying in place");
            return transform.position;
        }

        /// <summary>
        /// Initiates a backup maneuver to the specified target position, then waits before resuming.
        /// Called by CollisionRecoveryManager for coordinated collision recovery.
        /// </summary>
        /// <param name="backupTarget">The position to backup to</param>
        /// <param name="waitTime">Time to wait after reaching backup position before resuming</param>
        public void StartBackupAndWait(Vector3 backupTarget, float waitTime)
        {
            if (isBackingUp) return;
            
            StartCoroutine(BackupAndWaitCoroutine(backupTarget, waitTime));
        }

        private IEnumerator BackupAndWaitCoroutine(Vector3 backupTarget, float waitTime)
        {
            behaviorBeforeBackup = currentBehavior;
            isBackingUp = true;
            navigationEnabled = false;
            
            Debug.Log($"Robot {Id}: Starting backup to {backupTarget}, will wait {waitTime:F2}s");
            
            if (rb != null)
            {
                rb.velocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
            }
            
            Vector3 backupDirection = (backupTarget - transform.position).normalized;
            backupDirection.y = 0;
            
            float backupDistance = Vector3.Distance(transform.position, backupTarget);
            float backupSpeed = defaultSpeed * 0.4f;
            float backupTime = Mathf.Min(backupDistance / backupSpeed, 1.5f);
            
            float elapsedTime = 0f;
            while (elapsedTime < backupTime)
            {
                if (rb != null)
                {
                    rb.velocity = backupDirection * backupSpeed;
                }
                
                elapsedTime += Time.deltaTime;
                yield return null;
            }
            
            if (rb != null)
            {
                rb.velocity = Vector3.zero;
            }
            
            Debug.Log($"Robot {Id}: Backup complete, waiting {waitTime:F2}s");
            EventManager.Instance?.TriggerRobotWaitStarted(Id.ToString(), "CollisionRecovery");
            
            yield return new WaitForSeconds(waitTime);
            
            EventManager.Instance?.TriggerRobotWaitEnded(Id.ToString(), "CollisionRecovery");
            isBackingUp = false;
            navigationEnabled = true;
            
            Debug.Log($"Robot {Id}: Resuming after backup and wait (was {behaviorBeforeBackup})");
            
            if (currentOrder != null)
            {
                bool wasInQueueState = behaviorBeforeBackup == RobotBehavior.Queuing || 
                                       behaviorBeforeBackup == RobotBehavior.AdvancingInQueue;
                bool isInPickingQueue = currentOrder.PickingPoint != null && 
                    PickingQueueManager.Instance.IsRobotInQueueOrActive(currentOrder.PickingPoint.Id, Id.ToString());
                
                if (ShouldQueueAtPickingPoint())
                {
                    SetBehavior(RobotBehavior.Queuing);
                    if (currentOrder.PickingPoint != null && !isInPickingQueue)
                    {
                        PickingQueueManager.Instance.RegisterArrival(currentOrder.PickingPoint.Id, Id.ToString());
                    }
                    Debug.Log($"Robot {Id}: Entering queue after backup - front robot still at picking point");
                }
                else if (wasInQueueState && isInPickingQueue && !hasReportedArrival)
                {
                    SetBehavior(RobotBehavior.AdvancingInQueue);
                    Debug.Log($"Robot {Id}: Resuming as AdvancingInQueue after backup");
                }
                else if (currentPath != null && currentPath.Count > 0 && currentPathIndex < currentPath.Count)
                {
                    if (hasReportedArrival && currentOrder.PickingPoint != null)
                    {
                        SetBehavior(RobotBehavior.PickingPending);
                        Debug.Log($"Robot {Id}: Resuming as PickingPending after backup");
                    }
                    else
                    {
                        SetBehavior(RobotBehavior.Moving);
                        Debug.Log($"Robot {Id}: Resuming as Moving after backup");
                    }
                }
                else
                {
                    Debug.Log($"Robot {Id}: Path invalid after backup, replanning");
                    StartCoroutine(ReplanPath());
                }
            }
            else
            {
                SetBehavior(RobotBehavior.Idle);
            }
        }

        private bool ShouldQueueAtPickingPoint()
        {
            if (currentOrder?.PickingPoint == null) return false;
            
            string myPickingPointId = currentOrder.PickingPoint.Id;
            LayerMask robotLayer = LayerMask.GetMask("RobotLayer");
            Collider[] hitColliders = Physics.OverlapSphere(transform.position, detectionRadius * 1.5f, robotLayer);
            
            foreach (var hitCollider in hitColliders)
            {
                if (hitCollider.gameObject == gameObject) continue;
                
                if (CentralController.Instance != null && CentralController.Instance.TryGetRobotId(hitCollider.gameObject, out int otherRobotId))
                {
                    bool otherIsPicking = CentralController.Instance.IsRobotPicking(otherRobotId);
                    bool otherInQueue = CentralController.Instance.IsRobotInPickingQueue(myPickingPointId, otherRobotId);
                    string otherPickingPointId = CentralController.Instance.GetRobotPickingPointId(otherRobotId);
                    bool samePickingPoint = otherPickingPointId == myPickingPointId;
                    
                    Vector3 toOther = (hitCollider.transform.position - transform.position).normalized;
                    toOther.y = 0;
                    float dot = Vector3.Dot(transform.forward, toOther);
                    bool inFront = dot >= 0.5f;
                    
                    if (inFront && samePickingPoint && (otherIsPicking || otherInQueue))
                    {
                        return true;
                    }
                }
            }
            
            return false;
        }

        /// <summary>
        /// Requests a path replan with a random delay to avoid synchronization issues.
        /// Called by CollisionRecoveryManager when multiple collision threshold is reached.
        /// </summary>
        /// <param name="reason">The reason for requesting replan (for logging)</param>
        public void RequestReplan(string reason)
        {
            if (isReplanning) return;
            
            Debug.Log($"Robot {Id}: Replan requested, reason: {reason}");
            
            StartCoroutine(DelayedReplan(reason));
        }

        private IEnumerator DelayedReplan(string reason)
        {
            float randomDelay = UnityEngine.Random.Range(0.2f, 0.8f);
            yield return new WaitForSeconds(randomDelay);
            
            ClearPathReservations();
            currentPath.Clear();
            currentMovement.Clear();
            currentPathIndex = 0;
            
            needsPathRecovery = false;
            
            Debug.Log($"Robot {Id}: Executing delayed replan after {randomDelay:F2}s, reason: {reason}");
            
            if (currentOrder != null)
            {
                SetBehavior(RobotBehavior.Moving);
                navigationEnabled = true;
                StartCoroutine(ReplanPath());
            }
            else
            {
                SetBehavior(RobotBehavior.Idle);
            }
        }
        #endregion

        #region Collision Detection
        private void CheckSurroundings()
        {
            if (Time.time - lastCollisionClearTime > collisionClearInterval)
            {
                recentCollisions.Clear();
                lastCollisionClearTime = Time.time;
            }

            LayerMask dynamicObstacleLayer = LayerMask.GetMask("RobotLayer", "PickerLayer");
            Collider[] hitColliders = Physics.OverlapSphere(transform.position, detectionRadius, dynamicObstacleLayer);
            bool emergencySituation = false;
            bool shouldSlowDown = false;
            float closestObstacleDistance = float.MaxValue;
            
            foreach (var hitCollider in hitColliders)
            {
                if (hitCollider.gameObject == gameObject)
                    continue;
                        
                float distance = Vector3.Distance(transform.position, hitCollider.transform.position);
                closestObstacleDistance = Mathf.Min(closestObstacleDistance, distance);
                
                // Check if obstacle is in front of us (not behind)
                Vector3 toObstacle = (hitCollider.transform.position - transform.position).normalized;
                toObstacle.y = 0;
                float dotProduct = Vector3.Dot(transform.forward, toObstacle);
                
                // Only react to obstacles in front or to the side (dot > -0.3)
                // Ignore obstacles directly behind us (dot < -0.3)
                if (dotProduct < -0.3f)
                {
                    continue; // Obstacle is behind us, ignore
                }

                bool inFrontCone = dotProduct >= 0.7071f;
                        
                float currentEmergencyStopDistance = emergencyStopDistance;
                bool isRobotObstacle = false;
                bool otherRobotPicking = false;
                bool otherRobotInQueue = false;
                bool otherRobotSamePickingPoint = false;
                Vector3 otherForward = hitCollider.transform.forward;

                if (CentralController.Instance != null && CentralController.Instance.TryGetRobotId(hitCollider.gameObject, out int otherRobotId))
                {
                    isRobotObstacle = true;
                    otherRobotPicking = CentralController.Instance.IsRobotPicking(otherRobotId);
                    otherRobotInQueue = currentOrder?.PickingPoint != null &&
                        CentralController.Instance.IsRobotInPickingQueue(currentOrder.PickingPoint.Id, otherRobotId);
                    otherRobotSamePickingPoint = currentOrder?.PickingPoint != null &&
                        CentralController.Instance.GetRobotPickingPointId(otherRobotId) == currentOrder.PickingPoint.Id;
                    currentEmergencyStopDistance = emergencyStopDistance;
                }
                else
                {
                    currentEmergencyStopDistance = emergencyStopDistance * 1.5f;
                }

                if (distance < currentEmergencyStopDistance)
                {
                    bool shouldQueueBehind = isRobotObstacle && otherRobotSamePickingPoint && (otherRobotPicking || otherRobotInQueue);

                    if (isRobotObstacle && otherRobotPicking && inFrontCone)
                    {
                        if (shouldQueueBehind && !IsPickingPending && !IsAdvancingInQueue)
                        {
                            emergencySituation = true;
                            if (!IsQueuing)
                            {
                                SetBehavior(RobotBehavior.Queuing);
                                lastEmergencyStopTime = Time.time;
                                if (rb != null)
                                {
                                    rb.velocity = Vector3.zero;
                                    rb.angularVelocity = Vector3.zero;
                                }
                                
                                if (currentOrder?.PickingPoint != null)
                                {
                                    PickingQueueManager.Instance.RegisterArrival(currentOrder.PickingPoint.Id, Id.ToString());
                                }
                                
                                Debug.Log($"Robot {Id}: Queuing behind picking robot at same picking point");
                            }
                        }
                        else if (IsAdvancingInQueue && otherRobotSamePickingPoint)
                        {
                            emergencySituation = true;
                            SetBehavior(RobotBehavior.Queuing);
                            if (rb != null)
                            {
                                rb.velocity = Vector3.zero;
                                rb.angularVelocity = Vector3.zero;
                            }
                            Debug.Log($"Robot {Id}: AdvancingInQueue stopped behind picking robot");
                        }
                        else if (IsPickingPending && otherRobotSamePickingPoint)
                        {
                            emergencySituation = true;
                            if (rb != null)
                            {
                                rb.velocity = Vector3.zero;
                                rb.angularVelocity = Vector3.zero;
                            }
                            Debug.Log($"Robot {Id}: PickingPending waiting - front robot still picking (unexpected)");
                        }
                        else if (!otherRobotSamePickingPoint)
                        {
                            emergencySituation = true;
                            if (!IsEmergencyStopped && !IsQueuing && !IsPickingPending && !IsAdvancingInQueue)
                            {
                                SetBehavior(RobotBehavior.EmergencyStopped);
                                lastEmergencyStopTime = Time.time;
                                if (rb != null)
                                {
                                    rb.velocity = Vector3.zero;
                                    rb.angularVelocity = Vector3.zero;
                                }
                                Debug.Log($"Robot {Id}: Emergency stop - front robot is picking (different picking point)");
                            }
                        }
                        
                        if (!recentCollisions.Contains(hitCollider.gameObject))
                        {
                            recentCollisions.Add(hitCollider.gameObject);
                        }
                        break;
                    }

                    if (isRobotObstacle)
                    {
                        otherForward.y = 0;
                        Vector3 myForward = transform.forward;
                        myForward.y = 0;
                        float headingDot = Vector3.Dot(myForward.normalized, otherForward.normalized);
                        bool isSameDirection = headingDot >= 0.7071f;
                        bool isOppositeDirection = headingDot <= -0.7071f;

                        if (inFrontCone && (isSameDirection || isOppositeDirection))
                        {
                            shouldSlowDown = true;
                            if (!recentCollisions.Contains(hitCollider.gameObject))
                            {
                                recentCollisions.Add(hitCollider.gameObject);
                            }
                            continue;
                        }
                    }

                    if (shouldQueueBehind && !IsPickingPending && !IsAdvancingInQueue)
                    {
                        emergencySituation = true;
                        if (!IsQueuing)
                        {
                            SetBehavior(RobotBehavior.Queuing);
                            lastEmergencyStopTime = Time.time;
                            if (rb != null)
                            {
                                rb.velocity = Vector3.zero;
                                rb.angularVelocity = Vector3.zero;
                            }
                        }
                    }
                    else if (isRobotObstacle && otherRobotInQueue && IsAdvancingInQueue)
                    {
                        emergencySituation = true;
                        SetBehavior(RobotBehavior.Queuing);
                        if (rb != null)
                        {
                            rb.velocity = Vector3.zero;
                            rb.angularVelocity = Vector3.zero;
                        }
                        Debug.Log($"Robot {Id}: AdvancingInQueue stopped behind queuing robot");
                    }
                    else if (isRobotObstacle && otherRobotInQueue && !IsPickingPending && !IsAdvancingInQueue)
                    {
                        emergencySituation = true;
                        if (!IsEmergencyStopped)
                        {
                            SetBehavior(RobotBehavior.EmergencyStopped);
                            lastEmergencyStopTime = Time.time;
                            if (rb != null)
                            {
                                rb.velocity = Vector3.zero;
                                rb.angularVelocity = Vector3.zero;
                            }
                        }
                    }
                    else if (!isRobotObstacle)
                    {
                        emergencySituation = true;
                        TriggerEmergencyBackup(hitCollider.transform.position);
                    }
                    
                    if (!recentCollisions.Contains(hitCollider.gameObject))
                    {
                        recentCollisions.Add(hitCollider.gameObject);
                    }
                    
                    if (emergencySituation)
                    {
                        break;
                    }
                }
            }

            if (shouldSlowDown && rb != null)
            {
                rb.velocity *= 0.7f;
            }
            
            if (IsEmergencyStopped && !emergencySituation && !isBackingUp)
            {
                float safeResumeDistance = emergencyStopDistance * 1.5f;
                bool canResume = hitColliders.Length == 0 || closestObstacleDistance > safeResumeDistance;
                
                if (canResume)
                {
                    ResumeFromEmergencyStop();
                }
            }
            
            if (IsQueuing && !emergencySituation && !isBackingUp && currentOrder?.PickingPoint != null)
            {
                bool frontRobotStillBlocking = false;
                float frontRobotDistance = float.MaxValue;
                
                foreach (var hitCollider in hitColliders)
                {
                    if (hitCollider.gameObject == gameObject) continue;
                    
                    if (CentralController.Instance != null && CentralController.Instance.TryGetRobotId(hitCollider.gameObject, out int otherRobotId))
                    {
                        string otherPickingPointId = CentralController.Instance.GetRobotPickingPointId(otherRobotId);
                        bool samePickingPoint = otherPickingPointId == currentOrder.PickingPoint.Id;
                        bool otherIsPicking = CentralController.Instance.IsRobotPicking(otherRobotId);
                        bool otherInQueue = CentralController.Instance.IsRobotInPickingQueue(currentOrder.PickingPoint.Id, otherRobotId);
                        
                        Vector3 toOther = (hitCollider.transform.position - transform.position).normalized;
                        toOther.y = 0;
                        float dot = Vector3.Dot(transform.forward, toOther);
                        bool inFront = dot >= 0.3f;
                        
                        if (inFront && samePickingPoint && (otherIsPicking || otherInQueue))
                        {
                            float distance = Vector3.Distance(transform.position, hitCollider.transform.position);
                            frontRobotStillBlocking = true;
                            frontRobotDistance = Mathf.Min(frontRobotDistance, distance);
                        }
                    }
                }
                
                if (!frontRobotStillBlocking)
                {
                    Debug.Log($"Robot {Id}: Queue front cleared, advancing in queue");
                    SetBehavior(RobotBehavior.AdvancingInQueue);
                }
                else if (frontRobotDistance > emergencyStopDistance * 2.0f)
                {
                    Debug.Log($"Robot {Id}: Front robot moved away (distance: {frontRobotDistance:F2}), advancing in queue");
                    SetBehavior(RobotBehavior.AdvancingInQueue);
                }
            }
            
            if (IsAdvancingInQueue && currentOrder?.PickingPoint != null)
            {
                bool shouldStopAdvancing = false;
                
                foreach (var hitCollider in hitColliders)
                {
                    if (hitCollider.gameObject == gameObject) continue;
                    
                    float distance = Vector3.Distance(transform.position, hitCollider.transform.position);
                    if (distance >= emergencyStopDistance) continue;
                    
                    if (CentralController.Instance != null && CentralController.Instance.TryGetRobotId(hitCollider.gameObject, out int otherRobotId))
                    {
                        string otherPickingPointId = CentralController.Instance.GetRobotPickingPointId(otherRobotId);
                        bool samePickingPoint = otherPickingPointId == currentOrder.PickingPoint.Id;
                        bool otherIsPicking = CentralController.Instance.IsRobotPicking(otherRobotId);
                        bool otherInQueue = CentralController.Instance.IsRobotInPickingQueue(currentOrder.PickingPoint.Id, otherRobotId);
                        
                        Vector3 toOther = (hitCollider.transform.position - transform.position).normalized;
                        toOther.y = 0;
                        float dot = Vector3.Dot(transform.forward, toOther);
                        bool inFront = dot >= 0.5f;
                        
                        if (inFront && samePickingPoint && (otherIsPicking || otherInQueue))
                        {
                            shouldStopAdvancing = true;
                            break;
                        }
                    }
                }
                
                if (shouldStopAdvancing)
                {
                    Debug.Log($"Robot {Id}: Reached front robot, stopping in queue");
                    SetBehavior(RobotBehavior.Queuing);
                    if (rb != null)
                    {
                        rb.velocity = Vector3.zero;
                        rb.angularVelocity = Vector3.zero;
                    }
                }
            }
        }

        private void TriggerEmergencyBackup(Vector3 obstaclePosition)
        {
            if (IsEmergencyStopped || isBackingUp) return;
            
            SetBehavior(RobotBehavior.EmergencyStopped);
            lastEmergencyStopTime = Time.time;
            
            if (rb != null)
            {
                rb.velocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
            }
            currentSpeed = 0f;
            currentAcceleration = 0f;
            
            float randomDelay = UnityEngine.Random.Range(EMERGENCY_RECOVERY_DELAY, EMERGENCY_RECOVERY_DELAY + 0.3f);
            
            Debug.Log($"Robot {Id}: Emergency stop, will backup after {randomDelay:F2}s");
            
            StartCoroutine(EmergencyBackupCoroutine(obstaclePosition, randomDelay));
        }
        
        private IEnumerator EmergencyBackupCoroutine(Vector3 obstaclePosition, float delay)
        {
            yield return new WaitForSeconds(delay);
            
            if (!IsEmergencyStopped || isBackingUp)
                yield break;
            
            Vector3 backupTarget = CalculateSafeBackupTarget(obstaclePosition);
            
            if (Vector3.Distance(backupTarget, transform.position) < 0.1f)
            {
                Debug.Log($"Robot {Id}: No safe backup position during emergency, waiting and replanning");
                yield return new WaitForSeconds(UnityEngine.Random.Range(0.5f, 1.0f));
                
                if (currentOrder != null)
                {
                    SetBehavior(RobotBehavior.Moving);
                    navigationEnabled = true;
                    StartCoroutine(ReplanPath());
                }
                else
                {
                    SetBehavior(RobotBehavior.Idle);
                }
                yield break;
            }
            
            float randomWait = UnityEngine.Random.Range(0.3f, 0.8f);
            
            Debug.Log($"Robot {Id}: Executing safe emergency backup to {backupTarget}");
            
            StartBackupAndWait(backupTarget, randomWait);
        }

        private void ResumeFromEmergencyStop()
        {
            if (IsEmergencyStopped)
            {
                Debug.Log($"Robot {Id}: Resuming from emergency stop at time {Time.time}");
                
                StartCoroutine(ResumeMovementWithDelay());
            }
        }

        private IEnumerator ResumeMovementWithDelay()
        {
            yield return new WaitForSeconds(0.1f);
            
            if (currentOrder != null && finalDestination != Vector3.zero)
            {
                Debug.Log($"Robot {Id}: Resuming movement to destination {finalDestination}");
                
                if (currentPath.Count > 0 && currentPathIndex < currentPath.Count)
                {
                    // If hasReportedArrival is true and we're resuming, we should be in PickingPending state
                    // (we were allowed to pick from queue and are moving to picking point)
                    if (hasReportedArrival && currentOrder.PickingPoint != null)
                    {
                        Debug.Log($"Robot {Id}: Resuming as PickingPending (was allowed to pick from queue)");
                        SetBehavior(RobotBehavior.PickingPending);
                    }
                    else
                    {
                        Debug.Log($"Robot {Id}: Resuming existing path from point {currentPathIndex} of {currentPath.Count}");
                        SetBehavior(RobotBehavior.Moving);
                    }
                }
                else
                {
                    Debug.Log($"Robot {Id}: Path invalid after emergency stop, replanning");
                    StartCoroutine(ReplanPath());
                }
            }
            else
            {
                Debug.Log($"Robot {Id}: No current order to resume after emergency stop");
            }
        }

        private bool IsCurrentPathValid()
        {
            if (currentPath == null || currentPath.Count == 0 || currentPathIndex >= currentPath.Count)
            {
                return false;
            }
            
            ConnectionPoint currentPoint = currentPath[currentPathIndex];
            Vector3 targetPos = GetConnectionPointPosition(currentPoint);
            
            float distanceToTarget = Vector3.Distance(transform.position, targetPos);
            
            return distanceToTarget < pathPointAcceptanceRadius * 3.0f;
        }

        private void OnCollisionEnter(Collision collision)
        {
            if (collision.gameObject.layer == LayerMask.NameToLayer("GroundLayer"))
                return;
            
            // Queue state robots should not trigger backup recovery - they handle collisions via CheckSurroundings
            if (IsQueuing || IsAdvancingInQueue || IsPickingPending)
            {
                return;
            }
                    
            GameObject collisionObject = collision.gameObject;
            string collisionObjectName = collisionObject.name;
            
            Debug.LogWarning($"Robot {Id}: Detected collision with {collisionObjectName} at time {Time.time}");
            
            if (CentralController.Instance != null && CentralController.Instance.TryGetRobotId(collisionObject, out int otherRobotId))
            {
                bool otherIsPicking = CentralController.Instance.IsRobotPicking(otherRobotId);
                
                if (IsPicking && otherIsPicking)
                {
                    EventManager.Instance?.ReportRobotCollision(
                        Id.ToString(),
                        otherRobotId.ToString(),
                        collisionObject,
                        Time.time,
                        transform.position
                    );
                    return;
                }
                
                if (IsPicking && !otherIsPicking)
                {
                    EventManager.Instance?.ReportRobotCollision(
                        Id.ToString(),
                        otherRobotId.ToString(),
                        collisionObject,
                        Time.time,
                        transform.position
                    );
                    return;
                }
                
                if (!IsPicking && otherIsPicking)
                {
                    EventManager.Instance?.ReportRobotCollision(
                        Id.ToString(),
                        otherRobotId.ToString(),
                        collisionObject,
                        Time.time,
                        transform.position
                    );
                    
                    if (collisionRecoveryManager != null)
                    {
                        collisionRecoveryManager.HandleNonPickingCollision(otherRobotId, transform.position);
                    }
                    else
                    {
                        StartCoroutine(HandlingRobotCollision(otherRobotId, collisionObject.transform.position));
                    }
                    return;
                }

                if (collisionRecoveryManager != null)
                {
                    collisionRecoveryManager.HandleRobotCollision(otherRobotId, transform.position);
                }
                else
                {
                    StartCoroutine(HandlingRobotCollision(otherRobotId, collisionObject.transform.position));
                }
            }
            else
            {
                if (collisionRecoveryManager != null)
                {
                    collisionRecoveryManager.HandleEnvironmentCollision(collisionObject.transform.position);
                }
                else
                {
                    StartCoroutine(HandlingEnvironmentCollision(collisionObject));
                }
            }
            
            if (!recentCollisions.Contains(collisionObject))
            {
                recentCollisions.Add(collisionObject);
            }
            
            if (!IsEmergencyStopped && !isBackingUp)
            {
                SetBehavior(RobotBehavior.EmergencyStopped);
                lastEmergencyStopTime = Time.time;
                if (rb != null)
                {
                    rb.velocity = Vector3.zero;
                    rb.angularVelocity = Vector3.zero;
                }
                currentSpeed = 0f;
                currentAcceleration = 0f;
            }
        }

        private IEnumerator HandlingRobotCollision(int otherRobotId, Vector3 otherRobotPosition)
        {
            navigationEnabled = false;
            
            Vector3 collisionDirection = (otherRobotPosition - transform.position).normalized;
            float dotProduct = Vector3.Dot(transform.forward, collisionDirection);
            string collisionType = dotProduct < -0.3f ? "head-on" : 
                                   dotProduct > 0.7f ? "tailgating" : 
                                   dotProduct < -0.7f ? "being tailgated" : "side";
            
            Debug.Log($"Robot {Id}: Collision with Robot {otherRobotId}, type: {collisionType}");
            
            yield return new WaitForSeconds(0.5f);
            
            float distance = Vector3.Distance(transform.position, otherRobotPosition);
            
            if (distance < emergencyStopDistance)
            {
                Vector3 backupTarget = CalculateSafeBackupTarget(otherRobotPosition);
                
                if (Vector3.Distance(backupTarget, transform.position) >= 0.1f)
                {
                    float randomWait = UnityEngine.Random.Range(0.3f, 0.6f);
                    StartBackupAndWait(backupTarget, randomWait);
                    yield break;
                }
            }
            
            navigationEnabled = true;
            
            ClearPathReservations();
            currentPath.Clear();
            currentMovement.Clear();
            currentPathIndex = 0;
            
            float replanDelay = UnityEngine.Random.Range(0.1f, 0.5f);
            yield return new WaitForSeconds(replanDelay);
            
            if (currentOrder != null)
            {
                StartCoroutine(ReplanPath());
            }
            else
            {
                SetBehavior(RobotBehavior.Idle);
            }
        }

        private IEnumerator HandlingEnvironmentCollision(GameObject obstacle)
        {
            navigationEnabled = false;
            
            yield return new WaitForSeconds(0.3f);
            
            Vector3 backupTarget = CalculateSafeBackupTarget(obstacle.transform.position);
            
            if (Vector3.Distance(backupTarget, transform.position) >= 0.1f)
            {
                float randomWait = UnityEngine.Random.Range(0.3f, 0.6f);
                StartBackupAndWait(backupTarget, randomWait);
                yield break;
            }
            
            navigationEnabled = true;
            
            yield return new WaitForSeconds(0.3f);
            
            if (currentOrder != null)
            {
                StartCoroutine(ReplanPath());
            }
            else
            {
                SetBehavior(RobotBehavior.Idle);
            }
        }
        #endregion

        #region Conflict Resolution
        private void ApplyTimeShiftModification(List<ConnectionPoint> path, List<float> newTimes, string planId, float shift)
        {
            // Update path and times
            currentPath = path;
            pathTimes = newTimes;
            currentPlanId = planId;
            
            Debug.Log($"Robot {Id}: Applying time shift modification: {shift}s, new path length: {path.Count}");
            
            // Save path to personal layer
            SavePathToPersonalLayer(path, newTimes, planId);
            
            // Adjust waiting time
            float newStartTime = newTimes[0];
            float currentTime = Time.time;
            float waitTime = Math.Max(0, newStartTime - currentTime);
            
            // If there's an existing waiting coroutine, stop it
            if (waitForExecutionCoroutine != null)
            {
                StopCoroutine(waitForExecutionCoroutine);
            }
            
            // Execute new waiting time
            waitForExecutionCoroutine = StartCoroutine(WaitForExecution(waitTime));
        }

        private void ApplySpeedAdjustmentModification(
            List<ConnectionPoint> path, 
            List<float> newTimes, 
            List<float> newSpeeds, 
            string planId, 
            float avgSpeedChange)
        {
            // Update path and times
            currentPath = path;
            pathTimes = newTimes;
            currentPlanId = planId;
            
            Debug.Log($"Robot {Id}: Applying speed adjustment modification: average speed change {avgSpeedChange:F2}x, new path length: {path.Count}");
            
            // Update path segments with new speeds
            UpdatePathSegmentsWithSpeed(path, newTimes, newSpeeds);
            
            // Save path to personal layer
            SavePathToPersonalLayer(path, newTimes, planId);
            
            // Adjust waiting time
            float newStartTime = newTimes[0];
            float currentTime = Time.time;
            float waitTime = Math.Max(0, newStartTime - currentTime);
            
            // If there's an existing waiting coroutine, stop it
            if (waitForExecutionCoroutine != null)
            {
                StopCoroutine(waitForExecutionCoroutine);
            }
            
            // Execute new waiting time
            waitForExecutionCoroutine = StartCoroutine(WaitForExecution(waitTime));
            
            // Notify state change
            SetBehavior(RobotBehavior.Moving);
        }

        private void UpdatePathSegmentsWithSpeed(List<ConnectionPoint> path, List<float> times, List<float> speeds)
        {
            // Clear current movement segments
            currentMovement.Clear();
            
            // Ensure valid input
            if (path == null || times == null || speeds == null || 
                path.Count != times.Count || path.Count != speeds.Count)
            {
                Debug.LogError($"Robot {Id}: Failed to update path segments - inconsistent data");
                return;
            }
            
            // Create new path segments
            for (int i = 0; i < path.Count; i++)
            {
                currentMovement.Add(new PathSegment(path[i], times[i], speeds[i]));
            }
        }

        private IEnumerator WaitForExecution(float waitTime)
        {
            Debug.Log($"Robot {Id}: Waiting {waitTime}s before executing path");
            
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
            
            Debug.Log($"Robot {Id}: Starting path execution, path length: {currentPath.Count}");
            
            SetBehavior(RobotBehavior.Moving);
            
            // Reset path tracking index
            currentPathIndex = 0;
        }
        #endregion

        #region Events Management
        private void HandlePathModified(object sender, RobotPathModificationEventArgs e)
        {
            // Check if this is for the current robot
            if (e.RobotId != Id.ToString())
                return;
            
            Debug.Log($"Robot {Id}: Received path modification: type {e.ModificationType}, value {e.ModificationValue}");
            
            // Handle based on modification type
            switch (e.ModificationType)
            {
                case "time_shift":
                    ApplyTimeShiftModification(e.Path, e.NewTimes, e.PlanId, e.ModificationValue);
                    break;
                case "speed_adjustment":
                    ApplySpeedAdjustmentModification(e.Path, e.NewTimes, e.NewSpeeds, e.PlanId, e.ModificationValue);
                    break;
                default:
                    Debug.LogWarning($"Robot {Id}: Unknown path modification type: {e.ModificationType}");
                    break;
            }
        }

        private void HandlePathAccepted(object sender, RobotPathAcceptedEventArgs e)
        {
            // Check if this is for the current robot
            if (e.RobotId != Id.ToString())
                return;
            
            Debug.Log($"Robot {Id}: Path has been accepted by central controller with {e.Path.Count} points");
            
            // Update path and times
            currentPath = new List<ConnectionPoint>(e.Path);
            pathTimes = new List<float>(e.Times);
            currentPlanId = e.PlanId;
            
            // Update current movement information
            UpdateCurrentMovementInfo(e.Path, e.Times, e.Speeds);

            // Execute response logic such as starting path execution
            BeginPathExecution();
        }

        private void HandlePickingCompletedEvent(object sender, PickingEventArgs e)
        {
            // Confirm event is for this robot
            if (e.RobotId != Id.ToString())
                return;
            
            Debug.Log($"Robot {Id}: Received picking completed event, PickingPoint: {e.PickingPoint.Id}");
            
            // Confirm it's for the current order's picking point
            if (currentOrder != null && currentOrder.PickingPoint.Id == e.PickingPoint.Id)
            {
                NotifyPickingCompleted();
            }
            else
            {
                Debug.LogWarning($"Robot {Id}: Received picking completed event, but no matching order or picking point");
            }
        }

        private void HandleGlobalOccupancyLayerUpdated(object sender, GlobalOccupancyLayerEventArgs e)
        {
            UpdateGlobalLayer(e.GlobalLayer);
            
            if (isReplanning)
            {
                Debug.Log($"Robot {Id}: Received updated global occupancy layer, triggering path replanning");
                StartCoroutine(ReplanPath());
            }
        }
        #endregion

        #region Interaction with Picker
        private void CreateInteractionTrigger()
        {
            // Check if interaction trigger already exists
            Transform existingTrigger = transform.Find("InteractionTrigger");
            if (existingTrigger != null)
            {
                return; // Already exists, no need to create again
            }
            
            // Create trigger object
            GameObject triggerObj = new GameObject("InteractionTrigger");
            triggerObj.transform.SetParent(transform);
            triggerObj.transform.localPosition = Vector3.zero;
            
            // Add spherical trigger, larger than NavMeshObstacle
            SphereCollider interactionTrigger = triggerObj.AddComponent<SphereCollider>();
            interactionTrigger.radius = 2.0f; // Large range for picker detection
            interactionTrigger.center = new Vector3(0, 0.5f, 0);
            interactionTrigger.isTrigger = true; // Set as trigger
            
            // Add interaction handler component
            RobotInteractionZone interactionZone = triggerObj.AddComponent<RobotInteractionZone>();
            interactionZone.robotController = this;
            
            Debug.Log($"Robot {Id}: Created interaction trigger zone");
        }

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
                // Get current speed
                float currentSpeed = 0;
                if (rb != null)
                {
                    currentSpeed = rb.velocity.magnitude;
                }
                
                // When robot is moving quickly, reduce obstacle size
                if (currentSpeed > 0.5f)
                {
                    obstacle.size = movingSize;
                }
                else
                {
                    // When stationary or slow, use normal size
                    obstacle.size = normalSize;
                }
                
                yield return new WaitForSeconds(0.5f);
            }
        }
        #endregion

        #region Visualization
        private void OnDrawGizmos()
        {
            if (!Application.isPlaying) return;
            
            // Visualize velocity
            if (rb != null)
            {
                Gizmos.color = Color.blue;
                Gizmos.DrawLine(transform.position, transform.position + rb.velocity * 0.5f);
            }
            
            // Visualize path
            if (currentPath != null && currentPath.Count > 0)
            {
                Gizmos.color = Color.cyan;
                
                // Draw the entire path
                for (int i = 0; i < currentPath.Count - 1; i++)
                {
                    Vector3 start = GetConnectionPointPosition(currentPath[i]);
                    Vector3 end = GetConnectionPointPosition(currentPath[i + 1]);
                    
                    Gizmos.DrawLine(start, end);
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
        private Vector3 GetPointVector3(Point point)
        {
            if (point == null) return Vector3.zero;
            
            return new Vector3(
                (float)point.X, 
                transform.position.y, 
                (float)point.Y
            );
        }

        private Vector3 GetConnectionPointPosition(ConnectionPoint cp)
        {
            if (cp == null || cp.Point == null)
                return transform.position;
            
            float x = (float)cp.Point.Coordinate.X;
            float z = (float)cp.Point.Coordinate.Y;
            float y = DetermineConnectionPointHeight(cp);
            
            return new Vector3(x, y, z);
        }

        private float DetermineConnectionPointHeight(ConnectionPoint cp)
        {
            if (cp == null)
            {
                return transform.position.y;
            }

            string floorLevel = cp.FloorLevel;
            return GetFloorHeight(floorLevel);
        }
        
        private int ParseFloorLevel(string floorLevel)
        {
            if (string.IsNullOrEmpty(floorLevel)) 
                return 1;
            
            if (floorLevel.StartsWith("F") || floorLevel.EndsWith("F"))
            {
                string numPart = floorLevel.Replace("F", "");
                if (int.TryParse(numPart, out int level))
                {
                    return level;
                }
            }
            
            if (floorLevel.StartsWith("B") || floorLevel.StartsWith("b"))
            {
                string numPart = floorLevel.Substring(1).Replace("F", "").Replace("f", "");
                if (int.TryParse(numPart, out int level))
                {
                    return -level;
                }
            }

            if (int.TryParse(floorLevel, out int simpleLevel))
            {
                return simpleLevel;
            }
            
            return 1;
        }
        
        private float GetFloorHeight(string floorLevel)
        {
            int level = ParseFloorLevel(floorLevel);
            if (level > 0)
            {
                return ((level - 1) * floorHeight) + baseHeight;
            }
            return (level * floorHeight) + baseHeight;
        }

        private string GetFloorLevelFromPosition(Vector3 position)
        {
            if (floorHeight <= 0) floorHeight = 3.0f;
            
            int floorLevel = Mathf.RoundToInt((position.y - baseHeight) / floorHeight) + 1;
            
            if (floorLevel <= 0)
            {
                return $"B{Math.Abs(floorLevel)}F";
            }
            else
            {
                return $"{floorLevel}F";
            }
        }
        
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
