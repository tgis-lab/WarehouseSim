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
        Picking,
        UsingElevator, // New state for elevator movement
        UsingStairs    // New state for stair movement
    }
    
    public class RobotController : MonoBehaviour
    {
        #region Inspector Fields
        public Transform targetIndicator;
        public SearchAlgorithm selectedAlgorithm = SearchAlgorithm.AnytimeSIPP;
        public SearchMethod selectedSearchMethod = SearchMethod.Manhattan_Distance;
        public float baseDetectionRadius = 0.7f;
        public float emergencyStopDistance = 0.2f;
        public float defaultSpeed = 2.0f;
        public float maxAcceleration = 2.0f;
        public float maxDeceleration = 5.0f;
        public int MaxOrder = 20;
        public float verticalMovementSpeed = 1.0f;
        public float floorHeight = 3.0f;
        public float elevatorBaseWaitTime = 3.0f;
        public float elevatorFloorTime = 1.0f;
        public float stairsFloorTime = 2.0f;
        #endregion

        #region Public Properties
        public int Id { get; private set; }
        public Graph graph { get; private set; }
        public IndoorSpace indoorSpace { get; private set; }
        public MapGrid mapGrid { get; private set; }
        public bool lastReportedAvailability { get; set; } = false;
        public bool lastReportedFreeStatus { get; set; } = false;
        public int lastReportedOrderCount { get; set; } = -1;
        public string CurrentFloor { get; private set; } = "1F"; // Current floor level
        public bool IsMoving => currentStates.Contains(RobotState.Moving);
        public bool IsPicking => currentStates.Contains(RobotState.Picking);
        public bool IsUsingElevator => currentStates.Contains(RobotState.UsingElevator);
        public bool IsUsingStairs => currentStates.Contains(RobotState.UsingStairs);
        public bool IsVerticallyMoving => IsUsingElevator || IsUsingStairs;
        public bool IsFree => currentStates.Count == 0;
        public bool IsOnTask => !IsFree;
        public bool IsAvailable => ordersQueue.Count < MaxOrder;
        public bool IsProcessingOrders => isProcessingOrdersRunning;
        public Order CurrentOrder => currentOrder;
        public int GetRobotOrdersQueueCount => ordersQueue.Count;
        public List<string> listOrdersQueue => new List<string>(ordersQueue.Select(order => order.Id));
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
        private bool isEmergencyStopped = false;
        private float lastEmergencyStopTime = 0f;
        private const float EMERGENCY_COOLDOWN = 3.0f;
        private LayerMask robotCollisionLayer;
        private LayerMask pickerCollisionLayer;
        private CellSpace lastCellSpace = null;
        private CellSpace firstCellSpace = null;
        private Dictionary<string, float> cellSpaceEntryTimes = new Dictionary<string, float>();
        
        // Floor transition related fields
        private float baseHeight = 0.05f;
        
        // Path recovery
        private bool needsPathRecovery = false;
        private float lastPathRecoveryTime = 0f;
        private const int MAX_RECOVERY_ATTEMPTS = 3;
        private const float RECOVERY_INTERVAL = 2.0f;
        
        // Conflict resolution
        private Coroutine waitForExecutionCoroutine;
        private List<float> pathTimes = new List<float>();
        
        // Collision
        private List<GameObject> recentCollisions = new List<GameObject>();
        private float collisionClearInterval = 5.0f;
        private float lastCollisionClearTime = 0f;
        private float pathPointAcceptanceRadius = 0.8f;
        private float finalDestinationAcceptanceRadius = 0.3f;
        private bool hasReportedArrival = false;
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

            // Update personal occupancy layer
            UpdatePersonalLayer();
            
            // Update robot status
            UpdateRobotStatus();
            
            // Collision Detection
            CheckSurroundings();

            // Update movement info
            lastPosition = transform.position;
            lastPositionChangeTime = Time.time;
            
            // Handle different movement states
            if (navigationEnabled && !isEmergencyStopped && !isWaitingForPicking && IsMoving)
            {
                UpdateMovement();
            }
            
            // Update visualization
            UpdateTargetIndicator();
            
            if (isEmergencyStopped || needsPathRecovery)
            {
                VisualizeEmergencyState();
            }
            
            // Update intersection status
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

        private void OnDestroy()
        {
            if (EventManager.Instance != null)
            {
                EventManager.Instance.OnRobotPathModified -= HandlePathModified;
                EventManager.Instance.OnRobotPathAccepted -= HandlePathAccepted;
                EventManager.Instance.OnPickingCompleted -= HandlePickingCompletedEvent;
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

            // Determine current floor level from position
            CurrentFloor = GetFloorLevelFromPosition(transform.position);
            Debug.Log($"Robot {Id} initialized at {transform.position} on floor {CurrentFloor}");

            ConfigureNavMeshObstacle();
            CreateInteractionTrigger();
            StartCoroutine(ManageObstacleCarving());

            personalOccupancyLayer = new DynamicOccupancyLayer(indoorSpace, 300f, Time.time);
            
            pathPlanner = new PathPlanner(graph, indoorSpace, selectedAlgorithm, selectedSearchMethod, 
                                          id.ToString(), maxAcceleration, maxDeceleration);

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
            // Plan path to destination
            Debug.Log($"Robot {Id} planning path to {order.Destination}");
            SetState(RobotState.Moving, true);
            
            Vector3 targetPosition = GetTargetPosition(order.PickingPoint);
            string targetFloorLevel = order.PickingPoint.FloorLevel;
            targetPosition.y = GetFloorHeight(targetFloorLevel);
            
            Debug.Log($"Robot {Id} planning path from floor {CurrentFloor} to floor {targetFloorLevel}");
            
            var result = PlanPath(targetPosition, targetFloorLevel);
            if (result.path == null || result.path.Count == 0)
            {
                Debug.LogError($"Robot {Id} failed to plan path to {order.Destination}");
                SetState(RobotState.Moving, false);
                yield break;
            }

            currentPlanId = result.planId;
            RaisePathPlannedEvent(currentPlanId, result.path, result.times, result.speeds);
            personalOccupancyLayer.RecordPathOccupancy(result.path, result.times, this.Id.ToString(), currentPlanId);
            UpdateCurrentMovementInfo(result.path, result.times, result.speeds);
            
            // Wait until destination reached or timeout
            float timeout = 300f;  // 5 minutes timeout
            float startTime = Time.time;
            
            while (!hasReportedArrival && Time.time - startTime < timeout)
            {
                // Check if we're still moving or in vertical transition
                if (!IsMoving && !IsVerticallyMoving)
                {
                    Debug.LogWarning($"Robot {Id} stopped moving before reaching destination");
                    break;
                }
                
                yield return null;
            }
            
            if (!hasReportedArrival)
            {
                Debug.LogWarning($"Robot {Id} timed out while trying to reach destination");
                SetState(RobotState.Moving, false);
                SetState(RobotState.UsingElevator, false);
                SetState(RobotState.UsingStairs, false);
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

        #region Path Planning and Movement
        private Vector3 GetTargetPosition(PickingPoint pickingPoint)
        {
            CellSpace cellSpace = indoorSpace.GetCellSpaceFromId(pickingPoint.Id);
            Point point = (Point)cellSpace.Node;
            float y = GetFloorHeight(pickingPoint.FloorLevel);
            return new Vector3((float)point.X, y, (float)point.Y);
        }

        private (List<ConnectionPoint> path, List<float> times, List<float> speeds, string planId) PlanPath(Vector3 targetPosition, string targetFloorLevel = null)
        {
            currentPath.Clear();
            currentMovement.Clear();
            finalDestination = targetPosition;
            hasReportedArrival = false;

            RoutePoint startPoint = graph.GetRoutePointFromCoordinate(transform.position, true);
            RoutePoint endPoint = graph.GetRoutePointFromCoordinate(targetPosition, false);
            
            Debug.Log($"Robot {Id}: Planning path from {transform.position} (Floor {CurrentFloor}) to {targetPosition} (Floor {targetFloorLevel})");

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

        private bool HasReachedFinalDestination()
        {
            if (currentOrder == null)
                return false;
                
            CellSpace cellSpace = indoorSpace.GetCellSpaceFromId(currentOrder.PickingPoint.Id);
            if (cellSpace == null) 
                return false;
            
            // Check if in the cell containing the picking point
            CellSpace currentCell = indoorSpace.GetCellSpaceFromCoordinates(transform.position, GetFloorLevelFromPosition(transform.position));
            if (currentCell != null && currentCell.Id == cellSpace.Id)
            {
                // Check distance to the center of the picking point
                float distanceToPickingPoint = Vector3.Distance(
                    transform.position,
                    finalDestination
                );
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
            if (!IsMoving || currentPath == null || currentPath.Count == 0 || currentPathIndex >= currentPath.Count)
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
                        
                        SetState(RobotState.Moving, false);
                        
                        rb.velocity = Vector3.zero;
                    }
                    
                    return;
                }
                
                return;
            }
            
            float moveSpeed = currentPathIndex < currentMovement.Count ? 
                currentMovement[currentPathIndex].Speed : defaultSpeed;
            
            Vector3 direction = (targetPos - transform.position).normalized;
            
            Vector3 targetVelocity = direction * moveSpeed;
            rb.velocity = targetVelocity;
            
            Vector3 horizontalDirection = new Vector3(direction.x, 0, direction.z).normalized;
            if (horizontalDirection.magnitude > 0.1f)
            {
                Quaternion targetRotation = Quaternion.LookRotation(horizontalDirection);
                transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, Time.deltaTime * 5f);
            }
        }

        private void MoveFromConnectionPointToTargetCellSpace(ConnectionPoint targetPoint)
        {
            if (targetPoint == null)
                return;
            
            string targetCellSapceId = targetPoint.CellBoundary.Target;
            CellSpace targetCellSpace = indoorSpace.GetCellSpaceFromId(targetCellSapceId);
            
            Point point = (Point)targetCellSpace.Node;
            float y = GetFloorHeight(targetCellSpace.GetLocatedFloor());
            
            Vector3 targetPos = new Vector3((float)point.X, y, (float)point.Y);
            
            finalDestination = targetPos;
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
            bool previousState = currentStates.Contains(state);
            
            if (value)
            {
                currentStates.Add(state);
            }
            else
            {
                currentStates.Remove(state);
            }
            
            // Log state changes
            if (previousState != value)
            {
                Debug.Log($"Robot {Id}: State {state} changed to {value}");
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
            SetState(RobotState.Moving, false);
            SetState(RobotState.Picking, false);
            SetState(RobotState.UsingElevator, false);
            SetState(RobotState.UsingStairs, false);
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
            
            // Reset state variables
            needsPathRecovery = true;
            lastPathRecoveryTime = Time.time;
            
            // Cancel current movement
            if (IsMoving)
            {
                StopMoving();
                SetState(RobotState.Moving, false);
            }
            
            // Also cancel vertical movement if in progress
            if (IsVerticallyMoving)
            {
                SetState(RobotState.UsingElevator, false);
                SetState(RobotState.UsingStairs, false);
            }
            
            // Brief delay before recovery
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
            Debug.Log($"Robot {Id}: Replanning path to destination");
            
            if (currentOrder == null)
            {
                Debug.LogWarning($"Robot {Id}: No current order to replan path for");
                yield break;
            }
            
            // Save current position and final target
            Vector3 finalTarget = GetTargetPosition(currentOrder.PickingPoint);
            
            // Temporarily disable navigation
            bool wasNavigationEnabled = navigationEnabled;
            navigationEnabled = false;
            
            // Wait for stabilization
            yield return new WaitForSeconds(0.5f);
            
            // Replan path
            var result = PlanPath(finalTarget, currentOrder.PickingPoint.FloorLevel);
            if (result.path != null && result.path.Count > 0)
            {
                currentPlanId = result.planId;
                RaisePathPlannedEvent(currentPlanId, result.path, result.times, result.speeds);
                personalOccupancyLayer.RecordPathOccupancy(result.path, result.times, this.Id.ToString(), currentPlanId);
                UpdateCurrentMovementInfo(result.path, result.times, result.speeds);
                currentPathIndex = 0;
                
                Debug.Log($"Robot {Id}: Replanning path succeeded.");
                
                // Resume movement
                SetState(RobotState.Moving, true);
            }
            else
            {
                Debug.LogError($"Robot {Id}: Replanning path failed.");
            }
            
            // Re-enable navigation
            navigationEnabled = wasNavigationEnabled;
        }
        #endregion

        #region Occupancy Layer
        public DynamicOccupancyLayer GetPersonalOccupancyLayer()
        {
            return personalOccupancyLayer;
        }

        public void UpdatePersonalLayer()
        {
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
            
            Debug.Log($"Robot {Id} entered intersection {intersection.Id}, entry direction: {entryDirection}, " +
                    $"suggested exit: {suggestedExitDirection}");
        }

        private void ExitIntersection()
        {
            if (currentIntersectionId == null) return;
            
            Debug.Log($"Robot {Id} exited intersection {currentIntersectionId}");
            currentIntersectionId = null;
        }
        #endregion

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
            
            // Check if emergency stop can be resumed
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
                
                Debug.Log($"Robot {Id}: Emergency stop activated");
                
                // Mark for path recovery
                needsPathRecovery = true;
            }
        }

        private void ResumeFromEmergencyStop()
        {
            if (isEmergencyStopped)
            {
                isEmergencyStopped = false;
                
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
            SetState(RobotState.Moving, true);
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
            
            SetState(RobotState.Moving, true);
            
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
            
            // 设置当前路径
            currentPath = new List<ConnectionPoint>(e.Path);
            pathTimes = new List<float>(e.Times);
            currentPlanId = e.PlanId;
            
            // 更新移动信息
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
                    Vector3 start = GetPointVector3((Point)currentPath[i].Point);
                    Vector3 end = GetPointVector3((Point)currentPath[i + 1].Point);
                    
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
