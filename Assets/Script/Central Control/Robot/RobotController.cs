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

namespace CentralControl.RobotControl
{
    public enum RobotState
    {
        Moving, 
        Picking
    }
    
    public class RobotController : MonoBehaviour
    {
        public Transform targetIndicator;
        public SearchAlgorithm selectedAlgorithm = SearchAlgorithm.Astar_Basic;
        public SearchMethod selectedSearchMethod = SearchMethod.Manhattan_Distance;
        public float baseDetectionRadius = 0.7f;
        public float emergencyStopDistance = 0.2f;
        public float defaultSpeed = 2.0f;
        public float maxAcceleration = 2.0f;
        public float maxDeceleration = 5.0f;
        public int MaxOrder = 20;
        public int Id { get; private set; }
        public Graph graph { get; private set; }
        public IndoorSpace indoorSpace { get; private set; }
        public MapGrid mapGrid { get; private set; }
        public bool lastReportedAvailability { get; set; }
        public bool lastReportedFreeStatus { get; set; }
        public int lastReportedOrderCount { get; set; } = -1;

        # region path visualization paramters
        public LineRenderer pathLineRenderer;
        public float lineWidth = 0.1f;
        public Color pathColor = Color.yellow;
        # endregion

        # region map parameters
        private DynamicOccupancyLayer personalOccupancyLayer;
        private DynamicOccupancyLayer globalOccupancyLayer;
        private const float TIME_CORRECTION_THRESHOLD = 0.5f;
        # endregion

        # region robot status parameters
        private HashSet<RobotState> currentStates = new HashSet<RobotState>();
        private HashSet<RobotState> lastReportedState = new HashSet<RobotState>();
        private ConcurrentQueue<Order> ordersQueue = new ConcurrentQueue<Order>();
        private int isProcessingOrder = 0;
        private bool isProcessingOrdersRunning = false;
        private Order currentOrder;
        private Coroutine statusCheckCoroutine;

        public bool IsMoving => currentStates.Contains(RobotState.Moving);
        public bool IsPicking => currentStates.Contains(RobotState.Picking);
        public bool IsFree => currentStates.Count == 0;
        public bool IsOnTask => !IsFree;
        public bool IsAvailable => ordersQueue.Count < MaxOrder;
        public bool IsProcessingOrders => isProcessingOrdersRunning;
        public int GetRobotOrdersQueueCount => ordersQueue.Count;
        public List<string> listOrdersQueue => new List<string>(ordersQueue.Select(order => order.Id));
        # endregion

        # region system status parameters
        private bool isInitialized = false;
        # endregion

        # region dwa parameters
        private DWAPlanner dwaPlanner;
        private float dwaScoreThreshold = 0.3f;
        private IntersectionAreaManager intersectionManager;
        private bool isDWAEnabled = false;
        private const float INTERSECTION_ACTIVATION_DISTANCE = 1.0f;
        private const int LOOK_AHEAD_POINTS = 2;
        private float lastDWAStateChangeTime = 0f;
        private const float DWA_STATE_COOLDOWN = 2.0f;
        private bool isIntersectionControlled = false;
        # endregion

        # region collision avoidance parameters
        private float detectionRadius;
        private Coroutine currentMovementCoroutine;
        private bool isEmergencyStop = false;
        
        private float lastReplanTime = 0f;
        private float replanCooldown = 2f;
        # endregion

        # region path deviation parameters
        public float pathDeviationDistanceThreshold = 1.0f;
        public float pathCheckInterval = 0.5f;
        private static readonly float maxPathDeviationDistance = Mathf.Sqrt(17f) / 2f;
        # endregion

        # region robot movement parameters 
        private Vector3 currentPosition;
        private List<ConnectionPoint> currentPath;
        private List<PathSegment> currentMovement;
        private int currentPathIndex;
        private string currentPlanId;
        private bool isPathValid = false;
        # endregion

        # region dynamic occupancy layer parameters
        private CellSpace lastCellSpace;
        private Dictionary<string, float> cellSpaceEntryTimes;
        # endregion

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


        private void Start()
        {
            statusCheckCoroutine = StartCoroutine(PeriodicStatusCheck());
            StartCoroutine(CheckPathDeviation());
        }

        private void Update()
        {
            if (!isInitialized) return;

            UpdatePersonalLayer();
            CheckSurroundings();
        }

        private IEnumerator PeriodicStatusCheck()
        {
            while (true)
            {
                yield return new WaitForSeconds(10f);
                if (IsFree && ordersQueue.Count > 0 && !isProcessingOrdersRunning)
                {
                    Debug.LogWarning($"Robot {Id} is idle but has orders. Restarting order processing.");
                    StartCoroutine(ProcessOrders());
                }
                else
                {
                    if (lastReportedState != currentStates || lastReportedOrderCount != ordersQueue.Count)
                    {
                        Debug.Log($"Robot {Id} status - Moving: {IsMoving}, Picking: {IsPicking}, Orders: {ordersQueue.Count}, Processing: {isProcessingOrdersRunning}");
                        lastReportedState = new HashSet<RobotState>(currentStates);
                        lastReportedOrderCount = ordersQueue.Count;
                    }
                }
            }
        }

        # region initialization module
        public void InitializeRobot(
            int id, 
            IndoorSpace indoorSpace, 
            Graph graph, 
            MapGrid mapGrid, 
            DynamicOccupancyLayer globalOccupancyLayer, 
            IntersectionAreaManager intersectionManager)
        {
            Id = id;
            this.indoorSpace = indoorSpace;
            this.graph = graph;
            this.mapGrid = mapGrid;
            this.globalOccupancyLayer = globalOccupancyLayer;
            this.intersectionManager = intersectionManager;

            currentStates.Clear();
            ordersQueue = new ConcurrentQueue<Order>();

            cellSpaceEntryTimes = new Dictionary<string, float>();
            lastCellSpace = null;
            currentPosition = transform.position;
            currentPath = new List<ConnectionPoint>();
            currentPathIndex = 0;
            currentMovement = new List<PathSegment>();

            this.personalOccupancyLayer = new DynamicOccupancyLayer(
                indoorSpace,
                globalOccupancyLayer.GetWindowDuration(), 
                Time.time
                );

            InitializeDWAPlanner();

            isInitialized = true;
        }

        private void InitializeDWAPlanner()
        {
            dwaPlanner = new DWAPlanner
            {
                MaxSpeed = defaultSpeed,
                MinSpeed = 0.5f,
                MaxRotSpeed = 0.5f,
                MaxAccel = maxAcceleration,
                VelocityResolution = 0.05f,
                RotationResolution = 0.1f,
                PredictionTime = 1.5f,
                HeadingWeight = 0.25f,
                DistanceWeight = 0.2f,
                VelocityWeight = 0.3f, 
                ObstacleWeight = 0.25f,
                DWAScoreThreshold = dwaScoreThreshold, 
                ObstacleLayer = ~(LayerMask.GetMask("GroundLayer") | LayerMask.GetMask("BoundaryWall"))
            };
        }
        # endregion

        # region order processing module
        public void ReceiveOrder(Order order)
        {
            if (order.AssignedRobotId == null || order.AssignedRobotId == Id)
            {
                if (ordersQueue.Any(o => o.Id == order.Id))
                {
                    Debug.LogWarning($"Robot {Id}: Order {order.Id} is already in the queue. Skipping.");
                    return;
                }

                order.AssignedRobotId = Id;
                ordersQueue.Enqueue(order);
                Debug.Log($"Robot {Id} received order {order.Id}");
                
                if (Interlocked.CompareExchange(ref isProcessingOrder, 1, 0) == 0 && !isProcessingOrdersRunning)
                {
                    StartCoroutine(ProcessOrders());
                }
            }
            else
            {
                Debug.LogError($"Order {order.Id} is already assigned to another robot {order.AssignedRobotId}");
            }
        }

        private IEnumerator ProcessOrders()
        {
            isProcessingOrdersRunning = true;
            while (ordersQueue.TryDequeue(out Order order))
            {
                currentOrder = order;
                Debug.Log($"Robot {Id} starts processing order {order.Id}");
                yield return StartCoroutine(ExecuteOrderCoroutine(order));
            }

            Debug.Log($"Robot {Id} has no more orders to execute");
            CentralController.NotifyRobotFree(this);
            Interlocked.Exchange(ref isProcessingOrder, 0);
            isProcessingOrdersRunning = false;
        }

        private IEnumerator ExecuteOrderCoroutine(Order order)
        {
            float startTime = Time.time;
            float timeout = 45f;

            if (!ValidateOrder(order))
            {
                Debug.LogError($"Robot {Id}: Order {order.Id} validation failed.");
                FailOrder(order);
                yield break;
            }
            
            Vector3 targetPosition = GetTargetPosition(order.PickingPoint);
            SetTargetIndicator(targetPosition);
            SetState(RobotState.Moving, true);

            List<ConnectionPoint> path = new List<ConnectionPoint>();
            List<float> times = new List<float>();
            List<float> speeds = new List<float>();
            string newPlanId = null;

            (path, times, speeds, newPlanId) = PlanPath(targetPosition);
            if (path != null && path.Count > 0)
            {
                currentPlanId = newPlanId;
                currentMovementCoroutine = StartCoroutine(MoveAlongPath(path, times, speeds, order.ExecutionTime, targetPosition));
                yield return currentMovementCoroutine;
            }
            else
            {
                FailOrder(order);
            }

            if (Time.time - startTime >= timeout)
            {
                FailOrder(order);
            }
            UpdateRobotStatus(); 
        }

        private bool ValidateOrder(Order order)
        {
            if (order == null || order.PickingPoint == null)
            {
                Debug.LogError($"Robot {Id}: Invalid order or picking point");
                return false;
            }

            CellSpace correspondingCellSpace = indoorSpace.GetCellSpaceFromId(order.PickingPoint.Id);
            if (correspondingCellSpace == null || correspondingCellSpace.Node == null)
            {
                Debug.LogError($"Robot {Id}: Invalid cell space for order {order.Id}");
                return false;
            }

            return true;
        }

        private void FailOrder(Order order)
        {
            Debug.LogError($"Robot {Id}: Failed to execute order {order.Id}");
            SetState(RobotState.Moving, false);
            SetState(RobotState.Picking, false);
            UpdateRobotStatus();
        }
        # endregion

        # region path planning module
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

            RoutePoint startPoint = graph.GetRoutePointFromCoordinate(transform.position, true);
            RoutePoint endPoint = graph.GetRoutePointFromCoordinate(targetPosition, false);

            Debug.Log($"Start RoutePoint: {startPoint?.ConnectionPoint.Id}, End RoutePoint: {endPoint?.ConnectionPoint.Id}");

            if (startPoint == null || endPoint == null)
            {
                Debug.LogError($"Robot {Id}: Invalid start or end point for path planning. Start: {startPoint}, End: {endPoint}");
                return (null, null, null, null);
            }
            
            PathPlanner pathPlanner = new PathPlanner(
                graph, 
                indoorSpace,
                selectedAlgorithm, 
                selectedSearchMethod, 
                Id.ToString(), 
                maxAcceleration, 
                maxDeceleration);
            var (path, times, speeds, newPlanId) = pathPlanner.FindPathWithDynamicObstacles(
                this.personalOccupancyLayer, 
                this.globalOccupancyLayer, 
                startPoint, 
                endPoint, 
                Id.ToString(),
                currentPlanId,
                Time.time, 
                defaultSpeed);
            
            if (path == null || path.Count == 0)
            {
                Debug.LogWarning($"Robot {Id}: Failed to find path with dynamic obstacles");
            }
            else
            {
                UpdateCurrentMovementInfo(path, times, speeds);
                List<string> pathId = new List<string>();
                foreach (var connectionPoint in path)
                {
                    pathId.Add(connectionPoint.Id);
                    currentPath.Add(connectionPoint);
                }

                Debug.Log($"Robot {Id}: Path found with {path.Count} steps: {string.Join(" -> ", pathId)}");
                Debug.Log($"Robot {Id}: New plan ID: {newPlanId}");
            }

            return (path, times, speeds, newPlanId);
        }
        # endregion

        # region moving module
        private IEnumerator MoveAlongPath(List<ConnectionPoint> path, List<float> times, List<float>speeds, float executionTime, Vector3 finalTargetPosition)
        {
            if (path == null || times == null || speeds == null || 
                path.Count == 0 || times.Count == 0 || speeds.Count == 0)
            {
                Debug.LogError($"Robot {Id}: Invalid path data received. Path count: {path?.Count}, Times count: {times?.Count}, Speeds count: {speeds?.Count}");
                StartCoroutine(ReplanPath());
                yield break;
            }

            if (path.Count != times.Count || path.Count != speeds.Count)
            {
                Debug.LogError($"Robot {Id}: Mismatched array lengths - Path: {path.Count}, Times: {times.Count}, Speeds: {speeds.Count}");
                StartCoroutine(ReplanPath());
                yield break;
            }
            
            VisualizePath(path);
            SetState(RobotState.Moving, true);
            SetState(RobotState.Picking, false);
            
            for (int index = 0; index < path.Count; index++)
            {
                if (!IsMoving)
                {
                    Debug.LogWarning($"Robot {Id}: Movement interrupted");
                    yield break;
                }

                ConnectionPoint connectionPoint = path[index];
                float targetTime = times[index];
                float speed = speeds[index];
                Vector3 targetPosition = GetPointVector3(connectionPoint.Point);

                UpdateDWAMaxSpeed(speed);

                yield return StartCoroutine(CorrectTimeAndWait(targetTime));

                if (index < path.Count - 1)
                {
                    yield return StartCoroutine(MoveToPosition(targetPosition, speed, applySlowdown: false));
                }
                else
                {
                    yield return StartCoroutine(MoveToPosition(targetPosition, speed, applySlowdown: true));
                }
            }

            yield return StartCoroutine(MoveToPosition(finalTargetPosition, defaultSpeed, applySlowdown: true));

            UpdateDWAMaxSpeed(defaultSpeed);
            
            Debug.Log($"Robot {Id} reached destination, executing order for {executionTime} seconds");
            
            SetState(RobotState.Moving, false);
            SetState(RobotState.Picking, true);
            Debug.Log($"Robot {Id} finished moving, starting picking operation");

            yield return new WaitForSeconds(executionTime);
            Debug.Log($"Robot {Id} finished picking operation");

            UpdateRobotStatus();
        }

        private IEnumerator MoveToPosition(Vector3 target, float maxSpeed, bool applySlowdown = true)
        {
            Rigidbody rb = GetComponent<Rigidbody>();
            float slowdownDistance = applySlowdown ? 2.0f : 0.1f;
            float minSpeed = applySlowdown ? 0.5f : maxSpeed;

            while (Vector3.Distance(transform.position, target) > 0.1f)
            {
                var (optimalVelocity, needReplan) = dwaPlanner.CalculateVelocity(transform.position, rb.velocity, target);
                // Debug.Log($"Robot {Id}: Optimal velocity: {optimalVelocity}");

                float distanceToTarget = Vector3.Distance(transform.position, target);

                if (distanceToTarget < slowdownDistance)
                {
                    float t = distanceToTarget / slowdownDistance;
                    optimalVelocity = Vector3.Lerp(optimalVelocity.normalized * minSpeed, optimalVelocity, t);
                }
                else
                {
                    rb.velocity = optimalVelocity;
                }

                yield return null;
            }
            rb.velocity = Vector3.zero;
        }

        private IEnumerator MoveToNearestPointThenFollowPath(List<ConnectionPoint> remainingPath, List<float> remainingTimes, List<float> remainingSpeeds)
        {
            Vector3 nearestPoint = GetPointVector3(remainingPath[0].Point);
            yield return StartCoroutine(MoveToPosition(nearestPoint, remainingSpeeds[0], true));

            yield return StartCoroutine(MoveAlongPath(remainingPath, remainingTimes, remainingSpeeds, currentOrder.ExecutionTime, GetTargetPosition(currentOrder.PickingPoint)));
        }

        private Vector3 GetPointVector3(Geometry point)
        {
            return new Vector3((float)((Point)point).X, mapGrid.height, (float)((Point)point).Y);
        }

        public void SetTargetIndicator(Vector3 position)
        {
            if (targetIndicator != null)
            {
                targetIndicator.position = position;
            }
        }
        # endregion

        # region replanning module
        private IEnumerator ReplanPath(int retryCount = 0, int maxRetries = 5)
        {
            isPathValid = false;
            ClearPathVisualization();
            lastReplanTime = Time.time;

            Vector3 targetPosition = GetTargetPosition(currentOrder.PickingPoint);
            string planIdForReplanning = currentPlanId;

            while (retryCount < maxRetries)
            {
                RoutePoint startPoint = graph.GetRoutePointFromCoordinate(transform.position, true);
                RoutePoint endPoint = graph.GetRoutePointFromCoordinate(targetPosition, false);

                if (startPoint == null || endPoint == null)
                {
                    Debug.LogWarning($"Robot {Id}: Invalid start or end point for path planning. Start: {startPoint}, End: {endPoint}. Retry {retryCount + 1}/{maxRetries}");
                    retryCount++;
                    yield return new WaitForSeconds(Mathf.Pow(2, retryCount));
                    continue;
                }

                var (newPath, newTimes, newSpeeds, newPlanId) = PlanPath(targetPosition, planIdForReplanning);

                if (newPath != null && newPath.Count > 0)
                {
                    isPathValid = true;
                    currentPathIndex = 0;
                    currentPath = newPath;
                    currentPlanId = newPlanId;

                    currentMovementCoroutine = StartCoroutine(MoveAlongPath(newPath, newTimes, newSpeeds, currentOrder.ExecutionTime, targetPosition));
                    yield break;
                }
                else
                {
                    Debug.LogWarning($"Robot {Id}: Path planning failed on attempt {retryCount + 1}/{maxRetries}");
                    retryCount++;
                    if (retryCount < maxRetries)
                    {
                        float waitTime = Mathf.Pow(2, retryCount);
                        Debug.Log($"Robot {Id}: Waiting {waitTime} seconds before retry...");
                        yield return new WaitForSeconds(waitTime);
                    }
                }
            }

            if (retryCount >= maxRetries)
            {
                Debug.LogError($"Robot {Id}: All path planning attempts failed after {maxRetries} retries");
                FailOrder(currentOrder);
            }
        }
        # endregion

        # region DWA control module
        private void CheckIntersectionAndControlDWA()
        {
            // check if the robot is approaching an intersection entry point or has reached an intersection exit point,
            // and enable/disable DWA accordingly

            if (currentPath == null || currentPath.Count <= currentPathIndex)
                return;

            // check if dwa control is on cooldown
            if (Time.time - lastDWAStateChangeTime < DWA_STATE_COOLDOWN)
            {
                return;
            }


            // check if the robot is approaching an intersection entry point or has reached an intersection exit point
            bool shouldActivateDWA = CheckUpcomingIntersectionEntry();
            bool shouldDeactivateDWA = CheckCurrentIntersectionExit();

            // only when dwa is not controlled by collision avoidance, we can control it by intersection
            if (shouldActivateDWA && !isDWAEnabled && !isEmergencyStop)
            {
                Debug.Log($"Robot {Id}: Activating DWA due to intersection");
                EnableDWA();
                isIntersectionControlled = true;
                lastDWAStateChangeTime = Time.time;
            }
            else if (shouldDeactivateDWA && isDWAEnabled && isIntersectionControlled)
            {
                // only when dwa is controlled by intersection, we can deactivate it
                Debug.Log($"Robot {Id}: Deactivating DWA due to intersection exit");
                DisableDWA();
                isIntersectionControlled = false;
                lastDWAStateChangeTime = Time.time;
            }
        }

        private bool CheckUpcomingIntersectionEntry()
        {
            // check if the robot is approaching an intersection entry point
            
            Vector3 robotPosition = transform.position;
            foreach (var intersection in intersectionManager.GetAllIntersections().Values)
            {
                if (intersection.Contains(robotPosition))
                {
                    return true;
                }
            }

            for (int i = currentPathIndex; i < Mathf.Min(currentPathIndex + LOOK_AHEAD_POINTS, currentPath.Count); i++)
            {
                ConnectionPoint point = currentPath[i];
                foreach (var intersection in intersectionManager.GetAllIntersections().Values)
                {
                    if (intersection.EntryPoints.Any(ep => ep.Id == point.Id))
                    {
                        // calculate distance to entry point
                        Vector3 entryPosition = GetPointVector3(point.Point);
                        float distance = Vector3.Distance(transform.position, entryPosition);
                        
                        if (distance <= INTERSECTION_ACTIVATION_DISTANCE)
                        {
                            Debug.Log($"Robot {Id}: Approaching intersection entry point, distance: {distance}");
                            return true;
                        }
                    }
                }
            }
            return false;
        }

        private bool CheckCurrentIntersectionExit()
        {
            // check if the robot has reached an intersection exit point

            if (currentPathIndex >= currentPath.Count)
                return false;

            ConnectionPoint currentPoint = currentPath[currentPathIndex];
            foreach (var intersection in intersectionManager.GetAllIntersections().Values)
            {
                if (!intersection.Contains(transform.position))
                {
                    if (intersection.ExitPoints.Any(ep => ep.Id == currentPoint.Id))
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        private void EnableDWA()
        {
            if (currentPath == null || currentPath.Count == 0 || currentPathIndex >= currentPath.Count)
            {
                Debug.LogWarning($"Robot {Id}: Cannot enable DWA without valid path");
                return;
            }

            if (!isDWAEnabled)
            {
                isDWAEnabled = true;
                dwaPlanner.HeadingWeight = 0.25f;
                dwaPlanner.DistanceWeight = 0.2f;
                dwaPlanner.VelocityWeight = 0.3f;
                dwaPlanner.ObstacleWeight = 0.25f;
                lastDWAStateChangeTime = Time.time;
            }
        }

        private void DisableDWA()
        {
            if (isDWAEnabled)
            {
                if (!CanDisableDWA())
                {
                    Debug.Log($"Robot {Id}: Cannot safely disable DWA yet");
                    return;
                }

                isDWAEnabled = false;
                isIntersectionControlled = false;
                dwaPlanner.HeadingWeight = 0.4f;
                dwaPlanner.DistanceWeight = 0.3f;
                dwaPlanner.VelocityWeight = 0.2f;
                dwaPlanner.ObstacleWeight = 0.1f;
                lastDWAStateChangeTime = Time.time;
            }
        }
        # endregion

        # region collision detection & avoidance module
        private float CalculateStoppingDistance()
        {
            Rigidbody rb = GetComponent<Rigidbody>();
            float currentSpeed = rb.velocity.magnitude;
            float brakingDistance = (currentSpeed * currentSpeed) / (2 * maxDeceleration);
            return brakingDistance;
        }

        private void UpdateDetectionRadius()
        {
            detectionRadius = Mathf.Max(baseDetectionRadius, 0.7f * CalculateStoppingDistance());
        }

        private float CalculateCollisionPredictionThreshold()
        {
            Rigidbody rb = GetComponent<Rigidbody>();
            float currentSpeed = rb.velocity.magnitude;
            
            const float BASE_PREDICTION_TIME = 1.0f;
            const float MIN_PREDICTION_TIME = 0.5f;
            const float MAX_PREDICTION_TIME = 3.0f;
            
            const float MIN_SPEED = 0.5f;
            const float MAX_SPEED = 5.0f;
            
            if (currentSpeed <= MIN_SPEED)
                return MAX_PREDICTION_TIME;
            
            if (currentSpeed >= MAX_SPEED)
                return MIN_PREDICTION_TIME;
            
            float speedFactor = (currentSpeed - MIN_SPEED) / (MAX_SPEED - MIN_SPEED);
            float predictionTime = Mathf.Lerp(MAX_PREDICTION_TIME, MIN_PREDICTION_TIME, speedFactor);
            
            float stoppingTime = currentSpeed / maxDeceleration;
            
            predictionTime = Mathf.Max(predictionTime, stoppingTime + BASE_PREDICTION_TIME);
            
            return Mathf.Clamp(predictionTime, MIN_PREDICTION_TIME, MAX_PREDICTION_TIME);
        }

        private bool CheckFrontalCollisionRisk(Vector3 otherPosition, Vector3 otherVelocity)
        {
            Vector3 relativePosition = otherPosition - transform.position;
            Vector3 robotVelocity = GetComponent<Rigidbody>().velocity;
            Vector3 relativeVelocity = robotVelocity - otherVelocity;
            
            if (relativeVelocity.magnitude > 0.1f)
            {
                float angleThreshold = 90f;
                float angle = Vector3.Angle(robotVelocity, -otherVelocity);
                
                if (angle < angleThreshold)
                {
                    // Time to Collision - TTC
                    float ttc = relativePosition.magnitude / relativeVelocity.magnitude;
                    
                    float predictionThreshold = CalculateCollisionPredictionThreshold();
                    
                    float minSafeDistance = Mathf.Max(
                        baseDetectionRadius,
                        robotVelocity.magnitude * predictionThreshold * 0.5f
                    );
                    
                    bool timeRisk = ttc < predictionThreshold;
                    bool distanceRisk = relativePosition.magnitude < minSafeDistance;
                    
                    if (timeRisk || distanceRisk)
                    {
                        return true;
                    }
                }
            }
            
            return false;
        }

        private bool CanDisableDWA()
        {
            LayerMask detectionMask = ~(LayerMask.GetMask("GroundLayer") | LayerMask.GetMask("BoundaryWall"));
            Collider[] colliders = Physics.OverlapSphere(transform.position, detectionRadius * 1.5f, detectionMask);
            
            foreach (var collider in colliders)
            {
                if (collider.gameObject == gameObject) continue;
                
                // 获取其他机器人的信息
                RobotController otherRobot = collider.GetComponent<RobotController>();
                if (otherRobot != null)
                {
                    Vector3 otherPosition = otherRobot.transform.position;
                    Vector3 otherVelocity = otherRobot.GetComponent<Rigidbody>().velocity;
                    
                    // 检查距离和相对速度
                    float distance = Vector3.Distance(transform.position, otherPosition);
                    if (distance < detectionRadius * 1.2f || CheckFrontalCollisionRisk(otherPosition, otherVelocity))
                    {
                        return false;
                    }
                }
            }
            
            // 检查是否回到了原计划路径附近
            if (currentPath != null && currentPath.Count > currentPathIndex)
            {
                Vector3 plannedPosition = GetPointVector3(currentPath[currentPathIndex].Point);
                float deviationDistance = Vector3.Distance(transform.position, plannedPosition);
                if (deviationDistance > pathDeviationDistanceThreshold)
                {
                    return false;
                }
            }
            
            return true;
        }

        private void CheckSurroundings()
        {
            CheckIntersectionAndControlDWA();
            UpdateDetectionRadius();

            // get surrounding objects
            Vector3 robotVelocity = GetComponent<Rigidbody>().velocity;
            LayerMask detectionMask = ~(LayerMask.GetMask("GroundLayer") | LayerMask.GetMask("BoundaryWall"));
            Collider[] colliders = Physics.OverlapSphere(transform.position, detectionRadius, detectionMask);

            bool potentialCollision = false;
            bool needEmergencyStop = false;

            foreach (var collider in colliders)
            {
                if (collider.gameObject == gameObject) continue;

                RobotController otherRobot = collider.GetComponent<RobotController>();
                Vector3 directionToOther = collider.transform.position - transform.position;
                float distance = directionToOther.magnitude;

                // check if the robot is too close to another robot
                if (distance < emergencyStopDistance)
                {
                    needEmergencyStop = true;
                    break;
                }

                // check if the robot is at risk of frontal collision with another robot
                if (otherRobot != null)
                {
                    Vector3 otherVelocity = otherRobot.GetComponent<Rigidbody>().velocity;
                    if (CheckFrontalCollisionRisk(collider.transform.position, otherVelocity))
                    {
                        potentialCollision = true;
                    }
                }
            }

            if (needEmergencyStop)
            {
                StartCoroutine(EmergencyStop());
            }
            else if (potentialCollision && !isDWAEnabled)
            {
                EnableDWA();
            }
            else if (isDWAEnabled && !potentialCollision && CanDisableDWA())
            {
                DisableDWA();
            }

            if (isDWAEnabled)
            {
                if (currentPath == null || currentPath.Count == 0 || currentPathIndex >= currentPath.Count)
                {
                    Debug.LogWarning($"Robot {Id}: Invalid path or path index. Path count: {currentPath?.Count}, Current index: {currentPathIndex}");
                    StartCoroutine(ReplanPath());
                    return;
                }

                Vector3 targetPosition = GetPointVector3(currentPath[currentPathIndex].Point);
                var (optimalVelocity, needReplan) = dwaPlanner.CalculateVelocity(
                    transform.position,
                    robotVelocity,
                    targetPosition
                );

                GetComponent<Rigidbody>().velocity = Vector3.SmoothDamp(
                    GetComponent<Rigidbody>().velocity,
                    optimalVelocity,
                    ref robotVelocity,
                    0.1f,
                    maxAcceleration
                );

                if (needReplan && Time.time - lastReplanTime > replanCooldown)
                {
                    StartCoroutine(ReplanPath());
                }
            }
        }

        private IEnumerator EmergencyStop()
        {
            // Stop -> Replan -> DWA avoid obstacles
            isEmergencyStop = true;
            Debug.LogWarning($"Robot {Id} performing emergency stop!");

            Rigidbody rb = GetComponent<Rigidbody>();
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;

            yield return new WaitForSeconds(1.0f);
            yield return StartCoroutine(ReplanPath());

            isIntersectionControlled = false;
            EnableDWA();
            isEmergencyStop = false;
        }

        private void OnCollisionEnter(Collision collision)
        {
            HandleCollision(collision.gameObject);
        }

        private void HandleCollision(GameObject collisionObject)
        {
            Debug.Log($"Robot {Id} collided with {collisionObject.name}.");
            StartCoroutine(EmergencyStop());
        }

        private void OnTriggerEnter(Collider other)
        {
            HandleTrigger(other.gameObject);
        }
        private void HandleTrigger(GameObject triggeringObject)
        {
            Debug.Log($"Robot {Id} triggered by {triggeringObject.name}");
        }
        # endregion

        # region dynamic occupancy layer module
        private void UpdatePersonalLayer()
        {
            Vector3 newPosition = transform.position;
            if (newPosition != currentPosition)
            {
                currentPosition = newPosition;
            }

            CellSpace currentCellSpace = indoorSpace.GetCellSpaceFromCoordinates(currentPosition);
            if (currentCellSpace != null)
            {
                // Check if the robot has entered a new cell space
                if (lastCellSpace != null && lastCellSpace.Id != currentCellSpace.Id)
                {
                    // End the occupancy in the last CellSpace
                    personalOccupancyLayer.EndCurrentOccupancy(
                        lastCellSpace.Id,
                        Id.ToString(),
                        Time.time
                    );

                    // Update the entry time for the new CellSpace
                    personalOccupancyLayer.UpdateCurrentOccupancy(
                        currentCellSpace.Id,
                        Time.time,
                        Id.ToString()
                    );
                }
                else if (lastCellSpace == null)
                {
                    // If the robot has just started moving
                    personalOccupancyLayer.UpdateCurrentOccupancy(
                        currentCellSpace.Id,
                        Time.time,
                        Id.ToString()
                    );
                }

                lastCellSpace = currentCellSpace;
            }
            else
            {
                // If the robot is not within any cell space
                if (lastCellSpace != null)
                {
                    personalOccupancyLayer.EndCurrentOccupancy(
                        lastCellSpace.Id,
                        Id.ToString(),
                        Time.time
                    );
                    lastCellSpace = null;
                }
                Debug.LogError($"Robot {Id}: Current position is not within any cell space.");
            }
        }

        public void UpdateGlobalLayer(DynamicOccupancyLayer globalLayer)
        {
            if (globalOccupancyLayer == null)
            {
                Debug.LogError($"Robot {Id}: Global occupancy layer is null.");
                return;
            }
            this.globalOccupancyLayer = globalLayer;
        }

        private IEnumerator CorrectTimeAndWait(float targetTime)
        {
            while (Time.time < targetTime - TIME_CORRECTION_THRESHOLD)
            {
                float timeDifference = Time.time - targetTime;

                if (Mathf.Abs(timeDifference) > TIME_CORRECTION_THRESHOLD)
                {
                    UpdateTimeSpaceMatrix(Time.time);
                }
                yield return null;
            }
        }

        private void UpdateTimeSpaceMatrix(float currentTime)
        {
            CellSpace currentCellSpace = indoorSpace.GetCellSpaceFromCoordinates(transform.position);
            
            if (currentCellSpace == null)
            {
                Debug.Log($"Robot {Id}: Currently in transition between CellSpaces");
                return;
            }

            if (lastCellSpace == null)
            {
                Debug.Log($"Robot {Id}: First time entering a CellSpace");
                cellSpaceEntryTimes[currentCellSpace.Id] = currentTime;
                lastCellSpace = currentCellSpace;
                return;
            }

            if (currentCellSpace.Id != lastCellSpace.Id)
            {
                Debug.Log($"Robot {Id}: Transitioning from CellSpace {lastCellSpace.Id} to {currentCellSpace.Id}");
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
                
                cellSpaceEntryTimes[currentCellSpace.Id] = currentTime;
                lastCellSpace = currentCellSpace;
                return;
            }

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

        public DynamicOccupancyLayer GetPersonalOccupancyLayer()
        {
            return personalOccupancyLayer;
        }
        # endregion

        # region path detection module
        private IEnumerator CheckPathDeviation()
        {
            while (true)
            {
                yield return new WaitForSeconds(pathCheckInterval);

                if (isPathValid && currentPath != null && currentPath.Count > currentPathIndex)
                {
                    Vector3 targetPosition = GetPointVector3(currentPath[currentPathIndex].Point);
                    float deviationDistance = Vector3.Distance(transform.position, targetPosition);

                    if (deviationDistance > maxPathDeviationDistance)
                    {
                        HandlePathDeviation(deviationDistance);
                    }
                }
            }
        }

        private void HandlePathDeviation(float deviationDistance)
        {
            Debug.LogWarning($"Robot {Id} has deviated from the path. Deviation distance: {deviationDistance}. Attempting to recover.");

            int nearestPointIndex = FindNearestPathPoint();

            if (nearestPointIndex != -1)
            {
                if (deviationDistance <= pathDeviationDistanceThreshold)
                {
                    currentPathIndex = nearestPointIndex;
                    List<PathSegment> remainingMovement = currentMovement.GetRange(nearestPointIndex, currentPath.Count - nearestPointIndex);
                    List<ConnectionPoint> remainingPath = new List<ConnectionPoint>();
                    List<float> remainingTimes = new List<float>();
                    List<float> remainingSpeeds = new List<float>();

                    foreach (PathSegment segment in remainingMovement)
                    {
                        remainingPath.Add(segment.Point);
                        remainingTimes.Add(segment.Time);
                        remainingSpeeds.Add(segment.Speed);
                    }
                    
                    if (currentMovementCoroutine != null)
                    {
                        StopCoroutine(currentMovementCoroutine);
                    }

                    currentMovementCoroutine = StartCoroutine(MoveToNearestPointThenFollowPath(remainingPath, remainingTimes, remainingSpeeds));
                    
                    Debug.Log($"Robot {Id} found a nearest path point. Resuming from index {currentPathIndex}");
                }
                else
                {
                    Debug.LogWarning($"Robot {Id} deviation too large. Initiating replan.");
                    StartCoroutine(ReplanPath());
                }
            }
            else
            {
                Debug.LogWarning($"Robot {Id} unable to find nearest path point. Initiating replan.");
                StartCoroutine(ReplanPath());
            }
        }

        private int FindNearestPathPoint()
        {
            float minDistance = float.MaxValue;
            int nearestIndex = -1;

            for (int i = currentPathIndex; i < currentPath.Count; i++)
            {
                float distance = Vector3.Distance(transform.position, GetPointVector3(currentPath[i].Point));
                if (distance < minDistance && distance <= pathDeviationDistanceThreshold)
                {
                    minDistance = distance;
                    nearestIndex = i;
                }
            }

            return nearestIndex;
        }
        # endregion
        
        # region status check module
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

        private void UpdateRobotStatus()
        {
            bool previousFreeStatus = IsFree;
            
            SetState(RobotState.Moving, false);
            SetState(RobotState.Picking, false);
            currentMovementCoroutine = null;

            if (previousFreeStatus != IsFree || lastReportedOrderCount != ordersQueue.Count)
            {
                Debug.Log($"Robot {Id} status updated. Is free: {IsFree}, Orders remaining: {ordersQueue.Count}");
                lastReportedOrderCount = ordersQueue.Count;
            }

            if (ordersQueue.Count > 0)
            {
                if (!isProcessingOrdersRunning)
                {
                    StartCoroutine(ProcessOrders());
                }
            }
            else if (!previousFreeStatus)
            {
                Debug.Log($"Robot {Id} is now free and has no orders.");
                CentralController.NotifyRobotFree(this);
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

        private void UpdateDWAMaxSpeed(float newMaxSpeed)
        {
            dwaPlanner.MaxSpeed = newMaxSpeed;
            // Debug.Log($"Robot {Id}: Updated DWA max speed to {newMaxSpeed}");
        }

        public void ResetRobot()
        {
            StopAllCoroutines();
            SetState(RobotState.Moving, false);
            SetState(RobotState.Picking, false);
            isProcessingOrdersRunning = false;
            Interlocked.Exchange(ref isProcessingOrder, 0);

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
                currentPlanId = null;
            }
            
            while (ordersQueue.TryDequeue(out _)) { }
            
            Debug.Log($"Robot {Id} has been reset.");
            StartCoroutine(ProcessOrders());
        }

        private void OnDisable()
        {
            if (lastCellSpace != null && cellSpaceEntryTimes.TryGetValue(lastCellSpace.Id, out float entryTime))
            {
                personalOccupancyLayer.SetPlannedOccupancy(
                    lastCellSpace.Id,
                    entryTime,
                    Time.time,
                    Id.ToString(),
                    "RealTimeMovement"
                );
            }

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
            StopAllCoroutines();
        }
        # endregion

        # region visualization module
        private void OnDrawGizmos()
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(transform.position, detectionRadius);

            Gizmos.color = new Color(1, 1, 0, 0.1f); 
            Gizmos.DrawSphere(transform.position, detectionRadius);

            UnityEditor.Handles.Label(transform.position + Vector3.up * 2, $"Detection Radius: {detectionRadius}");

            Rigidbody rb = GetComponent<Rigidbody>();
            if (rb != null)
            {
                Gizmos.color = Color.blue;
                Gizmos.DrawLine(transform.position, transform.position + rb.velocity * 0.5f);
                Gizmos.color = Color.red;
                Gizmos.DrawRay(transform.position, (GetTargetPosition(currentOrder.PickingPoint) - transform.position).normalized);
            }
        }

        private void VisualizePath(List<ConnectionPoint> path)
        {
            if (pathLineRenderer == null)
            {
                pathLineRenderer = gameObject.AddComponent<LineRenderer>();
            }

            pathLineRenderer.positionCount = path.Count;
            pathLineRenderer.startWidth = lineWidth;
            pathLineRenderer.endWidth = lineWidth;
            pathLineRenderer.material = new Material(Shader.Find("Sprites/Default"));
            pathLineRenderer.startColor = pathColor;
            pathLineRenderer.endColor = pathColor;

            Vector3[] positions = new Vector3[path.Count];
            for (int i = 0; i < path.Count; i++)
            {
                Point point = (Point)path[i].Point;
                positions[i] = new Vector3((float)point.X, 0.1f, (float)point.Y);
            }

            pathLineRenderer.SetPositions(positions);
        }

        private void ClearPathVisualization()
        {
            if (pathLineRenderer != null)
            {
                pathLineRenderer.positionCount = 0;
            }
        }
        # endregion
    }
}