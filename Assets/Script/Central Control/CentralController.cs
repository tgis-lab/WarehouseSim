using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;
using NetTopologySuite.Geometries;
using Map;
using CentralControl.OrderAssignment;
using CentralControl.RobotControl;
using CentralControl.PickerControl;
using PathPlanning;

namespace CentralControl
{
    public class CentralController : MonoBehaviour
    {
        #region Inspector Fields
        [Header("Components")]
        public OrderManager orderManager;
        public RobotManager robotManager;
        public PickerManager pickerManager;
        public MapLoader mapLoader;
        public IntersectionAreaManager intersectionAreaManager;
        public DeadlockDetector deadlockDetector;
        
        [Header("Map Settings")]
        public float width, length;
        
        [Header("Order Generation")]
        public float interval = 5f;
        public int orderCount = 10;
        [System.Serializable]
        public class OrderFloorDistribution
        {
            public string floorLevel = "1F";
            [Range(0, 1)]
            public float ratio = 0.5f;
        }

        [Tooltip("Distribution of orders across different floors")]
        public List<OrderFloorDistribution> floorOrderDistribution;
        
        [Header("Navigation Settings")]
        public float correctionFactor = 0.1f;
        public float dynamicUpdateInterval = 0.5f;
        public float windowDuration = 300f;
        public float timeStep = 0.1f;
        
        [Header("NavMesh Settings")]
        public bool autoRebuildNavMesh = true;
        public float navMeshRebuildInterval = 5.0f;
        public float offMeshConnectionRadius = 2.0f;
        
        [Header("Multi-Floor Settings")]
        public float floorHeight = 3.0f;
        public float baseHeight = 0.05f;
        #endregion
        
        #region Private Fields
        private DynamicOccupancyLayer globalOccupancyLayer;
        private IndoorSpace indoorSpace;
        private Graph graph;
        private MapGrid mapGrid;
        private ConflictResolver conflictResolver;
        private CellSpaceConflictDetector cellSpaceConflictDetector;
        private float nextNavMeshRebuildTime = 0f;
        private Coroutine orderGenerationCoroutine;
        private bool isInitialized = false;
        private MultiFloorOrderGenerator orderGenerator;
        private Dictionary<string, List<NavMeshLink>> floorNavMeshLinks = new Dictionary<string, List<NavMeshLink>>();
        public static event EventHandler<RobotStatusEventArgs> OnRobotBecameFree;
        private int totalRobotCollisions = 0;
        private int totalEnvironmentCollisions = 0;
        private Dictionary<string, int> robotCollisionCounts = new Dictionary<string, int>();
        private Dictionary<string, PathPlanningMetrics> pathPlanningMetrics = new Dictionary<string, PathPlanningMetrics>();

        #endregion

        #region Unity Lifecycle
        private void Start()
        {
            // Initialize the system
            StartCoroutine(InitializeSystem());
        }

        private IEnumerator InitializeSystem()
        {
            Debug.Log("[CentralControl] Initializing system...");

            // Wait for the first frame to complete
            yield return null;

            // Load map
            StartCoroutine(LoadMap());

            // Wait for map to be loaded AND NavMesh to be ready
            yield return new WaitUntil(() => indoorSpace != null && mapLoader.IsNavMeshReady);

            // Initialize global occupancy layer
            if (globalOccupancyLayer == null)
            {
                globalOccupancyLayer = new DynamicOccupancyLayer(
                    mapLoader.indoorSpace,
                    windowDuration,
                    Time.time,
                    true); // Enable 3D support
            }

            // Initialize Intersection Area Manager
            InitializeIntersectionAreaManager();

            // Initialize conflict resolver
            InitializeConflictDetector();

            // Initialize robot manager with elevator manager
            InitializeRobotManager();

            // Initialize picker manager
            InitializePickerManager();

            // Initialize order generator
            InitializeOrderGenerator();

            // Start generating orders
            if (orderGenerationCoroutine != null) StopCoroutine(orderGenerationCoroutine);
            orderGenerationCoroutine = StartCoroutine(GenerateOrders());

            // Initialize collision tracker
            InitializeCollisionTracking();

            // Initialize deadlock detector
            InitializeDeadlockDetector();

            // Mark initialization as complete
            isInitialized = true;

            Debug.Log("[CentralControl] System initialization complete!");

            // Notify system components that initialization is complete
            if (EventManager.Instance != null)
            {
                EventManager.Instance.TriggerMapLoaded();
                EventManager.Instance.TriggerRobotsInitialized();
                EventManager.Instance.TriggerPickersInitialized();
                EventManager.Instance.OnPathPlanningMetricsRecorded += OnPathPlanningMetricsRecorded;
            }

            // Start auto-saving collision reports
            StartCoroutine(AutoSaveCollisionReports(60f));
            StartCoroutine(AutoSavePathPlanningReports(60f));
        }
        
        void Update()
        {
            if (!isInitialized) return;

            // Update dynamic occupancy layer
            // if (Time.time >= nextUpdateTime)
            // {
            //     UpdateDynamicOccupancyLayer();
            //     nextUpdateTime = Time.time + dynamicUpdateInterval;
            // }
            UpdateDynamicOccupancyLayer();
            
            // Rebuild NavMesh if needed
            if (autoRebuildNavMesh && Time.time >= nextNavMeshRebuildTime)
            {
                RebuildNavMesh();
                nextNavMeshRebuildTime = Time.time + navMeshRebuildInterval;
            }
        }

        private void OnDestroy()
        {
            // Unregister event handlers
            if (EventManager.Instance != null)
            {
                EventManager.Instance.OnRobotCollisionDetected -= HandleRobotCollision;
                EventManager.Instance.OnRobotPathPlanned -= HandleRobotPathPlanned;
            }
            
            // Stop order generation coroutine
            if (orderGenerationCoroutine != null)
            {
                StopCoroutine(orderGenerationCoroutine);
                orderGenerationCoroutine = null;
            }
        }

        private void OnApplicationQuit()
        {
            // Save final reports before application exits to prevent data loss
            Debug.Log("[CentralControl] Application quitting - saving final reports...");
            
            try
            {
                SaveCollisionReportToFile();
                SavePathPlanningReportToFile();
                Debug.Log("[CentralControl] Final reports saved successfully");
            }
            catch (System.Exception e)
            {
                Debug.LogError($"[CentralControl] Error saving final reports on quit: {e.Message}");
            }
        }
        #endregion
        
        #region Initialization Methods
        private IEnumerator LoadMap()
        {
            Debug.Log("[CentralControl] Loading map...");
            
            if (mapLoader != null)
            {
                // Wait for the map to be loaded
                yield return new WaitUntil(() => mapLoader.IsMapLoaded);
                
                indoorSpace = mapLoader.indoorSpace;
                graph = mapLoader.GetGraph();
                mapGrid = mapLoader.GetMapGrid();
                
                Debug.Log($"[CentralControl] Map loaded with {indoorSpace.CellSpaces.Count} cell spaces and {indoorSpace.CellBoundaries.Count} cell boundaries");
                Debug.Log($"[CentralControl] Floor levels: {string.Join(", ", indoorSpace.GetAllFloorLevels())}");
            }
            else
            {
                Debug.LogError("[CentralControl] MapLoader component is missing!");
            }
        }
        
        private void InitializeRobotManager()
        {
            Debug.Log("[CentralControl] Initializing robot manager...");
            
            if (robotManager != null)
            {
                // Pass the necessary components
                robotManager.InitializeRobots(indoorSpace, graph, mapGrid, globalOccupancyLayer, intersectionAreaManager);
                
                // Register robot event handlers
                foreach (var robot in robotManager.GetAllRobots())
                {
                    robot.OnPathPlanned += HandleRobotPathPlanned;
                }
                
                Debug.Log($"[CentralControl] Robot manager initialized with {robotManager.TotalRobots} robots");
            }
            else
            {
                Debug.LogError("[CentralControl] RobotManager component is missing!");
            }
        }
        
        private void InitializePickerManager()
        {
            Debug.Log("[CentralControl] Initializing picker manager...");
            
            if (pickerManager != null)
            {
                // Pass the necessary components including elevator manager
                pickerManager.InitializePickers(indoorSpace, mapGrid);
                
                Debug.Log($"[CentralControl] Picker manager initialized with {pickerManager.TotalPickers} pickers");
            }
            else
            {
                Debug.LogError("[CentralControl] PickerManager component is missing!");
            }
        }
        
        private void InitializeOrderGenerator()
        {
            Debug.Log("[CentralControl] Initializing order generator...");
            
            if (indoorSpace == null)
            {
                Debug.LogError("[CentralControl] Cannot initialize OrderGenerator: IndoorSpace is null");
                return;
            }
            
            // Create MultiFloorOrderGenerator
            orderGenerator = new MultiFloorOrderGenerator(indoorSpace);
            
            // Convert inspector floor distributions to dictionary
            Dictionary<string, float> distribution = new Dictionary<string, float>();
            if (floorOrderDistribution != null && floorOrderDistribution.Count > 0)
            {
                foreach (var item in floorOrderDistribution)
                {
                    distribution[item.floorLevel] = item.ratio;
                }
            }
            else
            {
                // Create default distribution based on available floors
                var floors = indoorSpace.GetAllFloorLevels();
                float equalRatio = 1.0f / floors.Count;
                foreach (var floor in floors)
                {
                    distribution[floor] = equalRatio;
                }
            }
            
            Debug.Log($"[CentralControl] Order generator initialized with distribution: {string.Join(", ", distribution.Select(kv => $"{kv.Key}:{kv.Value}"))}");
        }

        private void InitializeIntersectionAreaManager()
        {
            Debug.Log("[CentralControl] Initializing intersection area manager...");
            
            if (intersectionAreaManager != null && indoorSpace != null)
            {
                intersectionAreaManager.InitializeIntersectionAreas(indoorSpace);
                Debug.Log("[CentralControl] Intersection area manager initialized");
            }
            else
            {
                Debug.LogWarning("[CentralControl] IntersectionAreaManager component is missing or IndoorSpace is null");
            }
        }

        private void InitializeCollisionTracking()
        {
            Debug.Log("[CentralControl] Initializing collision tracking system...");
            
            totalRobotCollisions = 0;
            totalEnvironmentCollisions = 0;
            robotCollisionCounts.Clear();
            
            if (EventManager.Instance != null)
            {
                EventManager.Instance.OnRobotCollisionDetected -= HandleRobotCollision;
                
                EventManager.Instance.OnRobotCollisionDetected += HandleRobotCollision;
                
                Debug.Log("[CentralControl] Collision tracking system initialized!");
            }
            else
            {
                Debug.LogError("[CentralControl] Failed to initialize collision tracking: EventManager not found!");
            }
        }

        private void InitializeDeadlockDetector()
        {
            Debug.Log("[CentralControl] Initializing deadlock detector...");
            
            if (deadlockDetector != null && globalOccupancyLayer != null && indoorSpace != null)
            {
                deadlockDetector.Initialize(globalOccupancyLayer, indoorSpace);
                Debug.Log("[CentralControl] Deadlock detector initialized!");
            }
            else
            {
                Debug.LogWarning("[CentralControl] DeadlockDetector component is missing or dependencies are null");
            }
        }
        #endregion
        
        #region Order Management
        private IEnumerator GenerateOrders()
        {
            Debug.Log("[CentralControl] Starting order generation...");
            
            while (true)
            {
                // Wait for the specified interval
                yield return new WaitForSeconds(interval);
                
                if (orderManager == null || orderGenerator == null)
                {
                    Debug.LogError("[CentralControl] OrderManager or OrderGenerator is missing!");
                    continue;
                }
                
                // Generate orders across multiple floors using the floor distribution
                List<Order> newOrders;
                
                if (floorOrderDistribution != null && floorOrderDistribution.Count > 0)
                {
                    Dictionary<string, float> distribution = floorOrderDistribution.ToDictionary(
                        fd => fd.floorLevel,
                        fd => fd.ratio
                    );
                    newOrders = orderGenerator.GenerateOrders(orderCount, distribution);
                }
                else
                {
                    newOrders = orderGenerator.GenerateOrders(orderCount);
                }
                
                // Add the orders to the order manager
                orderManager.AddOrders(newOrders);
                
                Debug.Log($"[CentralControl] Generated {newOrders.Count} orders across multiple floors");
                
                // Distribute orders to available robots
                DistributeOrders();
            }
        }
        
        private void DistributeOrders()
        {
            Debug.Log("[CentralControl] Distributing orders to robots...");
            
            if (robotManager == null || orderManager == null) return;
            
            // Get available free robots
            List<RobotController> freeRobots = robotManager.GetFreeRobots();
            
            if (freeRobots.Count == 0)
            {
                Debug.Log("[CentralControl] No free robots available for order distribution");
                return;
            }
            
            // Get unassigned orders
            List<Order> orders = orderManager.GetAllOrders()
                .Where(o => o.AssignedRobotId == null)
                .ToList();
            
            if (orders.Count == 0)
            {
                Debug.Log("[CentralControl] No unassigned orders available for distribution");
                return;
            }
            
            Debug.Log($"[CentralControl] Found {freeRobots.Count} free robots and {orders.Count} unassigned orders");
            
            // Create order assigner with indoor space for multi-floor path calculations
            OrderAssignmentSelector assigner = new OrderAssignmentSelector(indoorSpace);
            
            // Initial distribution (one order per robot)
            assigner.InitialOrderDistribution(orders, freeRobots);
            
            // After initial distribution, distribute remaining orders using clustering
            orders = orderManager.GetAllOrders()
                .Where(o => o.AssignedRobotId == null)
                .ToList();
            
            if (orders.Count > 0)
            {
                // Get all robots for cluster assignment
                List<RobotController> allAvailableRobots = robotManager.GetAvailableRobots();
                assigner.AssignOrders(orders, allAvailableRobots);
            }
            
            Debug.Log("[CentralControl] Order distribution complete");
        }
        #endregion
        
        #region Dynamic Occupancy
        private void UpdateDynamicOccupancyLayer()
        {
            globalOccupancyLayer.PruneExpiredOccupancies(Time.time - windowDuration);
            MergeRobotOccupancyLayer();

            if (EventManager.Instance != null)
            {
                EventManager.Instance.TriggerGlobalOccupancyLayerUpdated(globalOccupancyLayer, Time.time);
            }
            else
            {
                Debug.LogError("[CentralControl] EventManager not found. Cannot notify robots of global layer update.");
            }
        }
        
        public void UpdateOccupancy(List<ConnectionPoint> path, List<float> times, string robotId, string planId)
        {
            if (globalOccupancyLayer == null)
            {
                Debug.LogError("[CentralControl] Global occupancy layer is not initialized");
                return;
            }
            
            // Record path occupancy
            globalOccupancyLayer.RecordPathOccupancy(path, times, robotId, planId);
            
            Debug.Log($"[CentralControl] Updated occupancy for robot {robotId}, plan {planId}, path length {path.Count}");
        }

        private void MergeRobotOccupancyLayer()
        {
            foreach (var robot in robotManager.GetAllRobots())
            {
                if (robot == null) continue;
                
                var personalLayer = robot.GetPersonalOccupancyLayer();
                if (personalLayer != null)
                {
                    globalOccupancyLayer.MergeLayer(personalLayer, Time.time - windowDuration, Time.time + 60);
                }
            }
        }

        public DynamicOccupancyLayer GetGlobalOccupancyLayer()
        {
            return globalOccupancyLayer;
        }
        #endregion
        
        #region Conflict Detection
        // Initialize collision detector
        private void InitializeConflictDetector()
        {
            if (cellSpaceConflictDetector == null)
            {
                cellSpaceConflictDetector = new CellSpaceConflictDetector();
            }
            
            if (conflictResolver == null && globalOccupancyLayer != null && indoorSpace != null)
            {
                conflictResolver = new ConflictResolver(globalOccupancyLayer, cellSpaceConflictDetector, indoorSpace);
            }
            
            // Register event listeners
            if (EventManager.Instance != null)
            {
                EventManager.Instance.OnRobotPathPlanned += HandleRobotPathPlanned;
            }
        }
        
        // Detect path conflicts for a specific robot
        private bool DetectPathConflicts(string robotId, List<ConnectionPoint> path, List<float> times, string planId)
        {
            try
            {
                // Initial conflict detection
                Dictionary<string, List<TimeOccupancy>> conflicts = cellSpaceConflictDetector.DetectConflicts(
                    globalOccupancyLayer,
                    indoorSpace,
                    path,
                    times,
                    robotId, 
                    "CentralControl_Initial");
                
                bool hasConflict = conflicts.Count > 0;
                
                // If conflicts exist, try to resolve them
                if (hasConflict)
                {
                    Debug.LogWarning($"[CentralControl] Robot {robotId} path has {conflicts.Count} conflicts!");
                    
                    List<float> modifiedTimes = times;
                    List<float> modifiedSpeeds = null;
                    string modificationType = null;
                    float modificationValue = 0;
                    bool conflictResolved = false;
                    
                    // 1. Try time shift strategy
                    var (timeShiftSuccess, newTimes, shift) = conflictResolver.ResolveConflictByTimeShift(
                        robotId, path, times, conflicts);
                    
                    if (timeShiftSuccess)
                    {
                        modifiedTimes = newTimes;
                        modificationType = "time_shift";
                        modificationValue = shift;
                        conflictResolved = true;
                        Debug.Log($"[CentralControl] Applied time shift strategy: Robot {robotId}, shift {shift}s");
                    }
                    else
                    {
                        // 2. If time shift failed, try speed adjustment
                        var (speedAdjustSuccess, adjustedTimes, adjustedSpeeds, hasChanges) = 
                            conflictResolver.ResolveConflictBySpeedAdjustment(
                                robotId, path, times, conflicts);
                        
                        if (speedAdjustSuccess)
                        {
                            modifiedTimes = adjustedTimes;
                            modifiedSpeeds = adjustedSpeeds;
                            modificationType = "speed_adjustment";
                            modificationValue = CalculateAverageSpeedChange(adjustedSpeeds);
                            conflictResolved = true;
                            Debug.Log($"[CentralControl] Applied speed adjustment strategy: Robot {robotId}, average adjustment ratio {modificationValue:F2}");
                        }
                        else if (hasChanges)
                        {
                            // Speed adjustment partially successful
                            modifiedTimes = adjustedTimes;
                            modifiedSpeeds = adjustedSpeeds;
                            modificationType = "partial_speed_adjustment";
                            modificationValue = CalculateAverageSpeedChange(adjustedSpeeds);
                            Debug.LogWarning($"[CentralControl] Speed adjustment partially resolved conflicts: Robot {robotId}");
                            // Note: conflictResolved is still false
                        }
                    }
                    
                    // Verify if conflicts are resolved after strategy application
                    if (modifiedTimes != times)
                    {
                        // Use a more specific source identifier
                        Dictionary<string, List<TimeOccupancy>> remainingConflicts = cellSpaceConflictDetector.DetectConflicts(
                            globalOccupancyLayer,
                            indoorSpace,
                            path,
                            modifiedTimes,
                            robotId, 
                            "CentralControl_AfterResolve");
                        
                        // Check if conflicts are truly resolved
                        if (remainingConflicts.Count == 0)
                        {
                            conflictResolved = true;
                            Debug.Log($"[CentralControl] Conflicts successfully resolved: Robot {robotId}, strategy: {modificationType}");
                        }
                        else
                        {
                            conflictResolved = false;
                            Debug.LogWarning($"[CentralControl] After applying {modificationType} strategy, {remainingConflicts.Count} conflicts remain");
                            
                            // Save latest conflicts for recording
                            conflicts = remainingConflicts;
                        }
                    }
                    
                    // Handle the final result
                    if (conflictResolved)
                    {
                        // Update path reservation with modified times
                        ReserveCellSpacePath(robotId, planId, path, modifiedTimes);
                        
                        // Notify robot based on modification type
                        if (modificationType == "time_shift")
                        {
                            NotifyRobotPathModified(robotId, planId, path, modifiedTimes, modificationType, modificationValue);
                        }
                        else if (modificationType == "speed_adjustment" || modificationType == "partial_speed_adjustment")
                        {
                            NotifyRobotPathModifiedWithSpeed(robotId, planId, path, modifiedTimes, modifiedSpeeds, modificationValue);
                        }
                        
                        return false; // Conflict resolved
                    }
                    else
                    {
                        // All strategies failed, record conflict information
                        RecordCellSpaceConflict(robotId, planId, path, times, conflicts);
                        return true; // Conflict not resolved
                    }
                }
                else
                {
                    return false;
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"[CentralControl] Error detecting path conflicts: {ex.Message}\n{ex.StackTrace}");
                return true; // Assume conflict on error
            }
        }
        
        private void ReserveCellSpacePath(string robotId, string planId, List<ConnectionPoint> path, List<float> times)
        {
            globalOccupancyLayer.RecordPathOccupancy(path, times, robotId, planId);
            Debug.Log($"[CentralControl] Reserved path for robot {robotId}, plan {planId}, path length {path.Count}");
        }
        
        private void RecordCellSpaceConflict(string robotId, string planId, List<ConnectionPoint> path, List<float> times, Dictionary<string, List<TimeOccupancy>> conflicts)
        {
            Debug.LogWarning($"[CentralControl] Could not resolve conflicts for robot {robotId}, plan {planId}");
            
            foreach (var kvp in conflicts)
            {
                string cellId = kvp.Key;
                List<TimeOccupancy> cellConflicts = kvp.Value;
                
                Debug.LogWarning($"[CentralControl] Cell {cellId} has {cellConflicts.Count} conflicts:");
                foreach (var conflict in cellConflicts)
                {
                    Debug.LogWarning($"[CentralControl]   With robot {conflict.Occupier.RobotId} at time {conflict.StartTime:F2}-{conflict.EndTime:F2}");
                }
            }
        }
        #endregion
        
        #region Event Handlers
        private void HandleRobotPathPlanned(object sender, RobotPathEventArgs e)
        {
            Debug.Log($"[CentralControl] Handling path planned event for robot {e.RobotId}, plan {e.PlanId}, path length {e.Path.Count}");
            
            if (e.Path == null || e.Times == null || e.Times.Count != e.Path.Count || e.Times.Count != e.Speeds.Count)
            {
                Debug.LogError($"[CentralControl] Invalid path data for robot {e.RobotId}");
                return;
            }

            ReserveCellSpacePath(e.RobotId, e.PlanId, e.Path, e.Times);
            NotifyRobotPathAccepted(e.RobotId, e.PlanId, e.Path, e.Times, e.Speeds);
        }
        
        private void NotifyRobotPathModified(string robotId, string planId, List<ConnectionPoint> path, List<float> newTimes, string modificationType, float modificationValue)
        {
            if (EventManager.Instance != null)
            {
                var args = new RobotPathModificationEventArgs(
                    robotId, planId, path, newTimes, modificationType, modificationValue);
                
                EventManager.Instance.TriggerRobotPathModified(args);
                Debug.Log($"[CentralControl] Notified robot {robotId} of path modification: {modificationType}, value {modificationValue}");
            }
        }
        
        private void NotifyRobotPathModifiedWithSpeed(string robotId, string planId, List<ConnectionPoint> path, List<float> newTimes, List<float> newSpeeds, float modificationValue)
        {
            if (EventManager.Instance != null)
            {
                var args = new RobotPathModificationEventArgs(
                    robotId, planId, path, newTimes, "speed_adjustment", modificationValue);
                
                args.NewSpeeds = newSpeeds;
                
                EventManager.Instance.TriggerRobotPathModified(args);
                Debug.Log($"[CentralControl] Notified robot {robotId} of path modification with speeds: value {modificationValue}");
            }
        }
        

        private void NotifyRobotPathAccepted(string robotId, string planId, List<ConnectionPoint> path, List<float> times, List<float> speeds)
        {
            if (EventManager.Instance != null)
            {
                EventManager.Instance.TriggerRobotPathAccepted(robotId, planId, path, times, speeds);
                Debug.Log($"[CentralControl] Notified robot {robotId} that path {planId} was accepted with {path.Count} points");
            }
        }

        public static void NotifyRobotFree(RobotController robot)
        {
            if (robot == null) return;
            
            OnRobotBecameFree?.Invoke(null, new RobotStatusEventArgs(robot));
            
            var centralController = FindObjectOfType<CentralController>();
            if (centralController != null)
            {
                centralController.HandleRobotBecameFree(new RobotStatusEventArgs(robot));
            }
            else
            {
                Debug.LogWarning($"Robot {robot.Id} became free, but no CentralController instance found to notify.");
            }
        }

        private void HandleRobotBecameFree(RobotStatusEventArgs args)
        {
            Debug.Log($"[CentralControl] Robot {args.Robot.Id} became free");
            
            StartCoroutine(AssignOrderToRobot(args.Robot));
        }

        private IEnumerator AssignOrderToRobot(RobotController robot)
        {
            yield return new WaitForSeconds(0.5f);
            
            if (robot.IsFree && orderManager != null)
            {
                List<Order> unassignedOrders = orderManager.GetAllOrders()
                    .Where(o => o.AssignedRobotId == null)
                    .ToList();
                    
                if (unassignedOrders.Count > 0)
                {
                    OrderAssignmentSelector assigner = new OrderAssignmentSelector(indoorSpace);
                    Order nearestOrder = assigner.FindNearestOrder(robot, unassignedOrders);
                    
                    if (nearestOrder != null)
                    {
                        robot.ReceiveOrder(nearestOrder);
                        nearestOrder.AssignedRobotId = robot.Id;
                        
                        Debug.Log($"[CentralControl] Assigned order {nearestOrder.Id} to free robot {robot.Id}");
                    }
                }
            }
        }

        private void HandleRobotCollision(object sender, CollisionEventArgs e)
        {
            if (e.IsRobotCollision)
            {
                totalRobotCollisions++;
                
                IncrementRobotCollisionCount(e.RobotId1);
                
                IncrementRobotCollisionCount(e.RobotId2);
                
                Debug.Log($"[CentralControl] Detected robot-robot collision: {e.RobotId1} with {e.RobotId2} at time {e.Time:F2}");
            }
            else
            {
                totalEnvironmentCollisions++;
                
                IncrementRobotCollisionCount(e.RobotId1);
                
                Debug.Log($"[CentralControl] Detected robot-environment collision: {e.RobotId1} with {e.RobotId2} at time {e.Time:F2}");
            }
        }

        private void IncrementRobotCollisionCount(string robotId)
        {
            if (!robotCollisionCounts.ContainsKey(robotId))
            {
                robotCollisionCounts[robotId] = 0;
            }
            
            robotCollisionCounts[robotId]++;
        }

        public (int totalRobotCollisions, int totalEnvironmentCollisions, Dictionary<string, int> robotCollisions) GetCollisionStatistics()
        {
            return (totalRobotCollisions, totalEnvironmentCollisions, new Dictionary<string, int>(robotCollisionCounts));
        }

        public int GetRobotCollisionCount(string robotId)
        {
            if (robotCollisionCounts.TryGetValue(robotId, out int count))
            {
                return count;
            }
            return 0;
        }

        public string GenerateCollisionReport()
        {
            System.Text.StringBuilder report = new System.Text.StringBuilder();
            report.AppendLine($"Collision Report - Generated at {DateTime.Now}");
            report.AppendLine($"Simulation Time: {Time.time:F2} seconds");
            report.AppendLine($"Total Robot-Robot Collisions: {totalRobotCollisions}");
            report.AppendLine($"Total Robot-Environment Collisions: {totalEnvironmentCollisions}");
            report.AppendLine("Per Robot Collision Counts:");
            
            foreach (var kvp in robotCollisionCounts.OrderByDescending(x => x.Value))
            {
                report.AppendLine($"  Robot {kvp.Key}: {kvp.Value} collisions");
            }
            
            if (EventManager.Instance != null)
            {
                var allCollisions = EventManager.Instance.GetAllRobotCollisions()
                    .Concat(EventManager.Instance.GetAllEnvironmentCollisions())
                    .OrderBy(c => c.Time);
                
                report.AppendLine("\nDetailed Collision Timeline:");
                foreach (var collision in allCollisions)
                {
                    if (collision.IsRobotCollision)
                    {
                        report.AppendLine($"  Time {collision.Time:F2}s: Robot {collision.RobotId1} collided with Robot {collision.RobotId2}");
                    }
                    else
                    {
                        report.AppendLine($"  Time {collision.Time:F2}s: Robot {collision.RobotId1} collided with Environment {collision.RobotId2}");
                    }
                }
            }
            
            return report.ToString();
        }

        public void ResetCollisionStatistics()
        {
            totalRobotCollisions = 0;
            totalEnvironmentCollisions = 0;
            robotCollisionCounts.Clear();
            
            if (EventManager.Instance != null)
            {
                EventManager.Instance.ClearCollisionRecords();
            }
            
            Debug.Log("[CentralControl] Collision statistics reset");
        }
        

        private void OnPathPlanningMetricsRecorded(object sender, PathPlanningMetricsEventArgs e)
        {
            RecordPathPlanningMetrics(e.Metrics);
        }
        #endregion

        #region Utility Methods
        public string GenerateCollisionReportCSV()
        {
            System.Text.StringBuilder csv = new System.Text.StringBuilder();
            csv.AppendLine("Time,Robot1,Robot2,IsRobotCollision,Position_X,Position_Y,Position_Z,RecoveryResult,RecoveryTime");
            
            if (EventManager.Instance != null)
            {
                var allCollisions = EventManager.Instance.GetAllRobotCollisions()
                    .Concat(EventManager.Instance.GetAllEnvironmentCollisions())
                    .OrderBy(c => c.Time);
                
                foreach (var collision in allCollisions)
                {
                    csv.AppendLine($"{collision.Time:F3},{collision.RobotId1},{collision.RobotId2},{collision.IsRobotCollision},{collision.Position.x:F2},{collision.Position.y:F2},{collision.Position.z:F2},{collision.RecoveryResult},{collision.RecoveryTime:F2}");
                }
            }
            
            return csv.ToString();
        }

        public void SaveCollisionReportToFile()
        {
            string timestamp = System.DateTime.Now.ToString("yyyyMMdd_HHmmss");
            
            try
            {
                string txtReport = GenerateCollisionReport();
                string txtPath = $"{Application.persistentDataPath}/collision_report_{timestamp}.txt";
                System.IO.File.WriteAllText(txtPath, txtReport);
                
                string csvReport = GenerateCollisionReportCSV();
                string csvPath = $"{Application.persistentDataPath}/collision_report_{timestamp}.csv";
                System.IO.File.WriteAllText(csvPath, csvReport);
                
                Debug.Log($"Collision reports saved: {txtPath}, {csvPath}");
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Error saving collision report: {e.Message}");
            }
        }
        
        private IEnumerator AutoSaveCollisionReports(float interval)
        {
            while (true)
            {
                yield return new WaitForSeconds(interval);
                SaveCollisionReportToFile();
            }
        }

        public void RecordPathPlanningMetrics(PathPlanningMetrics metrics)
        {
            string key = $"{metrics.RobotId}_{metrics.PlanId}";
            
            if (pathPlanningMetrics.ContainsKey(key))
            {
                pathPlanningMetrics[key] = metrics;
            }
            else
            {
                pathPlanningMetrics.Add(key, metrics);
            }
            
            Debug.Log($"[CentralControl] Path planning metrics recorded for Robot {metrics.RobotId}, Plan {metrics.PlanId}");
        }

        public string GeneratePathPlanningReport()
        {
            System.Text.StringBuilder report = new System.Text.StringBuilder();
            report.AppendLine($"Path Planning Metrics Report - Generated at {DateTime.Now}");
            report.AppendLine($"Simulation Time: {Time.time:F2} seconds");
            report.AppendLine($"Total Plans Recorded: {pathPlanningMetrics.Count}");
            report.AppendLine();
            
            report.AppendLine("RobotId,PlanId,PlanningTime,ActualPathLength,TravelTime,IdealPathLength,PathEfficiency,ReplanCount,WaitTime,IsReplan,Timestamp");
            
            foreach (var metrics in pathPlanningMetrics.Values.OrderBy(m => m.RobotId).ThenBy(m => m.Timestamp))
            {
                report.AppendLine($"{metrics.RobotId},{metrics.PlanId},{metrics.PlanningTime:F4},{metrics.ActualPathLength:F2},{metrics.TravelTime:F2},{metrics.IdealPathLength:F2},{metrics.PathEfficiency:F3},{metrics.ReplanCount},{metrics.WaitTime:F2},{metrics.IsReplan},{metrics.Timestamp}");
            }
            
            return report.ToString();
        }

        public void SavePathPlanningReportToFile()
        {
            string report = GeneratePathPlanningReport();
            string timestamp = System.DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string filepath = $"{Application.persistentDataPath}/path_planning_metrics_{timestamp}.csv";
            
            try
            {
                System.IO.File.WriteAllText(filepath, report);
                Debug.Log($"Path planning metrics report saved to: {filepath}");
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Error saving path planning metrics report: {e.Message}");
            }
        }

        private IEnumerator AutoSavePathPlanningReports(float interval)
        {
            while (true)
            {
                yield return new WaitForSeconds(interval);
                SavePathPlanningReportToFile();
            }
        }

        private void RebuildNavMesh()
        {
            if (mapLoader != null)
            {
                mapLoader.BuildNavMesh();
                Debug.Log("[CentralControl] NavMesh rebuilt");
            }
        }
        
        private float CalculateAverageSpeedChange(List<float> speedAdjustments)
        {
            if (speedAdjustments == null || speedAdjustments.Count == 0)
                return 0;
                
            return speedAdjustments.Average();
        }
        
        private int ParseFloorLevel(string floorLevel)
        {
            if (string.IsNullOrEmpty(floorLevel)) 
                return 1; // Default to 1st floor
            
            // Handle above-ground floors: "F1" or "1F"
            if (floorLevel.StartsWith("F") || floorLevel.EndsWith("F"))
            {
                string numPart = floorLevel.Replace("F", "");
                if (int.TryParse(numPart, out int level))
                {
                    return level;
                }
            }
            
            // Handle below-ground floors: "B1"
            if (floorLevel.StartsWith("B") || floorLevel.StartsWith("b"))
            {
                string numPart = floorLevel.Substring(1).Replace("F", "").Replace("f", "");
                if (int.TryParse(numPart, out int level))
                {
                    return -level;
                }
            }
            
            // Try simple number parsing
            if (int.TryParse(floorLevel, out int simpleLevel))
            {
                return simpleLevel;
            }
            
            return 1; // If parsing fails, default to 1st floor
        }
        #endregion
    }
}
