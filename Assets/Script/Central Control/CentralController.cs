using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;
using Unity.AI.Navigation;
using NetTopologySuite.Geometries;
using Map;
using CentralControl.OrderAssignment;
using CentralControl.RobotControl;
using CentralControl.PickerControl;
using CentralControl.PickingQueueControl;
using PathPlanning;

namespace CentralControl
{
    public class CentralController : MonoBehaviour
    {
        #region Singleton
        public static CentralController Instance { get; private set; }
        #endregion
        
        #region Inspector Fields
        [Header("Components")]
        public OrderManager orderManager;
        public RobotManager robotManager;
        public PickerManager pickerManager;
        public MapLoader mapLoader;
        public IntersectionAreaManager intersectionAreaManager;
        public DeadlockDetector deadlockDetector;

        [Header("Deadlock Resolution")]
        public float deadlockDecisionTtl = 5f;
        
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
        private CellSpaceGraph cellSpaceGraph;
        private ConflictResolver conflictResolver;
        private CellSpaceConflictDetector cellSpaceConflictDetector;
        private float nextNavMeshRebuildTime = 0f;
        private float nextDynamicUpdateTime = 0f;
        private Coroutine orderGenerationCoroutine;
        private bool isInitialized = false;
        private MultiFloorOrderGenerator orderGenerator;
        private Dictionary<string, List<NavMeshLink>> floorNavMeshLinks = new Dictionary<string, List<NavMeshLink>>();
        public static event EventHandler<RobotStatusEventArgs> OnRobotBecameFree;
        private int totalRobotCollisions = 0;
        private int totalEnvironmentCollisions = 0;
        private Dictionary<string, int> robotCollisionCounts = new Dictionary<string, int>();
        private Dictionary<string, PathPlanningMetrics> pathPlanningMetrics = new Dictionary<string, PathPlanningMetrics>();
        private readonly object deadlockDecisionLock = new object();
        private readonly Dictionary<string, DeadlockDecision> deadlockDecisions = new Dictionary<string, DeadlockDecision>();
        
        // Dirty flag for incremental layer merge

        private readonly HashSet<int> dirtyRobotIds = new HashSet<int>();
        private readonly object dirtyRobotLock = new object();

        // Tracks the active plan ID for each robot, used by AtomicSwapGlobalPath
        private ConcurrentDictionary<string, string> robotActivePlanIds = new ConcurrentDictionary<string, string>();

        private class AcceptedPlanMetadata
        {
            public string PlanId;
            public float AcceptedAt;
            public float? LastModifiedAt;
        }

        private readonly Dictionary<string, AcceptedPlanMetadata> acceptedPlanRegistry = new Dictionary<string, AcceptedPlanMetadata>();

        // Replan storm prevention: per-robot cooldown
        private Dictionary<string, float> lastReplanRequestTime = new Dictionary<string, float>();
        private const float REPLAN_COOLDOWN = 5.0f;

        #endregion
        
        public bool TryGetRobotId(GameObject robotObject, out int robotId)
        {
            robotId = 0;
            if (robotObject == null)
            {
                return false;
            }

            if (robotObject.TryGetComponent<RobotController>(out var robot))
            {
                robotId = robot.Id;
                return true;
            }

            return false;
        }

        public bool IsRobotInPickingQueue(string pickingPointId, int robotId)
        {
            return PickingQueueManager.Instance.IsRobotInQueueOrActive(pickingPointId, robotId.ToString());
        }

        public bool IsRobotPicking(int robotId)
        {
            if (robotManager == null)
            {
                return false;
            }

            var robot = robotManager.GetRobotById(robotId);
            return robot != null && robot.IsPicking;
        }

        public string GetRobotPickingPointId(int robotId)
        {
            if (robotManager == null)
            {
                return null;
            }

            var robot = robotManager.GetRobotById(robotId);
            return robot?.CurrentOrder?.PickingPoint?.Id;
        }

        private class DeadlockDecision
        {
            public string YieldRobotId;
            public float CreatedTime;
            public string[] InvolvedRobotIds;
        }
 
        #region Unity Lifecycle

        private void Awake()
        {
            // Singleton initialization
            if (Instance != null && Instance != this)
            {
                Debug.LogWarning("[CentralControl] Duplicate CentralController instance detected. Destroying this one.");
                Destroy(gameObject);
                return;
            }
            Instance = this;
        }
        
        private void Start()
        {
            // Initialize the system
            StartCoroutine(InitializeSystem());
        }

        private IEnumerator InitializeSystem()
        {
            Debug.Log("[CentralControl] Initializing system...");

            if (mapLoader == null)
            {
                Debug.LogError("[CentralControl] MapLoader not assigned in Inspector!");
                yield break;
            }

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

            // Reset PickingQueueManager singleton state (survives scene reload in Unity)
            PickingQueueManager.Instance.Reset();

            // Initialize robot manager with elevator manager
            InitializeRobotManager();

            // Pass CellSpaceGraph to robots using CellSpaceAStar algorithm
            if (cellSpaceGraph != null)
            {
                var allRobots = robotManager.GetAllRobots();
                if (allRobots != null)
                {
                    foreach (var robot in allRobots)
                    {
                        if (robot.selectedAlgorithm == SearchAlgorithm.CellSpaceAStar)
                        {
                            robot.GetPathPlanner().SetCellSpaceGraph(cellSpaceGraph);
                        }
                    }
                }
            }

            // Mount experiment metrics collector only if not already managed by ExperimentRunner
            if (GetComponentInParent<ExperimentRunner>() == null &&
                FindObjectOfType<ExperimentRunner>() == null &&
                GetComponent<ExperimentMetricsCollector>() == null)
            {
                gameObject.AddComponent<ExperimentMetricsCollector>();
            }

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
                EventManager.Instance.OnPathPlanningMetricsRecorded -= OnPathPlanningMetricsRecorded;
                EventManager.Instance.OnPathPlanningMetricsRecorded += OnPathPlanningMetricsRecorded;
            }

            // Start auto-saving collision reports
            StartCoroutine(AutoSaveCollisionReports(60f));
            StartCoroutine(AutoSavePathPlanningReports(60f));
        }
        
        void Update()
        {
            SafeEventInvoker.ProcessMainThreadQueue();

            if (!isInitialized) return;

            // Update dynamic occupancy layer (throttled to avoid per-frame overhead)
            if (Time.time >= nextDynamicUpdateTime)
            {
                UpdateDynamicOccupancyLayer();
                nextDynamicUpdateTime = Time.time + dynamicUpdateInterval;
            }

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
                EventManager.Instance.OnDeadlockDetected -= HandleDeadlockDetected;
                EventManager.Instance.OnRequestGlobalMapUpdate -= HandleGlobalMapUpdateRequest;
                EventManager.Instance.OnPathPlanningMetricsRecorded -= OnPathPlanningMetricsRecorded;
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

                // Build CellSpace-level graph for baseline experiments
                cellSpaceGraph = new CellSpaceGraph(indoorSpace);

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

                // Note: HandleRobotPathPlanned is subscribed via the global EventManager event
                // in InitializeConflictDetector() to avoid double subscriptions.

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
                EventManager.Instance.OnDeadlockDetected -= HandleDeadlockDetected;

                EventManager.Instance.OnRobotCollisionDetected += HandleRobotCollision;
                EventManager.Instance.OnDeadlockDetected += HandleDeadlockDetected;
                EventManager.Instance.OnRequestGlobalMapUpdate -= HandleGlobalMapUpdateRequest;
                EventManager.Instance.OnRequestGlobalMapUpdate += HandleGlobalMapUpdateRequest;

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
            
            // Get unassigned orders (GetAllOrders already filters by Pending status)
            List<Order> orders = orderManager.GetAllOrders();

            if (orders.Count == 0)
            {
                Debug.Log("[CentralControl] No unassigned orders available for distribution");
                return;
            }

            Debug.Log($"[CentralControl] Found {freeRobots.Count} free robots and {orders.Count} unassigned orders");

            // Create order assigner with indoor space and order manager for transactional assignment
            OrderAssignmentSelector assigner = new OrderAssignmentSelector(indoorSpace, orderManager);

            // Initial distribution (one order per robot)
            assigner.InitialOrderDistribution(orders, freeRobots);

            // After initial distribution, distribute remaining orders using clustering
            orders = orderManager.GetAllOrders();
            
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

        private void HandleGlobalMapUpdateRequest(object sender, GlobalMapUpdateRequestEventArgs e)
        {
            if (globalOccupancyLayer == null || robotManager == null) return;

            if (e.RobotIds != null && e.RobotIds.Length > 0)
            {
                foreach (int robotId in e.RobotIds)
                {
                    var robot = robotManager.GetRobotById(robotId);
                    if (robot == null) continue;
                    var personalLayer = robot.GetPersonalOccupancyLayer();
                    if (personalLayer != null)
                    {
                        globalOccupancyLayer.MergeLayer(personalLayer, Time.time - windowDuration, Time.time + 60, robotId.ToString());
                    }
                }
                Debug.Log($"[CentralControl] Global occupancy layer updated for robots: {string.Join(", ", e.RobotIds)}");
            }
            else
            {
                var allRobots = robotManager.GetAllRobots();
                foreach (var robot in allRobots)
                {
                    var personalLayer = robot.GetPersonalOccupancyLayer();
                    if (personalLayer != null)
                    {
                        globalOccupancyLayer.MergeLayer(personalLayer, Time.time - windowDuration, Time.time + 60, robot.Id.ToString());
                    }
                }
                Debug.Log("[CentralControl] Global occupancy layer fully rebuilt after deadlock recovery");
            }
        }

        private void MergeRobotOccupancyLayer()
        {
            // Only merge dirty robots instead of all robots every frame
            HashSet<int> toProcess;
            lock (dirtyRobotLock)
            {
                if (dirtyRobotIds.Count == 0) return;
                toProcess = new HashSet<int>(dirtyRobotIds);
                dirtyRobotIds.Clear();
            }
            
            foreach (var robotId in toProcess)
            {
                var robot = robotManager.GetRobotById(robotId);
                if (robot == null) continue;
                
                var personalLayer = robot.GetPersonalOccupancyLayer();
                if (personalLayer != null)
                {
                    globalOccupancyLayer.MergeLayer(personalLayer, Time.time - windowDuration, Time.time + 60, robotId.ToString());
                }
            }
        }
        
        /// <summary>
        /// Mark a robot's personal layer as dirty, requiring merge on next update.
        /// Called by RobotController when personal layer is modified.
        /// </summary>
        public void MarkRobotLayerDirty(int robotId)
        {
            lock (dirtyRobotLock)
            {
                dirtyRobotIds.Add(robotId);
            }
        }

        /// <summary>
        /// Immediately merges a specific robot's personal layer into the global layer.
        /// Used for time-critical events (checkpoint drift, accept, emergency stop, etc.)
        /// that cannot wait for the 0.5s periodic merge.
        /// </summary>
        public void FlushRobotLayer(int robotId)
        {
            var robot = robotManager?.GetRobotById(robotId);
            if (robot == null || globalOccupancyLayer == null) return;

            var personalLayer = robot.GetPersonalOccupancyLayer();
            if (personalLayer != null)
            {
                globalOccupancyLayer.MergeLayer(personalLayer, Time.time - windowDuration, Time.time + 60, robotId.ToString());
            }

            // Remove from dirty set since we just flushed
            lock (dirtyRobotLock)
            {
                dirtyRobotIds.Remove(robotId);
            }
        }

        public DynamicOccupancyLayer GetGlobalOccupancyLayer()
        {
            return globalOccupancyLayer;
        }

        public CellSpaceGraph GetCellSpaceGraph()
        {
            return cellSpaceGraph;
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
                EventManager.Instance.OnRobotPathPlanned -= HandleRobotPathPlanned;
                EventManager.Instance.OnRobotPathPlanned += HandleRobotPathPlanned;
            }
        }

        private void HandleDeadlockDetected(object sender, DeadlockDetectedEventArgs e)
        {
            if (robotManager == null || e == null || e.InvolvedRobotIds == null || e.InvolvedRobotIds.Length == 0)
            {
                return;
            }

            float now = Time.time;
            string groupKey = GetDeadlockGroupKey(e.InvolvedRobotIds);

            string yieldRobotId;
            lock (deadlockDecisionLock)
            {
                if (deadlockDecisions.TryGetValue(groupKey, out var existingDecision) &&
                    now - existingDecision.CreatedTime <= deadlockDecisionTtl)
                {
                    yieldRobotId = existingDecision.YieldRobotId;
                }
                else
                {
                    yieldRobotId = SelectYieldRobot(e.InvolvedRobotIds);
                    deadlockDecisions[groupKey] = new DeadlockDecision
                    {
                        YieldRobotId = yieldRobotId,
                        CreatedTime = now,
                        InvolvedRobotIds = e.InvolvedRobotIds
                    };
                }
            }

            foreach (var robotId in e.InvolvedRobotIds)
            {
                // Replan cooldown: skip if this robot recently received a replan request
                if (lastReplanRequestTime.TryGetValue(robotId, out float lastTime) && Time.time - lastTime < REPLAN_COOLDOWN)
                {
                    Debug.Log($"[CentralControl] Replan cooldown active for Robot {robotId}, skipping deadlock replan ({Time.time - lastTime:F1}s < {REPLAN_COOLDOWN}s)");
                    continue;
                }

                float delay = GetDeadlockStaggerDelay(e.InvolvedRobotIds, robotId);
                if (robotId == yieldRobotId)
                {
                    EventManager.Instance?.TriggerReplanRequested(robotId, "DeadlockYield", delay);
                }
                else
                {
                    EventManager.Instance?.TriggerReplanRequested(robotId, "DeadlockResolution", delay);
                }
                lastReplanRequestTime[robotId] = Time.time;
            }
        }

        private string GetDeadlockGroupKey(string[] involvedRobotIds)
        {
            var ordered = involvedRobotIds.OrderBy(id => id).ToArray();
            return string.Join("_", ordered);
        }

        private float GetDeadlockStaggerDelay(string[] involvedRobotIds, string robotId)
        {
            int index = Array.IndexOf(involvedRobotIds, robotId);
            if (index < 0)
            {
                return 0f;
            }

            return index * 0.5f;
        }

        private string SelectYieldRobot(string[] involvedRobotIds)
        {
            string bestId = null;
            float bestCost = float.MaxValue;

            foreach (var robotId in involvedRobotIds)
            {
                var robot = robotManager.GetRobotById(robotId);
                if (robot == null)
                {
                    continue;
                }

                float cost = EstimateYieldCost(robot);
                if (cost < bestCost)
                {
                    bestCost = cost;
                    bestId = robotId;
                }
            }

            return bestId ?? involvedRobotIds.FirstOrDefault();
        }

        private float EstimateYieldCost(RobotController robot)
        {
            float cost = 0f;
            cost += robot.GetRobotOrdersQueueCount * 2f;

            if (robot.CurrentOrder != null)
            {
                cost += 1f;
            }

            if (robot.IsBackingUp)
            {
                cost += 3f;
            }

            if (!robot.IsMoving)
            {
                cost += 2f;
            }

            return cost;
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
                    "CentralControl_Initial",
                    excludeTypes: new[] { OccupancyType.Intent });
                
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
                            "CentralControl_AfterResolve",
                            excludeTypes: new[] { OccupancyType.Intent });
                        
                        // Check if conflicts are truly resolved
                        if (remainingConflicts.Count == 0)
                        {
                            // Unit 12B: Reject partial_speed_adjustment — do not send incomplete
                            // modifications to the robot. Fall through to replan instead.
                            if (modificationType == "partial_speed_adjustment")
                            {
                                conflictResolved = false;
                                Debug.LogWarning($"[CentralControl] Rejecting partial_speed_adjustment for Robot {robotId} — falling through to replan");
                            }
                            else
                            {
                                conflictResolved = true;
                                Debug.Log($"[CentralControl] Conflicts successfully resolved: Robot {robotId}, strategy: {modificationType}");
                            }
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
                        // Occupancy reservation handled by robot personal layer + merge flush.
                        
                        // Notify robot based on modification type
                        if (modificationType == "time_shift")
                        {
                            NotifyRobotPathModified(robotId, planId, path, modifiedTimes, modificationType, modificationValue);
                        }
                        else if (modificationType == "speed_adjustment")
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
        
        [System.Obsolete("Use AtomicSwapGlobalPath instead for safe plan replacement.")]
        private void ReserveCellSpacePath(string robotId, string planId, List<ConnectionPoint> path, List<float> times)
        {
            globalOccupancyLayer.RecordPathOccupancy(path, times, robotId, planId);
            Debug.Log($"[CentralControl] Reserved path for robot {robotId}, plan {planId}, path length {path.Count}");
        }

        private void AtomicSwapGlobalPath(string robotId, string newPlanId,
            List<ConnectionPoint> path, List<float> times)
        {
            string oldPlanId = robotActivePlanIds.TryGetValue(robotId, out var id) ? id : null;

            // Convert ConnectionPoint path to cell occupancies
            var cellIds = new List<string>();
            var startTimes = new List<float>();
            var endTimes = new List<float>();

            for (int i = 0; i < path.Count; i++)
            {
                CellSpace cell = indoorSpace.GetCellSpaceFromConnectionPoint(path[i], true);
                if (cell != null && (cellIds.Count == 0 || cellIds[cellIds.Count - 1] != cell.Id))
                {
                    cellIds.Add(cell.Id);
                    startTimes.Add(times[i]);
                    endTimes.Add(i + 1 < times.Count ? times[i + 1] : times[i] + 5f);
                }
            }

            // Atomic swap on global layer (single lock)
            bool swapOk = globalOccupancyLayer.AtomicSwapPath(robotId, oldPlanId, cellIds, startTimes, endTimes, newPlanId);

            if (swapOk)
            {
                robotActivePlanIds[robotId] = newPlanId;
                Debug.Log($"[CentralControl] AtomicSwap for robot {robotId}: {oldPlanId} -> {newPlanId}");
            }
            else
            {
                Debug.LogWarning($"[CentralControl] AtomicSwap conflict for robot {robotId}, plan {newPlanId} rejected");
            }
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

            // Baseline robots skip conflict detection and occupancy management
            var robot = robotManager?.GetRobotById(e.RobotId);
            if (robot != null && robot.IsBaselineMode)
            {
                NotifyRobotPathAccepted(e.RobotId, e.PlanId, e.Path, e.Times, e.Speeds);
                return;
            }

            // First check if any conflicts exist at all (lightweight check).
            // If no conflicts, skip the full resolution pipeline and accept directly.
            if (cellSpaceConflictDetector != null && globalOccupancyLayer != null && indoorSpace != null)
            {
                Dictionary<string, List<TimeOccupancy>> initialConflicts = cellSpaceConflictDetector.DetectConflicts(
                    globalOccupancyLayer, indoorSpace, e.Path, e.Times, e.RobotId, "CentralControl_PreCheck",
                    excludeTypes: new[] { OccupancyType.Intent });

                if (initialConflicts.Count > 0)
                {
                    // Conflicts exist — run full detection and resolution pipeline.
                    // DetectPathConflicts will reserve the modified path and send
                    // modification notifications if conflicts are resolved.
                    bool unresolvedConflicts = DetectPathConflicts(e.RobotId, e.Path, e.Times, e.PlanId);

                    if (unresolvedConflicts)
                    {
                        // All resolution strategies failed — request replan
                        // Replan cooldown: skip if this robot recently received a replan request
                        if (lastReplanRequestTime.TryGetValue(e.RobotId, out float lastReplanTime) && Time.time - lastReplanTime < REPLAN_COOLDOWN)
                        {
                            Debug.Log($"[CentralControl] Replan cooldown active for Robot {e.RobotId}, skipping conflict replan ({Time.time - lastReplanTime:F1}s < {REPLAN_COOLDOWN}s)");
                        }
                        else
                        {
                            Debug.LogWarning($"[CentralControl] Robot {e.RobotId} path has unresolvable conflicts, requesting replan");
                            EventManager.Instance?.TriggerReplanRequested(e.RobotId, "UnresolvedPathConflict");
                            lastReplanRequestTime[e.RobotId] = Time.time;
                        }
                    }
                    // else: DetectPathConflicts already reserved the modified path and
                    // notified the robot of the modification. No further action needed.
                    return;
                }
            }

            // No conflicts detected — accept path directly.
            // Occupancy reservation is handled by the robot's personal layer + merge flush,
            // not by CentralController writing to global layer (single truth source principle).
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

            if (acceptedPlanRegistry.TryGetValue(robotId, out var meta))
            {
                meta.LastModifiedAt = Time.time;
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

            if (acceptedPlanRegistry.TryGetValue(robotId, out var meta))
            {
                meta.LastModifiedAt = Time.time;
            }
        }
        

        private void NotifyRobotPathAccepted(string robotId, string planId, List<ConnectionPoint> path, List<float> times, List<float> speeds)
        {
            if (EventManager.Instance != null)
            {
                EventManager.Instance.TriggerRobotPathAccepted(robotId, planId, path, times, speeds);
                Debug.Log($"[CentralControl] Notified robot {robotId} that path {planId} was accepted with {path.Count} points");
            }

            acceptedPlanRegistry[robotId] = new AcceptedPlanMetadata
            {
                PlanId = planId,
                AcceptedAt = Time.time,
                LastModifiedAt = null
            };
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
                List<Order> unassignedOrders = orderManager.GetAllOrders();

                if (unassignedOrders.Count > 0)
                {
                    OrderAssignmentSelector assigner = new OrderAssignmentSelector(indoorSpace, orderManager);
                    Order bestOrder = assigner.FindBestOrder(robot, unassignedOrders);

                    if (bestOrder != null)
                    {
                        if (orderManager.TryAssignOrderToRobot(bestOrder, robot))
                        {
                            Debug.Log($"[CentralControl] Assigned order {bestOrder.Id} to free robot {robot.Id}");
                        }
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
            return Map.FloorLevelUtils.Parse(floorLevel, defaultValue: 1);
        }
        #endregion
    }
}
