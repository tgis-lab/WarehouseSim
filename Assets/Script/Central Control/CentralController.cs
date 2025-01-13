using System;
using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System.Threading.Tasks;
using NetTopologySuite.Geometries;
using UnityEngine;
using Map;
using CentralControl.OrderAssignment;
using CentralControl.RobotControl;

namespace CentralControl
{
    public class CentralController : MonoBehaviour
    {
        public OrderManager orderManager;
        public RobotManager robotManager;
        public IndoorSpace indoorSpace;
        public MapLoader mapLoader;
        public IntersectionAreaManager intersectionAreaManager;
        public float width, length;
        public Graph graph;
        public float interval = 5f;
        public int orderCount = 10;
        public float correctionFactor = 0.1f;
        public float dynamicUpdateInterval = 0.5f;
        public float windowDuration = 300f;
        public float timeStep = 0.1f;

        private OrderAssignmentSelector orderAssignmentSelector;
        private bool isInitialDispatchDone = false;
        private bool generatingOrders;
        private List<Order> pendingOrders = new List<Order>();
        private List<Order> unassignedOrders = new List<Order>();
        private readonly object dispatchLock = new object();
        private Coroutine systemCheckCoroutine;

        private DynamicOccupancyLayer globalOccupancyLayer;
        private float lastUpdateTime;

        public static event EventHandler<RobotStatusEventArgs> OnRobotBecameFree;

        private void Start()
        {
            EventManager.Instance.OnMapLoaded += OnMapLoaded;
            EventManager.Instance.OnRobotsInitialized += OnRobotsInitialized;
            orderAssignmentSelector = new OrderAssignmentSelector();
            OnRobotBecameFree += HandleRobotBecameFree;
        }

        private void OnMapLoaded()
        {
            Debug.Log("Map loaded, initializing global occupancy layer...");
            InitializeGlobalOccupancyLayer();
            if (globalOccupancyLayer != null)
            {
                Debug.Log("Global occupancy layer initialized, initializing intersections...");
                intersectionAreaManager.InitializeIntersectionAreas(mapLoader.indoorSpace);
                
                Debug.Log("Map loaded, initializing robots...");
                robotManager.InitializeRobots(mapLoader.indoorSpace, mapLoader.GetGraph(), mapLoader.GetMapGrid(), globalOccupancyLayer, intersectionAreaManager);
            }
            else
            {
                Debug.LogError("Failed to initialize global occupancy layer.");
            }
        }

        private void OnRobotsInitialized()
        {
            Debug.Log("Robots initialized, starting order generation...");
            StartCoroutine(StartGeneratingOrders());
            StartCoroutine(PeriodicSystemCheck());
            StartCoroutine(UpdateGlobalOccupancyLayerRoutine());
        }

        private void OnDestroy()
        {
            EventManager.Instance.OnMapLoaded -= OnMapLoaded;
            EventManager.Instance.OnRobotsInitialized -= OnRobotsInitialized;
        }

        private void OnDisable()
        {
            OnRobotBecameFree -= HandleRobotBecameFree;
            if (systemCheckCoroutine != null)
            {
                StopCoroutine(systemCheckCoroutine);
            }
        }

        # region order generation module
        IEnumerator StartGeneratingOrders()
        {
            generatingOrders = true;
            while (generatingOrders)
            {
                Debug.Log("Starting to generate orders...");
                GenerateRandomOrders();
                DispatchOrders();
                yield return new WaitForSeconds(interval);
            }
        }

        private void GenerateRandomOrders()
        {
            pendingOrders.Clear();

            for (int i = 0; i < orderCount; i++)
            {
                Order newOrder = CreateRandomOrder();
                if (newOrder != null)
                {
                    orderManager.AddOrder(newOrder);
                    pendingOrders.Add(newOrder);
                }
            }
        }

        private Order CreateRandomOrder()
        {
            string id = System.Guid.NewGuid().ToString();
            IndoorSpace indoorSpace = mapLoader.indoorSpace;
            int randomIndex = UnityEngine.Random.Range(0, indoorSpace.BusinessPoints.Count);
            CellSpace cellSpace = indoorSpace.BusinessPoints[randomIndex];
            Vector3 position = new Vector3((float)cellSpace.Space.Centroid.X, 0, (float)cellSpace.Space.Centroid.Y);

            float executionTime = UnityEngine.Random.Range(0f, 30f) * correctionFactor;
            PickingPoint pickingPointCellSpace = indoorSpace.GetPickingPointFromBusinessPoint(cellSpace);

            if (pickingPointCellSpace == null)
            {
                Debug.LogError($"Failed to get picking point for cell space {cellSpace.Id}");
                return null;
            }

            Order newOrder = new Order(id, position, pickingPointCellSpace, executionTime);
            // Debug.Log($"Generated order {id} at {cellSpace.Id} with execution time {executionTime}");
            return newOrder;
        }
        # endregion

        # region order dispatch module
        private void DispatchOrders()
        {
            lock (dispatchLock)
            {
                List<Order> ordersToAssign = new List<Order>(pendingOrders);
                ordersToAssign.AddRange(unassignedOrders);
                List<RobotController> robotsToAssign = isInitialDispatchDone 
                    ? robotManager.GetAvailableRobots() 
                    : robotManager.GetAllRobots();

                if (!isInitialDispatchDone)
                {
                    orderAssignmentSelector.InitialOrderDistribution(ordersToAssign, robotsToAssign);
                    isInitialDispatchDone = true;
                }
                else
                {
                    orderAssignmentSelector.AssignOrders(ordersToAssign, robotsToAssign);
                }

                int assignedCount = ordersToAssign.Count(o => o.AssignedRobotId != null);
                unassignedOrders = ordersToAssign.Where(o => o.AssignedRobotId == null).ToList();
                pendingOrders.Clear();

                Debug.Log($"Successfully assigned {assignedCount} orders. Unassigned orders: {unassignedOrders.Count}");
            }
        }

        public static void NotifyRobotFree(RobotController robot)
        {
            OnRobotBecameFree?.Invoke(null, new RobotStatusEventArgs(robot));
        }

        private void HandleRobotBecameFree(object sender, RobotStatusEventArgs e)
        {
            Debug.Log($"Robot {e.Robot.Id} became free. Attempting to assign unassigned orders.");
            AssignOrdersToFreeRobot(e.Robot);
        }

        private void AssignOrdersToFreeRobot(RobotController freeRobot)
        {
            lock (dispatchLock)
            {
                if (unassignedOrders.Count > 0)
                {
                    List<RobotController> robots = new List<RobotController> { freeRobot };
                    List<Order> ordersToAssign = new List<Order>(unassignedOrders);

                    orderAssignmentSelector.AssignOrders(ordersToAssign, robots);

                    int assignedCount = ordersToAssign.Count(o => o.AssignedRobotId == freeRobot.Id);
                    unassignedOrders = ordersToAssign.Where(o => o.AssignedRobotId == null).ToList();

                    if (assignedCount > 0)
                    {
                        Debug.Log($"{assignedCount} unassigned order(s) assigned to newly freed robot {freeRobot.Id}");
                    }
                    else
                    {
                        Debug.Log($"No unassigned orders could be assigned to newly freed robot {freeRobot.Id}");
                    }
                }
                else
                {
                    Debug.Log($"No unassigned orders for newly freed robot {freeRobot.Id}");
                }
            }
        }
        # endregion

        #region global occupancy layer module
        private void InitializeGlobalOccupancyLayer()
        {
            globalOccupancyLayer = new DynamicOccupancyLayer(
                mapLoader.indoorSpace,
                windowDuration,
                Time.time
            );
        }

        private IEnumerator UpdateGlobalOccupancyLayerRoutine()
        {
            while (true)
            {
                yield return new WaitForSeconds(dynamicUpdateInterval);
                UpdateGlobalOccupancyLayer();
            }
        }

        private void UpdateGlobalOccupancyLayer()
        {
            float currentTime = Time.time;
            float windowStart = currentTime - windowDuration;

            CleanupGlobalOccupancyLayer(windowStart);
            
            MergeRobotOccupancyLayer(robotManager);

            foreach (var robot in robotManager.GetAllRobots())
            {
                robot.UpdateGlobalLayer(globalOccupancyLayer);
            }
        }

        private void CleanupGlobalOccupancyLayer(float cleanupBeforeTime)
        {
            globalOccupancyLayer.ClearOccupancyInTimeWindow(
                float.MinValue,
                cleanupBeforeTime,
                null
            );
        }

        private void MergeRobotOccupancyLayer(RobotManager robots)
        {
            foreach (var robot in robots.GetAllRobots())
            {
                var robotLayer = robot.GetPersonalOccupancyLayer();
                globalOccupancyLayer.MergeLayer(
                    robotLayer,
                    lastUpdateTime,
                    Time.time
                );
            }
        }

        public DynamicOccupancyLayer GetGlobalOccupancyLayer()
        {
            return globalOccupancyLayer;
        }
        # endregion

        # region system check module
        private IEnumerator PeriodicSystemCheck()
        {
            while (true)
            {
                yield return new WaitForSeconds(30f);
                PerformSystemCheck();
            }
        }

        private void PerformSystemCheck()
        {
            Debug.Log($"System check - Unassigned orders: {unassignedOrders.Count}, Pending orders: {pendingOrders.Count}, Total robots: {robotManager.TotalRobots}, Busy robots: {robotManager.BusyRobots}");

            if (unassignedOrders.Count > 0 || pendingOrders.Count > 0)
            {
                Debug.Log("Attempting to dispatch orders.");
                DispatchOrders();
            }

            CheckRobotStatus();

            if (pendingOrders.Count == 0 && unassignedOrders.Count == 0)
            {
                Debug.Log("No pending or unassigned orders. Generating new orders.");
                GenerateRandomOrders();
                DispatchOrders();
            }
        }

        private void CheckRobotStatus()
        {
            foreach (var robot in robotManager.GetAllRobots())
            {
                if (robot.IsFree && !robot.IsProcessingOrders)
                {
                    Debug.Log($"Robot {robot.Id} is free but not processing orders. Notifying as free.");
                    NotifyRobotFree(robot);
                }
            }
        }
        # endregion
    }
}