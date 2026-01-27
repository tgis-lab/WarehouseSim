using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using UnityEngine;
using NetTopologySuite.Geometries;
using Map;

namespace CentralControl.RobotControl
{
    public class RobotManager : MonoBehaviour
    {
        public GameObject robotPrefab;
        public int numberOfRobots = 10;
        public Transform targetIndicatorPrefab;
        
        [Header("Floor Distribution")]
        [Tooltip("Distribute robots evenly")]
        public bool distributeEvenly = true;
        
        [Tooltip("Custom distribution of robots on floors")]
        public List<RobotFloorDistribution> customRobotFloorDistributions = new List<RobotFloorDistribution>();
        
        [Tooltip("Floor height")]
        public float floorHeight = 3.0f;
        
        [Tooltip("base height")]
        public float baseHeight = 0.05f;

        private IndoorSpace indoorSpace;
        private Graph graph;
        private MapGrid mapGrid;
        private DynamicOccupancyLayer globalOccupancyLayer;
        private IntersectionAreaManager intersectionAreaManager;
        private List<RobotController> robots = new List<RobotController>();
        private Dictionary<int, RobotController> robotsById = new Dictionary<int, RobotController>();
        private Dictionary<string, HashSet<CellSpace>> usedSpawnPointsByFloor = new Dictionary<string, HashSet<CellSpace>>();

        public int TotalRobots => robots.Count;
        public int BusyRobots => robots.Count(r => !r.IsFree);

        void Update()
        {
            CheckRobotStatus();
        }

        public void InitializeRobots(
            IndoorSpace indoorSpace, 
            Graph graph, 
            MapGrid mapGrid, 
            DynamicOccupancyLayer globalOccupancyLayer, 
            IntersectionAreaManager intersectionAreaManager)
        {
            this.indoorSpace = indoorSpace;
            this.graph = graph;
            this.mapGrid = mapGrid;
            this.globalOccupancyLayer = globalOccupancyLayer;
            this.intersectionAreaManager = intersectionAreaManager;

            List<string> availableFloors = indoorSpace.GetAllFloorLevels();
            
            if (availableFloors.Count == 0)
            {
                Debug.LogError("Can not find any available Floor to place Robots.");
                return;
            }

            Dictionary<string, int> robotsPerFloor;
            
            if (distributeEvenly)
            {
                robotsPerFloor = DistributeRobotsEvenly(numberOfRobots, availableFloors);
            }
            else
            {
                robotsPerFloor = DistributeRobotsByCustomRatio(numberOfRobots, availableFloors);
            }

            int robotId = 1;
            
            foreach (var entry in robotsPerFloor)
            {
                string floor = entry.Key;
                int count = entry.Value;

                for (int i = 0; i < count; i++)
                {
                    Vector3 initialPosition = GetValidInitialPosition(floor);
                    if (initialPosition != Vector3.zero)
                    {
                        CreateRobot(robotId, initialPosition);
                        robotId++;
                    }
                    else
                    {
                        Debug.LogWarning($"Cannot find valid position to place Robot {robotId} at Floor {floor}.");
                    }
                }
            }

            EventManager.Instance?.TriggerRobotsInitialized();
        }
        
        private Dictionary<string, int> DistributeRobotsEvenly(int totalRobots, List<string> floors)
        {
            Dictionary<string, int> distribution = new Dictionary<string, int>();
            int robotsPerFloor = totalRobots / floors.Count;
            int remainingRobots = totalRobots % floors.Count;
            
            foreach (string floor in floors)
            {
                distribution[floor] = robotsPerFloor;
                
                if (remainingRobots > 0)
                {
                    distribution[floor]++;
                    remainingRobots--;
                }
            }
            
            return distribution;
        }
        
        private Dictionary<string, int> DistributeRobotsByCustomRatio(int totalRobots, List<string> floors)
        {
            Dictionary<string, float> ratios = new Dictionary<string, float>();
            
            if (customRobotFloorDistributions != null && customRobotFloorDistributions.Count > 0)
            {
                foreach (var dist in customRobotFloorDistributions)
                {
                    if (floors.Contains(dist.floorLevel))
                    {
                        ratios[dist.floorLevel] = dist.ratio;
                    }
                }
            }
            
            float configuredRatioSum = ratios.Values.Sum();
            float remainingRatio = Mathf.Max(0, 1.0f - configuredRatioSum);
            int unconfiguredFloors = floors.Count - ratios.Count;
            
            if (unconfiguredFloors > 0 && remainingRatio > 0)
            {
                float ratioPerFloor = remainingRatio / unconfiguredFloors;
                foreach (string floor in floors)
                {
                    if (!ratios.ContainsKey(floor))
                    {
                        ratios[floor] = ratioPerFloor;
                    }
                }
            }
            else if (configuredRatioSum <= 0)
            {
                foreach (string floor in floors)
                {
                    ratios[floor] = 1.0f / floors.Count;
                }
            }
            
            Dictionary<string, int> distribution = new Dictionary<string, int>();
            int assignedRobots = 0;
            
            foreach (var entry in ratios)
            {
                int robotsOnFloor = (int)Math.Floor(totalRobots * entry.Value);
                distribution[entry.Key] = robotsOnFloor;
                assignedRobots += robotsOnFloor;
            }
            
            int remainingRobots = totalRobots - assignedRobots;
            
            if (remainingRobots > 0)
            {
                var sortedFloors = ratios.OrderByDescending(kv => kv.Value).Select(kv => kv.Key).ToList();
                foreach (string floor in sortedFloors)
                {
                    if (remainingRobots <= 0) break;
                    
                    distribution[floor]++;
                    remainingRobots--;
                }
            }
            
            return distribution;
        }
        
        private Vector3 GetValidInitialPosition(string floorLevel)
        {
            if (!usedSpawnPointsByFloor.ContainsKey(floorLevel))
            {
                usedSpawnPointsByFloor[floorLevel] = new HashSet<CellSpace>();
            }
            
            List<CellSpace> spawnPoints = indoorSpace.GetCellSpaceFromLayerTypeAndFloor("aisle", floorLevel);
            
            if (spawnPoints == null || spawnPoints.Count == 0)
            {
                spawnPoints = indoorSpace.GetNavigableCellSpacesOnFloor(floorLevel);
                
                if (spawnPoints == null || spawnPoints.Count == 0)
                {
                    return Vector3.zero;
                }
            }
            
            List<CellSpace> availableSpawnPoints = spawnPoints
                .Where(sp => !usedSpawnPointsByFloor[floorLevel].Contains(sp))
                .ToList();
            
            if (availableSpawnPoints.Count == 0)
            {
                usedSpawnPointsByFloor[floorLevel].Clear();
                availableSpawnPoints = spawnPoints;
            }
            
            CellSpace selectedCellSpace = availableSpawnPoints[UnityEngine.Random.Range(0, availableSpawnPoints.Count)];
            
            usedSpawnPointsByFloor[floorLevel].Add(selectedCellSpace);
            
            Point cellSpaceCenterPoint = (Point)selectedCellSpace.Node;
            
            float y = GetFloorYPosition(floorLevel);
            
            Vector3 cellSpacePosition = new Vector3(
                (float)cellSpaceCenterPoint.X, 
                y, 
                (float)cellSpaceCenterPoint.Y
            );
            return cellSpacePosition;
        }
        
        private float GetFloorYPosition(string floorLevel)
        {
            int level = ParseFloorLevel(floorLevel);
            return ((level - 1) * floorHeight) + baseHeight;
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

        public void ResetSpawnPoints()
        {
            usedSpawnPointsByFloor.Clear();
        }

        private void CreateRobot(int id, Vector3 position)
        {
            GameObject robotObj = Instantiate(robotPrefab, position, Quaternion.identity);
            robotObj.tag = "Robot";
            robotObj.name = $"Robot_{id}";
            robotObj.layer = LayerMask.NameToLayer("RobotLayer");
            RobotController robot = robotObj.GetComponent<RobotController>();
            if (robot != null)
            {
                if (robotObj.GetComponent<Collider>() == null)
                {
                    BoxCollider collider = robotObj.AddComponent<BoxCollider>();

                    collider.size = new Vector3(2.5f, 0.8f, 5.4f); // AGV size
                    collider.center = new Vector3(0, 0.1f, 0.05f); // Adjust collider position
                }

                if (robotObj.GetComponent<Rigidbody>() == null)
                {
                    Rigidbody rb = robotObj.AddComponent<Rigidbody>();
                    rb.isKinematic = false;
                    rb.useGravity = false;
                    rb.constraints = RigidbodyConstraints.FreezeRotation;
                    rb.mass = 50f; // AGV mass
                    rb.drag = 1f; // Add some drag to slow down the AGV
                    rb.angularDrag = 0.05f;
                    rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
                }
                
                robot.InitializeRobot(id, indoorSpace, graph, mapGrid, globalOccupancyLayer, 
                    intersectionAreaManager);

                Transform targetIndicatorInstance = Instantiate(targetIndicatorPrefab);
                robot.targetIndicator = targetIndicatorInstance;

                robots.Add(robot);
                robotsById[id] = robot;
                Debug.Log($"Create Robot {id} at {position}");
            }
            else
            {
                Debug.LogError($"Cannot get Robot {id}'s RobotController component");
            }
        }

        public RobotController GetClosestRobot(Vector3 position, string floorLevel = null)
        {
            if (!string.IsNullOrEmpty(floorLevel))
            {
                var closestRobotOnFloor = GetClosestRobotByCondition(
                    position, 
                    robot => robot.IsAvailable && robot.CurrentFloor == floorLevel
                );
                
                if (closestRobotOnFloor != null)
                {
                    return closestRobotOnFloor;
                }
            }
            
            var closestAvailableRobot = GetClosestRobotByCondition(
                position, 
                robot => robot.IsAvailable
            );
            
            if (closestAvailableRobot != null)
            {
                return closestAvailableRobot;
            }

            // 找空闲Robot
            var closestFreeRobot = GetClosestRobotByCondition(
                position, 
                robot => robot.IsFree
            );
            
            if (closestFreeRobot != null)
            {
                return closestFreeRobot;
            }

            Debug.Log("No available or free robots found.");
            return null;
        }

        private RobotController GetClosestRobotByCondition(Vector3 position, System.Func<RobotController, bool> condition)
        {
            var closestRobot = robots.Where(condition)
                                     .OrderBy(robot => Calculate3DDistance(robot.transform.position, position, robot.CurrentFloor))
                                     .FirstOrDefault();
            return closestRobot;
        }
        
        private float Calculate3DDistance(Vector3 robotPos, Vector3 targetPos, string robotFloor)
        {
            float horizontalDistance = Vector2.Distance(
                new Vector2(robotPos.x, robotPos.z),
                new Vector2(targetPos.x, targetPos.z)
            );
            
            float verticalPenalty = Mathf.Abs(robotPos.y - targetPos.y) * 50f;
            
            return horizontalDistance + verticalPenalty;
        }

        public RobotController GetRobotById(int robotId)
        {
            if (robotsById.TryGetValue(robotId, out var robot))
            {
                return robot;
            }

            return null;
        }

        public List<RobotController> GetAllRobots()
        {
            return new List<RobotController>(robots);
        }


        public List<RobotController> GetAvailableRobots()
        {
            return robots.Where(r => r.IsAvailable).ToList();
        }
        
        public List<RobotController> GetFreeRobots()
        {
            return robots.Where(r => r.IsFree).ToList();
        }
        
        public List<RobotController> GetAvailableRobotsOnFloor(string floorLevel)
        {
            return robots.Where(r => r.IsAvailable && r.CurrentFloor == floorLevel).ToList();
        }
        
        public Dictionary<string, int> GetRobotCountByFloor()
        {
            return robots.GroupBy(r => r.CurrentFloor)
                        .ToDictionary(g => g.Key, g => g.Count());
        }

        public RobotController GetRobotById(string id)
        {
            return robots.FirstOrDefault(r => r.Id.ToString() == id);
        }


        public void CheckRobotStatus()
        {
            foreach (var robot in robots)
            {
                bool statusChanged = false;

                // Check if robot is busy
                if (!robot.IsAvailable && (robot.lastReportedAvailability || 
                    robot.lastReportedOrderCount != robot.GetRobotOrdersQueueCount))
                {
                    Debug.Log($"Robot {robot.Id} (Floor {robot.CurrentFloor}) is busy with {robot.GetRobotOrdersQueueCount} orders: " +
                            $"{string.Join(", ", robot.listOrdersQueue)}");
                    statusChanged = true;
                }
                
                // Check if robot is free
                if (robot.IsFree && !robot.lastReportedFreeStatus)
                {
                    Debug.Log($"Robot {robot.Id} (Floor {robot.CurrentFloor}) is free now at {robot.transform.position}");
                    statusChanged = true;
                }

                // Check if robot is stuck
                if (robot.IsFree && !robot.IsProcessingOrders && robot.GetRobotOrdersQueueCount > 0)
                {
                    Debug.LogWarning($"Robot {robot.Id} (Floor {robot.CurrentFloor}) is free but not in process of order");
                    statusChanged = true;
                }

                // Update status if changed
                if (statusChanged)
                {
                    // Update last reported status
                    robot.lastReportedAvailability = robot.IsAvailable;
                    robot.lastReportedFreeStatus = robot.IsFree;
                    robot.lastReportedOrderCount = robot.GetRobotOrdersQueueCount;
                }
            }
        }

        public void ResetStuckRobots()
        {
            foreach (var robot in robots)
            {
                if (robot.IsFree && !robot.IsProcessingOrders && robot.GetRobotOrdersQueueCount > 0)
                {
                    Debug.LogWarning($"Reset stuck Robot {robot.Id} (Floor {robot.CurrentFloor})");
                    robot.ResetRobot();
                }
            }
        }

        public List<RobotController> GetFreeRobotsOnFloor(string floorLevel)
        {
            return robots.Where(r => r.IsFree && r.CurrentFloor == floorLevel).ToList();
        }
    }
    
    [System.Serializable]
    public class RobotFloorDistribution
    {
        public string floorLevel = "1F";
        [Range(0, 1)]
        public float ratio = 0.5f;
    }
}