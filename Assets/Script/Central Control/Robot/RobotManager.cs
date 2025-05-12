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

            // 获取所有可用楼层
            List<string> availableFloors = indoorSpace.GetAllFloorLevels();
            
            if (availableFloors.Count == 0)
            {
                Debug.LogError("没有找到可用楼层，无法初始化机器人");
                return;
            }

            // 计算每层楼的机器人数量
            Dictionary<string, int> robotsPerFloor;
            
            if (distributeEvenly)
            {
                // 均匀分配机器人到各楼层
                robotsPerFloor = DistributeRobotsEvenly(numberOfRobots, availableFloors);
            }
            else
            {
                // 按照自定义比例分配
                robotsPerFloor = DistributeRobotsByCustomRatio(numberOfRobots, availableFloors);
            }

            int robotId = 1;
            // 在每层创建机器人
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
                        Debug.LogWarning($"无法在楼层 {floor} 找到有效位置放置机器人 {robotId}");
                    }
                }
            }

            EventManager.Instance?.TriggerRobotsInitialized();
        }
        
        // 均匀分配机器人到各楼层
        private Dictionary<string, int> DistributeRobotsEvenly(int totalRobots, List<string> floors)
        {
            Dictionary<string, int> distribution = new Dictionary<string, int>();
            int robotsPerFloor = totalRobots / floors.Count;
            int remainingRobots = totalRobots % floors.Count;
            
            foreach (string floor in floors)
            {
                // 基本分配每层相同数量的机器人
                distribution[floor] = robotsPerFloor;
                
                // 如果有剩余，每层额外分配一个，直到分完
                if (remainingRobots > 0)
                {
                    distribution[floor]++;
                    remainingRobots--;
                }
            }
            
            return distribution;
        }
        
        // 按照自定义比例分配机器人
        private Dictionary<string, int> DistributeRobotsByCustomRatio(int totalRobots, List<string> floors)
        {
            Dictionary<string, float> ratios = new Dictionary<string, float>();
            
            // 如果配置了自定义分布，使用配置的比例
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
            
            // 如果没有配置完整的比例，为剩余楼层分配均等的比例
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
                // 如果没有有效的配置，使用均等分配
                foreach (string floor in floors)
                {
                    ratios[floor] = 1.0f / floors.Count;
                }
            }
            
            // 按比例计算每层机器人数量
            Dictionary<string, int> distribution = new Dictionary<string, int>();
            int assignedRobots = 0;
            
            foreach (var entry in ratios)
            {
                int robotsOnFloor = (int)Math.Floor(totalRobots * entry.Value);
                distribution[entry.Key] = robotsOnFloor;
                assignedRobots += robotsOnFloor;
            }
            
            // 处理因舍入导致的剩余机器人
            int remainingRobots = totalRobots - assignedRobots;
            
            // 将剩余的机器人分配给比例最高的楼层
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
        
        // 获取指定楼层的有效初始位置
        private Vector3 GetValidInitialPosition(string floorLevel)
        {
            // 确保楼层对应的集合已初始化
            if (!usedSpawnPointsByFloor.ContainsKey(floorLevel))
            {
                usedSpawnPointsByFloor[floorLevel] = new HashSet<CellSpace>();
            }
            
            // 获取该楼层的所有可用通道CellSpace
            List<CellSpace> spawnPoints = indoorSpace.GetCellSpaceFromLayerTypeAndFloor("aisle", floorLevel);
            
            if (spawnPoints == null || spawnPoints.Count == 0)
            {
                // 尝试使用任何可导航的CellSpace
                spawnPoints = indoorSpace.GetNavigableCellSpacesOnFloor(floorLevel);
                
                if (spawnPoints == null || spawnPoints.Count == 0)
                {
                    return Vector3.zero;
                }
            }
            
            // 过滤出未使用的生成点
            List<CellSpace> availableSpawnPoints = spawnPoints
                .Where(sp => !usedSpawnPointsByFloor[floorLevel].Contains(sp))
                .ToList();
            
            // 如果没有可用的生成点，尝试重置该楼层的使用记录
            if (availableSpawnPoints.Count == 0)
            {
                // 考虑部分重置或完全重置
                usedSpawnPointsByFloor[floorLevel].Clear();
                availableSpawnPoints = spawnPoints;
            }
            
            // 从可用的生成点中随机选择一个
            CellSpace selectedCellSpace = availableSpawnPoints[UnityEngine.Random.Range(0, availableSpawnPoints.Count)];
            
            // 标记选择的生成点为已使用
            usedSpawnPointsByFloor[floorLevel].Add(selectedCellSpace);
            
            // 获取选定CellSpace的中心点
            Point cellSpaceCenterPoint = (Point)selectedCellSpace.Node;
            
            // 计算Y坐标 (基于楼层)
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
                Debug.Log($"创建机器人 {id} 在位置 {position}");
            }
            else
            {
                Debug.LogError($"无法获取机器人 {id} 的RobotController组件");
            }
        }

        public RobotController GetClosestRobot(Vector3 position, string floorLevel = null)
        {
            // 如果指定了楼层，优先返回同楼层的机器人
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
            
            // 找任意楼层的可用机器人
            var closestAvailableRobot = GetClosestRobotByCondition(
                position, 
                robot => robot.IsAvailable
            );
            
            if (closestAvailableRobot != null)
            {
                return closestAvailableRobot;
            }

            // 找空闲机器人
            var closestFreeRobot = GetClosestRobotByCondition(
                position, 
                robot => robot.IsFree
            );
            
            if (closestFreeRobot != null)
            {
                return closestFreeRobot;
            }

            Debug.Log("没有找到合适的机器人。");
            return null;
        }

        private RobotController GetClosestRobotByCondition(Vector3 position, System.Func<RobotController, bool> condition)
        {
            // 考虑3D距离
            var closestRobot = robots.Where(condition)
                                     .OrderBy(robot => Calculate3DDistance(robot.transform.position, position, robot.CurrentFloor))
                                     .FirstOrDefault();
            return closestRobot;
        }
        
        // 计算考虑楼层的3D距离
        private float Calculate3DDistance(Vector3 robotPos, Vector3 targetPos, string robotFloor)
        {
            // 水平距离
            float horizontalDistance = Vector2.Distance(
                new Vector2(robotPos.x, robotPos.z),
                new Vector2(targetPos.x, targetPos.z)
            );
            
            // 垂直距离惩罚(通过Y坐标差值)
            float verticalPenalty = Mathf.Abs(robotPos.y - targetPos.y) * 50f; // 垂直距离惩罚权重
            
            return horizontalDistance + verticalPenalty;
        }

        public List<RobotController> GetAllRobots()
        {
            return new List<RobotController>(robots);
        }

        public List<RobotController> GetAvailableRobots()
        {
            return robots.Where(r => r.IsAvailable).ToList();
        }
        
        // 获取指定楼层的可用机器人
        public List<RobotController> GetAvailableRobotsOnFloor(string floorLevel)
        {
            return robots.Where(r => r.IsAvailable && r.CurrentFloor == floorLevel).ToList();
        }
        
        // 获取所有楼层的机器人数量统计
        public Dictionary<string, int> GetRobotCountByFloor()
        {
            return robots.GroupBy(r => r.CurrentFloor)
                        .ToDictionary(g => g.Key, g => g.Count());
        }

        public RobotController GetRobotById(int id)
        {
            return robots.FirstOrDefault(r => r.Id == id);
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
                    Debug.Log($"机器人 {robot.Id} (楼层 {robot.CurrentFloor}) 正忙于 {robot.GetRobotOrdersQueueCount} 个订单: " +
                            $"{string.Join(", ", robot.listOrdersQueue)}");
                    statusChanged = true;
                }
                
                // Check if robot is free
                if (robot.IsFree && !robot.lastReportedFreeStatus)
                {
                    Debug.Log($"机器人 {robot.Id} (楼层 {robot.CurrentFloor}) 现在空闲，位置 {robot.transform.position}");
                    statusChanged = true;
                }

                // Check if robot is stuck
                if (robot.IsFree && !robot.IsProcessingOrders && robot.GetRobotOrdersQueueCount > 0)
                {
                    Debug.LogWarning($"机器人 {robot.Id} (楼层 {robot.CurrentFloor}) 空闲但未处理订单队列，这可能表明存在问题。");
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
                    Debug.LogWarning($"重置卡住的机器人 {robot.Id} (楼层 {robot.CurrentFloor})");
                    robot.ResetRobot();
                }
            }
        }

        public List<RobotController> GetFreeRobots()
        {
            return robots.Where(r => r.IsFree).ToList();
        }
        
        // 获取特定楼层的空闲机器人
        public List<RobotController> GetFreeRobotsOnFloor(string floorLevel)
        {
            return robots.Where(r => r.IsFree && r.CurrentFloor == floorLevel).ToList();
        }
    }
    
    // 用于Inspector设置楼层分布的辅助类
    [System.Serializable]
    public class RobotFloorDistribution
    {
        public string floorLevel = "1F";
        [Range(0, 1)]
        public float ratio = 0.5f;
    }
}