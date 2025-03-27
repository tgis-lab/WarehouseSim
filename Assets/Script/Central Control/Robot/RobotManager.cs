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

        private IndoorSpace indoorSpace;
        private Graph graph;
        private MapGrid mapGrid;
        private DynamicOccupancyLayer globalOccupancyLayer;
        private IntersectionAreaManager intersectionAreaManager;
        private List<RobotController> robots = new List<RobotController>();
        private HashSet<string> usedSpawnPointIds = new HashSet<string>();

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

            for (int i = 0; i < numberOfRobots; i++)
            {
                Vector3 initialPosition = GetValidInitialPosition();
                if (initialPosition != Vector3.zero)
                {
                    CreateRobot(i + 1, initialPosition);
                }
                else
                {
                    Debug.LogWarning($"Failed to find valid position for robot {i + 1}");
                }
            }

            EventManager.Instance.TriggerRobotsInitialized();
        }

        private Vector3 GetValidInitialPosition()
        {
            // Use a list of cell IDs for spawn points
            List<string> spawnPointCellIds = new List<string> {"c132", "c133", "c357", "c358", "c139", "c140", "c364", "c365", "c146", "c147", "c371", "c372"};
            
            // Filter out used spawn points
            List<string> availableSpawnPointIds = spawnPointCellIds.Where(id => !usedSpawnPointIds.Contains(id)).ToList();
            
            // If no available spawn points, log a warning and return zero vector
            if (availableSpawnPointIds.Count == 0)
            {
                Debug.LogWarning("No available spawn points found.");
                return Vector3.zero;
            }
            
            // Pick a random spawn point from the available list
            string selectedCellId = availableSpawnPointIds[UnityEngine.Random.Range(0, availableSpawnPointIds.Count)];
            
            // Mark the selected spawn point as used
            usedSpawnPointIds.Add(selectedCellId);
            
            // Get the center point of the selected cell space
            var cellSpace = indoorSpace.GetCellSpaceFromId(selectedCellId);
            if (cellSpace != null)
            {
                Point cellSpaceCenterPoint = (Point)cellSpace.Node;
                Vector3 cellSpacePosition = new Vector3((float)cellSpaceCenterPoint.X, mapGrid.height + 0.05f, (float)cellSpaceCenterPoint.Y);
                
                Debug.Log($"Robot spawn point selected: {selectedCellId} at position {cellSpacePosition}");
                return cellSpacePosition;
            }
            
            Debug.LogWarning($"Failed to find cell space for spawn point {selectedCellId}");
            return Vector3.zero;
        }

        public void ResetSpawnPoints()
        {
            usedSpawnPointIds.Clear();
        }

        public void ReleaseSpawnPoint(string cellId)
        {
            if (usedSpawnPointIds.Contains(cellId))
            {
                usedSpawnPointIds.Remove(cellId);
                Debug.Log($"Spawn point {cellId} released.");
            }
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
                
                robot.InitializeRobot(id, indoorSpace, graph, mapGrid, globalOccupancyLayer, intersectionAreaManager);

                Transform targetIndicatorInstance = Instantiate(targetIndicatorPrefab);
                robot.targetIndicator = targetIndicatorInstance;

                robots.Add(robot);
                Debug.Log($"Robot {id} created at position {position}");
            }
            else
            {
                Debug.LogError($"Failed to get RobotController component for robot {id}");
            }
        }

        public void ForceCreateRobots(List<Vector3> positions)
        {
            Debug.Log($"强制创建机器人: 请求创建{positions.Count}个机器人");
            
            int initialCount = robots.Count;
            
            try
            {
                for (int i = 0; i < positions.Count; i++)
                {
                    int robotId = robots.Count + 1; // 从1开始编号，避免ID冲突
                    Debug.Log($"尝试创建机器人 {robotId} 在位置 {positions[i]}");
                    CreateRobot(robotId, positions[i]);
                }
                
                int createdCount = robots.Count - initialCount;
                Debug.Log($"强制创建完成: 成功创建了{createdCount}个机器人");
                
                if (createdCount > 0)
                {
                    // 通知机器人已初始化
                    EventManager.Instance?.TriggerRobotsInitialized();
                    Debug.Log("触发RobotsInitialized事件");
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"强制创建机器人时出错: {ex.Message}");
            }
        }

        public RobotController GetClosestRobot(Vector3 position)
        {
            var closestAvailableRobot = GetClosestRobotByCondition(position, robot => robot.IsAvailable);
            if (closestAvailableRobot != null)
            {
                return closestAvailableRobot;
            }

            var closestFreeRobot = GetClosestRobotByCondition(position, robot => robot.IsFree);
            if (closestFreeRobot != null)
            {
                return closestFreeRobot;
            }

            Debug.Log("No suitable robot found.");
            return null;
        }

        private RobotController GetClosestRobotByCondition(Vector3 position, System.Func<RobotController, bool> condition)
        {
            var closestRobot = robots.Where(condition)
                                     .OrderBy(robot => Vector3.Distance(robot.transform.position, position))
                                     .FirstOrDefault();

            if (closestRobot != null)
            {
                Debug.Log($"Closest robot meeting condition is {closestRobot.Id} at position {closestRobot.transform.position}");
            }
            else
            {
                Debug.Log("No robot meeting condition found.");
            }
            return closestRobot;
        }

        public List<RobotController> GetAllRobots()
        {
            return new List<RobotController>(robots);
        }

        public List<RobotController> GetAvailableRobots()
        {
            return robots.Where(r => r.IsAvailable).ToList();
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
                    Debug.Log($"Robot {robot.Id} is busy with {robot.GetRobotOrdersQueueCount} orders: " +
                            $"{string.Join(", ", robot.listOrdersQueue)}");
                    statusChanged = true;
                }
                
                // Check if robot is free
                if (robot.IsFree && !robot.lastReportedFreeStatus)
                {
                    Debug.Log($"Robot {robot.Id} is now free at position {robot.transform.position}");
                    statusChanged = true;
                }

                // Check if robot is stuck
                if (robot.IsFree && !robot.IsProcessingOrders && robot.GetRobotOrdersQueueCount > 0)
                {
                    Debug.LogWarning($"Robot {robot.Id} is free but not processing orders. " +
                                $"This might indicate an issue.");
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
                    Debug.LogWarning($"Resetting stuck robot {robot.Id}");
                    robot.ResetRobot();
                }
            }
        }

        public List<RobotController> GetFreeRobots()
        {
            return robots.Where(r => r.IsFree).ToList();
        }
    }
}