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
            for (int attempts = 0; attempts < 10; attempts++)
            {
                Vector3 initialPosition = new Vector3(Random.Range(0, mapGrid.width), mapGrid.height, Random.Range(0, mapGrid.length));
                CellSpace cellSpace = indoorSpace.GetCellSpaceFromCoordinates(initialPosition);
                if (cellSpace != null && cellSpace.IsNavigable())
                {
                    Point point = (Point)cellSpace.Node;
                    return new Vector3((float)point.X, mapGrid.height, (float)point.Y);
                }
            }
            return Vector3.zero;
        }

        private void CreateRobot(int id, Vector3 position)
        {
            GameObject robotObj = Instantiate(robotPrefab, position, Quaternion.identity);
            robotObj.tag = "Robot";
            robotObj.name = $"Robot {id}";
            robotObj.layer = LayerMask.NameToLayer("RobotLayer");
            RobotController robot = robotObj.GetComponent<RobotController>();
            if (robot != null)
            {
                if (robotObj.GetComponent<Collider>() == null)
                {
                    CapsuleCollider collider = robotObj.AddComponent<CapsuleCollider>();
                    collider.radius = 0.5f;
                }

                if (robotObj.GetComponent<Rigidbody>() == null)
                {
                    Rigidbody rb = robotObj.AddComponent<Rigidbody>();
                    rb.isKinematic = false;
                    rb.useGravity = false;
                    rb.constraints = RigidbodyConstraints.FreezeRotation;
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