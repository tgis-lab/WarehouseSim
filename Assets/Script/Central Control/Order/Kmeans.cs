using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using CentralControl.RobotControl;

namespace CentralControl.OrderAssignment
{
    public class KMeans
    {
        private int maxIterations = 10;
        private const float FLOOR_DIFFERENCE_PENALTY = 150f;

        public Dictionary<RobotController, List<Order>> ClusterOrders(List<Order> orders, List<RobotController> robots)
        {
            int k = Mathf.Min(robots.Count, orders.Count);
            List<Vector3> centers = InitializeRandomCenters(k, orders);
            Dictionary<int, List<Order>> clusters = new Dictionary<int, List<Order>>();
            Dictionary<int, string> dominantFloors = new Dictionary<int, string>();

            for (int iteration = 0; iteration < maxIterations; iteration++)
            {
                clusters.Clear();
                for (int i = 0; i < k; i++)
                {
                    clusters[i] = new List<Order>();
                }

                foreach (var order in orders)
                {
                    int nearestCenterIndex = FindNearestCenterIndex(order.Destination, order.FloorLevel, centers, dominantFloors);
                    clusters[nearestCenterIndex].Add(order);
                }

                for (int i = 0; i < k; i++)
                {
                    if (clusters[i].Count > 0)
                    {
                        var clusterResult = CalculateClusterCenterAndDominantFloor(clusters[i]);
                        centers[i] = clusterResult.center;
                        dominantFloors[i] = clusterResult.dominantFloor;
                    }
                }
            }

            Dictionary<RobotController, List<Order>> assignments = new Dictionary<RobotController, List<Order>>();
            for (int i = 0; i < k; i++)
            {
                if (clusters[i].Count > 0)
                {
                    string clusterFloor = dominantFloors.ContainsKey(i) ? dominantFloors[i] : "1F";
                    RobotController nearestRobot = FindNearestRobot(centers[i], clusterFloor, robots);
                    assignments[nearestRobot] = clusters[i];
                }
            }

            return assignments;
        }

        private List<Vector3> InitializeRandomCenters(int k, List<Order> orders)
        {
            return orders.OrderBy(x => UnityEngine.Random.value).Take(k).Select(o => o.Destination).ToList();
        }

        private int FindNearestCenterIndex(Vector3 point, string floorLevel, List<Vector3> centers, Dictionary<int, string> dominantFloors)
        {
            Dictionary<int, float> distances = new Dictionary<int, float>();
            
            for (int i = 0; i < centers.Count; i++)
            {
                string centerFloor = dominantFloors.ContainsKey(i) ? dominantFloors[i] : "1F";
                float distance = CalculateDistance(point, floorLevel, centers[i], centerFloor);
                distances[i] = distance;
            }
            
            return distances.OrderBy(kv => kv.Value).First().Key;
        }

        private (Vector3 center, string dominantFloor) CalculateClusterCenterAndDominantFloor(List<Order> cluster)
        {
            if (cluster.Count == 0)
                return (Vector3.zero, "1F");
            
            var floorGroups = cluster.GroupBy(o => o.FloorLevel)
                                    .OrderByDescending(g => g.Count())
                                    .ToList();
            
            string dominantFloor = floorGroups.First().Key;
            
            Vector3 sum = Vector3.zero;
            foreach (var order in cluster)
            {
                sum += order.Destination;
            }
            
            return (sum / cluster.Count, dominantFloor);
        }

        private RobotController FindNearestRobot(Vector3 point, string floorLevel, List<RobotController> robots)
        {
            return robots.OrderBy(r => CalculateDistance(point, floorLevel, r.transform.position, r.CurrentFloor)).First();
        }
        
        private float CalculateDistance(Vector3 point1, string floor1, Vector3 point2, string floor2)
        {
            float horizontalDistance = Vector2.Distance(
                new Vector2(point1.x, point1.z),
                new Vector2(point2.x, point2.z));
            
            float floorPenalty = 0;
            if (floor1 != floor2)
            {
                int floor1Level = ParseFloorLevel(floor1);
                int floor2Level = ParseFloorLevel(floor2);
                int floorDifference = Mathf.Abs(floor1Level - floor2Level);
                floorPenalty = floorDifference * FLOOR_DIFFERENCE_PENALTY;
            }
            
            return horizontalDistance + floorPenalty;
        }
        
        private int ParseFloorLevel(string floorLevel)
        {
            return Map.FloorLevelUtils.Parse(floorLevel, defaultValue: 1);
        }
    }
}