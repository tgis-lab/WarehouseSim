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
            
            // 返回距离最小的中心点索引
            return distances.OrderBy(kv => kv.Value).First().Key;
        }

        private (Vector3 center, string dominantFloor) CalculateClusterCenterAndDominantFloor(List<Order> cluster)
        {
            if (cluster.Count == 0)
                return (Vector3.zero, "1F");
            
            // 确定主要楼层（出现频率最高的楼层）
            var floorGroups = cluster.GroupBy(o => o.FloorLevel)
                                    .OrderByDescending(g => g.Count())
                                    .ToList();
            
            string dominantFloor = floorGroups.First().Key;
            
            // 计算聚类中心
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
        
        // 考虑楼层差异的距离计算
        private float CalculateDistance(Vector3 point1, string floor1, Vector3 point2, string floor2)
        {
            // 水平距离计算 (XZ平面)
            float horizontalDistance = Vector2.Distance(
                new Vector2(point1.x, point1.z),
                new Vector2(point2.x, point2.z));
            
            // 楼层差异带来的额外距离惩罚
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
        
        // 解析楼层编号
        private int ParseFloorLevel(string floorLevel)
        {
            if (string.IsNullOrEmpty(floorLevel)) 
                return 1; // 默认为1楼
            
            // 处理地上楼层："F1"或"1F"
            if (floorLevel.StartsWith("F") || floorLevel.EndsWith("F"))
            {
                string numPart = floorLevel.Replace("F", "");
                if (int.TryParse(numPart, out int level))
                {
                    return level;
                }
            }
            
            // 处理地下楼层："B1"
            if (floorLevel.StartsWith("B") || floorLevel.StartsWith("b"))
            {
                string numPart = floorLevel.Substring(1).Replace("F", "").Replace("f", "");
                if (int.TryParse(numPart, out int level))
                {
                    return -level;
                }
            }
            
            // 尝试简单数字解析
            if (int.TryParse(floorLevel, out int simpleLevel))
            {
                return simpleLevel;
            }
            
            return 1; // 如果解析失败，默认为1楼
        }
    }
}