using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using CentralControl.RobotControl;

namespace CentralControl.OrderAssignment
{
    public class KMeans
    {
        private int maxIterations = 10;

        public Dictionary<RobotController, List<Order>> ClusterOrders(List<Order> orders, List<RobotController> robots)
        {
            int k = Mathf.Min(robots.Count, orders.Count);
            List<Vector3> centers = InitializeRandomCenters(k, orders);
            Dictionary<int, List<Order>> clusters = new Dictionary<int, List<Order>>();

            for (int iteration = 0; iteration < maxIterations; iteration++)
            {
                clusters.Clear();
                for (int i = 0; i < k; i++)
                {
                    clusters[i] = new List<Order>();
                }

                foreach (var order in orders)
                {
                    int nearestCenterIndex = FindNearestCenterIndex(order.Destination, centers);
                    clusters[nearestCenterIndex].Add(order);
                }

                for (int i = 0; i < k; i++)
                {
                    if (clusters[i].Count > 0)
                    {
                        centers[i] = CalculateClusterCenter(clusters[i]);
                    }
                }
            }

            Dictionary<RobotController, List<Order>> assignments = new Dictionary<RobotController, List<Order>>();
            for (int i = 0; i < k; i++)
            {
                if (clusters[i].Count > 0)
                {
                    RobotController nearestRobot = FindNearestRobot(centers[i], robots);
                    assignments[nearestRobot] = clusters[i];
                }
            }

            return assignments;
        }

        private List<Vector3> InitializeRandomCenters(int k, List<Order> orders)
        {
            return orders.OrderBy(x => Random.value).Take(k).Select(o => o.Destination).ToList();
        }

        private int FindNearestCenterIndex(Vector3 point, List<Vector3> centers)
        {
            return centers.IndexOf(centers.OrderBy(c => Vector3.Distance(c, point)).First());
        }

        private Vector3 CalculateClusterCenter(List<Order> cluster)
        {
            return cluster.Aggregate(Vector3.zero, (sum, order) => sum + order.Destination) / cluster.Count;
        }

        private RobotController FindNearestRobot(Vector3 point, List<RobotController> robots)
        {
            return robots.OrderBy(r => Vector3.Distance(r.transform.position, point)).First();
        }
    }
}
