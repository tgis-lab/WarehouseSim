using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using CentralControl.RobotControl;

namespace CentralControl.OrderAssignment
{
    public class OrderAssignmentSelector
    {
        private KMeans kMeans;

        public OrderAssignmentSelector()
        {
            kMeans = new KMeans();
        }

        public void AssignOrders(List<Order> orders, List<RobotController> robots)
        {
            var assignments = kMeans.ClusterOrders(orders, robots);
            foreach (var kvp in assignments)
            {
                RobotController robot = kvp.Key;
                List<Order> assignedOrders = kvp.Value;
                foreach (var order in assignedOrders)
                {
                    AssignOrderToRobot(order, robot);
                }
            }
        }

        public void InitialOrderDistribution(List<Order> orders, List<RobotController> robots)
        {
            foreach (var robot in robots)
            {
                if (orders.Count > 0)
                {
                    Order nearestOrder = FindNearestOrder(robot, orders);
                    AssignOrderToRobot(nearestOrder, robot);
                    orders.Remove(nearestOrder);
                }
            }

            if (orders.Count > 0)
            {
                AssignOrders(orders, robots);
            }
        }

        private Order FindNearestOrder(RobotController robot, List<Order> orders)
        {
            return orders.OrderBy(o => Vector3.Distance(robot.transform.position, o.Destination)).FirstOrDefault();
        }

        private void AssignOrderToRobot(Order order, RobotController robot)
        {
            robot.ReceiveOrder(order);
            order.AssignedRobotId = robot.Id;
        }
    }
}
