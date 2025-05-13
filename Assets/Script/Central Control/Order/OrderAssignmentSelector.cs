using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using CentralControl.RobotControl;
using Map;

namespace CentralControl.OrderAssignment
{
    public class OrderAssignmentSelector
    {
        private KMeans kMeans;
        private IndoorSpace indoorSpace;

        public OrderAssignmentSelector()
        {
            kMeans = new KMeans();
            indoorSpace = null;
        }
        
        public OrderAssignmentSelector(IndoorSpace indoorSpace)
        {
            kMeans = new KMeans();
            this.indoorSpace = indoorSpace;
        }
        
        public void SetIndoorSpace(IndoorSpace indoorSpace)
        {
            this.indoorSpace = indoorSpace;
        }

        public void AssignOrders(List<Order> orders, List<RobotController> robots)
        {
            var assignments = kMeans.ClusterOrders(orders, robots);
            foreach (var kvp in assignments)
            {
                RobotController robot = kvp.Key;
                List<Order> assignedOrders = kvp.Value;
                
                var sortedOrders = SortOrdersByDistanceForRobot(robot, assignedOrders);
                
                foreach (var order in sortedOrders)
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
                    if (nearestOrder != null)
                    {
                        AssignOrderToRobot(nearestOrder, robot);
                        orders.Remove(nearestOrder);
                    }
                }
            }

            if (orders.Count > 0)
            {
                AssignOrders(orders, robots);
            }
        }

        public Order FindNearestOrder(RobotController robot, List<Order> orders)
        {
            if (orders.Count == 0) return null;
            
            return orders.OrderBy(o => CalculatePathDistance(robot, o)).FirstOrDefault();
        }
        
        private List<Order> SortOrdersByDistanceForRobot(RobotController robot, List<Order> orders)
        {
            return orders.OrderBy(o => CalculatePathDistance(robot, o)).ToList();
        }

        private void AssignOrderToRobot(Order order, RobotController robot)
        {
            robot.ReceiveOrder(order);
            order.AssignedRobotId = robot.Id;
        }
        
        private float CalculatePathDistance(RobotController robot, Order order)
        {
            float horizontalDistance = Vector2.Distance(
                new Vector2(robot.transform.position.x, robot.transform.position.z),
                new Vector2(order.Destination.x, order.Destination.z));
            
            float floorPenalty = 0;
            
            if (robot.CurrentFloor != order.FloorLevel)
            {
                int robotFloor = ParseFloorLevel(robot.CurrentFloor);
                int orderFloor = ParseFloorLevel(order.FloorLevel);
                int floorDifference = Mathf.Abs(robotFloor - orderFloor);
                
                floorPenalty = floorDifference * 200f;
            }
            
            return horizontalDistance + floorPenalty;
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
    }
}