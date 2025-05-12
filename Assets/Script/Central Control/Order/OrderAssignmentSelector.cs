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
        private IndoorSpace indoorSpace; // 添加对地图的引用以检查楼层连接

        public OrderAssignmentSelector()
        {
            kMeans = new KMeans();
            indoorSpace = null; // 需要在使用前初始化
        }
        
        public OrderAssignmentSelector(IndoorSpace indoorSpace)
        {
            kMeans = new KMeans();
            this.indoorSpace = indoorSpace;
        }
        
        // 设置地图引用
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
                
                // 对每个机器人的订单按距离排序
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
            
            // 考虑楼层因素的距离计算
            return orders.OrderBy(o => CalculatePathDistance(robot, o)).FirstOrDefault();
        }
        
        // 对机器人的订单按距离排序
        private List<Order> SortOrdersByDistanceForRobot(RobotController robot, List<Order> orders)
        {
            return orders.OrderBy(o => CalculatePathDistance(robot, o)).ToList();
        }

        private void AssignOrderToRobot(Order order, RobotController robot)
        {
            robot.ReceiveOrder(order);
            order.AssignedRobotId = robot.Id;
        }
        
        // 计算路径距离（考虑楼层和垂直连接）
        private float CalculatePathDistance(RobotController robot, Order order)
        {
            // 获取水平距离
            float horizontalDistance = Vector2.Distance(
                new Vector2(robot.transform.position.x, robot.transform.position.z),
                new Vector2(order.Destination.x, order.Destination.z));
            
            // 楼层差异带来的额外成本
            float floorPenalty = 0;
            
            if (robot.CurrentFloor != order.FloorLevel)
            {
                int robotFloor = ParseFloorLevel(robot.CurrentFloor);
                int orderFloor = ParseFloorLevel(order.FloorLevel);
                int floorDifference = Mathf.Abs(robotFloor - orderFloor);
                
                // 楼层差异的基本成本（每层楼相当于200米水平距离）
                floorPenalty = floorDifference * 200f;
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