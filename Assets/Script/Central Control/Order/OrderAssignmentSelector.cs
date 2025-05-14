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
        private Queue<Order> waitingQueue = new Queue<Order>(); // 添加等待队列

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
            // 首先处理已有的等待队列中的订单
            ProcessWaitingQueue(robots.Where(r => r.IsAvailable).ToList());
            
            // 然后处理新的订单
            var availableRobots = robots.Where(r => r.IsAvailable).ToList();
            
            if (availableRobots.Count == 0)
            {
                // 如果没有可用机器人，将所有订单添加到等待队列
                AddToWaitingQueue(orders);
                return;
            }

            List<Order> remainingOrders = new List<Order>(orders);

            // 尝试一一分配订单给最适合的机器人
            while (remainingOrders.Count > 0 && availableRobots.Any())
            {
                Dictionary<RobotController, Order> bestMatches = new Dictionary<RobotController, Order>();
                
                foreach (var robot in availableRobots)
                {
                    Order nearestOrder = FindNearestOrder(robot, remainingOrders);
                    if (nearestOrder != null)
                    {
                        bestMatches[robot] = nearestOrder;
                    }
                }
                
                if (bestMatches.Count == 0)
                    break;
                
                // 找出最优的机器人-订单对
                var bestPair = bestMatches
                    .OrderBy(pair => CalculatePathDistance(pair.Key, pair.Value))
                    .First();
                
                // 分配订单
                AssignOrderToRobot(bestPair.Value, bestPair.Key);
                remainingOrders.Remove(bestPair.Value);
                
                // 更新可用机器人列表
                availableRobots = robots.Where(r => r.IsAvailable).ToList();
            }
            
            // 如果还有剩余订单，添加到等待队列
            if (remainingOrders.Count > 0)
            {
                AddToWaitingQueue(remainingOrders);
            }
        }

        public void InitialOrderDistribution(List<Order> orders, List<RobotController> robots)
        {
            // 先处理等待队列中的订单
            ProcessWaitingQueue(robots.Where(r => r.IsAvailable).ToList());
            
            List<Order> remainingOrders = new List<Order>(orders);
            List<RobotController> availableRobots = robots.Where(r => r.IsAvailable).ToList();
            
            // 为每个可用机器人找一个最近的订单
            foreach (var robot in availableRobots.ToList())
            {
                if (remainingOrders.Count > 0)
                {
                    Order nearestOrder = FindNearestOrder(robot, remainingOrders);
                    if (nearestOrder != null)
                    {
                        AssignOrderToRobot(nearestOrder, robot);
                        remainingOrders.Remove(nearestOrder);
                        
                        // 更新可用机器人列表
                        if (!robot.IsAvailable)
                        {
                            availableRobots.Remove(robot);
                        }
                    }
                }
            }

            // 如果还有订单和可用机器人，继续分配
            if (remainingOrders.Count > 0 && availableRobots.Any())
            {
                var ordersToAssign = new List<Order>(remainingOrders);
                AssignOrders(ordersToAssign, availableRobots);
                remainingOrders = remainingOrders.Except(ordersToAssign).ToList();
            }
            
            // 将剩余订单添加到等待队列
            if (remainingOrders.Count > 0)
            {
                AddToWaitingQueue(remainingOrders);
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
        
        // 新增的等待队列相关方法
        
        /// <summary>
        /// 将订单添加到等待队列
        /// </summary>
        private void AddToWaitingQueue(List<Order> orders)
        {
            foreach (var order in orders)
            {
                waitingQueue.Enqueue(order);
                Debug.Log($"Order {order.Id} added to waiting queue. Queue size: {waitingQueue.Count}");
            }
        }
        
        /// <summary>
        /// 处理等待队列中的订单，尝试分配给可用机器人
        /// </summary>
        /// <param name="availableRobots">可用的机器人列表</param>
        /// <returns>成功分配的订单数量</returns>
        public int ProcessWaitingQueue(List<RobotController> availableRobots)
        {
            if (waitingQueue.Count == 0 || !availableRobots.Any())
                return 0;
            
            int assignedCount = 0;
            List<Order> tempList = new List<Order>(waitingQueue);
            List<Order> assignedOrders = new List<Order>();
            
            foreach (var robot in availableRobots.ToList())
            {
                if (!tempList.Any() || !robot.IsAvailable)
                    continue;
                
                Order bestOrder = FindNearestOrder(robot, tempList);
                if (bestOrder != null)
                {
                    AssignOrderToRobot(bestOrder, robot);
                    tempList.Remove(bestOrder);
                    assignedOrders.Add(bestOrder);
                    assignedCount++;
                    
                    Debug.Log($"Order {bestOrder.Id} assigned from waiting queue to Robot {robot.Id}");
                    
                    // 如果机器人不再可用，则从列表中移除
                    if (!robot.IsAvailable)
                    {
                        availableRobots.Remove(robot);
                    }
                }
            }
            
            // 更新等待队列，移除已分配的订单
            waitingQueue = new Queue<Order>(waitingQueue.Where(o => !assignedOrders.Contains(o)));
            
            return assignedCount;
        }
        
        /// <summary>
        /// 当某个机器人变为可用时，尝试从等待队列为其分配订单
        /// </summary>
        /// <param name="robot">变为可用的机器人</param>
        /// <returns>是否成功分配订单</returns>
        public bool TryAssignOrderFromQueue(RobotController robot)
        {
            if (waitingQueue.Count == 0 || !robot.IsAvailable)
                return false;
            
            List<Order> tempList = new List<Order>(waitingQueue);
            Order bestOrder = FindNearestOrder(robot, tempList);
            
            if (bestOrder != null)
            {
                AssignOrderToRobot(bestOrder, robot);
                waitingQueue = new Queue<Order>(tempList.Where(o => o != bestOrder));
                Debug.Log($"Order {bestOrder.Id} assigned from waiting queue to Robot {robot.Id}");
                return true;
            }
            
            return false;
        }
        
        /// <summary>
        /// 获取等待队列中的订单数量
        /// </summary>
        public int GetWaitingQueueCount()
        {
            return waitingQueue.Count;
        }
        
        /// <summary>
        /// 清空等待队列
        /// </summary>
        public void ClearWaitingQueue()
        {
            waitingQueue.Clear();
            Debug.Log("Waiting queue cleared");
        }
    }
}