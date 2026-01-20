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
        private Queue<Order> waitingQueue = new Queue<Order>();

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
            ProcessWaitingQueue(robots.Where(r => r.IsAvailable).ToList());
            
            var availableRobots = robots.Where(r => r.IsAvailable).ToList();
            
            if (availableRobots.Count == 0)
            {
                AddToWaitingQueue(orders);
                return;
            }

            List<Order> remainingOrders = new List<Order>(orders);

            while (remainingOrders.Count > 0 && availableRobots.Any())
            {
                Dictionary<RobotController, Order> bestMatches = new Dictionary<RobotController, Order>();
                
                foreach (var robot in availableRobots)
                {
                    Order bestOrder = FindBestOrder(robot, remainingOrders);
                    if (bestOrder != null)
                    {
                        bestMatches[robot] = bestOrder;
                    }
                }
                
                if (bestMatches.Count == 0)
                    break;
                
                var bestPair = bestMatches
                    .OrderByDescending(pair => CalculateAssignmentScore(pair.Key, pair.Value))
                    .First();
                
                AssignOrderToRobot(bestPair.Value, bestPair.Key);
                remainingOrders.Remove(bestPair.Value);
                
                availableRobots = robots.Where(r => r.IsAvailable).ToList();
            }
            
            if (remainingOrders.Count > 0)
            {
                AddToWaitingQueue(remainingOrders);
            }
        }

        public void InitialOrderDistribution(List<Order> orders, List<RobotController> robots)
        {
            ProcessWaitingQueue(robots.Where(r => r.IsAvailable).ToList());
            
            List<Order> remainingOrders = new List<Order>(orders);
            List<RobotController> availableRobots = robots.Where(r => r.IsAvailable).ToList();
            
            foreach (var robot in availableRobots.ToList())
            {
                if (remainingOrders.Count > 0)
                {
                    Order bestOrder = FindBestOrder(robot, remainingOrders);
                    if (bestOrder != null)
                    {
                        AssignOrderToRobot(bestOrder, robot);
                        remainingOrders.Remove(bestOrder);
                        
                        if (!robot.IsAvailable)
                        {
                            availableRobots.Remove(robot);
                        }
                    }
                }
            }

            if (remainingOrders.Count > 0 && availableRobots.Any())
            {
                var ordersToAssign = new List<Order>(remainingOrders);
                AssignOrders(ordersToAssign, availableRobots);
                remainingOrders = remainingOrders.Except(ordersToAssign).ToList();
            }
            
            if (remainingOrders.Count > 0)
            {
                AddToWaitingQueue(remainingOrders);
            }
        }

        public Order FindBestOrder(RobotController robot, List<Order> orders)
        {
            if (orders.Count == 0) return null;
            
            return orders.OrderByDescending(o => CalculateAssignmentScore(robot, o)).FirstOrDefault();
        }
        
        private List<Order> SortOrdersByScoreForRobot(RobotController robot, List<Order> orders)
        {
            return orders.OrderByDescending(o => CalculateAssignmentScore(robot, o)).ToList();
        }

        private float CalculateAssignmentScore(RobotController robot, Order order)
        {
            // 1. Distance Score
            Vector3 robotPos = robot.GetLastOrderDestination();
            float distance = Vector3.Distance(robotPos, order.Destination);
            
            // Floor penalty
            if (robot.CurrentFloor != order.FloorLevel)
            {
                distance += 50f; // Add virtual distance for floor change
            }
            
            // Normalize distance: 0m = 1.0, 150m = 0.0
            float distanceScore = Mathf.Clamp01(1.0f - (distance / 150.0f));
            
            // 2. Load Score
            // Max orders = 20. 0 orders = 1.0, 20 orders = 0.0
            float loadScore = 1.0f - ((float)robot.GetRobotOrdersQueueCount / robot.MaxOrder);
            
            // 3. Priority Score
            // Level 1 = 0.2, Level 5 = 1.0
            float priorityScore = Mathf.Clamp01(order.PriorityLevel / 5.0f);
            
            // Weights
            float w1 = 0.4f; // Distance
            float w2 = 0.4f; // Load
            float w3 = 0.2f; // Priority
            
            return (distanceScore * w1) + (loadScore * w2) + (priorityScore * w3);
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
        
        private void AddToWaitingQueue(List<Order> orders)
        {
            foreach (var order in orders)
            {
                waitingQueue.Enqueue(order);
                Debug.Log($"Order {order.Id} added to waiting queue. Queue size: {waitingQueue.Count}");
            }
        }

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
                
                Order bestOrder = FindBestOrder(robot, tempList);
                if (bestOrder != null)
                {
                    AssignOrderToRobot(bestOrder, robot);
                    tempList.Remove(bestOrder);
                    assignedOrders.Add(bestOrder);
                    assignedCount++;
                    
                    Debug.Log($"Order {bestOrder.Id} assigned from waiting queue to Robot {robot.Id}");
                    
                    if (!robot.IsAvailable)
                    {
                        availableRobots.Remove(robot);
                    }
                }
            }
            
            waitingQueue = new Queue<Order>(waitingQueue.Where(o => !assignedOrders.Contains(o)));
            
            return assignedCount;
        }
        
        public bool TryAssignOrderFromQueue(RobotController robot)
        {
            if (waitingQueue.Count == 0 || !robot.IsAvailable)
                return false;

            List<Order> tempList = new List<Order>(waitingQueue);
            Order bestOrder = FindBestOrder(robot, tempList);

            if (bestOrder != null)
            {
                AssignOrderToRobot(bestOrder, robot);
                waitingQueue = new Queue<Order>(tempList.Where(o => o != bestOrder));
                Debug.Log($"Order {bestOrder.Id} assigned from waiting queue to Robot {robot.Id}");
                return true;
            }

            return false;
        }
        
        public int GetWaitingQueueCount()
        {
            return waitingQueue.Count;
        }
        
        public void ClearWaitingQueue()
        {
            waitingQueue.Clear();
            Debug.Log("Waiting queue cleared");
        }
    }
}