using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Map;

namespace CentralControl.OrderAssignment
{
    public class OrderManager: MonoBehaviour
    {
        private List<Order> orders = new List<Order>();

        public void AddOrder(Order order)
        {
            orders.Add(order);
            Debug.Log($"Added new order: {order.Id} at {order.Destination} on floor {order.FloorLevel}");
        }

        public List<Order> GetAllOrders()
        {
            return orders.OrderBy(o => o.Priority).ToList();
        }

        public Order GetOrderById(string id)
        {
            return orders.FirstOrDefault(o => o.Id == id);
        }

        public Order GetNextOrder()
        {
            var order = orders.OrderBy(o => o.Priority).FirstOrDefault();
            if (order != null)
            {
                orders.Remove(order);
            }
            return order;
        }
        
        // 添加按楼层获取订单的方法
        public List<Order> GetOrdersByFloor(string floorLevel)
        {
            return orders.Where(o => o.FloorLevel == floorLevel)
                        .OrderBy(o => o.Priority)
                        .ToList();
        }

        // 获取所有可用楼层
        public List<string> GetAllFloors()
        {
            return orders.Select(o => o.FloorLevel)
                        .Distinct()
                        .ToList();
        }

        // 添加楼层分布统计方法
        public Dictionary<string, int> GetOrderCountByFloor()
        {
            return orders.GroupBy(o => o.FloorLevel)
                        .ToDictionary(g => g.Key, g => g.Count());
        }
        
        // 获取特定楼层的下一个订单
        public Order GetNextOrderOnFloor(string floorLevel)
        {
            var order = orders.Where(o => o.FloorLevel == floorLevel)
                            .OrderBy(o => o.Priority)
                            .FirstOrDefault();
            
            if (order != null)
            {
                orders.Remove(order);
            }
            
            return order;
        }
        
        // 添加多个订单
        public void AddOrders(IEnumerable<Order> newOrders)
        {
            if (newOrders == null) return;
            
            foreach (var order in newOrders)
            {
                AddOrder(order);
            }
        }
    }
}