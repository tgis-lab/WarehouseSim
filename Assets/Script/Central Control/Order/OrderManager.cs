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
            Debug.Log($"Added new order: {order.Id} at {order.Destination}");
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
    }
}