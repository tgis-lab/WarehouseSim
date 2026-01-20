using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Map;

namespace CentralControl.OrderAssignment
{
    public class OrderManager: MonoBehaviour
    {
        [Header("Retry Settings")]
        [SerializeField] private float retryDelaySeconds = 5f;
        [SerializeField] private float retryCheckIntervalSeconds = 2f;
        
        private List<Order> pendingOrders = new List<Order>();
        private List<Order> failedOrders = new List<Order>();
        private Dictionary<string, Order> allOrders = new Dictionary<string, Order>();
        
        private Coroutine retryCoroutine;

        public event Action<Order> OnOrderFailed;
        public event Action<Order> OnOrderRetryScheduled;
        public event Action<Order> OnOrderMaxRetriesExceeded;

        private void Start()
        {
            retryCoroutine = StartCoroutine(RetryFailedOrdersRoutine());
            
            if (EventManager.Instance != null)
            {
                EventManager.Instance.OnOrderCompleted += HandleOrderCompleted;
                EventManager.Instance.OnOrderFailed += HandleOrderFailed;
            }
        }

        private void OnDestroy()
        {
            if (retryCoroutine != null)
            {
                StopCoroutine(retryCoroutine);
            }
            
            if (EventManager.Instance != null)
            {
                EventManager.Instance.OnOrderCompleted -= HandleOrderCompleted;
                EventManager.Instance.OnOrderFailed -= HandleOrderFailed;
            }
        }

        private void HandleOrderCompleted(string orderId, string robotId)
        {
            ReportOrderCompleted(orderId);
        }

        private void HandleOrderFailed(string orderId, string robotId, string reason)
        {
            ReportOrderFailed(orderId, reason);
        }

        public void AddOrder(Order order)
        {
            pendingOrders.Add(order);
            allOrders[order.Id] = order;
            Debug.Log($"Added new order: {order.Id} at {order.Destination} on floor {order.FloorLevel}");
        }

        public List<Order> GetAllOrders()
        {
            return pendingOrders.Where(o => o.Status == OrderStatus.Pending)
                               .OrderBy(o => o.Priority)
                               .ToList();
        }

        public Order GetOrderById(string id)
        {
            allOrders.TryGetValue(id, out var order);
            return order;
        }

        public Order GetNextOrder()
        {
            var order = pendingOrders.Where(o => o.Status == OrderStatus.Pending)
                                     .OrderBy(o => o.Priority)
                                     .FirstOrDefault();
            if (order != null)
            {
                pendingOrders.Remove(order);
            }
            return order;
        }
        
        public List<Order> GetOrdersByFloor(string floorLevel)
        {
            return pendingOrders.Where(o => o.FloorLevel == floorLevel && o.Status == OrderStatus.Pending)
                               .OrderBy(o => o.Priority)
                               .ToList();
        }

        public List<string> GetAllFloors()
        {
            return pendingOrders.Where(o => o.Status == OrderStatus.Pending)
                               .Select(o => o.FloorLevel)
                               .Distinct()
                               .ToList();
        }

        public Dictionary<string, int> GetOrderCountByFloor()
        {
            return pendingOrders.Where(o => o.Status == OrderStatus.Pending)
                               .GroupBy(o => o.FloorLevel)
                               .ToDictionary(g => g.Key, g => g.Count());
        }
        
        public Order GetNextOrderOnFloor(string floorLevel)
        {
            var order = pendingOrders.Where(o => o.FloorLevel == floorLevel && o.Status == OrderStatus.Pending)
                                     .OrderBy(o => o.Priority)
                                     .FirstOrDefault();
            
            if (order != null)
            {
                pendingOrders.Remove(order);
            }
            
            return order;
        }
        
        public void AddOrders(IEnumerable<Order> newOrders)
        {
            if (newOrders == null) return;
            
            foreach (var order in newOrders)
            {
                AddOrder(order);
            }
        }

        public void ReportOrderFailed(string orderId, string reason)
        {
            if (!allOrders.TryGetValue(orderId, out var order)) return;
            
            order.MarkFailed(reason);
            
            Debug.LogWarning($"Order {orderId} failed: {reason} (Attempt {order.RetryCount}/{Order.MaxRetries})");
            
            OnOrderFailed?.Invoke(order);
            
            if (order.CanRetry())
            {
                failedOrders.Add(order);
                OnOrderRetryScheduled?.Invoke(order);
                Debug.Log($"Order {orderId} scheduled for retry in {retryDelaySeconds}s");
            }
            else
            {
                OnOrderMaxRetriesExceeded?.Invoke(order);
                Debug.LogError($"Order {orderId} exceeded max retries, marked as permanently failed");
            }
        }

        public void ReportOrderCompleted(string orderId)
        {
            if (!allOrders.TryGetValue(orderId, out var order)) return;
            
            order.MarkCompleted();
            Debug.Log($"Order {orderId} completed successfully");
        }

        public void ReportOrderInProgress(string orderId)
        {
            if (!allOrders.TryGetValue(orderId, out var order)) return;
            
            order.MarkInProgress();
        }

        private IEnumerator RetryFailedOrdersRoutine()
        {
            WaitForSeconds wait = new WaitForSeconds(retryCheckIntervalSeconds);
            
            while (true)
            {
                yield return wait;
                
                ProcessFailedOrders();
            }
        }

        private void ProcessFailedOrders()
        {
            if (failedOrders.Count == 0) return;
            
            DateTime now = DateTime.Now;
            List<Order> ordersToRetry = new List<Order>();
            
            foreach (var order in failedOrders.ToList())
            {
                if (order.LastAttemptTime.HasValue)
                {
                    TimeSpan elapsed = now - order.LastAttemptTime.Value;
                    if (elapsed.TotalSeconds >= retryDelaySeconds)
                    {
                        ordersToRetry.Add(order);
                    }
                }
            }
            
            foreach (var order in ordersToRetry)
            {
                failedOrders.Remove(order);
                order.PrepareForRetry();
                pendingOrders.Add(order);
                Debug.Log($"Order {order.Id} requeued for retry (Attempt {order.RetryCount + 1}/{Order.MaxRetries})");
            }
        }

        public int GetPendingOrderCount()
        {
            return pendingOrders.Count(o => o.Status == OrderStatus.Pending);
        }

        public int GetFailedOrderCount()
        {
            return failedOrders.Count;
        }

        public List<Order> GetFailedOrders()
        {
            return new List<Order>(failedOrders);
        }

        public (int pending, int inProgress, int completed, int failed) GetOrderStatistics()
        {
            int pending = allOrders.Values.Count(o => o.Status == OrderStatus.Pending);
            int inProgress = allOrders.Values.Count(o => o.Status == OrderStatus.InProgress || o.Status == OrderStatus.Assigned);
            int completed = allOrders.Values.Count(o => o.Status == OrderStatus.Completed);
            int failed = allOrders.Values.Count(o => o.Status == OrderStatus.Failed && !o.CanRetry());
            
            return (pending, inProgress, completed, failed);
        }
    }
}