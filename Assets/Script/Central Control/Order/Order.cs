using System;
using UnityEngine;
using Map;

namespace CentralControl.OrderAssignment
{
    public enum OrderStatus
    {
        Pending,
        Assigned,
        InProgress,
        Completed,
        Failed,
        Cancelled
    }

    public class Order
    {
        public string Id { get; private set; }
        public Vector3 Destination { get; private set; }
        public PickingPoint PickingPoint { get; private set; }
        public DateTime CreationTime { get; private set; }
        public DateTime Priority => CreationTime;
        public float ExecutionTime { get; private set; }
        public int? AssignedRobotId { get; private set; }
        public string FloorLevel { get; private set; }
        
        public OrderStatus Status { get; private set; } = OrderStatus.Pending;
        public int RetryCount { get; private set; } = 0;
        public string LastFailureReason { get; private set; }
        public DateTime? LastAttemptTime { get; private set; }
        
        public int PriorityLevel { get; set; } = 1;

        public const int MaxRetries = 3;

        public Order(string id, Vector3 destination, PickingPoint pickingPoint, float executionTime, string floorLevel)
        {
            if (string.IsNullOrEmpty(id))
            {
                throw new ArgumentNullException(nameof(id), "Order id cannot be null or empty.");
            }

            if (destination == null)
            {
                throw new ArgumentNullException(nameof(destination), "Order destination cannot be null.");
            }

            if (pickingPoint == null)
            {
                throw new ArgumentNullException(nameof(pickingPoint), "Order picking point cannot be null.");
            }
            
            this.Id = id;
            this.Destination = destination;
            this.PickingPoint = pickingPoint;
            this.CreationTime = DateTime.Now;
            this.ExecutionTime = executionTime;
            this.AssignedRobotId = null;
            this.FloorLevel = floorLevel ?? pickingPoint.FloorLevel;
        }

        public void MarkAssigned(int robotId)
        {
            AssignedRobotId = robotId;
            Status = OrderStatus.Assigned;
            LastAttemptTime = DateTime.Now;
        }

        public void ResetAssignment()
        {
            AssignedRobotId = null;
            Status = OrderStatus.Pending;
        }

        public void MarkInProgress()
        {
            Status = OrderStatus.InProgress;
        }

        public void MarkCompleted()
        {
            Status = OrderStatus.Completed;
        }

        public void MarkFailed(string reason)
        {
            Status = OrderStatus.Failed;
            LastFailureReason = reason;
            RetryCount++;
            AssignedRobotId = null;
        }

        public void MarkCancelled()
        {
            Status = OrderStatus.Cancelled;
        }

        public bool CanRetry()
        {
            return Status == OrderStatus.Failed && RetryCount < MaxRetries;
        }

        public void PrepareForRetry()
        {
            if (CanRetry())
            {
                Status = OrderStatus.Pending;
            }
        }
        
        public override string ToString()
        {
            return $"Order {Id} [Floor: {FloorLevel}, Point: {PickingPoint?.Id ?? "unknown"}, Status: {Status}]";
        }
    }
}