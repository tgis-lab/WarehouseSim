using System;
using UnityEngine;
using Map;

namespace CentralControl.OrderAssignment
{
    public class Order
    {
        public string Id { get; private set; }
        public Vector3 Destination { get; private set; }
        public PickingPoint PickingPoint { get; private set; }
        public DateTime CreationTime { get; private set; }
        public DateTime Priority => CreationTime;  // 优先级 -> 创建时间
        public float ExecutionTime { get; private set; }
        public int? AssignedRobotId { get; set; }

        public Order(string id, Vector3 destination, PickingPoint pickingPoint, float executionTime)
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
        }
    }
}
