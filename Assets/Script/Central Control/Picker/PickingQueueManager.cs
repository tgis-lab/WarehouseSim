using System;
using System.Collections.Generic;
using UnityEngine;

namespace CentralControl.PickingQueueControl
{
    public class PickingQueueManager
    {
        private static PickingQueueManager instance;
        public static PickingQueueManager Instance => instance ??= new PickingQueueManager();

        // Queue for each picking point: Key = PickingPointId, Value = Queue of RobotIds
        private Dictionary<string, Queue<string>> pickingQueues = new Dictionary<string, Queue<string>>();
        
        private Dictionary<string, string> activeRobots = new Dictionary<string, string>();

        // Event to notify robot it can start picking
        public event Action<string> OnRobotAllowedToPick;

        private PickingQueueManager() { }

        /// <summary>
        /// Registers a robot's arrival at a picking point.
        /// </summary>
        /// <param name="pickingPointId">The picking point ID</param>
        /// <param name="robotId">The robot ID</param>
        /// <returns>True if robot can pick immediately, False if it must queue</returns>
        public bool RegisterArrival(string pickingPointId, string robotId)
        {
            if (string.IsNullOrEmpty(pickingPointId) || string.IsNullOrEmpty(robotId))
                return false;

            if (!pickingQueues.ContainsKey(pickingPointId))
            {
                pickingQueues[pickingPointId] = new Queue<string>();
            }

            if (!activeRobots.ContainsKey(pickingPointId))
            {
                activeRobots[pickingPointId] = robotId;
                Debug.Log($"PickingQueueManager: Robot {robotId} started picking at {pickingPointId} (Immediate)");
                return true;
            }
            
            if (!pickingQueues[pickingPointId].Contains(robotId) && activeRobots[pickingPointId] != robotId)
            {
                 pickingQueues[pickingPointId].Enqueue(robotId);
                 Debug.Log($"PickingQueueManager: Robot {robotId} queued at {pickingPointId}. Position: {pickingQueues[pickingPointId].Count}");
            }
            return false;
        }

        /// <summary>
        /// Called when a robot finishes picking.
        /// </summary>
        public void PickingCompleted(string pickingPointId, string robotId)
        {
            if (string.IsNullOrEmpty(pickingPointId)) return;

            if (!activeRobots.TryGetValue(pickingPointId, out string activeRobot))
                return;
                
            if (activeRobot != robotId)
            {
                Debug.LogWarning($"PickingQueueManager: Robot {robotId} tried to complete picking at {pickingPointId} but active robot is {activeRobot}");
                return;
            }
            
            activeRobots.Remove(pickingPointId);
            Debug.Log($"PickingQueueManager: Robot {robotId} finished picking at {pickingPointId}");
            
            if (pickingQueues.ContainsKey(pickingPointId) && pickingQueues[pickingPointId].Count > 0)
            {
                string nextRobotId = pickingQueues[pickingPointId].Dequeue();
                activeRobots[pickingPointId] = nextRobotId;
                Debug.Log($"PickingQueueManager: Robot {nextRobotId} promoted to picking at {pickingPointId}");
                OnRobotAllowedToPick?.Invoke(nextRobotId);
            }
        }
        
        public bool IsRobotActive(string pickingPointId, string robotId)
        {
             return activeRobots.TryGetValue(pickingPointId, out string active) && active == robotId;
        }

        public bool IsRobotInQueueOrActive(string pickingPointId, string robotId)
        {
            if (string.IsNullOrEmpty(pickingPointId) || string.IsNullOrEmpty(robotId))
                return false;

            if (activeRobots.TryGetValue(pickingPointId, out string activeRobot) && activeRobot == robotId)
                return true;

            if (pickingQueues.TryGetValue(pickingPointId, out var queue))
                return queue.Contains(robotId);

            return false;
        }

        public void Reset()
        {
            pickingQueues.Clear();
            activeRobots.Clear();
        }
    }
}
