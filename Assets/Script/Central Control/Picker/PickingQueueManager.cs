using System;
using System.Collections.Generic;
using UnityEngine;

namespace CentralControl.PickerControl
{
    public class PickingQueueManager
    {
        private static PickingQueueManager instance;
        public static PickingQueueManager Instance => instance ??= new PickingQueueManager();

        // Queue for each picking point: Key = PickingPointId, Value = Queue of RobotIds
        private Dictionary<string, Queue<string>> pickingQueues = new Dictionary<string, Queue<string>>();
        
        // Current picker for each picking point: Key = PickingPointId, Value = RobotId
        private Dictionary<string, string> currentPickers = new Dictionary<string, string>();

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

            // Check if picking point is free
            if (!currentPickers.ContainsKey(pickingPointId))
            {
                // Picking point is free, this robot becomes the picker
                currentPickers[pickingPointId] = robotId;
                Debug.Log($"PickingQueueManager: Robot {robotId} started picking at {pickingPointId} (Immediate)");
                return true;
            }
            else
            {
                // Picking point is busy, add to queue
                // Avoid adding duplicates (Queue enumeration is O(N) but N is small)
                if (!pickingQueues[pickingPointId].Contains(robotId) && currentPickers[pickingPointId] != robotId)
                {
                     pickingQueues[pickingPointId].Enqueue(robotId);
                     Debug.Log($"PickingQueueManager: Robot {robotId} queued at {pickingPointId}. Position: {pickingQueues[pickingPointId].Count}");
                }
                return false;
            }
        }

        /// <summary>
        /// Called when a robot finishes picking.
        /// </summary>
        public void PickingCompleted(string pickingPointId, string robotId)
        {
            if (string.IsNullOrEmpty(pickingPointId)) return;

            if (currentPickers.TryGetValue(pickingPointId, out string currentPicker))
            {
                if (currentPicker == robotId)
                {
                    currentPickers.Remove(pickingPointId);
                    Debug.Log($"PickingQueueManager: Robot {robotId} finished picking at {pickingPointId}");
                    
                    // Process next in queue
                    if (pickingQueues.ContainsKey(pickingPointId) && pickingQueues[pickingPointId].Count > 0)
                    {
                        string nextRobotId = pickingQueues[pickingPointId].Dequeue();
                        currentPickers[pickingPointId] = nextRobotId;
                        Debug.Log($"PickingQueueManager: Robot {nextRobotId} promoted to picking at {pickingPointId}");
                        OnRobotAllowedToPick?.Invoke(nextRobotId);
                    }
                }
                else
                {
                    Debug.LogWarning($"PickingQueueManager: Robot {robotId} tried to complete picking at {pickingPointId} but current picker is {currentPicker}");
                }
            }
        }
        
        /// <summary>
        /// Checks if a robot is currently the designated picker for a point.
        /// </summary>
        public bool IsRobotPicking(string pickingPointId, string robotId)
        {
             return currentPickers.TryGetValue(pickingPointId, out string picker) && picker == robotId;
        }

        public void Reset()
        {
            pickingQueues.Clear();
            currentPickers.Clear();
        }
    }
}
