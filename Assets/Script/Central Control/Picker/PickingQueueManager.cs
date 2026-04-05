using System;
using System.Collections.Generic;
using UnityEngine;

namespace CentralControl.PickingQueueControl
{
    public class PickingQueueManager
    {
        private static PickingQueueManager instance;
        private static readonly object _instanceLock = new object();
        public static PickingQueueManager Instance
        {
            get
            {
                if (instance == null)
                {
                    lock (_instanceLock)
                    {
                        instance ??= new PickingQueueManager();
                    }
                }
                return instance;
            }
        }

        // Queue for each picking point: Key = PickingPointId, Value = Queue of RobotIds
        private Dictionary<string, Queue<string>> pickingQueues = new Dictionary<string, Queue<string>>();

        private Dictionary<string, string> activeRobots = new Dictionary<string, string>();

        // Protects all access to pickingQueues and activeRobots
        private readonly object _queueLock = new object();

        // Event to notify robot it can start picking
        public event Action<string> OnRobotAllowedToPick;

        // Event to notify all robots in a queue to advance one position
        // Parameters: pickingPointId, robotId that should advance
        public event Action<string, string> OnQueueAdvance;

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

            lock (_queueLock)
            {
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
        }

        /// <summary>
        /// Called when a robot finishes picking.
        /// </summary>
        public void PickingCompleted(string pickingPointId, string robotId)
        {
            if (string.IsNullOrEmpty(pickingPointId)) return;

            string nextRobotId = null;
            string[] robotsToAdvance = null;

            lock (_queueLock)
            {
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
                    // Get all robots that need to advance before dequeuing
                    robotsToAdvance = pickingQueues[pickingPointId].ToArray();

                    // Dequeue the next robot to pick
                    nextRobotId = pickingQueues[pickingPointId].Dequeue();
                    activeRobots[pickingPointId] = nextRobotId;
                    Debug.Log($"PickingQueueManager: Robot {nextRobotId} promoted to picking at {pickingPointId}");
                }
            }

            // Fire events outside lock to avoid deadlock with event handlers
            if (nextRobotId != null)
            {
                OnRobotAllowedToPick?.Invoke(nextRobotId);

                if (robotsToAdvance != null)
                {
                    for (int i = 1; i < robotsToAdvance.Length; i++)
                    {
                        Debug.Log($"PickingQueueManager: Notifying Robot {robotsToAdvance[i]} to advance in queue at {pickingPointId}");
                        OnQueueAdvance?.Invoke(pickingPointId, robotsToAdvance[i]);
                    }
                }
            }
        }

        /// <summary>
        /// Called when a robot abandons its wait at a picking point (e.g., timeout).
        /// Cleans up queue state without claiming picking was completed.
        /// </summary>
        public void AbandonWait(string pickingPointId, string robotId)
        {
            if (string.IsNullOrEmpty(pickingPointId) || string.IsNullOrEmpty(robotId))
                return;

            string nextRobotId = null;
            string[] robotsToAdvance = null;

            lock (_queueLock)
            {
                // Case 1: Robot is the active robot at this picking point
                if (activeRobots.TryGetValue(pickingPointId, out string activeRobot) && activeRobot == robotId)
                {
                    activeRobots.Remove(pickingPointId);
                    Debug.Log($"PickingQueueManager: Robot {robotId} abandoned wait at {pickingPointId} (was active)");

                    // Promote next robot in queue if any
                    if (pickingQueues.ContainsKey(pickingPointId) && pickingQueues[pickingPointId].Count > 0)
                    {
                        robotsToAdvance = pickingQueues[pickingPointId].ToArray();
                        nextRobotId = pickingQueues[pickingPointId].Dequeue();
                        activeRobots[pickingPointId] = nextRobotId;
                        Debug.Log($"PickingQueueManager: Robot {nextRobotId} promoted to picking at {pickingPointId} (after abandon)");
                    }
                }
                // Case 2: Robot is waiting in the queue
                else if (pickingQueues.ContainsKey(pickingPointId))
                {
                    var queue = pickingQueues[pickingPointId];
                    if (queue.Contains(robotId))
                    {
                        // Rebuild queue without this robot
                        var newQueue = new Queue<string>();
                        foreach (var id in queue)
                        {
                            if (id != robotId)
                                newQueue.Enqueue(id);
                        }
                        pickingQueues[pickingPointId] = newQueue;
                        Debug.Log($"PickingQueueManager: Robot {robotId} removed from queue at {pickingPointId}");
                    }
                }
            }

            // Fire events outside lock to avoid deadlock with event handlers
            if (nextRobotId != null)
            {
                OnRobotAllowedToPick?.Invoke(nextRobotId);

                if (robotsToAdvance != null)
                {
                    for (int i = 1; i < robotsToAdvance.Length; i++)
                    {
                        Debug.Log($"PickingQueueManager: Notifying Robot {robotsToAdvance[i]} to advance in queue at {pickingPointId} (after abandon)");
                        OnQueueAdvance?.Invoke(pickingPointId, robotsToAdvance[i]);
                    }
                }
            }
        }

        public bool IsRobotActive(string pickingPointId, string robotId)
        {
            lock (_queueLock)
            {
                return activeRobots.TryGetValue(pickingPointId, out string active) && active == robotId;
            }
        }

        public bool IsRobotInQueueOrActive(string pickingPointId, string robotId)
        {
            if (string.IsNullOrEmpty(pickingPointId) || string.IsNullOrEmpty(robotId))
                return false;

            lock (_queueLock)
            {
                if (activeRobots.TryGetValue(pickingPointId, out string activeRobot) && activeRobot == robotId)
                    return true;

                if (pickingQueues.TryGetValue(pickingPointId, out var queue))
                    return queue.Contains(robotId);

                return false;
            }
        }

        public void Reset()
        {
            lock (_queueLock)
            {
                pickingQueues.Clear();
                activeRobots.Clear();
            }
        }
    }
}
