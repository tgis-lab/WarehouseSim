using System;
using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using Map;
using CentralControl.RobotControl;
using CentralControl.PickerControl;

namespace CentralControl
{
    public class PickingEventArgs : EventArgs
    {
        public string RobotId { get; private set; }
        public PickingPoint PickingPoint { get; private set; }
        public Vector3 Location { get; private set; }

        public PickingEventArgs(string robotId, PickingPoint pickingPoint, Vector3 location)
        {
            RobotId = robotId;
            PickingPoint = pickingPoint;
            Location = location;
        }
    }

    public class RobotPathEventArgs : EventArgs
    {
        public string RobotId { get; private set; }
        public string PlanId { get; private set; }
        public List<ConnectionPoint> Path { get; private set; }
        public List<float> Times { get; private set; }
        public List<float> Speeds { get; set; }
        
        public RobotPathEventArgs(string robotId, string planId, List<ConnectionPoint> path, List<float> times, List<float> speeds)
        {
            RobotId = robotId;
            PlanId = planId;
            Path = path;
            Times = times;
            Speeds = speeds;
        }
    }

    public class RobotPathAcceptedEventArgs : EventArgs
    {
        public string RobotId { get; private set; }
        public string PlanId { get; private set; }
        public List<ConnectionPoint> Path { get; private set; }
        public List<float> Times { get; private set; }
        public List<float> Speeds { get; set; }
        
        public RobotPathAcceptedEventArgs(string robotId, string planId, List<ConnectionPoint> path, List<float> times, List<float> speeds)
        {
            RobotId = robotId;
            PlanId = planId;
            Path = path;
            Times = times;
            Speeds = speeds;
        }
    }

    public class RobotPathModificationEventArgs : EventArgs
    {
        public string RobotId { get; private set; }
        public string PlanId { get; private set; }
        public List<ConnectionPoint> Path { get; private set; }
        public List<float> NewTimes { get; private set; }
        public string ModificationType { get; private set; }
        public float ModificationValue { get; private set; }
        public List<float> NewSpeeds { get; set; }
        
        public RobotPathModificationEventArgs(
            string robotId, string planId, List<ConnectionPoint> path, 
            List<float> newTimes, string modificationType, float modificationValue)
        {
            RobotId = robotId;
            PlanId = planId;
            Path = path;
            NewTimes = newTimes;
            ModificationType = modificationType;
            ModificationValue = modificationValue;
            NewSpeeds = new List<float>();
        }
    }

    public class GlobalOccupancyLayerEventArgs : EventArgs
    {
        public DynamicOccupancyLayer GlobalLayer { get; private set; }
        public float UpdateTime { get; private set; }

        public GlobalOccupancyLayerEventArgs(DynamicOccupancyLayer globalLayer, float updateTime)
        {
            GlobalLayer = globalLayer;
            UpdateTime = updateTime;
        }
    }

    public class CollisionEventArgs : EventArgs
    {
        public string CollisionId { get; private set; }
        public string RobotId1 { get; private set; }
        public string RobotId2 { get; private set; }
        public float Time { get; private set; }
        public bool IsRobotCollision { get; private set; }
        public GameObject CollisionObject { get; private set; }

        public CollisionEventArgs(
            string collisionId, 
            string robotId1, 
            string robotId2, 
            float time, 
            bool isRobotCollision,
            GameObject collisionObject = null)
        {
            CollisionId = collisionId;
            RobotId1 = robotId1;
            RobotId2 = robotId2;
            Time = time;
            IsRobotCollision = isRobotCollision;
            CollisionObject = collisionObject;
        }
    }

    public class EventManager : MonoBehaviour
    {
        public static EventManager Instance { get; private set; }

        // System events
        public event Action OnMapLoaded;
        public event Action OnRobotsInitialized;
        public event Action OnPickersInitialized;

        // Global occupancy layer events
        public event EventHandler<GlobalOccupancyLayerEventArgs> OnGlobalOccupancyLayerUpdated;

        // Picking-related events
        public event EventHandler<PickingEventArgs> OnRobotReadyForPicking;
        public event EventHandler<PickingEventArgs> OnPickingCompleted;

        // Path-related events
        public event EventHandler<RobotPathEventArgs> OnRobotPathPlanned;
        public event EventHandler<RobotPathModificationEventArgs> OnRobotPathModified;
        public event EventHandler<RobotPathAcceptedEventArgs> OnRobotPathAccepted;

        // Collision-related events
        public event EventHandler<CollisionEventArgs> OnRobotCollisionDetected;
        
        // Collision records
        private Dictionary<string, CollisionEventArgs> robotCollisions = new Dictionary<string, CollisionEventArgs>();
        private Dictionary<string, CollisionEventArgs> environmentCollisions = new Dictionary<string, CollisionEventArgs>();

        private void Awake()
        {
            if (Instance == null)
            {
                Instance = this;
                DontDestroyOnLoad(gameObject);
            }
            else
            {
                Destroy(gameObject);
            }
        }

        // System triggers
        public void TriggerMapLoaded()
        {
            OnMapLoaded?.Invoke();
        }

        public void TriggerRobotsInitialized()
        {
            OnRobotsInitialized?.Invoke();
        }
        
        public void TriggerPickersInitialized()
        {
            OnPickersInitialized?.Invoke();
        }

        // Global occupancy layer triggers
        public void TriggerGlobalOccupancyLayerUpdated(DynamicOccupancyLayer globalLayer, float updateTime)
        {
            OnGlobalOccupancyLayerUpdated?.Invoke(this, new GlobalOccupancyLayerEventArgs(globalLayer, updateTime));
        }

        // Picking triggers
        public void TriggerRobotReadyForPicking(string robotId, PickingPoint pickingPoint, Vector3 location)
        {
            Debug.Log($"EventManager: Robot {robotId} ready for picking at {location}");
            
            // Check for invalid listeners
            if (OnRobotReadyForPicking != null)
            {
                var delegates = OnRobotReadyForPicking.GetInvocationList();
                foreach (var del in delegates)
                {
                    PickerController picker = del.Target as PickerController;
                    if (picker != null)
                    {
                        Debug.Log($"EventManager: Event will notify Picker {picker.Id}");
                        
                        // Check for invalid Picker (ID=0 or destroyed GameObject)
                        if (picker.Id == 0 || picker == null || picker.gameObject == null)
                        {
                            Debug.LogWarning($"EventManager: Detected invalid Picker listener (ID: {picker.Id}), auto-removing");
                            OnRobotReadyForPicking -= (EventHandler<PickingEventArgs>)del;
                        }
                    }
                }
            }
            
            OnRobotReadyForPicking?.Invoke(this, new PickingEventArgs(robotId, pickingPoint, location));
        }

        public void TriggerPickingCompleted(string robotId, PickingPoint pickingPoint, Vector3 location)
        {
            Debug.Log($"EventManager: Picking completed for robot {robotId} at {location}");
            OnPickingCompleted?.Invoke(this, new PickingEventArgs(robotId, pickingPoint, location));
        }

        // Path triggers
        public void TriggerRobotPathPlanned(string robotId, string planId, List<ConnectionPoint> path, List<float> times, List<float> speeds)
        {
            Debug.Log($"EventManager: Robot {robotId} planned a path with {path.Count} points");
            OnRobotPathPlanned?.Invoke(this, new RobotPathEventArgs(robotId, planId, path, times, speeds));
        }
        
        public void TriggerRobotPathModified(RobotPathModificationEventArgs args)
        {
            Debug.Log($"EventManager: Triggered robot path modification event: Robot {args.RobotId}, Type {args.ModificationType}, Value {args.ModificationValue}");
            OnRobotPathModified?.Invoke(this, args);
        }
        
        public void TriggerRobotPathAccepted(string robotId, string planId, List<ConnectionPoint> path, List<float> times, List<float> speeds)
        {
            Debug.Log($"EventManager: Robot {robotId}'s path {planId} has been accepted, with {path.Count} points");
            OnRobotPathAccepted?.Invoke(this, new RobotPathAcceptedEventArgs(robotId, planId, path, times, speeds));
        }

        // Collision triggers
        public void ReportRobotCollision(string robotId1, string robotId2, GameObject collisionObject, float time)
        {
            // Unique ID for the collision
            // Sort the robot IDs to ensure consistent ordering
            string[] robotIds = new string[] { robotId1, robotId2 };
            Array.Sort(robotIds);
            string collisionId = $"{robotIds[0]}_collided_with_{robotIds[1]}_{time:F5}";
            
            // If already recorded, do not process again
            if (robotCollisions.ContainsKey(collisionId))
            {
                return;
            }
            
            Debug.LogWarning($"EventManager: Reported collision between Robot {robotId1} and Robot {robotId2}, ID: {collisionId}");
            
            CollisionEventArgs args = new CollisionEventArgs(
                collisionId,
                robotId1,
                robotId2,
                time,
                true,
                collisionObject
            );
            
            robotCollisions[collisionId] = args;
            
            OnRobotCollisionDetected?.Invoke(this, args);
        }

        public void ReportEnvironmentCollision(string robotId, string environmentObjectId, GameObject collisionObject, float time)
        {
            string collisionId = $"{robotId}_collided_with_env_{environmentObjectId}_{time:F5}";
            
            if (environmentCollisions.ContainsKey(collisionId))
            {
                return;
            }
            
            Debug.LogWarning($"EventManager: Reported collision between Robot {robotId} and Environment {environmentObjectId}, ID: {collisionId}");
            
            CollisionEventArgs args = new CollisionEventArgs(
                collisionId,
                robotId,
                environmentObjectId,
                time,
                false,
                collisionObject
            );
            
            environmentCollisions[collisionId] = args;
            
            OnRobotCollisionDetected?.Invoke(this, args);
        }

        public List<CollisionEventArgs> GetAllRobotCollisions()
        {
            return robotCollisions.Values.ToList();
        }

        public List<CollisionEventArgs> GetAllEnvironmentCollisions()
        {
            return environmentCollisions.Values.ToList();
        }

        public List<CollisionEventArgs> GetRobotCollisions(string robotId)
        {
            List<CollisionEventArgs> result = new List<CollisionEventArgs>();
            
            foreach (var record in robotCollisions.Values)
            {
                if (record.RobotId1 == robotId || record.RobotId2 == robotId)
                {
                    result.Add(record);
                }
            }
            
            foreach (var record in environmentCollisions.Values)
            {
                if (record.RobotId1 == robotId)
                {
                    result.Add(record);
                }
            }
            
            return result;
        }

        public (int totalRobotCollisions, int totalEnvironmentCollisions, Dictionary<string, int> robotCollisionCounts) GetCollisionStatistics()
        {
            Dictionary<string, int> robotCounts = new Dictionary<string, int>();
            
            foreach (var collision in robotCollisions.Values)
            {
                if (!robotCounts.ContainsKey(collision.RobotId1))
                {
                    robotCounts[collision.RobotId1] = 0;
                }
                robotCounts[collision.RobotId1]++;
                
                if (!robotCounts.ContainsKey(collision.RobotId2))
                {
                    robotCounts[collision.RobotId2] = 0;
                }
                robotCounts[collision.RobotId2]++;
            }
            
            foreach (var collision in environmentCollisions.Values)
            {
                if (!robotCounts.ContainsKey(collision.RobotId1))
                {
                    robotCounts[collision.RobotId1] = 0;
                }
                robotCounts[collision.RobotId1]++;
            }
            
            return (robotCollisions.Count, environmentCollisions.Count, robotCounts);
        }

        public void ClearCollisionRecords()
        {
            robotCollisions.Clear();
            environmentCollisions.Clear();
            
            Debug.Log("EventManager: All collision records have been cleared");
        }
    }
}