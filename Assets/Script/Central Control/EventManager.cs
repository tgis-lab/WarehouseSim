using System;
using UnityEngine;
using System.Collections.Generic;
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

    public class EventManager : MonoBehaviour
    {
        public static EventManager Instance { get; private set; }

        // System events
        public event Action OnMapLoaded;
        public event Action OnRobotsInitialized;
        public event Action OnPickersInitialized; // Added for consistency

        // Picking-related events
        public event EventHandler<PickingEventArgs> OnRobotReadyForPicking;
        public event EventHandler<PickingEventArgs> OnPickingCompleted;

        // Path-related events
        public event EventHandler<RobotPathEventArgs> OnRobotPathPlanned;
        public event EventHandler<RobotPathModificationEventArgs> OnRobotPathModified;
        public event EventHandler<RobotPathAcceptedEventArgs> OnRobotPathAccepted;

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
    }
}
