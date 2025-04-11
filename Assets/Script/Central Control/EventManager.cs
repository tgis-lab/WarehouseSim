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

    public class RobotPathModificationEventArgs : EventArgs
    {
        public string RobotId { get; private set; }
        public string PlanId { get; private set; }
        public List<ConnectionPoint> Path { get; private set; }
        public List<float> NewTimes { get; private set; }
        public string ModificationType { get; private set; }
        public float ModificationValue { get; private set; }
        
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
        }
    }

    public class EventManager : MonoBehaviour
    {
        public static EventManager Instance { get; private set; }

        public event Action OnMapLoaded;
        public event Action OnRobotsInitialized;

        // Picking-related events
        public event EventHandler<PickingEventArgs> OnRobotReadyForPicking;
        public event EventHandler<PickingEventArgs> OnPickingCompleted;

        // Path-related events
        public event EventHandler<RobotPathEventArgs> OnRobotPathPlanned;
        
        public event EventHandler<RobotPathModificationEventArgs> OnRobotPathModified;
        
        public event EventHandler<string> OnRobotPathAccepted;

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

        public void TriggerMapLoaded()
        {
            OnMapLoaded?.Invoke();
        }

        public void TriggerRobotsInitialized()
        {
            OnRobotsInitialized?.Invoke();
        }

        public void TriggerRobotReadyForPicking(string robotId, PickingPoint pickingPoint, Vector3 location)
        {
            Debug.Log($"EventManager: Robot {robotId} ready for picking at {location}");
            
            // 检查无效的监听器
            if (OnRobotReadyForPicking != null)
            {
                // 获取所有订阅者
                var delegates = OnRobotReadyForPicking.GetInvocationList();
                foreach (var del in delegates)
                {
                    // 检查订阅者的目标对象是否为PickerController
                    PickerController picker = del.Target as PickerController;
                    if (picker != null)
                    {
                        Debug.Log($"EventManager: 事件将通知 Picker {picker.Id}");
                        
                        // 检查无效的Picker (ID为0或已销毁的GameObject)
                        if (picker.Id == 0 || picker == null || picker.gameObject == null)
                        {
                            Debug.LogWarning($"EventManager: 检测到无效的Picker监听器 (ID: {picker.Id})，自动移除");
                            OnRobotReadyForPicking -= (EventHandler<PickingEventArgs>)del;
                        }
                    }
                }
            }
            
            OnRobotReadyForPicking?.Invoke(this, new PickingEventArgs(robotId, pickingPoint, location));
        }

        public void TriggerPickingCompleted(string robotId, PickingPoint pickingPoint, Vector3 location)
        {
            OnPickingCompleted?.Invoke(this, new PickingEventArgs(robotId, pickingPoint, location));
        }

        public void TriggerRobotPathPlanned(string robotId, string planId, List<ConnectionPoint> path, List<float> times)
        {
            Debug.Log($"EventManager: Robot {robotId} planned a path with {path.Count} points");
            OnRobotPathPlanned?.Invoke(this, new RobotPathEventArgs(robotId, planId, path, times));
        }
        
        public void TriggerRobotPathModified(RobotPathModificationEventArgs args)
        {
            Debug.Log($"EventManager: 触发机器人路径修改事件: 机器人 {args.RobotId}, 类型 {args.ModificationType}, 值 {args.ModificationValue}");
            OnRobotPathModified?.Invoke(this, args);
        }
        
        public void TriggerRobotPathAccepted(string robotId, string planId)
        {
            Debug.Log($"EventManager: 机器人 {robotId} 的路径 {planId} 已被接受");
            OnRobotPathAccepted?.Invoke(this, robotId);
        }
    }
}