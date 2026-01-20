using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Map;
using CentralControl.RobotControl;

namespace CentralControl
{
    public enum CongestionLevel
    {
        None,
        Low,
        Medium,
        High,
        Deadlock
    }

    public class DeadlockDetector : MonoBehaviour
    {
        #region Singleton
        public static DeadlockDetector Instance { get; private set; }
        
        private void Awake()
        {
            if (Instance != null && Instance != this)
            {
                Destroy(gameObject);
                return;
            }
            Instance = this;
        }
        #endregion
        
        #region Configuration
        [Header("Detection Settings")]
        [Tooltip("Time a robot must be emergency stopped before considered deadlocked")]
        [SerializeField] private float deadlockTimeThreshold = 5f;
        
        [Tooltip("Interval between detection cycles (seconds)")]
        [SerializeField] private float detectionInterval = 1f;
        
        [Tooltip("Cooldown between deadlock notifications for same robot")]
        [SerializeField] private float notificationCooldown = 30f;
        
        [Tooltip("Distance threshold to group nearby deadlocked robots")]
        [SerializeField] private float groupingDistance = 5f;
        #endregion
        
        #region Private Fields
        private RobotManager robotManager;
        private Coroutine detectionCoroutine;
        private bool isInitialized = false;
        
        private Dictionary<int, float> robotEmergencyStopStartTime = new Dictionary<int, float>();
        private Dictionary<int, float> lastNotificationTime = new Dictionary<int, float>();
        #endregion

        #region Initialization
        public void Initialize(DynamicOccupancyLayer globalLayer, IndoorSpace space)
        {
            robotManager = FindObjectOfType<RobotManager>();
            
            if (robotManager == null)
            {
                Debug.LogError("[DeadlockDetector] RobotManager not found!");
                return;
            }
            
            isInitialized = true;
            
            if (detectionCoroutine != null)
            {
                StopCoroutine(detectionCoroutine);
            }
            detectionCoroutine = StartCoroutine(DeadlockDetectionRoutine());
            
            Debug.Log("[DeadlockDetector] Initialized - detecting emergency stopped robots");
        }

        private void OnDestroy()
        {
            if (detectionCoroutine != null)
            {
                StopCoroutine(detectionCoroutine);
            }
        }
        #endregion
        
        #region Detection Logic
        private IEnumerator DeadlockDetectionRoutine()
        {
            var wait = new WaitForSeconds(detectionInterval);
            
            while (true)
            {
                yield return wait;
                
                if (!isInitialized || robotManager == null)
                    continue;
                
                var allRobots = robotManager.GetAllRobots();
                if (allRobots == null || allRobots.Count == 0)
                    continue;
                
                DetectDeadlockedRobots(allRobots);
            }
        }
        
        private void DetectDeadlockedRobots(List<RobotController> robots)
        {
            float currentTime = Time.time;
            var deadlockedRobots = new List<RobotController>();
            
            foreach (var robot in robots)
            {
                if (robot == null) continue;
                
                bool isValidDeadlockCandidate = robot.IsEmergencyStopped && 
                                                 !robot.IsPicking && 
                                                 !robot.IsQueuing &&
                                                 robot.IsOnTask;
                
                if (isValidDeadlockCandidate)
                {
                    if (!robotEmergencyStopStartTime.ContainsKey(robot.Id))
                    {
                        robotEmergencyStopStartTime[robot.Id] = currentTime;
                    }
                    
                    float stoppedDuration = currentTime - robotEmergencyStopStartTime[robot.Id];
                    
                    if (stoppedDuration >= deadlockTimeThreshold)
                    {
                        deadlockedRobots.Add(robot);
                    }
                }
                else
                {
                    robotEmergencyStopStartTime.Remove(robot.Id);
                }
            }
            
            if (deadlockedRobots.Count == 0)
                return;
            
            var groups = GroupNearbyRobots(deadlockedRobots);
            
            foreach (var group in groups)
            {
                NotifyDeadlockGroup(group, currentTime);
            }
        }
        
        private List<List<RobotController>> GroupNearbyRobots(List<RobotController> robots)
        {
            var groups = new List<List<RobotController>>();
            var assigned = new HashSet<int>();
            
            foreach (var robot in robots)
            {
                if (assigned.Contains(robot.Id))
                    continue;
                
                var group = new List<RobotController> { robot };
                assigned.Add(robot.Id);
                
                foreach (var other in robots)
                {
                    if (assigned.Contains(other.Id))
                        continue;
                    
                    float distance = Vector3.Distance(robot.transform.position, other.transform.position);
                    bool sameFloor = Mathf.Abs(robot.transform.position.y - other.transform.position.y) < 1f;
                    
                    if (distance <= groupingDistance && sameFloor)
                    {
                        group.Add(other);
                        assigned.Add(other.Id);
                    }
                }
                
                groups.Add(group);
            }
            
            return groups;
        }
        
        private void NotifyDeadlockGroup(List<RobotController> group, float currentTime)
        {
            bool shouldNotify = false;
            
            foreach (var robot in group)
            {
                if (!lastNotificationTime.TryGetValue(robot.Id, out float lastTime) ||
                    currentTime - lastTime >= notificationCooldown)
                {
                    shouldNotify = true;
                    break;
                }
            }
            
            if (!shouldNotify)
                return;
            
            var robotIds = group.Select(r => r.Id).ToList();
            var robotIdStrings = robotIds.Select(id => id.ToString()).ToArray();
            
            foreach (var robot in group)
            {
                float stoppedDuration = currentTime - robotEmergencyStopStartTime[robot.Id];
                lastNotificationTime[robot.Id] = currentTime;
                
                Debug.LogWarning($"[DeadlockDetector] Robot {robot.Id} deadlocked - " +
                               $"EmergencyStopped for {stoppedDuration:F1}s at {robot.transform.position}");
            }
            
            if (group.Count > 1)
            {
                Debug.LogWarning($"[DeadlockDetector] DEADLOCK GROUP: Robots {string.Join(", ", robotIds)} " +
                               $"are all stuck near each other");
            }
            
            TriggerDeadlockEvent(robotIdStrings);
            TriggerRecoveryRequest(robotIds);
        }
        
        private void TriggerDeadlockEvent(string[] robotIds)
        {
            EventManager.Instance?.TriggerDeadlockDetected(robotIds);
        }
        
        private void TriggerRecoveryRequest(List<int> robotIds)
        {
            EventManager.Instance?.TriggerRequestGlobalMapUpdate(robotIds.ToArray());
        }
        #endregion
        
        #region Public API
        public List<int> GetDeadlockedRobotIds()
        {
            float currentTime = Time.time;
            var result = new List<int>();
            
            foreach (var kvp in robotEmergencyStopStartTime)
            {
                if (currentTime - kvp.Value >= deadlockTimeThreshold)
                {
                    result.Add(kvp.Key);
                }
            }
            
            return result;
        }
        
        public bool IsRobotDeadlocked(int robotId)
        {
            if (!robotEmergencyStopStartTime.TryGetValue(robotId, out float startTime))
                return false;
            
            return Time.time - startTime >= deadlockTimeThreshold;
        }
        
        public float GetRobotStoppedDuration(int robotId)
        {
            if (!robotEmergencyStopStartTime.TryGetValue(robotId, out float startTime))
                return 0f;
            
            return Time.time - startTime;
        }
        #endregion
    }
}
