using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace CentralControl
{
    public class SimulationMetricsCollector : MonoBehaviour
    {
        public static SimulationMetricsCollector Instance { get; private set; }

        [Header("Collection Settings")]
        [SerializeField] private float snapshotIntervalSeconds = 30f;
        [SerializeField] private float autoSaveIntervalSeconds = 60f;

        private float simulationStartTime;
        private int totalOrdersGenerated;
        private int totalOrdersCompleted;
        private int totalOrdersFailed;
        private int totalDeadlocks;
        private float totalWaitTime;
        private int waitEventCount;
        private Dictionary<string, RobotMetrics> robotMetrics = new Dictionary<string, RobotMetrics>();
        private Dictionary<string, OrderMetrics> orderMetrics = new Dictionary<string, OrderMetrics>();
        private List<SnapshotData> snapshots = new List<SnapshotData>();

        private class RobotMetrics
        {
            public float TotalActiveTime;
            public float TotalIdleTime;
            public float LastStateChangeTime;
            public bool IsActive;
            public int CollisionCount;
            public int ReplanCount;
            public float TotalDistanceTraveled;
        }

        private class OrderMetrics
        {
            public float AssignmentTime;
            public float CompletionTime;
            public string RobotId;
            public bool IsCompleted;
        }

        private class SnapshotData
        {
            public float Time;
            public int ActiveRobots;
            public int IdleRobots;
            public int PendingOrders;
            public int CompletedOrders;
            public float AverageUtilization;
        }

        private void Awake()
        {
            if (Instance != null && Instance != this)
            {
                Destroy(gameObject);
                return;
            }
            Instance = this;
        }

        private void Start()
        {
            simulationStartTime = Time.time;
            SubscribeToEvents();
            StartCoroutine(SnapshotRoutine());
            StartCoroutine(AutoSaveRoutine());
        }

        private void OnDestroy()
        {
            UnsubscribeFromEvents();
        }

        private void SubscribeToEvents()
        {
            if (EventManager.Instance == null) return;

            EventManager.Instance.OnRobotStateChanged += HandleRobotStateChanged;
            EventManager.Instance.OnOrderAssigned += HandleOrderAssigned;
            EventManager.Instance.OnOrderCompleted += HandleOrderCompleted;
            EventManager.Instance.OnOrderFailed += HandleOrderFailed;
            EventManager.Instance.OnCollisionRecoveryCompleted += HandleCollisionRecoveryCompleted;
            EventManager.Instance.OnDeadlockDetected += HandleDeadlockDetected;
            EventManager.Instance.OnRobotWaitStarted += HandleRobotWaitStarted;
            EventManager.Instance.OnRobotWaitEnded += HandleRobotWaitEnded;
        }

        private void UnsubscribeFromEvents()
        {
            if (EventManager.Instance == null) return;

            EventManager.Instance.OnRobotStateChanged -= HandleRobotStateChanged;
            EventManager.Instance.OnOrderAssigned -= HandleOrderAssigned;
            EventManager.Instance.OnOrderCompleted -= HandleOrderCompleted;
            EventManager.Instance.OnOrderFailed -= HandleOrderFailed;
            EventManager.Instance.OnCollisionRecoveryCompleted -= HandleCollisionRecoveryCompleted;
            EventManager.Instance.OnDeadlockDetected -= HandleDeadlockDetected;
            EventManager.Instance.OnRobotWaitStarted -= HandleRobotWaitStarted;
            EventManager.Instance.OnRobotWaitEnded -= HandleRobotWaitEnded;
        }

        private void HandleRobotStateChanged(object sender, RobotStateChangedEventArgs e)
        {
            EnsureRobotMetrics(e.RobotId);
            var metrics = robotMetrics[e.RobotId];

            float elapsed = e.Time - metrics.LastStateChangeTime;
            if (metrics.IsActive)
                metrics.TotalActiveTime += elapsed;
            else
                metrics.TotalIdleTime += elapsed;

            metrics.IsActive = e.StateName == "Moving" && e.NewValue;
            metrics.LastStateChangeTime = e.Time;
        }

        private void HandleOrderAssigned(object sender, OrderAssignedEventArgs e)
        {
            totalOrdersGenerated++;
            orderMetrics[e.OrderId] = new OrderMetrics
            {
                AssignmentTime = e.AssignmentTime,
                RobotId = e.RobotId,
                IsCompleted = false
            };
        }

        private void HandleOrderCompleted(string orderId, string robotId)
        {
            totalOrdersCompleted++;
            if (orderMetrics.TryGetValue(orderId, out var metrics))
            {
                metrics.CompletionTime = Time.time;
                metrics.IsCompleted = true;
            }
        }

        private void HandleOrderFailed(string orderId, string robotId, string reason)
        {
            totalOrdersFailed++;
        }

        private void HandleCollisionRecoveryCompleted(object sender, CollisionRecoveryCompletedEventArgs e)
        {
            EnsureRobotMetrics(e.RobotId1);
            EnsureRobotMetrics(e.RobotId2);
            robotMetrics[e.RobotId1].CollisionCount++;
            robotMetrics[e.RobotId2].CollisionCount++;
        }

        private void HandleDeadlockDetected(object sender, DeadlockDetectedEventArgs e)
        {
            totalDeadlocks++;
        }

        private void HandleRobotWaitStarted(object sender, RobotWaitEventArgs e)
        {
            waitEventCount++;
        }

        private void HandleRobotWaitEnded(object sender, RobotWaitEventArgs e)
        {
        }

        private void EnsureRobotMetrics(string robotId)
        {
            if (!robotMetrics.ContainsKey(robotId))
            {
                robotMetrics[robotId] = new RobotMetrics
                {
                    LastStateChangeTime = Time.time,
                    IsActive = false
                };
            }
        }

        private IEnumerator SnapshotRoutine()
        {
            WaitForSeconds wait = new WaitForSeconds(snapshotIntervalSeconds);
            while (true)
            {
                yield return wait;
                TakeSnapshot();
            }
        }

        private void TakeSnapshot()
        {
            int activeCount = robotMetrics.Values.Count(r => r.IsActive);
            int idleCount = robotMetrics.Count - activeCount;
            int pendingOrders = totalOrdersGenerated - totalOrdersCompleted - totalOrdersFailed;
            float simulationTime = Time.time - simulationStartTime;

            float avgUtilization = 0f;
            if (robotMetrics.Count > 0)
            {
                avgUtilization = robotMetrics.Values
                    .Select(r => r.TotalActiveTime / Mathf.Max(r.TotalActiveTime + r.TotalIdleTime, 0.001f))
                    .Average();
            }

            float throughput = simulationTime > 0 ? totalOrdersCompleted / simulationTime : 0;
            float completionRate = totalOrdersGenerated > 0 ? (float)totalOrdersCompleted / totalOrdersGenerated : 0;

            float avgTurnaround = 0f;
            var completedOrders = orderMetrics.Values.Where(o => o.IsCompleted).ToList();
            if (completedOrders.Count > 0)
            {
                avgTurnaround = completedOrders.Average(o => o.CompletionTime - o.AssignmentTime);
            }

            snapshots.Add(new SnapshotData
            {
                Time = simulationTime,
                ActiveRobots = activeCount,
                IdleRobots = idleCount,
                PendingOrders = pendingOrders,
                CompletedOrders = totalOrdersCompleted,
                AverageUtilization = avgUtilization
            });

            EventManager.Instance?.TriggerSimulationMetricsSnapshot(
                simulationTime, throughput, completionRate,
                avgTurnaround, avgUtilization, totalDeadlocks,
                activeCount, idleCount, pendingOrders, totalOrdersCompleted
            );
        }

        private IEnumerator AutoSaveRoutine()
        {
            WaitForSeconds wait = new WaitForSeconds(autoSaveIntervalSeconds);
            while (true)
            {
                yield return wait;
                SaveMetricsReport();
            }
        }

        public void SaveMetricsReport()
        {
            string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string filepath = $"{Application.persistentDataPath}/simulation_metrics_{timestamp}.csv";

            try
            {
                StringBuilder csv = new StringBuilder();
                csv.AppendLine("Metric,Value");
                csv.AppendLine($"SimulationDuration,{Time.time - simulationStartTime:F2}");
                csv.AppendLine($"TotalOrdersGenerated,{totalOrdersGenerated}");
                csv.AppendLine($"TotalOrdersCompleted,{totalOrdersCompleted}");
                csv.AppendLine($"TotalOrdersFailed,{totalOrdersFailed}");
                csv.AppendLine($"OrderCompletionRate,{(totalOrdersGenerated > 0 ? (float)totalOrdersCompleted / totalOrdersGenerated : 0):F3}");
                csv.AppendLine($"Throughput,{(Time.time - simulationStartTime > 0 ? totalOrdersCompleted / (Time.time - simulationStartTime) : 0):F4}");
                csv.AppendLine($"TotalDeadlocks,{totalDeadlocks}");
                csv.AppendLine($"TotalWaitEvents,{waitEventCount}");

                float avgTurnaround = 0f;
                var completedOrders = orderMetrics.Values.Where(o => o.IsCompleted).ToList();
                if (completedOrders.Count > 0)
                {
                    avgTurnaround = completedOrders.Average(o => o.CompletionTime - o.AssignmentTime);
                }
                csv.AppendLine($"AverageOrderTurnaroundTime,{avgTurnaround:F2}");

                float avgUtilization = 0f;
                if (robotMetrics.Count > 0)
                {
                    avgUtilization = robotMetrics.Values
                        .Select(r => r.TotalActiveTime / Mathf.Max(r.TotalActiveTime + r.TotalIdleTime, 0.001f))
                        .Average();
                }
                csv.AppendLine($"AverageRobotUtilization,{avgUtilization:F3}");

                int totalCollisions = robotMetrics.Values.Sum(r => r.CollisionCount) / 2;
                csv.AppendLine($"TotalCollisions,{totalCollisions}");

                System.IO.File.WriteAllText(filepath, csv.ToString());
                Debug.Log($"[SimulationMetrics] Report saved to: {filepath}");
            }
            catch (Exception e)
            {
                Debug.LogError($"[SimulationMetrics] Error saving report: {e.Message}");
            }
        }

        public (float throughput, float completionRate, float avgTurnaround, float avgUtilization, int deadlocks) GetCurrentMetrics()
        {
            float duration = Time.time - simulationStartTime;
            float throughput = duration > 0 ? totalOrdersCompleted / duration : 0;
            float completionRate = totalOrdersGenerated > 0 ? (float)totalOrdersCompleted / totalOrdersGenerated : 0;

            float avgTurnaround = 0f;
            var completedOrders = orderMetrics.Values.Where(o => o.IsCompleted).ToList();
            if (completedOrders.Count > 0)
            {
                avgTurnaround = completedOrders.Average(o => o.CompletionTime - o.AssignmentTime);
            }

            float avgUtilization = 0f;
            if (robotMetrics.Count > 0)
            {
                avgUtilization = robotMetrics.Values
                    .Select(r => r.TotalActiveTime / Mathf.Max(r.TotalActiveTime + r.TotalIdleTime, 0.001f))
                    .Average();
            }

            return (throughput, completionRate, avgTurnaround, avgUtilization, totalDeadlocks);
        }
    }
}
