using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;
using CentralControl.RobotControl;

namespace CentralControl
{
    /// <summary>
    /// Automatically collects experiment metrics via EventManager events.
    /// Attach to any GameObject in the scene. Subscribes to collision, deadlock,
    /// and order-completed events. Saves summary + time-series CSV files on destroy.
    ///
    /// Tracked metrics:
    /// - Collision Rate: collisions per robot per minute
    /// - Deadlock Frequency: deadlocks per minute
    /// - Operational Duration: time until system breakdown (80%+ robots EmergencyStopped)
    /// - Task Throughput: orders completed per minute
    /// </summary>
    public class ExperimentMetricsCollector : MonoBehaviour
    {
        [Header("Experiment Configuration")]
        [Tooltip("Experiment config identifier (e.g. C1, C2, C3, C4)")]
        public string experimentId = "C1";

        [Tooltip("Scenario identifier (robot density setting)")]
        public int scenarioId = 1;

        [Tooltip("Trial number within this scenario")]
        public int trialId = 1;

        [Header("Output")]
        [Tooltip("Output directory. Empty = Application.persistentDataPath/ExperimentResults")]
        public string outputDirectory = "";

        [Header("Breakdown Detection")]
        [Tooltip("Fraction of robots that must be EmergencyStopped to declare system breakdown")]
        [Range(0.5f, 1.0f)]
        public float breakdownThreshold = 0.8f;

        // Tracked metrics
        private int totalCollisions = 0;
        private int totalDeadlocks = 0;
        private int tasksCompleted = 0;
        private int totalOrdersGenerated = 0;
        private float experimentStartTime;
        private float firstSystemBreakdownTime = -1f;
        private bool hasSystemBreakdown = false;
        private int totalRobots = 0;

        // Path planning aggregation (for Average Velocity, DRE, Replan Rate)
        private int totalPlans = 0;
        private int totalReplans = 0;
        private float sumVelocity = 0f;
        private int velocityCount = 0;
        private float sumDistanceRelativeError = 0f;
        private int dreCount = 0;

        // Per-interval tracking for time series
        private List<MetricSnapshot> snapshots = new List<MetricSnapshot>();
        private float lastSnapshotTime = 0f;
        public float snapshotInterval = 5f;

        private struct MetricSnapshot
        {
            public float time;
            public int collisions;
            public int deadlocks;
            public int tasksCompleted;
            public int emergencyStoppedRobots;
        }

        void Awake()
        {
            // Subscribe in Awake so we don't miss events fired between AddComponent and Start.
            SubscribeToEvents();
        }

        void Start()
        {
            experimentStartTime = Time.time;
            lastSnapshotTime = Time.time;

            // Auto-detect total robot count if not explicitly set
            if (totalRobots == 0)
            {
                var robots = GetAllRobots();
                if (robots != null && robots.Count > 0)
                {
                    totalRobots = robots.Count;
                }
            }
        }

        private void SubscribeToEvents()
        {
            if (EventManager.Instance != null)
            {
                EventManager.Instance.OnRobotCollisionDetected += OnCollision;
                EventManager.Instance.OnDeadlockDetected += OnDeadlock;
                EventManager.Instance.OnOrderCompleted += OnTaskCompleted;
                EventManager.Instance.OnOrderGenerated += OnOrderGeneratedHandler;
                EventManager.Instance.OnPathPlanningMetricsRecorded += OnPathMetrics;
            }
            else
            {
                Debug.LogWarning("[ExperimentMetrics] EventManager.Instance is null at Awake(). " +
                    "Metrics collection will not function correctly.");
            }
        }

        void Update()
        {
            if (Time.time - lastSnapshotTime >= snapshotInterval)
            {
                TakeSnapshot();
                lastSnapshotTime = Time.time;
            }

            CheckSystemBreakdown();
        }

        void OnDestroy()
        {
            if (EventManager.Instance != null)
            {
                EventManager.Instance.OnRobotCollisionDetected -= OnCollision;
                EventManager.Instance.OnDeadlockDetected -= OnDeadlock;
                EventManager.Instance.OnOrderCompleted -= OnTaskCompleted;
                EventManager.Instance.OnOrderGenerated -= OnOrderGeneratedHandler;
                EventManager.Instance.OnPathPlanningMetricsRecorded -= OnPathMetrics;
            }

            SaveResults();
        }

        #region Event Handlers

        private void OnCollision(object sender, CollisionEventArgs args)
        {
            totalCollisions++;
        }

        private void OnDeadlock(object sender, DeadlockDetectedEventArgs args)
        {
            totalDeadlocks++;
        }

        /// <summary>
        /// Handler for OnOrderCompleted which is Action&lt;string, string&gt; (orderId, robotId).
        /// </summary>
        private void OnTaskCompleted(string orderId, string robotId)
        {
            tasksCompleted++;
        }

        private void OnOrderGeneratedHandler(string orderId)
        {
            totalOrdersGenerated++;
        }

        private void OnPathMetrics(object sender, PathPlanningMetricsEventArgs args)
        {
            var m = args.Metrics;
            if (m == null) return;

            totalPlans++;
            if (m.IsReplan) totalReplans++;

            if (m.TravelTime > 0 && m.ActualPathLength > 0)
            {
                sumVelocity += m.ActualPathLength / m.TravelTime;
                velocityCount++;
            }

            if (m.IdealPathLength > 0 && m.ActualPathLength > 0)
            {
                sumDistanceRelativeError += m.DistanceRelativeError;
                dreCount++;
            }
        }

        #endregion

        #region Public API

        /// <summary>
        /// Sets total robot count for collision rate normalization.
        /// Call this after robots are initialized.
        /// </summary>
        public void SetTotalRobots(int count)
        {
            totalRobots = count;
        }

        /// <summary>
        /// Returns current experiment elapsed time in seconds.
        /// </summary>
        public float GetElapsedTime()
        {
            return Time.time - experimentStartTime;
        }

        /// <summary>
        /// Returns whether a system breakdown has been detected.
        /// </summary>
        public bool HasSystemBreakdown => hasSystemBreakdown;

        /// <summary>
        /// Returns current cumulative metrics as a formatted string for debugging.
        /// </summary>
        public string GetCurrentMetricsSummary()
        {
            float elapsed = GetElapsedTime();
            float minutes = elapsed / 60f;
            return $"[{experimentId} S{scenarioId} T{trialId}] " +
                $"t={elapsed:F0}s collisions={totalCollisions} deadlocks={totalDeadlocks} " +
                $"tasks={tasksCompleted} breakdown={hasSystemBreakdown}";
        }

        #endregion

        #region Breakdown Detection

        private void CheckSystemBreakdown()
        {
            if (hasSystemBreakdown) return;

            var robots = GetAllRobots();
            if (robots == null || robots.Count == 0) return;

            // Cache totalRobots from actual robot count if not set
            if (totalRobots == 0)
            {
                totalRobots = robots.Count;
            }

            int stuckCount = robots.Count(r => r.IsEmergencyStopped);

            // System breakdown = breakdownThreshold+ robots stuck
            if (stuckCount >= robots.Count * breakdownThreshold && robots.Count > 1)
            {
                hasSystemBreakdown = true;
                firstSystemBreakdownTime = Time.time - experimentStartTime;
                Debug.LogWarning($"[ExperimentMetrics] System breakdown detected at " +
                    $"t={firstSystemBreakdownTime:F1}s ({stuckCount}/{robots.Count} stuck)");
            }
        }

        #endregion

        #region Snapshot

        private void TakeSnapshot()
        {
            int emergencyStopped = 0;
            var robots = GetAllRobots();
            if (robots != null)
            {
                emergencyStopped = robots.Count(r => r.IsEmergencyStopped);
            }

            snapshots.Add(new MetricSnapshot
            {
                time = Time.time - experimentStartTime,
                collisions = totalCollisions,
                deadlocks = totalDeadlocks,
                tasksCompleted = tasksCompleted,
                emergencyStoppedRobots = emergencyStopped
            });
        }

        #endregion

        #region CSV Output

        private void SaveResults()
        {
            // Take a final snapshot before saving
            TakeSnapshot();

            float duration = Time.time - experimentStartTime;
            float durationMinutes = duration / 60f;

            // Ensure output directory exists
            string outputDir = string.IsNullOrEmpty(outputDirectory)
                ? Path.Combine(Application.persistentDataPath, "ExperimentResults")
                : outputDirectory;
            if (!Directory.Exists(outputDir))
            {
                Directory.CreateDirectory(outputDir);
            }

            SaveSummaryCSV(outputDir, duration, durationMinutes);
            SaveTimeSeriesCSV(outputDir);
        }

        private void SaveSummaryCSV(string outputDir, float duration, float durationMinutes)
        {
            string summaryPath = Path.Combine(outputDir,
                $"experiment_{experimentId}_S{scenarioId}_T{trialId}_summary.csv");

            try
            {
                using (var writer = new StreamWriter(summaryPath))
                {
                    writer.WriteLine("Metric,Value");
                    writer.WriteLine($"ExperimentConfig,{experimentId}");
                    writer.WriteLine($"Scenario,{scenarioId}");
                    writer.WriteLine($"Trial,{trialId}");
                    writer.WriteLine($"TotalRobots,{totalRobots}");
                    writer.WriteLine($"DurationSeconds,{duration:F2}");
                    writer.WriteLine($"TotalCollisions,{totalCollisions}");
                    writer.WriteLine($"TotalDeadlocks,{totalDeadlocks}");
                    writer.WriteLine($"TasksCompleted,{tasksCompleted}");

                    float collisionRate = (totalRobots > 0 && durationMinutes > 0)
                        ? totalCollisions / (float)(totalRobots * durationMinutes)
                        : 0f;
                    writer.WriteLine($"CollisionRate,{collisionRate:F4}");

                    float deadlockFrequency = durationMinutes > 0
                        ? totalDeadlocks / durationMinutes
                        : 0f;
                    writer.WriteLine($"DeadlockFrequency,{deadlockFrequency:F4}");

                    float operationalDuration = hasSystemBreakdown
                        ? firstSystemBreakdownTime
                        : duration;
                    writer.WriteLine($"OperationalDuration,{operationalDuration:F2}");

                    float taskThroughput = durationMinutes > 0
                        ? tasksCompleted / durationMinutes
                        : 0f;
                    writer.WriteLine($"TaskThroughput,{taskThroughput:F4}");

                    writer.WriteLine($"SystemBreakdown,{hasSystemBreakdown}");

                    // === Unified 7 metrics ===
                    float taskCompletionRate = totalOrdersGenerated > 0
                        ? tasksCompleted / (float)totalOrdersGenerated
                        : 0f;
                    writer.WriteLine($"TaskCompletionRate,{taskCompletionRate:F4}");

                    float avgVelocity = velocityCount > 0 ? sumVelocity / velocityCount : 0f;
                    writer.WriteLine($"AverageVelocity,{avgVelocity:F4}");

                    float avgDRE = dreCount > 0 ? sumDistanceRelativeError / dreCount : 0f;
                    writer.WriteLine($"DistanceRelativeError,{avgDRE:F4}");

                    float replanRate = totalPlans > 0 ? totalReplans / (float)totalPlans : 0f;
                    writer.WriteLine($"ReplanRate,{replanRate:F4}");
                }

                Debug.Log($"[ExperimentMetrics] Summary saved: {summaryPath}");
            }
            catch (Exception e)
            {
                Debug.LogError($"[ExperimentMetrics] Failed to save summary CSV: {e.Message}");
            }
        }

        private void SaveTimeSeriesCSV(string outputDir)
        {
            string timeseriesPath = Path.Combine(outputDir,
                $"experiment_{experimentId}_S{scenarioId}_T{trialId}_timeseries.csv");

            try
            {
                using (var writer = new StreamWriter(timeseriesPath))
                {
                    writer.WriteLine("Time,CumulativeCollisions,CumulativeDeadlocks,TasksCompleted,EmergencyStoppedRobots");
                    foreach (var s in snapshots)
                    {
                        writer.WriteLine($"{s.time:F1},{s.collisions},{s.deadlocks},{s.tasksCompleted},{s.emergencyStoppedRobots}");
                    }
                }

                Debug.Log($"[ExperimentMetrics] Time series saved: {timeseriesPath}");
            }
            catch (Exception e)
            {
                Debug.LogError($"[ExperimentMetrics] Failed to save time series CSV: {e.Message}");
            }
        }

        #endregion

        #region Helper

        /// <summary>
        /// Gets all robots via CentralController.Instance.robotManager.
        /// Returns null if not available.
        /// </summary>
        private List<RobotController> GetAllRobots()
        {
            if (CentralController.Instance == null) return null;
            if (CentralController.Instance.robotManager == null) return null;
            return CentralController.Instance.robotManager.GetAllRobots();
        }

        #endregion
    }
}
