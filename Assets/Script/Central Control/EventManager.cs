using System;
using System.IO;
using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using Map;
using CentralControl.RobotControl;
using CentralControl.PickerControl;
using PathPlanning;

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
    
    public class PathPlanningMetricsEventArgs : EventArgs
    {
        public PathPlanningMetrics Metrics { get; private set; }
        
        public PathPlanningMetricsEventArgs(PathPlanningMetrics metrics)
        {
            Metrics = metrics;
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

    public enum CollisionRecoveryResult
    {
        Pending,
        Success,
        Failed,
        GlobalMapUpdateRequested
    }

    public class CollisionEventArgs : EventArgs
    {
        public string CollisionId { get; private set; }
        public string RobotId1 { get; private set; }
        public string RobotId2 { get; private set; }
        public float Time { get; private set; }
        public bool IsRobotCollision { get; private set; }
        public GameObject CollisionObject { get; private set; }
        public Vector3 Position { get; private set; }
        public CollisionRecoveryResult RecoveryResult { get; set; }
        public float RecoveryTime { get; set; }

        public CollisionEventArgs(
            string collisionId, 
            string robotId1, 
            string robotId2, 
            float time, 
            bool isRobotCollision,
            Vector3 position,
            GameObject collisionObject = null)
        {
            CollisionId = collisionId;
            RobotId1 = robotId1;
            RobotId2 = robotId2;
            Time = time;
            IsRobotCollision = isRobotCollision;
            Position = position;
            CollisionObject = collisionObject;
            RecoveryResult = CollisionRecoveryResult.Pending;
            RecoveryTime = 0f;
        }
    }

    public class GlobalMapUpdateRequestEventArgs : EventArgs
    {
        public int[] RobotIds { get; private set; }
        public float RequestTime { get; private set; }

        public GlobalMapUpdateRequestEventArgs(int[] robotIds, float requestTime)
        {
            RobotIds = robotIds;
            RequestTime = requestTime;
        }
    }

    public class RobotStateChangedEventArgs : EventArgs
    {
        public string RobotId { get; private set; }
        public string StateName { get; private set; }
        public bool NewValue { get; private set; }
        public float Time { get; private set; }

        public RobotStateChangedEventArgs(string robotId, string stateName, bool newValue, float time)
        {
            RobotId = robotId;
            StateName = stateName;
            NewValue = newValue;
            Time = time;
        }
    }

    public class CollisionRecoveryCompletedEventArgs : EventArgs
    {
        public string RobotId1 { get; private set; }
        public string RobotId2 { get; private set; }
        public CollisionRecoveryResult Result { get; private set; }
        public float RecoveryTime { get; private set; }
        public float CollisionTime { get; private set; }

        public CollisionRecoveryCompletedEventArgs(string robotId1, string robotId2, CollisionRecoveryResult result, float recoveryTime, float collisionTime)
        {
            RobotId1 = robotId1;
            RobotId2 = robotId2;
            Result = result;
            RecoveryTime = recoveryTime;
            CollisionTime = collisionTime;
        }
    }

    public class DeadlockDetectedEventArgs : EventArgs
    {
        public string[] InvolvedRobotIds { get; private set; }
        public float DetectionTime { get; private set; }

        public DeadlockDetectedEventArgs(string[] involvedRobotIds, float detectionTime)
        {
            InvolvedRobotIds = involvedRobotIds;
            DetectionTime = detectionTime;
        }
    }

    public class RobotWaitEventArgs : EventArgs
    {
        public string RobotId { get; private set; }
        public string WaitReason { get; private set; }
        public float Time { get; private set; }

        public RobotWaitEventArgs(string robotId, string waitReason, float time)
        {
            RobotId = robotId;
            WaitReason = waitReason;
            Time = time;
        }
    }

    public class OrderAssignedEventArgs : EventArgs
    {
        public string OrderId { get; private set; }
        public string RobotId { get; private set; }
        public float AssignmentTime { get; private set; }

        public OrderAssignedEventArgs(string orderId, string robotId, float assignmentTime)
        {
            OrderId = orderId;
            RobotId = robotId;
            AssignmentTime = assignmentTime;
        }
    }

    public class AreaCongestedEventArgs : EventArgs
    {
        public string AreaId { get; private set; }
        public string[] CongestedRobotIds { get; private set; }
        public CongestionLevel Level { get; private set; }
        public float DetectionTime { get; private set; }

        public AreaCongestedEventArgs(string areaId, string[] congestedRobotIds, CongestionLevel level, float detectionTime)
        {
            AreaId = areaId;
            CongestedRobotIds = congestedRobotIds;
            Level = level;
            DetectionTime = detectionTime;
        }
    }

    public class CongestionResolvedEventArgs : EventArgs
    {
        public string AreaId { get; private set; }
        public float ResolvedTime { get; private set; }

        public CongestionResolvedEventArgs(string areaId, float resolvedTime)
        {
            AreaId = areaId;
            ResolvedTime = resolvedTime;
        }
    }

    public class ReplanRequestEventArgs : EventArgs
    {
        public string RobotId { get; private set; }
        public string Reason { get; private set; }
        public float RequestTime { get; private set; }
        public float SuggestedDelay { get; private set; }

        public ReplanRequestEventArgs(string robotId, string reason, float requestTime, float suggestedDelay = 0f)
        {
            RobotId = robotId;
            Reason = reason;
            RequestTime = requestTime;
            SuggestedDelay = suggestedDelay;
        }
    }

    public class SimulationMetricsSnapshotEventArgs : EventArgs
    {
        public float SimulationTime { get; private set; }
        public float Throughput { get; private set; }
        public float CompletionRate { get; private set; }
        public float AverageTurnaroundTime { get; private set; }
        public float AverageUtilization { get; private set; }
        public int TotalDeadlocks { get; private set; }
        public int ActiveRobots { get; private set; }
        public int IdleRobots { get; private set; }
        public int PendingOrders { get; private set; }
        public int CompletedOrders { get; private set; }

        public SimulationMetricsSnapshotEventArgs(
            float simulationTime, float throughput, float completionRate,
            float avgTurnaround, float avgUtilization, int deadlocks,
            int activeRobots, int idleRobots, int pendingOrders, int completedOrders)
        {
            SimulationTime = simulationTime;
            Throughput = throughput;
            CompletionRate = completionRate;
            AverageTurnaroundTime = avgTurnaround;
            AverageUtilization = avgUtilization;
            TotalDeadlocks = deadlocks;
            ActiveRobots = activeRobots;
            IdleRobots = idleRobots;
            PendingOrders = pendingOrders;
            CompletedOrders = completedOrders;
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
        public event EventHandler<PathPlanningMetricsEventArgs> OnPathPlanningMetricsRecorded;

        private Dictionary<string, PathPlanningMetrics> pathPlanningMetrics = new Dictionary<string, PathPlanningMetrics>();

        // Collision-related events
        public event EventHandler<CollisionEventArgs> OnRobotCollisionDetected;
        public event EventHandler<GlobalMapUpdateRequestEventArgs> OnRequestGlobalMapUpdate;

        // Order-related events
        public event Action<string> OnOrderGenerated;
        public event Action<string, string> OnOrderCompleted;
        public event Action<string, string, string> OnOrderFailed;
        public event EventHandler<OrderAssignedEventArgs> OnOrderAssigned;

        // Robot state events
        public event EventHandler<RobotStateChangedEventArgs> OnRobotStateChanged;
        public event EventHandler<CollisionRecoveryCompletedEventArgs> OnCollisionRecoveryCompleted;
        public event EventHandler<DeadlockDetectedEventArgs> OnDeadlockDetected;
        public event EventHandler<RobotWaitEventArgs> OnRobotWaitStarted;
        public event EventHandler<RobotWaitEventArgs> OnRobotWaitEnded;

        // Area congestion events (from central DeadlockDetector)
        public event EventHandler<AreaCongestedEventArgs> OnAreaCongested;
        public event EventHandler<CongestionResolvedEventArgs> OnCongestionResolved;

        // Replan request events (central to robot communication)
        public event EventHandler<ReplanRequestEventArgs> OnReplanRequested;

        // Simulation metrics snapshot event
        public event EventHandler<SimulationMetricsSnapshotEventArgs> OnSimulationMetricsSnapshot;

        // Collision records
        private Dictionary<string, CollisionEventArgs> robotCollisions = new Dictionary<string, CollisionEventArgs>();
        private Dictionary<string, CollisionEventArgs> environmentCollisions = new Dictionary<string, CollisionEventArgs>();

        // Throttling
        private Dictionary<string, float> lastStateChangeTime = new Dictionary<string, float>();
        private const float STATE_CHANGE_THROTTLE_INTERVAL = 0.1f;

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
            SafeEventInvoker.SafeInvoke(OnMapLoaded);
        }

        public void TriggerRobotsInitialized()
        {
            SafeEventInvoker.SafeInvoke(OnRobotsInitialized);
        }

        public void TriggerPickersInitialized()
        {
            SafeEventInvoker.SafeInvoke(OnPickersInitialized);
        }

        // Global occupancy layer triggers
        public void TriggerGlobalOccupancyLayerUpdated(DynamicOccupancyLayer globalLayer, float updateTime)
        {
            SafeEventInvoker.SafeInvoke(OnGlobalOccupancyLayerUpdated, this, new GlobalOccupancyLayerEventArgs(globalLayer, updateTime));
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

            SafeEventInvoker.SafeInvoke(OnRobotReadyForPicking, this, new PickingEventArgs(robotId, pickingPoint, location));
        }

        public void TriggerPickingCompleted(string robotId, PickingPoint pickingPoint, Vector3 location)
        {
            Debug.Log($"EventManager: Picking completed for robot {robotId} at {location}");
            SafeEventInvoker.SafeInvoke(OnPickingCompleted, this, new PickingEventArgs(robotId, pickingPoint, location));
        }

        // Path triggers
        public void TriggerRobotPathPlanned(string robotId, string planId, List<ConnectionPoint> path, List<float> times, List<float> speeds)
        {
            Debug.Log($"EventManager: Robot {robotId} planned a path with {path.Count} points");
            SafeEventInvoker.SafeInvoke(OnRobotPathPlanned, this, new RobotPathEventArgs(robotId, planId, path, times, speeds));
        }

        public void TriggerRobotPathModified(RobotPathModificationEventArgs args)
        {
            Debug.Log($"EventManager: Triggered robot path modification event: Robot {args.RobotId}, Type {args.ModificationType}, Value {args.ModificationValue}");
            SafeEventInvoker.SafeInvoke(OnRobotPathModified, this, args);
        }

        public void TriggerRobotPathAccepted(string robotId, string planId, List<ConnectionPoint> path, List<float> times, List<float> speeds)
        {
            Debug.Log($"EventManager: Robot {robotId}'s path {planId} has been accepted, with {path.Count} points");
            SafeEventInvoker.SafeInvoke(OnRobotPathAccepted, this, new RobotPathAcceptedEventArgs(robotId, planId, path, times, speeds));
        }

        // Collision triggers
        public void ReportRobotCollision(string robotId1, string robotId2, GameObject collisionObject, float time, Vector3 position = default)
        {
            string[] robotIds = new string[] { robotId1, robotId2 };
            Array.Sort(robotIds);
            string collisionId = $"{robotIds[0]}_collided_with_{robotIds[1]}_{time:F5}";

            if (robotCollisions.ContainsKey(collisionId))
            {
                return;
            }

            Debug.LogWarning($"EventManager: Collision Robot {robotId1} <-> Robot {robotId2} at {position}");

            CollisionEventArgs args = new CollisionEventArgs(
                collisionId,
                robotId1,
                robotId2,
                time,
                true,
                position,
                collisionObject
            );

            robotCollisions[collisionId] = args;

            SafeEventInvoker.SafeInvoke(OnRobotCollisionDetected, this, args);
        }

        public void ReportEnvironmentCollision(string robotId, string environmentObjectId, GameObject collisionObject, float time, Vector3 position = default)
        {
            string collisionId = $"{robotId}_collided_with_env_{environmentObjectId}_{time:F5}";

            if (environmentCollisions.ContainsKey(collisionId))
            {
                return;
            }

            Debug.LogWarning($"EventManager: Collision Robot {robotId} <-> Env {environmentObjectId} at {position}");

            CollisionEventArgs args = new CollisionEventArgs(
                collisionId,
                robotId,
                environmentObjectId,
                time,
                false,
                position,
                collisionObject
            );

            environmentCollisions[collisionId] = args;

            SafeEventInvoker.SafeInvoke(OnRobotCollisionDetected, this, args);
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

        public string UpdateCollisionRecoveryResult(string robotId1, string robotId2, float collisionTime, CollisionRecoveryResult result, float recoveryTime)
        {
            string[] robotIds = new string[] { robotId1, robotId2 };
            Array.Sort(robotIds);
            
            foreach (var kvp in robotCollisions)
            {
                if (kvp.Key.StartsWith($"{robotIds[0]}_collided_with_{robotIds[1]}") &&
                    Mathf.Abs(kvp.Value.Time - collisionTime) < 0.1f)
                {
                    kvp.Value.RecoveryResult = result;
                    kvp.Value.RecoveryTime = recoveryTime;
                    return kvp.Key;
                }
            }
            
            return null;
        }

        public void TriggerRequestGlobalMapUpdate(params int[] robotIds)
        {
            Debug.Log($"EventManager: Global map update requested by robots: {string.Join(", ", robotIds)}");
            SafeEventInvoker.SafeInvoke(OnRequestGlobalMapUpdate, this, new GlobalMapUpdateRequestEventArgs(robotIds, Time.time));
        }

        public void TriggerOrderGenerated(string orderId)
        {
            try
            {
                OnOrderGenerated?.Invoke(orderId);
            }
            catch (System.Exception e)
            {
                Debug.LogError($"[EventManager] Error invoking OnOrderGenerated: {e.Message}");
            }
        }

        public void TriggerOrderCompleted(string orderId, string robotId)
        {
            Debug.Log($"EventManager: Order {orderId} completed by Robot {robotId}");
            SafeEventInvoker.SafeInvoke(OnOrderCompleted, orderId, robotId);
        }

        public void TriggerOrderFailed(string orderId, string robotId, string reason)
        {
            Debug.LogWarning($"EventManager: Order {orderId} failed by Robot {robotId}: {reason}");
            SafeEventInvoker.SafeInvoke(OnOrderFailed, orderId, robotId, reason);
        }

        public void TriggerOrderAssigned(string orderId, string robotId)
        {
            SafeEventInvoker.SafeInvoke(OnOrderAssigned, this, new OrderAssignedEventArgs(orderId, robotId, Time.time));
        }

        public void TriggerRobotStateChanged(string robotId, string stateName, bool newValue)
        {
            string key = $"{robotId}_{stateName}";
            float currentTime = Time.time;
            
            if (lastStateChangeTime.TryGetValue(key, out float lastTime))
            {
                if (currentTime - lastTime < STATE_CHANGE_THROTTLE_INTERVAL)
                {
                    return;
                }
            }
            
            lastStateChangeTime[key] = currentTime;
            SafeEventInvoker.SafeInvoke(OnRobotStateChanged, this, new RobotStateChangedEventArgs(robotId, stateName, newValue, currentTime));
        }

        public void TriggerCollisionRecoveryCompleted(string robotId1, string robotId2, CollisionRecoveryResult result, float recoveryTime, float collisionTime)
        {
            UpdateCollisionRecoveryResult(robotId1, robotId2, collisionTime, result, recoveryTime);
            SafeEventInvoker.SafeInvoke(OnCollisionRecoveryCompleted, this, new CollisionRecoveryCompletedEventArgs(robotId1, robotId2, result, recoveryTime, collisionTime));
        }

        public void TriggerDeadlockDetected(string[] involvedRobotIds)
        {
            SafeEventInvoker.SafeInvoke(OnDeadlockDetected, this, new DeadlockDetectedEventArgs(involvedRobotIds, Time.time));
        }

        public void TriggerRobotWaitStarted(string robotId, string reason)
        {
            SafeEventInvoker.SafeInvoke(OnRobotWaitStarted, this, new RobotWaitEventArgs(robotId, reason, Time.time));
        }

        public void TriggerRobotWaitEnded(string robotId, string reason)
        {
            SafeEventInvoker.SafeInvoke(OnRobotWaitEnded, this, new RobotWaitEventArgs(robotId, reason, Time.time));
        }

        public void TriggerAreaCongested(string areaId, string[] congestedRobotIds, CongestionLevel level)
        {
            SafeEventInvoker.SafeInvoke(OnAreaCongested, this, new AreaCongestedEventArgs(areaId, congestedRobotIds, level, Time.time));
        }

        public void TriggerCongestionResolved(string areaId)
        {
            SafeEventInvoker.SafeInvoke(OnCongestionResolved, this, new CongestionResolvedEventArgs(areaId, Time.time));
        }

        public void TriggerReplanRequested(string robotId, string reason, float suggestedDelay = 0f)
        {
            SafeEventInvoker.SafeInvoke(OnReplanRequested, this, new ReplanRequestEventArgs(robotId, reason, Time.time, suggestedDelay));
        }

        public void TriggerSimulationMetricsSnapshot(
            float simulationTime, float throughput, float completionRate,
            float avgTurnaround, float avgUtilization, int deadlocks,
            int activeRobots, int idleRobots, int pendingOrders, int completedOrders)
        {
            SafeEventInvoker.SafeInvoke(OnSimulationMetricsSnapshot, this, new SimulationMetricsSnapshotEventArgs(
                simulationTime, throughput, completionRate,
                avgTurnaround, avgUtilization, deadlocks,
                activeRobots, idleRobots, pendingOrders, completedOrders
            ));
        }

        // Path planning metrics
        public void ReportPathPlanningMetrics(string robotId, string planId, float planningTime, float plannedPathLength, float estimatedTravelTime, float idealPathLength, bool isReplan = false)
        {
            string metricsId = $"{robotId}_plan_{planId}";

            if (pathPlanningMetrics.ContainsKey(metricsId))
            {
                return;
            }

            Debug.Log($"EventManager: Reported path planning metrics for Robot {robotId}, Plan {planId}, Path Length: {plannedPathLength:F2}, IsReplan: {isReplan}");

            PathPlanningMetrics metrics = new PathPlanningMetrics(
                robotId,
                planId,
                isReplan
            )
            {
                PlanningTime = planningTime,
                ActualPathLength = plannedPathLength,
                TravelTime = estimatedTravelTime,
                IdealPathLength = idealPathLength
            };

            pathPlanningMetrics[metricsId] = metrics;

            SafeEventInvoker.SafeInvoke(OnPathPlanningMetricsRecorded, this, new PathPlanningMetricsEventArgs(metrics));
        }

        /// <summary>
        /// Reports a pre-built PathPlanningMetrics object (preserving IsReplan and all fields).
        /// Preferred over the field-by-field overload when the caller already has a complete metrics object.
        /// </summary>
        public void ReportPathPlanningMetrics(PathPlanningMetrics metrics)
        {
            if (metrics == null) return;

            string metricsId = $"{metrics.RobotId}_plan_{metrics.PlanId}";
            if (pathPlanningMetrics.ContainsKey(metricsId)) return;

            pathPlanningMetrics[metricsId] = metrics;
            SafeEventInvoker.SafeInvoke(OnPathPlanningMetricsRecorded, this, new PathPlanningMetricsEventArgs(metrics));
        }

        public List<PathPlanningMetrics> GetAllPathPlanningMetrics()
        {
            return pathPlanningMetrics.Values.ToList();
        }

        public List<PathPlanningMetrics> GetRobotPathPlanningMetrics(string robotId)
        {
            List<PathPlanningMetrics> result = new List<PathPlanningMetrics>();
            
            foreach (var metrics in pathPlanningMetrics.Values)
            {
                if (metrics.RobotId == robotId)
                {
                    result.Add(metrics);
                }
            }
            
            return result;
        }

        public (int totalPlans, Dictionary<string, int> robotPlanCounts, float avgPlanningTime, float avgPathLength, float avgIdealLength) GetPathPlanningStatistics()
        {
            Dictionary<string, int> robotCounts = new Dictionary<string, int>();
            float totalPlanningTime = 0;
            float totalPathLength = 0;
            float totalIdealLength = 0;
            
            foreach (var metrics in pathPlanningMetrics.Values)
            {
                if (!robotCounts.ContainsKey(metrics.RobotId))
                {
                    robotCounts[metrics.RobotId] = 0;
                }
                robotCounts[metrics.RobotId]++;
                
                totalPlanningTime += metrics.PlanningTime;
                totalPathLength += metrics.ActualPathLength;
                totalIdealLength += metrics.IdealPathLength;
            }
            
            int count = pathPlanningMetrics.Count;
            float avgPlanningTime = count > 0 ? totalPlanningTime / count : 0;
            float avgPathLength = count > 0 ? totalPathLength / count : 0;
            float avgIdealLength = count > 0 ? totalIdealLength / count : 0;
            
            return (count, robotCounts, avgPlanningTime, avgPathLength, avgIdealLength);
        }

        public void ClearPathPlanningRecords()
        {
            pathPlanningMetrics.Clear();
            
            Debug.Log("EventManager: All path planning metrics have been cleared");
        }
    }
}