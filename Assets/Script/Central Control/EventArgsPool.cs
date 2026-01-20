using System;
using System.Collections.Generic;
using UnityEngine;

namespace CentralControl
{
    /// <summary>
    /// Generic object pool for high-frequency EventArgs to reduce GC pressure.
    /// Use for events triggered multiple times per frame (state changes, position updates).
    /// </summary>
    public static class EventArgsPool
    {
        private static readonly Dictionary<Type, Stack<object>> pools = new Dictionary<Type, Stack<object>>();
        private static readonly Dictionary<Type, int> poolCapacities = new Dictionary<Type, int>();
        private const int DEFAULT_CAPACITY = 32;
        private const int MAX_CAPACITY = 128;

        public static T Get<T>() where T : EventArgs, new()
        {
            Type type = typeof(T);
            
            lock (pools)
            {
                if (!pools.TryGetValue(type, out var pool))
                {
                    pool = new Stack<object>(DEFAULT_CAPACITY);
                    pools[type] = pool;
                    poolCapacities[type] = DEFAULT_CAPACITY;
                }

                if (pool.Count > 0)
                {
                    return (T)pool.Pop();
                }
            }

            return new T();
        }

        public static void Return<T>(T args) where T : EventArgs
        {
            if (args == null) return;

            Type type = typeof(T);

            lock (pools)
            {
                if (!pools.TryGetValue(type, out var pool))
                {
                    pool = new Stack<object>(DEFAULT_CAPACITY);
                    pools[type] = pool;
                    poolCapacities[type] = DEFAULT_CAPACITY;
                }

                if (pool.Count < MAX_CAPACITY)
                {
                    pool.Push(args);
                }
            }
        }

        public static void Clear()
        {
            lock (pools)
            {
                pools.Clear();
                poolCapacities.Clear();
            }
        }

        public static int GetPoolSize<T>() where T : EventArgs
        {
            Type type = typeof(T);
            lock (pools)
            {
                if (pools.TryGetValue(type, out var pool))
                {
                    return pool.Count;
                }
            }
            return 0;
        }
    }

    /// <summary>
    /// Poolable version of RobotStateChangedEventArgs for high-frequency state updates.
    /// </summary>
    public class PoolableRobotStateChangedEventArgs : EventArgs
    {
        public string RobotId { get; private set; }
        public string StateName { get; private set; }
        public bool NewValue { get; private set; }
        public float Time { get; private set; }

        public PoolableRobotStateChangedEventArgs() { }

        public PoolableRobotStateChangedEventArgs Initialize(string robotId, string stateName, bool newValue, float time)
        {
            RobotId = robotId;
            StateName = stateName;
            NewValue = newValue;
            Time = time;
            return this;
        }

        public void Reset()
        {
            RobotId = null;
            StateName = null;
            NewValue = false;
            Time = 0f;
        }
    }

    /// <summary>
    /// Poolable version of RobotWaitEventArgs for wait start/end events.
    /// </summary>
    public class PoolableRobotWaitEventArgs : EventArgs
    {
        public string RobotId { get; private set; }
        public string WaitReason { get; private set; }
        public float Time { get; private set; }

        public PoolableRobotWaitEventArgs() { }

        public PoolableRobotWaitEventArgs Initialize(string robotId, string waitReason, float time)
        {
            RobotId = robotId;
            WaitReason = waitReason;
            Time = time;
            return this;
        }

        public void Reset()
        {
            RobotId = null;
            WaitReason = null;
            Time = 0f;
        }
    }

    /// <summary>
    /// Poolable version of AreaCongestedEventArgs for congestion updates.
    /// </summary>
    public class PoolableAreaCongestedEventArgs : EventArgs
    {
        public string AreaId { get; private set; }
        public string[] CongestedRobotIds { get; private set; }
        public CongestionLevel Level { get; private set; }
        public float DetectionTime { get; private set; }

        public PoolableAreaCongestedEventArgs() { }

        public PoolableAreaCongestedEventArgs Initialize(string areaId, string[] congestedRobotIds, CongestionLevel level, float detectionTime)
        {
            AreaId = areaId;
            CongestedRobotIds = congestedRobotIds;
            Level = level;
            DetectionTime = detectionTime;
            return this;
        }

        public void Reset()
        {
            AreaId = null;
            CongestedRobotIds = null;
            Level = CongestionLevel.None;
            DetectionTime = 0f;
        }
    }
}
