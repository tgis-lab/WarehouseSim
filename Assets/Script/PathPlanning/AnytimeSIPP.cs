using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using UnityEngine;
using CentralControl;
using Map;
using NetTopologySuite.Geometries;
using Debug = UnityEngine.Debug;

namespace PathPlanning
{
    /// <summary>
    /// Anytime Safe Interval Path Planning 算法
    /// 整合了时间连续性、安全间隔、任何时间特性和等待标记
    /// </summary>
    public class AnytimeSIPP
    {
        // 状态类，表示SIPP算法中的节点
        private class SIPPState : IComparable<SIPPState>
        {
            public RoutePoint RoutePoint { get; private set; }
            public int IntervalIndex { get; private set; }
            public float ArrivalTime { get; private set; }
            public float DepartureTime { get; private set; }
            public float GScore { get; private set; }
            public float FScore { get; private set; }
            public SIPPState Parent { get; private set; }
            public SafeInterval SafeInterval { get; private set; }
            public float Speed { get; private set; }
            public bool IsOptimal { get; private set; }
            public bool HasWait { get; private set; }
            public float WaitTime { get; private set; }

            public SIPPState(
                RoutePoint routePoint, 
                int intervalIndex, 
                float arrivalTime,
                float departureTime,
                float gScore, 
                float fScore, 
                SIPPState parent, 
                SafeInterval safeInterval, 
                float speed = 1.0f,
                bool isOptimal = false,
                bool hasWait = false,
                float waitTime = 0.0f)
            {
                RoutePoint = routePoint;
                IntervalIndex = intervalIndex;
                ArrivalTime = arrivalTime;
                DepartureTime = departureTime;
                GScore = gScore;
                FScore = fScore;
                Parent = parent;
                SafeInterval = safeInterval;
                Speed = speed;
                IsOptimal = isOptimal;
                HasWait = hasWait;
                WaitTime = waitTime;
            }

            public int CompareTo(SIPPState other)
            {
                // 优先队列中，较低的F值优先级更高
                return FScore.CompareTo(other.FScore);
            }

            public override bool Equals(object obj)
            {
                if (obj is SIPPState other)
                {
                    return RoutePoint.ConnectionPoint.Id == other.RoutePoint.ConnectionPoint.Id &&
                           IntervalIndex == other.IntervalIndex &&
                           IsOptimal == other.IsOptimal;
                }
                return false;
            }

            public override int GetHashCode()
            {
                return HashCode.Combine(RoutePoint.ConnectionPoint.Id, IntervalIndex, IsOptimal);
            }

            public override string ToString()
            {
                return $"State[Point={RoutePoint.ConnectionPoint.Id}, Interval={IntervalIndex}, " +
                       $"Optimal={IsOptimal}, Arrival={ArrivalTime:F2}, Departure={DepartureTime:F2}, " +
                       $"Wait={WaitTime:F2}, G={GScore:F2}, F={FScore:F2}]";
            }

            // 创建状态副本但有新的F值
            public SIPPState WithNewFScore(float newFScore)
            {
                return new SIPPState(
                    RoutePoint, 
                    IntervalIndex, 
                    ArrivalTime, 
                    DepartureTime,
                    GScore, 
                    newFScore, 
                    Parent, 
                    SafeInterval, 
                    Speed, 
                    IsOptimal,
                    HasWait,
                    WaitTime);
            }
        }

        // 安全时间间隔类
        private class SafeInterval
        {
            public float StartTime { get; private set; }
            public float EndTime { get; private set; }

            public SafeInterval(float startTime, float endTime)
            {
                if (endTime < startTime)
                {
                    throw new ArgumentException("End time must be greater than start time");
                }

                StartTime = startTime;
                EndTime = endTime;
            }

            public bool Contains(float time)
            {
                return time >= StartTime && time <= EndTime;
            }

            public override string ToString()
            {
                return $"[{StartTime:F2} - {EndTime:F2}]";
            }
        }

        // 冲突检测器
        private static CellSpaceConflictDetector conflictDetector = new CellSpaceConflictDetector();

        /// <summary>
        /// 动作类型枚举
        /// </summary>
        private enum MotionType
        {
            Wait,   // 等待
            Move    // 移动
        }

        /// <summary>
        /// 动作类（包含动作类型、目标位置和距离信息）
        /// </summary>
        private class Motion
        {
            public RoutePoint TargetPoint { get; private set; }
            public float Distance { get; private set; }
            public MotionType Type { get; private set; }
            
            public Motion(RoutePoint targetPoint, float distance, MotionType type)
            {
                TargetPoint = targetPoint;
                Distance = distance;
                Type = type;
            }
        }

        /// <summary>
        /// Anytime SIPP算法入口点
        /// </summary>
        public static (List<ConnectionPoint> path, List<float> times, List<float> speeds, string planId) SIPPAlgorithm(
            Graph graph,
            IndoorSpace indoorSpace,
            DynamicOccupancyLayer personalOccupancyLayer,
            DynamicOccupancyLayer globalOccupancyLayer,
            RoutePoint startPosition,
            RoutePoint endPosition,
            string robotId,
            string currentPlanId,
            float startTime,
            float defaultSpeed,
            float maxAcceleration,
            float maxDeceleration,
            SearchMethod searchMethod)
        {
            if (graph == null || indoorSpace == null || startPosition == null || endPosition == null)
            {
                Debug.LogError($"Robot {robotId}: Invalid parameters for SIPP planning");
                return (new List<ConnectionPoint>(), new List<float>(), new List<float>(), null);
            }

            // 创建交通规则知识库
            TrafficRuleKnowledge trafficRules = new TrafficRuleKnowledge(indoorSpace, graph);

            // 创建一个新的计划ID
            string newPlanId = $"SIPPPlan_{robotId}_{DateTime.Now.Ticks}";

            // 设置时间限制和初始epsilon
            float timeLimit = 0.5f;  // 500毫秒的规划时间限制
            float initialEpsilon = 3.0f;  // 初始的启发式权重
            float finalEpsilon = 1.0f;    // 最终的启发式权重（1.0表示最优）
            float epsilonStep = 0.5f;     // 每次迭代的epsilon减少量

            // 构建安全时间间隔数据结构
            Dictionary<string, List<SafeInterval>> safeIntervals = BuildSafeIntervals(
                indoorSpace, globalOccupancyLayer, robotId, startTime);

            // 开始Anytime SIPP算法
            var result = RunAnytimeSIPP(
                graph,
                indoorSpace,
                safeIntervals,
                startPosition,
                endPosition,
                robotId,
                startTime,
                defaultSpeed,
                maxAcceleration,
                maxDeceleration,
                searchMethod,
                trafficRules,
                timeLimit,
                initialEpsilon,
                finalEpsilon,
                epsilonStep,
                globalOccupancyLayer);

            if (result.success)
            {
                Debug.Log($"Robot {robotId}: Anytime SIPP succeeded with epsilon = {result.epsilon:F2}, " +
                         $"path length = {result.path.Count}");
                return (result.path, result.times, result.speeds, newPlanId);
            }
            else
            {
                Debug.LogError($"Robot {robotId}: Anytime SIPP failed to find a path");
                return (new List<ConnectionPoint>(), new List<float>(), new List<float>(), null);
            }
        }

        /// <summary>
        /// 运行Anytime SIPP算法
        /// </summary>
        private static (bool success, List<ConnectionPoint> path, List<float> times, List<float> speeds, float epsilon) RunAnytimeSIPP(
            Graph graph,
            IndoorSpace indoorSpace,
            Dictionary<string, List<SafeInterval>> safeIntervals,
            RoutePoint startPosition,
            RoutePoint endPosition,
            string robotId,
            float startTime,
            float defaultSpeed,
            float maxAcceleration,
            float maxDeceleration,
            SearchMethod searchMethod,
            TrafficRuleKnowledge trafficRules,
            float timeLimit,
            float initialEpsilon,
            float finalEpsilon,
            float epsilonStep,
            DynamicOccupancyLayer globalOccupancyLayer)
        {
            Stopwatch stopwatch = new Stopwatch();
            stopwatch.Start();

            // 初始化结果变量
            bool foundSolution = false;
            List<ConnectionPoint> bestPath = null;
            List<float> bestTimes = null;
            List<float> bestSpeeds = null;
            float currentEpsilon = initialEpsilon;
            float bestEpsilon = initialEpsilon;

            // 初始化搜索状态
            string startPointId = startPosition.ConnectionPoint.Id;
            if (!safeIntervals.ContainsKey(startPointId))
            {
                safeIntervals[startPointId] = new List<SafeInterval> { new SafeInterval(startTime, float.MaxValue) };
            }

            // 获取起点安全间隔
            SafeInterval startInterval = GetSafeIntervalAt(safeIntervals, startPointId, startTime);
            if (startInterval == null)
            {
                Debug.LogError($"Robot {robotId}: No safe interval at start position at time {startTime:F2}");
                return (false, null, null, null, currentEpsilon);
            }

            int startIntervalIndex = safeIntervals[startPointId].IndexOf(startInterval);
            
            // 维护所有已知状态的集合
            Dictionary<string, SIPPState> allStates = new Dictionary<string, SIPPState>();
            
            // ARA*循环：随着时间的推移，逐渐减小epsilon
            while (currentEpsilon >= finalEpsilon && stopwatch.ElapsedMilliseconds < timeLimit * 1000)
            {
                Debug.Log($"Robot {robotId}: Running SIPP with epsilon = {currentEpsilon:F2}");

                // 初始化开放列表、闭合列表和INCONS列表
                PriorityQueue<SIPPState> openSet = new PriorityQueue<SIPPState>();
                HashSet<string> closedSet = new HashSet<string>();
                HashSet<string> inconsSet = new HashSet<string>();
                
                // 创建起始状态(初始状态为最优状态，即O(s)=true)
                float h = CalculateHeuristic(startPosition, endPosition, searchMethod, defaultSpeed);
                float fValue = currentEpsilon * (0 + h);  // 对于优化状态，f = ε * (g + h)
                
                SIPPState startState = new SIPPState(
                    startPosition,
                    startIntervalIndex,
                    startTime,
                    startTime,  // 初始离开时间 = 到达时间
                    0f,         // g-score = 0
                    fValue,
                    null,
                    startInterval,
                    defaultSpeed,
                    true,       // 优化状态
                    false,      // 无等待
                    0f);        // 等待时间为0

                // 将起始状态添加到开放列表
                openSet.Enqueue(startState, startState.FScore);
                string startStateKey = GetStateKey(startState);
                allStates[startStateKey] = startState;

                // 如果存在上一轮的搜索结果，将INCONS节点转移到OPEN
                if (inconsSet.Count > 0)
                {
                    foreach (var stateKey in inconsSet)
                    {
                        if (allStates.TryGetValue(stateKey, out SIPPState state))
                        {
                            // 重新计算f值
                            float newF;
                            if (state.IsOptimal)
                            {
                                newF = currentEpsilon * (state.GScore + CalculateHeuristic(state.RoutePoint, endPosition, searchMethod, defaultSpeed));
                            }
                            else
                            {
                                newF = state.GScore + currentEpsilon * CalculateHeuristic(state.RoutePoint, endPosition, searchMethod, defaultSpeed);
                            }
                            
                            SIPPState updatedState = state.WithNewFScore(newF);
                            openSet.Enqueue(updatedState, updatedState.FScore);
                            allStates[stateKey] = updatedState;
                        }
                    }
                    inconsSet.Clear();
                }

                // 运行ImprovePath函数(核心的SIPP搜索过程)
                SIPPState goalState = ImprovePath(
                    graph,
                    indoorSpace,
                    safeIntervals,
                    endPosition,
                    robotId,
                    defaultSpeed,
                    maxAcceleration,
                    maxDeceleration,
                    searchMethod,
                    trafficRules,
                    currentEpsilon,
                    openSet,
                    closedSet,
                    inconsSet,
                    allStates,
                    globalOccupancyLayer);

                if (goalState != null)
                {
                    // 找到了解决方案
                    foundSolution = true;
                    
                    // 重建路径
                    (List<ConnectionPoint> path, List<float> times, List<float> speeds) = ReconstructPath(goalState, defaultSpeed);
                    
                    // 保存最佳路径
                    bestPath = path;
                    bestTimes = times;
                    bestSpeeds = speeds;
                    bestEpsilon = currentEpsilon;

                    Debug.Log($"Robot {robotId}: Found valid path with epsilon = {currentEpsilon:F2}, " +
                              $"path length = {path.Count}, time = {stopwatch.ElapsedMilliseconds}ms");

                    // 检查是否还有时间继续优化
                    if (stopwatch.ElapsedMilliseconds < timeLimit * 1000)
                    {
                        // 减小epsilon以寻找更优解
                        currentEpsilon -= epsilonStep;
                    }
                    else
                    {
                        Debug.Log($"Robot {robotId}: Time limit reached, stopping optimization at epsilon = {currentEpsilon:F2}");
                        break;
                    }
                }
                else
                {
                    Debug.LogWarning($"Robot {robotId}: Failed to find path with epsilon = {currentEpsilon:F2}");
                    
                    // 如果这是第一次迭代且失败，增加epsilon尝试找到可行解
                    if (!foundSolution && currentEpsilon == initialEpsilon)
                    {
                        currentEpsilon += 1.0f;
                        Debug.Log($"Robot {robotId}: Increasing epsilon to {currentEpsilon:F2} to find initial solution");
                    }
                    else
                    {
                        // 否则使用之前找到的最佳解
                        break;
                    }
                }
            }

            stopwatch.Stop();
            Debug.Log($"Robot {robotId}: Anytime SIPP completed in {stopwatch.ElapsedMilliseconds}ms, " +
                      $"final epsilon = {bestEpsilon:F2}");

            return (foundSolution, bestPath, bestTimes, bestSpeeds, bestEpsilon);
        }

        /// <summary>
        /// ImprovePath函数 - 使用正确的SIPPState接口
        /// </summary>
        private static SIPPState ImprovePath(
            Graph graph,
            IndoorSpace indoorSpace,
            Dictionary<string, List<SafeInterval>> safeIntervals,
            RoutePoint endPosition,
            string robotId,
            float defaultSpeed,
            float maxAcceleration,
            float maxDeceleration,
            SearchMethod searchMethod,
            TrafficRuleKnowledge trafficRules,
            float epsilon,
            PriorityQueue<SIPPState> openSet,
            HashSet<string> closedSet,
            HashSet<string> inconsSet,
            Dictionary<string, SIPPState> allStates,
            DynamicOccupancyLayer globalOccupancyLayer)
        {
            // 记录目标状态
            SIPPState goalState = null;
            float goalFValue = float.MaxValue;
            
            // 标识目标位置
            string goalLocationId = endPosition.ConnectionPoint.Id;
            
            // 小的时间精度常量
            const float TIME_EPSILON = 0.001f;
            
            // 主循环 - while f(sgoal) > min_{s∈OPEN} f(s) do
            while (openSet.Count > 0 && (goalState == null || goalFValue > openSet.Peek().FScore))
            {
                // remove s with the smallest f(s) from OPEN
                SIPPState current = openSet.Dequeue();
                
                // 创建状态的标识符
                string stateKey = GetStateKey(current);
                
                // CLOSED = CLOSED ∪ {s}
                closedSet.Add(stateKey);
                
                // 检查是否到达目标
                if (current.RoutePoint.ConnectionPoint.Id == goalLocationId)
                {
                    goalState = current;
                    goalFValue = current.FScore;
                    // 不终止，继续搜索可能的更优路径
                }
                
                // for all motion m ∈ M(s) do
                List<Motion> motions = GetAvailableMotions(graph, current, defaultSpeed);
                
                foreach (var motion in motions)
                {
                    RoutePoint targetRP = motion.TargetPoint;
                    string targetId = targetRP.ConnectionPoint.Id;
                    
                    if (motion.Type == MotionType.Move)
                    {
                        bool wouldCreateLoop = false;
                        SIPPState checkState = current;
                        while (checkState != null)
                        {
                            if (checkState.RoutePoint.ConnectionPoint.Id == targetId)
                            {
                                wouldCreateLoop = true;
                                break;
                            }
                            checkState = checkState.Parent;
                        }
                        
                        if (wouldCreateLoop)
                        {
                            continue;
                        }
                    }
                    
                    // 获取当前状态的安全间隔
                    SafeInterval currentInterval = safeIntervals[current.RoutePoint.ConnectionPoint.Id][current.IntervalIndex];
                    
                    // 获取目标位置的所有安全间隔
                    if (!safeIntervals.ContainsKey(targetId))
                    {
                        continue;
                    }
                    
                    // 遍历目标位置的所有安全间隔
                    foreach (var targetInterval in safeIntervals[targetId])
                    {
                        // x' = configuration of m applied to x(s)
                        // 在我们的系统中，这就是targetRP

                        // tm = time to execute m
                        // 获取前一个点（用于交通规则）
                        RoutePoint prevPoint = current.Parent?.RoutePoint;
                        
                        // 计算执行动作m所需的时间tm（包含可能的等待）
                        var (tm, waitTime, moveTime) = CalculateActionTime(
                            motion,
                            current,
                            current.ArrivalTime, // g(s)
                            targetInterval,
                            defaultSpeed,
                            trafficRules,
                            prevPoint,
                            maxAcceleration,
                            maxDeceleration);
                        
                        // 如果tm小于最小时间精度，不考虑这个动作
                        if (tm < TIME_EPSILON)
                        {
                            continue;
                        }
                        
                        // tstart = g(s) + tm
                        float tstart = current.ArrivalTime + tm;
                        
                        // tend = endTime(interval(s)) + tm
                        float tend = currentInterval.EndTime + tm;
                        
                        // for all safe interval i ∈ x' do
                        // 注意：我们已经在遍历targetInterval，这就是i
                        
                        // if startTime(i) > tend or endTime(i) < tstart then continue
                        if (targetInterval.StartTime > tend || targetInterval.EndTime < tstart)
                        {
                            continue;
                        }
                        
                        // t = earliest arrival time at x' in interval i with no collisions
                        // 计算最早可能的到达时间
                        float earliestArrival = Math.Max(tstart, targetInterval.StartTime);
                        
                        // 计算出发时间 = 到达时间 - 移动时间
                        float departureTime = earliestArrival - moveTime;
                        
                        // 检查是否有碰撞
                        bool hasConflict = false;
                        if (motion.Type == MotionType.Move)
                        {
                            // 只有移动动作才需要检测碰撞
                            hasConflict = CheckPreciseConflict(
                                indoorSpace,
                                globalOccupancyLayer,
                                current.RoutePoint,
                                targetRP,
                                departureTime, // 离开时间
                                earliestArrival, // 到达时间
                                robotId,
                                "AnytimeSIPP_Check");
                        }
                        
                        // if t does not exist then continue
                        if (hasConflict)
                        {
                            continue;
                        }
                        
                        // 确定优化选项
                        List<bool> optimalOptions;
                        if (current.IsOptimal)
                        {
                            optimalOptions = new List<bool> { true, false };
                        }
                        else
                        {
                            optimalOptions = new List<bool> { false };
                        }
                        
                        // for all o ∈ Opt do
                        foreach (bool isOptimal in optimalOptions)
                        {
                            // s' = {x', i, o}
                            int intervalIndex = safeIntervals[targetId].IndexOf(targetInterval);
                            
                            // 创建后继状态的键
                            string successorKey = GetStateKey(targetId, intervalIndex, isOptimal);
                            
                            // 计算g值和f值
                            float gScore = earliestArrival; // g值就是到达时间
                            float hScore = CalculateHeuristic(targetRP, endPosition, searchMethod, defaultSpeed);
                            
                            // 计算f值
                            float fValue;
                            if (isOptimal)
                            {
                                // f(s') = ε * (g(s') + h(s'))
                                fValue = epsilon * (gScore + hScore);
                            }
                            else
                            {
                                // f(s') = g(s') + ε * h(s')
                                fValue = gScore + epsilon * hScore;
                            }
                            
                            // 计算是否需要等待以及等待时间
                            bool needsWait = waitTime > TIME_EPSILON;
                            
                            // 创建或更新状态
                            SIPPState successor;
                            
                            // 初始化未访问状态
                            if (allStates.TryGetValue(successorKey, out SIPPState existingState))
                            {
                                // if g(s') > t then
                                if (existingState.GScore > gScore)
                                {
                                    // g(s') = t
                                    
                                    // 更新状态 - 使用正确的SIPPState构造函数
                                    successor = new SIPPState(
                                        targetRP,
                                        intervalIndex,
                                        earliestArrival,  // arrivalTime
                                        departureTime,    // departureTime
                                        gScore,           // gScore
                                        fValue,           // fScore
                                        current,          // parent
                                        targetInterval,   // safeInterval
                                        motion.Type == MotionType.Move ? motion.Distance / moveTime : 0, // speed
                                        isOptimal,        // isOptimal
                                        needsWait,        // hasWait
                                        waitTime          // waitTime
                                    );
                                    
                                    allStates[successorKey] = successor;
                                    
                                    // if s' ∉ CLOSED then
                                    if (!closedSet.Contains(successorKey))
                                    {
                                        // 将状态添加到OPEN
                                        if (openSet.Contains(successor))
                                        {
                                            openSet.UpdatePriority(successor, fValue);
                                        }
                                        else
                                        {
                                            // 伪代码第35行: insert s' into OPEN with f(s')
                                            openSet.Enqueue(successor, fValue);
                                        }
                                    }
                                    else
                                    {
                                        // insert s' into INCONS
                                        inconsSet.Add(successorKey);
                                    }
                                }
                            }
                            else
                            {
                                // 首次见到这个状态
                                successor = new SIPPState(
                                    targetRP,
                                    intervalIndex,
                                    earliestArrival,  // arrivalTime
                                    departureTime,    // departureTime
                                    gScore,           // gScore
                                    fValue,           // fScore
                                    current,          // parent
                                    targetInterval,   // safeInterval
                                    motion.Type == MotionType.Move ? motion.Distance / moveTime : 0, // speed
                                    isOptimal,        // isOptimal
                                    needsWait,        // hasWait
                                    waitTime          // waitTime
                                );
                                
                                allStates[successorKey] = successor;
                                
                                // insert s' into OPEN with f(s')
                                openSet.Enqueue(successor, fValue);
                            }
                        }
                    }
                }
            }
            
            if (goalState != null)
            {
                Debug.Log($"ImprovePath完成: 找到目标, F值={goalFValue:F6}");
            }
            else
            {
                Debug.Log("ImprovePath完成: 未找到路径");
            }
            
            return goalState;
        }

        /// <summary>
        /// 获取状态的唯一标识符
        /// </summary>
        private static string GetStateKey(string locationId, int intervalIndex, bool isOptimal)
        {
            return $"{locationId}_{intervalIndex}_{(isOptimal ? 1 : 0)}";
        }

        /// <summary>
        /// 获取状态的唯一标识符
        /// </summary>
        private static string GetStateKey(SIPPState state)
        {
            return GetStateKey(state.RoutePoint.ConnectionPoint.Id, state.IntervalIndex, state.IsOptimal);
        }

        /// <summary>
        /// 获取可用的动作集合（包括等待和移动）
        /// </summary>
        private static List<Motion> GetAvailableMotions(
            Graph graph,
            SIPPState current,
            float defaultSpeed)
        {
            List<Motion> motions = new List<Motion>();
            
            // 1. 添加等待动作 - 在原地不动
            motions.Add(new Motion(
                current.RoutePoint,  // 保持在同一位置
                0,                   // 距离为0
                MotionType.Wait      // 动作类型为等待
            ));
            
            // 2. 添加移动到子节点的动作
            foreach (var childCP in current.RoutePoint.Children)
            {
                var childRP = graph.GetRoutePointFormConnectionPoint(childCP);
                if (childRP != null)
                {
                    float distance = CalculateDistance(current.RoutePoint, childRP);
                    motions.Add(new Motion(
                        childRP,              // 目标位置
                        distance,             // 距离
                        MotionType.Move       // 动作类型为移动
                    ));
                }
            }
            
            return motions;
        }

        /// <summary>
        /// 计算动作执行时间tm，可能包含等待时间
        /// </summary>
        private static (float tm, float waitTime, float moveTime) CalculateActionTime(
            Motion motion,
            SIPPState current,
            float currentTime,
            SafeInterval targetInterval,
            float defaultSpeed,
            TrafficRuleKnowledge trafficRules,
            RoutePoint prevPoint,
            float maxAcceleration,
            float maxDeceleration)
        {
            // 初始化返回值
            float tm = 0;
            float waitTime = 0;
            float moveTime = 0;
            
            if (motion.Type == MotionType.Wait)
            {
                // 等待动作 - 计算等待到目标间隔开始的时间
                waitTime = Math.Max(0, targetInterval.StartTime - currentTime);
                tm = waitTime;
            }
            else if (motion.Type == MotionType.Move)
            {
                // 移动动作 - 先计算纯移动时间
                var (modifiedSpeed, travelTime) = trafficRules.ModifySpeed(
                    current.RoutePoint,
                    motion.TargetPoint,
                    current.Speed,
                    defaultSpeed,
                    motion.Distance,
                    maxAcceleration,
                    maxDeceleration,
                    prevPoint);
                    
                moveTime = travelTime;
                
                // 计算执行动作后的到达时间
                float arrivalTimeWithoutWait = currentTime + moveTime;
                
                // 如果到达时间早于目标间隔的开始，需要等待
                if (arrivalTimeWithoutWait < targetInterval.StartTime)
                {
                    waitTime = targetInterval.StartTime - arrivalTimeWithoutWait;
                    // 注意：这里的等待是在目标位置等待，不是在出发前等待
                }
                
                // 总执行时间 = 移动时间 + 等待时间
                tm = moveTime + waitTime;
            }
            
            return (tm, waitTime, moveTime);
        }

        /// <summary>
        /// 精确的ConnectionPoint路径冲突检测
        /// </summary>
        private static bool CheckPreciseConflict(
            IndoorSpace indoorSpace,
            DynamicOccupancyLayer globalOccupancyLayer,
            RoutePoint fromPoint,
            RoutePoint toPoint,
            float startTime,
            float endTime,
            string robotId,
            string source = "AnytimeSIPP_Precise")
        {
            // 获取对应的ConnectionPoints
            ConnectionPoint fromCP = fromPoint.ConnectionPoint;
            ConnectionPoint toCP = toPoint.ConnectionPoint;
            
            if (fromCP == null || toCP == null)
            {
                return false; // 不是连接点，无法检查
            }
            
            // 获取相关CellSpace
            string fromSourceCellId = fromCP.CellBoundary.Source;
            string fromTargetCellId = fromCP.CellBoundary.Target;
            string toSourceCellId = toCP.CellBoundary.Source;
            string toTargetCellId = toCP.CellBoundary.Target;
            
            // 确定实际经过的单元格
            // 通常，fromCell的Target应该等于toCell的Source
            string middleCellId = "";
            
            // 寻找共享的单元格
            if (fromSourceCellId == toSourceCellId || fromSourceCellId == toTargetCellId)
            {
                middleCellId = fromSourceCellId;
            }
            else if (fromTargetCellId == toSourceCellId || fromTargetCellId == toTargetCellId)
            {
                middleCellId = fromTargetCellId;
            }
            
            if (string.IsNullOrEmpty(middleCellId))
            {
                Debug.LogWarning($"[{source}] 无法确定从{fromCP.Id}到{toCP.Id}的共享单元格");
                
                // 创建一个小的路径来检查这个段 - 回退到标准检测
                List<ConnectionPoint> miniPath = new List<ConnectionPoint> { fromCP, toCP };
                List<float> miniTimes = new List<float> { startTime, endTime };
                
                var conflicts = conflictDetector.DetectConflicts(
                    globalOccupancyLayer,
                    indoorSpace,
                    miniPath,
                    miniTimes,
                    robotId,
                    source);
                
                return conflicts.Count > 0;
            }
            
            // 获取中间单元格的时间占用
            var timeOccupancies = globalOccupancyLayer.GetTimeOccupanciesForCellSpace(middleCellId);
            
            // 过滤掉自己的占用
            timeOccupancies = timeOccupancies.Where(o => o.Occupier.RobotId != robotId).ToList();
            
            // 检查是否与任何现有占用重叠
            float transitionStart = startTime;
            float transitionEnd = endTime;
            
            const float timeEpsilon = 0.005f; // 5毫秒的容差
            
            foreach (var occupancy in timeOccupancies)
            {
                // 检查区间重叠
                // [a, b]和[c, d]重叠的条件: !(b < c || d < a)
                // 注意：我们使用严格小于，因为恰好相等的情况应该是允许的
                if (!(transitionEnd + timeEpsilon < occupancy.StartTime || 
                    occupancy.EndTime + timeEpsilon < transitionStart))
                {
                    Debug.LogWarning($"[{source}] 在单元格{middleCellId}检测到精确冲突: " +
                                $"计划时间[{transitionStart:F6}-{transitionEnd:F6}] 与 " +
                                $"占用[{occupancy.StartTime:F6}-{occupancy.EndTime:F6}]重叠");
                    return true;
                }
            }
            
            // 检查起点和终点单元格（如果不同于中间单元格）
            string sourceCellId = fromSourceCellId;
            if (sourceCellId != middleCellId)
            {
                var sourceOccupancies = globalOccupancyLayer.GetTimeOccupanciesForCellSpace(sourceCellId)
                    .Where(o => o.Occupier.RobotId != robotId).ToList();
                
                foreach (var occupancy in sourceOccupancies)
                {
                    if (!(startTime + timeEpsilon < occupancy.StartTime || 
                        occupancy.EndTime + timeEpsilon < startTime))
                    {
                        Debug.LogWarning($"[{source}] 在起点单元格{sourceCellId}检测到冲突");
                        return true;
                    }
                }
            }
            
            string targetCellId = toTargetCellId;
            if (targetCellId != middleCellId && targetCellId != sourceCellId)
            {
                var targetOccupancies = globalOccupancyLayer.GetTimeOccupanciesForCellSpace(targetCellId)
                    .Where(o => o.Occupier.RobotId != robotId).ToList();
                
                foreach (var occupancy in targetOccupancies)
                {
                    if (!(endTime + timeEpsilon < occupancy.StartTime || 
                        occupancy.EndTime + timeEpsilon < endTime))
                    {
                        Debug.LogWarning($"[{source}] 在终点单元格{targetCellId}检测到冲突");
                        return true;
                    }
                }
            }
            
            // 如果没有检测到冲突，则认为路径是安全的
            return false;
        }

        /// <summary>
        /// 构建安全时间间隔
        /// </summary>
        private static Dictionary<string, List<SafeInterval>> BuildSafeIntervals(
            IndoorSpace indoorSpace,
            DynamicOccupancyLayer globalOccupancyLayer,
            string robotId,
            float startTime)
        {
            Dictionary<string, List<SafeInterval>> safeIntervals = new Dictionary<string, List<SafeInterval>>();

            // 为每个ConnectionPoint创建安全时间间隔
            foreach (var cellBoundary in indoorSpace.CellBoundaries)
            {
                string connectionPointId = cellBoundary.Id;
                
                // 获取与此ConnectionPoint相关的两个CellSpace
                string sourceCellId = cellBoundary.Source;
                string targetCellId = cellBoundary.Target;
                
                // 获取这些CellSpace对应的时间占用情况
                var sourceCellOccupancies = globalOccupancyLayer.GetTimeOccupanciesForCellSpace(sourceCellId);
                var targetCellOccupancies = globalOccupancyLayer.GetTimeOccupanciesForCellSpace(targetCellId);
                
                // 过滤掉当前机器人自己的占用
                sourceCellOccupancies = sourceCellOccupancies.Where(o => o.Occupier.RobotId != robotId).ToList();
                targetCellOccupancies = targetCellOccupancies.Where(o => o.Occupier.RobotId != robotId).ToList();
                
                List<TimeOccupancy> connectionPointOccupancies = new List<TimeOccupancy>();
                
                // 处理从source到target的移动
                foreach (var sourceOcc in sourceCellOccupancies)
                {
                    // 寻找对应的目标单元格占用
                    var matchingTargetOccs = targetCellOccupancies.Where(to => 
                        to.Occupier.RobotId == sourceOcc.Occupier.RobotId &&
                        Math.Abs(to.StartTime - sourceOcc.EndTime) < 0.5f // 假设0.5秒是合理的过渡时间
                    ).ToList();
                    
                    foreach (var targetOcc in matchingTargetOccs)
                    {
                        // 创建连接点占用: 从source离开到target进入的时间段
                        // 这表示机器人正在穿过连接点
                        float cpStartTime = sourceOcc.EndTime - 0.1f; // 稍早开始
                        float cpEndTime = targetOcc.StartTime + 0.1f; // 稍晚结束
                        
                        // 确保时间有效
                        if (cpEndTime > cpStartTime)
                        {
                            var cpOccupancy = new TimeOccupancy(
                                cpStartTime,
                                cpEndTime,
                                sourceOcc.Occupier,
                                false,
                                false);
                            
                            connectionPointOccupancies.Add(cpOccupancy);
                        }
                    }
                }
                
                // 按开始时间排序
                connectionPointOccupancies = connectionPointOccupancies.OrderBy(o => o.StartTime).ToList();
                
                // 创建安全间隔
                var intervals = new List<SafeInterval>();
                
                // 如果没有占用，整个时间轴都是安全的
                if (connectionPointOccupancies.Count == 0)
                {
                    intervals.Add(new SafeInterval(startTime, float.MaxValue));
                }
                else
                {
                    // 添加第一个占用前的安全间隔
                    if (connectionPointOccupancies[0].StartTime > startTime)
                    {
                        intervals.Add(new SafeInterval(startTime, connectionPointOccupancies[0].StartTime));
                    }
                    
                    // 添加占用之间的安全间隔
                    for (int i = 0; i < connectionPointOccupancies.Count - 1; i++)
                    {
                        float intervalStart = connectionPointOccupancies[i].EndTime;
                        float intervalEnd = connectionPointOccupancies[i + 1].StartTime;
                        
                        // 确保间隔有效
                        if (intervalEnd > intervalStart && intervalEnd > startTime)
                        {
                            // 确保开始时间不早于startTime
                            intervalStart = Math.Max(intervalStart, startTime);
                            
                            // 添加一点安全余量
                            intervalStart += 0.05f;
                            intervalEnd -= 0.05f;
                            
                            if (intervalEnd > intervalStart)
                            {
                                intervals.Add(new SafeInterval(intervalStart, intervalEnd));
                            }
                        }
                    }
                    
                    // 添加最后一个占用后的安全间隔
                    float lastEnd = connectionPointOccupancies[connectionPointOccupancies.Count - 1].EndTime;
                    if (lastEnd >= startTime)
                    {
                        // 添加小的安全余量
                        intervals.Add(new SafeInterval(lastEnd + 0.05f, float.MaxValue));
                    }
                }
                
                safeIntervals[connectionPointId] = intervals;
            }
            
            return safeIntervals;
        }

        /// <summary>
        /// 获取给定位置和时间的安全时间间隔
        /// </summary>
        private static SafeInterval GetSafeIntervalAt(
            Dictionary<string, List<SafeInterval>> safeIntervals,
            string locationId,
            float time)
        {
            if (!safeIntervals.ContainsKey(locationId))
            {
                return null;
            }

            foreach (var interval in safeIntervals[locationId])
            {
                if (interval.Contains(time))
                {
                    return interval;
                }
            }

            return null;
        }

        /// <summary>
        /// 计算启发式值（预估到目标的时间）
        /// </summary>
        private static float CalculateHeuristic(
            RoutePoint from,
            RoutePoint to,
            SearchMethod searchMethod,
            float speed)
        {
            if (from == null || to == null || from.ConnectionPoint == null || to.ConnectionPoint == null)
            {
                return 0f;
            }

            Point fromPoint = (Point)from.ConnectionPoint.Point;
            Point toPoint = (Point)to.ConnectionPoint.Point;

            Vector2 fromVector = new Vector2((float)fromPoint.X, (float)fromPoint.Y);
            Vector2 toVector = new Vector2((float)toPoint.X, (float)toPoint.Y);

            float distance = 0f;

            switch (searchMethod)
            {
                case SearchMethod.Euclidean_Distance:
                    distance = Vector2.Distance(fromVector, toVector);
                    break;
                case SearchMethod.Manhattan_Distance:
                    distance = Math.Abs(fromVector.x - toVector.x) + Math.Abs(fromVector.y - toVector.y);
                    break;
                default:
                    distance = Vector2.Distance(fromVector, toVector);
                    break;
            }

            // 返回估计的时间成本 - 距离除以速度 = 时间
            return distance / speed;
        }

        /// <summary>
        /// 计算两个路由点之间的距离
        /// </summary>
        private static float CalculateDistance(RoutePoint from, RoutePoint to)
        {
            Point fromPoint = (Point)from.ConnectionPoint.Point;
            Point toPoint = (Point)to.ConnectionPoint.Point;

            Vector2 fromVector = new Vector2((float)fromPoint.X, (float)fromPoint.Y);
            Vector2 toVector = new Vector2((float)toPoint.X, (float)toPoint.Y);

            return Vector2.Distance(fromVector, toVector);
        }

        /// <summary>
        /// 根据目标状态重建完整路径，直接使用状态中标记的等待信息
        /// </summary>
        private static (List<ConnectionPoint> path, List<float> times, List<float> speeds) ReconstructPath(
            SIPPState goalState,
            float defaultSpeed)
        {
            List<SIPPState> statesReversed = new List<SIPPState>();
            
            // 从目标状态回溯到起始状态
            SIPPState current = goalState;
            while (current != null)
            {
                statesReversed.Add(current);
                current = current.Parent;
            }
            
            // 反转列表获得正向顺序
            List<SIPPState> states = new List<SIPPState>(statesReversed);
            states.Reverse();
            
            // 准备路径、时间和速度列表
            List<ConnectionPoint> path = new List<ConnectionPoint>();
            List<float> times = new List<float>();
            List<float> speeds = new List<float>();
            
            // 处理第一个状态
            if (states.Count > 0)
            {
                path.Add(states[0].RoutePoint.ConnectionPoint);
                times.Add(states[0].ArrivalTime);
                speeds.Add(states[0].Speed);
            }
            
            // 处理后续状态，显式添加等待
            for (int i = 1; i < states.Count; i++)
            {
                SIPPState state = states[i];
                
                // 如果前一个状态包含等待，添加等待点
                SIPPState prevState = states[i-1];
                if (prevState.HasWait && prevState.WaitTime > 0.01f) // 使用小阈值避免浮点误差
                {
                    // 添加等待点 (与前一点位置相同，表示原地等待)
                    path.Add(prevState.RoutePoint.ConnectionPoint);
                    times.Add(prevState.ArrivalTime + prevState.WaitTime); // 等待结束时间
                    speeds.Add(0.0f); // 等待速度为0
                }
                
                // 添加当前状态
                path.Add(state.RoutePoint.ConnectionPoint);
                times.Add(state.ArrivalTime);
                speeds.Add(state.Speed);
            }
            
            // 如果最后一个状态包含等待，也添加等待点
            if (states.Count > 0)
            {
                SIPPState lastState = states[states.Count - 1];
                if (lastState.HasWait && lastState.WaitTime > 0.01f)
                {
                    path.Add(lastState.RoutePoint.ConnectionPoint);
                    times.Add(lastState.ArrivalTime + lastState.WaitTime);
                    speeds.Add(0.0f);
                }
            }
            
            return (path, times, speeds);
        }
    }
}