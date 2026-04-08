using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using UnityEngine;
using Map;
using Debug = UnityEngine.Debug;
using CentralControl;

namespace PathPlanning
{
    /// <summary>
    /// AnytimeSIPP (Safe Interval Path Planning)
    /// </summary>
    public class AnytimeSIPP
    {
        #region private fields

        private static readonly Dictionary<string, int> floorLevelCache = new Dictionary<string, int>();

        private readonly struct SIPPStateKey : IEquatable<SIPPStateKey>
        {
            public readonly string ConnectionPointId;
            public readonly int IntervalIndex;
            public readonly string FloorLevel;

            public SIPPStateKey(string connectionPointId, int intervalIndex, string floorLevel)
            {
                ConnectionPointId = connectionPointId;
                IntervalIndex = intervalIndex;
                FloorLevel = floorLevel;
            }

            public bool Equals(SIPPStateKey other) =>
                ConnectionPointId == other.ConnectionPointId &&
                IntervalIndex == other.IntervalIndex &&
                FloorLevel == other.FloorLevel;

            public override bool Equals(object obj) => obj is SIPPStateKey other && Equals(other);

            public override int GetHashCode()
            {
                unchecked
                {
                    int hash = ConnectionPointId?.GetHashCode() ?? 0;
                    hash = (hash * 397) ^ IntervalIndex;
                    hash = (hash * 397) ^ (FloorLevel?.GetHashCode() ?? 0);
                    return hash;
                }
            }
        }

        /// <summary>
        /// SIPP State class, representing a node in the search process
        /// </summary>
        private class SIPPState : IComparable<SIPPState>
        {
            public Waypoint Waypoint { get; private set; }
            public int IntervalIndex { get; private set; }
            public float ArrivalTime { get; private set; }
            public float DepartureTime { get; private set; }
            public float GScore { get; private set; }
            public float FScore { get; private set; }
            public SIPPState Parent { get; private set; }
            public SafeInterval SafeInterval { get; private set; }
            public float Speed { get; private set; }

            // 3D related properties
            public string FloorLevel { get; private set; }
            public bool IsVerticalTransition { get; private set; }
            public string VerticalConnectorId { get; private set; }

            // waiting related properties
            public bool HasWait { get; private set; }
            public float WaitTime { get; private set; }

            public bool IsOptimal { get; private set; }

            public SIPPState(
                Waypoint waypoint,
                int intervalIndex,
                float arrivalTime,
                float departureTime,
                float gScore,
                float fScore,
                SIPPState parent,
                SafeInterval safeInterval,
                float speed,
                string floorLevel,
                bool isVerticalTransition = false,
                string verticalConnectorId = null,
                bool isOptimal = false)
            {
                this.Waypoint = waypoint;
                this.IntervalIndex = intervalIndex;
                this.ArrivalTime = arrivalTime;
                this.DepartureTime = departureTime;
                this.GScore = gScore;
                this.FScore = fScore;
                this.Parent = parent;
                this.SafeInterval = safeInterval;
                this.Speed = speed;
                this.FloorLevel = floorLevel;
                this.IsVerticalTransition = isVerticalTransition;
                this.VerticalConnectorId = verticalConnectorId;
                this.IsOptimal = isOptimal;

                this.WaitTime = departureTime - arrivalTime;
                this.HasWait = WaitTime > 0.001f;
            }

            public int CompareTo(SIPPState other)
            {
                int fScoreComparison = FScore.CompareTo(other.FScore);
                if (fScoreComparison != 0)
                {
                    return fScoreComparison;
                }
                
                // if f-value is equal, prefer the one with larger g-value
                return -GScore.CompareTo(other.GScore);
            }
            
            public SIPPStateKey GetStateKey()
            {
                return new SIPPStateKey(Waypoint.Id, IntervalIndex, FloorLevel);
            }
        }

        /// <summary>
        /// Safe Interval class, representing a time interval when a location is safe to occupy
        /// </summary>
        private class SafeInterval
        {
            public float Start { get; private set; }
            public float End { get; private set; }

            public SafeInterval(float start, float end)
            {
                this.Start = start;
                this.End = end;
            }

            public bool Contains(float time)
            {
                return time >= Start && time <= End;
            }

            public override string ToString()
            {
                return $"[{Start:F2}, {End:F2}]";
            }
        }

        /// <summary>
        /// Vertical connection information class, recording vertical movement information
        /// </summary>
        private class VerticalConnectionInfo
        {
            public string CellSpaceId { get; set; }
            public string FromFloor { get; set; }
            public string ToFloor { get; set; }
            public float StartTime { get; set; }
            public float TravelTime { get; set; }
            public float EndTime => StartTime + TravelTime;
        }

        #endregion

        #region private fields
        private const float ELEVATOR_BASE_TIME = 3.0f;
        private const float ELEVATOR_FLOOR_TIME = 0.5f;
        private const float STAIR_FLOOR_TIME = 2.0f;
        private const float DEFAULT_VERTICAL_TIME = 6.0f;
        private const float TIME_EPSILON = 0.001f;
        private const float INTENT_COST_FACTOR = 1.3f;
        
        #endregion

        #region main methods

        /// <summary>
        /// Anytime SIPP Algorithm Entry Point
        /// </summary>
        public static (List<Waypoint> path, List<float> times, List<float> speeds, string planId, PathPlanningMetrics metrics) SIPPAlgorithm(
            NavigationGraph graph,
            IndoorSpace indoorSpace,
            DynamicOccupancyLayer personalOccupancyLayer,
            DynamicOccupancyLayer globalOccupancyLayer,
            Waypoint startPosition,
            Waypoint endPosition,
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
                Debug.LogError($"Robot {robotId}: SIPP parameters are invalid");
                return (new List<Waypoint>(), new List<float>(), new List<float>(), null, null);
            }

            // Early return for trivial case: start and end are the same position
            if (startPosition.Id == endPosition.Id)
            {
                string trivialPlanId = $"SIPPPlan_{robotId}_{DateTime.Now.Ticks}";
                var trivialPath = new List<Waypoint> { startPosition };
                var trivialTimes = new List<float> { startTime };
                var trivialSpeeds = new List<float> { defaultSpeed };
                var trivialMetrics = new PathPlanningMetrics(robotId, trivialPlanId);
                trivialMetrics.PlanningTime = 0f;
                Debug.Log($"Robot {robotId}: Start == End ({startPosition.Id}), returning trivial path");
                return (trivialPath, trivialTimes, trivialSpeeds, trivialPlanId, trivialMetrics);
            }

            // Initialize
            TrafficRuleKnowledge trafficRules = new TrafficRuleKnowledge(indoorSpace);

            string newPlanId = $"SIPPPlan_{robotId}_{DateTime.Now.Ticks}";

            float timeLimit = 1.0f;
            float initialEpsilon = 3.0f;
            float finalEpsilon = 1.0f;
            float epsilonStep = 0.5f;

            var verticalConnectionInfo = new Dictionary<string, VerticalConnectionInfo>();

            // Build safe intervals according to the occupancy layer for robot
            Dictionary<string, List<SafeInterval>> safeIntervals = BuildSafeIntervals(
                indoorSpace, globalOccupancyLayer, robotId, startTime, CalculateSafeIntervalHorizon(indoorSpace, startPosition, endPosition, defaultSpeed));

            string startFloor = startPosition.FloorLevel;
            string endFloor = endPosition.FloorLevel;
            Debug.Log($"Robot {robotId}: startPoint {startPosition.Id}, endPoint {endPosition.Id}");

            PathPlanningMetrics metrics = new PathPlanningMetrics(robotId, newPlanId ?? "unknown");
            Debug.Log($"Robot {robotId}: Start 3D path planning，from Floor {startFloor} to Floor {endFloor}");

            // Start Anytime SIPP
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
                globalOccupancyLayer,
                verticalConnectionInfo,
                metrics);

            // If no path was found, return nulls to caller
            if (!result.success)
            {
                return (null, null, null, null, null);
            }

            // Vertical connection occupancy is NOT written directly to the global layer here.
            // It flows through the normal path commit (AtomicSwapPath) along with all other cells,
            // ensuring occupancy is only committed after conflict checking passes.

            return (result.path, result.times, result.speeds, newPlanId, metrics);
        }

        /// <summary>
        /// Run Anytime SIPP Algorithm
        /// </summary>
        private static (bool success, List<Waypoint> path, List<float> times, List<float> speeds, float epsilon) RunAnytimeSIPP(
            NavigationGraph graph,
            IndoorSpace indoorSpace,
            Dictionary<string, List<SafeInterval>> safeIntervals,
            Waypoint startPosition,
            Waypoint endPosition,
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
            DynamicOccupancyLayer globalOccupancyLayer,
            Dictionary<string, VerticalConnectionInfo> verticalConnectionInfo,
            PathPlanningMetrics metrics = null)
        {
            Stopwatch stopwatch = new Stopwatch();
            stopwatch.Start();

            // Initialize variables
            bool foundSolution = false;
            List<Waypoint> bestPath = null;
            List<float> bestTimes = null;
            List<float> bestSpeeds = null;
            float currentEpsilon = initialEpsilon;
            float bestEpsilon = initialEpsilon;

            string startFloor = startPosition.FloorLevel;
            string endFloor = endPosition.FloorLevel;

            // Calculate the ideal path length as shortest graph distance (no occupancy)
            float idealPathLength = graph.ComputeShortestDistance(startPosition.Id, endPosition.Id);
            if (idealPathLength < 0)
                idealPathLength = CalculateHeuristicDistance(startPosition, endPosition, searchMethod);
            metrics.IdealPathLength = idealPathLength;

            // 1-3: OPEN = CLOSED = INCONS = ∅
            PriorityQueue<SIPPState> openSet = new PriorityQueue<SIPPState>();
            HashSet<SIPPStateKey> closedSet = new HashSet<SIPPStateKey>();
            HashSet<SIPPStateKey> inconsSet = new HashSet<SIPPStateKey>();
            
            Dictionary<SIPPStateKey, SIPPState> allStates = new Dictionary<SIPPStateKey, SIPPState>();

            // 1-4: insert s_start into OPEN with ε * h(s_start)
            // Get the initial safe interval for the start position (using CellSpace key)
            string startCellKey = GetCellKeyForWaypoint(startPosition, indoorSpace);
            SafeInterval initialInterval = GetSafeIntervalContainingTime(
                safeIntervals, startCellKey, startTime);

            if (initialInterval == null)
            {
                // The robot is physically at this position but another robot's planned
                // occupancy blocks it. Since the robot IS here and cannot teleport away,
                // extend the earliest existing safe interval backward to startTime.
                // SIPP's wait-successor mechanism will naturally compute the optimal
                // departure time; CentralController's conflict detection handles any
                // residual conflicts post-planning.
                if (safeIntervals.TryGetValue(startCellKey, out var existingIntervals) && existingIntervals.Count > 0)
                {
                    // Extend the first (earliest) interval backward to startTime
                    var earliest = existingIntervals[0];
                    initialInterval = new SafeInterval(startTime, earliest.End);
                    existingIntervals[0] = initialInterval;
                }
                else
                {
                    // No intervals at all — create an open-ended one
                    initialInterval = new SafeInterval(startTime, float.MaxValue);
                    safeIntervals[startCellKey] = new List<SafeInterval> { initialInterval };
                }
                Debug.LogWarning($"Robot {robotId}: No safe interval at start {startCellKey} at time {startTime}, extended earliest interval to [{startTime}, {initialInterval.End}]");
            }

            // Calculate the initial heuristic value
            float initialHeuristic = CalculateHeuristic(
                startPosition, endPosition, searchMethod, defaultSpeed, startFloor, endFloor);

            // Create the initial state
            SIPPState initialState = new SIPPState(
                startPosition,
                0,
                startTime,
                startTime,
                0,
                initialHeuristic * currentEpsilon,
                null,
                initialInterval,
                defaultSpeed,
                startFloor);

            openSet.Enqueue(initialState, initialState.FScore);
            SIPPStateKey initialStateKey = initialState.GetStateKey();
            allStates[initialStateKey] = initialState;

            bool finalEpsilonRun = false;
            // 1-8: while ε > 1 do
            while (currentEpsilon > finalEpsilon || (currentEpsilon == finalEpsilon && !finalEpsilonRun))
            {
                Debug.Log($"Robot {robotId}: Attempt to plan for epsilon={currentEpsilon:F1}");

                if (foundSolution)
                {
                    // A3 fix: Construct new OPEN from only:
                    // 1. States remaining in OPEN (not yet expanded — i.e., not in CLOSED)
                    // 2. States in INCONS (improved g-value while already in CLOSED)
                    // Do NOT re-enqueue states that were in CLOSED but not in INCONS.
                    // Note: closedSet/inconsSet still hold results from previous ImprovePath call.

                    // Collect state keys that should be in the new OPEN
                    HashSet<SIPPStateKey> newOpenKeys = new HashSet<SIPPStateKey>();

                    foreach (var kvp in allStates)
                    {
                        SIPPStateKey key = kvp.Key;
                        if (!closedSet.Contains(key) || inconsSet.Contains(key))
                        {
                            newOpenKeys.Add(key);
                        }
                    }

                    openSet = new PriorityQueue<SIPPState>();

                    // 1-11: Update the priorities for all s ∈ (OPEN ∪ INCONS) according to f(s)
                    foreach (var key in newOpenKeys)
                    {
                        if (!allStates.TryGetValue(key, out SIPPState state)) continue;

                        if (!state.IsOptimal)
                        {
                            float newHeuristic = CalculateHeuristic(
                                state.Waypoint, endPosition, searchMethod, defaultSpeed, state.FloorLevel, endFloor);

                            float newFScore = state.GScore + newHeuristic * currentEpsilon;

                            SIPPState updatedState = new SIPPState(
                                state.Waypoint,
                                state.IntervalIndex,
                                state.ArrivalTime,
                                state.DepartureTime,
                                state.GScore,
                                newFScore,
                                state.Parent,
                                state.SafeInterval,
                                state.Speed,
                                state.FloorLevel,
                                state.IsVerticalTransition,
                                state.VerticalConnectorId);

                            allStates[key] = updatedState;
                            openSet.Enqueue(updatedState, updatedState.FScore);
                        }
                    }
                }

                // Clear CLOSED and INCONS for the new iteration (after using them above)
                closedSet.Clear();
                inconsSet.Clear();

                // 1-13: ImprovePath()
                var result = ImprovePath(
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
                    globalOccupancyLayer,
                    verticalConnectionInfo,
                    endFloor);

                if (result != null)
                {
                    foundSolution = true;
                    bestEpsilon = currentEpsilon;
                    
                    var pathResult = ReconstructPath(result, defaultSpeed);
                    bestPath = pathResult.path;
                    bestTimes = pathResult.times;
                    bestSpeeds = pathResult.speeds;
                    
                    Debug.Log($"Robot {robotId}: Path is found! (epsilon={currentEpsilon:F1}), Length: {bestPath.Count}, Time: {bestTimes.Last():F2}, Path: {string.Join(" -> ", bestPath.Select(p => p.Id))}");
                    
                    if (currentEpsilon == finalEpsilon)
                    {
                        finalEpsilonRun = true;
                    }
                    
                    // 1-9: Decrease ε
                    currentEpsilon = Math.Max(currentEpsilon - epsilonStep, finalEpsilon);
                }
                else
                {
                    Debug.LogWarning($"Robot {robotId}: Cannot find path when epsilon={currentEpsilon:F1}");
                    break;
                }
                
                if (stopwatch.ElapsedMilliseconds >= timeLimit * 1000)
                {
                    Debug.Log($"Robot {robotId}: time out ({timeLimit}s) when planning for epsilon={currentEpsilon:F1}");
                    break;
                }
            }

            stopwatch.Stop();

            if (!foundSolution)
            {
                Debug.LogWarning($"SIPP: No path found for robot {robotId} after {stopwatch.ElapsedMilliseconds}ms");
                return (false, null, null, null, initialEpsilon);
            }

            Debug.Log($"Robot {robotId}: SIPP success! Time: {stopwatch.ElapsedMilliseconds}ms, epsilon={bestEpsilon:F1}");

            float actualPathLength = CalculatePathLength(bestPath);
            metrics.ActualPathLength = actualPathLength;
            metrics.TravelTime = bestTimes.Last() - bestTimes.First();
            metrics.PlanningTime = stopwatch.ElapsedMilliseconds / 1000f;

            if (EventManager.Instance != null)
            {
                // Metrics event is now fired from RobotController.PlanPath() to preserve IsReplan flag
            }

            return (foundSolution, bestPath, bestTimes, bestSpeeds, bestEpsilon);
        }

        /// <summary>
        /// ImprovePath function - Improves the path using the specified epsilon value
        /// </summary>
        private static SIPPState ImprovePath(
            NavigationGraph graph,
            IndoorSpace indoorSpace,
            Dictionary<string, List<SafeInterval>> safeIntervals,
            Waypoint endPosition,
            string robotId,
            float defaultSpeed,
            float maxAcceleration,
            float maxDeceleration,
            SearchMethod searchMethod,
            TrafficRuleKnowledge trafficRules,
            float epsilon,
            PriorityQueue<SIPPState> openSet,
            HashSet<SIPPStateKey> closedSet,
            HashSet<SIPPStateKey> inconsSet,
            Dictionary<SIPPStateKey, SIPPState> allStates,
            DynamicOccupancyLayer globalOccupancyLayer,
            Dictionary<string, VerticalConnectionInfo> verticalConnectionInfo,
            string goalFloor)
        {
            SIPPState goalState = null;
            float goalFValue = float.MaxValue;
            
            string goalLocationId = endPosition.Id;
            
            // 2-1: while f(s_goal) > min_{s ∈ OPEN} {f(s)} do
            while (openSet.Count > 0 && (goalState == null || goalFValue > openSet.Peek().FScore))
            {
                // 2-2: remove s with the smallest f(s) from OPEN
                SIPPState current = openSet.Dequeue();

                SIPPStateKey stateKey = current.GetStateKey();

                // Stale-pop guard: skip if allStates has a better state for this key.
                // When a successor with a lower g-score is found, allStates is updated
                // and the new state is enqueued, but the old (worse) entry remains in
                // the heap. This guard discards those stale entries on dequeue.
                if (allStates.TryGetValue(stateKey, out var bestKnown) && bestKnown != current)
                {
                    continue;
                }

                // Defense-in-depth: skip states already expanded in this iteration
                if (closedSet.Contains(stateKey))
                {
                    continue;
                }

                // 2-3: CLOSED = CLOSED ∪ {s}
                closedSet.Add(stateKey);
                
                // Check if we've reached the goal
                if (current.Waypoint.Id == goalLocationId &&
                    current.FloorLevel == goalFloor)
                {
                    if (goalState == null || current.GScore < goalState.GScore)
                    {
                        goalState = current;
                        goalFValue = current.FScore;
                    }
                    continue;
                }
                
                // 2-4 - 2-42: Generate and process successors
                List<SIPPState> successors = GenerateSuccessors(
                    graph,
                    indoorSpace,
                    safeIntervals,
                    current,
                    endPosition,
                    robotId,
                    defaultSpeed,
                    maxAcceleration,
                    maxDeceleration,
                    searchMethod,
                    trafficRules,
                    epsilon,
                    globalOccupancyLayer,
                    verticalConnectionInfo,
                    goalFloor
                );
                
                // Process each successor
                foreach (var successor in successors)
                {
                    SIPPStateKey successorKey = successor.GetStateKey();
                    
                    bool shouldUpdate = false;
                    
                    // 2-27: If g(s') > new_cost
                    if (!allStates.TryGetValue(successorKey, out SIPPState existingState) || 
                        successor.GScore < existingState.GScore)
                    {
                        shouldUpdate = true;
                    }
                    
                    if (shouldUpdate)
                    {
                        allStates[successorKey] = successor;
                        
                        // 2-29 - 2-38: If s' ∈ CLOSED, add to INCONS, else update OPEN
                        if (closedSet.Contains(successorKey))
                        {
                            inconsSet.Add(successorKey);
                        }
                        else
                        {
                            openSet.Enqueue(successor, successor.FScore);
                        }
                    }
                }
            }
            
            return goalState;
        }

        /// <summary>
        /// Generates all possible successor states from a given state
        /// This handles both movement to neighboring nodes and waiting actions
        /// </summary>
        private static List<SIPPState> GenerateSuccessors(
            NavigationGraph graph,
            IndoorSpace indoorSpace,
            Dictionary<string, List<SafeInterval>> safeIntervals,
            SIPPState current,
            Waypoint endPosition,
            string robotId,
            float defaultSpeed,
            float maxAcceleration,
            float maxDeceleration,
            SearchMethod searchMethod,
            TrafficRuleKnowledge trafficRules,
            float epsilon,
            DynamicOccupancyLayer globalOccupancyLayer,
            Dictionary<string, VerticalConnectionInfo> verticalConnectionInfo,
            string goalFloor)
        {
            List<SIPPState> successors = new List<SIPPState>();
            string currentNodeId = current.Waypoint.Id;
            string currentFloor = current.FloorLevel;
            
            // 1. Motion Type: Move to neighboring nodes
            // Get all possible physical neighbors
            var neighbors = GetNeighbors(
                current, 
                graph, 
                indoorSpace, 
                robotId,
                current.DepartureTime, 
                defaultSpeed, 
                maxAcceleration,
                maxDeceleration,
                trafficRules,
                verticalConnectionInfo);
            
            // Process each neighbor (direct movement + wait-then-move via A2 analytical computation)
            foreach (var (nextWaypoint, travelTime, travelCost, nextFloor, isVertical, verticalConnectorId) in neighbors)
            {
                // Resolve successor waypoint to its containing CellSpace key for interval lookup
                string nextCellKey = GetCellKeyForWaypoint(nextWaypoint, indoorSpace);

                // Get all safe intervals at the destination (keyed by CellSpace ID)
                List<SafeInterval> intervals = GetSafeIntervals(safeIntervals, nextCellKey);

                if (intervals == null || intervals.Count == 0)
                {
                    Debug.LogWarning($"Robot {robotId}: Cannot find safe interval at {nextCellKey} (Waypoint: {nextWaypoint.Id})");
                    continue;
                }
                
                // Check each interval at the destination to see if we can arrive within it
                for (int i = 0; i < intervals.Count; i++)
                {
                    SafeInterval interval = intervals[i];
                    
                    // A2 fix: For each safe interval, compute the required departure time
                    // to arrive within it (wait-then-move for all reachable intervals)
                    float requiredDepart = Math.Max(current.DepartureTime, interval.Start - travelTime);
                    float analyticalArriveTime = requiredDepart + travelTime;

                    // Verify departure is within current safe interval and arrival within target interval
                    if (requiredDepart >= current.SafeInterval.Start
                        && requiredDepart <= current.SafeInterval.End
                        && analyticalArriveTime >= interval.Start
                        && analyticalArriveTime <= interval.End)
                    {
                        float departureTime = analyticalArriveTime; // No waiting after arrival

                        bool hasWait = requiredDepart > current.DepartureTime;
                        float waitTime = hasWait ? requiredDepart - current.DepartureTime : 0f;

                        // A1 fix: g-score = pure arrival time (wait + travel), no intent penalty
                        float newGScore = current.GScore + waitTime + travelCost;

                        // Apply intent cost penalty as an additive f-score bonus, not in g-score
                        float intentBonus = 0f;
                        if (globalOccupancyLayer != null)
                        {
                            string neighborCellId = nextWaypoint.PrimaryCellId;
                            if (!string.IsNullOrEmpty(neighborCellId) && globalOccupancyLayer.HasOccupancyOfType(
                                neighborCellId, analyticalArriveTime, OccupancyType.Intent, robotId))
                            {
                                intentBonus = travelCost * (INTENT_COST_FACTOR - 1.0f);
                            }
                        }

                        float heuristic = CalculateHeuristic(
                            nextWaypoint,
                            endPosition,
                            searchMethod,
                            defaultSpeed,
                            nextFloor,
                            goalFloor);

                        float newFScore = newGScore + heuristic * epsilon + intentBonus;

                        // Create successor state for movement (with optional wait)
                        SIPPState newState = new SIPPState(
                            nextWaypoint,
                            i,
                            analyticalArriveTime,
                            departureTime,
                            newGScore,
                            newFScore,
                            current,
                            interval,
                            defaultSpeed,
                            nextFloor,
                            isVertical,
                            verticalConnectorId);

                        successors.Add(newState);

                        // Update vertical connection info with actual departure time
                        // (GetNeighbors recorded currentTime, but requiredDepart may be later due to waiting)
                        if (isVertical && !string.IsNullOrEmpty(verticalConnectorId))
                        {
                            string connectionKey = $"{current.Waypoint.Id}_to_{verticalConnectorId}";
                            lock (verticalConnectionInfo)
                            {
                                if (verticalConnectionInfo.TryGetValue(connectionKey, out var info))
                                {
                                    info.StartTime = requiredDepart;
                                }
                            }
                        }
                    }
                }
            }
            
            // 2. Motion Type: Wait at current location until next safe interval
            // Resolve current waypoint to its containing CellSpace key for interval lookup
            string currentCellKey = GetCellKeyForWaypoint(current.Waypoint, indoorSpace);
            // Check if there's a next safe interval at the current location
            List<SafeInterval> currentIntervals = GetSafeIntervals(safeIntervals, currentCellKey);
            if (currentIntervals != null && current.IntervalIndex < currentIntervals.Count - 1)
            {
                // Get the next safe interval
                SafeInterval nextInterval = currentIntervals[current.IntervalIndex + 1];
                
                // Calculate wait time to reach the next interval
                float waitTime = nextInterval.Start - current.DepartureTime;
                
                // Ensure wait time is positive
                if (waitTime > TIME_EPSILON)
                {
                    float newArrivalTime = current.DepartureTime + waitTime;
                    float newDepartureTime = newArrivalTime; // No additional waiting after arrival
                    float newGScore = current.GScore + waitTime;
                    
                    float heuristic = CalculateHeuristic(
                        current.Waypoint,
                        endPosition,
                        searchMethod,
                        defaultSpeed,
                        currentFloor,
                        goalFloor);
                    
                    float newFScore = newGScore + heuristic * epsilon;
                    
                    // Create successor state for waiting until next interval
                    SIPPState waitSuccessor = new SIPPState(
                        current.Waypoint,
                        current.IntervalIndex + 1,
                        newArrivalTime,
                        newDepartureTime,
                        newGScore,
                        newFScore,
                        current,
                        nextInterval,
                        defaultSpeed,
                        currentFloor);
                    
                    successors.Add(waitSuccessor);
                }
            }
            
            // Motion Type 3 (vertical wait-then-move) removed — A2 fix already covers
            // all neighbors (horizontal + vertical) with analytical wait-then-move computation.

            return successors;
        }

        /// <summary>
        /// Get neighbors for a given state, including vertical connections
        /// </summary>
        private static List<(Waypoint waypoint, float time, float cost, string floor, bool isVertical, string verticalConnectorId)> GetNeighbors(
            SIPPState state,
            NavigationGraph graph,
            IndoorSpace indoorSpace,
            string robotId,
            float currentTime,
            float defaultSpeed,
            float maxAcceleration,
            float maxDeceleration,
            TrafficRuleKnowledge trafficRules,
            Dictionary<string, VerticalConnectionInfo> verticalConnectionInfo)
        {
            var neighbors = new List<(Waypoint, float, float, string, bool, string)>();

            string currentFloor = state.FloorLevel;

            var neighborWaypoints = graph.GetNeighbors(state.Waypoint.Id);

            foreach (var neighborWaypoint in neighborWaypoints)
            {
                string childFloor = neighborWaypoint.FloorLevel;

                bool isVertical = !string.IsNullOrEmpty(childFloor)
                                  && !string.IsNullOrEmpty(currentFloor)
                                  && childFloor != currentFloor;
                string verticalConnectorId = null;

                float travelTime, travelCost;
                float speedForMove = defaultSpeed;

                if (isVertical)
                {
                    int floorDifference = Math.Abs(ParseFloorLevel(childFloor) - ParseFloorLevel(currentFloor));
                    float baseTime;
                    if (neighborWaypoint.IsStair)
                    {
                        baseTime = STAIR_FLOOR_TIME * floorDifference;
                    }
                    else
                    {
                        baseTime = ELEVATOR_BASE_TIME + ELEVATOR_FLOOR_TIME * floorDifference;
                    }

                    float distance = CalculateDistance(state.Waypoint, neighborWaypoint);

                    var (adjustedSpeed, adjustedTime) = trafficRules.ModifySpeed(
                        state.Waypoint,
                        neighborWaypoint,
                        state.Speed,
                        defaultSpeed,
                        distance,
                        maxAcceleration,
                        maxDeceleration);

                    travelTime = Math.Max(baseTime, adjustedTime);
                    travelCost = travelTime;
                    speedForMove = adjustedSpeed;

                    // Resolve vertical connector to its CellSpace key (consistent with safe-interval keying)
                    string verticalCellKey = GetCellKeyForWaypoint(neighborWaypoint, indoorSpace);
                    verticalConnectorId = verticalCellKey;

                    string connectionKey = $"{state.Waypoint.Id}_to_{verticalCellKey}";
                    lock (verticalConnectionInfo)
                    {
                        verticalConnectionInfo[connectionKey] = new VerticalConnectionInfo
                        {
                            CellSpaceId = verticalCellKey,
                            FromFloor = currentFloor,
                            ToFloor = childFloor,
                            StartTime = currentTime,
                            TravelTime = travelTime
                        };
                    }
                }
                else
                {
                    float distance = CalculateDistance(state.Waypoint, neighborWaypoint);

                    var (adjustedSpeed, adjustedTime) = trafficRules.ModifySpeed(
                        state.Waypoint,
                        neighborWaypoint,
                        state.Speed,
                        defaultSpeed,
                        distance,
                        maxAcceleration,
                        maxDeceleration);

                    travelTime = adjustedTime;
                    travelCost = travelTime;
                    speedForMove = adjustedSpeed;
                }

                neighbors.Add((neighborWaypoint, travelTime, travelCost, childFloor, isVertical, verticalConnectorId));
            }

            return neighbors;
        }

        /// <summary>
        /// Reconstruct the complete path from the goal state
        /// </summary>
        private static (List<Waypoint> path, List<float> times, List<float> speeds) ReconstructPath(
            SIPPState goalState,
            float defaultSpeed)
        {
            List<SIPPState> statesReversed = new List<SIPPState>();

            SIPPState current = goalState;
            while (current != null)
            {
                statesReversed.Add(current);
                current = current.Parent;
            }

            List<SIPPState> states = new List<SIPPState>(statesReversed);
            states.Reverse();

            List<Waypoint> path = new List<Waypoint>();
            List<float> times = new List<float>();
            List<float> speeds = new List<float>();

            if (states.Count > 0)
            {
                path.Add(states[0].Waypoint);
                times.Add(states[0].ArrivalTime);
                speeds.Add(states[0].Speed);
            }

            for (int i = 1; i < states.Count; i++)
            {
                SIPPState state = states[i];
                SIPPState prevState = states[i-1];

                // If previous state had waiting, add a point at the departure time
                if (prevState.HasWait && prevState.WaitTime > 0.01f)
                {
                    path.Add(prevState.Waypoint);
                    times.Add(prevState.DepartureTime);
                    speeds.Add(0); // Speed is zero when waiting
                }

                path.Add(state.Waypoint);
                times.Add(state.ArrivalTime);
                speeds.Add(state.Speed);
            }

            return (path, times, speeds);
        }
        
        #endregion

        #region Helper

        /// <summary>
        /// Resolve a Waypoint to its containing CellSpace ID.
        /// The occupancy layer stores data by CellSpace ID, so all safe-interval
        /// lookups must use CellSpace ID as key. For CP-backed waypoints, uses
        /// PrimaryCellId. For CellSpace-backed waypoints, uses the waypoint's own Id.
        /// Falls back to waypoint Id if the CellSpace cannot be resolved.
        /// </summary>
        private static string GetCellKeyForWaypoint(Waypoint waypoint, IndoorSpace indoorSpace)
        {
            if (waypoint == null)
                return null;

            // CellSpace-backed waypoints already are the cell
            if (waypoint.Kind == WaypointKind.CellSpace)
                return waypoint.Id;

            // CP-backed waypoints: use PrimaryCellId (the source cell of the boundary)
            if (!string.IsNullOrEmpty(waypoint.PrimaryCellId))
                return waypoint.PrimaryCellId;

            // Fallback: try to resolve via IndoorSpace
            if (indoorSpace != null && waypoint.ConnectionPointSource != null)
            {
                CellSpace cell = indoorSpace.GetCellSpaceFromConnectionPoint(waypoint.ConnectionPointSource, true);
                if (cell != null)
                    return cell.Id;
            }

            return waypoint.Id;
        }

        /// <summary>
        /// Build safe intervals from occupancy data
        /// </summary>
        private static Dictionary<string, List<SafeInterval>> BuildSafeIntervals(
            IndoorSpace indoorSpace,
            DynamicOccupancyLayer occupancyLayer,
            string robotId,
            float startTime,
            float horizonSeconds)
        {
            Dictionary<string, List<SafeInterval>> safeIntervals = new Dictionary<string, List<SafeInterval>>();

            var allConnectionPoints = indoorSpace.GetAllConnectionPoints().ToList();
            float endTime = startTime + horizonSeconds;

            foreach (var connectionPoint in allConnectionPoints)
            {
                // Resolve ConnectionPoint to its containing CellSpace ID.
                // DynamicOccupancyLayer stores data by CellSpace ID, not ConnectionPoint ID.
                // Multiple ConnectionPoints can share the same CellSpace (entry and exit
                // boundaries of one aisle cell), so we skip duplicate builds.
                CellSpace containingCell = indoorSpace.GetCellSpaceFromConnectionPoint(connectionPoint, true);
                string cellKey = containingCell?.Id ?? connectionPoint.Id;

                // Skip if we already built intervals for this CellSpace
                if (safeIntervals.ContainsKey(cellKey))
                    continue;

                List<TimeOccupancy> allOccupancies = occupancyLayer != null ?
                    occupancyLayer.GetOccupancies(cellKey, startTime, endTime) :
                    new List<TimeOccupancy>();

                // Filter out Intent occupancies — they are soft constraints (cost penalty only),
                // not hard blocks that carve safe intervals
                List<TimeOccupancy> occupancies = allOccupancies
                    .Where(o => o.Occupier.Type != OccupancyType.Intent)
                    .ToList();

                if (occupancies.Count > 1)
                {
                    occupancies.Sort((a, b) => a.StartTime.CompareTo(b.StartTime));
                }

                List<SafeInterval> intervals = new List<SafeInterval>();

                if (occupancies.Count == 0)
                {
                    intervals.Add(new SafeInterval(startTime, float.MaxValue));
                }
                else
                {
                    float lastEndTime = startTime;

                    foreach (var occupancy in occupancies)
                    {
                        if (occupancy.Occupier.RobotId == robotId)
                        {
                            continue;
                        }

                        if (occupancy.StartTime > lastEndTime)
                        {
                            intervals.Add(new SafeInterval(lastEndTime, occupancy.StartTime));
                        }

                        lastEndTime = Math.Max(lastEndTime, occupancy.EndTime);
                    }

                    intervals.Add(new SafeInterval(lastEndTime, float.MaxValue));
                }

                safeIntervals[cellKey] = intervals;
            }
            
            return safeIntervals;
        }

        private static float CalculateSafeIntervalHorizon(
            IndoorSpace indoorSpace,
            Waypoint start,
            Waypoint goal,
            float defaultSpeed)
        {
            if (indoorSpace == null || start == null || goal == null)
            {
                return 300f;
            }

            float horizontalDistance = CalculateHeuristicDistance(
                start,
                goal,
                SearchMethod.Euclidean_Distance);

            int startLevel = ParseFloorLevel(start.FloorLevel);
            int goalLevel = ParseFloorLevel(goal.FloorLevel);
            int floorDiff = Math.Abs(goalLevel - startLevel);
            float verticalCost = floorDiff > 0 ? Math.Min(ELEVATOR_BASE_TIME + floorDiff * ELEVATOR_FLOOR_TIME, STAIR_FLOOR_TIME * floorDiff) : 0f;

            float travelTime = horizontalDistance / Mathf.Max(defaultSpeed, 0.1f) + verticalCost;
            float buffer = Mathf.Max(20f, travelTime * 1.5f);
            float horizon = Mathf.Clamp(buffer, 60f, 300f);
            return horizon;
        }

        /// <summary>
        /// Get the safe interval that contains the specified time using binary search
        /// </summary>
        private static SafeInterval GetSafeIntervalContainingTime(
            Dictionary<string, List<SafeInterval>> safeIntervals,
            string locationId,
            float time)
        {
            if (!safeIntervals.TryGetValue(locationId, out var intervals) || intervals.Count == 0)
            {
                return null;
            }
            
            int left = 0;
            int right = intervals.Count - 1;
            
            while (left <= right)
            {
                int mid = left + (right - left) / 2;
                var interval = intervals[mid];
                
                if (interval.Contains(time))
                {
                    return interval;
                }
                
                if (time < interval.Start)
                {
                    right = mid - 1;
                }
                else
                {
                    left = mid + 1;
                }
            }
            
            return null;
        }

        /// <summary>
        /// Get all safe intervals for a specified location
        /// </summary>
        private static List<SafeInterval> GetSafeIntervals(
            Dictionary<string, List<SafeInterval>> safeIntervals,
            string locationId)
        {
            if (!safeIntervals.TryGetValue(locationId, out var intervals))
            {
                return new List<SafeInterval>();
            }
            
            return intervals;
        }

        /// <summary>
        /// Calculate the heuristic value, considering 3D distance
        /// </summary>
        private static float CalculateHeuristic(
            Waypoint current,
            Waypoint goal,
            SearchMethod searchMethod,
            float defaultSpeed,
            string currentFloor,
            string goalFloor)
        {
            float horizontalDistance = 0;

            switch (searchMethod)
            {
                case SearchMethod.Euclidean_Distance:
                    horizontalDistance = Vector3.Distance(
                        current.Position,
                        goal.Position);
                    break;
                case SearchMethod.Manhattan_Distance:
                    horizontalDistance = Math.Abs(current.Position.x - goal.Position.x) + Math.Abs(current.Position.z - goal.Position.z);
                    break;
                default:
                    horizontalDistance = Vector3.Distance(
                        current.Position,
                        goal.Position);
                    break;
            }
            
            float verticalCost = 0;
            if (currentFloor != goalFloor)
            {
                int currentLevel = ParseFloorLevel(currentFloor);
                int goalLevel = ParseFloorLevel(goalFloor);
                int floorDifference = Math.Abs(goalLevel - currentLevel);
                
                verticalCost = Math.Min(
                    ELEVATOR_BASE_TIME + floorDifference * ELEVATOR_FLOOR_TIME,
                    STAIR_FLOOR_TIME * floorDifference);
            }
            
            return horizontalDistance / defaultSpeed + verticalCost;
        }

        /// <summary>
        /// Calculate the heuristic distance between two connection points
        /// </summary>
        private static float CalculateHeuristicDistance(
            Waypoint from,
            Waypoint to,
            SearchMethod searchMethod)
        {
            float horizontalDistance = 0;

            switch (searchMethod)
            {
                case SearchMethod.Euclidean_Distance:
                    horizontalDistance = Vector3.Distance(from.Position, to.Position);
                    break;
                case SearchMethod.Manhattan_Distance:
                    horizontalDistance = Math.Abs(from.Position.x - to.Position.x) + Math.Abs(from.Position.z - to.Position.z);
                    break;
                default:
                    horizontalDistance = Vector3.Distance(from.Position, to.Position);
                    break;
            }
            return horizontalDistance;
        }

        /// <summary>
        /// Calculate distance between two waypoints
        /// </summary>
        private static float CalculateDistance(Waypoint from, Waypoint to)
        {
            return Vector3.Distance(from.Position, to.Position);
        }
        
        /// <summary>
        /// Calculate the length of a path
        /// </summary>
        private static float CalculatePathLength(List<Waypoint> path)
        {
            float length = 0f;
            
            for (int i = 1; i < path.Count; i++)
            {
                length += CalculateDistance(path[i - 1], path[i]);
            }
            
            return length;
        }

        private static int ParseFloorLevel(string floorLevel)
        {
            if (string.IsNullOrEmpty(floorLevel))
                return 0;

            if (floorLevelCache.TryGetValue(floorLevel, out int cachedLevel))
                return cachedLevel;

            int result = Map.FloorLevelUtils.Parse(floorLevel);

            floorLevelCache[floorLevel] = result;
            return result;
        }
        #endregion
    }
}
