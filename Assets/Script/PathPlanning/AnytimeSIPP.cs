using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using UnityEngine;
using Map;
using NetTopologySuite.Geometries;
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

        /// <summary>
        /// SIPP State class, representing a node in the search process
        /// </summary>
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
            
            // 3D related properties
            public string FloorLevel { get; private set; }
            public bool IsVerticalTransition { get; private set; }
            public string VerticalConnectorId { get; private set; }
            
            // waiting related properties
            public bool HasWait { get; private set; }
            public float WaitTime { get; private set; }
            
            public bool IsOptimal { get; private set; }

            public SIPPState(
                RoutePoint routePoint,
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
                this.RoutePoint = routePoint;
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
            
            public string GetStateKey()
            {
                return $"{RoutePoint.ConnectionPoint.Id}_{IntervalIndex}_{FloorLevel}";
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
        private static Dictionary<string, VerticalConnectionInfo> verticalConnectionInfo = new Dictionary<string, VerticalConnectionInfo>();
        private const float ELEVATOR_BASE_TIME = 3.0f;
        private const float ELEVATOR_FLOOR_TIME = 0.5f;
        private const float DEFAULT_VERTICAL_TIME = 6.0f;
        private const float TIME_EPSILON = 0.001f;
        
        #endregion

        #region main methods

        /// <summary>
        /// Anytime SIPP Algorithm Entry Point
        /// </summary>
        public static (List<ConnectionPoint> path, List<float> times, List<float> speeds, string planId, PathPlanningMetrics metrics) SIPPAlgorithm(
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
                Debug.LogError($"Robot {robotId}: SIPP parameters are invalid");
                return (new List<ConnectionPoint>(), new List<float>(), new List<float>(), null, null);
            }

            // Initialize
            TrafficRuleKnowledge trafficRules = new TrafficRuleKnowledge(indoorSpace, graph);

            string newPlanId = $"SIPPPlan_{robotId}_{DateTime.Now.Ticks}";

            float timeLimit = 1.0f;
            float initialEpsilon = 3.0f;
            float finalEpsilon = 1.0f;
            float epsilonStep = 0.5f;

            verticalConnectionInfo.Clear();

            // Build safe intervals according to the occupancy layer for robot
            Dictionary<string, List<SafeInterval>> safeIntervals = BuildSafeIntervals(
                indoorSpace, globalOccupancyLayer, robotId, startTime);

            string startFloor = startPosition.ConnectionPoint.FloorLevel;
            string endFloor = endPosition.ConnectionPoint.FloorLevel;
            Debug.Log($"Robot {robotId}: startPoint {startPosition.ConnectionPoint.Id}: {startPosition.ConnectionPoint.ToJson()}, endPoint {endPosition.ConnectionPoint.Id}: {endPosition.ConnectionPoint.ToJson()}");

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
                metrics);

            // Check if the path is found, if yes, set the occupancy layer
            if (result.success && globalOccupancyLayer != null)
            {
                foreach (var info in verticalConnectionInfo.Values)
                {
                    if (!string.IsNullOrEmpty(info.CellSpaceId))
                    {
                        globalOccupancyLayer.SetDynamicOccupancy(
                            info.CellSpaceId,
                            info.StartTime,
                            info.EndTime,
                            robotId,
                            newPlanId,
                            false,
                            false
                        );
                    }
                }
            }

            return (result.path, result.times, result.speeds, newPlanId, metrics);
        }

        /// <summary>
        /// Run Anytime SIPP Algorithm
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
            DynamicOccupancyLayer globalOccupancyLayer, 
            PathPlanningMetrics metrics = null)
        {
            Stopwatch stopwatch = new Stopwatch();
            stopwatch.Start();

            // Initialize variables
            bool foundSolution = false;
            List<ConnectionPoint> bestPath = null;
            List<float> bestTimes = null;
            List<float> bestSpeeds = null;
            float currentEpsilon = initialEpsilon;
            float bestEpsilon = initialEpsilon;

            string startFloor = startPosition.ConnectionPoint.FloorLevel;
            string endFloor = endPosition.ConnectionPoint.FloorLevel;

            // Calculate the ideal path length
            float idealPathLength = CalculateHeuristicDistance(startPosition, endPosition, searchMethod);
            metrics.IdealPathLength = idealPathLength;

            // 1-3: OPEN = CLOSED = INCONS = ∅
            PriorityQueue<SIPPState> openSet = new PriorityQueue<SIPPState>();
            HashSet<string> closedSet = new HashSet<string>();
            HashSet<string> inconsSet = new HashSet<string>();
            
            Dictionary<string, SIPPState> allStates = new Dictionary<string, SIPPState>();

            // 1-4: insert s_start into OPEN with ε * h(s_start)
            // Get the initial safe interval for the start position
            SafeInterval initialInterval = GetSafeIntervalContainingTime(
                safeIntervals, startPosition.ConnectionPoint.Id, startTime);
                
            if (initialInterval == null)
            {
                Debug.LogError($"Robot {robotId}: Can not find safe interval for start position {startPosition.ConnectionPoint.Id} at time {startTime}");
                return (false, null, null, null, initialEpsilon);
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
            string initialStateKey = initialState.GetStateKey();
            allStates[initialStateKey] = initialState;

            bool finalEpsilonRun = false;
            // 1-8: while ε > 1 do
            while (currentEpsilon > finalEpsilon || (currentEpsilon == finalEpsilon && !finalEpsilonRun))
            {
                Debug.Log($"Robot {robotId}: Attempt to plan for epsilon={currentEpsilon:F1}");

                closedSet.Clear();
                inconsSet.Clear();

                if (foundSolution)
                {
                    openSet = new PriorityQueue<SIPPState>();
                    
                    Dictionary<string, SIPPState> statesCopy = new Dictionary<string, SIPPState>(allStates);
                    
                    // 1-11: Update the priorities for all s ∈ OPEN according to f(s)
                    foreach (var kvp in statesCopy)
                    {
                        SIPPState state = kvp.Value;
                        if (!state.IsOptimal)
                        {
                            float newHeuristic = CalculateHeuristic(
                                state.RoutePoint, endPosition, searchMethod, defaultSpeed, state.FloorLevel, endFloor);
                            
                            float newFScore = state.GScore + newHeuristic * currentEpsilon;
                            
                            SIPPState updatedState = new SIPPState(
                                state.RoutePoint,
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
                                
                            allStates[updatedState.GetStateKey()] = updatedState;
                            openSet.Enqueue(updatedState, updatedState.FScore);
                        }
                    }
                }

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
            Debug.Log($"Robot {robotId}: SIPP success! Time: {stopwatch.ElapsedMilliseconds}ms, epsilon={bestEpsilon:F1}");

            float actualPathLength = CalculatePathLength(bestPath);
            metrics.ActualPathLength = actualPathLength;
            metrics.TravelTime = bestTimes.Last() - bestTimes.First();
            metrics.PlanningTime = stopwatch.ElapsedMilliseconds / 1000f;
            
            if (EventManager.Instance != null)
            {
                EventManager.Instance.ReportPathPlanningMetrics(
                    robotId,
                    metrics.PlanId,
                    metrics.PlanningTime,
                    metrics.ActualPathLength,
                    metrics.TravelTime,
                    metrics.IdealPathLength
                );
            }

            return (foundSolution, bestPath, bestTimes, bestSpeeds, bestEpsilon);
        }

        /// <summary>
        /// ImprovePath function - Improves the path using the specified epsilon value
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
            DynamicOccupancyLayer globalOccupancyLayer,
            string goalFloor)
        {
            SIPPState goalState = null;
            float goalFValue = float.MaxValue;
            
            string goalLocationId = endPosition.ConnectionPoint.Id;
            
            // 2-1: while f(s_goal) > min_{s ∈ OPEN} {f(s)} do
            while (openSet.Count > 0 && (goalState == null || goalFValue > openSet.Peek().FScore))
            {
                // 2-2: remove s with the smallest f(s) from OPEN
                SIPPState current = openSet.Dequeue();
                
                string stateKey = current.GetStateKey();
                
                if (closedSet.Contains(stateKey))
                {
                    continue;
                }
                
                // 2-3: CLOSED = CLOSED ∪ {s}
                closedSet.Add(stateKey);
                
                // Check if we've reached the goal
                if (current.RoutePoint.ConnectionPoint.Id == goalLocationId && 
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
                    goalFloor
                );
                
                // Process each successor
                foreach (var successor in successors)
                {
                    string successorKey = successor.GetStateKey();
                    
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
            Graph graph,
            IndoorSpace indoorSpace,
            Dictionary<string, List<SafeInterval>> safeIntervals,
            SIPPState current,
            RoutePoint endPosition,
            string robotId,
            float defaultSpeed,
            float maxAcceleration,
            float maxDeceleration,
            SearchMethod searchMethod,
            TrafficRuleKnowledge trafficRules,
            float epsilon,
            string goalFloor)
        {
            List<SIPPState> successors = new List<SIPPState>();
            string currentNodeId = current.RoutePoint.ConnectionPoint.Id;
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
                trafficRules);
            
            // Process each neighbor (direct movement)
            foreach (var (nextRoutePoint, travelTime, travelCost, nextFloor, isVertical, verticalConnectorId) in neighbors)
            {
                // Calculate arrival time at the neighbor
                float arriveTime = current.DepartureTime + travelTime;
                
                // Get all safe intervals at the destination
                List<SafeInterval> intervals = GetSafeIntervals(safeIntervals, nextRoutePoint.ConnectionPoint.Id);
                
                if (intervals == null || intervals.Count == 0)
                {
                    Debug.LogWarning($"Robot {robotId}: Cannot find safe interval at {nextRoutePoint.ConnectionPoint.Id}");
                    continue;
                }
                
                // Check each interval at the destination to see if we can arrive within it
                for (int i = 0; i < intervals.Count; i++)
                {
                    SafeInterval interval = intervals[i];
                    
                    // Check if arrival time is within the safe interval
                    if (interval.Contains(arriveTime) || Math.Abs(interval.Start - arriveTime) < TIME_EPSILON)
                    {
                        float departureTime = arriveTime; // No waiting after arrival
                        float newGScore = current.GScore + travelCost;
                        
                        float heuristic = CalculateHeuristic(
                            nextRoutePoint, 
                            endPosition, 
                            searchMethod, 
                            defaultSpeed, 
                            nextFloor, 
                            goalFloor);
                        
                        float newFScore = newGScore + heuristic * epsilon;
                        
                        // Create successor state for direct movement
                        SIPPState newState = new SIPPState(
                            nextRoutePoint,
                            i,
                            arriveTime,
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
                    }
                }
            }
            
            // 2. Motion Type: Wait at current location until next safe interval
            // Check if there's a next safe interval at the current location
            List<SafeInterval> currentIntervals = GetSafeIntervals(safeIntervals, currentNodeId);
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
                        current.RoutePoint, 
                        endPosition, 
                        searchMethod, 
                        defaultSpeed, 
                        currentFloor, 
                        goalFloor);
                    
                    float newFScore = newGScore + heuristic * epsilon;
                    
                    // Create successor state for waiting until next interval
                    SIPPState waitSuccessor = new SIPPState(
                        current.RoutePoint,
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
            
            // 3. Motion Type: Wait and then move (especially for vertical transitions)
            // Calculate maximum wait time possible in current interval
            float maxWaitTimeInCurrentInterval = current.SafeInterval.End - current.DepartureTime;
            
            if (maxWaitTimeInCurrentInterval > TIME_EPSILON)
            {
                // For each neighbor, try waiting different amounts of time to find valid arrival
                foreach (var (nextRoutePoint, baseTravelTime, baseTravelCost, nextFloor, isVertical, verticalConnectorId) in neighbors)
                {
                    // Focus on vertical transitions which might need waiting
                    if (isVertical)
                    {
                        // Try different wait times to find valid transitions
                        float waitStep = Math.Min(0.5f, maxWaitTimeInCurrentInterval / 10);
                        bool foundValidWait = false;
                        
                        for (float waitTime = 0; waitTime <= maxWaitTimeInCurrentInterval && !foundValidWait; waitTime += waitStep)
                        {
                            // Calculate new departure and arrival times
                            float newDepartureTime = current.DepartureTime + waitTime;
                            float arriveTime = newDepartureTime + baseTravelTime;
                            
                            // Get safe intervals at destination
                            List<SafeInterval> intervals = GetSafeIntervals(safeIntervals, nextRoutePoint.ConnectionPoint.Id);
                            
                            if (intervals == null || intervals.Count == 0)
                            {
                                continue;
                            }
                            
                            // Check each interval to see if we can arrive within it after waiting
                            for (int i = 0; i < intervals.Count && !foundValidWait; i++)
                            {
                                SafeInterval interval = intervals[i];
                                
                                if (interval.Contains(arriveTime) || Math.Abs(interval.Start - arriveTime) < TIME_EPSILON)
                                {
                                    float newGScore = current.GScore + waitTime + baseTravelCost;
                                    
                                    float heuristic = CalculateHeuristic(
                                        nextRoutePoint, 
                                        endPosition, 
                                        searchMethod, 
                                        defaultSpeed, 
                                        nextFloor, 
                                        goalFloor);
                                    
                                    float newFScore = newGScore + heuristic * epsilon;
                                    
                                    // Create successor state with waiting and then movement
                                    SIPPState waitThenMoveSuccessor = new SIPPState(
                                        nextRoutePoint,
                                        i,
                                        arriveTime,
                                        arriveTime, // No additional waiting after arrival
                                        newGScore,
                                        newFScore,
                                        current,
                                        interval,
                                        defaultSpeed,
                                        nextFloor,
                                        isVertical,
                                        verticalConnectorId);
                                    
                                    successors.Add(waitThenMoveSuccessor);
                                    foundValidWait = true;
                                }
                            }
                        }
                    }
                }
            }
            
            return successors;
        }

        /// <summary>
        /// Get neighbors for a given state, including vertical connections
        /// </summary>
        private static List<(RoutePoint routePoint, float time, float cost, string floor, bool isVertical, string verticalConnectorId)> GetNeighbors(
            SIPPState state,
            Graph graph,
            IndoorSpace indoorSpace,
            string robotId,
            float currentTime,
            float defaultSpeed,
            float maxAcceleration,
            float maxDeceleration,
            TrafficRuleKnowledge trafficRules)
        {
            var neighbors = new List<(RoutePoint, float, float, string, bool, string)>();
            
            string currentFloor = state.FloorLevel;
            
            var childConnections = new List<ConnectionPoint>(state.RoutePoint.Children);
            
            foreach (var childConnection in childConnections)
            {
                if (childConnection.IsStair) continue;

                RoutePoint childRoutePoint = graph.GetRoutePointFromConnectionPoint(childConnection);
                if (childRoutePoint == null) continue;
                
                string childFloor = childConnection.FloorLevel;
                
                bool isVertical = childFloor != currentFloor;
                string verticalConnectorId = null;
                
                float travelTime, travelCost;
                float speedForMove = defaultSpeed;
                
                if (isVertical)
                {
                    int floorDifference = Math.Abs(ParseFloorLevel(childFloor) - ParseFloorLevel(currentFloor));
                    float baseTime = ELEVATOR_BASE_TIME + ELEVATOR_FLOOR_TIME * floorDifference;
                    
                    float distance = CalculateDistance(state.RoutePoint.ConnectionPoint, childConnection);
                    
                    var (adjustedSpeed, adjustedTime) = trafficRules.ModifySpeed(
                        state.RoutePoint,
                        childRoutePoint,
                        state.Speed,
                        defaultSpeed,
                        distance,
                        maxAcceleration,
                        maxDeceleration);
                    
                    travelTime = Math.Max(baseTime, adjustedTime);
                    travelCost = travelTime;
                    speedForMove = adjustedSpeed;
                    
                    verticalConnectorId = childConnection.Id;
                    
                    string connectionKey = $"{state.RoutePoint.ConnectionPoint.Id}_to_{childConnection.Id}";
                    lock (verticalConnectionInfo)
                    {
                        verticalConnectionInfo[connectionKey] = new VerticalConnectionInfo
                        {
                            CellSpaceId = verticalConnectorId,
                            FromFloor = currentFloor,
                            ToFloor = childFloor,
                            StartTime = currentTime,
                            TravelTime = travelTime
                        };
                    }
                }
                else
                {
                    float distance = CalculateDistance(state.RoutePoint.ConnectionPoint, childConnection);
                    
                    var (adjustedSpeed, adjustedTime) = trafficRules.ModifySpeed(
                        state.RoutePoint,
                        childRoutePoint,
                        state.Speed,
                        defaultSpeed,
                        distance,
                        maxAcceleration,
                        maxDeceleration);
                    
                    travelTime = adjustedTime;
                    travelCost = travelTime;
                    speedForMove = adjustedSpeed;
                }
                
                neighbors.Add((childRoutePoint, travelTime, travelCost, childFloor, isVertical, verticalConnectorId));
            }
            
            return neighbors;
        }

        /// <summary>
        /// Reconstruct the complete path from the goal state
        /// </summary>
        private static (List<ConnectionPoint> path, List<float> times, List<float> speeds) ReconstructPath(
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
            
            List<ConnectionPoint> path = new List<ConnectionPoint>();
            List<float> times = new List<float>();
            List<float> speeds = new List<float>();
            
            if (states.Count > 0)
            {
                path.Add(states[0].RoutePoint.ConnectionPoint);
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
                    path.Add(prevState.RoutePoint.ConnectionPoint);
                    times.Add(prevState.DepartureTime);
                    speeds.Add(0); // Speed is zero when waiting
                }
                
                path.Add(state.RoutePoint.ConnectionPoint);
                times.Add(state.ArrivalTime);
                speeds.Add(state.Speed);
            }
            
            return (path, times, speeds);
        }
        
        #endregion

        #region Helper

        /// <summary>
        /// Build safe intervals from occupancy data
        /// </summary>
        private static Dictionary<string, List<SafeInterval>> BuildSafeIntervals(
            IndoorSpace indoorSpace,
            DynamicOccupancyLayer occupancyLayer,
            string robotId,
            float startTime)
        {
            Dictionary<string, List<SafeInterval>> safeIntervals = new Dictionary<string, List<SafeInterval>>();
            
            var allConnectionPoints = indoorSpace.GetAllConnectionPoints().ToList();
            
            foreach (var connectionPoint in allConnectionPoints)
            {
                string connectionPointId = connectionPoint.Id;
                
                List<TimeOccupancy> occupancies = occupancyLayer != null ? 
                    occupancyLayer.GetOccupancies(connectionPointId, startTime, startTime + 300) :
                    new List<TimeOccupancy>();
                
                occupancies = occupancies.Where(o => o.Occupier.RobotId != robotId).ToList();
                
                List<SafeInterval> intervals = new List<SafeInterval>();
                
                if (occupancies.Count == 0)
                {
                    intervals.Add(new SafeInterval(startTime, float.MaxValue));
                }
                else
                {
                    occupancies = occupancies.OrderBy(o => o.StartTime).ToList();
                    
                    float lastEndTime = startTime;
                    
                    foreach (var occupancy in occupancies)
                    {
                        if (occupancy.StartTime > lastEndTime)
                        {
                            intervals.Add(new SafeInterval(lastEndTime, occupancy.StartTime));
                        }
                        
                        lastEndTime = Math.Max(lastEndTime, occupancy.EndTime);
                    }
                    
                    intervals.Add(new SafeInterval(lastEndTime, float.MaxValue));
                }
                
                safeIntervals[connectionPointId] = intervals;
            }
            
            return safeIntervals;
        }

        /// <summary>
        /// Get the safe interval that contains the specified time
        /// </summary>
        private static SafeInterval GetSafeIntervalContainingTime(
            Dictionary<string, List<SafeInterval>> safeIntervals,
            string locationId,
            float time)
        {
            if (!safeIntervals.TryGetValue(locationId, out var intervals))
            {
                return null;
            }
            
            foreach (var interval in intervals)
            {
                if (interval.Contains(time))
                {
                    return interval;
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
            RoutePoint current,
            RoutePoint goal,
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
                        GetPointVector(current.ConnectionPoint),
                        GetPointVector(goal.ConnectionPoint));
                    break;
                case SearchMethod.Manhattan_Distance:
                    Vector3 currentPos = GetPointVector(current.ConnectionPoint);
                    Vector3 goalPos = GetPointVector(goal.ConnectionPoint);
                    horizontalDistance = Math.Abs(currentPos.x - goalPos.x) + Math.Abs(currentPos.z - goalPos.z);
                    break;
                default:
                    horizontalDistance = Vector3.Distance(
                        GetPointVector(current.ConnectionPoint),
                        GetPointVector(goal.ConnectionPoint));
                    break;
            }
            
            float verticalCost = 0;
            if (currentFloor != goalFloor)
            {
                int currentLevel = ParseFloorLevel(currentFloor);
                int goalLevel = ParseFloorLevel(goalFloor);
                int floorDifference = Math.Abs(goalLevel - currentLevel);
                
                verticalCost = ELEVATOR_BASE_TIME + floorDifference * ELEVATOR_FLOOR_TIME;
            }
            
            return horizontalDistance / defaultSpeed + verticalCost;
        }

        /// <summary>
        /// Calculate the heuristic distance between two connection points
        /// </summary>
        private static float CalculateHeuristicDistance(
            RoutePoint from,
            RoutePoint to,
            SearchMethod searchMethod)
        {
            float horizontalDistance = 0;

            switch (searchMethod)
            {
                case SearchMethod.Euclidean_Distance:
                    horizontalDistance = Vector3.Distance(
                        GetPointVector(from.ConnectionPoint),
                        GetPointVector(to.ConnectionPoint));
                    break;
                case SearchMethod.Manhattan_Distance:
                    Vector3 fromPos = GetPointVector(from.ConnectionPoint);
                    Vector3 toPos = GetPointVector(to.ConnectionPoint);
                    horizontalDistance = Math.Abs(fromPos.x - toPos.x) + Math.Abs(fromPos.z - toPos.z);
                    break;
                default:
                    horizontalDistance = Vector3.Distance(
                        GetPointVector(from.ConnectionPoint),
                        GetPointVector(to.ConnectionPoint));
                    break;
            }
            return horizontalDistance;
        }

        /// <summary>
        /// Calculate distance between two connection points
        /// </summary>
        private static float CalculateDistance(ConnectionPoint from, ConnectionPoint to)
        {
            Vector3 fromPos = GetPointVector(from);
            Vector3 toPos = GetPointVector(to);

            return Vector3.Distance(fromPos, toPos);
        }
        
        /// <summary>
        /// Get the Vector3 representation of a connection point
        /// </summary>
        private static Vector3 GetPointVector(ConnectionPoint connectionPoint)
        {
            if (connectionPoint.Point is Point point)
            {
                return new Vector3((float)point.X, 0, (float)point.Y);
            }

            return Vector3.zero;
        }
        
        /// <summary>
        /// Calculate the length of a path
        /// </summary>
        private static float CalculatePathLength(List<ConnectionPoint> path)
        {
            float length = 0f;
            
            for (int i = 1; i < path.Count; i++)
            {
                length += CalculateDistance(path[i - 1], path[i]);
            }
            
            return length;
        }

        /// <summary>
        /// Parse floor level string to integer
        /// </summary>
        private static int ParseFloorLevel(string floorLevel)
        {
            if (string.IsNullOrEmpty(floorLevel))
                return 0;

            if (floorLevel.StartsWith("B") || floorLevel.StartsWith("b"))
            {
                if (int.TryParse(floorLevel.Substring(1).Replace("F", "").Replace("f", ""), out int level))
                {
                    return -level;
                }
            }
            else
            {
                if (int.TryParse(floorLevel.Replace("F", "").Replace("f", ""), out int level))
                {
                    return level;
                }
            }

            return 0;
        }
        #endregion
    }
}