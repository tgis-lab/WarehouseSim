using System;
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
    /// Time-Space A* Algorithm for cross-floor path planning
    /// </summary>
    public class TimeSpaceAStar
    {
        #region private fields
        
        private readonly struct TSAStarStateKey : IEquatable<TSAStarStateKey>
        {
            public readonly string ConnectionPointId;
            public readonly int ArrivalTimeKey;
            public readonly string FloorLevel;

            public TSAStarStateKey(string connectionPointId, int arrivalTimeKey, string floorLevel)
            {
                ConnectionPointId = connectionPointId;
                ArrivalTimeKey = arrivalTimeKey;
                FloorLevel = floorLevel;
            }

            public bool Equals(TSAStarStateKey other) =>
                ConnectionPointId == other.ConnectionPointId &&
                ArrivalTimeKey == other.ArrivalTimeKey &&
                FloorLevel == other.FloorLevel;

            public override bool Equals(object obj) => obj is TSAStarStateKey other && Equals(other);

            public override int GetHashCode()
            {
                unchecked
                {
                    int hash = ConnectionPointId?.GetHashCode() ?? 0;
                    hash = (hash * 397) ^ ArrivalTimeKey;
                    hash = (hash * 397) ^ (FloorLevel?.GetHashCode() ?? 0);
                    return hash;
                }
            }
        }

        /// <summary>
        /// State class for Time-Space A* search process
        /// </summary>
        private class TSAStarState : IComparable<TSAStarState>
        {
            public RoutePoint RoutePoint { get; private set; }
            public float ArrivalTime { get; private set; }
            public float GScore { get; private set; }
            public float FScore { get; private set; }
            public TSAStarState Parent { get; private set; }
            public float Speed { get; private set; }
            
            // Cross-floor properties
            public string FloorLevel { get; private set; }
            public bool IsVerticalTransition { get; private set; }
            public string VerticalConnectorId { get; private set; }

            public TSAStarState(
                RoutePoint routePoint,
                float arrivalTime,
                float gScore,
                float fScore,
                TSAStarState parent,
                float speed,
                string floorLevel,
                bool isVerticalTransition = false,
                string verticalConnectorId = null)
            {
                RoutePoint = routePoint;
                ArrivalTime = arrivalTime;
                GScore = gScore;
                FScore = fScore;
                Parent = parent;
                Speed = speed;
                FloorLevel = floorLevel;
                IsVerticalTransition = isVerticalTransition;
                VerticalConnectorId = verticalConnectorId;
            }

            public int CompareTo(TSAStarState other)
            {
                int fScoreComparison = FScore.CompareTo(other.FScore);
                if (fScoreComparison != 0)
                {
                    return fScoreComparison;
                }
                
                // if f-score is equal, prefer the one with larger g-value
                return -GScore.CompareTo(other.GScore);
            }
            
            public TSAStarStateKey GetStateKey()
            {
                int arrivalTimeKey = Mathf.RoundToInt(ArrivalTime * 10f);
                return new TSAStarStateKey(RoutePoint.ConnectionPoint.Id, arrivalTimeKey, FloorLevel);
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

        #region constants
        
        private const float ELEVATOR_BASE_TIME = 3.0f;
        private const float ELEVATOR_FLOOR_TIME = 0.5f;
        private const float STAIR_FLOOR_TIME = 2.0f;
        private const float DEFAULT_VERTICAL_TIME = 2.5f;
        private const float ELEVATOR_COST_FACTOR = 1.1f;
        private const float STAIR_COST_FACTOR = 1.2f;
        private const float DEFAULT_VERTICAL_COST_FACTOR = 1.3f;
        private const float TIME_RESOLUTION = 0.1f;  // Round time to nearest 0.1 second
        private const float TIME_EPSILON = 0.001f;
        private const float OCCUPANCY_BUFFER = 0.5f; // Buffer time for occupancy checks
        private const float INTENT_COST_FACTOR = 1.3f;

        #endregion
        
        #region main methods
        
        /// <summary>
        /// Cross-floor Time-Space A* Algorithm Entry Point
        /// </summary>
        public static (List<ConnectionPoint> path, List<float> times, List<float> speeds, string planId, PathPlanningMetrics metrics) TimeSpaceAStarAlgorithm(
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
                Debug.LogError($"Robot {robotId}: TimeSpaceAStar parameters are invalid");
                return (new List<ConnectionPoint>(), new List<float>(), new List<float>(), null, null);
            }

            // Initialize
            TrafficRuleKnowledge trafficRules = new TrafficRuleKnowledge(indoorSpace, graph);
            string newPlanId = $"TimeSpaceAStarPlan_{robotId}_{DateTime.Now.Ticks}";

            var verticalConnectionInfo = new Dictionary<string, VerticalConnectionInfo>();
            
            string startFloor = startPosition.ConnectionPoint.FloorLevel;
            string endFloor = endPosition.ConnectionPoint.FloorLevel;
            
            Debug.Log($"Robot {robotId}: Starting TimeSpaceAStar path planning from Floor {startFloor} to Floor {endFloor}");
            PathPlanningMetrics metrics = new PathPlanningMetrics(robotId, newPlanId ?? "unknown");

            // Run Time-Space A* search
            var result = RunTimeSpaceAStar(
                graph,
                indoorSpace,
                personalOccupancyLayer,
                globalOccupancyLayer,
                startPosition,
                endPosition,
                robotId,
                startTime,
                defaultSpeed,
                maxAcceleration,
                maxDeceleration,
                searchMethod,
                trafficRules,
                verticalConnectionInfo,
                metrics);
            
            // Check if the path is found, if yes, set the occupancy layer for vertical connectors
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
        /// Run Time-Space A* Algorithm
        /// </summary>
        private static (bool success, List<ConnectionPoint> path, List<float> times, List<float> speeds) RunTimeSpaceAStar(
            Graph graph,
            IndoorSpace indoorSpace,
            DynamicOccupancyLayer personalOccupancyLayer,
            DynamicOccupancyLayer globalOccupancyLayer,
            RoutePoint startPosition,
            RoutePoint endPosition,
            string robotId,
            float startTime,
            float defaultSpeed,
            float maxAcceleration,
            float maxDeceleration,
            SearchMethod searchMethod,
            TrafficRuleKnowledge trafficRules,
            Dictionary<string, VerticalConnectionInfo> verticalConnectionInfo,
            PathPlanningMetrics metrics = null)
        {
            Stopwatch stopwatch = new Stopwatch();
            stopwatch.Start();
            
            PriorityQueue<TSAStarState> openSet = new PriorityQueue<TSAStarState>();
            HashSet<TSAStarStateKey> closedSet = new HashSet<TSAStarStateKey>();
            Dictionary<TSAStarStateKey, TSAStarState> allStates = new Dictionary<TSAStarStateKey, TSAStarState>();
            
            string startFloor = startPosition.ConnectionPoint.FloorLevel;
            string endFloor = endPosition.ConnectionPoint.FloorLevel;

            float idealPathLength = CalculateHeuristicDistance(startPosition, endPosition, searchMethod);
            metrics.IdealPathLength = idealPathLength;
            
            // Calculate initial heuristic
            float initialHeuristic = CalculateHeuristic(
                startPosition, endPosition, searchMethod, defaultSpeed, startFloor, endFloor);
            
            // Create initial state
            TSAStarState initialState = new TSAStarState(
                startPosition,
                startTime,
                0,
                initialHeuristic,
                null,
                defaultSpeed,
                startFloor);
            
            // Add initial state to open set
            openSet.Enqueue(initialState, initialState.FScore);
            TSAStarStateKey initialStateKey = initialState.GetStateKey();
            allStates[initialStateKey] = initialState;
            
            TSAStarState goalState = null;
            
            // Main search loop
            while (openSet.Count > 0)
            {
                // Get the state with lowest f-score
                TSAStarState current = openSet.Dequeue();
                TSAStarStateKey currentStateKey = current.GetStateKey();
                
                // If we reached the goal
                if (current.RoutePoint.ConnectionPoint.Id == endPosition.ConnectionPoint.Id &&
                    current.FloorLevel == endFloor)
                {
                    goalState = current;
                    break;
                }
                
                // Skip if already processed
                if (closedSet.Contains(currentStateKey))
                {
                    continue;
                }
                
                // Add to closed set
                closedSet.Add(currentStateKey);
                
                // Generate successors
                var successors = GenerateSuccessors(
                    graph,
                    indoorSpace,
                    personalOccupancyLayer,
                    globalOccupancyLayer,
                    current,
                    endPosition,
                    robotId,
                    defaultSpeed,
                    maxAcceleration,
                    maxDeceleration,
                    searchMethod,
                    trafficRules,
                    verticalConnectionInfo,
                    endFloor
                );
                
                // Process each successor
                foreach (var successor in successors)
                {
                    TSAStarStateKey successorKey = successor.GetStateKey();
                    
                    if (closedSet.Contains(successorKey))
                    {
                        continue;
                    }
                    
                    bool shouldUpdate = false;
                    
                    // Check if this is a better path
                    if (!allStates.TryGetValue(successorKey, out TSAStarState existingState) ||
                        successor.GScore < existingState.GScore)
                    {
                        shouldUpdate = true;
                    }
                    
                    if (shouldUpdate)
                    {
                        allStates[successorKey] = successor;
                        
                        // Update or add to open set
                        if (openSet.Contains(successor))
                        {
                            openSet.UpdatePriority(successor, successor.FScore);
                        }
                        else
                        {
                            openSet.Enqueue(successor, successor.FScore);
                        }
                    }
                }
            }
            
            stopwatch.Stop();
            
            // Check if we found a path
            if (goalState == null)
            {
                Debug.LogWarning($"Robot {robotId}: TimeSpaceAStar failed to find a path. Time: {stopwatch.ElapsedMilliseconds}ms");
                return (false, new List<ConnectionPoint>(), new List<float>(), new List<float>());
            }
            
            // Reconstruct the path
            var pathResult = ReconstructPath(goalState, defaultSpeed);

            float actualPathLength = CalculatePathLength(pathResult.path);
            metrics.ActualPathLength = actualPathLength;
            metrics.TravelTime = pathResult.times.Last() - pathResult.times.First();
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
            
            Debug.Log($"Robot {robotId}: TimeSpaceAStar found a path with {pathResult.path.Count} points. Time: {stopwatch.ElapsedMilliseconds}ms");
            Debug.Log($"Robot {robotId}: Path: {string.Join(" -> ", pathResult.path.Select(p => p.Id))}");
            
            return (true, pathResult.path, pathResult.times, pathResult.speeds);
        }
        
        /// <summary>
        /// Generate successor states from the current state
        /// </summary>
        private static List<TSAStarState> GenerateSuccessors(
            Graph graph,
            IndoorSpace indoorSpace,
            DynamicOccupancyLayer personalOccupancyLayer,
            DynamicOccupancyLayer globalOccupancyLayer,
            TSAStarState current,
            RoutePoint endPosition,
            string robotId,
            float defaultSpeed,
            float maxAcceleration,
            float maxDeceleration,
            SearchMethod searchMethod,
            TrafficRuleKnowledge trafficRules,
            Dictionary<string, VerticalConnectionInfo> verticalConnectionInfo,
            string goalFloor)
        {
            List<TSAStarState> successors = new List<TSAStarState>();
            string currentNodeId = current.RoutePoint.ConnectionPoint.Id;
            string currentFloor = current.FloorLevel;
            
            var childConnections = new List<ConnectionPoint>(current.RoutePoint.Children);
            
            // Test for time-space occupancy at current node and time
            if (!IsNodeAvailableAtTime(globalOccupancyLayer, personalOccupancyLayer, currentNodeId, current.ArrivalTime, robotId))
            {
                // Current node is not available at this time, so no valid successors
                return successors;
            }
            
            // Process each neighbor
            foreach (var childConnection in childConnections)
            {
                RoutePoint childRoutePoint = graph.GetRoutePointFromConnectionPoint(childConnection);
                if (childRoutePoint == null) continue;
                
                string childFloor = childConnection.FloorLevel;
                bool isVertical = childFloor != currentFloor;
                string verticalConnectorId = null;
                
                float travelTime, travelCost;
                float speedForMove = defaultSpeed;
                
                // Calculate travel time and cost
                if (isVertical)
                {
                    // Handle vertical transition (elevator, stairs)
                    int floorDifference = Math.Abs(ParseFloorLevel(childFloor) - ParseFloorLevel(currentFloor));
                    float baseTime = ELEVATOR_BASE_TIME + ELEVATOR_FLOOR_TIME * floorDifference;
                    
                    float distance = CalculateDistance(current.RoutePoint.ConnectionPoint, childConnection);
                    
                    var (adjustedSpeed, adjustedTime) = trafficRules.ModifySpeed(
                        current.RoutePoint,
                        childRoutePoint,
                        current.Speed,
                        defaultSpeed,
                        distance,
                        maxAcceleration,
                        maxDeceleration);
                    
                    travelTime = Math.Max(baseTime, adjustedTime);
                    travelCost = travelTime * ELEVATOR_COST_FACTOR;
                    speedForMove = adjustedSpeed;
                    
                    verticalConnectorId = childConnection.Id;
                    
                    // For vertical transitions, record the usage
                    CellSpace cellSpace = indoorSpace.GetCellSpaceFromConnectionPoint(childConnection);
                    if (cellSpace != null)
                    {
                        string connectionKey = $"{current.RoutePoint.ConnectionPoint.Id}_to_{childConnection.Id}";
                        verticalConnectionInfo[connectionKey] = new VerticalConnectionInfo
                        {
                            CellSpaceId = cellSpace.Id,
                            FromFloor = currentFloor,
                            ToFloor = childFloor,
                            StartTime = current.ArrivalTime,
                            TravelTime = travelTime
                        };
                    }
                }
                else
                {
                    // Handle regular horizontal movement
                    float distance = CalculateDistance(current.RoutePoint.ConnectionPoint, childConnection);
                    
                    var (adjustedSpeed, adjustedTime) = trafficRules.ModifySpeed(
                        current.RoutePoint,
                        childRoutePoint,
                        current.Speed,
                        defaultSpeed,
                        distance,
                        maxAcceleration,
                        maxDeceleration);
                    
                    travelTime = adjustedTime;
                    travelCost = travelTime;
                    speedForMove = adjustedSpeed;
                }
                
                // Round the arrival time to the defined resolution
                float arrivalTime = RoundTime(current.ArrivalTime + travelTime);
                
                // Check time-space availability at target node and arrival time
                if (!IsNodeAvailableAtTime(globalOccupancyLayer, personalOccupancyLayer, childConnection.Id, arrivalTime, robotId))
                {
                    float nextAvailableTime = GetNextAvailableTime(
                        globalOccupancyLayer, personalOccupancyLayer, childConnection.Id, arrivalTime, robotId, 3.0f);
                    
                    if (nextAvailableTime < 0)
                    {
                        continue;
                    }
                    
                    float delay = nextAvailableTime - arrivalTime;
                    arrivalTime = RoundTime(nextAvailableTime);
                    travelTime += delay;
                    travelCost += delay;
                }
                
                // Apply intent cost penalty if another robot has Intent on the target cell
                float intentPenalty = 1.0f;
                if (globalOccupancyLayer != null)
                {
                    CellSpace neighborCell = indoorSpace.GetCellSpaceFromConnectionPoint(childConnection, true);
                    if (neighborCell != null && globalOccupancyLayer.HasOccupancyOfType(
                        neighborCell.Id, arrivalTime, OccupancyType.Intent, robotId))
                    {
                        intentPenalty = INTENT_COST_FACTOR;
                    }
                }

                // Calculate path costs
                float newGScore = current.GScore + travelCost * intentPenalty;
                
                // Calculate heuristic
                float heuristic = CalculateHeuristic(
                    childRoutePoint,
                    endPosition,
                    searchMethod,
                    speedForMove,
                    childFloor,
                    goalFloor);
                
                float newFScore = newGScore + heuristic;
                
                // Create successor state
                TSAStarState successor = new TSAStarState(
                    childRoutePoint,
                    arrivalTime,
                    newGScore,
                    newFScore,
                    current,
                    speedForMove,
                    childFloor,
                    isVertical,
                    verticalConnectorId);
                
                successors.Add(successor);
            }
            
            // Add waiting option
            if (successors.Count == 0)
            {
                // Try to wait at current position
                float waitTime = 1.0f;
                float waitArrivalTime = RoundTime(current.ArrivalTime + waitTime);
                
                if (IsNodeAvailableAtTime(globalOccupancyLayer, personalOccupancyLayer, currentNodeId, waitArrivalTime, robotId))
                {
                    float waitGScore = current.GScore + waitTime;
                    
                    float waitHeuristic = CalculateHeuristic(
                        current.RoutePoint,
                        endPosition,
                        searchMethod,
                        current.Speed,
                        currentFloor,
                        goalFloor);
                    
                    float waitFScore = waitGScore + waitHeuristic;
                    
                    TSAStarState waitSuccessor = new TSAStarState(
                        current.RoutePoint,
                        waitArrivalTime,
                        waitGScore,
                        waitFScore,
                        current,
                        current.Speed,
                        currentFloor);
                    
                    successors.Add(waitSuccessor);
                }
            }
            
            return successors;
        }
        
        /// <summary>
        /// Check if a node is available at a specific time
        /// </summary>
        private static bool IsNodeAvailableAtTime(
            DynamicOccupancyLayer globalOccupancyLayer,
            DynamicOccupancyLayer personalOccupancyLayer,
            string nodeId,
            float time,
            string robotId)
        {
            // Check if the node is occupied by any other robot at the specified time
            // Intent occupancies are soft constraints (cost penalty only), not hard blocks
            if (globalOccupancyLayer != null)
            {
                var occupancies = globalOccupancyLayer.GetOccupancies(
                    nodeId,
                    time - OCCUPANCY_BUFFER,
                    time + OCCUPANCY_BUFFER);

                if (occupancies.Any(o => o.Occupier.RobotId != robotId
                    && o.Occupier.Type != OccupancyType.Intent))
                {
                    return false;
                }
            }
            
            // Check the personal occupancy layer (e.g., robot's own planned path)
            if (personalOccupancyLayer != null)
            {
                var personalOccupancies = personalOccupancyLayer.GetOccupancies(
                    nodeId,
                    time - OCCUPANCY_BUFFER,
                    time + OCCUPANCY_BUFFER);
                
                if (personalOccupancies.Any())
                {
                    return false;
                }
            }
            
            return true;
        }
        
        private static float GetNextAvailableTime(
            DynamicOccupancyLayer globalOccupancyLayer,
            DynamicOccupancyLayer personalOccupancyLayer,
            string nodeId,
            float startTime,
            string robotId,
            float maxLookahead)
        {
            float endTime = startTime + maxLookahead;
            float latestOccupancyEnd = startTime;
            
            if (globalOccupancyLayer != null)
            {
                var occupancies = globalOccupancyLayer.GetOccupancies(nodeId, startTime, endTime);
                foreach (var occ in occupancies)
                {
                    // Intent occupancies are soft constraints, skip them for hard-block timing
                    if (occ.Occupier.RobotId != robotId
                        && occ.Occupier.Type != OccupancyType.Intent
                        && occ.EndTime > latestOccupancyEnd)
                    {
                        latestOccupancyEnd = occ.EndTime;
                    }
                }
            }
            
            if (personalOccupancyLayer != null)
            {
                var personalOccupancies = personalOccupancyLayer.GetOccupancies(nodeId, startTime, endTime);
                foreach (var occ in personalOccupancies)
                {
                    if (occ.EndTime > latestOccupancyEnd)
                    {
                        latestOccupancyEnd = occ.EndTime;
                    }
                }
            }
            
            float candidateTime = latestOccupancyEnd + OCCUPANCY_BUFFER;
            
            if (candidateTime > endTime)
            {
                return -1f;
            }
            
            if (IsNodeAvailableAtTime(globalOccupancyLayer, personalOccupancyLayer, nodeId, candidateTime, robotId))
            {
                return candidateTime;
            }
            
            return -1f;
        }
        
        private static (List<ConnectionPoint> path, List<float> times, List<float> speeds) ReconstructPath(
            TSAStarState goalState,
            float defaultSpeed)
        {
            List<TSAStarState> statesReversed = new List<TSAStarState>();
            
            TSAStarState current = goalState;
            while (current != null)
            {
                statesReversed.Add(current);
                current = current.Parent;
            }
            
            List<TSAStarState> states = new List<TSAStarState>(statesReversed);
            states.Reverse();
            
            List<ConnectionPoint> path = new List<ConnectionPoint>();
            List<float> times = new List<float>();
            List<float> speeds = new List<float>();
            
            for (int i = 0; i < states.Count; i++)
            {
                TSAStarState state = states[i];
                
                path.Add(state.RoutePoint.ConnectionPoint);
                times.Add(state.ArrivalTime);
                speeds.Add(state.Speed);
                
                // If this is not the last state, check if we need to add a waiting state
                if (i < states.Count - 1)
                {
                    TSAStarState nextState = states[i + 1];
                    
                    // If we're on the same node but at a different time, this indicates waiting
                    if (state.RoutePoint.ConnectionPoint.Id == nextState.RoutePoint.ConnectionPoint.Id &&
                        Math.Abs(nextState.ArrivalTime - state.ArrivalTime) > TIME_EPSILON)
                    {
                        // Add a waiting point with zero speed
                        path.Add(state.RoutePoint.ConnectionPoint);
                        times.Add(nextState.ArrivalTime - TIME_EPSILON);
                        speeds.Add(0);
                    }
                }
            }
            
            return (path, times, speeds);
        }
        
        #endregion
        
        #region helper methods
        
        /// <summary>
        /// Round time to the nearest time resolution unit
        /// </summary>
        private static float RoundTime(float time)
        {
            return (float)Math.Round(time / TIME_RESOLUTION) * TIME_RESOLUTION;
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
        
        private static Vector3 GetPointVector(ConnectionPoint connectionPoint)
        {
            return connectionPoint.Position;
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
