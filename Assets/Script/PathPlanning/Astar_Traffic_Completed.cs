using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Map;
using NetTopologySuite.Geometries;
using System.Diagnostics;
using CentralControl;

namespace PathPlanning
{
    /// <summary>
    /// Smooth Path Point information
    /// </summary>
    public class SmoothPathPoint
    {
        public ConnectionPoint ConnectionPoint { get; set; }
        public Vector3 Position { get; set; }
        public float Time { get; set; }
        public float Speed { get; set; }
        public bool IsAuxiliary { get; set; }
    }

    /// <summary>
    /// Robot Smooth Path information
    /// </summary>
    public class RobotSmoothPathInfo
    {
        public long Timestamp { get; set; }
        public List<ConnectionPoint> ConnectionPoints { get; set; }
        public List<Vector3> Positions { get; set; }
        public List<float> Times { get; set; }
        public List<float> Speeds { get; set; }
        public List<bool> IsAuxiliary { get; set; }
    }
    
    /// <summary>
    /// Path Segment information
    /// </summary>
    public class PathSegment
    {
        public ConnectionPoint From { get; set; }
        public ConnectionPoint To { get; set; }
        public float StartTime { get; set; }  // Departure time from From
        public float EndTime { get; set; }    // Arrival time at To
        public float Speed { get; set; }      // Speed during the segment
        
        public PathSegment(ConnectionPoint from, ConnectionPoint to, float startTime, float endTime, float speed)
        {
            From = from;
            To = to;
            StartTime = startTime;
            EndTime = endTime;
            Speed = speed;
        }
    }
    
    /// <summary>
    /// Time-Space Path information
    /// </summary>
    public class TimeSpacePath
    {
        public List<PathSegment> Segments { get; private set; }
        public List<ConnectionPoint> Points { get; private set; }
        
        public TimeSpacePath()
        {
            Segments = new List<PathSegment>();
            Points = new List<ConnectionPoint>();
        }
        
        public void AddSegment(PathSegment segment)
        {
            Segments.Add(segment);
            
            // Maintain the list of points
            if (Points.Count == 0 || Points[Points.Count-1].Id != segment.From.Id)
            {
                Points.Add(segment.From);
            }
            Points.Add(segment.To);
        }
        
        public void AddTerminalPoint(ConnectionPoint point, float arrivalTime)
        {
            // For terminal points, we only add the point if it's not already present
            if (Points.Count == 0 || Points[Points.Count-1].Id != point.Id)
            {
                Points.Add(point);
            }
        }
    }
    
    /// <summary>
    /// A* Search Result information
    /// </summary>
    public class AStarSearchResult
    {
        public bool success;
        public Dictionary<RoutePoint, Tuple<RoutePoint, float, float>> previous;
    }
    
    public class Astar_Traffic
    {
        private static CellSpaceConflictDetector conflictDetector = new CellSpaceConflictDetector();
        private static Dictionary<string, List<RobotSmoothPathInfo>> robotSmoothPaths = new Dictionary<string, List<RobotSmoothPathInfo>>();
        
        // Time delay and retry attempts for smart avoidance
        private const int MAX_RETRY_ATTEMPTS = 3;
        private const float TIME_DELAY_INCREMENT = 0.5f;
        
        public static (List<ConnectionPoint> path, List<float> times, List<float> speeds, string planId) AstarAlgorithm(
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
                UnityEngine.Debug.LogError($"Robot {robotId}: Invalid parameters for path planning");
                return (new List<ConnectionPoint>(), new List<float>(), new List<float>(), null);
            }
            
            // Create a new traffic rule knowledge base
            TrafficRuleKnowledge trafficRules = new TrafficRuleKnowledge(indoorSpace, graph);
            
            // Create a new plan ID
            string newPlanId = $"Plan_{robotId}_{DateTime.Now.Ticks}";
            
            // Smart Avoidance Strategy: Try multiple path planning attempts
            for (int attempt = 0; attempt < MAX_RETRY_ATTEMPTS; attempt++)
            {
                // Calculate the current start time
                float currentStartTime = startTime + attempt * TIME_DELAY_INCREMENT;
                
                if (attempt > 0)
                {
                    UnityEngine.Debug.Log($"Robot {robotId}: Path planning retry #{attempt}, dalay: {TIME_DELAY_INCREMENT * attempt:F2} seconds.");
                }
                
                // A* Search
                var searchResult = AstarSearch(
                    graph, 
                    indoorSpace, 
                    globalOccupancyLayer, 
                    startPosition, 
                    endPosition, 
                    robotId, 
                    trafficRules,
                    currentStartTime, 
                    defaultSpeed, 
                    maxAcceleration, 
                    maxDeceleration, 
                    searchMethod);
                
                // Check if the search was successful
                if (searchResult.success)
                {
                    // Rebuild the time-space path
                    var timeSpacePath = RebuildTimeSpacePath(
                        endPosition, 
                        searchResult.previous, 
                        currentStartTime, 
                        defaultSpeed
                    );
                    
                    // Generate smooth path
                    var smoothPath = GenerateSmoothPath(timeSpacePath);
                    
                    // Save the smooth path for the robot
                    SaveSmoothPathForRobot(robotId, smoothPath);
                    
                    var (path, times, speeds) = ConvertTimeSpacePathToLists(timeSpacePath);
                    
                    return (path, times, speeds, newPlanId);
                }
            }
            
            // All attempts failed
            UnityEngine.Debug.LogError($"Robot {robotId}: path planning failed, attempting to give up after {MAX_RETRY_ATTEMPTS} attempts.");
            return (new List<ConnectionPoint>(), new List<float>(), new List<float>(), null);
        }
        
        /// <summary>
        /// A* Search algorithm implementation
        /// </summary>
        private static AStarSearchResult AstarSearch(
            Graph graph, 
            IndoorSpace indoorSpace, 
            DynamicOccupancyLayer globalOccupancyLayer, 
            RoutePoint startPosition, 
            RoutePoint endPosition, 
            string robotId,
            TrafficRuleKnowledge trafficRules,
            float startTime, 
            float defaultSpeed, 
            float maxAcceleration, 
            float maxDeceleration, 
            SearchMethod searchMethod)
        {
            var stopwatch = new Stopwatch();
            stopwatch.Start();
            
            var result = new AStarSearchResult
            {
                success = false,
                previous = new Dictionary<RoutePoint, Tuple<RoutePoint, float, float>>()
            };
            
            var openSet = new PriorityQueue<RoutePoint>();
            var closedSet = new HashSet<RoutePoint>();
            
            var g_score = new Dictionary<RoutePoint, float>();
            g_score[startPosition] = 0;
            
            var previous = new Dictionary<RoutePoint, Tuple<RoutePoint, float, float>>();
            
            float h_score = CalculateHeuristic(startPosition, endPosition, searchMethod, defaultSpeed);
            
            float f_score = g_score[startPosition] + h_score;
            
            openSet.Enqueue(startPosition, f_score);
            
            var current_speed = new Dictionary<RoutePoint, float>();
            current_speed[startPosition] = defaultSpeed;
            
            var arrival_time = new Dictionary<RoutePoint, float>();
            arrival_time[startPosition] = startTime;
            
            while (openSet.Count > 0)
            {
                RoutePoint current;
                float currentPriority;
                
                if (!openSet.TryDequeue(out current, out currentPriority))
                {
                    break;
                }
                
                if (current == endPosition)
                {
                    stopwatch.Stop();
                    UnityEngine.Debug.Log($"A* Algorithm Execution Time: {stopwatch.ElapsedMilliseconds}ms");
                    
                    result.success = true;
                    result.previous = previous;
                    
                    return result;
                }
                
                closedSet.Add(current);
                
                foreach (var neighborConnectionPoint in current.Children)
                {
                    var neighbor = graph.GetRoutePointFormConnectionPoint(neighborConnectionPoint);
                    
                    if (neighbor == null || closedSet.Contains(neighbor))
                    {
                        continue;
                    }
                    
                    float distance = CalculateDistance(current, neighbor);
                    
                    float currentSpeed = current_speed[current];
                    float currentTime = arrival_time[current];
                    
                    RoutePoint prevPoint = null;
                    if (previous.TryGetValue(current, out var prevInfo))
                    {
                        prevPoint = prevInfo.Item1;
                    }
                    
                    var (newSpeed, travelTime) = trafficRules.ModifySpeed(
                        current,
                        neighbor,
                        currentSpeed,
                        defaultSpeed,
                        distance,
                        maxAcceleration,
                        maxDeceleration,
                        prevPoint);
                    
                    float tentative_g_score = g_score[current] + travelTime;
                    float arrivalTime = currentTime + travelTime;
                    
                    bool hasConflict = CheckCellSpaceConflict(
                        indoorSpace, 
                        globalOccupancyLayer, 
                        conflictDetector, 
                        current, 
                        neighbor, 
                        arrivalTime, 
                        arrivalTime + CalculateTime(current, neighbor, defaultSpeed), 
                        robotId);
                    
                    if (hasConflict)
                    {
                        tentative_g_score += 5.0f;
                    }
                    
                    if (!g_score.ContainsKey(neighbor) || tentative_g_score < g_score[neighbor])
                    {
                        previous[neighbor] = new Tuple<RoutePoint, float, float>(current, newSpeed, arrivalTime);
                        g_score[neighbor] = tentative_g_score;
                        current_speed[neighbor] = newSpeed;
                        arrival_time[neighbor] = arrivalTime;
                        
                        float new_h_score = CalculateHeuristic(neighbor, endPosition, searchMethod, newSpeed);
                        float new_f_score = g_score[neighbor] + new_h_score;
                        
                        if (openSet.Contains(neighbor))
                        {
                            openSet.UpdatePriority(neighbor, new_f_score);
                        }
                        else
                        {
                            openSet.Enqueue(neighbor, new_f_score);
                        }
                    }
                }
            }
            
            stopwatch.Stop();
            UnityEngine.Debug.Log($"A* search failed: {stopwatch.ElapsedMilliseconds}ms");
            return result;
        }
        
        /// <summary>
        /// Rebuild the time-space path from the end position to the start position
        /// </summary>
        private static TimeSpacePath RebuildTimeSpacePath(
            RoutePoint endPosition, 
            Dictionary<RoutePoint, Tuple<RoutePoint, float, float>> previous,
            float startTime,
            float defaultSpeed)
        {
            var timeSpacePath = new TimeSpacePath();
            
            List<RoutePoint> reversedPath = new List<RoutePoint>();
            RoutePoint currentPoint = endPosition;
            
            while (currentPoint != null)
            {
                reversedPath.Add(currentPoint);
                
                if (!previous.TryGetValue(currentPoint, out var previousInfo))
                {
                    break;
                }
                
                currentPoint = previousInfo.Item1;
            }
            
            reversedPath.Reverse();
            
            if (reversedPath.Count <= 1)
            {
                if (reversedPath.Count == 1)
                {
                    timeSpacePath.AddTerminalPoint(reversedPath[0].ConnectionPoint, startTime);
                }
                return timeSpacePath;
            }
            
            float currentTime = startTime;
            
            for (int i = 0; i < reversedPath.Count - 1; i++)
            {
                RoutePoint fromPoint = reversedPath[i];
                RoutePoint toPoint = reversedPath[i + 1];
                
                float segmentSpeed;
                float arrivalTime;
                
                // Trace back the path to find the previous point and its speed
                if (previous.TryGetValue(toPoint, out var info) && info.Item1 == fromPoint)
                {
                    segmentSpeed = info.Item2;
                    arrivalTime = info.Item3;
                }
                else
                {
                    // Backup: if no record is found, use default speed and calculate distance
                    float distance = CalculateDistance(fromPoint, toPoint);
                    segmentSpeed = defaultSpeed;
                    arrivalTime = currentTime + (distance / segmentSpeed);
                    UnityEngine.Debug.LogWarning($"Cannot find the info from {fromPoint.ConnectionPoint.Id} to {toPoint.ConnectionPoint.Id}, using estimated speed and time.");
                }
                
                var segment = new PathSegment(
                    fromPoint.ConnectionPoint,
                    toPoint.ConnectionPoint,
                    currentTime,
                    arrivalTime,
                    segmentSpeed
                );
                
                timeSpacePath.AddSegment(segment);
                
                // Update the current time for the next segment
                currentTime = arrivalTime;
            }
            
            return timeSpacePath;
        }
        
        /// <summary>
        /// Convert the time-space path to lists of points, times, and speeds
        /// </summary>
        private static (List<ConnectionPoint> path, List<float> times, List<float> speeds) ConvertTimeSpacePathToLists(TimeSpacePath timeSpacePath)
        {
            var path = new List<ConnectionPoint>();
            var times = new List<float>();
            var speeds = new List<float>();
            
            // Convert the path points to a list
            foreach (var point in timeSpacePath.Points)
            {
                path.Add(point);
            }
            
            // Convert the time-space path segments to lists of times and speeds
            if (timeSpacePath.Segments.Count > 0)
            {
                // Departure time from the first segment
                times.Add(timeSpacePath.Segments[0].StartTime);
                
                // Make sure the index of time and speed lists match the path points
                speeds.Add(timeSpacePath.Segments[0].Speed);
                
                // Add the times and speeds for each segment
                for (int i = 0; i < timeSpacePath.Segments.Count; i++)
                {
                    times.Add(timeSpacePath.Segments[i].EndTime);
                    speeds.Add(timeSpacePath.Segments[i].Speed);
                }
            }
            else if (path.Count > 0)
            {
                // Deal with the case where there are no segments but we have points
                times.Add(Time.time);
                speeds.Add(0); // Staying still
            }
            
            return (path, times, speeds);
        }
        
        /// <summary>
        /// Check for conflicts in the occupancy layer
        /// </summary>
        private static bool CheckCellSpaceConflict(
            IndoorSpace indoorSpace,
            DynamicOccupancyLayer occupancyLayer,
            CellSpaceConflictDetector conflictDetector,
            RoutePoint fromPoint,
            RoutePoint toPoint,
            float startTime,
            float endTime,
            string robotId)
        {
            // Get the corresponding ConnectionPoints
            ConnectionPoint fromCP = fromPoint.ConnectionPoint;
            ConnectionPoint toCP = toPoint.ConnectionPoint;
            
            if (fromCP == null || toCP == null)
            {
                return false; // Not connection points, can't check
            }
            
            // Create a mini-path to check just this segment
            List<ConnectionPoint> miniPath = new List<ConnectionPoint> { fromCP, toCP };
            List<float> miniTimes = new List<float> { startTime, endTime };
            
            // Use the conflict detector to check for conflicts
            var conflicts = conflictDetector.DetectConflicts(
                occupancyLayer,
                indoorSpace,
                miniPath,
                miniTimes,
                robotId, 
                "PathPlanning");
            
            return conflicts.Count > 0;
        }

        /// <summary>
        /// Calculate the time required to travel between two route points
        /// </summary>
        private static float CalculateTime(RoutePoint p1, RoutePoint p2, float speed)
        {
            if (p1 == null || p2 == null || 
                p1.ConnectionPoint == null || p2.ConnectionPoint == null ||
                p1.ConnectionPoint.Point == null || p2.ConnectionPoint.Point == null)
            {
                return 1000;
            }
            
            Point point1 = (Point)p1.ConnectionPoint.Point;
            Point point2 = (Point)p2.ConnectionPoint.Point;
            
            // Manhattan distance
            float distance = (float)(Math.Abs(point1.X - point2.X) + Math.Abs(point1.Y - point2.Y));
            
            return distance / speed;
        }
        
        /// <summary>
        /// Calculate the distance between two route points
        /// </summary>
        private static float CalculateDistance(RoutePoint p1, RoutePoint p2)
        {
            if (p1 == null || p2 == null || 
                p1.ConnectionPoint == null || p2.ConnectionPoint == null ||
                p1.ConnectionPoint.Point == null || p2.ConnectionPoint.Point == null)
            {
                return 1000;
            }
            
            Point point1 = (Point)p1.ConnectionPoint.Point;
            Point point2 = (Point)p2.ConnectionPoint.Point;
            
            return (float)(Math.Abs(point1.X - point2.X) + Math.Abs(point1.Y - point2.Y));
        }
        
        /// <summary>
        /// Calculate the heuristic value for A* search
        /// </summary>
        private static float CalculateHeuristic(RoutePoint current, RoutePoint goal, SearchMethod method, float speed)
        {
            if (current == null || goal == null || 
                current.ConnectionPoint == null || goal.ConnectionPoint == null ||
                current.ConnectionPoint.Point == null || goal.ConnectionPoint.Point == null)
            {
                return 1000;
            }
            
            Point currentPoint = (Point)current.ConnectionPoint.Point;
            Point goalPoint = (Point)goal.ConnectionPoint.Point;
            
            float distance = 0;
            
            switch (method)
            {
                case SearchMethod.Euclidean_Distance:
                    distance = (float)Math.Sqrt(
                        Math.Pow(currentPoint.X - goalPoint.X, 2) + 
                        Math.Pow(currentPoint.Y - goalPoint.Y, 2));
                    break;
                    
                case SearchMethod.Manhattan_Distance:
                    distance = (float)(
                        Math.Abs(currentPoint.X - goalPoint.X) + 
                        Math.Abs(currentPoint.Y - goalPoint.Y));
                    break;
                    
                default:
                    distance = (float)Math.Sqrt(
                        Math.Pow(currentPoint.X - goalPoint.X, 2) + 
                        Math.Pow(currentPoint.Y - goalPoint.Y, 2));
                    break;
            }
            
            return distance / speed;
        }
        
        /// <summary>
        /// Generate a smooth path from the time-space path
        /// </summary>
        private static List<SmoothPathPoint> GenerateSmoothPath(TimeSpacePath timeSpacePath)
        {
            List<SmoothPathPoint> smoothPath = new List<SmoothPathPoint>();
            
            if (timeSpacePath == null || timeSpacePath.Points.Count == 0)
            {
                return smoothPath;
            }
            
            // Short-circuit for very short paths
            if (timeSpacePath.Points.Count < 3)
            {
                for (int i = 0; i < timeSpacePath.Points.Count; i++)
                {
                    var point = timeSpacePath.Points[i];
                    Point geoPoint = (Point)point.Point;
                    Vector3 position = new Vector3((float)geoPoint.X, 0, (float)geoPoint.Y);
                    
                    float time = 0, speed = 0;
                    
                    if (i == 0 && timeSpacePath.Segments.Count > 0)
                    {
                        time = timeSpacePath.Segments[0].StartTime;
                        speed = timeSpacePath.Segments[0].Speed;
                    }
                    else if (i > 0 && i <= timeSpacePath.Segments.Count)
                    {
                        time = timeSpacePath.Segments[i-1].EndTime;
                        speed = timeSpacePath.Segments[i-1].Speed;
                    }
                    
                    smoothPath.Add(new SmoothPathPoint 
                    { 
                        ConnectionPoint = point,
                        Position = position,
                        Time = time,
                        Speed = speed,
                        IsAuxiliary = false
                    });
                }
                
                return smoothPath;
            }
            
            // Add the first point
            {
                var firstPoint = timeSpacePath.Points[0];
                Point geoPoint = (Point)firstPoint.Point;
                Vector3 position = new Vector3((float)geoPoint.X, 0, (float)geoPoint.Y);
                
                smoothPath.Add(new SmoothPathPoint
                {
                    ConnectionPoint = firstPoint,
                    Position = position,
                    Time = timeSpacePath.Segments[0].StartTime,
                    Speed = timeSpacePath.Segments[0].Speed,
                    IsAuxiliary = false
                });
            }
            
            // Iterate through the path points to find corners and add auxiliary points
            for (int i = 1; i < timeSpacePath.Points.Count - 1; i++)
            {
                var prevPoint = timeSpacePath.Points[i-1];
                var currentPoint = timeSpacePath.Points[i];
                var nextPoint = timeSpacePath.Points[i+1];
                
                Point prevGeoPoint = (Point)prevPoint.Point;
                Point currentGeoPoint = (Point)currentPoint.Point;
                Point nextGeoPoint = (Point)nextPoint.Point;
                
                Vector3 prev = new Vector3((float)prevGeoPoint.X, 0, (float)prevGeoPoint.Y);
                Vector3 current = new Vector3((float)currentGeoPoint.X, 0, (float)currentGeoPoint.Y);
                Vector3 next = new Vector3((float)nextGeoPoint.X, 0, (float)nextGeoPoint.Y);
                
                // Calculate the direction vectors
                Vector3 inDirection = (current - prev).normalized;
                Vector3 outDirection = (next - current).normalized;
                
                // Calculate the angle between the two direction vectors
                float turnAngle = Vector3.Angle(inDirection, outDirection);
                
                // Find the corresponding segments
                PathSegment inSegment = null;
                PathSegment outSegment = null;
                
                for (int j = 0; j < timeSpacePath.Segments.Count; j++)
                {
                    var segment = timeSpacePath.Segments[j];
                    if (segment.To.Id == currentPoint.Id)
                    {
                        inSegment = segment;
                    }
                    if (segment.From.Id == currentPoint.Id)
                    {
                        outSegment = segment;
                    }
                }
                
                // If the angle is small, just add the current point
                if (turnAngle < 20)
                {
                    smoothPath.Add(new SmoothPathPoint
                    {
                        ConnectionPoint = currentPoint,
                        Position = current,
                        Time = inSegment?.EndTime ?? 0,
                        Speed = outSegment?.Speed ?? 0,
                        IsAuxiliary = false
                    });
                    continue;
                }
                
                // Add auxiliary points for the corner
                float cornerDistance = 0.3f; // distance to the corner point
                
                // Calculate the position of auxiliary points
                Vector3 backPoint = current - inDirection * cornerDistance;
                Vector3 forwardPoint = current + outDirection * cornerDistance;
                
                // Calculate the time and speed for the auxiliary points
                float inTime = inSegment?.EndTime ?? 0;
                float outTime = outSegment?.StartTime ?? inTime;
                
                float inSpeed = inSegment?.Speed ?? 0;
                float outSpeed = outSegment?.Speed ?? inSpeed;
                
                // Add the backward auxiliary point (before the corner)
                smoothPath.Add(new SmoothPathPoint
                {
                    ConnectionPoint = currentPoint,
                    Position = backPoint,
                    Time = inTime - 0.1f, // Earlier than the corner time
                    Speed = inSpeed * 0.8f, // slightly slower speed
                    IsAuxiliary = true
                });
                
                // Add the actual corner point
                smoothPath.Add(new SmoothPathPoint
                {
                    ConnectionPoint = currentPoint,
                    Position = current,
                    Time = inTime,
                    Speed = Math.Min(inSpeed, outSpeed) * 0.7f, // slightly slower speed
                    IsAuxiliary = false
                });
                
                // Add the forward auxiliary point (after the corner)
                smoothPath.Add(new SmoothPathPoint
                {
                    ConnectionPoint = currentPoint,
                    Position = forwardPoint,
                    Time = outTime + 0.1f, // Later than the corner time
                    Speed = outSpeed * 0.9f, // slightly faster speed
                    IsAuxiliary = true
                });
            }
            
            // Add the last point
            {
                var lastPoint = timeSpacePath.Points[timeSpacePath.Points.Count-1];
                Point geoPoint = (Point)lastPoint.Point;
                Vector3 position = new Vector3((float)geoPoint.X, 0, (float)geoPoint.Y);
                
                float time = 0, speed = 0;
                
                if (timeSpacePath.Segments.Count > 0)
                {
                    var lastSegment = timeSpacePath.Segments[timeSpacePath.Segments.Count-1];
                    time = lastSegment.EndTime;
                    speed = lastSegment.Speed;
                }
                
                smoothPath.Add(new SmoothPathPoint
                {
                    ConnectionPoint = lastPoint,
                    Position = position,
                    Time = time,
                    Speed = speed,
                    IsAuxiliary = false
                });
            }
            
            return smoothPath;
        }
        
        /// <summary>
        /// Save the smooth path history for a robot
        /// </summary>
        private static void SaveSmoothPathForRobot(string robotId, List<SmoothPathPoint> smoothPath)
        {
            if (smoothPath == null || smoothPath.Count == 0)
                return;
                
            var pathInfo = new RobotSmoothPathInfo
            {
                Timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
                ConnectionPoints = new List<ConnectionPoint>(),
                Positions = new List<Vector3>(),
                Times = new List<float>(),
                Speeds = new List<float>(),
                IsAuxiliary = new List<bool>()
            };
            
            foreach (var point in smoothPath)
            {
                pathInfo.ConnectionPoints.Add(point.ConnectionPoint);
                pathInfo.Positions.Add(point.Position);
                pathInfo.Times.Add(point.Time);
                pathInfo.Speeds.Add(point.Speed);
                pathInfo.IsAuxiliary.Add(point.IsAuxiliary);
            }
            
            // Save the path info to the dictionary
            if (!robotSmoothPaths.ContainsKey(robotId))
            {
                robotSmoothPaths[robotId] = new List<RobotSmoothPathInfo>();
            }
            
            // Only keep the last 3 paths
            var paths = robotSmoothPaths[robotId];
            paths.Add(pathInfo);
            if (paths.Count > 3)
            {
                paths.RemoveAt(0);
            }
        }
        
        /// <summary>
        /// Get the history smooth path for a robot
        /// </summary>
        public static RobotSmoothPathInfo GetRobotLastSmoothPath(string robotId)
        {
            if (robotSmoothPaths.TryGetValue(robotId, out var paths) && paths.Count > 0)
            {
                return paths[paths.Count - 1];
            }
            return null;
        }
    }
}