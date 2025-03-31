using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Map;
using System.Diagnostics;
using NetTopologySuite.Geometries;
using PointLocation = Map.Graph.PointLocation;

namespace PathPlanning
{
    public class SmoothPathPoint
    {
        public ConnectionPoint ConnectionPoint;
        public Vector3 Position;
        public float Time;
        public float Speed;
        public bool IsAuxiliary;
        
        public SmoothPathPoint(ConnectionPoint point, float time, float speed, bool isAuxiliary = false)
        {
            ConnectionPoint = point;
            Position = new Vector3((float)((Point)point.Point).X, 0, (float)((Point)point.Point).Y);
            Time = time;
            Speed = speed;
            IsAuxiliary = isAuxiliary;
        }
        
        public SmoothPathPoint(ConnectionPoint relatedPoint, Vector3 position, float time, float speed)
        {
            ConnectionPoint = relatedPoint;
            Position = position;
            Time = time;
            Speed = speed;
            IsAuxiliary = true;
        }
    }

    public class RobotSmoothPathInfo
    {
        public long Timestamp { get; set; }
        public List<ConnectionPoint> ConnectionPoints { get; set; }
        public List<Vector3> Positions { get; set; }
        public List<float> Times { get; set; }
        public List<float> Speeds { get; set; }
        public List<bool> IsAuxiliary { get; set; }
    }
    
    public class Astar_Traffic
    {
        private static Dictionary<string, List<RobotSmoothPathInfo>> robotSmoothPaths = new Dictionary<string, List<RobotSmoothPathInfo>>();
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
            // Input validation
            if (graph == null)
            {
                UnityEngine.Debug.LogError("Graph is null");
                return (new List<ConnectionPoint>(), new List<float>(), new List<float>(), null);
            }

            if (startPosition == null || endPosition == null)
            {
                UnityEngine.Debug.LogError("Start position or end position is null");
                return (new List<ConnectionPoint>(), new List<float>(), new List<float>(), null);
            }

            if (startPosition.ConnectionPoint == null || endPosition.ConnectionPoint == null)
            {
                UnityEngine.Debug.LogError("Start or end ConnectionPoint is null");
                return (new List<ConnectionPoint>(), new List<float>(), new List<float>(), null);
            }

            UnityEngine.Debug.Log($"Starting A* algorithm from {startPosition.ConnectionPoint.Id} to {endPosition.ConnectionPoint.Id}");

            if (!string.IsNullOrEmpty(currentPlanId))
            {
                ClearUnusedReservations(personalOccupancyLayer, robotId, currentPlanId, startTime);
            }

            string newPlanId;
            if (string.IsNullOrEmpty(currentPlanId))
            {
                // If it's a new plan, generate a new ID with timestamp
                newPlanId = $"Robot_{robotId}_Plan_{DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()}";
            }
            else
            {
                // If it's a replan, append timestamp to the current plan ID
                string timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds().ToString();
                newPlanId = $"{currentPlanId}_Replan_{timestamp}";
            }
            
            var stopwatch = new Stopwatch();
            stopwatch.Start();
            float lastSpeed = defaultSpeed;
            Dictionary<RoutePoint, float> gScore = new Dictionary<RoutePoint, float>();
            Dictionary<RoutePoint, float> fScore = new Dictionary<RoutePoint, float>();
            Dictionary<RoutePoint, Tuple<RoutePoint, float, float>> previous = new Dictionary<RoutePoint, Tuple<RoutePoint, float, float>>();
            Dictionary<RoutePoint, float> timeAtNode = new Dictionary<RoutePoint, float>();
            var openSet = new PriorityQueue<RoutePoint, float>();
            
            foreach (var node in graph.RoutePoints)
            {
                gScore[node] = float.PositiveInfinity;
                fScore[node] = float.PositiveInfinity;
                timeAtNode[node] = startTime;
            }

            gScore[startPosition] = 0;
            fScore[startPosition] = HeuristicCostEstimate(personalOccupancyLayer, startPosition, endPosition, defaultSpeed, searchMethod);
            openSet.Enqueue(startPosition, fScore[startPosition]);
            timeAtNode[startPosition] = startTime;

            while (openSet.TryDequeue(out RoutePoint current, out float currentPriority))
            {
                if (current == null || current.ConnectionPoint == null)
                {
                    UnityEngine.Debug.LogError("Encountered null RoutePoint or ConnectionPoint");
                    continue;
                }

                if (current == endPosition)
                {
                    stopwatch.Stop();
                    UnityEngine.Debug.Log($"A* Algorithm Execution Time: {stopwatch.ElapsedMilliseconds}ms");
                    var rawPath = new List<ConnectionPoint>();
                    var rawTimes = new List<float>();
                    var rawSpeeds = new List<float>();
                    
                    // Reconstruct path
                    RoutePoint currentRoutePoint = endPosition;
                    while (currentRoutePoint != null)
                    {
                        if (currentRoutePoint.ConnectionPoint == null)
                        {
                            UnityEngine.Debug.LogError($"Encountered RoutePoint with null ConnectionPoint while reconstructing path");
                            break;
                        }

                        rawPath.Add(currentRoutePoint.ConnectionPoint);
                        
                        if (!previous.TryGetValue(currentRoutePoint, out var previousInfo))
                        {
                            break;
                        }

                        // 使用ModifySpeed计算的速度和时间
                        rawSpeeds.Add(previousInfo.Item2);
                        rawTimes.Add(previousInfo.Item3);
                        currentRoutePoint = previousInfo.Item1;
                    }

                    // 反转得到正向路径
                    rawPath.Reverse();
                    rawTimes.Reverse();
                    rawSpeeds.Reverse();

                    // 修复时间列表 - 添加起点到达时间到开头
                    if (rawPath.Count > 0 && rawTimes.Count == rawPath.Count - 1)
                    {
                        UnityEngine.Debug.Log("Add arrival time at start point...");
                        rawTimes.Insert(0, startTime);  // 起点的到达时间是startTime
                    }

                    // 修复速度列表 - 添加终点速度到末尾
                    if (rawPath.Count > 0 && rawSpeeds.Count == rawPath.Count - 1)
                    {
                        UnityEngine.Debug.Log("Add speed arriving end point...");
                        
                        if (rawSpeeds.Count > 0)
                        {
                            // 使用前一个点的速度作为终点速度（或其他合适的值）
                            rawSpeeds.Add(lastSpeed);
                        }
                        else
                        {
                            // 如果速度列表为空，使用默认速度
                            rawSpeeds.Add(defaultSpeed);
                        }
                    }
                    
                    // 如果路径无效，返回空结果
                    if (rawPath.Count < 2)
                    {
                        return (new List<ConnectionPoint>(), new List<float>(), new List<float>(), newPlanId);
                    }
                    
                    // 生成平滑路径
                    var smoothPath = GenerateSmoothPath(rawPath, rawTimes, rawSpeeds, maxAcceleration, maxDeceleration);
                    
                    // 从平滑路径中提取原始点路径（用于返回）
                    var finalPath = new List<ConnectionPoint>();
                    var finalTimes = new List<float>();
                    var finalSpeeds = new List<float>();
                    
                    foreach (var point in smoothPath)
                    {
                        // 只添加非辅助点
                        if (!point.IsAuxiliary)
                        {
                            finalPath.Add(point.ConnectionPoint);
                            finalTimes.Add(point.Time);
                            finalSpeeds.Add(point.Speed);
                        }
                    }
                    
                    // 保存完整平滑路径到RobotController
                    SaveSmoothPathForRobot(robotId, smoothPath);
                    
                    ReservePathOccupancy(graph, indoorSpace, personalOccupancyLayer, finalPath, finalTimes, finalSpeeds, robotId, newPlanId);

                    // 返回原始点路径（不含辅助点）
                    return (finalPath, finalTimes, finalSpeeds, newPlanId);
                }

                foreach (var neighborConnection in current.Children)
                {
                    RoutePoint neighbor = graph.GetRoutePointFormConnectionPoint(neighborConnection);
                    if (neighbor == null) continue;
                    
                    Layer layerOfNeighbor = graph.GetLayerFromConnectionPoint(neighborConnection, false);
                    float currentSpeed = ModifySpeed(graph, current.ConnectionPoint, neighborConnection, defaultSpeed, lastSpeed, 
                                                    layerOfNeighbor, Time.deltaTime, maxAcceleration, maxDeceleration);
                    float distance = (float)current.ConnectionPoint.Point.Distance(neighborConnection.Point);
                    float timeToReachNeighbor = distance / currentSpeed;
                    float arrivalTimeAtNeighbor = timeAtNode[current] + timeToReachNeighbor;

                    CellSpace neighborCellSpace = indoorSpace.GetCellSpaceFromConnectionPoint(neighborConnection);
                    if (neighborCellSpace != null)
                    {
                        var personalOccupancies = personalOccupancyLayer.GetOccupancies(
                            neighborCellSpace.Id, 
                            arrivalTimeAtNeighbor, 
                            arrivalTimeAtNeighbor + timeToReachNeighbor
                        );
                        
                        var globalOccupancies = globalOccupancyLayer.GetOccupancies(
                            neighborCellSpace.Id,
                            arrivalTimeAtNeighbor,
                            arrivalTimeAtNeighbor + timeToReachNeighbor
                        );
                        bool isOccupied = personalOccupancies.Any() || globalOccupancies.Any();

                        float occupancyPenalty = isOccupied ? float.PositiveInfinity : 0f;

                        float tentativeGScore = gScore[current] + (distance / currentSpeed) + occupancyPenalty;

                        if (tentativeGScore < gScore[neighbor])
                        {
                            previous[neighbor] = new Tuple<RoutePoint, float, float>(current, currentSpeed, arrivalTimeAtNeighbor);
                            gScore[neighbor] = tentativeGScore;
                            timeAtNode[neighbor] = arrivalTimeAtNeighbor;
                            float f = gScore[neighbor] + HeuristicCostEstimate(personalOccupancyLayer, neighbor, endPosition, currentSpeed, searchMethod);
                            fScore[neighbor] = f;
                            openSet.Enqueue(neighbor, f);
                            lastSpeed = currentSpeed;
                        }
                    }
                }
            }

            UnityEngine.Debug.LogWarning($"No path found from {startPosition.ConnectionPoint.Id} to {endPosition.ConnectionPoint.Id}");
            return (new List<ConnectionPoint>(), new List<float>(), new List<float>(), null);
        }

        private static (List<ConnectionPoint> path, List<float> times, List<float> speeds) ReconstructPath(
            Dictionary<RoutePoint, Tuple<RoutePoint, float, float>> cameFrom, 
            RoutePoint current, 
            float initialSpeed)
        {
            var path = new List<ConnectionPoint>();
            var times = new List<float>();
            var speeds = new List<float>();
            float speed = initialSpeed;

            while (current != null)
            {
                if (current.ConnectionPoint == null)
                {
                    UnityEngine.Debug.LogError($"Encountered RoutePoint with null ConnectionPoint while reconstructing path");
                    break;
                }

                path.Add(current.ConnectionPoint);
                speeds.Add(speed);

                if (!cameFrom.TryGetValue(current, out var previousInfo))
                {
                    break;
                }

                times.Add(previousInfo.Item3);
                speed = previousInfo.Item2;
                current = previousInfo.Item1;
            }

            path.Reverse();
            times.Reverse();
            speeds.Reverse();

            if (times.Count < path.Count)
            {
                float lastTime = times.Count > 0 ? times[times.Count - 1] : 0;
                while (times.Count < path.Count)
                {
                    lastTime += 0.1f;
                    times.Add(lastTime);
                }
            }

            return (path, times, speeds);
        }

        private static List<SmoothPathPoint> GenerateSmoothPath(
            List<ConnectionPoint> rawPath, 
            List<float> rawTimes, 
            List<float> rawSpeeds,
            float maxAcceleration,
            float maxDeceleration)
        {
            var smoothPath = new List<SmoothPathPoint>();

            // 检查输入列表是否有效
            if (rawPath == null || rawPath.Count == 0 || 
                rawTimes == null || rawTimes.Count == 0 || 
                rawSpeeds == null || rawSpeeds.Count == 0)
            {
                UnityEngine.Debug.LogError("GenerateSmoothPath: 输入列表为空或无效");
                return smoothPath;
            }

            // 确保列表长度一致
            if (rawPath.Count != rawTimes.Count || rawPath.Count != rawSpeeds.Count)
            {
                UnityEngine.Debug.LogError($"GenerateSmoothPath: 输入列表长度不匹配 - 路径: {rawPath.Count}, 时间: {rawTimes.Count}, 速度: {rawSpeeds.Count}");
                UnityEngine.Debug.LogError("路径: " + string.Join(", ", rawPath.Select(p => p.Id)));
                UnityEngine.Debug.LogError("时间: " + string.Join(", ", rawTimes));
                UnityEngine.Debug.LogError("速度: " + string.Join(", ", rawSpeeds));
                return smoothPath;
            }
            
            // 添加第一个点（不变）
            smoothPath.Add(new SmoothPathPoint(rawPath[0], rawTimes[0], rawSpeeds[0]));
            
            // 处理中间点
            for (int i = 1; i < rawPath.Count - 1; i++)
            {
                // 获取前、当前、后点的位置
                Vector3 prevPos = new Vector3(
                    (float)((Point)rawPath[i-1].Point).X, 
                    0, 
                    (float)((Point)rawPath[i-1].Point).Y);
                    
                Vector3 currentPos = new Vector3(
                    (float)((Point)rawPath[i].Point).X, 
                    0, 
                    (float)((Point)rawPath[i].Point).Y);
                    
                Vector3 nextPos = new Vector3(
                    (float)((Point)rawPath[i+1].Point).X, 
                    0, 
                    (float)((Point)rawPath[i+1].Point).Y);
                
                // 计算转弯角度
                Vector3 prevDir = (currentPos - prevPos).normalized;
                Vector3 nextDir = (nextPos - currentPos).normalized;
                float turnAngle = Vector3.Angle(prevDir, nextDir);
                
                // 跟踪当前点的ModifySpeed速度
                float currentNodeSpeed = rawSpeeds[i-1]; // 注意，rawSpeeds索引与时间对应
                
                // 如果是急转弯，添加辅助点
                if (turnAngle > 30f)
                {
                    // 计算转弯半径 - 基于速度
                    float safeRadius = Mathf.Max(0.8f, currentNodeSpeed * 0.5f);
                    
                    // 计算可用距离
                    float prevDist = Vector3.Distance(prevPos, currentPos);
                    float nextDist = Vector3.Distance(currentPos, nextPos);
                    
                    // 确保转弯半径不超过路径长度的限制
                    float maxRadius = Mathf.Min(prevDist * 0.4f, nextDist * 0.4f);
                    float actualRadius = Mathf.Min(safeRadius, maxRadius);
                    
                    // 计算贝塞尔曲线控制点
                    Vector3 backPoint = currentPos - prevDir * actualRadius;
                    Vector3 forwardPoint = currentPos + nextDir * actualRadius;
                    
                    // 创建4个辅助点（2个在转弯前，2个在转弯后）
                    AddBezierAuxiliaryPoints(
                        smoothPath,
                        prevPos, currentPos, nextPos,
                        backPoint, forwardPoint,
                        rawPath[i-1], rawPath[i], rawPath[i+1],
                        rawTimes[i-1], rawTimes[i], 
                        rawSpeeds[i-1], rawSpeeds[i],
                        turnAngle);
                }
                
                // 添加原始点
                smoothPath.Add(new SmoothPathPoint(rawPath[i], rawTimes[i], rawSpeeds[i]));
            }
            
            // 添加最后一个点
            if (rawPath.Count > 1)
            {
                int lastIndex = rawPath.Count - 1;
                smoothPath.Add(new SmoothPathPoint(rawPath[lastIndex], rawTimes[lastIndex], rawSpeeds[lastIndex]));
            }
            
            return smoothPath;
        }

        // 添加贝塞尔曲线辅助点
        private static void AddBezierAuxiliaryPoints(
            List<SmoothPathPoint> smoothPath,
            Vector3 prevPos, Vector3 cornerPos, Vector3 nextPos,
            Vector3 backControlPoint, Vector3 forwardControlPoint,
            ConnectionPoint prevPoint, ConnectionPoint cornerPoint, ConnectionPoint nextPoint,
            float prevTime, float cornerTime,
            float prevSpeed, float cornerSpeed,
            float turnAngle)
        {
            // 辅助点数量（可调整）
            int numPoints = turnAngle > 60f ? 6 : 4;
            
            for (int i = 1; i < numPoints; i++)
            {
                float t = i / (float)(numPoints);
                
                // 计算贝塞尔曲线点
                Vector3 smoothPos;
                if (t <= 0.5f)
                {
                    // 前半段：从前一点到转弯点
                    float localT = t * 2.0f; // 重新缩放t为0-1
                    Vector3 p0 = Vector3.Lerp(prevPos, backControlPoint, localT);
                    Vector3 p1 = Vector3.Lerp(backControlPoint, cornerPos, localT);
                    smoothPos = Vector3.Lerp(p0, p1, localT);
                }
                else
                {
                    // 后半段：从转弯点到下一点
                    float localT = (t - 0.5f) * 2.0f; // 重新缩放t为0-1
                    Vector3 p0 = Vector3.Lerp(cornerPos, forwardControlPoint, localT);
                    Vector3 p1 = Vector3.Lerp(forwardControlPoint, nextPos, localT);
                    smoothPos = Vector3.Lerp(p0, p1, localT);
                }
                
                // 计算辅助点的速度 - 基于ModifySpeed计算的速度
                float speed;
                if (t <= 0.5f)
                {
                    // 前半段：从前一点的速度过渡到转弯点速度
                    speed = Mathf.Lerp(prevSpeed, cornerSpeed, t * 2.0f);
                }
                else
                {
                    // 后半段：从转弯点速度过渡到下一点速度（假设与转弯点相同）
                    speed = cornerSpeed;
                }
                
                // 急转弯处略微降低速度，但幅度小
                if (turnAngle > 60f)
                {
                    speed *= 0.9f; // 只降低10%，保持与ModifySpeed的一致性
                }
                
                // 计算辅助点的时间
                float time;
                if (t <= 0.5f)
                {
                    // 前半段：时间线性插值
                    time = Mathf.Lerp(prevTime, cornerTime, t * 2.0f);
                }
                else
                {
                    // 后半段：时间预估（基于到下一个点的距离和速度）
                    time = cornerTime;
                }
                
                // 创建并添加辅助点 - 关联到最接近的原始点
                ConnectionPoint relatedPoint = t <= 0.5f ? prevPoint : cornerPoint;
                
                smoothPath.Add(new SmoothPathPoint(relatedPoint, smoothPos, time, speed));
            }
        }

        private static void SaveSmoothPathForRobot(string robotId, List<SmoothPathPoint> smoothPath)
        {
            // 转换为RobotController可以使用的格式
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
            
            // 存储到字典
            if (!robotSmoothPaths.ContainsKey(robotId))
            {
                robotSmoothPaths[robotId] = new List<RobotSmoothPathInfo>();
            }
            
            // 限制存储的路径数量（只保留最近的3条）
            var paths = robotSmoothPaths[robotId];
            paths.Add(pathInfo);
            if (paths.Count > 3)
            {
                paths.RemoveAt(0);
            }
        }

        public static RobotSmoothPathInfo GetSmoothPathForRobot(string robotId, long timestamp = 0)
        {
            if (!robotSmoothPaths.ContainsKey(robotId) || robotSmoothPaths[robotId].Count == 0)
            {
                return null;
            }
            
            var paths = robotSmoothPaths[robotId];
            
            // 如果指定了时间戳，查找匹配的路径
            if (timestamp > 0)
            {
                foreach (var path in paths)
                {
                    if (path.Timestamp == timestamp)
                    {
                        return path;
                    }
                }
            }
            
            // 否则返回最新的路径
            return paths[paths.Count - 1];
        }

        private static void ClearUnusedReservations(
            DynamicOccupancyLayer personalOccupancyLayer, 
            string robotId,
            string planId, 
            float currentTime)
        {
            personalOccupancyLayer.ClearPlannedOccupancy(robotId, planId, currentTime);
        }

        private static void ReservePathOccupancy(
            Graph graph,
            IndoorSpace indoorSpace,
            DynamicOccupancyLayer personalOccupancyLayer,
            List<ConnectionPoint> path,
            List<float> times,
            List<float> speeds, 
            string robotId, 
            string newPlanId)
        {
            for (int i = 0; i < path.Count - 1; i++)
            {
                CellSpace currentCellSpace = indoorSpace.GetCellSpaceFromConnectionPoint(path[i]);
                if (currentCellSpace != null)
                {
                    float startTime = times[i];
                    float endTime = times[i + 1];
                    
                    personalOccupancyLayer.SetPlannedOccupancy(
                        currentCellSpace.Id,
                        startTime,
                        endTime,
                        robotId, 
                        newPlanId
                    );
                }
            }
        }

        private static float HeuristicCostEstimate(DynamicOccupancyLayer personalOccupancyLayer, RoutePoint a, RoutePoint b, float speed, SearchMethod searchMethod)
        {
            var p1 = (Point)a.ConnectionPoint.Point;
            var p2 = (Point)b.ConnectionPoint.Point;
            float distance;
            
            switch (searchMethod)
            {
                case SearchMethod.Euclidean_Distance:
                    distance = (float)Math.Sqrt(Math.Pow(p1.X - p2.X, 2) + Math.Pow(p1.Y - p2.Y, 2));
                    break;
                case SearchMethod.Manhattan_Distance:
                    distance = (float)(Math.Abs(p1.X - p2.X) + Math.Abs(p1.Y - p2.Y));
                    break;
                default:
                    UnityEngine.Debug.LogWarning($"Unknown search method: {searchMethod}. Using Euclidean distance.");
                    distance = (float)Math.Sqrt(Math.Pow(p1.X - p2.X, 2) + Math.Pow(p1.Y - p2.Y, 2));
                    break;
            }
            
            float baseHeuristic = distance / speed;
            
            return baseHeuristic;
        }

        private static float ModifySpeed(Graph graph, ConnectionPoint current, ConnectionPoint neighbor, 
            float defaultSpeed, float lastSpeed, Layer layerOfNeighbor, float deltaTime, 
            float maxAcceleration, float maxDeceleration)
        {
            float speedLimit = layerOfNeighbor.SpeedLimit();
            float targetSpeed = defaultSpeed;

            PointLocation currentLocation = graph.GetPointLocation(current);
            PointLocation neighborLocation = graph.GetPointLocation(neighbor);

            switch (currentLocation)
            {
                case PointLocation.InAisle when neighborLocation == PointLocation.ApproachingIntersection:
                    targetSpeed = speedLimit * 0.8f;
                    break;
                case PointLocation.ApproachingIntersection:
                case PointLocation.InIntersection:
                    targetSpeed = speedLimit;
                    break;
                case PointLocation.OutIntersection when neighborLocation == PointLocation.InAisle:
                    targetSpeed = Mathf.Min(defaultSpeed, speedLimit);
                    break;
                default:
                    targetSpeed = Mathf.Min(defaultSpeed, speedLimit);
                    break;
            }

            float speedDifference = targetSpeed - lastSpeed;
            float acceleration = speedDifference / deltaTime;
            
            acceleration = Mathf.Clamp(acceleration, -maxDeceleration, maxAcceleration);
            
            float newSpeed = lastSpeed + acceleration * deltaTime;

            newSpeed = Mathf.Clamp(newSpeed, speedLimit * 0.5f, speedLimit);

            float randomFactor = UnityEngine.Random.Range(0.98f, 1.02f);
            newSpeed *= randomFactor;

            return newSpeed;
        }
    }
}