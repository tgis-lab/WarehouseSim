using System;
using System.Collections.Generic;
using Map;
using UnityEngine;

namespace PathPlanning
{
    /// <summary>
    /// A* path finder on CellSpace NavigationGraph. Returns centroid waypoints directly
    /// without ConnectionPoint remap.
    /// </summary>
    public class CellSpaceAStarPathFinder : IPathFinder
    {
        public NavigationPathResult FindPath(
            NavigationGraph graph, Waypoint start, Waypoint end,
            DynamicOccupancyLayer personalOcc, DynamicOccupancyLayer globalOcc,
            string robotId, string currentPlanId,
            float startTime, float speed, float maxAccel, float maxDecel)
        {
            if (graph == null || start == null || end == null)
                return null;

            if (start.Id == end.Id)
            {
                string pid = $"CellPlan_{robotId}_{DateTime.Now.Ticks}";
                return new NavigationPathResult
                {
                    Path = new List<Waypoint> { start },
                    Times = new List<float> { startTime },
                    Speeds = new List<float> { speed },
                    PlanId = pid,
                    Metrics = new PathPlanningMetrics(robotId, pid)
                };
            }

            var openSet = new List<string> { start.Id };
            var closedSet = new HashSet<string>();
            var gScore = new Dictionary<string, float> { { start.Id, 0f } };
            var fScore = new Dictionary<string, float> { { start.Id, HeuristicCost(graph, start.Id, end.Id) } };
            var cameFrom = new Dictionary<string, string>();

            while (openSet.Count > 0)
            {
                string currentId = FindLowest(openSet, fScore);
                if (currentId == end.Id)
                    return ReconstructPath(graph, cameFrom, start.Id, end.Id, robotId, startTime, speed);

                openSet.Remove(currentId);
                closedSet.Add(currentId);

                foreach (var neighbor in graph.GetNeighbors(currentId))
                {
                    if (closedSet.Contains(neighbor.Id)) continue;

                    float edgeCost = EdgeCost(graph, currentId, neighbor.Id);
                    float tentativeG = GetVal(gScore, currentId) + edgeCost;

                    if (!openSet.Contains(neighbor.Id))
                        openSet.Add(neighbor.Id);
                    else if (tentativeG >= GetVal(gScore, neighbor.Id))
                        continue;

                    cameFrom[neighbor.Id] = currentId;
                    gScore[neighbor.Id] = tentativeG;
                    fScore[neighbor.Id] = tentativeG + HeuristicCost(graph, neighbor.Id, end.Id);
                }
            }

            UnityEngine.Debug.LogError($"CellSpaceAStar A* exhausted: explored {closedSet.Count}/{graph.NodeCount} nodes from {start.Id} to {end.Id}. " +
                $"Start has {graph.GetNeighbors(start.Id).Count} neighbors, End has {graph.GetInEdges(end.Id).Count} in-edges. " +
                $"End in closedSet={closedSet.Contains(end.Id)}");
            return null; // No path found
        }

        private float HeuristicCost(NavigationGraph graph, string fromId, string toId)
        {
            var from = graph.GetNode(fromId);
            var to = graph.GetNode(toId);
            if (from == null || to == null) return float.MaxValue;
            return Vector3.Distance(from.Position, to.Position);
        }

        private float EdgeCost(NavigationGraph graph, string fromId, string toId)
        {
            var from = graph.GetNode(fromId);
            var to = graph.GetNode(toId);
            if (from == null || to == null) return float.MaxValue;
            return Vector3.Distance(from.Position, to.Position);
        }

        private string FindLowest(List<string> openSet, Dictionary<string, float> fScore)
        {
            string best = openSet[0];
            float bestVal = GetVal(fScore, best);
            for (int i = 1; i < openSet.Count; i++)
            {
                float val = GetVal(fScore, openSet[i]);
                if (val < bestVal)
                {
                    best = openSet[i];
                    bestVal = val;
                }
            }
            return best;
        }

        private float GetVal(Dictionary<string, float> dict, string key)
        {
            return dict.TryGetValue(key, out float v) ? v : float.PositiveInfinity;
        }

        private NavigationPathResult ReconstructPath(
            NavigationGraph graph, Dictionary<string, string> cameFrom,
            string startId, string endId, string robotId, float startTime, float speed)
        {
            var path = new List<Waypoint>();
            string current = endId;
            while (cameFrom.ContainsKey(current))
            {
                path.Add(graph.GetNode(current));
                current = cameFrom[current];
            }
            path.Add(graph.GetNode(startId));
            path.Reverse();

            // Compute times and speeds
            var times = new List<float> { startTime };
            var speeds = new List<float> { speed };
            float t = startTime;
            float totalLength = 0f;

            for (int i = 1; i < path.Count; i++)
            {
                float d = Vector3.Distance(path[i - 1].Position, path[i].Position);
                totalLength += d;
                t += d / speed;
                times.Add(t);
                speeds.Add(speed);
            }

            float idealLength = graph.ComputeShortestDistance(startId, endId);
            if (idealLength < 0) idealLength = totalLength;

            string planId = $"CellPlan_{robotId}_{DateTime.Now.Ticks}";
            var metrics = new PathPlanningMetrics(robotId, planId)
            {
                ActualPathLength = totalLength,
                TravelTime = t - startTime,
                IdealPathLength = idealLength
            };

            return new NavigationPathResult
            {
                Path = path,
                Times = times,
                Speeds = speeds,
                PlanId = planId,
                Metrics = metrics
            };
        }
    }
}
