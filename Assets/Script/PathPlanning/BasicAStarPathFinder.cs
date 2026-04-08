using System;
using System.Collections.Generic;
using Map;
using UnityEngine;

namespace PathPlanning
{
    /// <summary>
    /// A* path finder on NavigationGraph. Ports Astar_Basic logic to Waypoint-based graph.
    /// </summary>
    public class BasicAStarPathFinder : IPathFinder
    {
        private readonly SearchMethod searchMethod;

        public BasicAStarPathFinder(SearchMethod searchMethod)
        {
            this.searchMethod = searchMethod;
        }

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
                string pid = $"BasicPlan_{robotId}_{DateTime.Now.Ticks}";
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
            var fScore = new Dictionary<string, float> { { start.Id, Heuristic(start, end) } };
            var previous = new Dictionary<string, string>();
            var speedTo = new Dictionary<string, float> { { start.Id, speed } };

            while (openSet.Count > 0)
            {
                string currentId = FindLowestFScore(openSet, fScore);
                if (currentId == end.Id)
                    return Reconstruct(graph, previous, speedTo, start, end, robotId, startTime);

                openSet.Remove(currentId);
                closedSet.Add(currentId);

                foreach (var neighbor in graph.GetNeighbors(currentId))
                {
                    if (closedSet.Contains(neighbor.Id)) continue;

                    var current = graph.GetNode(currentId);
                    float distance = Vector3.Distance(current.Position, neighbor.Position);
                    float currentSpeed = speed;
                    float tentativeG = GetScore(gScore, currentId) + (distance / currentSpeed);

                    if (!openSet.Contains(neighbor.Id))
                        openSet.Add(neighbor.Id);
                    else if (tentativeG >= GetScore(gScore, neighbor.Id))
                        continue;

                    previous[neighbor.Id] = currentId;
                    speedTo[neighbor.Id] = currentSpeed;
                    gScore[neighbor.Id] = tentativeG;
                    fScore[neighbor.Id] = tentativeG + Heuristic(neighbor, end);
                }
            }

            return null; // No path found
        }

        private float Heuristic(Waypoint a, Waypoint b)
        {
            switch (searchMethod)
            {
                case SearchMethod.Euclidean_Distance:
                    return Vector3.Distance(a.Position, b.Position);
                case SearchMethod.Manhattan_Distance:
                    return Mathf.Abs(a.Position.x - b.Position.x) +
                           Mathf.Abs(a.Position.z - b.Position.z);
                default:
                    return Vector3.Distance(a.Position, b.Position);
            }
        }

        private string FindLowestFScore(List<string> openSet, Dictionary<string, float> fScore)
        {
            string lowest = openSet[0];
            float lowestScore = GetScore(fScore, lowest);
            for (int i = 1; i < openSet.Count; i++)
            {
                float score = GetScore(fScore, openSet[i]);
                if (score < lowestScore)
                {
                    lowest = openSet[i];
                    lowestScore = score;
                }
            }
            return lowest;
        }

        private float GetScore(Dictionary<string, float> scores, string id)
        {
            return scores.TryGetValue(id, out float v) ? v : float.PositiveInfinity;
        }

        private NavigationPathResult Reconstruct(
            NavigationGraph graph, Dictionary<string, string> previous,
            Dictionary<string, float> speedTo,
            Waypoint start, Waypoint end, string robotId, float startTime)
        {
            var path = new List<Waypoint>();
            var speeds = new List<float>();
            string current = end.Id;

            while (previous.ContainsKey(current))
            {
                path.Add(graph.GetNode(current));
                speeds.Add(speedTo.TryGetValue(current, out float s) ? s : 1f);
                current = previous[current];
            }
            path.Add(start);
            speeds.Add(speedTo.TryGetValue(start.Id, out float startSpeed) ? startSpeed : 1f);

            path.Reverse();
            speeds.Reverse();

            // Compute times
            var times = new List<float> { startTime };
            float t = startTime;
            for (int i = 1; i < path.Count; i++)
            {
                float d = Vector3.Distance(path[i - 1].Position, path[i].Position);
                t += d / speeds[i];
                times.Add(t);
            }

            // Metrics
            float totalLength = 0f;
            for (int i = 1; i < path.Count; i++)
                totalLength += Vector3.Distance(path[i - 1].Position, path[i].Position);

            float idealLength = graph.ComputeShortestDistance(path[0].Id, path[path.Count - 1].Id);
            if (idealLength < 0) idealLength = totalLength;

            string planId = $"BasicPlan_{robotId}_{DateTime.Now.Ticks}";
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
