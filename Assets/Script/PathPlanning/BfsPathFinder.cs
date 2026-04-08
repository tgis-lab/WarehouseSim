using System;
using System.Collections.Generic;
using Map;
using UnityEngine;

namespace PathPlanning
{
    /// <summary>
    /// BFS path finder on NavigationGraph. Returns waypoint path with uniform speed.
    /// </summary>
    public class BfsPathFinder : IPathFinder
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
                return MakeSingleNodeResult(start, robotId, startTime, speed);
            }

            var visited = new HashSet<string>();
            var queue = new Queue<string>();
            var parent = new Dictionary<string, string>();

            visited.Add(start.Id);
            queue.Enqueue(start.Id);

            while (queue.Count > 0)
            {
                string currentId = queue.Dequeue();
                if (currentId == end.Id)
                    return BuildResult(graph, parent, start, end, robotId, startTime, speed);

                foreach (var neighbor in graph.GetNeighbors(currentId))
                {
                    if (!visited.Contains(neighbor.Id))
                    {
                        visited.Add(neighbor.Id);
                        parent[neighbor.Id] = currentId;
                        queue.Enqueue(neighbor.Id);
                    }
                }
            }

            return null; // No path found
        }

        private NavigationPathResult BuildResult(
            NavigationGraph graph, Dictionary<string, string> parent,
            Waypoint start, Waypoint end, string robotId, float startTime, float speed)
        {
            var path = new List<Waypoint>();
            string current = end.Id;
            while (current != start.Id)
            {
                path.Add(graph.GetNode(current));
                current = parent[current];
            }
            path.Add(start);
            path.Reverse();

            return ComputeTimesAndMetrics(graph, path, robotId, startTime, speed);
        }

        private NavigationPathResult MakeSingleNodeResult(Waypoint node, string robotId, float startTime, float speed)
        {
            string planId = $"BfsPlan_{robotId}_{DateTime.Now.Ticks}";
            return new NavigationPathResult
            {
                Path = new List<Waypoint> { node },
                Times = new List<float> { startTime },
                Speeds = new List<float> { speed },
                PlanId = planId,
                Metrics = new PathPlanningMetrics(robotId, planId)
            };
        }

        private NavigationPathResult ComputeTimesAndMetrics(
            NavigationGraph graph, List<Waypoint> path, string robotId, float startTime, float speed)
        {
            string planId = $"BfsPlan_{robotId}_{DateTime.Now.Ticks}";
            var times = new List<float> { startTime };
            var speeds = new List<float> { speed };
            float totalLength = 0f;
            float t = startTime;

            for (int i = 1; i < path.Count; i++)
            {
                float d = Vector3.Distance(path[i - 1].Position, path[i].Position);
                totalLength += d;
                t += d / speed;
                times.Add(t);
                speeds.Add(speed);
            }

            float idealLength = graph.ComputeShortestDistance(path[0].Id, path[path.Count - 1].Id);
            if (idealLength < 0) idealLength = totalLength;

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
