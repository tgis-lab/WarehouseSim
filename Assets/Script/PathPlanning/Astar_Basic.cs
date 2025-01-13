using System;
using System.Collections.Generic;
using UnityEngine;
using Map;
using NetTopologySuite.Geometries;

namespace PathPlanning
{
    public class Astar_Basic
    {
        public static (List<ConnectionPoint> path, List<float> speeds) AstarAlgorithm(
            Graph graph, 
            RoutePoint startPosition, 
            RoutePoint endPosition, 
            float defaultSpeed, 
            SearchMethod searchMethod)
        {
            if (graph == null || startPosition == null || endPosition == null)
            {
                Debug.LogError("Invalid input parameters for A* algorithm");
                return (new List<ConnectionPoint>(), new List<float>());
            }

            Debug.Log($"Starting A* algorithm from {startPosition.ConnectionPoint.Id} to {endPosition.ConnectionPoint.Id}");

            var openSet = new List<RoutePoint> { startPosition };
            var closedSet = new HashSet<RoutePoint>();
            var gScore = new Dictionary<RoutePoint, float>();
            var fScore = new Dictionary<RoutePoint, float>();
            var previous = new Dictionary<RoutePoint, RoutePoint>();
            var speedTo = new Dictionary<RoutePoint, float>();

            gScore[startPosition] = 0;
            fScore[startPosition] = HeuristicCostEstimate(searchMethod, startPosition, endPosition);
            speedTo[startPosition] = defaultSpeed;

            while (openSet.Count > 0)
            {
                RoutePoint current = FindLowestFScore(openSet, fScore);

                if (current == endPosition)
                {
                    Debug.Log("Path found");
                    return ReconstructPath(previous, speedTo, current);
                }

                openSet.Remove(current);
                closedSet.Add(current);

                foreach (var neighborConnection in current.Children)
                {
                    RoutePoint neighbor = graph.GetRoutePointFormConnectionPoint(neighborConnection);
                    if (neighbor == null || closedSet.Contains(neighbor)) continue;

                    float currentSpeed = ModifySpeed(graph, current.ConnectionPoint, neighborConnection, defaultSpeed, speedTo[current]);
                    float distance = (float)current.ConnectionPoint.Point.Distance(neighborConnection.Point);
                    float tentativeGScore = gScore[current] + (distance / currentSpeed);

                    if (!openSet.Contains(neighbor))
                        openSet.Add(neighbor);
                    else if (tentativeGScore >= gScore.GetValueOrDefault(neighbor, float.PositiveInfinity))
                        continue;

                    previous[neighbor] = current;
                    speedTo[neighbor] = currentSpeed;
                    gScore[neighbor] = tentativeGScore;
                    fScore[neighbor] = gScore[neighbor] + HeuristicCostEstimate(searchMethod, neighbor, endPosition);
                }
            }

            Debug.LogWarning($"No path found from {startPosition.ConnectionPoint.Id} to {endPosition.ConnectionPoint.Id}");
            return (new List<ConnectionPoint>(), new List<float>());
        }

        private static RoutePoint FindLowestFScore(List<RoutePoint> openSet, Dictionary<RoutePoint, float> fScore)
        {
            RoutePoint lowest = openSet[0];
            foreach (var node in openSet)
            {
                if (fScore.GetValueOrDefault(node, float.PositiveInfinity) < fScore.GetValueOrDefault(lowest, float.PositiveInfinity))
                {
                    lowest = node;
                }
            }
            return lowest;
        }

        private static float HeuristicCostEstimate(SearchMethod searchMethod, RoutePoint a, RoutePoint b)
        {
            var p1 = (Point)a.ConnectionPoint.Point;
            var p2 = (Point)b.ConnectionPoint.Point;

            // Euclidean distance
            switch (searchMethod)
            {
                case SearchMethod.Euclidean_Distance:
                    return (float)Math.Sqrt(Math.Pow(p1.X - p2.X, 2) + Math.Pow(p1.Y - p2.Y, 2));
                case SearchMethod.Manhattan_Distance:
                    return (float)(Math.Abs(p1.X - p2.X) + Math.Abs(p1.Y - p2.Y));
                default: return (float)(Math.Abs(p1.X - p2.X) + Math.Abs(p1.Y - p2.Y));
            }
        }

        private static (List<ConnectionPoint> path, List<float> speeds) ReconstructPath(
            Dictionary<RoutePoint, RoutePoint> previous, 
            Dictionary<RoutePoint, float> speedTo, 
            RoutePoint current)
        {
            var path = new List<ConnectionPoint>();
            var speeds = new List<float>();

            while (previous.ContainsKey(current))
            {
                path.Add(current.ConnectionPoint);
                speeds.Add(speedTo[current]);
                current = previous[current];
            }

            path.Add(current.ConnectionPoint);
            speeds.Add(speedTo[current]);

            path.Reverse();
            speeds.Reverse();

            return (path, speeds);
        }

        private static float ModifySpeed(Graph graph, ConnectionPoint current, ConnectionPoint neighbor, float defaultSpeed, float lastSpeed)
        {
            return defaultSpeed;
        }
    }
}