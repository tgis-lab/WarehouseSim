using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using NetTopologySuite.Geometries;

namespace Map
{
    /// <summary>
    /// CellSpace-level adjacency graph for Standard IndoorGML NRG baseline.
    /// Nodes = CellSpaces, Edges = CellBoundary adjacency (no Container routing).
    /// </summary>
    public class CellSpaceGraph
    {
        private readonly IndoorSpace indoorSpace;
        private readonly Dictionary<string, List<string>> adjacency; // cellId -> [adjacent cellIds]
        private readonly Dictionary<string, CellSpace> cellSpaceMap;  // cellId -> CellSpace

        public CellSpaceGraph(IndoorSpace indoorSpace)
        {
            this.indoorSpace = indoorSpace;
            this.adjacency = new Dictionary<string, List<string>>();
            this.cellSpaceMap = new Dictionary<string, CellSpace>();
            BuildGraph();
        }

        private void BuildGraph()
        {
            // Index all navigable CellSpaces
            foreach (var cs in indoorSpace.CellSpaces)
            {
                if (cs.IsNavigable())
                {
                    cellSpaceMap[cs.Id] = cs;
                    adjacency[cs.Id] = new List<string>();
                }
            }

            // Build adjacency from CellBoundaries
            foreach (var cb in indoorSpace.CellBoundaries)
            {
                string src = cb.Source;
                string tgt = cb.Target;

                if (cellSpaceMap.ContainsKey(src) && cellSpaceMap.ContainsKey(tgt))
                {
                    if (!adjacency[src].Contains(tgt))
                        adjacency[src].Add(tgt);
                    if (!adjacency[tgt].Contains(src))
                        adjacency[tgt].Add(src);
                }
            }

            Debug.Log($"CellSpaceGraph built: {cellSpaceMap.Count} nodes, " +
                      $"{adjacency.Values.Sum(a => a.Count)} edges (bidirectional)");
        }

        public CellSpace GetCellSpace(string id) =>
            cellSpaceMap.TryGetValue(id, out var cs) ? cs : null;

        public List<string> GetNeighbors(string cellId) =>
            adjacency.TryGetValue(cellId, out var neighbors) ? neighbors : new List<string>();

        public bool Contains(string cellId) => cellSpaceMap.ContainsKey(cellId);

        public int NodeCount => cellSpaceMap.Count;

        /// <summary>
        /// A* search on CellSpace adjacency graph.
        /// Returns ordered list of CellSpace IDs from start to goal.
        /// </summary>
        public List<string> FindPath(string startCellId, string goalCellId)
        {
            if (!Contains(startCellId) || !Contains(goalCellId))
            {
                Debug.LogWarning($"CellSpaceGraph: start {startCellId} or goal {goalCellId} not in graph");
                return null;
            }

            var openSet = new SortedSet<(float f, string id)>(
                Comparer<(float f, string id)>.Create((a, b) =>
                {
                    int cmp = a.f.CompareTo(b.f);
                    return cmp != 0 ? cmp : string.Compare(a.id, b.id, StringComparison.Ordinal);
                }));

            var gScore = new Dictionary<string, float>();
            var cameFrom = new Dictionary<string, string>();
            var inOpen = new HashSet<string>();

            gScore[startCellId] = 0f;
            float h = HeuristicCost(startCellId, goalCellId);
            openSet.Add((h, startCellId));
            inOpen.Add(startCellId);

            var closedSet = new HashSet<string>();

            while (openSet.Count > 0)
            {
                var (currentF, currentId) = openSet.Min;
                openSet.Remove(openSet.Min);
                inOpen.Remove(currentId);

                if (currentId == goalCellId)
                    return ReconstructCellPath(cameFrom, currentId);

                closedSet.Add(currentId);

                foreach (var neighborId in GetNeighbors(currentId))
                {
                    if (closedSet.Contains(neighborId)) continue;

                    float tentativeG = gScore[currentId] + EdgeCost(currentId, neighborId);

                    if (!gScore.ContainsKey(neighborId) || tentativeG < gScore[neighborId])
                    {
                        cameFrom[neighborId] = currentId;
                        gScore[neighborId] = tentativeG;
                        float f = tentativeG + HeuristicCost(neighborId, goalCellId);

                        if (inOpen.Contains(neighborId))
                        {
                            // Remove old entry and re-add with new f
                            openSet.RemoveWhere(x => x.id == neighborId);
                        }
                        openSet.Add((f, neighborId));
                        inOpen.Add(neighborId);
                    }
                }
            }

            Debug.LogWarning($"CellSpaceGraph: No path from {startCellId} to {goalCellId}");
            return null;
        }

        private float HeuristicCost(string fromId, string toId)
        {
            var from = cellSpaceMap[fromId];
            var to = cellSpaceMap[toId];
            if (from.Node is not Point p1 || to.Node is not Point p2) return float.MaxValue;
            return (float)(Math.Abs(p1.X - p2.X) + Math.Abs(p1.Y - p2.Y));
        }

        private float EdgeCost(string fromId, string toId)
        {
            var from = cellSpaceMap[fromId];
            var to = cellSpaceMap[toId];
            if (from.Node is not Point p1 || to.Node is not Point p2) return float.MaxValue;

            float baseCost = (float)Math.Sqrt(
                Math.Pow(p1.X - p2.X, 2) + Math.Pow(p1.Y - p2.Y, 2));

            // Vertical transition penalty
            string fromFloor = from.GetLocatedFloor();
            string toFloor = to.GetLocatedFloor();
            if (!string.IsNullOrEmpty(fromFloor) && !string.IsNullOrEmpty(toFloor)
                && fromFloor != toFloor)
            {
                baseCost += 5.0f; // Vertical transition cost
            }

            return baseCost;
        }

        private List<string> ReconstructCellPath(Dictionary<string, string> cameFrom, string current)
        {
            var path = new List<string> { current };
            while (cameFrom.ContainsKey(current))
            {
                current = cameFrom[current];
                path.Add(current);
            }
            path.Reverse();
            return path;
        }

        /// <summary>
        /// Maps a CellSpace-level path to ConnectionPoint-level path.
        /// For each pair of adjacent CellSpaces, finds the CellBoundary between them
        /// and returns its ConnectionPoint.
        /// </summary>
        public (List<ConnectionPoint> path, List<float> speeds) MapToConnectionPointPath(
            List<string> cellPath, float defaultSpeed)
        {
            if (cellPath == null || cellPath.Count < 2)
            {
                Debug.LogWarning("CellSpaceGraph: cellPath too short to map");
                return (new List<ConnectionPoint>(), new List<float>());
            }

            var cpPath = new List<ConnectionPoint>();
            var speeds = new List<float>();

            // For each consecutive pair, find the boundary between them
            for (int i = 0; i < cellPath.Count - 1; i++)
            {
                string fromCell = cellPath[i];
                string toCell = cellPath[i + 1];

                ConnectionPoint cp = FindConnectionPointBetweenCells(fromCell, toCell);
                if (cp != null && (cpPath.Count == 0 || cpPath[cpPath.Count - 1].Id != cp.Id))
                {
                    cpPath.Add(cp);
                    speeds.Add(defaultSpeed);
                }
            }

            Debug.Log($"CellSpaceGraph: Mapped {cellPath.Count} cells to {cpPath.Count} ConnectionPoints");
            return (cpPath, speeds);
        }

        /// <summary>
        /// Finds the ConnectionPoint on the CellBoundary between two adjacent CellSpaces.
        /// </summary>
        private ConnectionPoint FindConnectionPointBetweenCells(string cellId1, string cellId2)
        {
            foreach (var cb in indoorSpace.CellBoundaries)
            {
                if ((cb.Source == cellId1 && cb.Target == cellId2) ||
                    (cb.Source == cellId2 && cb.Target == cellId1))
                {
                    return indoorSpace.GetConnectionPointFromCellBoundary(cb.Id);
                }
            }
            return null;
        }

        /// <summary>
        /// Full pipeline: find CellSpace path, map to ConnectionPoints, generate times.
        /// Compatible with PathPlanner return signature.
        /// </summary>
        public (List<ConnectionPoint> path, List<float> times, List<float> speeds,
                string planId, PathPlanning.PathPlanningMetrics metrics)
            FindPathForRobot(
                RoutePoint startPoint, RoutePoint endPoint,
                string robotId, float startTime, float defaultSpeed)
        {
            var sw = System.Diagnostics.Stopwatch.StartNew();

            // Find CellSpaces containing start and end ConnectionPoints
            string startCellId = FindCellSpaceForConnectionPoint(startPoint.ConnectionPoint);
            string goalCellId = FindCellSpaceForConnectionPoint(endPoint.ConnectionPoint);

            if (startCellId == null || goalCellId == null)
            {
                Debug.LogWarning($"CellSpaceGraph: Cannot locate cells for start/end CPs");
                return (null, null, null, null, null);
            }

            // A* on CellSpace graph
            List<string> cellPath = FindPath(startCellId, goalCellId);
            if (cellPath == null || cellPath.Count == 0)
                return (null, null, null, null, null);

            // Map to ConnectionPoints
            var (cpPath, speeds) = MapToConnectionPointPath(cellPath, defaultSpeed);
            if (cpPath.Count < 1)
                return (null, null, null, null, null);

            // Generate times (uniform speed, no temporal awareness)
            var times = new List<float>();
            float currentTime = startTime;
            times.Add(currentTime);
            float totalLength = 0f;
            for (int i = 1; i < cpPath.Count; i++)
            {
                float dist;
                if (cpPath[i].IsVertical() || cpPath[i - 1].IsVertical())
                {
                    // Vertical transition: use fixed time penalty (3s base + 0.5s per floor)
                    dist = defaultSpeed * 3.5f; // approximate distance equivalent
                }
                else
                {
                    dist = (float)cpPath[i - 1].Point.Distance(cpPath[i].Point);
                }
                totalLength += dist;
                currentTime += dist / defaultSpeed;
                times.Add(currentTime);
            }

            sw.Stop();
            string planId = $"CellSpacePlan_{robotId}_{DateTime.Now.Ticks}";
            var metrics = new PathPlanning.PathPlanningMetrics(robotId, planId);
            metrics.PlanningTime = (float)sw.Elapsed.TotalSeconds;
            metrics.ActualPathLength = totalLength;

            return (cpPath, times, speeds, planId, metrics);
        }

        private string FindCellSpaceForConnectionPoint(ConnectionPoint cp)
        {
            // Try source cell of the boundary
            CellSpace cs = indoorSpace.GetCellSpaceFromConnectionPoint(cp, true);
            if (cs != null && Contains(cs.Id)) return cs.Id;

            // Fallback: try target cell
            cs = indoorSpace.GetCellSpaceFromConnectionPoint(cp, false);
            if (cs != null && Contains(cs.Id)) return cs.Id;

            return null;
        }
    }
}
