using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Map;
using CentralControl;

namespace PathPlanning
{
    public enum SearchAlgorithm
    {
        BFS,
        Astar_Basic,
        TimeSpaceAStar,
        AnytimeSIPP,
        CellSpaceAStar  // CellSpace NRG baseline
    }

    public enum SearchMethod
    {
        Euclidean_Distance,
        Manhattan_Distance,
        NotApplicable
    }

    public class PathPlanner
    {
        // Navigation graph fields
        private NavigationGraph cpGraph;
        private NavigationGraph cellSpaceNavGraph;
        private NavigationQueryService cpQueries;
        private NavigationQueryService cellQueries;

        private IndoorSpace indoorSpace;
        private SearchMethod searchMethod;
        private SearchAlgorithm searchAlgorithm;
        private string robotId;
        private float maxAcceleration;
        private float maxDeceleration;

        // Non-temporal path finders
        private BfsPathFinder bfsPathFinder;
        private BasicAStarPathFinder basicAStarPathFinder;
        private CellSpaceAStarPathFinder cellSpaceAStarPathFinder;

        // Temporal path finders (adapters wrapping ported algorithms)
        private IPathFinder timeSpaceAStarPathFinder;
        private IPathFinder anytimeSippPathFinder;

        public PathPlanner(
            NavigationGraph cpGraph,
            NavigationGraph cellSpaceNavGraph,
            NavigationQueryService cpQueries,
            NavigationQueryService cellQueries,
            IndoorSpace indoorSpace,
            SearchAlgorithm algorithm,
            SearchMethod method,
            string robotId,
            float maxAcceleration,
            float maxDeceleration)
        {
            this.cpGraph = cpGraph;
            this.cellSpaceNavGraph = cellSpaceNavGraph;
            this.cpQueries = cpQueries;
            this.cellQueries = cellQueries;
            this.indoorSpace = indoorSpace;
            this.searchAlgorithm = algorithm;
            this.searchMethod = method;
            this.robotId = robotId;
            this.maxAcceleration = maxAcceleration;
            this.maxDeceleration = maxDeceleration;

            this.bfsPathFinder = new BfsPathFinder();
            this.basicAStarPathFinder = new BasicAStarPathFinder(method);
            this.cellSpaceAStarPathFinder = new CellSpaceAStarPathFinder();

            // Create temporal path finders
            this.timeSpaceAStarPathFinder = new TimeSpaceAStarPathFinder(indoorSpace, method);
            this.anytimeSippPathFinder = new AnytimeSippPathFinder(indoorSpace, method);
        }

        public void SetCellSpaceNavigationGraph(NavigationGraph graph) { this.cellSpaceNavGraph = graph; }
        public void SetAlgorithm(SearchAlgorithm algorithm) { this.searchAlgorithm = algorithm; }
        public SearchAlgorithm GetAlgorithm() { return searchAlgorithm; }
        public void SetDynamics(float accel, float decel) { this.maxAcceleration = accel; this.maxDeceleration = decel; }

        /// <summary>
        /// Registers temporal path finders (adapters wrapping ported algorithms).
        /// Must be called after construction, before first path planning with temporal algorithms.
        /// </summary>
        public void SetTemporalPathFinders(IPathFinder timeSpaceAStar, IPathFinder anytimeSipp)
        {
            this.timeSpaceAStarPathFinder = timeSpaceAStar;
            this.anytimeSippPathFinder = anytimeSipp;
        }

        /// <summary>
        /// Waypoint-based path finding. Returns NavigationPathResult.
        /// </summary>
        public NavigationPathResult FindPathWithDynamicObstacles(
            DynamicOccupancyLayer personalOccupancyLayer,
            DynamicOccupancyLayer globalOccupancyLayer,
            Waypoint startPoint,
            Waypoint endPoint,
            string robotId,
            string currentPlanId,
            float startTime,
            float speed)
        {
            switch (this.searchAlgorithm)
            {
                case SearchAlgorithm.BFS:
                    return bfsPathFinder.FindPath(cpGraph, startPoint, endPoint,
                        personalOccupancyLayer, globalOccupancyLayer, robotId, currentPlanId,
                        startTime, speed, maxAcceleration, maxDeceleration);

                case SearchAlgorithm.Astar_Basic:
                    return basicAStarPathFinder.FindPath(cpGraph, startPoint, endPoint,
                        personalOccupancyLayer, globalOccupancyLayer, robotId, currentPlanId,
                        startTime, speed, maxAcceleration, maxDeceleration);

                case SearchAlgorithm.CellSpaceAStar:
                    // CellSpaceAStar uses cellSpaceNavGraph with CellSpace nodes.
                    // If start/end are CP waypoints, remap to the corresponding CellSpace nodes.
                    if (cellSpaceNavGraph == null)
                    {
                        Debug.LogError($"CellSpaceAStar: cellSpaceNavGraph is null");
                        return null;
                    }
                    Waypoint cellStart = startPoint;
                    Waypoint cellEnd = endPoint;
                    if (startPoint.Kind == WaypointKind.ConnectionPoint)
                    {
                        cellStart = cellSpaceNavGraph.GetNode(startPoint.PrimaryCellId);
                        if (cellStart == null)
                        {
                            Debug.LogWarning($"CellSpaceAStar: PrimaryCellId '{startPoint.PrimaryCellId}' not found in cellSpaceNavGraph, trying SecondaryCellId '{startPoint.SecondaryCellId}'");
                            cellStart = cellSpaceNavGraph.GetNode(startPoint.SecondaryCellId);
                        }
                    }
                    if (endPoint.Kind == WaypointKind.ConnectionPoint)
                    {
                        cellEnd = cellSpaceNavGraph.GetNode(endPoint.SecondaryCellId ?? endPoint.PrimaryCellId);
                        if (cellEnd == null)
                        {
                            Debug.LogWarning($"CellSpaceAStar: SecondaryCellId '{endPoint.SecondaryCellId}' not found in cellSpaceNavGraph, trying PrimaryCellId '{endPoint.PrimaryCellId}'");
                            cellEnd = cellSpaceNavGraph.GetNode(endPoint.PrimaryCellId);
                        }
                    }
                    if (cellStart == null || cellEnd == null)
                    {
                        Debug.LogError($"CellSpaceAStar: Could not remap CP waypoints to CellSpace nodes. start={startPoint.Id}→{cellStart?.Id ?? "null"}, end={endPoint.Id}→{cellEnd?.Id ?? "null"}, cellSpaceNavGraph nodes={cellSpaceNavGraph.NodeCount}");
                        return null;
                    }
                    Debug.Log($"CellSpaceAStar: Remapped {startPoint.Id}→{cellStart.Id}, {endPoint.Id}→{cellEnd.Id}. " +
                              $"cellSpaceNavGraph nodes={cellSpaceNavGraph.NodeCount} edges={cellSpaceNavGraph.EdgeCount}. " +
                              $"startNeighbors={cellSpaceNavGraph.GetNeighbors(cellStart.Id).Count}, endInEdges={cellSpaceNavGraph.GetInEdges(cellEnd.Id).Count}");
                    var cellResult = cellSpaceAStarPathFinder.FindPath(cellSpaceNavGraph, cellStart, cellEnd,
                        personalOccupancyLayer, globalOccupancyLayer, robotId, currentPlanId,
                        startTime, speed, maxAcceleration, maxDeceleration);
                    if (cellResult == null)
                    {
                        Debug.LogError($"CellSpaceAStar: A* returned null from {cellStart.Id} to {cellEnd.Id}. Graph is likely not strongly connected for this pair.");
                    }
                    return cellResult;

                case SearchAlgorithm.TimeSpaceAStar:
                    if (timeSpaceAStarPathFinder == null)
                        throw new InvalidOperationException("TimeSpaceAStar path finder not set");
                    return timeSpaceAStarPathFinder.FindPath(cpGraph, startPoint, endPoint,
                        personalOccupancyLayer, globalOccupancyLayer, robotId, currentPlanId,
                        startTime, speed, maxAcceleration, maxDeceleration);

                case SearchAlgorithm.AnytimeSIPP:
                    if (anytimeSippPathFinder == null)
                        throw new InvalidOperationException("AnytimeSIPP path finder not set");
                    return anytimeSippPathFinder.FindPath(cpGraph, startPoint, endPoint,
                        personalOccupancyLayer, globalOccupancyLayer, robotId, currentPlanId,
                        startTime, speed, maxAcceleration, maxDeceleration);

                default:
                    Debug.LogError($"Unknown search algorithm: {this.searchAlgorithm}");
                    return null;
            }
        }
    }
}
