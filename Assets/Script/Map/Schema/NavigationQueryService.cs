using System.Collections.Generic;
using System.Linq;
using NetTopologySuite.Geometries;
using UnityEngine;

namespace Map
{
    /// <summary>
    /// Rehosts current Graph query semantics on top of NavigationGraph.
    /// Provides world-position lookup, floor resolution, traversal cell resolution,
    /// point location classification, and vertical waypoint discovery.
    /// </summary>
    public sealed class NavigationQueryService
    {
        private readonly IndoorSpace indoorSpace;
        private readonly NavigationGraph graph;
        private readonly float floorHeight;
        private readonly float baseHeight;

        public NavigationQueryService(IndoorSpace indoorSpace, NavigationGraph graph,
            float floorHeight, float baseHeight)
        {
            this.indoorSpace = indoorSpace;
            this.graph = graph;
            this.floorHeight = floorHeight;
            this.baseHeight = baseHeight;
        }

        /// <summary>
        /// Resolves a world position to a Waypoint, preserving the sequence/direction semantics
        /// from the old Graph.GetRoutePointFromCoordinate(position, sequence).
        /// </summary>
        public Waypoint ResolveWaypointFromWorldPosition(Vector3 position, TraversalDirection direction)
        {
            string floorLevel = GetFloorLevelFromPosition(position);
            bool sequence = direction == TraversalDirection.Outbound;

            // Try to find which CellSpace contains this position
            Point point = new Point(position.x, position.z);
            CellSpace foundCellSpace = null;

            // Search on estimated floor first
            if (!string.IsNullOrEmpty(floorLevel))
            {
                var cellSpacesOnFloor = indoorSpace.GetCellSpacesOnFloor(floorLevel);
                foreach (var cellSpace in cellSpacesOnFloor)
                {
                    if (!cellSpace.IsNavigable()) continue;
                    if (cellSpace.Space is Polygon polygon && polygon.Contains(point))
                    {
                        foundCellSpace = cellSpace;

                        // For CellSpace-backed graphs, nodes are keyed by CellSpace Id
                        var directNode = graph.GetNode(cellSpace.Id);
                        if (directNode != null && directNode.Kind == WaypointKind.CellSpace)
                            return directNode;

                        string boundaryId = GetBoundaryIdFromCellSpace(cellSpace.Id, sequence);
                        if (boundaryId != null)
                        {
                            var waypoint = graph.GetNode(boundaryId);
                            if (waypoint != null) return waypoint;
                        }
                    }
                }
            }

            // Fallback: search all CellSpaces
            foreach (var cellSpace in indoorSpace.CellSpaces)
            {
                if (!cellSpace.IsNavigable()) continue;
                if (cellSpace.Space is Polygon polygon && polygon.Contains(point))
                {
                    foundCellSpace = cellSpace;

                    var directNode = graph.GetNode(cellSpace.Id);
                    if (directNode != null && directNode.Kind == WaypointKind.CellSpace)
                        return directNode;

                    string boundaryId = GetBoundaryIdFromCellSpace(cellSpace.Id, sequence);
                    if (boundaryId != null)
                    {
                        var waypoint = graph.GetNode(boundaryId);
                        if (waypoint != null) return waypoint;
                    }
                }
            }

            // Fallback: find nearest navigable cell
            CellSpace nearestCell = foundCellSpace ?? FindNearestNavigableCell(position, floorLevel);
            if (nearestCell != null)
            {
                // Try CellSpace node first
                var cellNode = graph.GetNode(nearestCell.Id);
                if (cellNode != null) return cellNode;

                string boundaryId = GetBoundaryIdFromCellSpace(nearestCell.Id, sequence);
                if (boundaryId != null)
                {
                    var waypoint = graph.GetNode(boundaryId);
                    if (waypoint != null) return waypoint;
                }
            }

            return null;
        }

        public Vector3 GetWorldPosition(Waypoint waypoint)
        {
            return waypoint.Position;
        }

        public (string enterCellId, string exitCellId) GetTraversalCells(Waypoint waypoint)
        {
            return (waypoint.PrimaryCellId, waypoint.SecondaryCellId);
        }

        public string GetTraversalFloor(Waypoint waypoint)
        {
            if (!string.IsNullOrEmpty(waypoint.FloorLevel))
                return waypoint.FloorLevel;

            // Fallback for vertical CPs with null floor
            if (waypoint.ConnectionPointSource != null)
                return indoorSpace.GetConnectionPointFloorLevel(waypoint.ConnectionPointSource.CellBoundary);

            return null;
        }

        public NavigationPointLocation GetPointLocation(Waypoint waypoint, TraversalDirection direction)
        {
            CellSpace next = indoorSpace.GetCellSpaceFromId(waypoint.SecondaryCellId);
            CellSpace past = indoorSpace.GetCellSpaceFromId(waypoint.PrimaryCellId);
            Layer nextLayer = next != null ? indoorSpace.GetLayerFromCellSpaceId(next.Id) : null;
            Layer pastLayer = past != null ? indoorSpace.GetLayerFromCellSpaceId(past.Id) : null;

            if (pastLayer == null || nextLayer == null)
                return NavigationPointLocation.InAisle;
            if (!pastLayer.IsIntersection() && !nextLayer.IsIntersection())
                return NavigationPointLocation.InAisle;
            if (!pastLayer.IsIntersection() && nextLayer.IsIntersection())
                return NavigationPointLocation.ApproachingIntersection;
            if (pastLayer.IsIntersection() && nextLayer.IsIntersection())
                return NavigationPointLocation.InIntersection;

            return NavigationPointLocation.OutIntersection;
        }

        public List<Waypoint> FindVerticalWaypointsBetweenFloors(string fromFloor, string toFloor)
        {
            var sorted = new[] { fromFloor, toFloor }.OrderBy(f => FloorLevelUtils.Parse(f)).ToArray();

            return graph.GetAllNodes()
                .Where(w => w.IsVertical)
                .Where(w =>
                {
                    var floors = new[] { w.PrimaryCellId, w.SecondaryCellId }
                        .Select(id => indoorSpace.GetCellSpaceFromId(id)?.GetLocatedFloor())
                        .Where(f => !string.IsNullOrEmpty(f))
                        .OrderBy(f => FloorLevelUtils.Parse(f))
                        .ToArray();
                    return floors.Length == 2 && floors[0] == sorted[0] && floors[1] == sorted[1];
                })
                .ToList();
        }

        // =====================================================================
        //  Private helpers
        // =====================================================================

        private string GetFloorLevelFromPosition(Vector3 position)
        {
            int floorIndex = Mathf.RoundToInt((position.y - baseHeight) / floorHeight) + 1;
            if (floorIndex <= 0)
                return $"B{Mathf.Abs(floorIndex)}";
            return $"{floorIndex}F";
        }

        private string GetBoundaryIdFromCellSpace(string cellSpaceId, bool sequence)
        {
            var cp = indoorSpace.GetConnectionPointFromCellSpace(cellSpaceId, sequence);
            return cp?.Id;
        }

        private CellSpace FindNearestNavigableCell(Vector3 position, string floorLevel)
        {
            float minDist = float.MaxValue;
            CellSpace nearest = null;

            var candidates = string.IsNullOrEmpty(floorLevel)
                ? indoorSpace.CellSpaces
                : indoorSpace.GetCellSpacesOnFloor(floorLevel);

            if (candidates == null || !candidates.Any())
                candidates = indoorSpace.CellSpaces;

            foreach (var cs in candidates)
            {
                if (!cs.IsNavigable()) continue;
                if (cs.Node is not Point p) continue;

                float dist = Vector2.Distance(
                    new Vector2(position.x, position.z),
                    new Vector2((float)p.X, (float)p.Y));

                if (dist < minDist)
                {
                    minDist = dist;
                    nearest = cs;
                }
            }
            return nearest;
        }
    }
}
