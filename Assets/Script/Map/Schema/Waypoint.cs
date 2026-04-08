using NetTopologySuite.Geometries;
using UnityEngine;

namespace Map
{
    public enum WaypointKind
    {
        ConnectionPoint,
        CellSpace
    }

    /// <summary>
    /// Unified navigation node for both CP-backed and CellSpace-backed navigation points.
    /// Replaces RoutePoint (CP graph) and CellSpaceGraph node (cell-space graph).
    /// </summary>
    public sealed class Waypoint
    {
        public string Id { get; }
        public Geometry Point { get; }
        public Vector3 Position { get; }
        public string FloorLevel { get; }
        public WaypointKind Kind { get; }
        public bool IsVertical { get; }
        public bool IsElevator { get; }
        public bool IsStair { get; }
        public string PrimaryCellId { get; }
        public string SecondaryCellId { get; }
        public ConnectionPoint ConnectionPointSource { get; }
        public CellSpace CellSpaceSource { get; }

        private Waypoint(
            string id, Geometry point, Vector3 position, string floorLevel,
            WaypointKind kind, bool isVertical, bool isElevator, bool isStair,
            string primaryCellId, string secondaryCellId,
            ConnectionPoint connectionPointSource, CellSpace cellSpaceSource)
        {
            Id = id;
            Point = point;
            Position = position;
            FloorLevel = floorLevel;
            Kind = kind;
            IsVertical = isVertical;
            IsElevator = isElevator;
            IsStair = isStair;
            PrimaryCellId = primaryCellId;
            SecondaryCellId = secondaryCellId;
            ConnectionPointSource = connectionPointSource;
            CellSpaceSource = cellSpaceSource;
        }

        public static Waypoint FromConnectionPoint(ConnectionPoint cp, IndoorSpace indoorSpace, float floorHeight, float baseHeight)
        {
            string floorLevel = cp.FloorLevel ?? indoorSpace.GetConnectionPointFloorLevel(cp.CellBoundary);
            float y = FloorLevelUtils.GetFloorHeight(floorLevel, floorHeight, baseHeight);
            Vector3 position;
            if (cp.Point is Point p)
                position = new Vector3((float)p.X, y, (float)p.Y);
            else
                position = new Vector3(0, y, 0);

            return new Waypoint(
                cp.Id,
                cp.Point,
                position,
                floorLevel,
                WaypointKind.ConnectionPoint,
                cp.IsVertical(),
                cp.IsElevator,
                cp.IsStair,
                cp.CellBoundary.Source,
                cp.CellBoundary.Target,
                cp,
                null);
        }

        public static Waypoint FromCellSpace(CellSpace cs, float floorHeight, float baseHeight)
        {
            string floorLevel = cs.GetLocatedFloor();
            bool isElevator = cs.IsElevator();
            bool isStair = cs.IsStair();
            bool isVertical = isElevator || isStair;

            float y = FloorLevelUtils.GetFloorHeight(floorLevel, floorHeight, baseHeight);
            Vector3 position;
            if (cs.Node is Point p)
                position = new Vector3((float)p.X, y, (float)p.Y);
            else
                position = Vector3.zero;

            return new Waypoint(
                cs.Id,
                cs.Node,
                position,
                floorLevel,
                WaypointKind.CellSpace,
                isVertical,
                isElevator,
                isStair,
                cs.Id,
                null,
                null,
                cs);
        }
    }
}
