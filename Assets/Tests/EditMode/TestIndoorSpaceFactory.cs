using System.Collections.Generic;
using Map;
using NetTopologySuite.Geometries;
using Newtonsoft.Json.Linq;

namespace EditModeTests
{
    /// <summary>
    /// Shared test fixture factory for IndoorSpace topology tests.
    /// Provides reusable helpers to build directed corridors, multi-floor elevator setups,
    /// and legacy route graphs without duplicating topology setup across test classes.
    /// </summary>
    internal static class TestIndoorSpaceFactory
    {
        private static readonly GeometryFactory gf = new GeometryFactory();

        // =====================================================================
        //  Cell Factories
        // =====================================================================

        public static CellSpace MakeNavigableCell(string id, double x, double y, string floorLevel = "1F")
        {
            var props = new Dictionary<string, object> { { "navigable", true } };
            AddFloorProperties(props, floorLevel);

            Geometry node = gf.CreatePoint(new Coordinate(x, y));
            Geometry space = gf.CreatePolygon(new[]
            {
                new Coordinate(x - 0.5, y - 0.5),
                new Coordinate(x + 0.5, y - 0.5),
                new Coordinate(x + 0.5, y + 0.5),
                new Coordinate(x - 0.5, y + 0.5),
                new Coordinate(x - 0.5, y - 0.5)
            });

            return new CellSpace(id, props, space, node);
        }

        public static CellSpace MakeVerticalCell(string id, double x, double y,
            string locatedFloor, string connectedFloor, string verticalType = "elevator")
        {
            var props = new Dictionary<string, object>
            {
                { "navigable", true },
                { "isVerticalConnection", true }
            };

            var floorAssociations = new JArray
            {
                new JObject
                {
                    ["floorLevel"] = locatedFloor,
                    ["associationType"] = "located"
                },
                new JObject
                {
                    ["floorLevel"] = connectedFloor,
                    ["associationType"] = "connected"
                }
            };

            props["verticalFeatures"] = new JObject
            {
                ["verticalType"] = verticalType,
                ["floorAssociation"] = floorAssociations
            };

            Geometry node = gf.CreatePoint(new Coordinate(x, y));
            Geometry space = gf.CreatePolygon(new[]
            {
                new Coordinate(x - 0.5, y - 0.5),
                new Coordinate(x + 0.5, y - 0.5),
                new Coordinate(x + 0.5, y + 0.5),
                new Coordinate(x - 0.5, y + 0.5),
                new Coordinate(x - 0.5, y - 0.5)
            });

            return new CellSpace(id, props, space, node);
        }

        // =====================================================================
        //  Boundary Factories
        // =====================================================================

        public static CellBoundary MakeHorizontalBoundary(string id, string sourceId, string targetId,
            double midX, double midY)
        {
            Geometry boundary = gf.CreatePoint(new Coordinate(midX, midY));
            Geometry edge = gf.CreateLineString(new[]
            {
                new Coordinate(midX - 0.5, midY),
                new Coordinate(midX + 0.5, midY)
            });

            return new CellBoundary(id, new Dictionary<string, object>(),
                sourceId, targetId, boundary, edge, "horizontal");
        }

        public static CellBoundary MakeVerticalBoundary(string id, string sourceId, string targetId)
        {
            return new CellBoundary(id, new Dictionary<string, object>(),
                sourceId, targetId, null, null, "vertical");
        }

        // =====================================================================
        //  Property Helpers
        // =====================================================================

        public static void AddFloorProperties(Dictionary<string, object> props, string floorLevel)
        {
            if (string.IsNullOrEmpty(floorLevel)) return;

            var floorAssociation = new JObject
            {
                ["floorLevel"] = floorLevel,
                ["associationType"] = "located"
            };

            props["verticalFeatures"] = new JObject
            {
                ["verticalType"] = (string)null,
                ["floorAssociation"] = new JArray { floorAssociation }
            };
        }

        // =====================================================================
        //  Composite Fixtures
        // =====================================================================

        /// <summary>
        /// Builds a single-floor directed corridor:
        ///   c1 (0,0) --cb12--> c2 (2,0) --cb23--> c3 (4,0)
        /// CellBoundary source/target defines the directed edges.
        /// </summary>
        public static IndoorSpace BuildDirectedCorridor()
        {
            var space = new IndoorSpace();
            space.CellSpaces.Add(MakeNavigableCell("c1", 0, 0, "1F"));
            space.CellSpaces.Add(MakeNavigableCell("c2", 2, 0, "1F"));
            space.CellSpaces.Add(MakeNavigableCell("c3", 4, 0, "1F"));
            space.CellBoundaries.Add(MakeHorizontalBoundary("cb12", "c1", "c2", 1, 0));
            space.CellBoundaries.Add(MakeHorizontalBoundary("cb23", "c2", "c3", 3, 0));
            return space;
        }

        /// <summary>
        /// Builds a two-floor elevator topology:
        ///   Floor 1: c1F1 (0,0) --cb_c1F1_EV1--> EV1 (2,0)
        ///   Vertical: EV1 --cb_vert--> EV2
        ///   Floor 2: EV2 (2,0) --cb_EV2_c1F2--> c1F2 (4,0)
        /// </summary>
        public static IndoorSpace BuildTwoFloorElevator()
        {
            var space = new IndoorSpace();
            space.CellSpaces.Add(MakeNavigableCell("c1F1", 0, 0, "1F"));
            space.CellSpaces.Add(MakeVerticalCell("EV1", 2, 0, "1F", "2F", "elevator"));
            space.CellSpaces.Add(MakeVerticalCell("EV2", 2, 0, "2F", "1F", "elevator"));
            space.CellSpaces.Add(MakeNavigableCell("c1F2", 4, 0, "2F"));
            space.CellBoundaries.Add(MakeHorizontalBoundary("cb_c1F1_EV1", "c1F1", "EV1", 1, 0));
            space.CellBoundaries.Add(MakeVerticalBoundary("cb_vert", "EV1", "EV2"));
            space.CellBoundaries.Add(MakeHorizontalBoundary("cb_EV2_c1F2", "EV2", "c1F2", 3, 0));
            return space;
        }

    }
}
