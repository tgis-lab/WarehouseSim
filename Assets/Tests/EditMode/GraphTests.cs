using System.Collections.Generic;
using System.Linq;
using Map;
using NetTopologySuite.Geometries;
using Newtonsoft.Json.Linq;
using NUnit.Framework;
using UnityEngine;

namespace EditModeTests
{
    /// <summary>
    /// Edit Mode tests for Graph (route-point graph built from IndoorSpace)
    /// and SpatialIndex (STRtree-backed spatial queries over CellSpaces).
    /// </summary>
    public class GraphTests
    {
        private IndoorSpace indoorSpace;
        private GeometryFactory gf;

        // =====================================================================
        //  Helper Methods
        // =====================================================================

        /// <summary>
        /// Creates a navigable CellSpace with a Point node at (x, y) and a 1x1 square polygon.
        /// Floor level is set via verticalFeatures / FloorAssociation JObject.
        /// </summary>
        private CellSpace MakeNavigableCell(string id, double x, double y, string floorLevel = "1F")
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

        /// <summary>
        /// Creates a navigable CellSpace that is flagged as a vertical connection
        /// (elevator or stair). Associates it with multiple floors via verticalFeatures.
        /// </summary>
        private CellSpace MakeVerticalCell(string id, double x, double y,
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

            var verticalFeatures = new JObject
            {
                ["verticalType"] = verticalType,
                ["floorAssociation"] = floorAssociations
            };

            props["verticalFeatures"] = verticalFeatures;

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

        /// <summary>
        /// Adds floor-level metadata via the verticalFeatures / FloorAssociation JObject
        /// pattern that CellSpace.GetLocatedFloor() expects.
        /// </summary>
        private void AddFloorProperties(Dictionary<string, object> props, string floorLevel)
        {
            if (string.IsNullOrEmpty(floorLevel)) return;

            var floorAssociation = new JObject
            {
                ["floorLevel"] = floorLevel,
                ["associationType"] = "located"
            };

            var verticalFeatures = new JObject
            {
                ["verticalType"] = (string)null,
                ["floorAssociation"] = new JArray { floorAssociation }
            };

            props["verticalFeatures"] = verticalFeatures;
        }

        /// <summary>
        /// Creates a horizontal CellBoundary between two cells.
        /// The boundary geometry is a Point at (midX, midY).
        /// </summary>
        private CellBoundary MakeHorizontalBoundary(string id, string sourceId, string targetId,
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

        /// <summary>
        /// Creates a vertical CellBoundary (elevator / stair transition between floors).
        /// </summary>
        private CellBoundary MakeVerticalBoundary(string id, string sourceId, string targetId)
        {
            return new CellBoundary(id, new Dictionary<string, object>(),
                sourceId, targetId, null, null, "vertical");
        }

        /// <summary>
        /// Builds a small single-floor IndoorSpace with 3 navigable cells in a line:
        ///   c1 (0,0) -- c2 (2,0) -- c3 (4,0)
        /// Two horizontal boundaries connect them.
        /// Returns the IndoorSpace and the Graph built on top of it.
        /// </summary>
        private (IndoorSpace space, Graph graph) BuildSingleFloorGraph()
        {
            var space = new IndoorSpace();

            space.CellSpaces.Add(MakeNavigableCell("c1", 0, 0, "1F"));
            space.CellSpaces.Add(MakeNavigableCell("c2", 2, 0, "1F"));
            space.CellSpaces.Add(MakeNavigableCell("c3", 4, 0, "1F"));

            var cb1 = MakeHorizontalBoundary("cb12", "c1", "c2", 1, 0);
            var cb2 = MakeHorizontalBoundary("cb23", "c2", "c3", 3, 0);
            space.CellBoundaries.Add(cb1);
            space.CellBoundaries.Add(cb2);

            var graph = new Graph(space, 3.0f);

            // Build route points from boundaries and add them
            foreach (var cb in space.CellBoundaries)
            {
                ConnectionPoint cp = space.GetConnectionPointFromCellBoundary(cb.Id);
                graph.AddNode(new RoutePoint(cp));
            }

            return (space, graph);
        }

        /// <summary>
        /// Builds a two-floor IndoorSpace:
        ///   Floor 1: c1F1 (0,0) -- EV1 (2,0)   [EV1 is elevator on 1F]
        ///   Floor 2: EV2 (2,0) -- c1F2 (4,0)    [EV2 is elevator on 2F]
        ///   Vertical boundary: cb_vert between EV1 and EV2
        /// Returns the IndoorSpace and Graph.
        /// </summary>
        private (IndoorSpace space, Graph graph) BuildTwoFloorGraph()
        {
            var space = new IndoorSpace();

            space.CellSpaces.Add(MakeNavigableCell("c1F1", 0, 0, "1F"));
            space.CellSpaces.Add(MakeVerticalCell("EV1", 2, 0, "1F", "2F", "elevator"));
            space.CellSpaces.Add(MakeVerticalCell("EV2", 2, 0, "2F", "1F", "elevator"));
            space.CellSpaces.Add(MakeNavigableCell("c1F2", 4, 0, "2F"));

            var cbH1 = MakeHorizontalBoundary("cb_c1F1_EV1", "c1F1", "EV1", 1, 0);
            var cbVert = MakeVerticalBoundary("cb_vert", "EV1", "EV2");
            var cbH2 = MakeHorizontalBoundary("cb_EV2_c1F2", "EV2", "c1F2", 3, 0);
            space.CellBoundaries.Add(cbH1);
            space.CellBoundaries.Add(cbVert);
            space.CellBoundaries.Add(cbH2);

            var graph = new Graph(space, 3.0f);

            foreach (var cb in space.CellBoundaries)
            {
                ConnectionPoint cp = space.GetConnectionPointFromCellBoundary(cb.Id);
                graph.AddNode(new RoutePoint(cp));
            }

            return (space, graph);
        }

        // =====================================================================
        //  SetUp
        // =====================================================================

        [SetUp]
        public void SetUp()
        {
            gf = new GeometryFactory();
            indoorSpace = null;
        }

        // =====================================================================
        //  Graph Tests
        // =====================================================================

        // ----- 1. AddNode + GetRoutePointFromConnectionPoint -----

        [Test]
        public void AddNode_And_GetRoutePointFromConnectionPoint_ReturnsCorrectRoutePoint()
        {
            var (space, graph) = BuildSingleFloorGraph();

            // Retrieve the ConnectionPoint for boundary cb12
            ConnectionPoint cp12 = space.GetConnectionPointFromCellBoundary("cb12");
            Assert.IsNotNull(cp12, "cp12 should exist");

            RoutePoint rp = graph.GetRoutePointFromConnectionPoint(cp12);

            Assert.IsNotNull(rp, "RoutePoint for cb12 should be found");
            Assert.AreEqual(cp12.Id, rp.ConnectionPoint.Id);
        }

        [Test]
        public void AddNode_DuplicateNode_IsNotAddedTwice()
        {
            var (space, graph) = BuildSingleFloorGraph();

            int countBefore = graph.RoutePoints.Count;

            // Try adding the same route point again
            ConnectionPoint cp12 = space.GetConnectionPointFromCellBoundary("cb12");
            RoutePoint existing = graph.GetRoutePointFromConnectionPoint(cp12);
            graph.AddNode(existing);

            Assert.AreEqual(countBefore, graph.RoutePoints.Count,
                "Duplicate AddNode should not increase RoutePoints count");
        }

        [Test]
        public void GetRoutePointFromConnectionPoint_NotFound_ReturnsNull()
        {
            var (space, graph) = BuildSingleFloorGraph();

            // Create a ConnectionPoint that was never added to the graph
            var fakeBoundary = MakeHorizontalBoundary("fake_cb", "c1", "c2", 99, 99);
            ConnectionPoint fakeCp = fakeBoundary.ConvertToConnectionPoint("1F", false, false);

            RoutePoint rp = graph.GetRoutePointFromConnectionPoint(fakeCp);
            Assert.IsNull(rp, "Should return null for a ConnectionPoint not in the graph");
        }

        // ----- 2. GetCoordinatesFromConnectionPoint -----

        [Test]
        public void GetCoordinatesFromConnectionPoint_ReturnsCorrectVector3()
        {
            var (space, graph) = BuildSingleFloorGraph();

            // cb12 boundary centroid is at (1, 0) -- the midpoint we created it at
            ConnectionPoint cp12 = space.GetConnectionPointFromCellBoundary("cb12");
            Vector3 coords = graph.GetCoordinatesFromConnectionPoint(cp12);

            // X = point.X, Z = point.Y, Y = floor height (unified: includes baseHeight 0.05)
            // 1F floor height = (1-1) * 3.0 + 0.05 = 0.05
            Assert.AreEqual(1.0f, coords.x, 0.01f, "X should be the point's X coordinate");
            Assert.AreEqual(0.05f, coords.y, 0.01f, "Y should be 0.05 for floor 1F (with baseHeight)");
            Assert.AreEqual(0.0f, coords.z, 0.01f, "Z should be the point's Y coordinate");
        }

        [Test]
        public void GetCoordinatesFromConnectionPoint_SecondFloor_HasCorrectHeight()
        {
            var (space, graph) = BuildTwoFloorGraph();

            // cb_EV2_c1F2 is a horizontal boundary between EV2 (2F) and c1F2 (2F)
            ConnectionPoint cpH2 = space.GetConnectionPointFromCellBoundary("cb_EV2_c1F2");
            Vector3 coords = graph.GetCoordinatesFromConnectionPoint(cpH2);

            // 2F floor height = (2-1) * 3.0 + 0.05 = 3.05 (unified with baseHeight)
            Assert.AreEqual(3.05f, coords.y, 0.01f,
                "Y coordinate on 2F should be 3.05 (one floor height above ground + baseHeight)");
        }

        [Test]
        public void GetCoordinatesFromConnectionPoint_NotFound_ReturnsZero()
        {
            var (space, graph) = BuildSingleFloorGraph();

            var fakeBoundary = MakeHorizontalBoundary("fake_cb", "c1", "c2", 99, 99);
            ConnectionPoint fakeCp = fakeBoundary.ConvertToConnectionPoint("1F", false, false);

            Vector3 coords = graph.GetCoordinatesFromConnectionPoint(fakeCp);
            Assert.AreEqual(Vector3.zero, coords,
                "Should return Vector3.zero for unknown ConnectionPoint");
        }

        // ----- 3. GetFloorLevelForConnectionPoint (horizontal and vertical CPs) -----

        [Test]
        public void GetFloorLevelForConnectionPoint_HorizontalCP_ReturnsSourceFloor()
        {
            var (space, graph) = BuildSingleFloorGraph();

            ConnectionPoint cp12 = space.GetConnectionPointFromCellBoundary("cb12");

            // AreOnSameFloor internally calls GetFloorLevelForConnectionPoint
            // If the CP is on 1F, then it and another 1F CP should be on the same floor
            ConnectionPoint cp23 = space.GetConnectionPointFromCellBoundary("cb23");
            Assert.IsTrue(graph.AreOnSameFloor(cp12, cp23),
                "Both horizontal CPs on 1F should be on the same floor");
        }

        [Test]
        public void GetFloorLevelForConnectionPoint_VerticalCP_ReturnsLowerFloor()
        {
            var (space, graph) = BuildTwoFloorGraph();

            // The vertical boundary connects EV1 (1F) and EV2 (2F)
            // GetFloorLevelForConnectionPoint should return the lower floor (1F)
            ConnectionPoint cpVert = space.GetConnectionPointFromCellBoundary("cb_vert");
            ConnectionPoint cpH1 = space.GetConnectionPointFromCellBoundary("cb_c1F1_EV1");

            // If vertical CP resolves to 1F, it should be on the same floor as cpH1 (which is 1F)
            Assert.IsTrue(graph.AreOnSameFloor(cpVert, cpH1),
                "Vertical CP should resolve to the lower floor (1F), matching cpH1");
        }

        // ----- 4. AreOnSameFloor -----

        [Test]
        public void AreOnSameFloor_SameFloor_ReturnsTrue()
        {
            var (space, graph) = BuildSingleFloorGraph();

            ConnectionPoint cp12 = space.GetConnectionPointFromCellBoundary("cb12");
            ConnectionPoint cp23 = space.GetConnectionPointFromCellBoundary("cb23");

            Assert.IsTrue(graph.AreOnSameFloor(cp12, cp23));
        }

        [Test]
        public void AreOnSameFloor_DifferentFloors_ReturnsFalse()
        {
            var (space, graph) = BuildTwoFloorGraph();

            // cb_c1F1_EV1 is on 1F, cb_EV2_c1F2 is on 2F
            ConnectionPoint cp1F = space.GetConnectionPointFromCellBoundary("cb_c1F1_EV1");
            ConnectionPoint cp2F = space.GetConnectionPointFromCellBoundary("cb_EV2_c1F2");

            Assert.IsFalse(graph.AreOnSameFloor(cp1F, cp2F),
                "CPs on different floors should not be on the same floor");
        }

        // ----- 5. GetFloorDifference -----

        [Test]
        public void GetFloorDifference_SameFloor_ReturnsZero()
        {
            var (space, graph) = BuildSingleFloorGraph();

            ConnectionPoint cp12 = space.GetConnectionPointFromCellBoundary("cb12");
            ConnectionPoint cp23 = space.GetConnectionPointFromCellBoundary("cb23");

            Assert.AreEqual(0, graph.GetFloorDifference(cp12, cp23));
        }

        [Test]
        public void GetFloorDifference_OneFloorApart_ReturnsOne()
        {
            var (space, graph) = BuildTwoFloorGraph();

            ConnectionPoint cp1F = space.GetConnectionPointFromCellBoundary("cb_c1F1_EV1");
            ConnectionPoint cp2F = space.GetConnectionPointFromCellBoundary("cb_EV2_c1F2");

            Assert.AreEqual(1, graph.GetFloorDifference(cp1F, cp2F),
                "1F and 2F should have a floor difference of 1");
        }

        [Test]
        public void GetFloorDifference_IsSymmetric()
        {
            var (space, graph) = BuildTwoFloorGraph();

            ConnectionPoint cp1F = space.GetConnectionPointFromCellBoundary("cb_c1F1_EV1");
            ConnectionPoint cp2F = space.GetConnectionPointFromCellBoundary("cb_EV2_c1F2");

            Assert.AreEqual(
                graph.GetFloorDifference(cp1F, cp2F),
                graph.GetFloorDifference(cp2F, cp1F),
                "Floor difference should be symmetric (absolute value)");
        }

        // ----- 6. FindVerticalConnectionsBetweenFloors -----

        [Test]
        public void FindVerticalConnectionsBetweenFloors_FindsCorrectCPs()
        {
            var (space, graph) = BuildTwoFloorGraph();

            List<ConnectionPoint> verticals = graph.FindVerticalConnectionsBetweenFloors("1F", "2F");

            Assert.IsNotNull(verticals);
            Assert.AreEqual(1, verticals.Count,
                "Should find exactly one vertical connection between 1F and 2F");
            Assert.AreEqual("cb_vert", verticals[0].Id);
            Assert.IsTrue(verticals[0].IsVertical(),
                "The returned ConnectionPoint should be a vertical transition");
        }

        [Test]
        public void FindVerticalConnectionsBetweenFloors_ReversedOrder_StillFinds()
        {
            var (space, graph) = BuildTwoFloorGraph();

            // Pass floors in reverse order -- should still find the connection
            List<ConnectionPoint> verticals = graph.FindVerticalConnectionsBetweenFloors("2F", "1F");

            Assert.IsNotNull(verticals);
            Assert.AreEqual(1, verticals.Count,
                "Should find vertical connection regardless of floor argument order");
        }

        [Test]
        public void FindVerticalConnectionsBetweenFloors_NoConnection_ReturnsEmpty()
        {
            var (space, graph) = BuildSingleFloorGraph();

            // Single-floor graph has no vertical connections
            List<ConnectionPoint> verticals = graph.FindVerticalConnectionsBetweenFloors("1F", "2F");

            Assert.IsNotNull(verticals);
            Assert.AreEqual(0, verticals.Count,
                "Should return empty list when no vertical connections exist");
        }

        // =====================================================================
        //  SpatialIndex Tests
        // =====================================================================

        // ----- 1. Insert + QueryPoint returns matching cells -----

        [Test]
        public void SpatialIndex_Insert_And_QueryPoint_ReturnsMatchingCell()
        {
            var index = new SpatialIndex(enable3D: false);

            // Cell centred at (5, 5) with a 1x1 polygon from (4.5,4.5) to (5.5,5.5)
            CellSpace cell = MakeNavigableCell("cell_5_5", 5, 5, "1F");
            index.Insert(cell);

            // Query at (5, 5) -- should hit the cell
            List<CellSpace> result = index.QueryPoint(new Vector2(5f, 5f));

            Assert.AreEqual(1, result.Count, "Query at centre should return the cell");
            Assert.AreEqual("cell_5_5", result[0].Id);
        }

        [Test]
        public void SpatialIndex_QueryPoint_OutsideCell_ReturnsEmpty()
        {
            var index = new SpatialIndex(enable3D: false);

            CellSpace cell = MakeNavigableCell("cell_5_5", 5, 5, "1F");
            index.Insert(cell);

            // Query far away from the cell
            List<CellSpace> result = index.QueryPoint(new Vector2(100f, 100f));

            Assert.AreEqual(0, result.Count, "Query outside any cell should return empty");
        }

        [Test]
        public void SpatialIndex_Insert_MultipleCells_QueryFindsCorrectOne()
        {
            var index = new SpatialIndex(enable3D: false);

            CellSpace cellA = MakeNavigableCell("cellA", 0, 0, "1F");
            CellSpace cellB = MakeNavigableCell("cellB", 10, 10, "1F");
            CellSpace cellC = MakeNavigableCell("cellC", 20, 20, "1F");
            index.Insert(cellA);
            index.Insert(cellB);
            index.Insert(cellC);

            List<CellSpace> result = index.QueryPoint(new Vector2(10f, 10f));

            Assert.AreEqual(1, result.Count);
            Assert.AreEqual("cellB", result[0].Id,
                "Should return only the cell that contains the query point");
        }

        // ----- 2. QueryIntersect with envelope returns cells in range -----

        [Test]
        public void SpatialIndex_QueryIntersect_Envelope_ReturnsCellsInRange()
        {
            var index = new SpatialIndex(enable3D: false);

            CellSpace cellA = MakeNavigableCell("cellA", 0, 0, "1F");   // polygon (-0.5,-0.5 to 0.5,0.5)
            CellSpace cellB = MakeNavigableCell("cellB", 2, 0, "1F");   // polygon (1.5,-0.5 to 2.5,0.5)
            CellSpace cellC = MakeNavigableCell("cellC", 10, 10, "1F"); // far away
            index.Insert(cellA);
            index.Insert(cellB);
            index.Insert(cellC);

            // Envelope covering (−1, −1) to (3, 1) -- should include cellA and cellB but not cellC
            var envelope = new Envelope(-1, 3, -1, 1);
            List<CellSpace> result = index.QueryIntersect(envelope);

            Assert.AreEqual(2, result.Count, "Envelope should cover cellA and cellB");
            var ids = result.Select(c => c.Id).ToList();
            Assert.Contains("cellA", ids);
            Assert.Contains("cellB", ids);
        }

        [Test]
        public void SpatialIndex_QueryIntersect_EmptyEnvelope_ReturnsEmpty()
        {
            var index = new SpatialIndex(enable3D: false);

            CellSpace cellA = MakeNavigableCell("cellA", 0, 0, "1F");
            index.Insert(cellA);

            // Envelope far away
            var envelope = new Envelope(100, 200, 100, 200);
            List<CellSpace> result = index.QueryIntersect(envelope);

            Assert.AreEqual(0, result.Count);
        }

        // ----- 3. Remove + Query no longer returns cell -----

        [Test]
        public void SpatialIndex_Remove_CellNoLongerReturnedByQuery()
        {
            var index = new SpatialIndex(enable3D: false);

            CellSpace cellA = MakeNavigableCell("cellA", 5, 5, "1F");
            CellSpace cellB = MakeNavigableCell("cellB", 5.2, 5.2, "1F");
            index.Insert(cellA);
            index.Insert(cellB);

            // Before removal -- both cells should be found at (5, 5)
            List<CellSpace> beforeRemoval = index.QueryPoint(new Vector2(5f, 5f));
            Assert.IsTrue(beforeRemoval.Any(c => c.Id == "cellA"),
                "cellA should be found before removal");

            // Remove cellA
            index.Remove("cellA");

            // After removal -- only cellB should be found at (5, 5)
            List<CellSpace> afterRemoval = index.QueryPoint(new Vector2(5f, 5f));
            Assert.IsFalse(afterRemoval.Any(c => c.Id == "cellA"),
                "cellA should NOT be returned after removal");
        }

        [Test]
        public void SpatialIndex_Remove_OtherCellsUnaffected()
        {
            var index = new SpatialIndex(enable3D: false);

            CellSpace cellA = MakeNavigableCell("cellA", 0, 0, "1F");
            CellSpace cellB = MakeNavigableCell("cellB", 10, 10, "1F");
            index.Insert(cellA);
            index.Insert(cellB);

            index.Remove("cellA");

            // cellB should still be findable
            List<CellSpace> result = index.QueryPoint(new Vector2(10f, 10f));
            Assert.AreEqual(1, result.Count);
            Assert.AreEqual("cellB", result[0].Id,
                "Removing cellA should not affect cellB");
        }

        // ----- 4. RebuildIndex preserves data -----

        [Test]
        public void SpatialIndex_RebuildIndex_PreservesData()
        {
            var index = new SpatialIndex(enable3D: false);

            CellSpace cellA = MakeNavigableCell("cellA", 0, 0, "1F");
            CellSpace cellB = MakeNavigableCell("cellB", 5, 5, "1F");
            CellSpace cellC = MakeNavigableCell("cellC", 10, 10, "1F");
            index.Insert(cellA);
            index.Insert(cellB);
            index.Insert(cellC);

            // Rebuild the index from the same list of CellSpaces
            index.RebuildIndex(new List<CellSpace> { cellA, cellB, cellC });

            // All cells should still be queryable
            Assert.AreEqual(1, index.QueryPoint(new Vector2(0f, 0f)).Count,
                "cellA should be queryable after rebuild");
            Assert.AreEqual(1, index.QueryPoint(new Vector2(5f, 5f)).Count,
                "cellB should be queryable after rebuild");
            Assert.AreEqual(1, index.QueryPoint(new Vector2(10f, 10f)).Count,
                "cellC should be queryable after rebuild");
        }

        [Test]
        public void SpatialIndex_RebuildIndex_RemovesOldData()
        {
            var index = new SpatialIndex(enable3D: false);

            CellSpace cellA = MakeNavigableCell("cellA", 0, 0, "1F");
            CellSpace cellB = MakeNavigableCell("cellB", 5, 5, "1F");
            index.Insert(cellA);
            index.Insert(cellB);

            // Rebuild with only cellA -- cellB should be gone
            index.RebuildIndex(new List<CellSpace> { cellA });

            Assert.AreEqual(1, index.QueryPoint(new Vector2(0f, 0f)).Count,
                "cellA should survive rebuild");
            Assert.AreEqual(0, index.QueryPoint(new Vector2(5f, 5f)).Count,
                "cellB should be removed after rebuild with subset");
        }

        [Test]
        public void SpatialIndex_RebuildIndex_WithNull_ClearsIndex()
        {
            var index = new SpatialIndex(enable3D: false);

            CellSpace cellA = MakeNavigableCell("cellA", 0, 0, "1F");
            index.Insert(cellA);

            // Rebuild with null list
            index.RebuildIndex(null);

            Assert.AreEqual(0, index.QueryPoint(new Vector2(0f, 0f)).Count,
                "Rebuild with null should clear the index");
        }
    }
}
