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
    /// Tests for NavigationGraph built from Hypergraph (CP-backed graph)
    /// and SpatialIndex (STRtree-backed spatial queries over CellSpaces).
    /// Rewrites of the original Graph/RoutePoint tests to use the unified Waypoint topology.
    /// </summary>
    public class GraphTests
    {
        private GeometryFactory gf;

        [SetUp]
        public void SetUp()
        {
            gf = new GeometryFactory();
        }

        // =====================================================================
        //  NavigationGraph Tests (replaces old Graph + RoutePoint tests)
        // =====================================================================

        [Test]
        public void AddNode_And_GetNode_ReturnsCorrectWaypoint()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var hypergraph = indoorSpace.GetHypergraph();
            var graph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);

            var wp = graph.GetNode("cb12");
            Assert.IsNotNull(wp, "Waypoint for cb12 should be found");
            Assert.AreEqual("cb12", wp.Id);
            Assert.AreEqual(WaypointKind.ConnectionPoint, wp.Kind);
        }

        [Test]
        public void GetNode_NotFound_ReturnsNull()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var hypergraph = indoorSpace.GetHypergraph();
            var graph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);

            Assert.IsNull(graph.GetNode("nonexistent"),
                "Should return null for a waypoint not in the graph");
        }

        [Test]
        public void WaypointPosition_ReturnsCorrectVector3()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var hypergraph = indoorSpace.GetHypergraph();
            var graph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);

            // cb12 boundary centroid is at (1, 0)
            var wp = graph.GetNode("cb12");
            Assert.AreEqual(1.0f, wp.Position.x, 0.01f);
            Assert.AreEqual(0.05f, wp.Position.y, 0.01f);
            Assert.AreEqual(0.0f, wp.Position.z, 0.01f);
        }

        [Test]
        public void WaypointPosition_SecondFloor_HasCorrectHeight()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildTwoFloorElevator();
            var hypergraph = indoorSpace.GetHypergraph();
            var graph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);

            var wp = graph.GetNode("cb_EV2_c1F2");
            Assert.AreEqual(3.05f, wp.Position.y, 0.01f,
                "Y coordinate on 2F should be 3.05");
        }

        [Test]
        public void FloorLevel_HorizontalCP_ReturnsSourceFloor()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var hypergraph = indoorSpace.GetHypergraph();
            var graph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);

            var wp12 = graph.GetNode("cb12");
            var wp23 = graph.GetNode("cb23");
            Assert.AreEqual(wp12.FloorLevel, wp23.FloorLevel,
                "Both CPs on 1F should have the same floor");
        }

        [Test]
        public void FloorLevel_VerticalCP_ReturnsLowerFloor()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildTwoFloorElevator();
            var hypergraph = indoorSpace.GetHypergraph();
            var graph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);

            var wpVert = graph.GetNode("cb_vert");
            var wp1F = graph.GetNode("cb_c1F1_EV1");
            Assert.AreEqual(wp1F.FloorLevel, wpVert.FloorLevel,
                "Vertical CP should resolve to the lower floor (1F)");
        }

        [Test]
        public void DifferentFloors_WaypointsHaveDifferentFloorLevels()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildTwoFloorElevator();
            var hypergraph = indoorSpace.GetHypergraph();
            var graph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);

            var wp1F = graph.GetNode("cb_c1F1_EV1");
            var wp2F = graph.GetNode("cb_EV2_c1F2");
            Assert.AreNotEqual(wp1F.FloorLevel, wp2F.FloorLevel);
        }

        [Test]
        public void FindVerticalWaypoints_FindsCorrectOnes()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildTwoFloorElevator();
            var hypergraph = indoorSpace.GetHypergraph();
            var graph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);
            var queries = new NavigationQueryService(indoorSpace, graph, 3.0f, 0.05f);

            var verticals = queries.FindVerticalWaypointsBetweenFloors("1F", "2F");
            Assert.AreEqual(1, verticals.Count);
            Assert.AreEqual("cb_vert", verticals[0].Id);
        }

        [Test]
        public void FindVerticalWaypoints_ReversedOrder_StillFinds()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildTwoFloorElevator();
            var hypergraph = indoorSpace.GetHypergraph();
            var graph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);
            var queries = new NavigationQueryService(indoorSpace, graph, 3.0f, 0.05f);

            var verticals = queries.FindVerticalWaypointsBetweenFloors("2F", "1F");
            Assert.AreEqual(1, verticals.Count);
        }

        [Test]
        public void FindVerticalWaypoints_NoConnection_ReturnsEmpty()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var hypergraph = indoorSpace.GetHypergraph();
            var graph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);
            var queries = new NavigationQueryService(indoorSpace, graph, 3.0f, 0.05f);

            var verticals = queries.FindVerticalWaypointsBetweenFloors("1F", "2F");
            Assert.AreEqual(0, verticals.Count);
        }

        // =====================================================================
        //  SpatialIndex Tests (unchanged — no Graph dependency)
        // =====================================================================

        private CellSpace MakeNavigableCell(string id, double x, double y, string floorLevel = "1F")
        {
            return TestIndoorSpaceFactory.MakeNavigableCell(id, x, y, floorLevel);
        }

        [Test]
        public void SpatialIndex_Insert_And_QueryPoint_ReturnsMatchingCell()
        {
            var index = new SpatialIndex(enable3D: false);
            CellSpace cell = MakeNavigableCell("cell_5_5", 5, 5, "1F");
            index.Insert(cell);
            List<CellSpace> result = index.QueryPoint(new Vector2(5f, 5f));
            Assert.AreEqual(1, result.Count);
            Assert.AreEqual("cell_5_5", result[0].Id);
        }

        [Test]
        public void SpatialIndex_QueryPoint_OutsideCell_ReturnsEmpty()
        {
            var index = new SpatialIndex(enable3D: false);
            CellSpace cell = MakeNavigableCell("cell_5_5", 5, 5, "1F");
            index.Insert(cell);
            List<CellSpace> result = index.QueryPoint(new Vector2(100f, 100f));
            Assert.AreEqual(0, result.Count);
        }

        [Test]
        public void SpatialIndex_Insert_MultipleCells_QueryFindsCorrectOne()
        {
            var index = new SpatialIndex(enable3D: false);
            index.Insert(MakeNavigableCell("cellA", 0, 0, "1F"));
            index.Insert(MakeNavigableCell("cellB", 10, 10, "1F"));
            index.Insert(MakeNavigableCell("cellC", 20, 20, "1F"));
            List<CellSpace> result = index.QueryPoint(new Vector2(10f, 10f));
            Assert.AreEqual(1, result.Count);
            Assert.AreEqual("cellB", result[0].Id);
        }

        [Test]
        public void SpatialIndex_QueryIntersect_Envelope_ReturnsCellsInRange()
        {
            var index = new SpatialIndex(enable3D: false);
            index.Insert(MakeNavigableCell("cellA", 0, 0, "1F"));
            index.Insert(MakeNavigableCell("cellB", 2, 0, "1F"));
            index.Insert(MakeNavigableCell("cellC", 10, 10, "1F"));
            var envelope = new Envelope(-1, 3, -1, 1);
            List<CellSpace> result = index.QueryIntersect(envelope);
            Assert.AreEqual(2, result.Count);
            var ids = result.Select(c => c.Id).ToList();
            Assert.Contains("cellA", ids);
            Assert.Contains("cellB", ids);
        }

        [Test]
        public void SpatialIndex_QueryIntersect_EmptyEnvelope_ReturnsEmpty()
        {
            var index = new SpatialIndex(enable3D: false);
            index.Insert(MakeNavigableCell("cellA", 0, 0, "1F"));
            var envelope = new Envelope(100, 200, 100, 200);
            Assert.AreEqual(0, index.QueryIntersect(envelope).Count);
        }

        [Test]
        public void SpatialIndex_Remove_CellNoLongerReturnedByQuery()
        {
            var index = new SpatialIndex(enable3D: false);
            index.Insert(MakeNavigableCell("cellA", 5, 5, "1F"));
            index.Insert(MakeNavigableCell("cellB", 5.2, 5.2, "1F"));
            index.Remove("cellA");
            List<CellSpace> result = index.QueryPoint(new Vector2(5f, 5f));
            Assert.IsFalse(result.Any(c => c.Id == "cellA"));
        }

        [Test]
        public void SpatialIndex_Remove_OtherCellsUnaffected()
        {
            var index = new SpatialIndex(enable3D: false);
            index.Insert(MakeNavigableCell("cellA", 0, 0, "1F"));
            index.Insert(MakeNavigableCell("cellB", 10, 10, "1F"));
            index.Remove("cellA");
            List<CellSpace> result = index.QueryPoint(new Vector2(10f, 10f));
            Assert.AreEqual(1, result.Count);
            Assert.AreEqual("cellB", result[0].Id);
        }

        [Test]
        public void SpatialIndex_RebuildIndex_PreservesData()
        {
            var index = new SpatialIndex(enable3D: false);
            var cellA = MakeNavigableCell("cellA", 0, 0, "1F");
            var cellB = MakeNavigableCell("cellB", 5, 5, "1F");
            var cellC = MakeNavigableCell("cellC", 10, 10, "1F");
            index.Insert(cellA);
            index.Insert(cellB);
            index.Insert(cellC);
            index.RebuildIndex(new List<CellSpace> { cellA, cellB, cellC });
            Assert.AreEqual(1, index.QueryPoint(new Vector2(0f, 0f)).Count);
            Assert.AreEqual(1, index.QueryPoint(new Vector2(5f, 5f)).Count);
            Assert.AreEqual(1, index.QueryPoint(new Vector2(10f, 10f)).Count);
        }

        [Test]
        public void SpatialIndex_RebuildIndex_RemovesOldData()
        {
            var index = new SpatialIndex(enable3D: false);
            var cellA = MakeNavigableCell("cellA", 0, 0, "1F");
            index.Insert(cellA);
            index.Insert(MakeNavigableCell("cellB", 5, 5, "1F"));
            index.RebuildIndex(new List<CellSpace> { cellA });
            Assert.AreEqual(1, index.QueryPoint(new Vector2(0f, 0f)).Count);
            Assert.AreEqual(0, index.QueryPoint(new Vector2(5f, 5f)).Count);
        }

        [Test]
        public void SpatialIndex_RebuildIndex_WithNull_ClearsIndex()
        {
            var index = new SpatialIndex(enable3D: false);
            index.Insert(MakeNavigableCell("cellA", 0, 0, "1F"));
            index.RebuildIndex(null);
            Assert.AreEqual(0, index.QueryPoint(new Vector2(0f, 0f)).Count);
        }
    }
}
