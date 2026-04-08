using System.Collections.Generic;
using System.Linq;
using Map;
using NUnit.Framework;
using UnityEngine;

namespace EditModeTests
{
    public class NavigationQueryServiceTests
    {
        // =====================================================================
        //  ResolveWaypointFromWorldPosition
        // =====================================================================

        [Test]
        public void ResolveWaypointFromWorldPosition_PreservesTraversalDirectionSemantics()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var hypergraph = indoorSpace.GetHypergraph();
            var graph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);
            var queries = new NavigationQueryService(indoorSpace, graph, 3.0f, 0.05f);

            // Position inside c1 (centroid 0,0), outbound → cb12 (c1 is source)
            var start = queries.ResolveWaypointFromWorldPosition(
                new Vector3(0f, 0.05f, 0f), TraversalDirection.Outbound);
            Assert.IsNotNull(start);
            Assert.AreEqual("cb12", start.Id,
                "Outbound from c1 should resolve to cb12");

            // Position inside c3 (centroid 4,0), inbound → cb23 (c3 is target)
            var end = queries.ResolveWaypointFromWorldPosition(
                new Vector3(4f, 0.05f, 0f), TraversalDirection.Inbound);
            Assert.IsNotNull(end);
            Assert.AreEqual("cb23", end.Id,
                "Inbound to c3 should resolve to cb23");
        }

        [Test]
        public void ResolveWaypointFromWorldPosition_FallbackToNearestCell_WhenOutsideAllCells()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var hypergraph = indoorSpace.GetHypergraph();
            var graph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);
            var queries = new NavigationQueryService(indoorSpace, graph, 3.0f, 0.05f);

            // Position far outside any cell polygon
            var result = queries.ResolveWaypointFromWorldPosition(
                new Vector3(100f, 0.05f, 100f), TraversalDirection.Outbound);

            // Should fallback to nearest navigable cell, not return null
            Assert.IsNotNull(result, "Should fallback to nearest cell when position is outside all cells");
        }

        // =====================================================================
        //  GetTraversalCells
        // =====================================================================

        [Test]
        public void GetTraversalCells_ForConnectionPointWaypoint_ReturnsBoundarySides()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var hypergraph = indoorSpace.GetHypergraph();
            var graph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);
            var queries = new NavigationQueryService(indoorSpace, graph, 3.0f, 0.05f);

            var waypoint = graph.GetNode("cb12");
            var (enterCellId, exitCellId) = queries.GetTraversalCells(waypoint);

            Assert.AreEqual("c1", enterCellId, "PrimaryCellId = Source of cb12");
            Assert.AreEqual("c2", exitCellId, "SecondaryCellId = Target of cb12");
        }

        [Test]
        public void GetTraversalCells_ForCellSpaceWaypoint_ReturnsSelfId()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);
            var queries = new NavigationQueryService(indoorSpace, graph, 3.0f, 0.05f);

            var waypoint = graph.GetNode("c2");
            var (enterCellId, exitCellId) = queries.GetTraversalCells(waypoint);

            Assert.AreEqual("c2", enterCellId);
            Assert.IsNull(exitCellId,
                "CellSpace waypoint has no secondary cell");
        }

        // =====================================================================
        //  GetTraversalFloor
        // =====================================================================

        [Test]
        public void GetTraversalFloor_ReturnsFloorLevel()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildTwoFloorElevator();
            var hypergraph = indoorSpace.GetHypergraph();
            var graph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);
            var queries = new NavigationQueryService(indoorSpace, graph, 3.0f, 0.05f);

            var verticalWp = graph.GetNode("cb_vert");
            Assert.AreEqual("1F", queries.GetTraversalFloor(verticalWp),
                "Vertical waypoint floor should be 1F (lower floor)");

            var horizontalWp = graph.GetNode("cb_c1F1_EV1");
            Assert.AreEqual("1F", queries.GetTraversalFloor(horizontalWp));
        }

        // =====================================================================
        //  FindVerticalWaypointsBetweenFloors
        // =====================================================================

        [Test]
        public void FindVerticalWaypointsBetweenFloors_FindsElevator()
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
        public void FindVerticalWaypointsBetweenFloors_ReversedOrder_StillFinds()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildTwoFloorElevator();
            var hypergraph = indoorSpace.GetHypergraph();
            var graph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);
            var queries = new NavigationQueryService(indoorSpace, graph, 3.0f, 0.05f);

            var verticals = queries.FindVerticalWaypointsBetweenFloors("2F", "1F");

            Assert.AreEqual(1, verticals.Count,
                "Should find vertical connection regardless of floor argument order");
        }

        [Test]
        public void FindVerticalWaypointsBetweenFloors_NoConnection_ReturnsEmpty()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var hypergraph = indoorSpace.GetHypergraph();
            var graph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);
            var queries = new NavigationQueryService(indoorSpace, graph, 3.0f, 0.05f);

            var verticals = queries.FindVerticalWaypointsBetweenFloors("1F", "2F");

            Assert.AreEqual(0, verticals.Count);
        }

        // =====================================================================
        //  GetPointLocation (placeholder for intersection classification)
        // =====================================================================

        [Test]
        public void GetPointLocation_NoIntersectionLayer_ReturnsInAisle()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var hypergraph = indoorSpace.GetHypergraph();
            var graph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);
            var queries = new NavigationQueryService(indoorSpace, graph, 3.0f, 0.05f);

            var waypoint = graph.GetNode("cb12");
            var location = queries.GetPointLocation(waypoint, TraversalDirection.Outbound);

            Assert.AreEqual(NavigationPointLocation.InAisle, location,
                "Without intersection layers, all points should be InAisle");
        }
    }
}
