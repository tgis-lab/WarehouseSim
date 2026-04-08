using Map;
using NUnit.Framework;
using UnityEngine;

namespace EditModeTests
{
    /// <summary>
    /// Characterization tests that lock the current topology semantics BEFORE the waypoint refactor.
    /// These tests must remain green throughout the entire refactor to prove semantic parity.
    /// </summary>
    public class TopologySemanticsBaselineTests
    {
        // =====================================================================
        //  1. CellBoundary source/target remains directed
        // =====================================================================

        [Test]
        public void CellBoundary_SourceTarget_RemainsDirected()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var cb = indoorSpace.GetCellBoundaryFromId("cb12");

            Assert.AreEqual("c1", cb.Source);
            Assert.AreEqual("c2", cb.Target);
        }

        // =====================================================================
        //  2. Graph sequence lookup: start uses outgoing, end uses incoming
        // =====================================================================

        [Test]
        public void GraphSequenceLookup_StartUsesOutgoing_EndUsesIncoming()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var hypergraph = indoorSpace.GetHypergraph();
            var graph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);
            var queries = new NavigationQueryService(indoorSpace, graph, 3.0f, 0.05f);

            // Position strictly inside c1 (centroid at 0,0; polygon -0.5,-0.5 to 0.5,0.5)
            // Outbound: look for boundary where c1 is Source -> cb12
            var start = queries.ResolveWaypointFromWorldPosition(new Vector3(0f, 0.05f, 0f), TraversalDirection.Outbound);
            Assert.IsNotNull(start, "Start Waypoint should be found inside c1");
            Assert.AreEqual("cb12", start.Id,
                "Outbound from c1: should resolve to cb12 (c1 is source of cb12)");

            // Position strictly inside c3 (centroid at 4,0; polygon 3.5,-0.5 to 4.5,0.5)
            // Inbound: look for boundary where c3 is Target -> cb23
            var goal = queries.ResolveWaypointFromWorldPosition(new Vector3(4f, 0.05f, 0f), TraversalDirection.Inbound);
            Assert.IsNotNull(goal, "Goal Waypoint should be found inside c3");
            Assert.AreEqual("cb23", goal.Id,
                "Inbound to c3: should resolve to cb23 (c3 is target of cb23)");
        }

        // =====================================================================
        //  3. Vertical ConnectionPoint floor uses lower floor
        // =====================================================================

        [Test]
        public void VerticalConnectionPointFloor_UsesLowerFloorForTraversalAnchor()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildTwoFloorElevator();

            // cb_vert connects EV1 (1F) → EV2 (2F)
            // GetConnectionPointFloorLevel should return the lower floor "1F"
            var cb = indoorSpace.GetCellBoundaryFromId("cb_vert");
            string floor = indoorSpace.GetConnectionPointFloorLevel(cb);

            Assert.AreEqual("1F", floor,
                "Vertical boundary floor should resolve to the lower floor (1F)");
        }
    }
}
