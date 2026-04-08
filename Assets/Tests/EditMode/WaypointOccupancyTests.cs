using System.Collections.Generic;
using System.Linq;
using Map;
using NUnit.Framework;
using CentralControl;

namespace EditModeTests
{
    public class WaypointOccupancyTests
    {
        [Test]
        public void RecordPathOccupancy_AcceptsWaypointPath()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var hypergraph = indoorSpace.GetHypergraph();
            var graph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);

            // Manually add edge so we have a proper path
            graph.AddEdge("cb12", "cb23");

            var waypointPath = new List<Waypoint>
            {
                graph.GetNode("cb12"),
                graph.GetNode("cb23")
            };
            var times = new List<float> { 0f, 2f };

            var layer = new DynamicOccupancyLayer(indoorSpace, 60f, 0f, false);

            // This should not throw — proves the Waypoint overload exists and works
            layer.RecordPathOccupancy(waypointPath, times, "R1", "plan-1");

            // Verify occupancy was recorded on at least one cell
            var c1Occ = layer.GetOccupancies("c1", 0f, 10f);
            var c2Occ = layer.GetOccupancies("c2", 0f, 10f);

            Assert.IsTrue(c1Occ.Count > 0 || c2Occ.Count > 0,
                "At least one cell should have occupancy recorded from the waypoint path");
        }

        [Test]
        public void RecordPathOccupancy_WaypointPath_MapsToSameCellsAsConnectionPointPath()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var hypergraph = indoorSpace.GetHypergraph();

            // Build both path representations
            var cp12 = indoorSpace.GetConnectionPointFromCellBoundary("cb12");
            var cp23 = indoorSpace.GetConnectionPointFromCellBoundary("cb23");
            var cpPath = new List<ConnectionPoint> { cp12, cp23 };

            var navGraph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);
            navGraph.AddEdge("cb12", "cb23");
            var wpPath = new List<Waypoint> { navGraph.GetNode("cb12"), navGraph.GetNode("cb23") };

            var times = new List<float> { 0f, 2f };

            // Record via old API
            var layerOld = new DynamicOccupancyLayer(indoorSpace, 60f, 0f, false);
            layerOld.RecordPathOccupancy(cpPath, times, "R1", "plan-old");

            // Record via new API
            var layerNew = new DynamicOccupancyLayer(indoorSpace, 60f, 0f, false);
            layerNew.RecordPathOccupancy(wpPath, times, "R1", "plan-new");

            // Both should produce occupancies on the same cells
            var oldC1 = layerOld.GetOccupancies("c1", 0f, 10f);
            var newC1 = layerNew.GetOccupancies("c1", 0f, 10f);
            var oldC2 = layerOld.GetOccupancies("c2", 0f, 10f);
            var newC2 = layerNew.GetOccupancies("c2", 0f, 10f);

            Assert.AreEqual(oldC1.Count, newC1.Count,
                "Waypoint path should produce same c1 occupancy count as ConnectionPoint path");
            Assert.AreEqual(oldC2.Count, newC2.Count,
                "Waypoint path should produce same c2 occupancy count as ConnectionPoint path");
        }

        [Test]
        public void ConflictDetector_AcceptsWaypointPath()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var hypergraph = indoorSpace.GetHypergraph();
            var navGraph = NavigationGraphBuilder.FromHypergraph(hypergraph, indoorSpace, 3.0f, 0.05f);
            navGraph.AddEdge("cb12", "cb23");

            var wpPath = new List<Waypoint> { navGraph.GetNode("cb12"), navGraph.GetNode("cb23") };
            var times = new List<float> { 0f, 2f };

            var layer = new DynamicOccupancyLayer(indoorSpace, 60f, 0f, false);
            var detector = new CellSpaceConflictDetector();

            // Should not throw
            var conflicts = detector.DetectConflicts(layer, indoorSpace, wpPath, times, "R1");

            Assert.IsNotNull(conflicts);
        }
        // =====================================================================
        //  Edge case tests for CellSpace waypoints in occupancy
        // =====================================================================

        [Test]
        public void RecordPathOccupancy_CellSpaceWaypoints_RecordsDirectly()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);

            var waypointPath = new List<Waypoint>
            {
                graph.GetNode("c1"),
                graph.GetNode("c2"),
                graph.GetNode("c3")
            };
            var times = new List<float> { 0f, 1f, 2f };

            var layer = new DynamicOccupancyLayer(indoorSpace, 60f, 0f, false);
            layer.RecordPathOccupancy(waypointPath, times, "R1", "plan-cell");

            var c1Occ = layer.GetOccupancies("c1", 0f, 10f);
            var c2Occ = layer.GetOccupancies("c2", 0f, 10f);

            Assert.IsTrue(c1Occ.Count > 0, "CellSpace waypoint c1 should have occupancy recorded");
            Assert.IsTrue(c2Occ.Count > 0, "CellSpace waypoint c2 should have occupancy recorded");
        }

        [Test]
        public void RecordPathOccupancy_NullPath_DoesNotThrow()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var layer = new DynamicOccupancyLayer(indoorSpace, 60f, 0f, false);

            Assert.DoesNotThrow(() =>
                layer.RecordPathOccupancy((List<Waypoint>)null, new List<float> { 0f }, "R1", "plan"));
        }

        [Test]
        public void RecordPathOccupancy_SingleWaypoint_DoesNotThrow()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var graph = NavigationGraphBuilder.FromCellSpaces(indoorSpace, 3.0f, 0.05f);
            var layer = new DynamicOccupancyLayer(indoorSpace, 60f, 0f, false);

            var singlePath = new List<Waypoint> { graph.GetNode("c1") };
            Assert.DoesNotThrow(() =>
                layer.RecordPathOccupancy(singlePath, new List<float> { 0f }, "R1", "plan"));
        }
    }
}
