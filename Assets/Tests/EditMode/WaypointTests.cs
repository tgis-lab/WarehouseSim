using Map;
using NUnit.Framework;
using UnityEngine;

namespace EditModeTests
{
    public class WaypointTests
    {
        [Test]
        public void FromConnectionPoint_CopiesBoundaryAndTraversalMetadata()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildTwoFloorElevator();
            var cp = indoorSpace.GetConnectionPointFromCellBoundary("cb_vert");

            Waypoint waypoint = Waypoint.FromConnectionPoint(cp, indoorSpace, 3.0f, 0.05f);

            Assert.AreEqual(cp.Id, waypoint.Id);
            Assert.AreEqual(WaypointKind.ConnectionPoint, waypoint.Kind);
            Assert.IsTrue(waypoint.IsVertical);
            Assert.AreEqual("EV1", waypoint.PrimaryCellId);
            Assert.AreEqual("EV2", waypoint.SecondaryCellId);
            Assert.AreEqual("1F", waypoint.FloorLevel);
        }

        [Test]
        public void FromConnectionPoint_HorizontalBoundary_HasCorrectPosition()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var cp = indoorSpace.GetConnectionPointFromCellBoundary("cb12");

            Waypoint waypoint = Waypoint.FromConnectionPoint(cp, indoorSpace, 3.0f, 0.05f);

            Assert.AreEqual("cb12", waypoint.Id);
            Assert.AreEqual(WaypointKind.ConnectionPoint, waypoint.Kind);
            Assert.IsFalse(waypoint.IsVertical);
            Assert.AreEqual("c1", waypoint.PrimaryCellId);
            Assert.AreEqual("c2", waypoint.SecondaryCellId);
            Assert.AreEqual("1F", waypoint.FloorLevel);
            // cb12 midpoint is at (1,0) → Vector3(1, 0.05, 0)
            Assert.AreEqual(1.0f, waypoint.Position.x, 0.01f);
            Assert.AreEqual(0.05f, waypoint.Position.y, 0.01f);
            Assert.AreEqual(0.0f, waypoint.Position.z, 0.01f);
        }

        [Test]
        public void FromCellSpace_CopiesCentroidAndFloor()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var cs = indoorSpace.GetCellSpaceFromId("c2");

            Waypoint waypoint = Waypoint.FromCellSpace(cs, 3.0f, 0.05f);

            Assert.AreEqual("c2", waypoint.Id);
            Assert.AreEqual(WaypointKind.CellSpace, waypoint.Kind);
            Assert.IsFalse(waypoint.IsVertical);
            Assert.AreEqual("c2", waypoint.PrimaryCellId);
            Assert.IsNull(waypoint.SecondaryCellId);
            Assert.AreEqual("1F", waypoint.FloorLevel);
            // c2 centroid at (2,0) → Vector3(2, 0.05, 0)
            Assert.AreEqual(2.0f, waypoint.Position.x, 0.01f);
            Assert.AreEqual(0.05f, waypoint.Position.y, 0.01f);
        }

        [Test]
        public void FromConnectionPoint_PreservesConnectionPointSource()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var cp = indoorSpace.GetConnectionPointFromCellBoundary("cb12");

            Waypoint waypoint = Waypoint.FromConnectionPoint(cp, indoorSpace, 3.0f, 0.05f);

            Assert.AreSame(cp, waypoint.ConnectionPointSource);
            Assert.IsNull(waypoint.CellSpaceSource);
        }

        [Test]
        public void FromCellSpace_PreservesCellSpaceSource()
        {
            var indoorSpace = TestIndoorSpaceFactory.BuildDirectedCorridor();
            var cs = indoorSpace.GetCellSpaceFromId("c1");

            Waypoint waypoint = Waypoint.FromCellSpace(cs, 3.0f, 0.05f);

            Assert.AreSame(cs, waypoint.CellSpaceSource);
            Assert.IsNull(waypoint.ConnectionPointSource);
        }
    }
}
