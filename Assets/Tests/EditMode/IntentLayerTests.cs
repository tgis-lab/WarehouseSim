using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Map;

namespace EditModeTests
{
    public class IntentLayerTests
    {
        // === OccupancyIdentifier Intent Tests ===

        [Test]
        public void OccupancyIdentifier_Intent_AllowsNullPlanId()
        {
            var id = new OccupancyIdentifier("robot1", null, OccupancyType.Intent);
            Assert.AreEqual(OccupancyType.Intent, id.Type);
            Assert.IsNull(id.PlanId);
        }

        [Test]
        public void OccupancyIdentifier_Intent_StoresRobotId()
        {
            var id = new OccupancyIdentifier("robot1", null, OccupancyType.Intent);
            Assert.AreEqual("robot1", id.RobotId);
        }

        [Test]
        public void OccupancyIdentifier_ToString_ContainsIntent()
        {
            var id = new OccupancyIdentifier("robot1", null, OccupancyType.Intent);
            StringAssert.Contains("Intent", id.ToString());
        }

        [Test]
        public void OccupancyIdentifier_ExistingRealtime_StillWorks()
        {
            var id = new OccupancyIdentifier("robot1");
            Assert.AreEqual(OccupancyType.Realtime, id.Type);
        }

        [Test]
        public void OccupancyIdentifier_ExistingPlanned_StillWorks()
        {
            var id = new OccupancyIdentifier("robot1", "plan1", OccupancyType.Planned);
            Assert.AreEqual(OccupancyType.Planned, id.Type);
            Assert.AreEqual("plan1", id.PlanId);
        }

        // === TemporalIndex.RemoveByFilter ===

        [Test]
        public void TemporalIndex_RemoveByFilter_RemovesMatchingOnly()
        {
            var index = new TemporalIndex();
            var planned = new TimeOccupancy(0, 10, new OccupancyIdentifier("r1", "p1", OccupancyType.Planned));
            var intent = new TimeOccupancy(0, 30, new OccupancyIdentifier("r1", null, OccupancyType.Intent));
            index.AddInterval(planned);
            index.AddInterval(intent);

            index.RemoveByFilter(o => o.Occupier.Type == OccupancyType.Intent);

            var remaining = index.GetAllOccupancies();
            Assert.AreEqual(1, remaining.Count);
            Assert.AreEqual(OccupancyType.Planned, remaining[0].Occupier.Type);
        }

        [Test]
        public void TemporalIndex_RemoveByFilter_NoMatchDoesNothing()
        {
            var index = new TemporalIndex();
            index.AddInterval(new TimeOccupancy(0, 10, new OccupancyIdentifier("r1", "p1", OccupancyType.Planned)));

            index.RemoveByFilter(o => o.Occupier.Type == OccupancyType.Intent);

            Assert.AreEqual(1, index.GetAllOccupancies().Count);
        }

        // === DynamicOccupancyLayer.SetIntentOccupancy ===

        [Test]
        public void SetIntentOccupancy_CreatesIntentType()
        {
            var layer = new DynamicOccupancyLayer();
            layer.SetIntentOccupancy("cellA", 0f, 30f, "robot1");

            var occupancies = layer.QueryOverlapping("cellA", 0f, 30f);
            Assert.AreEqual(1, occupancies.Count);
            Assert.AreEqual(OccupancyType.Intent, occupancies[0].Occupier.Type);
        }

        [Test]
        public void SetIntentOccupancy_CoexistsWithPlanned()
        {
            var layer = new DynamicOccupancyLayer();
            layer.SetDynamicOccupancy("cellA", 0, 10, "r1", "plan1");
            layer.SetIntentOccupancy("cellA", 0, 30, "r1");

            var occupancies = layer.QueryOverlapping("cellA", 0, 30);
            Assert.AreEqual(2, occupancies.Count);
        }

        // === DynamicOccupancyLayer.ClearIntent ===

        [Test]
        public void ClearIntent_RemovesOnlyIntentForRobot()
        {
            var layer = new DynamicOccupancyLayer();
            layer.SetDynamicOccupancy("cellA", 0, 10, "r1", "plan1");
            layer.SetIntentOccupancy("cellA", 0, 30, "r1");
            layer.SetIntentOccupancy("cellB", 0, 30, "r2");

            layer.ClearIntent("r1");

            var cellA = layer.QueryOverlapping("cellA", 0, 30);
            Assert.AreEqual(1, cellA.Count);
            Assert.AreEqual(OccupancyType.Planned, cellA[0].Occupier.Type);

            var cellB = layer.QueryOverlapping("cellB", 0, 30);
            Assert.AreEqual(1, cellB.Count); // r2 intent untouched
        }

        // === DynamicOccupancyLayer.AtomicSwapPath ===

        [Test]
        public void AtomicSwapPath_ClearsOldAndRegistersNew()
        {
            var layer = new DynamicOccupancyLayer();
            layer.SetDynamicOccupancy("cellA", 0, 10, "r1", "oldPlan");
            layer.SetDynamicOccupancy("cellB", 5, 15, "r1", "oldPlan");
            layer.SetIntentOccupancy("cellC", 0, 30, "r1");
            layer.SetDynamicOccupancy("cellA", 20, 30, "r2", "r2plan"); // other robot

            long versionBefore = layer.Version;

            bool result = layer.AtomicSwapPath("r1", "oldPlan",
                new List<string>{"cellC", "cellD"},
                new List<float>{10, 15},
                new List<float>{20, 25},
                "newPlan");

            Assert.IsTrue(result);

            // Old r1 planned cleared
            var cellA_r1 = layer.QueryOverlapping("cellA", 0, 10)
                .Where(o => o.Occupier.RobotId == "r1").ToList();
            Assert.AreEqual(0, cellA_r1.Count);

            // Intent cleared
            var cellC_intent = layer.QueryOverlapping("cellC", 0, 30)
                .Where(o => o.Occupier.Type == OccupancyType.Intent).ToList();
            Assert.AreEqual(0, cellC_intent.Count);

            // New planned exists
            var cellC_new = layer.QueryOverlapping("cellC", 10, 20)
                .Where(o => o.Occupier.PlanId == "newPlan").ToList();
            Assert.AreEqual(1, cellC_new.Count);

            var cellD_new = layer.QueryOverlapping("cellD", 15, 25)
                .Where(o => o.Occupier.PlanId == "newPlan").ToList();
            Assert.AreEqual(1, cellD_new.Count);

            // r2 untouched
            var cellA_r2 = layer.QueryOverlapping("cellA", 20, 30)
                .Where(o => o.Occupier.RobotId == "r2").ToList();
            Assert.AreEqual(1, cellA_r2.Count);

            // Version incremented exactly once
            Assert.AreEqual(versionBefore + 1, layer.Version);
        }

        [Test]
        public void AtomicSwapPath_NullOldPlanId_SkipsClearOld()
        {
            var layer = new DynamicOccupancyLayer();
            // No old occupancy - first-time path
            bool result = layer.AtomicSwapPath("r1", null,
                new List<string>{"cellA"},
                new List<float>{0},
                new List<float>{10},
                "firstPlan");

            Assert.IsTrue(result);
            var cellA = layer.QueryOverlapping("cellA", 0, 10);
            Assert.AreEqual(1, cellA.Count);
        }

        // === DynamicOccupancyLayer.CheckCellConflict with excludeTypes ===

        [Test]
        public void CheckCellConflict_ExcludesIntentType()
        {
            var layer = new DynamicOccupancyLayer();
            layer.SetIntentOccupancy("cellA", 5, 15, "r2");

            var conflicts = layer.CheckCellConflict("cellA", 0, 20, "r1",
                excludeTypes: new[] { OccupancyType.Intent });
            Assert.AreEqual(0, conflicts.Count);
        }

        [Test]
        public void CheckCellConflict_WithoutExclude_IncludesAll()
        {
            var layer = new DynamicOccupancyLayer();
            layer.SetIntentOccupancy("cellA", 5, 15, "r2");

            var conflicts = layer.CheckCellConflict("cellA", 0, 20, "r1");
            Assert.IsTrue(conflicts.Count > 0);
        }

        // === DynamicOccupancyLayer.HasOccupancyOfType ===

        [Test]
        public void HasOccupancyOfType_TrueForExistingIntent()
        {
            var layer = new DynamicOccupancyLayer();
            layer.SetIntentOccupancy("cellA", 5, 15, "r2");

            Assert.IsTrue(layer.HasOccupancyOfType("cellA", 10f, OccupancyType.Intent));
        }

        [Test]
        public void HasOccupancyOfType_FalseForWrongType()
        {
            var layer = new DynamicOccupancyLayer();
            layer.SetIntentOccupancy("cellA", 5, 15, "r2");

            Assert.IsFalse(layer.HasOccupancyOfType("cellA", 10f, OccupancyType.Planned));
        }

        [Test]
        public void HasOccupancyOfType_FalseForWrongTime()
        {
            var layer = new DynamicOccupancyLayer();
            layer.SetIntentOccupancy("cellA", 5, 15, "r2");

            Assert.IsFalse(layer.HasOccupancyOfType("cellA", 20f, OccupancyType.Intent));
        }
    }
}
