using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Map;

namespace EditModeTests
{
    /// <summary>
    /// Comprehensive Edit Mode tests for TemporalIndex and DynamicOccupancyLayer.
    /// Covers core add/remove/query operations, conflict detection, version tracking,
    /// merge, prune, and multi-robot coexistence.
    ///
    /// Tests already covered by IntentLayerTests.cs are NOT duplicated here:
    ///   - SetIntentOccupancy, ClearIntent, AtomicSwapPath
    ///   - CheckCellConflict excludeTypes, HasOccupancyOfType
    ///   - RemoveByFilter (matching / no-match)
    /// </summary>
    public class TemporalOccupancyTests
    {
        // =====================================================================
        // TemporalIndex Tests
        // =====================================================================

        #region TemporalIndex: AddInterval + GetOccupancies

        [Test]
        public void TemporalIndex_AddInterval_GetOccupancies_ReturnsWithinRange()
        {
            var index = new TemporalIndex();
            var occ = new TimeOccupancy(5f, 15f, new OccupancyIdentifier("r1"));
            index.AddInterval(occ);

            var results = index.GetOccupancies(0f, 20f);

            Assert.AreEqual(1, results.Count);
            Assert.AreEqual(occ.Id, results[0].Id);
        }

        [Test]
        public void TemporalIndex_GetOccupancies_ExcludesOutOfRange()
        {
            var index = new TemporalIndex();
            var inRange = new TimeOccupancy(5f, 15f, new OccupancyIdentifier("r1"));
            var outOfRange = new TimeOccupancy(50f, 60f, new OccupancyIdentifier("r2"));
            index.AddInterval(inRange);
            index.AddInterval(outOfRange);

            var results = index.GetOccupancies(0f, 20f);

            Assert.AreEqual(1, results.Count);
            Assert.AreEqual(inRange.Id, results[0].Id);
        }

        #endregion

        #region TemporalIndex: RemoveInterval

        [Test]
        public void TemporalIndex_RemoveInterval_ActuallyRemoves()
        {
            var index = new TemporalIndex();
            var occ = new TimeOccupancy(0f, 10f, new OccupancyIdentifier("r1"));
            index.AddInterval(occ);

            index.RemoveInterval(occ);

            var results = index.GetAllOccupancies();
            Assert.AreEqual(0, results.Count);
        }

        [Test]
        public void TemporalIndex_RemoveInterval_DoesNotAffectOthers()
        {
            var index = new TemporalIndex();
            var occ1 = new TimeOccupancy(0f, 10f, new OccupancyIdentifier("r1"));
            var occ2 = new TimeOccupancy(5f, 15f, new OccupancyIdentifier("r2"));
            index.AddInterval(occ1);
            index.AddInterval(occ2);

            index.RemoveInterval(occ1);

            var results = index.GetAllOccupancies();
            Assert.AreEqual(1, results.Count);
            Assert.AreEqual(occ2.Id, results[0].Id);
        }

        #endregion

        #region TemporalIndex: GetAllOccupancies

        [Test]
        public void TemporalIndex_GetAllOccupancies_ReturnsEverything()
        {
            var index = new TemporalIndex();
            var occ1 = new TimeOccupancy(0f, 10f, new OccupancyIdentifier("r1"));
            var occ2 = new TimeOccupancy(20f, 30f, new OccupancyIdentifier("r2"));
            var occ3 = new TimeOccupancy(100f, 200f, new OccupancyIdentifier("r3"));
            index.AddInterval(occ1);
            index.AddInterval(occ2);
            index.AddInterval(occ3);

            var results = index.GetAllOccupancies();

            Assert.AreEqual(3, results.Count);
            var ids = results.Select(r => r.Id).ToList();
            Assert.Contains(occ1.Id, ids);
            Assert.Contains(occ2.Id, ids);
            Assert.Contains(occ3.Id, ids);
        }

        #endregion

        #region TemporalIndex: CheckConflict

        [Test]
        public void TemporalIndex_CheckConflict_DetectsOverlappingFromDifferentRobots()
        {
            var index = new TemporalIndex();
            // Robot r2 occupies [5, 15]
            var existing = new TimeOccupancy(5f, 15f, new OccupancyIdentifier("r2"));
            index.AddInterval(existing);

            // Check conflict for r1 in [10, 20] -- overlaps with r2's [5, 15]
            var conflicts = index.CheckConflict(10f, 20f, excludeRobotId: "r1");

            Assert.AreEqual(1, conflicts.Count);
            Assert.AreEqual("r2", conflicts[0].Occupier.RobotId);
        }

        [Test]
        public void TemporalIndex_CheckConflict_ExcludesSameRobotOccupancies()
        {
            var index = new TemporalIndex();
            // Robot r1 already occupies [5, 15]
            var ownOcc = new TimeOccupancy(5f, 15f, new OccupancyIdentifier("r1"));
            index.AddInterval(ownOcc);

            // Check conflict for r1 in [10, 20] -- should exclude own occupancy
            var conflicts = index.CheckConflict(10f, 20f, excludeRobotId: "r1");

            Assert.AreEqual(0, conflicts.Count);
        }

        [Test]
        public void TemporalIndex_CheckConflict_NoConflictWhenNoOverlap()
        {
            var index = new TemporalIndex();
            var existing = new TimeOccupancy(0f, 5f, new OccupancyIdentifier("r2"));
            index.AddInterval(existing);

            // Check conflict for r1 in [10, 20] -- no overlap with [0, 5]
            var conflicts = index.CheckConflict(10f, 20f, excludeRobotId: "r1");

            Assert.AreEqual(0, conflicts.Count);
        }

        #endregion

        #region TemporalIndex: PruneExpired (RemoveIntervalsBefore)

        [Test]
        public void TemporalIndex_RemoveIntervalsBefore_RemovesOldEntries()
        {
            var index = new TemporalIndex();
            var old = new TimeOccupancy(0f, 5f, new OccupancyIdentifier("r1"));
            var recent = new TimeOccupancy(20f, 30f, new OccupancyIdentifier("r2"));
            index.AddInterval(old);
            index.AddInterval(recent);

            // Remove everything that ended at or before time 10
            index.RemoveIntervalsBefore(10f);

            var results = index.GetAllOccupancies();
            Assert.AreEqual(1, results.Count);
            Assert.AreEqual(recent.Id, results[0].Id);
        }

        [Test]
        public void TemporalIndex_RemoveIntervalsBefore_KeepsStillActiveEntries()
        {
            var index = new TemporalIndex();
            // This occupancy spans across the cutoff time
            var spanning = new TimeOccupancy(5f, 25f, new OccupancyIdentifier("r1"));
            index.AddInterval(spanning);

            // Remove everything that ended at or before time 10
            index.RemoveIntervalsBefore(10f);

            // The spanning occupancy ends at 25, so it should survive
            var results = index.GetAllOccupancies();
            Assert.AreEqual(1, results.Count);
        }

        #endregion

        // =====================================================================
        // DynamicOccupancyLayer Tests
        // =====================================================================

        #region DynamicOccupancyLayer: SetDynamicOccupancy + QueryOverlapping roundtrip

        [Test]
        public void Layer_SetDynamicOccupancy_QueryOverlapping_Roundtrip()
        {
            var layer = new DynamicOccupancyLayer();
            layer.SetDynamicOccupancy("cell1", 0f, 10f, "r1", "plan1");

            var results = layer.QueryOverlapping("cell1", 0f, 10f);

            Assert.AreEqual(1, results.Count);
            Assert.AreEqual("r1", results[0].Occupier.RobotId);
            Assert.AreEqual("plan1", results[0].Occupier.PlanId);
            Assert.AreEqual(OccupancyType.Planned, results[0].Occupier.Type);
        }

        [Test]
        public void Layer_SetDynamicOccupancy_WithoutPlanId_CreatesRealtimeType()
        {
            var layer = new DynamicOccupancyLayer();
            layer.SetDynamicOccupancy("cell1", 0f, 10f, "r1");

            var results = layer.QueryOverlapping("cell1", 0f, 10f);

            Assert.AreEqual(1, results.Count);
            Assert.AreEqual(OccupancyType.Realtime, results[0].Occupier.Type);
        }

        #endregion

        #region DynamicOccupancyLayer: Version increments

        [Test]
        public void Layer_Version_IncrementsOnEachModification()
        {
            var layer = new DynamicOccupancyLayer();
            long v0 = layer.Version;

            layer.SetDynamicOccupancy("cell1", 0f, 10f, "r1", "p1");
            long v1 = layer.Version;

            layer.SetDynamicOccupancy("cell2", 5f, 15f, "r2", "p2");
            long v2 = layer.Version;

            Assert.AreEqual(v0 + 1, v1);
            Assert.AreEqual(v0 + 2, v2);
        }

        [Test]
        public void Layer_Version_IncrementsOnClearPlannedOccupancy()
        {
            var layer = new DynamicOccupancyLayer();
            layer.SetDynamicOccupancy("cell1", 0f, 10f, "r1", "plan1");
            long vBefore = layer.Version;

            layer.ClearPlannedOccupancy("r1", "plan1", 0f);

            Assert.Greater(layer.Version, vBefore);
        }

        [Test]
        public void Layer_Version_IncrementsOnRemoveOccupanciesByRobot()
        {
            var layer = new DynamicOccupancyLayer();
            layer.SetDynamicOccupancy("cell1", 0f, 10f, "r1", "p1");
            long vBefore = layer.Version;

            layer.RemoveOccupanciesByRobot("r1");

            Assert.Greater(layer.Version, vBefore);
        }

        #endregion

        #region DynamicOccupancyLayer: ClearPlannedOccupancy

        [Test]
        public void Layer_ClearPlannedOccupancy_RemovesCorrectEntries()
        {
            var layer = new DynamicOccupancyLayer();
            layer.SetDynamicOccupancy("cell1", 0f, 10f, "r1", "plan1");
            layer.SetDynamicOccupancy("cell1", 15f, 25f, "r1", "plan1");
            layer.SetDynamicOccupancy("cell1", 0f, 10f, "r2", "plan2");

            // Clear r1's plan1 from time 0 onwards
            layer.ClearPlannedOccupancy("r1", "plan1", 0f);

            var results = layer.QueryOverlapping("cell1", 0f, 30f);
            // Only r2's occupancy should remain
            Assert.AreEqual(1, results.Count);
            Assert.AreEqual("r2", results[0].Occupier.RobotId);
        }

        [Test]
        public void Layer_ClearPlannedOccupancy_PreservesPastHistory()
        {
            var layer = new DynamicOccupancyLayer();
            // Past occupancy (ends before currentTime)
            layer.SetDynamicOccupancy("cell1", 0f, 5f, "r1", "plan1");
            // Future occupancy (starts at or after currentTime)
            layer.SetDynamicOccupancy("cell1", 10f, 20f, "r1", "plan1");

            // Clear from time 8 onwards -- only the future one should be removed
            layer.ClearPlannedOccupancy("r1", "plan1", 8f);

            var results = layer.QueryOverlapping("cell1", 0f, 30f);
            // Past occupancy at [0, 5] should remain since it ended before currentTime=8
            // and QueryOverlapping(8, MaxValue) won't find it
            Assert.AreEqual(1, results.Count);
            Assert.AreEqual(0f, results[0].StartTime, 0.001f);
            Assert.AreEqual(5f, results[0].EndTime, 0.001f);
        }

        #endregion

        #region DynamicOccupancyLayer: MergeLayer

        [Test]
        public void Layer_MergeLayer_CopiesFromSourceToTarget()
        {
            var source = new DynamicOccupancyLayer();
            source.SetDynamicOccupancy("cell1", 0f, 10f, "r1", "p1");
            source.SetDynamicOccupancy("cell2", 5f, 15f, "r1", "p1");

            var target = new DynamicOccupancyLayer();

            target.MergeLayer(source, 0f, 20f);

            var cell1Results = target.QueryOverlapping("cell1", 0f, 20f);
            var cell2Results = target.QueryOverlapping("cell2", 0f, 20f);
            Assert.AreEqual(1, cell1Results.Count);
            Assert.AreEqual(1, cell2Results.Count);
            Assert.AreEqual("r1", cell1Results[0].Occupier.RobotId);
        }

        [Test]
        public void Layer_MergeLayer_WithRobotId_ClearsStaleEntriesFirst()
        {
            var target = new DynamicOccupancyLayer();
            // Pre-existing stale entry for r1
            target.SetDynamicOccupancy("cell1", 0f, 5f, "r1");

            var source = new DynamicOccupancyLayer();
            // Updated entry for r1
            source.SetDynamicOccupancy("cell1", 10f, 20f, "r1");

            target.MergeLayer(source, 0f, 30f, robotId: "r1");

            var results = target.QueryOverlapping("cell1", 0f, 30f);
            // Stale entry should be gone, only updated entry remains
            Assert.AreEqual(1, results.Count);
            Assert.AreEqual(10f, results[0].StartTime, 0.001f);
        }

        [Test]
        public void Layer_MergeLayer_PreservesOtherRobots()
        {
            var target = new DynamicOccupancyLayer();
            target.SetDynamicOccupancy("cell1", 0f, 5f, "r2", "p2");

            var source = new DynamicOccupancyLayer();
            source.SetDynamicOccupancy("cell1", 10f, 20f, "r1");

            target.MergeLayer(source, 0f, 30f, robotId: "r1");

            var results = target.QueryOverlapping("cell1", 0f, 30f);
            // Both r2's existing entry and r1's merged entry should be present
            Assert.AreEqual(2, results.Count);
            var robotIds = results.Select(r => r.Occupier.RobotId).ToList();
            Assert.Contains("r1", robotIds);
            Assert.Contains("r2", robotIds);
        }

        #endregion

        #region DynamicOccupancyLayer: RemoveOccupanciesByRobot

        [Test]
        public void Layer_RemoveOccupanciesByRobot_RemovesAllForRobot()
        {
            var layer = new DynamicOccupancyLayer();
            layer.SetDynamicOccupancy("cell1", 0f, 10f, "r1", "p1");
            layer.SetDynamicOccupancy("cell2", 5f, 15f, "r1", "p2");
            layer.SetDynamicOccupancy("cell1", 20f, 30f, "r2", "p3");

            layer.RemoveOccupanciesByRobot("r1");

            var cell1 = layer.QueryOverlapping("cell1", 0f, 40f);
            var cell2 = layer.QueryOverlapping("cell2", 0f, 40f);
            // r1 entries should be gone from both cells
            Assert.AreEqual(1, cell1.Count);
            Assert.AreEqual("r2", cell1[0].Occupier.RobotId);
            Assert.AreEqual(0, cell2.Count);
        }

        [Test]
        public void Layer_RemoveOccupanciesByRobot_NullOrEmptyDoesNothing()
        {
            var layer = new DynamicOccupancyLayer();
            layer.SetDynamicOccupancy("cell1", 0f, 10f, "r1");
            long vBefore = layer.Version;

            layer.RemoveOccupanciesByRobot(null);
            layer.RemoveOccupanciesByRobot("");

            // Version should not change, data intact
            Assert.AreEqual(vBefore, layer.Version);
            Assert.AreEqual(1, layer.QueryOverlapping("cell1", 0f, 10f).Count);
        }

        #endregion

        #region DynamicOccupancyLayer: PruneExpiredOccupancies

        [Test]
        public void Layer_PruneExpiredOccupancies_RemovesExpiredEntries()
        {
            var layer = new DynamicOccupancyLayer();
            layer.SetDynamicOccupancy("cell1", 0f, 5f, "r1");
            layer.SetDynamicOccupancy("cell1", 20f, 30f, "r2");

            // Prune everything ending before time 10
            layer.PruneExpiredOccupancies(10f);

            var results = layer.QueryOverlapping("cell1", 0f, 40f);
            Assert.AreEqual(1, results.Count);
            Assert.AreEqual("r2", results[0].Occupier.RobotId);
        }

        [Test]
        public void Layer_PruneExpiredOccupancies_IncrementsVersion()
        {
            var layer = new DynamicOccupancyLayer();
            layer.SetDynamicOccupancy("cell1", 0f, 5f, "r1");
            long vBefore = layer.Version;

            layer.PruneExpiredOccupancies(10f);

            Assert.Greater(layer.Version, vBefore);
        }

        #endregion

        #region DynamicOccupancyLayer: Multiple robots coexist

        [Test]
        public void Layer_MultipleRobots_OccupanciesCoexistCorrectly()
        {
            var layer = new DynamicOccupancyLayer();
            layer.SetDynamicOccupancy("cell1", 0f, 10f, "r1", "p1");
            layer.SetDynamicOccupancy("cell1", 5f, 15f, "r2", "p2");
            layer.SetDynamicOccupancy("cell1", 10f, 20f, "r3", "p3");

            var allInCell = layer.QueryOverlapping("cell1", 0f, 20f);
            Assert.AreEqual(3, allInCell.Count);

            var robotIds = allInCell.Select(o => o.Occupier.RobotId).ToList();
            Assert.Contains("r1", robotIds);
            Assert.Contains("r2", robotIds);
            Assert.Contains("r3", robotIds);
        }

        [Test]
        public void Layer_MultipleRobots_ConflictDetectionIsPerRobot()
        {
            var layer = new DynamicOccupancyLayer();
            layer.SetDynamicOccupancy("cell1", 0f, 10f, "r1", "p1");
            layer.SetDynamicOccupancy("cell1", 5f, 15f, "r2", "p2");
            layer.SetDynamicOccupancy("cell1", 8f, 12f, "r3", "p3");

            // Check conflict for r1 -- should detect r2 and r3 but not itself
            var conflicts = layer.CheckCellConflict("cell1", 0f, 10f, "r1");
            var conflictRobots = conflicts.Select(c => c.Occupier.RobotId).ToList();

            Assert.IsFalse(conflictRobots.Contains("r1"));
            Assert.IsTrue(conflictRobots.Contains("r2"));
            Assert.IsTrue(conflictRobots.Contains("r3"));
        }

        #endregion

        #region DynamicOccupancyLayer: GetCellTimeIndex auto-creates

        [Test]
        public void Layer_GetCellTimeIndex_AutoCreatesForNewCells()
        {
            var layer = new DynamicOccupancyLayer();

            // Query a cell that was never explicitly registered
            var results = layer.QueryOverlapping("brand_new_cell", 0f, 10f);

            // Should return empty list (not throw), and the index is created internally
            Assert.IsNotNull(results);
            Assert.AreEqual(0, results.Count);

            // Now set occupancy on that cell and verify it works
            layer.SetDynamicOccupancy("brand_new_cell", 0f, 10f, "r1");
            var afterSet = layer.QueryOverlapping("brand_new_cell", 0f, 10f);
            Assert.AreEqual(1, afterSet.Count);
        }

        [Test]
        public void Layer_QueryOverlapping_DifferentCellsAreIndependent()
        {
            var layer = new DynamicOccupancyLayer();
            layer.SetDynamicOccupancy("cellA", 0f, 10f, "r1");
            layer.SetDynamicOccupancy("cellB", 0f, 10f, "r2");

            var cellA = layer.QueryOverlapping("cellA", 0f, 10f);
            var cellB = layer.QueryOverlapping("cellB", 0f, 10f);

            Assert.AreEqual(1, cellA.Count);
            Assert.AreEqual("r1", cellA[0].Occupier.RobotId);
            Assert.AreEqual(1, cellB.Count);
            Assert.AreEqual("r2", cellB[0].Occupier.RobotId);
        }

        #endregion

        // =====================================================================
        // P0: Occupancy ID Mismatch Tests
        // =====================================================================

        #region P0: Occupancy ID Mismatch Bug

        [Test]
        public void OccupancyLayer_StoredByCellSpaceId_QueryByCPId_ReturnsMismatch()
        {
            // This test demonstrates the bug: occupancy stored by CellSpace ID
            // but queried by ConnectionPoint ID returns nothing
            var layer = new DynamicOccupancyLayer();

            // Store occupancy using CellSpace ID (how RecordPathOccupancy works internally)
            layer.SetDynamicOccupancy("cellA", 0, 10, "robot2", "plan2");

            // Query using a ConnectionPoint ID (how SIPP/TSA* query)
            // ConnectionPoint IDs are typically CellBoundary IDs like "cb1", not "cellA"
            var result = layer.QueryOverlapping("cb1", 0, 10);

            // BUG: This returns 0 because "cb1" != "cellA"
            // After fix, the algorithms should resolve CP ID → CellSpace ID before querying
            Assert.AreEqual(0, result.Count,
                "Demonstrates the bug: querying by CP ID finds nothing when stored by CellSpace ID");
        }

        [Test]
        public void OccupancyLayer_QueryByCellSpaceId_ReturnsCorrectly()
        {
            // This test shows the correct behavior: query by CellSpace ID works
            var layer = new DynamicOccupancyLayer();
            layer.SetDynamicOccupancy("cellA", 0, 10, "robot2", "plan2");

            var result = layer.QueryOverlapping("cellA", 0, 10);
            Assert.AreEqual(1, result.Count, "Query by CellSpace ID should find the occupancy");
        }

        #endregion

        // =====================================================================
        // TemporalIndex: Capacity Limit
        // =====================================================================

        #region TemporalIndex: Capacity Limit

        [Test]
        public void TemporalIndex_AddInterval_EnforcesCapacityLimit()
        {
            var index = new TemporalIndex();

            // Add more than MAX_OCCUPANCY_COUNT intervals
            int overLimit = TemporalIndex.MAX_OCCUPANCY_COUNT + 100;
            for (int i = 0; i < overLimit; i++)
            {
                var occ = new TimeOccupancy(i, i + 1f, new OccupancyIdentifier($"r{i}"));
                index.AddInterval(occ);
            }

            var all = index.GetAllOccupancies();
            Assert.LessOrEqual(all.Count, TemporalIndex.MAX_OCCUPANCY_COUNT,
                "Occupancy count should not exceed MAX_OCCUPANCY_COUNT after cleanup");
        }

        #endregion

        // =====================================================================
        // DynamicOccupancyLayer: CheckCellConflictWithVersion
        // =====================================================================

        #region DynamicOccupancyLayer: CheckCellConflictWithVersion

        [Test]
        public void Layer_CheckCellConflictWithVersion_CapturesVersionAtQueryTime()
        {
            var layer = new DynamicOccupancyLayer();
            layer.SetDynamicOccupancy("cell1", 0f, 10f, "r1", "p1");
            long versionBefore = layer.Version;

            var (conflicts, capturedVersion) = layer.CheckCellConflictWithVersion("cell1", 0f, 10f, "r2");

            Assert.AreEqual(versionBefore, capturedVersion,
                "Captured version should match the version at query time");
            Assert.AreEqual(1, conflicts.Count,
                "Should detect conflict with r1's occupancy");
        }

        [Test]
        public void Layer_CheckCellConflictWithVersion_VersionChangesAfterModification()
        {
            var layer = new DynamicOccupancyLayer();
            layer.SetDynamicOccupancy("cell1", 0f, 10f, "r1", "p1");

            var (_, v1) = layer.CheckCellConflictWithVersion("cell1", 0f, 10f, "r2");

            layer.SetDynamicOccupancy("cell1", 5f, 15f, "r3", "p3");

            var (_, v2) = layer.CheckCellConflictWithVersion("cell1", 0f, 10f, "r2");

            Assert.Greater(v2, v1,
                "Version should be higher after additional modification");
        }

        #endregion

        // =====================================================================
        // DynamicOccupancyLayer: UpdateOccupancyEndTime entryTime match
        // =====================================================================

        #region DynamicOccupancyLayer: UpdateOccupancyEndTime entryTime match

        [Test]
        public void Layer_UpdateOccupancyEndTime_ReentryCreatesNewInterval()
        {
            var layer = new DynamicOccupancyLayer();

            // First entry at time 0
            layer.UpdateOccupancyEndTime("cell1", "r1", 5f, 0f);

            // Re-entry at time 100 (well beyond proximity threshold)
            layer.UpdateOccupancyEndTime("cell1", "r1", 105f, 100f);

            var results = layer.QueryOverlapping("cell1", 0f, 200f);
            var r1Results = results.FindAll(o => o.Occupier.RobotId == "r1");

            Assert.AreEqual(2, r1Results.Count,
                "Re-entering same cell should create a new interval, not extend the old one");
        }

        [Test]
        public void Layer_UpdateOccupancyEndTime_SameEntryExtendsExisting()
        {
            var layer = new DynamicOccupancyLayer();

            // First entry at time 10
            layer.UpdateOccupancyEndTime("cell1", "r1", 12f, 10f);

            // Update end time with same entry time (within proximity)
            layer.UpdateOccupancyEndTime("cell1", "r1", 15f, 10.5f);

            var results = layer.QueryOverlapping("cell1", 0f, 200f);
            var r1Results = results.FindAll(o => o.Occupier.RobotId == "r1");

            Assert.AreEqual(1, r1Results.Count,
                "Updating with same entry time should extend existing interval");
            Assert.AreEqual(15f, r1Results[0].EndTime, 0.001f);
        }

        #endregion
    }
}
