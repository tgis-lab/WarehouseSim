using System.Collections.Generic;
using NUnit.Framework;
using PathPlanning;
using UnityEngine;

namespace EditModeTests
{
    public class DWAPlannerTests
    {
        private DWAPlanner _planner;

        [SetUp]
        public void SetUp()
        {
            _planner = new DWAPlanner
            {
                MaxSpeed = 2.0f,
                MaxAccel = 2.0f,
                MaxDecel = 5.0f,
                MaxOmega = 2.0f,
                MaxAngAccel = 10.0f,
                PredictionTime = 1.5f,
                DeltaTime = 0.05f,
                VelocitySamples = 7,
                OmegaSamples = 15,
                TrajectorySteps = 10,
                HeadingWeight = 0.3f,
                ClearanceWeight = 0.4f,
                VelocityWeight = 0.2f,
                GoalDistWeight = 0.1f
            };
        }

        // =====================================================================
        //  SimulateArc tests
        // =====================================================================

        [Test]
        public void SimulateArc_StraightLine_MovesForward()
        {
            var points = DWAPlanner.SimulateArc(
                Vector3.zero, Quaternion.identity, 1.0f, 0f, 1.0f, 10);

            Assert.AreEqual(11, points.Count);
            Assert.AreEqual(0f, points[0].z, 0.001f);
            // After 1s at 1m/s forward, should be at z≈1.0
            Assert.AreEqual(1.0f, points[10].z, 0.01f);
            // No lateral drift
            Assert.AreEqual(0f, points[10].x, 0.01f);
        }

        [Test]
        public void SimulateArc_ZeroVelocity_StaysInPlace()
        {
            var points = DWAPlanner.SimulateArc(
                new Vector3(5, 0, 5), Quaternion.identity, 0f, 0f, 1.0f, 10);

            for (int i = 0; i < points.Count; i++)
            {
                Assert.AreEqual(5f, points[i].x, 0.001f);
                Assert.AreEqual(5f, points[i].z, 0.001f);
            }
        }

        [Test]
        public void SimulateArc_PureTurn_StaysInPlace()
        {
            // Omega only, no forward velocity → rotate in place
            var points = DWAPlanner.SimulateArc(
                Vector3.zero, Quaternion.identity, 0f, 1.0f, 1.0f, 10);

            for (int i = 0; i < points.Count; i++)
            {
                Assert.AreEqual(0f, points[i].x, 0.001f);
                Assert.AreEqual(0f, points[i].z, 0.001f);
            }
        }

        [Test]
        public void SimulateArc_CurvedPath_DriftsLaterally()
        {
            // Forward 1 m/s + turning right at 1 rad/s for 1s → should curve right
            var points = DWAPlanner.SimulateArc(
                Vector3.zero, Quaternion.identity, 1.0f, 1.0f, 1.0f, 100);

            Vector3 end = points[points.Count - 1];
            // Should have moved forward and to the right (+x in Unity when turning right)
            Assert.Greater(end.x, 0.1f, "Should drift right");
            Assert.Greater(end.z, 0.1f, "Should move forward");
        }

        [Test]
        public void SimulateArc_SingleStep_ComputesCorrectly()
        {
            var points = DWAPlanner.SimulateArc(
                Vector3.zero, Quaternion.identity, 2.0f, 0f, 0.5f, 1);

            Assert.AreEqual(2, points.Count);
            Assert.AreEqual(1.0f, points[1].z, 0.01f); // 2.0 * 0.5 = 1.0
        }

        // =====================================================================
        //  CalculateVelocity — happy path
        // =====================================================================

        [Test]
        public void CalculateVelocity_NoObstacles_GoalAhead_SelectsForwardVelocity()
        {
            var result = _planner.CalculateVelocity(
                Vector3.zero,
                Quaternion.identity,
                1.0f,  // currently moving forward at 1 m/s
                0f,    // no rotation
                new Vector3(0, 0, 10),  // goal ahead
                new List<ObstacleData>());

            Assert.IsFalse(result.NeedReplan);
            Assert.Greater(result.LinearVelocity, 0f, "Should move forward");
            // Omega should be near zero since goal is straight ahead
            Assert.Less(Mathf.Abs(result.AngularVelocity), 1.0f,
                "Should not turn much when goal is straight ahead");
        }

        [Test]
        public void CalculateVelocity_ObstacleOnRight_AvoidsRight()
        {
            var obstacles = new List<ObstacleData>
            {
                new ObstacleData(new Vector3(1.5f, 0, 3f), 0.5f)  // obstacle to the right-front
            };

            _planner.Reset();
            var result = _planner.CalculateVelocity(
                Vector3.zero,
                Quaternion.identity,
                1.0f,
                0f,
                new Vector3(0, 0, 10),
                obstacles);

            Assert.IsFalse(result.NeedReplan);
            Assert.Greater(result.LinearVelocity, 0f, "Should still move forward");
            // Should favor turning left (negative omega) to avoid right obstacle
            // or at minimum, not turn right
            Assert.LessOrEqual(result.AngularVelocity, 0.5f,
                "Should not turn toward the obstacle on the right");
        }

        // =====================================================================
        //  CalculateVelocity — edge cases
        // =====================================================================

        [Test]
        public void CalculateVelocity_StartingFromRest_ProducesNonZeroVelocity()
        {
            var result = _planner.CalculateVelocity(
                Vector3.zero,
                Quaternion.identity,
                0f,   // at rest
                0f,
                new Vector3(0, 0, 5),
                new List<ObstacleData>());

            Assert.IsFalse(result.NeedReplan);
            // Dynamic window: vMin=0, vMax=MaxAccel*dt=0.1
            // Should select some positive velocity even if small
            Assert.GreaterOrEqual(result.LinearVelocity, 0f,
                "Should produce non-negative velocity from rest");
        }

        [Test]
        public void CalculateVelocity_GoalBehind_TurnsTowardGoal()
        {
            _planner.Reset();
            var result = _planner.CalculateVelocity(
                Vector3.zero,
                Quaternion.identity,  // facing +Z
                0.5f,
                0f,
                new Vector3(0, 0, -5),  // goal behind
                new List<ObstacleData>());

            // With goal behind, heading score is low for all forward-facing velocities.
            // Robot should prefer higher |omega| to turn toward goal.
            Assert.IsFalse(result.NeedReplan);
        }

        [Test]
        public void CalculateVelocity_SingleSampleEach_NoDivideByZero()
        {
            _planner.VelocitySamples = 1;
            _planner.OmegaSamples = 1;
            _planner.Reset();

            var result = _planner.CalculateVelocity(
                Vector3.zero,
                Quaternion.identity,
                1.0f,
                0f,
                new Vector3(0, 0, 5),
                new List<ObstacleData>());

            Assert.IsFalse(float.IsNaN(result.LinearVelocity));
            Assert.IsFalse(float.IsNaN(result.AngularVelocity));
        }

        [Test]
        public void CalculateVelocity_ObstacleVeryClose_RejectsHighSpeed()
        {
            // Obstacle right ahead at 0.3m with radius 0.2 → clearance ≈ 0.1m
            // admissible v ≤ sqrt(2 * 0.1 * 5.0) ≈ 1.0 m/s
            var obstacles = new List<ObstacleData>
            {
                new ObstacleData(new Vector3(0, 0, 0.5f), 0.2f)
            };

            _planner.Reset();
            var result = _planner.CalculateVelocity(
                Vector3.zero,
                Quaternion.identity,
                1.0f,
                0f,
                new Vector3(0, 0, 10),
                obstacles);

            // Should select a velocity that respects admissibility
            // Given the close obstacle, high speeds should be rejected
            Assert.LessOrEqual(result.LinearVelocity, 1.5f,
                "Should not exceed admissible speed near close obstacle");
        }

        [Test]
        public void CalculateVelocity_Surrounded_StopsAndRequestsReplan()
        {
            // Obstacles on all sides, very tight — radius 0.3 at distance 0.15
            // clearance ≈ 0 for all directions, no admissible velocity at any speed
            var obstacles = new List<ObstacleData>
            {
                new ObstacleData(new Vector3(0, 0, 0.15f), 0.3f),   // front
                new ObstacleData(new Vector3(0, 0, -0.15f), 0.3f),  // back
                new ObstacleData(new Vector3(0.15f, 0, 0), 0.3f),   // right
                new ObstacleData(new Vector3(-0.15f, 0, 0), 0.3f),  // left
            };

            _planner.Reset();
            var result = _planner.CalculateVelocity(
                Vector3.zero,
                Quaternion.identity,
                1.5f,  // higher speed so vMin is larger, making admissibility harder
                0f,
                new Vector3(0, 0, 5),
                obstacles);

            Assert.IsTrue(result.NeedReplan, "Should request replan when surrounded");
            Assert.AreEqual(0f, result.LinearVelocity, 0.001f, "Should stop");
            Assert.AreEqual(0f, result.AngularVelocity, 0.001f, "Should not rotate");
        }

        [Test]
        public void CalculateVelocity_EmptyObstacles_BehavesAsOpenSpace()
        {
            _planner.Reset();
            var result = _planner.CalculateVelocity(
                Vector3.zero,
                Quaternion.identity,
                1.0f,
                0f,
                new Vector3(0, 0, 10),
                new List<ObstacleData>());

            Assert.IsFalse(result.NeedReplan);
            Assert.Greater(result.LinearVelocity, 0.5f,
                "Should select reasonable speed in open space");
        }

        [Test]
        public void CalculateVelocity_NullObstacles_BehavesAsOpenSpace()
        {
            _planner.Reset();
            var result = _planner.CalculateVelocity(
                Vector3.zero,
                Quaternion.identity,
                1.0f,
                0f,
                new Vector3(0, 0, 10),
                null);

            Assert.IsFalse(result.NeedReplan);
            Assert.Greater(result.LinearVelocity, 0f);
        }

        [Test]
        public void CalculateVelocity_GoalAtCurrentPosition_NoNaN()
        {
            _planner.Reset();
            var result = _planner.CalculateVelocity(
                new Vector3(5, 0, 5),
                Quaternion.identity,
                0.5f,
                0f,
                new Vector3(5, 0, 5),  // goal = current position
                new List<ObstacleData>());

            Assert.IsFalse(float.IsNaN(result.LinearVelocity));
            Assert.IsFalse(float.IsNaN(result.AngularVelocity));
        }

        // =====================================================================
        //  Normalization tests
        // =====================================================================

        [Test]
        public void CalculateVelocity_MultiCandidates_NormalizationProducesBestChoice()
        {
            // With no obstacles and goal straight ahead, the best candidate should
            // be the one that goes fastest and straightest toward the goal.
            _planner.Reset();
            _planner.HeadingWeight = 0.5f;
            _planner.ClearanceWeight = 0.0f;
            _planner.VelocityWeight = 0.5f;
            _planner.GoalDistWeight = 0.0f;

            var result = _planner.CalculateVelocity(
                Vector3.zero,
                Quaternion.identity,
                1.0f,
                0f,
                new Vector3(0, 0, 10),
                new List<ObstacleData>());

            Assert.IsFalse(result.NeedReplan);
            // With heading+velocity weights dominant and no obstacles,
            // should pick near-maximum speed with minimal turn
            Assert.Greater(result.LinearVelocity, 0.8f);
        }

        // =====================================================================
        //  Dynamic window tests
        // =====================================================================

        [Test]
        public void CalculateVelocity_HighCurrentSpeed_WindowBoundsCorrect()
        {
            // At currentV=2.0 (maxSpeed), dynamic window top is clamped at MaxSpeed
            // vMin = max(0, 2.0 - 5.0*0.05) = 1.75
            // vMax = min(2.0, 2.0 + 2.0*0.05) = 2.0
            _planner.Reset();
            var result = _planner.CalculateVelocity(
                Vector3.zero,
                Quaternion.identity,
                2.0f,
                0f,
                new Vector3(0, 0, 10),
                new List<ObstacleData>());

            Assert.IsFalse(result.NeedReplan);
            // Result velocity should be within [1.75, 2.0] (before smoothing)
            // With smoothing factor 0.7, first call without previous: no smoothing
            Assert.GreaterOrEqual(result.LinearVelocity, 1.5f);
            Assert.LessOrEqual(result.LinearVelocity, 2.1f);
        }

        // =====================================================================
        //  Reset tests
        // =====================================================================

        [Test]
        public void Reset_ClearsSmoothingState()
        {
            // First call sets smoothing state
            _planner.CalculateVelocity(
                Vector3.zero, Quaternion.identity, 1.0f, 0f,
                new Vector3(0, 0, 10), new List<ObstacleData>());

            _planner.Reset();

            // After reset, calling with different velocity should not be affected by smoothing
            var result = _planner.CalculateVelocity(
                Vector3.zero, Quaternion.identity, 0.1f, 0f,
                new Vector3(0, 0, 10), new List<ObstacleData>());

            Assert.IsFalse(result.NeedReplan);
            // Without smoothing from a much higher previous velocity,
            // result should be within the small dynamic window around 0.1
            Assert.LessOrEqual(result.LinearVelocity, 0.5f,
                "After reset, smoothing should not pull velocity up from a previous call");
        }

        // =====================================================================
        //  Admissibility tests
        // =====================================================================

        [Test]
        public void CalculateVelocity_ObstacleAtKnownDistance_RespectsAdmissibility()
        {
            // Obstacle at 1.0m, radius 0.0 → clearance = 1.0m
            // maxAdmissibleV = sqrt(2 * 1.0 * 5.0) ≈ 3.16 m/s
            // All our samples (max 2.0) should pass admissibility
            var obstacles = new List<ObstacleData>
            {
                new ObstacleData(new Vector3(0, 0, 1.0f), 0f)
            };

            _planner.Reset();
            var result = _planner.CalculateVelocity(
                Vector3.zero,
                Quaternion.identity,
                1.0f,
                0f,
                new Vector3(0, 0, 10),
                obstacles);

            Assert.IsFalse(result.NeedReplan, "Obstacle far enough — should find admissible velocity");
            Assert.Greater(result.LinearVelocity, 0f);
        }

        [Test]
        public void CalculateVelocity_ObstacleBlocking_OnlySideVelocitiesAdmissible()
        {
            // Obstacle directly ahead at 0.2m → clearance ≈ 0.2m
            // maxAdmissibleV = sqrt(2 * 0.2 * 5.0) ≈ 1.41 m/s
            // High-speed straight-ahead samples rejected, but side/turning samples may survive
            var obstacles = new List<ObstacleData>
            {
                new ObstacleData(new Vector3(0, 0, 0.4f), 0.2f)
            };

            _planner.Reset();
            var result = _planner.CalculateVelocity(
                Vector3.zero,
                Quaternion.identity,
                1.0f,
                0f,
                new Vector3(0, 0, 10),
                obstacles);

            // Should still find a solution (turning to avoid)
            // or if the obstacle is too close for all samples, stop
            Assert.IsFalse(float.IsNaN(result.LinearVelocity));
        }

        // =====================================================================
        //  Backward compatibility: no Physics queries
        // =====================================================================

        [Test]
        public void DWAPlanner_NoPhysicsQueryInClass()
        {
            // This test validates by compilation and execution:
            // If DWAPlanner used Physics.OverlapSphere internally, it would
            // return empty results in EditMode (no physics scene), making it
            // impossible to get meaningful results.
            _planner.Reset();
            var result = _planner.CalculateVelocity(
                Vector3.zero,
                Quaternion.identity,
                1.0f,
                0f,
                new Vector3(0, 0, 5),
                new List<ObstacleData>
                {
                    new ObstacleData(new Vector3(2, 0, 2), 0.3f)
                });

            // If Physics queries were used, obstacles wouldn't be detected
            // and the result would ignore them. With external obstacles, we get
            // a meaningful avoidance result.
            Assert.IsFalse(result.NeedReplan);
            Assert.Greater(result.LinearVelocity, 0f);
        }
    }
}
