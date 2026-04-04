using NUnit.Framework;
using PathPlanning;
using UnityEngine;

namespace EditModeTests
{
    public class DWAPlannerTests
    {
        // ===================================================================
        //  W4-T1: DWA sample count clamping — VelocitySamples=1 / AngleSamples=1
        // ===================================================================

        [Test]
        public void CalculateVelocity_VelocitySamplesOne_DoesNotThrow()
        {
            var planner = new DWAPlanner
            {
                VelocitySamples = 1,
                AngleSamples = 5,
                MaxSpeed = 2.0f,
                MinSpeed = 0.5f,
                MaxRotSpeed = 0.8f,
                PredictionTime = 0.5f
            };

            Assert.DoesNotThrow(() =>
            {
                planner.CalculateVelocity(Vector3.zero, Vector3.zero, Vector3.forward);
            });
        }

        [Test]
        public void CalculateVelocity_AngleSamplesOne_DoesNotThrow()
        {
            var planner = new DWAPlanner
            {
                VelocitySamples = 5,
                AngleSamples = 1,
                MaxSpeed = 2.0f,
                MinSpeed = 0.5f,
                MaxRotSpeed = 0.8f,
                PredictionTime = 0.5f
            };

            Assert.DoesNotThrow(() =>
            {
                planner.CalculateVelocity(Vector3.zero, Vector3.zero, Vector3.forward);
            });
        }

        [Test]
        public void CalculateVelocity_BothSamplesOne_DoesNotThrow()
        {
            var planner = new DWAPlanner
            {
                VelocitySamples = 1,
                AngleSamples = 1,
                MaxSpeed = 2.0f,
                MinSpeed = 0.5f,
                MaxRotSpeed = 0.8f,
                PredictionTime = 0.5f
            };

            Assert.DoesNotThrow(() =>
            {
                planner.CalculateVelocity(Vector3.zero, Vector3.zero, Vector3.forward);
            });
        }

        [Test]
        public void CalculateVelocity_ZeroSamples_DoesNotThrow()
        {
            var planner = new DWAPlanner
            {
                VelocitySamples = 0,
                AngleSamples = 0,
                MaxSpeed = 2.0f,
                MinSpeed = 0.5f,
                MaxRotSpeed = 0.8f,
                PredictionTime = 0.5f
            };

            Assert.DoesNotThrow(() =>
            {
                planner.CalculateVelocity(Vector3.zero, Vector3.zero, Vector3.forward);
            });
        }

        [Test]
        public void CalculateVelocity_NormalSamples_ReturnsVelocity()
        {
            var planner = new DWAPlanner
            {
                VelocitySamples = 7,
                AngleSamples = 15,
                MaxSpeed = 2.0f,
                MinSpeed = 0.5f,
                MaxRotSpeed = 0.8f,
                PredictionTime = 0.5f
            };

            var (velocity, needReplan) = planner.CalculateVelocity(
                Vector3.zero, Vector3.forward * 0.5f, Vector3.forward * 5.0f);

            Assert.IsFalse(float.IsNaN(velocity.x), "Velocity X should not be NaN");
            Assert.IsFalse(float.IsNaN(velocity.y), "Velocity Y should not be NaN");
            Assert.IsFalse(float.IsNaN(velocity.z), "Velocity Z should not be NaN");
        }
    }
}
