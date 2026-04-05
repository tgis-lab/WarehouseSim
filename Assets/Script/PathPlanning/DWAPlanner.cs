using System.Collections.Generic;
using UnityEngine;

namespace PathPlanning
{
    public struct ObstacleData
    {
        public Vector3 Position;
        public float Radius;

        public ObstacleData(Vector3 position, float radius)
        {
            Position = position;
            Radius = Mathf.Max(0f, radius);
        }
    }

    public struct DWAResult
    {
        public float LinearVelocity;
        public float AngularVelocity;
        public bool NeedReplan;

        public DWAResult(float linearVelocity, float angularVelocity, bool needReplan)
        {
            LinearVelocity = linearVelocity;
            AngularVelocity = angularVelocity;
            NeedReplan = needReplan;
        }

        public static DWAResult Stop(bool needReplan = true) =>
            new DWAResult(0f, 0f, needReplan);
    }

    /// <summary>
    /// Dynamic Window Approach (Fox et al. 1997) local obstacle avoidance planner.
    /// Uses (v, omega) differential drive model with admissibility checking.
    /// All obstacle data is provided externally — no internal Physics queries.
    /// </summary>
    public class DWAPlanner
    {
        // --- Motion limits ---
        public float MaxSpeed { get; set; } = 2.0f;
        public float MaxAccel { get; set; } = 2.0f;
        public float MaxDecel { get; set; } = 5.0f;
        public float MaxOmega { get; set; } = 2.0f;
        public float MaxAngAccel { get; set; } = 10.0f;

        // --- Timing ---
        public float PredictionTime { get; set; } = 1.5f;
        public float DeltaTime { get; set; } = 0.05f;

        // --- Sampling ---
        public int VelocitySamples { get; set; } = 7;
        public int OmegaSamples { get; set; } = 15;
        public int TrajectorySteps { get; set; } = 10;

        // --- Robot geometry ---
        public float RobotRadius { get; set; } = 0.3f;

        // --- Score weights ---
        public float HeadingWeight { get; set; } = 0.3f;
        public float ClearanceWeight { get; set; } = 0.4f;
        public float VelocityWeight { get; set; } = 0.2f;
        public float GoalDistWeight { get; set; } = 0.1f;

        // --- Legacy compatibility (remove when RobotController fully switches to new API) ---
        public LayerMask ObstacleLayer { get; set; }
        public int AngleSamples { get { return OmegaSamples; } set { OmegaSamples = value; } }
        public float MinSpeed { get; set; } = 0.5f;
        public float MaxRotSpeed { get; set; } = 0.8f;
        public float ObstacleInflation { get; set; } = 0.2f;
        public float HeadingWeight_Legacy { get; set; } = 0.6f;
        public float DistanceWeight { get; set; } = 0.2f;

        public (Vector3, bool) CalculateVelocity(
            Vector3 currentPosition, Vector3 currentVelocity, Vector3 goalPosition,
            Collider[] externalObstacles)
        {
            _legacyObstacleBuffer.Clear();
            if (externalObstacles != null)
            {
                foreach (var col in externalObstacles)
                {
                    if (col == null || col.isTrigger) continue;
                    float radius = Mathf.Max(col.bounds.extents.x, col.bounds.extents.z);
                    _legacyObstacleBuffer.Add(new ObstacleData(col.transform.position, radius));
                }
            }

            float currentSpeed = currentVelocity.magnitude;
            Quaternion orientation = currentVelocity.sqrMagnitude > 0.001f
                ? Quaternion.LookRotation(currentVelocity.normalized, Vector3.up)
                : Quaternion.LookRotation(Vector3.forward, Vector3.up);

            var result = CalculateVelocity(
                currentPosition, orientation, currentSpeed, 0f, goalPosition, _legacyObstacleBuffer);

            Vector3 worldVelocity;
            if (result.NeedReplan)
            {
                worldVelocity = Vector3.zero;
            }
            else
            {
                Vector3 forward = orientation * Vector3.forward;
                worldVelocity = forward * result.LinearVelocity;
            }

            return (worldVelocity, result.NeedReplan);
        }

        public (Vector3, bool) CalculateVelocity(
            Vector3 currentPosition, Vector3 currentVelocity, Vector3 goalPosition)
        {
            return CalculateVelocity(currentPosition, currentVelocity, goalPosition, (Collider[])null);
        }

        // --- Internal buffers (reused across calls to avoid GC) ---
        private float _previousV;
        private float _previousOmega;
        private bool _hasPreviousResult;
        private readonly List<Vector3> _trajectoryBuffer = new List<Vector3>(16);
        private readonly List<Candidate> _candidateBuffer = new List<Candidate>(128);
        private readonly List<ObstacleData> _legacyObstacleBuffer = new List<ObstacleData>(16);
        private List<Vector3> _lastSelectedTrajectory;

        /// <summary>
        /// Returns the trajectory of the last selected (v, omega) pair.
        /// Reusable by RobotController for corridor validation without recomputing.
        /// </summary>
        public List<Vector3> LastSelectedTrajectory => _lastSelectedTrajectory;

        public DWAResult CalculateVelocity(
            Vector3 position,
            Quaternion orientation,
            float currentV,
            float currentOmega,
            Vector3 goal,
            IReadOnlyList<ObstacleData> obstacles)
        {
            float dt = Mathf.Max(0.001f, DeltaTime);
            int vSamples = Mathf.Max(1, VelocitySamples);
            int wSamples = Mathf.Max(1, OmegaSamples);
            int trajSteps = Mathf.Max(1, TrajectorySteps);
            float predTime = Mathf.Max(0.01f, PredictionTime);

            // --- 1. Dynamic window bounds ---
            float vMin = Mathf.Max(0f, currentV - MaxDecel * dt);
            float vMax = Mathf.Min(MaxSpeed, currentV + MaxAccel * dt);
            float wMin = Mathf.Max(-MaxOmega, currentOmega - MaxAngAccel * dt);
            float wMax = Mathf.Min(MaxOmega, currentOmega + MaxAngAccel * dt);

            // --- 2. Sample, score, and track normalization bounds in single pass ---
            _candidateBuffer.Clear();
            float minH = float.MaxValue, maxH = float.MinValue;
            float minC = float.MaxValue, maxC = float.MinValue;
            float minV = float.MaxValue, maxV = float.MinValue;
            float minG = float.MaxValue, maxG = float.MinValue;

            for (int i = 0; i < vSamples; i++)
            {
                float v = vSamples == 1
                    ? (vMin + vMax) * 0.5f
                    : Mathf.Lerp(vMin, vMax, i / (float)(vSamples - 1));

                for (int j = 0; j < wSamples; j++)
                {
                    float omega = wSamples == 1
                        ? (wMin + wMax) * 0.5f
                        : Mathf.Lerp(wMin, wMax, j / (float)(wSamples - 1));

                    // Simulate circular arc into reusable buffer
                    SimulateArcInto(_trajectoryBuffer, position, orientation, v, omega, predTime, trajSteps);

                    // Min obstacle distance along the arc (with early exit), accounting for robot body
                    float minDist = MinObstacleDistanceAlongArc(_trajectoryBuffer, obstacles, RobotRadius);

                    // Admissibility: robot must be able to stop before nearest obstacle
                    if (v > 0f && minDist < float.MaxValue)
                    {
                        float maxAdmissibleV = Mathf.Sqrt(2f * minDist * MaxDecel);
                        if (v > maxAdmissibleV)
                            continue;
                    }

                    // Compute raw score components
                    Vector3 endPoint = _trajectoryBuffer[_trajectoryBuffer.Count - 1];
                    Quaternion endOrientation = orientation *
                        Quaternion.Euler(0f, omega * Mathf.Rad2Deg * predTime, 0f);

                    Vector3 endToGoal = goal - endPoint;
                    float headingRaw;
                    if (endToGoal.sqrMagnitude < 0.0001f)
                    {
                        headingRaw = 1f;
                    }
                    else
                    {
                        Vector3 endForward = endOrientation * Vector3.forward;
                        headingRaw = Mathf.Max(0f,
                            Vector3.Dot(endForward.normalized, endToGoal.normalized));
                    }

                    float clearanceRaw = minDist >= float.MaxValue ? 10f : minDist;
                    float velocityRaw = v;
                    float goalDistRaw = new Vector3(
                        endPoint.x - goal.x, 0f, endPoint.z - goal.z).magnitude;

                    // Track normalization bounds inline (single-pass)
                    if (headingRaw < minH) minH = headingRaw;
                    if (headingRaw > maxH) maxH = headingRaw;
                    if (clearanceRaw < minC) minC = clearanceRaw;
                    if (clearanceRaw > maxC) maxC = clearanceRaw;
                    if (velocityRaw < minV) minV = velocityRaw;
                    if (velocityRaw > maxV) maxV = velocityRaw;
                    if (goalDistRaw < minG) minG = goalDistRaw;
                    if (goalDistRaw > maxG) maxG = goalDistRaw;

                    _candidateBuffer.Add(new Candidate(
                        v, omega, headingRaw, clearanceRaw, velocityRaw, goalDistRaw));
                }
            }

            // --- 3. No admissible candidates → STOP ---
            if (_candidateBuffer.Count == 0)
            {
                _hasPreviousResult = false;
                _lastSelectedTrajectory = null;
                return DWAResult.Stop(needReplan: true);
            }

            // --- 4. Normalize and select best (second pass — required for min-max normalization) ---
            float rangeH = maxH - minH;
            float rangeC = maxC - minC;
            float rangeV = maxV - minV;
            float rangeG = maxG - minG;

            float bestScore = float.MinValue;
            float bestV = 0f, bestOmega = 0f;
            float bestMinDist = float.MaxValue;

            for (int i = 0; i < _candidateBuffer.Count; i++)
            {
                var c = _candidateBuffer[i];

                float hNorm = rangeH > 1e-6f ? (c.HeadingRaw - minH) / rangeH : 0.5f;
                float cNorm = rangeC > 1e-6f ? (c.ClearanceRaw - minC) / rangeC : 0.5f;
                float vNorm = rangeV > 1e-6f ? (c.VelocityRaw - minV) / rangeV : 0.5f;
                float gNorm = rangeG > 1e-6f ? (maxG - c.GoalDistRaw) / rangeG : 0.5f;

                float score = HeadingWeight * hNorm
                            + ClearanceWeight * cNorm
                            + VelocityWeight * vNorm
                            + GoalDistWeight * gNorm;

                if (score > bestScore)
                {
                    bestScore = score;
                    bestV = c.V;
                    bestOmega = c.Omega;
                    bestMinDist = c.ClearanceRaw;
                }
            }

            // --- 5. Apply smoothing with post-smoothing admissibility clamp ---
            float preSmoothBestV = bestV;
            if (_hasPreviousResult)
            {
                bestV = Mathf.Lerp(_previousV, bestV, 0.7f);
                bestOmega = Mathf.Lerp(_previousOmega, bestOmega, 0.7f);

                // Re-check admissibility after smoothing — smoothing may pull velocity
                // above the safe stopping threshold for the winning trajectory's obstacle distance
                if (bestV > 0f && bestMinDist < float.MaxValue)
                {
                    float maxAdmissibleV = Mathf.Sqrt(2f * bestMinDist * MaxDecel);
                    if (bestV > maxAdmissibleV)
                    {
                        bestV = preSmoothBestV;
                    }
                }
            }

            _previousV = bestV;
            _previousOmega = bestOmega;
            _hasPreviousResult = true;

            // Cache selected trajectory for corridor validation reuse
            SimulateArcInto(_trajectoryBuffer, position, orientation, bestV, bestOmega, predTime, trajSteps);
            _lastSelectedTrajectory = new List<Vector3>(_trajectoryBuffer);

            return new DWAResult(bestV, bestOmega, false);
        }

        /// <summary>
        /// Simulate circular arc trajectory for a differential drive robot.
        /// Public static version for external callers (allocates new list).
        /// </summary>
        public static List<Vector3> SimulateArc(
            Vector3 position, Quaternion orientation,
            float v, float omega, float totalTime, int steps)
        {
            steps = Mathf.Max(1, steps);
            var points = new List<Vector3>(steps + 1);
            points.Add(position);

            float stepDt = totalTime / steps;
            Vector3 pos = position;
            Quaternion rot = orientation;

            for (int i = 0; i < steps; i++)
            {
                Vector3 forward = rot * Vector3.forward;
                pos += forward * v * stepDt;
                rot *= Quaternion.Euler(0f, omega * Mathf.Rad2Deg * stepDt, 0f);
                points.Add(pos);
            }

            return points;
        }

        public void Reset()
        {
            _previousV = 0f;
            _previousOmega = 0f;
            _hasPreviousResult = false;
            _lastSelectedTrajectory = null;
        }

        // ----- Private helpers -----

        private static void SimulateArcInto(
            List<Vector3> buffer, Vector3 position, Quaternion orientation,
            float v, float omega, float totalTime, int steps)
        {
            steps = Mathf.Max(1, steps);
            buffer.Clear();
            buffer.Add(position);

            float stepDt = totalTime / steps;
            Vector3 pos = position;
            Quaternion rot = orientation;

            for (int i = 0; i < steps; i++)
            {
                Vector3 forward = rot * Vector3.forward;
                pos += forward * v * stepDt;
                rot *= Quaternion.Euler(0f, omega * Mathf.Rad2Deg * stepDt, 0f);
                buffer.Add(pos);
            }
        }

        private static float MinObstacleDistanceAlongArc(
            List<Vector3> trajectory, IReadOnlyList<ObstacleData> obstacles, float robotRadius = 0f)
        {
            if (obstacles == null || obstacles.Count == 0)
                return float.MaxValue;

            float minDist = float.MaxValue;

            for (int i = 0; i < trajectory.Count; i++)
            {
                float px = trajectory[i].x;
                float pz = trajectory[i].z;

                for (int j = 0; j < obstacles.Count; j++)
                {
                    float dx = px - obstacles[j].Position.x;
                    float dz = pz - obstacles[j].Position.z;
                    float dist = Mathf.Sqrt(dx * dx + dz * dz) - obstacles[j].Radius - robotRadius;
                    if (dist < minDist)
                        minDist = dist;
                    if (minDist <= 0f)
                        return 0f; // Early exit: trajectory intersects obstacle
                }
            }

            return Mathf.Max(0f, minDist);
        }

        private readonly struct Candidate
        {
            public readonly float V;
            public readonly float Omega;
            public readonly float HeadingRaw;
            public readonly float ClearanceRaw;
            public readonly float VelocityRaw;
            public readonly float GoalDistRaw;

            public Candidate(float v, float omega,
                float headingRaw, float clearanceRaw, float velocityRaw, float goalDistRaw)
            {
                V = v;
                Omega = omega;
                HeadingRaw = headingRaw;
                ClearanceRaw = clearanceRaw;
                VelocityRaw = velocityRaw;
                GoalDistRaw = goalDistRaw;
            }
        }
    }
}
