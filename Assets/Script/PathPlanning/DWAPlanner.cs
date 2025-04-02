using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;

namespace PathPlanning
{
    /// <summary>
    /// DWA
    /// </summary>
    public class DWAPlanner
    {
        public float MaxSpeed { get; set; } = 2.0f;          // Maximum speed
        public float MinSpeed { get; set; } = 0.5f;          // Minimum speed
        public float MaxRotSpeed { get; set; } = 0.8f;       // Maximum rotation speed
        public float MaxAccel { get; set; } = 2.5f;          // Maximum acceleration
        public float PredictionTime { get; set; } = 1.5f;    // Prediction time for trajectory

        // Sampling parameters
        public int VelocitySamples { get; set; } = 7;        // Speed samples
        public int AngleSamples { get; set; } = 15;          // Angle samples
        public float ObstacleInflation { get; set; } = 0.2f; // Obstacle inflation distance

        // Score weights
        public float HeadingWeight { get; set; } = 0.6f;     // Heading score weight
        public float DistanceWeight { get; set; } = 0.2f;    // Distance score weight
        public float VelocityWeight { get; set; } = 0.2f;    // Velocity score weight
        public float ObstacleWeight { get; set; } = 0.3f;    // Obstacle score weight

        // Physics layer for obstacles
        public LayerMask ObstacleLayer { get; set; }

        // Previous velocity for smoothing
        private Vector3 previousBestVelocity = Vector3.zero;
        private bool hasPreviousVelocity = false;

        /// <summary>
        /// Calculate the best velocity
        /// </summary>
        public (Vector3, bool) CalculateVelocity(Vector3 currentPosition, Vector3 currentVelocity, Vector3 goalPosition)
        {
            var samplingTimer = new Stopwatch();
            samplingTimer.Start();
            
            bool needReplan = false;
            Vector3 bestVelocity = Vector3.zero;
            float bestScore = -float.MaxValue;
            
            // Calculate the direction to the goal
            Vector3 dirToGoal = (goalPosition - currentPosition).normalized;
            float distToGoal = Vector3.Distance(currentPosition, goalPosition);
            
            // Adjust speed based on distance to goal
            if (distToGoal < 0.5f)
            {
                MaxSpeed = Mathf.Min(MaxSpeed, distToGoal + 0.1f);
            }
            
            // Velocity sampling
            for (int i = 0; i < VelocitySamples; i++)
            {
                float sampledVelocityMagnitude = Mathf.Lerp(MinSpeed, MaxSpeed, i / (float)(VelocitySamples - 1));
                
                // Angle sampling
                for (int j = 0; j < AngleSamples; j++)
                {
                    float angle = Mathf.Lerp(-MaxRotSpeed, MaxRotSpeed, j / (float)(AngleSamples - 1));
                    
                    // Velocity vector
                    Quaternion rot = Quaternion.Euler(0, angle * Mathf.Rad2Deg, 0);
                    Vector3 sampledDirection = rot * dirToGoal;
                    Vector3 sampledVelocity = sampledDirection * sampledVelocityMagnitude;
                    
                    // Skip samples that require too much acceleration
                    if ((sampledVelocity - currentVelocity).magnitude > MaxAccel * Time.fixedDeltaTime)
                    {
                        continue;
                    }
                    
                    // Predict the trajectory
                    Vector3 predictedPosition = currentPosition + sampledVelocity * PredictionTime;
                    
                    // Score calculation
                    float score = 0;
                    
                    // 1. Heading score - reward for heading towards the goal
                    float headingScore = Vector3.Dot(sampledVelocity.normalized, dirToGoal);
                    headingScore = Mathf.Max(0, headingScore);
                    
                    // 2. Distance score - reward for being closer to the goal
                    float predictedDistance = Vector3.Distance(predictedPosition, goalPosition);
                    float distanceScore = 1.0f - Mathf.Min(1.0f, predictedDistance / Mathf.Max(1.0f, distToGoal));
                    
                    // 3. Velocity score - reward for maintaining speed
                    float velocityScore = sampledVelocityMagnitude / MaxSpeed;
                    
                    // 4. Obstacle score - penalty for being close to obstacles
                    float obstacleScore = EvaluateObstacleDistance(currentPosition, sampledVelocity);
                    
                    // 5. Combine scores
                    score = HeadingWeight * headingScore
                          + DistanceWeight * distanceScore
                          + VelocityWeight * velocityScore
                          + ObstacleWeight * obstacleScore;
                    
                    // 6. Check for collisions
                    if (WillHitObstacle(currentPosition, sampledVelocity))
                    {
                        continue; // Skip this sample if it will hit an obstacle
                    }
                    
                    // 7. Update best score and velocity
                    if (score > bestScore)
                    {
                        bestScore = score;
                        bestVelocity = sampledVelocity;
                    }
                }
            }
            
            samplingTimer.Stop();
            if (samplingTimer.ElapsedMilliseconds > 10)
            {
                UnityEngine.Debug.Log($"DWA sampling cost: {samplingTimer.ElapsedMilliseconds}ms, sample count: {VelocitySamples * AngleSamples}");
            }
            
            // If no valid velocity found, replan
            if (bestScore == -float.MaxValue)
            {
                needReplan = true;
                
                // If no valid velocity found, use the previous best velocity
                if (hasPreviousVelocity)
                {
                    bestVelocity = previousBestVelocity.normalized * previousBestVelocity.magnitude * 0.7f;
                }
                else
                {
                    // Attempt to move towards the goal with minimum speed
                    bestVelocity = dirToGoal * MinSpeed;
                }
            }
            
            // Save the current best velocity for smoothing
            previousBestVelocity = bestVelocity;
            hasPreviousVelocity = true;
            
            return (bestVelocity, needReplan);
        }
        
        /// <summary>
        /// Obstacle score evaluation
        /// </summary>
        private float EvaluateObstacleDistance(Vector3 position, Vector3 velocity)
        {
            float minDistance = float.MaxValue;
            
            // Predict the trajectory along the velocity direction
            for (float t = 0; t <= PredictionTime; t += 0.1f)
            {
                Vector3 predictedPosition = position + velocity * t;
                
                // Detect surrounding obstacles
                Collider[] hitColliders = Physics.OverlapSphere(predictedPosition, 1.0f, ObstacleLayer);
                
                foreach (var hitCollider in hitColliders)
                {
                    // 忽略触发器
                    if (hitCollider.isTrigger)
                        continue;
                    
                    // 使用不同方法获取距离，不依赖于ClosestPoint
                    
                    // 直接使用边界盒计算距离
                    Bounds bounds = hitCollider.bounds;
                    Vector3 closest = bounds.ClosestPoint(predictedPosition);
                    float distance = Vector3.Distance(predictedPosition, closest);

                    // 更新最小距离
                    minDistance = Mathf.Min(minDistance, distance);
                }
            }
            

            if (minDistance == float.MaxValue)
                return 1.0f;
            
            return Mathf.Clamp01(minDistance / 1.5f);
        }
        
        /// <summary>
        /// Check if the predicted trajectory will hit an obstacle
        /// </summary>
        private bool WillHitObstacle(Vector3 position, Vector3 velocity)
        {
            // Calculate the length of the predicted trajectory
            float distance = velocity.magnitude * PredictionTime;
            
            if (Physics.Raycast(position, velocity.normalized, distance + ObstacleInflation, ObstacleLayer))
            {
                return true;
            }
            
            return false;
        }
        
        /// <summary>
        /// Reset the planner state
        /// </summary>
        public void Reset()
        {
            previousBestVelocity = Vector3.zero;
            hasPreviousVelocity = false;
        }
    }
}