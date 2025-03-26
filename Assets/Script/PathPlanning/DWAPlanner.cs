using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace PathPlanning
{
    public class DWAPlanner
    {
        public float MaxSpeed { get; set; } = 2.0f;
        public float MinSpeed { get; set; } = 0.5f;
        public float MaxRotSpeed { get; set; } = 0.8f;
        public float MaxAccel { get; set; } = 2.5f;
        public float VelocityResolution { get; set; } = 0.15f;
        public float RotationResolution { get; set; } = 0.2f;
        public float PredictionTime { get; set; } = 1.0f;
        public float HeadingWeight { get; set; } = 0.6f;
        public float DistanceWeight { get; set; } = 0.3f;
        public float VelocityWeight { get; set; } = 0.15f;
        public float ObstacleWeight { get; set; } = 0.15f;
        public float DWAScoreThreshold { get; set; } = 0.25f;
        public LayerMask ObstacleLayer { get; set; }
        public int VelocitySamples { get; set; } = 7;    // 速度采样数
        public int AngleSamples { get; set; } = 15;      // 角度采样数
        public float ObstacleInflation { get; set; } = 0.2f; // 障碍物膨胀系数

        private Vector3 previousBestVelocity = Vector3.zero;
        private bool hasPreviousVelocity = false;

        public (Vector3 velocity, bool needReplan) CalculateVelocity(
            Vector3 currentPosition, 
            Vector3 currentVelocity, 
            Vector3 goalPosition)
        {
            // Initialize the best score to the current velocity
            Vector3 bestVelocity = currentVelocity;
            float bestScore = float.MinValue;
            bool needReplan = false;

            // Calculate the current speed
            float currentSpeed = currentVelocity.magnitude;
            
            // Calculate the min and max available speeds
            float minSpeed = Mathf.Max(MinSpeed, currentSpeed - MaxAccel * Time.deltaTime);
            float maxSpeed = Mathf.Min(MaxSpeed, currentSpeed + MaxAccel * Time.deltaTime);
            
            // Use variable resolution
            float speedStep = (maxSpeed - minSpeed) / Mathf.Max(1, VelocitySamples - 1);
            float angleStep = (MaxRotSpeed * 2) / Mathf.Max(1, AngleSamples - 1);
            
            // Record the time spent on sampling
            System.Diagnostics.Stopwatch samplingTimer = new System.Diagnostics.Stopwatch();
            samplingTimer.Start();
            
            // Sample velocities
            for (int i = 0; i < VelocitySamples; i++)
            {
                float speed = minSpeed + i * speedStep;
                
                for (int j = 0; j < AngleSamples; j++)
                {
                    float angle = -MaxRotSpeed + j * angleStep;
                    
                    // Calculate the sampled velocity
                    Vector3 sampledVelocity = CalculateSampledVelocity(currentPosition, goalPosition, speed, angle);
                    
                    // Check if the velocity is safe
                    bool isSafe = IsVelocitySafe(currentPosition, sampledVelocity, ObstacleLayer);
                    
                    if (isSafe)
                    {
                        // Evaluate the velocity
                        float score = EvaluateVelocity(currentPosition, sampledVelocity, goalPosition, ObstacleLayer);
                        
                        // Smoothing score 
                        if (hasPreviousVelocity)
                        {
                            float smoothingScore = Vector3.Dot(sampledVelocity.normalized, previousBestVelocity.normalized);
                            smoothingScore = Mathf.Pow(smoothingScore, 2) * 0.1f; // 平滑权重
                            score += smoothingScore;
                        }
                        
                        // Update the best score
                        if (score > bestScore)
                        {
                            bestScore = score;
                            bestVelocity = sampledVelocity;
                        }
                    }
                }
            }
            
            samplingTimer.Stop();
            if (samplingTimer.ElapsedMilliseconds > 10)
            {
                Debug.Log($"DWA sampling cost: {samplingTimer.ElapsedMilliseconds}ms, sample count: {VelocitySamples * AngleSamples}");
            }
            
            // If the best score is lower than the threshold, replan
            if (bestScore < DWAScoreThreshold)
            {
                needReplan = true;
                
                // If there is a previous velocity, reduce the speed but keep the direction
                if (hasPreviousVelocity)
                {
                    // Reduce the speed but keep the direction
                    bestVelocity = previousBestVelocity.normalized * previousBestVelocity.magnitude * 0.7f;
                }
                else
                {
                    // If there is no previous velocity, try to go directly to the goal
                    Vector3 dirToGoal = (goalPosition - currentPosition).normalized;
                    bestVelocity = dirToGoal * MinSpeed;
                }
            }
            
            // Store the current best velocity for smoothing
            previousBestVelocity = bestVelocity;
            hasPreviousVelocity = true;
            
            return (bestVelocity, needReplan);
        }

        private Vector3 CalculateSampledVelocity(Vector3 currentPosition, Vector3 goalPosition, float speed, float rotation)
        {
            // Calculate the direction to the goal
            Vector3 directionToGoal = (goalPosition - currentPosition).normalized;
            
            // Rotate
            return Quaternion.Euler(0, rotation * Mathf.Rad2Deg, 0) * directionToGoal * speed;
        }

        private bool IsVelocitySafe(Vector3 currentPosition, Vector3 velocity, LayerMask obstacleLayer)
        {
            float speed = velocity.magnitude;

            // Calculate angular velocity
            float angularVelocity = Mathf.Atan2(velocity.x, velocity.z) / Time.deltaTime;
            
            // Predict the trajectory to see if it is safe
            Vector3[] trajectory = PredictTrajectory(currentPosition, velocity, speed, angularVelocity);
            
            // Check each point on the trajectory
            for (int i = 0; i < trajectory.Length; i++)
            {
                // Check with a larger radius to consider the actual size of the robot and obstacle inflation
                if (Physics.CheckSphere(trajectory[i], 0.5f + ObstacleInflation, obstacleLayer))
                {
                    return false;
                }
            }
            
            return true;
        }

        // 更精确的轨迹预测，考虑转向动态特性
        private Vector3[] PredictTrajectory(Vector3 startPos, Vector3 startVelocity, float speed, float angularVelocity)
        {
            int steps = 15; // 增加预测点数
            Vector3[] trajectory = new Vector3[steps];
            
            // 初始状态
            Vector3 position = startPos;
            Vector3 forward = startVelocity.normalized;
            float timeStep = PredictionTime / steps;
            
            // 模拟运动学模型
            for (int i = 0; i < steps; i++)
            {
                // 更新位置
                position += forward * speed * timeStep;
                
                // 更新朝向
                forward = Quaternion.Euler(0, angularVelocity * timeStep * Mathf.Rad2Deg, 0) * forward;
                
                trajectory[i] = position;
            }
            
            return trajectory;
        }

        private float EvaluateVelocity(Vector3 currentPosition, Vector3 velocity, Vector3 goalPosition, LayerMask obstacleLayer)
        {
            // Predict the target position
            Vector3 predictedPosition = currentPosition + velocity * PredictionTime;
            
            // 1. heading score
            Vector3 dirToGoal = (goalPosition - predictedPosition).normalized;
            float headingScore = Vector3.Dot(velocity.normalized, dirToGoal);
            headingScore = Mathf.Pow(headingScore, 2);
            
            // 2. Distance score
            float distance = Vector3.Distance(predictedPosition, goalPosition);
            float distanceScore = 1.0f / (1.0f + distance);
            
            // 3. Speed score
            float normalizedSpeed = velocity.magnitude / MaxSpeed;
            float velocityScore = normalizedSpeed;
            
            // Improve the score for straight-line motion
            if (Mathf.Abs(velocity.x) < 0.2f * velocity.magnitude)
            {
                velocityScore *= 1.2f;
            }
            
            // Improve the score for high speed
            if (normalizedSpeed > 0.75f)
            {
                velocityScore *= 1.3f;
            }
            else if (normalizedSpeed < 0.3f)
            {
                velocityScore *= 0.5f;
            }
            
            velocityScore = Mathf.Min(velocityScore, 1.0f);
            
            // 4. Obstacle score
            float obstacleScore = EvaluateObstacleDistance(currentPosition, velocity, obstacleLayer);
            
            // Combine the scores
            return HeadingWeight * headingScore + 
                DistanceWeight * distanceScore + 
                VelocityWeight * velocityScore + 
                ObstacleWeight * obstacleScore;
        }

        private float EvaluateObstacleDistance(Vector3 currentPosition, Vector3 velocity, LayerMask obstacleLayer)
        {
            float speed = velocity.magnitude;
            float angularVelocity = Mathf.Atan2(velocity.x, velocity.z) / Time.deltaTime;
            
            // Calculate the trajectory and check the minimum distance to obstacles
            Vector3[] trajectory = PredictTrajectory(currentPosition, velocity, speed, angularVelocity);
            
            float minDistance = float.MaxValue;
            
            foreach (Vector3 point in trajectory)
            {
                // 寻找最近的障碍物
                Collider[] obstacles = Physics.OverlapSphere(point, 1.5f, obstacleLayer);
                
                foreach (Collider obstacle in obstacles)
                {
                    float distance = Vector3.Distance(point, obstacle.ClosestPoint(point));
                    minDistance = Mathf.Min(minDistance, distance);
                }
            }
            
            // 如果没有找到障碍物，返回最高分
            if (minDistance == float.MaxValue)
                return 1.0f;
                
            // 根据距离计算分数，距离越大得分越高
            return Mathf.Clamp01(minDistance / 1.5f);
        }
    }
}
