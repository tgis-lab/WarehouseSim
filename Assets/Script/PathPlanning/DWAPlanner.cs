using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;

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
        public float TurningObstacleWeight { get; set; } = 0.7f;
        public float TurningHeadingWeight { get; set; } = 0.3f;
        public float TurningPredictionTime { get; set; } = 2.5f;
        public int TurningVelocitySamples { get; set; } = 9;
        public int TurningAngleSamples { get; set; } = 20;
        public float MinTurningRadius { get; set; } = 0.8f;

        private Vector3 previousBestVelocity = Vector3.zero;
        private bool hasPreviousVelocity = false;

        public (Vector3, bool) CalculateVelocity(Vector3 currentPosition, Vector3 currentVelocity, Vector3 goalPosition, bool specialAvoidanceMode = false)
        {
            // 保存原始参数，在函数结束时恢复
            float originalObstacleWeight = ObstacleWeight;
            float originalHeadingWeight = HeadingWeight;
            float originalPredictionTime = PredictionTime;
            int originalVelocitySamples = VelocitySamples;
            int originalAngleSamples = AngleSamples;
            
            // 如果是特殊避障模式（转弯处），应用特殊参数
            if (specialAvoidanceMode)
            {
                ObstacleWeight = TurningObstacleWeight;
                HeadingWeight = TurningHeadingWeight;
                PredictionTime = TurningPredictionTime;
                VelocitySamples = TurningVelocitySamples;
                AngleSamples = TurningAngleSamples;
            }
            
            // 计时（用于性能分析）
            var samplingTimer = new Stopwatch();
            samplingTimer.Start();
            
            bool needReplan = false;
            
            Vector3 bestVelocity = Vector3.zero;
            float bestScore = -float.MaxValue;
            
            // Speed sampling
            for (int i = 0; i < VelocitySamples; i++)
            {
                float speed = Mathf.Lerp(MinSpeed, MaxSpeed, i / (float)(VelocitySamples - 1));
                
                // Rotation sampling
                for (int j = 0; j < AngleSamples; j++)
                {
                    float angle = Mathf.Lerp(-MaxRotSpeed, MaxRotSpeed, j / (float)(AngleSamples - 1));
                    
                    Vector3 sampledVelocity = CalculateSampledVelocity(currentVelocity, speed, angle);
                    
                    // Check if velocity is safe (obstacle avoidance)
                    bool isSafe = IsSafeVelocity(currentPosition, sampledVelocity, ObstacleLayer);
                    
                    // 在特殊避障模式下增加对转弯半径的检查
                    if (specialAvoidanceMode)
                    {
                        isSafe = isSafe && CheckTurningRadius(sampledVelocity, angle);
                    }
                    
                    if (isSafe)
                    {
                        // Evaluate the velocity
                        float score = EvaluateVelocity(currentPosition, sampledVelocity, goalPosition, ObstacleLayer);
                        
                        // Smoothing score 
                        if (hasPreviousVelocity)
                        {
                            float smoothingScore = Vector3.Dot(sampledVelocity.normalized, previousBestVelocity.normalized);
                            smoothingScore = Mathf.Pow(smoothingScore, 2) * 0.1f;
                            score += smoothingScore;
                        }
                        
                        // 特殊避障模式下额外考虑转弯平滑性
                        if (specialAvoidanceMode)
                        {
                            // 计算转弯半径奖励
                            float radiusBonus = EvaluateTurningRadius(sampledVelocity, angle);
                            score += radiusBonus * 0.2f;
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
                UnityEngine.Debug.Log($"DWA sampling cost: {samplingTimer.ElapsedMilliseconds}ms, sample count: {VelocitySamples * AngleSamples}");
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
            
            // 恢复原始参数
            ObstacleWeight = originalObstacleWeight;
            HeadingWeight = originalHeadingWeight;
            PredictionTime = originalPredictionTime;
            VelocitySamples = originalVelocitySamples;
            AngleSamples = originalAngleSamples;
            
            return (bestVelocity, needReplan);
        }
        
        private bool CheckTurningRadius(Vector3 velocity, float angularVelocity)
        {
            if (Mathf.Abs(angularVelocity) < 0.001f)
                return true;
            
            // 计算转弯半径 R = V/ω
            float turningRadius = velocity.magnitude / Mathf.Abs(angularVelocity);
            
            // 如果转弯半径小于最小要求，认为不安全
            return turningRadius >= MinTurningRadius;
        }
        
        private float EvaluateTurningRadius(Vector3 velocity, float angularVelocity)
        {
            // 只对有转向的情况评估
            if (Mathf.Abs(angularVelocity) < 0.001f)
                return 0;
            
            // 计算转弯半径
            float turningRadius = velocity.magnitude / Mathf.Abs(angularVelocity);
            
            // 转弯半径越大越好，但有上限
            float idealRadius = MinTurningRadius * 2.0f;
            float maxRadius = MinTurningRadius * 4.0f;
            
            if (turningRadius < MinTurningRadius)
                return 0; // 低于最小半径，零分
            else if (turningRadius <= idealRadius)
                return Mathf.Lerp(0.5f, 1.0f, (turningRadius - MinTurningRadius) / (idealRadius - MinTurningRadius));
            else if (turningRadius <= maxRadius)
                return Mathf.Lerp(1.0f, 0.5f, (turningRadius - idealRadius) / (maxRadius - idealRadius));
            else
                return 0.5f; // 太大的半径也不理想
        }

        private Vector3 CalculateSampledVelocity(Vector3 currentVelocity, float speed, float angle)
        {
            // 计算旋转后的方向
            float currentAngle = Mathf.Atan2(currentVelocity.x, currentVelocity.z);
            float targetAngle = currentAngle + angle * Time.deltaTime;
            
            // 创建新的速度向量
            return new Vector3(Mathf.Sin(targetAngle), 0, Mathf.Cos(targetAngle)) * speed;
        }

        private bool IsSafeVelocity(Vector3 currentPosition, Vector3 velocity, LayerMask obstacleLayer)
        {
            // 预测轨迹
            Vector3[] trajectory = PredictTrajectory(currentPosition, velocity, velocity.magnitude, Mathf.Atan2(velocity.x, velocity.z) / Time.deltaTime);
            
            // 检查轨迹是否安全
            foreach (Vector3 point in trajectory)
            {
                // 检查附近是否有障碍物
                Collider[] obstacles = Physics.OverlapSphere(point, ObstacleInflation, obstacleLayer);
                if (obstacles.Length > 0)
                {
                    return false;
                }
            }
            
            return true;
        }

        // 更精确的轨迹预测，考虑转向动态特性
        private Vector3[] PredictTrajectory(Vector3 startPos, Vector3 startVelocity, float speed, float angularVelocity)
        {
            int steps = 20;
            Vector3[] trajectory = new Vector3[steps];
            
            Vector3 position = startPos;
            Vector3 velocity = startVelocity;
            float deltaTime = PredictionTime / steps;
            
            for (int i = 0; i < steps; i++)
            {
                position += velocity * deltaTime;
                
                // 简单模拟转向
                float angle = Mathf.Atan2(velocity.x, velocity.z) + angularVelocity * deltaTime;
                velocity = new Vector3(Mathf.Sin(angle), 0, Mathf.Cos(angle)) * speed;
                
                trajectory[i] = position;
            }
            
            return trajectory;
        }


        private float EvaluateVelocity(Vector3 currentPosition, Vector3 velocity, Vector3 goalPosition, LayerMask obstacleLayer)
        {
            // 方向评分：速度方向与目标方向的匹配度
            float headingScore = Vector3.Dot((goalPosition - currentPosition).normalized, velocity.normalized);
            headingScore = Mathf.Max(0, headingScore); // 确保不为负
            
            // 距离评分：鼓励接近目标
            float distScore = 1.0f / (1.0f + Vector3.Distance(
                currentPosition + velocity.normalized * PredictionTime, 
                goalPosition));
            
            // 速度评分：鼓励保持较高速度
            float velocityScore = velocity.magnitude / MaxSpeed;
            
            // 障碍物评分：远离障碍物
            float obstacleScore = EvaluateObstacleDistance(currentPosition, velocity, obstacleLayer);
            
            // 总分
            return HeadingWeight * headingScore + 
                   DistanceWeight * distScore + 
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
