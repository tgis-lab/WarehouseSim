using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace PathPlanning
{
    public class DWAPlanner
    {
        public float MaxSpeed { get; set; } = 2.0f;
        public float MinSpeed { get; set; } = 0.5f;
        public float MaxRotSpeed { get; set; } = 1.0f;
        public float MaxAccel { get; set; } = 3.0f;
        public float VelocityResolution { get; set; } = 0.1f;
        public float RotationResolution { get; set; } = 0.1f;
        public float PredictionTime { get; set; } = 1.0f;
        public float HeadingWeight { get; set; } = 0.4f;
        public float DistanceWeight { get; set; } = 0.3f;
        public float VelocityWeight { get; set; } = 0.2f;
        public float ObstacleWeight { get; set; } = 0.1f;
        public float DWAScoreThreshold { get; set; } = 0.3f;
        public LayerMask ObstacleLayer { get; set; }

        public (Vector3 velocity, bool needReplan) CalculateVelocity(
            Vector3 currentPosition, 
            Vector3 currentVelocity, 
            Vector3 goalPosition)
        {
            Vector3 bestVelocity = currentVelocity;
            float bestScore = float.MinValue;
            bool needReplan = false;

            for (float speed = MinSpeed; speed <= MaxSpeed; speed += VelocityResolution)
            {
                for (float rot = -MaxRotSpeed; rot <= MaxRotSpeed; rot += RotationResolution)
                {
                    Vector3 predictedVelocity = CalculatePredictedVelocity(currentPosition, goalPosition, speed, rot);
                    if (!IsVelocitySafe(currentPosition, predictedVelocity, ObstacleLayer))
                    {
                        continue;
                    }

                    float score = EvaluateVelocity(currentPosition, predictedVelocity, goalPosition, ObstacleLayer);
                    if (score > bestScore)
                    {
                        bestScore = score;
                        bestVelocity = predictedVelocity;
                    }
                }
            }

            if (bestScore < DWAScoreThreshold)
            {
                needReplan = true;
            }

            return (bestVelocity, needReplan);
        }

        private Vector3 CalculatePredictedVelocity(Vector3 currentPosition, Vector3 goalPosition, float speed, float rotation)
        {
            Vector3 directionToGoal = (goalPosition - currentPosition).normalized;
            return Quaternion.Euler(0, rotation * Mathf.Rad2Deg, 0) * directionToGoal * speed;
        }

        private float EvaluateObstacleDistance(Vector3 currentPosition, Vector3 velocity, LayerMask obstacleLayer)
        {
            RaycastHit hit;
            if (Physics.Raycast(currentPosition, velocity.normalized, out hit, PredictionTime * velocity.magnitude, obstacleLayer))
            {
                return hit.distance / (PredictionTime * velocity.magnitude);
            }
            return 1.0f;
        }

        private bool IsVelocitySafe(Vector3 currentPosition, Vector3 velocity, LayerMask obstacleLayer)
        {
            Vector3 predictedPosition = currentPosition + velocity * PredictionTime;
            return !Physics.Raycast(currentPosition, velocity.normalized, velocity.magnitude * PredictionTime, obstacleLayer);
        }

        private float EvaluateVelocity(Vector3 currentPosition, Vector3 velocity, Vector3 goalPosition, LayerMask obstacleLayer)
        {
            Vector3 predictedPosition = currentPosition + velocity * PredictionTime;
            
            float headingScore = Vector3.Dot((goalPosition - predictedPosition).normalized, velocity.normalized);
            
            float distanceScore = 1.0f / (1.0f + Vector3.Distance(predictedPosition, goalPosition));
            
            float normalizedSpeed = velocity.magnitude / MaxSpeed;
            float velocityScore = normalizedSpeed;

            if (normalizedSpeed > 0.8f)
            {
                velocityScore *= 1.5f;
            }
            else if (normalizedSpeed < 0.3f)
            {
                velocityScore *= 0.5f;
            }

            velocityScore = Mathf.Min(velocityScore, 1.0f);

            float obstacleScore = EvaluateObstacleDistance(currentPosition, velocity, ObstacleLayer);

            return HeadingWeight * headingScore + DistanceWeight * distanceScore + VelocityWeight * velocityScore + ObstacleWeight * obstacleScore;
        }
    }
}
