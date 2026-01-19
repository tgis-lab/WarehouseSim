using System;
using System.Collections.Generic;
using UnityEngine;
using Map;
using NetTopologySuite.Geometries;

namespace PathPlanning
{
    /// <summary>
    /// Traffic Rule Knowledge Base
    /// </summary>
    public class TrafficRuleKnowledge
    {
        // Speed limits for different areas
        private readonly Dictionary<string, float> areaSpeedLimits = new Dictionary<string, float>
        {
            { "aisle", 1.0f },           // Aisle
            { "intersection", 0.7f },    // Intersection
            { "queue", 0.6f },           // Queue area
            { "picking_point", 0.5f },   // Picking point
            { "packing_point", 0.5f }    // Packing point
        };
        
        // Speed adjustments for turns
        private readonly Dictionary<float, float> turnSpeedFactors = new Dictionary<float, float>
        {
            { 30f, 0.9f },   // Small angle turn speed reduced to 90%
            { 60f, 0.7f },   // Medium angle turn speed reduced to 70%
            { 90f, 0.5f },   // Right angle turn speed reduced to 50%
            { 120f, 0.4f },  // Large angle turn speed reduced to 40%
            { 180f, 0.3f }   // U-turn speed reduced to 30%
        };
        
        private IndoorSpace indoorSpace;
        private Graph graph;
        
        public TrafficRuleKnowledge(IndoorSpace indoorSpace, Graph graph)
        {
            this.indoorSpace = indoorSpace;
            this.graph = graph;
        }
        
        /// <summary>
        /// Modifies speed and time based on traffic rules
        /// </summary>
        public (float modifiedSpeed, float modifiedTime) ModifySpeed(
            RoutePoint current, 
            RoutePoint next,
            float currentSpeed, 
            float defaultSpeed, 
            float distance,
            float maxAcceleration, 
            float maxDeceleration,
            RoutePoint prev = null)
        {
            // Get speed limit influenced by traffic rules
            float speedLimit = GetSpeedLimit(current, next);
            
            // Check turn influence
            float turnFactor = 1.0f;
            if (prev != null)
            {
                turnFactor = GetTurnSpeedFactor(prev, current, next);
            }
            
            // Apply turn and area speed limits
            float targetSpeed = defaultSpeed * Math.Min(speedLimit, turnFactor);
            
            // Calculate time required for acceleration/deceleration
            float timeToReachTarget = 0;
            
            if (targetSpeed > currentSpeed)
            {
                // Accelerate
                float accelerationTime = (targetSpeed - currentSpeed) / maxAcceleration;
                float accelerationDistance = (currentSpeed + targetSpeed) * accelerationTime / 2;
                
                if (accelerationDistance > distance)
                {
                    // Distance insufficient to reach target speed
                    float maxReachableSpeed = CalculateMaxReachableSpeed(
                        currentSpeed, maxAcceleration, distance);
                    targetSpeed = maxReachableSpeed;
                    timeToReachTarget = CalculateTimeForDistance(
                        currentSpeed, targetSpeed, maxAcceleration, distance);
                }
                else
                {
                    float remainingDistance = distance - accelerationDistance;
                    float constantSpeedTime = remainingDistance / targetSpeed;
                    timeToReachTarget = accelerationTime + constantSpeedTime;
                }
            }
            else if (targetSpeed < currentSpeed)
            {
                // Decelerate
                float decelerationTime = (currentSpeed - targetSpeed) / maxDeceleration;
                float decelerationDistance = (currentSpeed + targetSpeed) * decelerationTime / 2;
                
                if (decelerationDistance > distance)
                {
                    // Distance insufficient to decelerate to target speed
                    float lowestReachableSpeed = CalculateMinReachableSpeed(
                        currentSpeed, maxDeceleration, distance);
                    targetSpeed = lowestReachableSpeed;
                    timeToReachTarget = CalculateTimeForDistance(
                        currentSpeed, targetSpeed, maxDeceleration, distance);
                }
                else
                {
                    float remainingDistance = distance - decelerationDistance;
                    float constantSpeedTime = remainingDistance / targetSpeed;
                    timeToReachTarget = decelerationTime + constantSpeedTime;
                }
            }
            else
            {
                // Constant speed
                timeToReachTarget = distance / currentSpeed;
            }
            
            return (targetSpeed, timeToReachTarget);
        }
        
        /// <summary>
        /// Gets the speed limit for an area
        /// </summary>
        public float GetSpeedLimit(RoutePoint current, RoutePoint next)
        {
            // Get relevant CellSpaces
            CellSpace currentCell = indoorSpace.GetCellSpaceFromConnectionPoint(current.ConnectionPoint, false);
            CellSpace nextCell = indoorSpace.GetCellSpaceFromConnectionPoint(next.ConnectionPoint, true);
            
            if (currentCell == null || nextCell == null)
                return 1.0f;
                
            // Check intersection
            bool isIntersection = IsIntersection(current, next);
            if (isIntersection)
                return areaSpeedLimits["intersection"];
                
            // Check picking point
            if (nextCell.IsPickingPoint())
                return areaSpeedLimits["picking_point"];
                
            // Check packing point
            if (nextCell.IsPackingPoint())
                return areaSpeedLimits["packing_point"];
                
            // Check queue area
            if (nextCell.IsQueue())
                return areaSpeedLimits["queue"];
                
            // Default to aisle speed
            return areaSpeedLimits["aisle"];
        }
        
        /// <summary>
        /// Gets the turn speed factor
        /// </summary>
        public float GetTurnSpeedFactor(RoutePoint prev, RoutePoint current, RoutePoint next)
        {
            // Calculate turn angle
            Vector3 prevPos = GetPointPosition(prev);
            Vector3 currentPos = GetPointPosition(current);
            Vector3 nextPos = GetPointPosition(next);
            
            Vector3 inDirection = (currentPos - prevPos).normalized;
            Vector3 outDirection = (nextPos - currentPos).normalized;
            
            float angle = Vector3.Angle(inDirection, outDirection);
            
            // Apply turn speed factor
            foreach (var entry in turnSpeedFactors)
            {
                if (angle <= entry.Key)
                    return entry.Value;
            }
            
            return 0.3f; // Default value for extreme angles
        }
        
        /// <summary>
        /// Determines if at an intersection
        /// </summary>
        private bool IsIntersection(RoutePoint current, RoutePoint next)
        {
            try
            {
                // Use Graph's PointLocation enum to determine location type
                var locationType = graph.GetPointLocation(current.ConnectionPoint);
                return locationType == Graph.PointLocation.InIntersection || 
                       locationType == Graph.PointLocation.ApproachingIntersection;
            }
            catch
            {
                // Return false conservatively on exception
                return false;
            }
        }
        
        /// <summary>
        /// Calculates max speed reachable under acceleration
        /// </summary>
        private float CalculateMaxReachableSpeed(float initialSpeed, float acceleration, float distance)
        {
            // v^2 = u^2 + 2as
            return Mathf.Sqrt(initialSpeed * initialSpeed + 2 * acceleration * distance);
        }
        
        /// <summary>
        /// Calculates min speed reachable under deceleration
        /// </summary>
        private float CalculateMinReachableSpeed(float initialSpeed, float deceleration, float distance)
        {
            // v^2 = u^2 - 2as
            float speedSquared = initialSpeed * initialSpeed - 2 * deceleration * distance;
            return speedSquared > 0 ? Mathf.Sqrt(speedSquared) : 0;
        }
        
        /// <summary>
        /// Calculates time required to move specified distance
        /// </summary>
        private float CalculateTimeForDistance(float initialSpeed, float finalSpeed, float acceleration, float distance)
        {
            // Using average speed formula: s = t * (u + v) / 2
            // Therefore t = 2s / (u + v)
            if (Mathf.Approximately(initialSpeed + finalSpeed, 0))
                return 0;
                
            return 2 * distance / (initialSpeed + finalSpeed);
        }
        
        /// <summary>
        /// Gets the position of a point
        /// </summary>
        private Vector3 GetPointPosition(RoutePoint point)
        {
            if (point == null || point.ConnectionPoint == null || point.ConnectionPoint.Point == null)
                return Vector3.zero;
                
            Point geometryPoint = (Point)point.ConnectionPoint.Point;
            return new Vector3((float)geometryPoint.X, 0, (float)geometryPoint.Y);
        }
    }
}