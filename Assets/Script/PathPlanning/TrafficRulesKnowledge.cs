using System;
using System.Collections.Generic;
using UnityEngine;
using Map;
using NetTopologySuite.Geometries;

namespace PathPlanning
{
    /// <summary>
    /// 交通规则知识库
    /// </summary>
    public class TrafficRuleKnowledge
    {
        // 不同区域的速度限制
        private readonly Dictionary<string, float> areaSpeedLimits = new Dictionary<string, float>
        {
            { "aisle", 1.0f },           // 通道
            { "intersection", 0.7f },    // 交叉路口
            { "queue", 0.6f },           // 队列区
            { "picking_point", 0.5f },   // 拣货点
            { "packing_point", 0.5f }    // 包装点
        };
        
        // 转弯的速度调整
        private readonly Dictionary<float, float> turnSpeedFactors = new Dictionary<float, float>
        {
            { 30f, 0.9f },   // 小角度转弯速度降为90%
            { 60f, 0.7f },   // 中等角度转弯速度降为70%
            { 90f, 0.5f },   // 直角转弯速度降为50%
            { 120f, 0.4f },  // 大角度转弯速度降为40%
            { 180f, 0.3f }   // U型转弯速度降为30%
        };
        
        private IndoorSpace indoorSpace;
        private Graph graph;
        
        public TrafficRuleKnowledge(IndoorSpace indoorSpace, Graph graph)
        {
            this.indoorSpace = indoorSpace;
            this.graph = graph;
        }
        
        /// <summary>
        /// 根据交通规则修改速度和时间
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
            // 获取交通规则影响下的速度限制
            float speedLimit = GetSpeedLimit(current, next);
            
            // 检查转弯影响
            float turnFactor = 1.0f;
            if (prev != null)
            {
                turnFactor = GetTurnSpeedFactor(prev, current, next);
            }
            
            // 应用转弯和区域速度限制
            float targetSpeed = defaultSpeed * Math.Min(speedLimit, turnFactor);
            
            // 计算加速/减速所需时间
            float timeToReachTarget = 0;
            
            if (targetSpeed > currentSpeed)
            {
                // 加速
                float accelerationTime = (targetSpeed - currentSpeed) / maxAcceleration;
                float accelerationDistance = (currentSpeed + targetSpeed) * accelerationTime / 2;
                
                if (accelerationDistance > distance)
                {
                    // 距离不足以达到目标速度
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
                // 减速
                float decelerationTime = (currentSpeed - targetSpeed) / maxDeceleration;
                float decelerationDistance = (currentSpeed + targetSpeed) * decelerationTime / 2;
                
                if (decelerationDistance > distance)
                {
                    // 距离不足以减速到目标速度
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
                // 匀速
                timeToReachTarget = distance / currentSpeed;
            }
            
            return (targetSpeed, timeToReachTarget);
        }
        
        /// <summary>
        /// 获取区域速度限制
        /// </summary>
        public float GetSpeedLimit(RoutePoint current, RoutePoint next)
        {
            // 获取相关的CellSpace
            CellSpace currentCell = indoorSpace.GetCellSpaceFromConnectionPoint(current.ConnectionPoint, false);
            CellSpace nextCell = indoorSpace.GetCellSpaceFromConnectionPoint(next.ConnectionPoint, true);
            
            if (currentCell == null || nextCell == null)
                return 1.0f;
                
            // 检查交叉路口
            bool isIntersection = IsIntersection(current, next);
            if (isIntersection)
                return areaSpeedLimits["intersection"];
                
            // 检查拣货点
            if (nextCell.IsPickingPoint())
                return areaSpeedLimits["picking_point"];
                
            // 检查包装点
            if (nextCell.IsPackingPoint())
                return areaSpeedLimits["packing_point"];
                
            // 检查队列区
            if (nextCell.IsQueue())
                return areaSpeedLimits["queue"];
                
            // 默认使用通道速度
            return areaSpeedLimits["aisle"];
        }
        
        /// <summary>
        /// 获取转弯速度因子
        /// </summary>
        public float GetTurnSpeedFactor(RoutePoint prev, RoutePoint current, RoutePoint next)
        {
            // 计算转弯角度
            Vector3 prevPos = GetPointPosition(prev);
            Vector3 currentPos = GetPointPosition(current);
            Vector3 nextPos = GetPointPosition(next);
            
            Vector3 inDirection = (currentPos - prevPos).normalized;
            Vector3 outDirection = (nextPos - currentPos).normalized;
            
            float angle = Vector3.Angle(inDirection, outDirection);
            
            // 应用转弯速度因子
            foreach (var entry in turnSpeedFactors)
            {
                if (angle <= entry.Key)
                    return entry.Value;
            }
            
            return 0.3f; // 默认值，用于极端角度
        }
        
        /// <summary>
        /// 判断是否在交叉路口
        /// </summary>
        private bool IsIntersection(RoutePoint current, RoutePoint next)
        {
            try
            {
                // 使用Graph的PointLocation枚举判断位置类型
                var locationType = graph.GetPointLocation(current.ConnectionPoint);
                return locationType == Graph.PointLocation.InIntersection || 
                       locationType == Graph.PointLocation.ApproachingIntersection;
            }
            catch
            {
                // 如果出现异常，保守返回false
                return false;
            }
        }
        
        /// <summary>
        /// 计算加速情况下能达到的最大速度
        /// </summary>
        private float CalculateMaxReachableSpeed(float initialSpeed, float acceleration, float distance)
        {
            // v^2 = u^2 + 2as
            return Mathf.Sqrt(initialSpeed * initialSpeed + 2 * acceleration * distance);
        }
        
        /// <summary>
        /// 计算减速情况下能降低到的最小速度
        /// </summary>
        private float CalculateMinReachableSpeed(float initialSpeed, float deceleration, float distance)
        {
            // v^2 = u^2 - 2as
            float speedSquared = initialSpeed * initialSpeed - 2 * deceleration * distance;
            return speedSquared > 0 ? Mathf.Sqrt(speedSquared) : 0;
        }
        
        /// <summary>
        /// 计算移动指定距离所需的时间
        /// </summary>
        private float CalculateTimeForDistance(float initialSpeed, float finalSpeed, float acceleration, float distance)
        {
            // 使用平均速度公式：s = t * (u + v) / 2
            // 因此 t = 2s / (u + v)
            if (Mathf.Approximately(initialSpeed + finalSpeed, 0))
                return 0;
                
            return 2 * distance / (initialSpeed + finalSpeed);
        }
        
        /// <summary>
        /// 获取点的位置
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