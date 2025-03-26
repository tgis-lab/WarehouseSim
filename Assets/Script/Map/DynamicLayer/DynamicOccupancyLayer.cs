using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Map
{
    public class DynamicOccupancyLayer
    {
        // 每个CellSpace的时间轴索引
        private Dictionary<string, TemporalIndex> cellTimeIndices;
        
        // 边界占用的时间轴索引（用于边界冲突检测）
        private Dictionary<string, TemporalIndex> boundaryTimeIndices;
        
        // 全局窗口设置
        private float windowDuration = 60f;
        private float lastCleanupTime = 0f;
        
        private IndoorSpace indoorSpace;

        public DynamicOccupancyLayer(IndoorSpace indoorSpace, float windowDuration, float initialTime)
        {
            this.indoorSpace = indoorSpace;
            this.windowDuration = windowDuration;
            this.lastCleanupTime = initialTime;
            
            cellTimeIndices = new Dictionary<string, TemporalIndex>();
            boundaryTimeIndices = new Dictionary<string, TemporalIndex>();
            
            // 初始化每个CellSpace的时间索引
            if (indoorSpace != null && indoorSpace.CellSpaces != null)
            {
                foreach (var cellSpace in indoorSpace.CellSpaces)
                {
                    cellTimeIndices[cellSpace.Id] = new TemporalIndex();
                }
            }
        }
        
        public DynamicOccupancyLayer()
        {
            this.windowDuration = 60f;
            this.lastCleanupTime = Time.time;
            
            cellTimeIndices = new Dictionary<string, TemporalIndex>();
            boundaryTimeIndices = new Dictionary<string, TemporalIndex>();
        }

        // 获取或创建指定CellSpace的时间索引
        private TemporalIndex GetCellTimeIndex(string cellId)
        {
            if (!cellTimeIndices.TryGetValue(cellId, out var index))
            {
                index = new TemporalIndex();
                cellTimeIndices[cellId] = index;
            }
            return index;
        }
        
        // 获取或创建指定边界的时间索引
        private TemporalIndex GetBoundaryTimeIndex(string boundaryId)
        {
            if (!boundaryTimeIndices.TryGetValue(boundaryId, out var index))
            {
                index = new TemporalIndex();
                boundaryTimeIndices[boundaryId] = index;
            }
            return index;
        }

        // 更新CellSpace的当前占用状态
        public void UpdateCurrentOccupancy(string cellId, float currentTime, string robotId)
        {
            var timeIndex = GetCellTimeIndex(cellId);
            timeIndex.UpdateCurrentOccupancy(robotId, currentTime);
        }
        
        // 更新边界的当前占用状态
        public void UpdateBoundaryOccupancy(string boundaryId, float currentTime, string robotId)
        {
            var timeIndex = GetBoundaryTimeIndex(boundaryId);
            timeIndex.UpdateCurrentOccupancy(robotId, currentTime);
        }

        // 结束CellSpace的当前占用
        public void EndCurrentOccupancy(string cellId, string robotId, float endTime)
        {
            if (cellTimeIndices.TryGetValue(cellId, out var timeIndex))
            {
                timeIndex.EndCurrentOccupancy(robotId, endTime);
            }
        }
        
        // 结束边界的当前占用
        public void EndBoundaryOccupancy(string boundaryId, string robotId, float endTime)
        {
            if (boundaryTimeIndices.TryGetValue(boundaryId, out var timeIndex))
            {
                timeIndex.EndCurrentOccupancy(robotId, endTime);
            }
        }

        // 设置CellSpace的占用
        public void SetOccupancy(string cellId, float startTime, float endTime, OccupancyIdentifier occupier)
        {
            var timeIndex = GetCellTimeIndex(cellId);
            var occupancy = new TimeOccupancy(startTime, endTime, occupier);
            timeIndex.AddInterval(occupancy);
        }
        
        // 添加CellSpace的计划占用
        public void SetPlannedOccupancy(string cellId, float startTime, float endTime, string robotId, string planId = null)
        {
            if (string.IsNullOrEmpty(planId))
            {
                planId = $"Plan_{robotId}_{DateTime.Now.Ticks}";
            }
            
            var timeIndex = GetCellTimeIndex(cellId);
            timeIndex.AddPlannedOccupancy(startTime, endTime, robotId, planId);
        }
        
        // 添加边界的计划占用
        public void SetBoundaryPlannedOccupancy(string boundaryId, float startTime, float endTime, string robotId, string planId = null)
        {
            if (string.IsNullOrEmpty(planId))
            {
                planId = $"Plan_{robotId}_{DateTime.Now.Ticks}";
            }
            
            var timeIndex = GetBoundaryTimeIndex(boundaryId);
            timeIndex.AddPlannedOccupancy(startTime, endTime, robotId, planId);
        }

        // 设置动态占用
        public void SetDynamicOccupancy(string cellId, float startTime, float endTime, string obstacleId, string planId = null)
        {
            var timeIndex = GetCellTimeIndex(cellId);
            timeIndex.AddPlannedOccupancy(startTime, endTime, obstacleId, planId);
        }

        // 清除指定机器人的计划占用
        public void ClearPlannedOccupancy(string robotId, string planId, float fromTime)
        {
            // 清除CellSpace占用
            foreach (var timeIndex in cellTimeIndices.Values)
            {
                timeIndex.ClearPlannedOccupancy(robotId, planId, fromTime);
            }
            
            // 清除边界占用
            foreach (var timeIndex in boundaryTimeIndices.Values)
            {
                timeIndex.ClearPlannedOccupancy(robotId, planId, fromTime);
            }
        }
        
        // 清除指定机器人的计划占用（带cellId）
        public void ClearPlannedOccupancy(string cellId, string robotId, string planId, float fromTime)
        {
            if (cellTimeIndices.TryGetValue(cellId, out var timeIndex))
            {
                timeIndex.ClearPlannedOccupancy(robotId, planId, fromTime);
            }
        }

        // 清除指定时间窗口内的占用
        public void ClearOccupancyInTimeWindow(float startTime, float endTime, string excludeRobotId = null)
        {
            foreach (var entry in cellTimeIndices)
            {
                var timeIndex = entry.Value;
                var occupancies = timeIndex.CheckConflict(startTime, endTime, excludeRobotId);
                
                foreach (var occupancy in occupancies)
                {
                    timeIndex.RemoveInterval(occupancy);
                }
            }
        }

        // 获取指定CellSpace在时间段内的占用
        public List<TimeOccupancy> GetOccupancies(string cellId, float startTime, float endTime)
        {
            if (cellTimeIndices.TryGetValue(cellId, out var timeIndex))
            {
                return timeIndex.QueryOverlapping(startTime, endTime);
            }
            return new List<TimeOccupancy>();
        }

        // 检查CellSpace的占用冲突
        public List<TimeOccupancy> CheckCellConflict(string cellId, float startTime, float endTime, string excludeRobotId = null)
        {
            if (cellTimeIndices.TryGetValue(cellId, out var timeIndex))
            {
                return timeIndex.CheckConflict(startTime, endTime, excludeRobotId);
            }
            return new List<TimeOccupancy>();
        }
        
        // 检查边界的占用冲突
        public List<TimeOccupancy> CheckBoundaryConflict(string boundaryId, float startTime, float endTime, string excludeRobotId = null)
        {
            if (boundaryTimeIndices.TryGetValue(boundaryId, out var timeIndex))
            {
                return timeIndex.CheckConflict(startTime, endTime, excludeRobotId);
            }
            return new List<TimeOccupancy>();
        }

        // 合并另一个占用层（用于全局层集成个人层）
        public void MergeLayer(DynamicOccupancyLayer other, float startTime, float endTime)
        {
            foreach (var cellId in other.cellTimeIndices.Keys)
            {
                var otherOccupancies = other.GetOccupancies(cellId, startTime, endTime);
                
                foreach (var occupancy in otherOccupancies)
                {
                    // 添加到当前层
                    var timeIndex = GetCellTimeIndex(cellId);
                    var newOccupancy = new TimeOccupancy(
                        occupancy.StartTime,
                        occupancy.EndTime,
                        occupancy.Occupier);
                        
                    timeIndex.AddInterval(newOccupancy);
                }
            }
        }

        // 周期性清理旧数据
        public void UpdateTime(float currentTime)
        {
            if (currentTime - lastCleanupTime > 30f) // 每30秒清理一次
            {
                Cleanup(currentTime);
            }
        }
        
        // 清理旧数据
        private void Cleanup(float currentTime)
        {
            lastCleanupTime = currentTime;
            
            // 清理CellSpace时间索引
            foreach (var timeIndex in cellTimeIndices.Values)
            {
                timeIndex.Cleanup(currentTime, windowDuration);
            }
            
            // 清理边界时间索引
            foreach (var timeIndex in boundaryTimeIndices.Values)
            {
                timeIndex.Cleanup(currentTime, windowDuration);
            }
        }

        // 获取时间窗口开始时间
        public float GetStartTime() => lastCleanupTime - windowDuration;

        // 获取时间窗口结束时间
        public float GetEndTime() => lastCleanupTime;
        
        // 获取窗口持续时间
        public float GetWindowDuration() => windowDuration;
    }
}