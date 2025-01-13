using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;

namespace Map
{
    // 表示时间轴上的一段占用时间
    // 记录了这段时间的起止时间以及占用者的标识信息
    public class TimeOccupancy
    {
        public float StartTime { get; private set; }
        public float EndTime { get; private set; }
        public OccupancyIdentifier Occupier { get; private set; }

        public TimeOccupancy(float startTime, float endTime, OccupancyIdentifier occupier)
        {
            StartTime = startTime;
            EndTime = endTime;
            Occupier = occupier;
        }

        public void UpdateEndTime(float newEndTime)
        {
            if (newEndTime < StartTime)
            {
                throw new ArgumentException("New end time cannot be earlier than start time");
            }
            EndTime = newEndTime;
        }

        // 检查此占用时间段是否与给定的时间段有重叠
        public bool OverlapsWith(float start, float end)
        {
            return !(EndTime <= start || StartTime >= end);
        }
    }

    // 管理单个CellSpace的时间轴, 维护这个空间内所有的占用记录
    // 每个CellSpace可以同时被多个机器人占用, 每个机器人也可以多次占用同一个CellSpace
    public class CellSpaceTimeline
    {
        private string cellSpaceId;
        private List<TimeOccupancy> occupancyPeriods;
        private Dictionary<string, TimeOccupancy> ongoingOccupancies;

        public CellSpaceTimeline(string id)
        {
            cellSpaceId = id;
            occupancyPeriods = new List<TimeOccupancy>();
            ongoingOccupancies = new Dictionary<string, TimeOccupancy>();
        }

        // 添加一个新的占用时间段
        public void AddOccupancy(float startTime, float endTime, OccupancyIdentifier occupier)
        {
            occupancyPeriods.Add(new TimeOccupancy(startTime, endTime, occupier));
            // 保持时间顺序, 便于后续查询
            occupancyPeriods = occupancyPeriods.OrderBy(p => p.StartTime).ToList();
        }

        // 更新当前占用情况这个方法处理机器人的实时位置更新
        // 它会维护一段连续的时间记录, 而不是创建多个独立的时间点
        public void UpdateCurrentOccupancy(float currentTime, OccupancyIdentifier occupier)
        {
            string robotId = occupier.RobotId;
            
            if (ongoingOccupancies.TryGetValue(robotId, out TimeOccupancy currentOccupancy))
            {
                // 更新现有占用的结束时间
                currentOccupancy.UpdateEndTime(currentTime);
            }
            else
            {
                // 开始新的占用记录
                var newOccupancy = new TimeOccupancy(currentTime, currentTime, occupier);
                occupancyPeriods.Add(newOccupancy);
                ongoingOccupancies[robotId] = newOccupancy;
            }
        }

        // 结束指定机器人的当前占用
        public void EndCurrentOccupancy(string robotId, float endTime)
        {
            if (ongoingOccupancies.TryGetValue(robotId, out TimeOccupancy currentOccupancy))
            {
                currentOccupancy.UpdateEndTime(endTime);
                ongoingOccupancies.Remove(robotId);
            }
        }

        // 添加一个计划的占用时间段
        public void AddPlannedOccupancy(float startTime, float endTime, OccupancyIdentifier occupier)
        {
            var newOccupancy = new TimeOccupancy(startTime, endTime, occupier);
            occupancyPeriods.Add(newOccupancy);
            occupancyPeriods = occupancyPeriods.OrderBy(p => p.StartTime).ToList();
        }

        public List<TimeOccupancy> GetOccupanciesInRange(float startTime, float endTime)
        {
            return occupancyPeriods
                .Where(period => period.OverlapsWith(startTime, endTime))
                .ToList();
        }

        public void CleanupBefore(float time)
        {
            occupancyPeriods.RemoveAll(period => period.EndTime < time);
        }

        // 清除指定机器人的计划占用记录
        public void ClearPlannedOccupancy(OccupancyIdentifier identifier, float fromTime)
        {
            occupancyPeriods.RemoveAll(period => 
                period.StartTime >= fromTime && 
                period.Occupier.RobotId == identifier.RobotId &&
                period.Occupier.Type == OccupancyType.Planned &&
                (identifier.PlanId == null || period.Occupier.PlanId == identifier.PlanId));
        }

        // 清除指定时间段内的占用记录
        public void ClearOccupancyInRange(float startTime, float endTime, string robotId = null)
        {
            // 找到需要清理的占用记录
            var recordsToRemove = occupancyPeriods
                .Where(period => 
                    period.OverlapsWith(startTime, endTime) &&
                    (robotId == null || period.Occupier.RobotId == robotId))
                .ToList();

            // 从列表中移除这些记录
            foreach (var record in recordsToRemove)
            {
                occupancyPeriods.Remove(record);
            }

            // 如果清理的是当前正在进行的占用，也需要更新ongoingOccupancies
            if (robotId != null && ongoingOccupancies.ContainsKey(robotId))
            {
                ongoingOccupancies.Remove(robotId);
            }
        }
    }

    // 动态占用层的主类, 管理整个地图中所有CellSpace的时间占用信息
    // 支持实时位置更新和路径规划预订, 并提供时间窗口管理机制
    public class DynamicOccupancyLayer
    {
        private IndoorSpace indoorSpace;
        private Dictionary<string, CellSpaceTimeline> cellSpaceTimelines;
        private float windowDuration;
        private float lastCleanupTime;
        private const float CLEANUP_INTERVAL = 60f;

        public DynamicOccupancyLayer(IndoorSpace indoorSpace, float windowDuration, float initialTime)
        {
            this.indoorSpace = indoorSpace;
            this.windowDuration = windowDuration;
            this.lastCleanupTime = initialTime;

            cellSpaceTimelines = new Dictionary<string, CellSpaceTimeline>();
            foreach (var cellSpace in indoorSpace.CellSpaces)
            {
                cellSpaceTimelines[cellSpace.Id] = new CellSpaceTimeline(cellSpace.Id);
            }
        }

        // 更新实时占用信息
        public void UpdateCurrentOccupancy(string cellSpaceId, float currentTime, string robotId)
        {
            if (!cellSpaceTimelines.TryGetValue(cellSpaceId, out CellSpaceTimeline timeline))
            {
                Debug.LogWarning($"CellSpace {cellSpaceId} not found");
                return;
            }

            var identifier = new OccupancyIdentifier(robotId);
            timeline.UpdateCurrentOccupancy(currentTime, identifier);
        }

        public void UpdateCurrentOccupancy(float currentTime, string robotId)
        {
            foreach (var timeline in cellSpaceTimelines.Values)
            {
                timeline.UpdateCurrentOccupancy(currentTime, new OccupancyIdentifier(robotId));
            }
        }

        public void EndCurrentOccupancy(string cellSpaceId, string robotId, float endTime)
        {
            if (cellSpaceTimelines.TryGetValue(cellSpaceId, out CellSpaceTimeline timeline))
            {
                timeline.EndCurrentOccupancy(robotId, endTime);
            }
        }

        // 设置计划路径的占用信息
        public void SetPlannedOccupancy(string cellSpaceId, float startTime, float endTime, string robotId, string planId)
        {
            if (!cellSpaceTimelines.TryGetValue(cellSpaceId, out CellSpaceTimeline timeline))
            {
                Debug.LogWarning($"CellSpace {cellSpaceId} not found");
                return;
            }

            var identifier = new OccupancyIdentifier(robotId, planId, OccupancyType.Planned);
            timeline.AddPlannedOccupancy(startTime, endTime, identifier);
        }

        // 基础的占用设置方法
        private void SetOccupancy(string cellSpaceId, float startTime, float endTime, OccupancyIdentifier identifier)
        {
            if (!cellSpaceTimelines.TryGetValue(cellSpaceId, out CellSpaceTimeline timeline))
            {
                Debug.LogWarning($"CellSpace {cellSpaceId} not found");
                return;
            }

            timeline.AddOccupancy(startTime, endTime, identifier);
        }
        
        // 清除指定计划的占用信息
        public void ClearPlannedOccupancy(string cellSpaceId, string robotId, string planId, float fromTime)
        {
            if (!cellSpaceTimelines.TryGetValue(cellSpaceId, out CellSpaceTimeline timeline))
            {
                return;
            }

            var identifier = new OccupancyIdentifier(robotId, planId, OccupancyType.Planned);
            timeline.ClearPlannedOccupancy(identifier, fromTime);
        }

        public void ClearPlannedOccupancy(string robotId, string planId, float fromTime)
        {
            foreach (var timeline in cellSpaceTimelines.Values)
            {
                timeline.ClearPlannedOccupancy(new OccupancyIdentifier(robotId, planId, OccupancyType.Planned), fromTime);
            }
        }

        public void ClearOccupancyInTimeWindow(float startTime, float endTime, string robotId = null)
        {
            foreach (var timeline in cellSpaceTimelines.Values)
            {
                timeline.ClearOccupancyInRange(startTime, endTime, robotId);
            }
        }

        // 获取指定时间段内的所有占用记录
        public List<TimeOccupancy> GetOccupancies(string cellSpaceId, float startTime, float endTime)
        {
            if (!cellSpaceTimelines.TryGetValue(cellSpaceId, out CellSpaceTimeline timeline))
            {
                return new List<TimeOccupancy>();
            }

            return timeline.GetOccupanciesInRange(startTime, endTime);
        }

        // 检查指定时间是否被占用
        public bool IsOccupied(string cellSpaceId, float time)
        {
            return GetOccupancies(cellSpaceId, time, time).Any();
        }

        // 更新系统时间并清理过期数据
        public void UpdateTime(float currentTime)
        {
            if (currentTime - lastCleanupTime > CLEANUP_INTERVAL)
            {
                float cleanupBefore = currentTime - windowDuration;
                foreach (var timeline in cellSpaceTimelines.Values)
                {
                    timeline.CleanupBefore(cleanupBefore);
                }
                lastCleanupTime = currentTime;
            }
        }

        // 主要用于合并个人层以获得全局层的占用信息
        public void MergeLayer(DynamicOccupancyLayer other, float startTime, float endTime)
        {
            foreach (var cellSpaceId in cellSpaceTimelines.Keys)
            {
                var otherOccupancies = other.GetOccupancies(cellSpaceId, startTime, endTime);
                foreach (var occupancy in otherOccupancies)
                {
                    SetOccupancy(
                        cellSpaceId,
                        occupancy.StartTime,
                        occupancy.EndTime,
                        occupancy.Occupier
                    );
                }
            }
        }

        // 获取所有CellSpace
        public IEnumerable<CellSpace> GetAllCellSpaces()
        {
            return indoorSpace.CellSpaces;
        }

        // 获取时间窗口长度
        public float GetWindowDuration() => windowDuration;

        // 获取开始时间
        public float GetStartTime() => lastCleanupTime - windowDuration;

        // 获取结束时间
        public float GetEndTime() => lastCleanupTime;
    }
}