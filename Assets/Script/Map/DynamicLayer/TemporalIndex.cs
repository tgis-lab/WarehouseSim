using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using C5;
// 使用别名避免冲突
using SystemCollections = System.Collections.Generic;

namespace Map
{
    public class TimeOccupancy : IComparable<TimeOccupancy>
    {
        public float StartTime { get; private set; }
        public float EndTime { get; private set; }
        public OccupancyIdentifier Occupier { get; private set; }
        public Guid Id { get; private set; }

        public TimeOccupancy(float startTime, float endTime, OccupancyIdentifier occupier)
        {
            if (endTime < startTime)
            {
                throw new ArgumentException("End time must be greater than or equal to start time");
            }

            StartTime = startTime;
            EndTime = endTime;
            Occupier = occupier;
            Id = Guid.NewGuid();
        }

        // 更新结束时间（用于实时跟踪）
        public void UpdateEndTime(float newEndTime)
        {
            if (newEndTime < StartTime)
            {
                throw new ArgumentException("New end time cannot be less than start time");
            }
            EndTime = newEndTime;
        }

        // 检查是否与给定的时间段重叠
        public bool OverlapsWith(float startTime, float endTime)
        {
            // 不重叠的情况：一个区间结束于另一个开始之前
            return !(EndTime <= startTime || StartTime >= endTime);
        }

        public override string ToString()
        {
            return $"TimeOccupancy({StartTime:F2}-{EndTime:F2}, {Occupier})";
        }

        // IComparable接口实现，用于C5集合
        public int CompareTo(TimeOccupancy other)
        {
            if (other == null) return 1;
            
            // 首先比较开始时间
            int startComparison = StartTime.CompareTo(other.StartTime);
            if (startComparison != 0) return startComparison;
            
            // 开始时间相同，比较结束时间
            int endComparison = EndTime.CompareTo(other.EndTime);
            if (endComparison != 0) return endComparison;
            
            // 时间完全相同，使用ID比较确保唯一性
            return Id.CompareTo(other.Id);
        }
    }

    /// <summary>
    /// 使用C5库实现的时间索引
    /// 基于中点排序的区间树实现
    /// </summary>
    public class TemporalIndex
    {
        // 存储所有占用的列表，用于快速迭代
        private readonly HashDictionary<Guid, TimeOccupancy> allOccupancies;
        
        // 按开始时间排序的占用索引
        private readonly TreeDictionary<float, HashedLinkedList<TimeOccupancy>> startTimeIndex;
        
        // 按结束时间排序的占用索引
        private readonly TreeDictionary<float, HashedLinkedList<TimeOccupancy>> endTimeIndex;
        
        // 当前进行中的占用
        private readonly Dictionary<string, TimeOccupancy> ongoingOccupancies;

        public TemporalIndex()
        {
            allOccupancies = new HashDictionary<Guid, TimeOccupancy>();
            startTimeIndex = new TreeDictionary<float, HashedLinkedList<TimeOccupancy>>();
            endTimeIndex = new TreeDictionary<float, HashedLinkedList<TimeOccupancy>>();
            ongoingOccupancies = new Dictionary<string, TimeOccupancy>();
        }

        // 添加一个时间区间到索引
        public void AddInterval(TimeOccupancy interval)
        {
            // 添加到主字典
            allOccupancies[interval.Id] = interval;
            
            // 添加到开始时间索引
            if (!startTimeIndex.Contains(interval.StartTime))
            {
                startTimeIndex[interval.StartTime] = new HashedLinkedList<TimeOccupancy>();
            }
            startTimeIndex[interval.StartTime].Add(interval);
            
            // 添加到结束时间索引
            if (!endTimeIndex.Contains(interval.EndTime))
            {
                endTimeIndex[interval.EndTime] = new HashedLinkedList<TimeOccupancy>();
            }
            endTimeIndex[interval.EndTime].Add(interval);
        }

        // 从索引中移除一个时间区间
        public void RemoveInterval(TimeOccupancy interval)
        {
            if (allOccupancies.Contains(interval.Id))
            {
                // 从主字典中移除
                allOccupancies.Remove(interval.Id);
                
                // 从开始时间索引中移除
                if (startTimeIndex.Contains(interval.StartTime))
                {
                    var list = startTimeIndex[interval.StartTime];
                    list.Remove(interval);
                    if (list.Count == 0)
                    {
                        startTimeIndex.Remove(interval.StartTime);
                    }
                }
                
                // 从结束时间索引中移除
                if (endTimeIndex.Contains(interval.EndTime))
                {
                    var list = endTimeIndex[interval.EndTime];
                    list.Remove(interval);
                    if (list.Count == 0)
                    {
                        endTimeIndex.Remove(interval.EndTime);
                    }
                }
            }
        }

        // 查询与给定时间段重叠的所有区间
        public SystemCollections.List<TimeOccupancy> QueryOverlapping(float startTime, float endTime)
        {
            var result = new SystemCollections.List<TimeOccupancy>();
            
            // 使用已有的索引结构进行高效查询，但避免使用Until方法
            if (startTimeIndex.Count > 0)
            {
                // 获取所有可能重叠的区间
                var potentialOverlaps = new SystemCollections.HashSet<TimeOccupancy>();
                
                // 手动迭代，而不是使用Until
                var enumerator = startTimeIndex.RangeFrom(startTimeIndex.FindMin().Key).GetEnumerator();
                while (enumerator.MoveNext())
                {
                    var entry = enumerator.Current;
                    if (entry.Key > endTime)
                        break;  // 如果开始时间大于查询结束时间，可以停止
                    
                    foreach (var interval in entry.Value)
                    {
                        potentialOverlaps.Add(interval);
                    }
                }
                
                // 筛选出确实重叠的区间
                foreach (var interval in potentialOverlaps)
                {
                    if (interval.EndTime >= startTime)
                    {
                        result.Add(interval);
                    }
                }
            }
            
            return result;
        }

        // 更新进行中的占用
        public void UpdateCurrentOccupancy(string robotId, float currentTime)
        {
            string key = robotId;
            
            if (ongoingOccupancies.TryGetValue(key, out TimeOccupancy current))
            {
                // 从索引中移除（因为结束时间会变）
                RemoveInterval(current);
                
                // 更新结束时间
                current.UpdateEndTime(currentTime);
                
                // 重新添加到索引
                AddInterval(current);
            }
            else
            {
                // 创建新的占用记录
                var occupier = new OccupancyIdentifier(robotId);
                var newOccupancy = new TimeOccupancy(currentTime, currentTime, occupier);
                ongoingOccupancies[key] = newOccupancy;
                AddInterval(newOccupancy);
            }
        }

        // 结束当前占用
        public void EndCurrentOccupancy(string robotId, float endTime)
        {
            string key = robotId;
            
            if (ongoingOccupancies.TryGetValue(key, out TimeOccupancy current))
            {
                // 从索引中移除
                RemoveInterval(current);
                
                // 更新结束时间
                current.UpdateEndTime(endTime);
                
                // 重新添加到索引
                AddInterval(current);
                
                // 从当前占用中移除
                ongoingOccupancies.Remove(key);
            }
        }

        // 添加计划的占用
        public void AddPlannedOccupancy(float startTime, float endTime, string robotId, string planId)
        {
            var occupier = new OccupancyIdentifier(robotId, planId, OccupancyType.Planned);
            var occupancy = new TimeOccupancy(startTime, endTime, occupier);
            AddInterval(occupancy);
        }

        // 清除指定机器人的计划占用
        public void ClearPlannedOccupancy(string robotId, string planId, float fromTime)
        {
            var toRemove = new SystemCollections.List<TimeOccupancy>();
            
            // 找出需要删除的占用
            foreach (var occupancy in allOccupancies.Values)
            {
                if (occupancy.Occupier.RobotId == robotId &&
                    occupancy.Occupier.Type == OccupancyType.Planned &&
                    (planId == null || occupancy.Occupier.PlanId == planId) &&
                    occupancy.StartTime >= fromTime)
                {
                    toRemove.Add(occupancy);
                }
            }
            
            // 移除找到的占用
            foreach (var occupancy in toRemove)
            {
                RemoveInterval(occupancy);
            }
        }

        // 检查占用冲突
        public SystemCollections.List<TimeOccupancy> CheckConflict(float startTime, float endTime, string excludeRobotId = null)
        {
            var overlapping = QueryOverlapping(startTime, endTime);
            
            // 过滤掉指定机器人的占用
            if (!string.IsNullOrEmpty(excludeRobotId))
            {
                return overlapping.Where(o => o.Occupier.RobotId != excludeRobotId).ToList();
            }
            
            return overlapping;
        }

        // 清理过期数据
        public void Cleanup(float currentTime, float windowDuration)
        {
            float cleanupThreshold = currentTime - windowDuration;
            
            var toRemove = new SystemCollections.List<TimeOccupancy>();
            
            // 手动迭代查找已过期的占用
            if (endTimeIndex.Count > 0)
            {
                var enumerator = endTimeIndex.RangeFrom(endTimeIndex.FindMin().Key).GetEnumerator();
                while (enumerator.MoveNext())
                {
                    var entry = enumerator.Current;
                    
                    // 如果结束时间超过了清理阈值，可以停止
                    if (entry.Key >= cleanupThreshold)
                        break;
                    
                    // 添加所有结束时间早于清理阈值的占用
                    foreach (var interval in entry.Value)
                    {
                        toRemove.Add(interval);
                    }
                }
            }
            
            // 移除过期数据
            foreach (var interval in toRemove)
            {
                RemoveInterval(interval);
            }
        }
    }
}