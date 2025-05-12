using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using C5;
// 使用别名避免冲突
using SystemCollections = System.Collections.Generic;

namespace Map
{
    /// <summary>
    /// 时间占用记录，表示特定时间段内的占用情况
    /// </summary>
    public class TimeOccupancy : IComparable<TimeOccupancy>
    {
        public float StartTime { get; private set; }
        public float EndTime { get; private set; }
        public OccupancyIdentifier Occupier { get; private set; }
        public Guid Id { get; private set; }
        public bool IsStartCell { get; private set; }
        public bool IsEndCell { get; private set; }

        public TimeOccupancy(float startTime, float endTime, OccupancyIdentifier occupier, bool isStartCell = false, bool isEndCell = false)
        {
            if (endTime < startTime)
            {
                throw new ArgumentException("End time must be greater than or equal to start time");
            }

            StartTime = startTime;
            EndTime = endTime;
            Occupier = occupier;
            Id = Guid.NewGuid();
            IsStartCell = isStartCell;
            IsEndCell = isEndCell;
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
    /// 时间索引 - 管理单一位置的时间占用
    /// 使用C5库实现的基于区间树概念的时间索引
    /// </summary>
    public class TemporalIndex
    {
        // 存储所有占用的字典，键为GUID，值为TimeOccupancy
        private readonly HashDictionary<Guid, TimeOccupancy> allOccupancies;
        
        // 按开始时间排序的占用索引
        private readonly TreeDictionary<float, HashedLinkedList<TimeOccupancy>> startTimeIndex;
        
        // 按结束时间排序的占用索引
        private readonly TreeDictionary<float, HashedLinkedList<TimeOccupancy>> endTimeIndex;
        
        // 当前进行中的占用
        private readonly Dictionary<string, TimeOccupancy> ongoingOccupancies;

        /// <summary>
        /// 创建新的时间索引
        /// </summary>
        public TemporalIndex()
        {
            allOccupancies = new HashDictionary<Guid, TimeOccupancy>();
            startTimeIndex = new TreeDictionary<float, HashedLinkedList<TimeOccupancy>>();
            endTimeIndex = new TreeDictionary<float, HashedLinkedList<TimeOccupancy>>();
            ongoingOccupancies = new Dictionary<string, TimeOccupancy>();
        }

        /// <summary>
        /// 添加时间区间到索引
        /// </summary>
        public void AddInterval(TimeOccupancy interval)
        {
            if (interval == null) return;
            
            // 添加到所有占用字典
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

        /// <summary>
        /// 从索引中移除时间区间
        /// </summary>
        public void RemoveInterval(TimeOccupancy interval)
        {
            if (interval == null) return;
            
            // 从所有占用字典移除
            if (allOccupancies.Contains(interval.Id))
            {
                allOccupancies.Remove(interval.Id);
                
                // 从开始时间索引移除
                if (startTimeIndex.Contains(interval.StartTime))
                {
                    var list = startTimeIndex[interval.StartTime];
                    list.Remove(interval);
                    
                    // 如果该时间点没有更多占用，删除整个条目
                    if (list.Count == 0)
                    {
                        startTimeIndex.Remove(interval.StartTime);
                    }
                }
                
                // 从结束时间索引移除
                if (endTimeIndex.Contains(interval.EndTime))
                {
                    var list = endTimeIndex[interval.EndTime];
                    list.Remove(interval);
                    
                    // 如果该时间点没有更多占用，删除整个条目
                    if (list.Count == 0)
                    {
                        endTimeIndex.Remove(interval.EndTime);
                    }
                }
            }
        }

        /// <summary>
        /// 查询与指定时间段重叠的所有占用
        /// </summary>
        public List<TimeOccupancy> QueryOverlapping(float startTime, float endTime)
        {
            List<TimeOccupancy> result = new List<TimeOccupancy>();
            
            // 提前检查特殊情况
            if (startTime > endTime || startTimeIndex.IsEmpty)
            {
                return result;
            }
            
            // 收集所有可能重叠的区间
            var potentialOverlaps = new SystemCollections.HashSet<TimeOccupancy>();
            
            try
            {
                // 遍历所有开始时间早于等于查询结束时间的区间
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
            catch (Exception ex)
            {
                Debug.LogError($"查询时间重叠时出错: {ex.Message}");
            }
            
            return result;
        }

        /// <summary>
        /// 检查时间冲突，可排除指定机器人的占用
        /// </summary>
        public List<TimeOccupancy> CheckConflict(
            float startTime, 
            float endTime, 
            string excludeRobotId = null, 
            bool isStartCell = false, 
            bool isEndCell = false)
        {
            // 查询重叠的占用
            var overlappingOccupancies = QueryOverlapping(startTime, endTime);
            var conflicts = new List<TimeOccupancy>();
            
            foreach (var occupancy in overlappingOccupancies)
            {
                // 排除指定机器人的占用
                if (!string.IsNullOrEmpty(excludeRobotId) && occupancy.Occupier.RobotId == excludeRobotId)
                {
                    continue;
                }
                
                // 应用特殊规则处理
                
                // 规则1: 如果当前检查的是起点单元格，我们只在特定情况下报告冲突
                if (isStartCell && !occupancy.IsEndCell)
                {
                    // 在起点允许与非终点的其他占用共存
                    continue;
                }
                
                // 规则2: 如果当前检查的是终点单元格，我们只在特定情况下报告冲突
                if (isEndCell && !occupancy.IsStartCell)
                {
                    // 在终点允许与非起点的其他占用共存
                    continue;
                }
                
                // 规则3: 如果被检查的占用是起点，而我们不是终点，则允许共存
                if (occupancy.IsStartCell && !isEndCell)
                {
                    continue;
                }
                
                // 规则4: 如果被检查的占用是终点，而我们不是起点，则允许共存
                if (occupancy.IsEndCell && !isStartCell)
                {
                    continue;
                }
                
                // 所有规则验证后仍存在，则为真实冲突
                conflicts.Add(occupancy);
            }
            
            return conflicts;
        }

        /// <summary>
        /// 更新进行中的占用
        /// </summary>
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

        /// <summary>
        /// 结束当前占用
        /// </summary>
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

        /// <summary>
        /// 获取此索引中的所有时间占用
        /// </summary>
        public List<TimeOccupancy> GetAllOccupancies()
        {
            return new List<TimeOccupancy>(allOccupancies.Values);
        }

        /// <summary>
        /// 移除指定时间之前的所有时间间隔
        /// </summary>
        public void RemoveIntervalsBefore(float cutoffTime)
        {
            // 查找早于cutoffTime的所有结束时间
            var endTimes = endTimeIndex.RangeFrom(float.MinValue).TakeWhile(kvp => kvp.Key <= cutoffTime).ToList();
            
            foreach (var kvp in endTimes)
            {
                float endTime = kvp.Key;
                var intervals = kvp.Value.ToList();
                
                foreach (var interval in intervals)
                {
                    // 移除这个时间间隔
                    RemoveInterval(interval);
                }
            }
        }

        /// <summary>
        /// 获取指定时间范围内的所有占用记录
        /// </summary>
        public List<TimeOccupancy> GetOccupancies(float startTime, float endTime)
        {
            return QueryOverlapping(startTime, endTime);
        }

        /// <summary>
        /// 清理过期数据
        /// </summary>
        public void Cleanup(float currentTime, float windowDuration)
        {
            float cutoffTime = currentTime - windowDuration;
            List<TimeOccupancy> toRemove = new List<TimeOccupancy>();
            
            try
            {
                // 查找所有结束时间早于阈值的占用
                if (!endTimeIndex.IsEmpty)
                {
                    var minKey = endTimeIndex.FindMin().Key;
                    var maxKey = cutoffTime;
                    
                    if (minKey <= maxKey)
                    {
                        var enumerator = endTimeIndex.RangeFrom(minKey).GetEnumerator();
                        while (enumerator.MoveNext())
                        {
                            var entry = enumerator.Current;
                            if (entry.Key > cutoffTime)
                                break;
                                
                            foreach (var interval in entry.Value)
                            {
                                toRemove.Add(interval);
                            }
                        }
                    }
                }
                
                // 移除过期的占用
                foreach (var interval in toRemove)
                {
                    RemoveInterval(interval);
                }
                
                Debug.Log($"已清理 {toRemove.Count} 个过期占用记录");
            }
            catch (Exception ex)
            {
                Debug.LogError($"清理占用记录时出错: {ex.Message}");
            }
        }
    }
}