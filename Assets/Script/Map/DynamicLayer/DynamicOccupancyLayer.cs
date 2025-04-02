using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Map
{
    /// <summary>
    /// 动态占用层 - 管理多个CellSpace的时间占用情况
    /// </summary>
    public class DynamicOccupancyLayer
    {
        private IndoorSpace indoorSpace;
        private Dictionary<string, TemporalIndex> cellTimeIndices;
        private float windowDuration;
        private float lastCleanupTime;

        /// <summary>
        /// 创建新的动态占用层
        /// </summary>
        public DynamicOccupancyLayer(IndoorSpace indoorSpace, float windowDuration, float initialTime)
        {
            this.indoorSpace = indoorSpace;
            this.windowDuration = windowDuration;
            this.lastCleanupTime = initialTime;
            
            cellTimeIndices = new Dictionary<string, TemporalIndex>();
            
            // 为每个CellSpace初始化时间索引
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
        }

        /// <summary>
        /// 获取CellSpace的时间索引，如不存在则创建
        /// </summary>
        private TemporalIndex GetCellTimeIndex(string cellId)
        {
            if (string.IsNullOrEmpty(cellId))
            {
                return null;
            }
            
            if (!cellTimeIndices.ContainsKey(cellId))
            {
                cellTimeIndices[cellId] = new TemporalIndex();
            }
            
            return cellTimeIndices[cellId];
        }

        /// <summary>
        /// 清除计划占用
        /// </summary>
        public void ClearPlannedOccupancy(string robotId, string planId, float startTime, float endTime)
        {
            // 清除在指定时间范围内的占用
            foreach (var entry in cellTimeIndices)
            {
                string cellId = entry.Key;
                var timeIndex = entry.Value;
                
                var occupancies = timeIndex.QueryOverlapping(startTime, endTime);
                foreach (var occupancy in occupancies)
                {
                    if (occupancy.Occupier.RobotId == robotId && 
                        (string.IsNullOrEmpty(planId) || occupancy.Occupier.PlanId == planId))
                    {
                        timeIndex.RemoveInterval(occupancy);
                    }
                }
            }
        }

        /// <summary>
        /// 设置CellSpace的动态占用
        /// </summary>
        public void SetDynamicOccupancy(
            string cellId, 
            float startTime, 
            float endTime, 
            string robotId, 
            string planId = null,
            bool isStartCell = false,
            bool isEndCell = false)
        {
            var timeIndex = GetCellTimeIndex(cellId);
            if (timeIndex != null)
            {
                var occupier = new OccupancyIdentifier(robotId, planId);
                var occupancy = new TimeOccupancy(startTime, endTime, occupier, isStartCell, isEndCell);
                timeIndex.AddInterval(occupancy);
            }
        }

        /// <summary>
        /// 清除指定机器人和计划的占用
        /// </summary>
        public void ClearPlannedOccupancy(string robotId, string planId, float currentTime)
        {
            // 只清除从当前时间开始的占用，保留已过去的历史记录
            foreach (var entry in cellTimeIndices)
            {
                string cellId = entry.Key;
                var timeIndex = entry.Value;
                
                var occupancies = timeIndex.QueryOverlapping(currentTime, float.MaxValue);
                foreach (var occupancy in occupancies)
                {
                    if (occupancy.Occupier.RobotId == robotId && 
                        (string.IsNullOrEmpty(planId) || occupancy.Occupier.PlanId == planId))
                    {
                        timeIndex.RemoveInterval(occupancy);
                    }
                }
            }
        }

        /// <summary>
        /// 清除指定时间窗口内的所有占用
        /// </summary>
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

        /// <summary>
        /// 获取指定CellSpace在时间段内的占用
        /// </summary>
        public List<TimeOccupancy> GetOccupancies(string cellId, float startTime, float endTime)
        {
            if (cellTimeIndices.TryGetValue(cellId, out var timeIndex))
            {
                return timeIndex.QueryOverlapping(startTime, endTime);
            }
            return new List<TimeOccupancy>();
        }

        /// <summary>
        /// 检查CellSpace的占用冲突
        /// </summary>
        public List<TimeOccupancy> CheckCellConflict(
            string cellId, 
            float startTime, 
            float endTime, 
            string excludeRobotId = null, 
            bool isStartCell = false, 
            bool isEndCell = false)
        {
            if (cellTimeIndices.TryGetValue(cellId, out var timeIndex))
            {
                return timeIndex.CheckConflict(startTime, endTime, excludeRobotId, isStartCell, isEndCell);
            }
            return new List<TimeOccupancy>();
        }

        /// <summary>
        /// 更新CellSpace的实时占用
        /// </summary>
        public void UpdateCellOccupancy(string cellId, string robotId, float currentTime)
        {
            var timeIndex = GetCellTimeIndex(cellId);
            if (timeIndex != null)
            {
                timeIndex.UpdateCurrentOccupancy(robotId, currentTime);
            }
        }

        /// <summary>
        /// 结束CellSpace的占用
        /// </summary>
        public void EndCellOccupancy(string cellId, string robotId, float endTime)
        {
            if (cellTimeIndices.TryGetValue(cellId, out var timeIndex))
            {
                timeIndex.EndCurrentOccupancy(robotId, endTime);
            }
        }

        /// <summary>
        /// 将路径信息录入到动态地图层
        /// </summary>
        public void RecordPathOccupancy(
            IndoorSpace indoorSpace, 
            List<ConnectionPoint> connectionPointPath, 
            List<float> times, 
            string robotId, 
            string planId)
        {
            try
            {
                // 验证输入参数
                if (indoorSpace == null)
                {
                    Debug.LogError("Error: IndoorSpace is null");
                    return;
                }
                
                if (connectionPointPath == null || connectionPointPath.Count < 2)
                {
                    Debug.LogError("Error: ConnectionPointPath is null or too short");
                    return;
                }
                
                if (times == null || times.Count < connectionPointPath.Count)
                {
                    Debug.LogError("Error: Times array is null or too short");
                    return;
                }
                
                // 存储每个CellSpace的入口和出口时间
                Dictionary<string, float?> entryTimes = new Dictionary<string, float?>();
                Dictionary<string, float?> exitTimes = new Dictionary<string, float?>();
                HashSet<string> processedCellIds = new HashSet<string>();
                
                // 确定起点CellSpace
                ConnectionPoint startCP = connectionPointPath[0];
                if (startCP == null || startCP.CellBoundary == null)
                {
                    Debug.LogError("Error: First connection point or its boundary is null");
                    return;
                }
                
                // 获取起点CellSpace
                CellSpace startCellSpace = indoorSpace.GetCellSpaceFromId(startCP.CellBoundary.Source);
                if (startCellSpace == null)
                {
                    Debug.LogError($"Error: Could not find source CellSpace for boundary {startCP.CellBoundary.Id}");
                    return;
                }
                
                string startCellId = startCellSpace.Id;
                entryTimes[startCellId] = times[0];
                processedCellIds.Add(startCellId);
                
                // 处理路径中的所有连接点
                string currentCellId = startCellId;
                
                for (int i = 0; i < connectionPointPath.Count; i++)
                {
                    if (i >= times.Count)
                    {
                        Debug.LogWarning($"Error: Index {i} exceeds times array length {times.Count}");
                        break;
                    }
                    
                    ConnectionPoint cp = connectionPointPath[i];
                    if (cp == null || cp.CellBoundary == null)
                    {
                        Debug.LogWarning($"Error: Connection point at index {i} or its boundary is null");
                        continue;
                    }
                    
                    // 确定下一个CellSpace
                    string targetCellId = null;
                    
                    // 如果我们在源单元格中，下一个单元格是目标单元格
                    if (currentCellId == cp.CellBoundary.Source)
                    {
                        targetCellId = cp.CellBoundary.Target;
                    }
                    // 如果我们在目标单元格中，下一个单元格是源单元格
                    else if (currentCellId == cp.CellBoundary.Target)
                    {
                        targetCellId = cp.CellBoundary.Source;
                    }
                    // 否则，通过单元格空间查询确定
                    else
                    {
                        CellSpace nextCellSpace = indoorSpace.GetCellSpaceFromConnectionPoint(cp, false);
                        
                        if (nextCellSpace != null)
                        {
                            targetCellId = nextCellSpace.Id;
                        }
                        else
                        {
                            Debug.LogWarning($"Error: Could not determine target cell for connection point {cp.Id}");
                            continue;
                        }
                    }
                    
                    // 如果目标单元格已经确定并且与当前不同
                    if (!string.IsNullOrEmpty(targetCellId) && targetCellId != currentCellId)
                    {
                        // 设置当前CellSpace的出口时间
                        exitTimes[currentCellId] = times[i];
                        
                        // 设置下一个CellSpace的入口时间
                        if (!entryTimes.ContainsKey(targetCellId))
                        {
                            entryTimes[targetCellId] = times[i];
                            processedCellIds.Add(targetCellId);
                        }
                        
                        // 更新当前CellSpace
                        currentCellId = targetCellId;
                    }
                }
                
                // 设置最后一个CellSpace的出口时间
                if (!string.IsNullOrEmpty(currentCellId))
                {
                    exitTimes[currentCellId] = times[times.Count - 1];
                }
                
                // 首先清除该机器人和计划ID的旧占用
                ClearPlannedOccupancy(robotId, planId, times[0]);
                
                // 将占用信息添加到时空索引中
                foreach (string cellId in processedCellIds)
                {
                    bool isStartCell = (cellId == startCellId);
                    bool isEndCell = (cellId == currentCellId);
                    
                    float? entryTime = entryTimes.ContainsKey(cellId) ? entryTimes[cellId] : null;
                    float? exitTime = exitTimes.ContainsKey(cellId) ? exitTimes[cellId] : null;
                    
                    if (entryTime.HasValue && exitTime.HasValue)
                    {
                        // 获取CellSpace的时间索引
                        var timeIndex = GetCellTimeIndex(cellId);
                        if (timeIndex != null)
                        {
                            // 创建占用标识符
                            var occupier = new OccupancyIdentifier(robotId, planId, OccupancyType.Planned);
                            
                            // 创建并添加时间占用，包含起点/终点信息
                            var occupancy = new TimeOccupancy(
                                entryTime.Value, 
                                exitTime.Value, 
                                occupier, 
                                isStartCell, 
                                isEndCell);
                            
                            timeIndex.AddInterval(occupancy);
                        }
                    }
                    else if (entryTime.HasValue)
                    {
                        // 终点CellSpace：只有入口时间，使用路径末尾时间作为出口
                        var timeIndex = GetCellTimeIndex(cellId);
                        if (timeIndex != null)
                        {
                            var occupier = new OccupancyIdentifier(robotId, planId, OccupancyType.Planned);
                            var occupancy = new TimeOccupancy(
                                entryTime.Value, 
                                times[times.Count - 1], 
                                occupier, 
                                isStartCell, 
                                isEndCell);
                            
                            timeIndex.AddInterval(occupancy);
                        }
                    }
                    else
                    {
                        Debug.LogWarning($"CellSpace {cellId} has no entry or exit time");
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"Error: {ex.Message}\n{ex.StackTrace}");
            }
        }

        /// <summary>
        /// 合并另一个占用层
        /// </summary>
        public void MergeLayer(DynamicOccupancyLayer other, float startTime, float endTime)
        {
            foreach (var cellId in other.cellTimeIndices.Keys)
            {
                var otherOccupancies = other.GetOccupancies(cellId, startTime, endTime);
                
                foreach (var occupancy in otherOccupancies)
                {
                    // 添加到当前层，保留起点/终点信息
                    var timeIndex = GetCellTimeIndex(cellId);
                    var newOccupancy = new TimeOccupancy(
                        occupancy.StartTime,
                        occupancy.EndTime,
                        occupancy.Occupier,
                        occupancy.IsStartCell,
                        occupancy.IsEndCell);
                        
                    timeIndex.AddInterval(newOccupancy);
                }
            }
        }

        /// <summary>
        /// 周期性清理旧数据
        /// </summary>
        public void UpdateTime(float currentTime)
        {
            if (currentTime - lastCleanupTime > 30f) // 每30秒清理一次
            {
                Cleanup(currentTime);
            }
        }
        
        /// <summary>
        /// 清理旧数据
        /// </summary>
        private void Cleanup(float currentTime)
        {
            lastCleanupTime = currentTime;
            
            // 清理CellSpace时间索引
            foreach (var timeIndex in cellTimeIndices.Values)
            {
                timeIndex.Cleanup(currentTime, windowDuration);
            }
        }

        /// <summary>
        /// 获取时间窗口开始时间
        /// </summary>
        public float GetStartTime() => lastCleanupTime - windowDuration;

        /// <summary>
        /// 获取时间窗口结束时间
        /// </summary>
        public float GetEndTime() => lastCleanupTime;
        
        /// <summary>
        /// 获取窗口持续时间
        /// </summary>
        public float GetWindowDuration() => windowDuration;
        
        /// <summary>
        /// 检测指定路径是否有冲突
        /// </summary>
        public Dictionary<string, List<TimeOccupancy>> DetectPathConflicts(
            IndoorSpace indoorSpace,
            List<ConnectionPoint> path, 
            List<float> times, 
            string robotId)
        {
            Dictionary<string, List<TimeOccupancy>> conflicts = new Dictionary<string, List<TimeOccupancy>>();
            
            try
            {
                // 将路径映射到CellSpace占用
                Dictionary<string, float?> entryTimes = new Dictionary<string, float?>();
                Dictionary<string, float?> exitTimes = new Dictionary<string, float?>();
                HashSet<string> processedCellIds = new HashSet<string>();
                
                // 确定起点CellSpace
                ConnectionPoint startCP = path[0];
                if (startCP == null || startCP.CellBoundary == null)
                {
                    Debug.LogError("Error: First connection point or its boundary is null");
                    return conflicts;
                }
                
                // 获取起点CellSpace
                CellSpace startCellSpace = indoorSpace.GetCellSpaceFromId(startCP.CellBoundary.Source);
                if (startCellSpace == null)
                {
                    Debug.LogError($"Error: Could not find source CellSpace for boundary {startCP.CellBoundary.Id}");
                    return conflicts;
                }
                
                string startCellId = startCellSpace.Id;
                entryTimes[startCellId] = times[0];
                processedCellIds.Add(startCellId);
                
                // 处理路径中的所有连接点
                string currentCellId = startCellId;
                
                for (int i = 0; i < path.Count; i++)
                {
                    if (i >= times.Count)
                    {
                        Debug.LogWarning($"Error: Index {i} exceeds times array length {times.Count}");
                        break;
                    }
                    
                    ConnectionPoint cp = path[i];
                    if (cp == null || cp.CellBoundary == null)
                    {
                        Debug.LogWarning($"Error: Connection point at index {i} or its boundary is null");
                        continue;
                    }
                    
                    // 确定下一个CellSpace
                    string targetCellId = null;
                    
                    // 如果我们在源单元格中，下一个单元格是目标单元格
                    if (currentCellId == cp.CellBoundary.Source)
                    {
                        targetCellId = cp.CellBoundary.Target;
                    }
                    // 如果我们在目标单元格中，下一个单元格是源单元格
                    else if (currentCellId == cp.CellBoundary.Target)
                    {
                        targetCellId = cp.CellBoundary.Source;
                    }
                    // 否则，通过单元格空间查询确定
                    else
                    {
                        CellSpace nextCellSpace = indoorSpace.GetCellSpaceFromConnectionPoint(cp, false);
                        
                        if (nextCellSpace != null)
                        {
                            targetCellId = nextCellSpace.Id;
                        }
                        else
                        {
                            Debug.LogWarning($"Error: Could not determine target cell for connection point {cp.Id}");
                            continue;
                        }
                    }
                    
                    // 如果目标单元格已经确定并且与当前不同
                    if (!string.IsNullOrEmpty(targetCellId) && targetCellId != currentCellId)
                    {
                        // 设置当前CellSpace的出口时间
                        exitTimes[currentCellId] = times[i];
                        
                        // 设置下一个CellSpace的入口时间
                        if (!entryTimes.ContainsKey(targetCellId))
                        {
                            entryTimes[targetCellId] = times[i];
                            processedCellIds.Add(targetCellId);
                        }
                        
                        // 更新当前CellSpace
                        currentCellId = targetCellId;
                    }
                }
                
                // 设置最后一个CellSpace的出口时间
                if (!string.IsNullOrEmpty(currentCellId))
                {
                    exitTimes[currentCellId] = times[times.Count - 1];
                }
                
                // 检查每个CellSpace的冲突
                foreach (string cellId in processedCellIds)
                {
                    bool isStartCell = (cellId == startCellId);
                    bool isEndCell = (cellId == currentCellId);
                    
                    float? entryTime = entryTimes.ContainsKey(cellId) ? entryTimes[cellId] : null;
                    float? exitTime = exitTimes.ContainsKey(cellId) ? exitTimes[cellId] : null;
                    
                    if (entryTime.HasValue && exitTime.HasValue)
                    {
                        // 检查冲突，考虑起点/终点特殊情况
                        var cellConflicts = CheckCellConflict(
                            cellId, 
                            entryTime.Value, 
                            exitTime.Value, 
                            robotId,
                            isStartCell,
                            isEndCell);
                        
                        if (cellConflicts.Count > 0)
                        {
                            conflicts[cellId] = cellConflicts;
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"Error: {ex.Message}\n{ex.StackTrace}");
            }
            
            return conflicts;
        }
    }
}