using System;
using System.Collections.Generic;
using UnityEngine;
using Map;

namespace CentralControl
{
    /// <summary>
    /// CellSpace冲突检测器
    /// </summary>
    public class CellSpaceConflictDetector
    {
        public Dictionary<string, List<TimeOccupancy>> DetectConflicts(
            DynamicOccupancyLayer layer, 
            IndoorSpace indoorSpace, 
            List<ConnectionPoint> connectionPointPath, 
            List<float> times, 
            string robotId)
        {
            if (connectionPointPath == null || connectionPointPath.Count < 2 || times == null || times.Count < connectionPointPath.Count)
            {
                Debug.LogWarning("Invalid boundary path or times");
                return new Dictionary<string, List<TimeOccupancy>>();
            }
            
            // Collect all conflicts
            Dictionary<string, List<TimeOccupancy>> conflicts = new Dictionary<string, List<TimeOccupancy>>();
            
            // Construct CellSpace path and its occupancy time
            var cellSpaceOccupancies = MapConnectionPointPathToCellSpaceOccupancies(indoorSpace, connectionPointPath, times);
            
            // 检查每个CellSpace的冲突
            foreach (var occupancy in cellSpaceOccupancies)
            {
                string cellId = occupancy.CellId;
                float entryTime = occupancy.EntryTime;
                float exitTime = occupancy.ExitTime;
                bool isStartCell = occupancy.IsStartCell;
                bool isEndCell = occupancy.IsEndCell;
                
                // 检查CellSpace冲突，传递起点/终点信息
                var cellConflicts = layer.CheckCellConflict(cellId, entryTime, exitTime, robotId, isStartCell, isEndCell);
                
                if (cellConflicts.Count > 0)
                {
                    // 先按机器人ID分组
                    Dictionary<string, List<TimeOccupancy>> robotConflicts = new Dictionary<string, List<TimeOccupancy>>();
                    
                    foreach (var conflict in cellConflicts)
                    {
                        string conflictRobotId = conflict.Occupier.RobotId;
                        
                        if (!robotConflicts.ContainsKey(conflictRobotId))
                        {
                            robotConflicts[conflictRobotId] = new List<TimeOccupancy>();
                        }
                        
                        robotConflicts[conflictRobotId].Add(conflict);
                    }
                    
                    // 合并每个机器人的时间段
                    if (!conflicts.ContainsKey(cellId))
                    {
                        conflicts[cellId] = new List<TimeOccupancy>();
                    }
                    
                    foreach (var robotEntry in robotConflicts)
                    {
                        string conflictRobotId = robotEntry.Key;
                        var robotOccupancies = robotEntry.Value;
                        
                        // 对于每个机器人，只保留一个合并后的占用记录
                        if (robotOccupancies.Count > 0)
                        {
                            // 找出最早的开始时间和最晚的结束时间
                            float minStart = float.MaxValue;
                            float maxEnd = float.MinValue;
                            bool hasStartCell = false;
                            bool hasEndCell = false;
                            
                            foreach (var occ in robotOccupancies)
                            {
                                minStart = Mathf.Min(minStart, occ.StartTime);
                                maxEnd = Mathf.Max(maxEnd, occ.EndTime);
                                hasStartCell |= occ.IsStartCell;
                                hasEndCell |= occ.IsEndCell;
                            }
                            
                            // 创建一个合并的占用记录，保留起点/终点标志
                            var mergedOccupier = robotOccupancies[0].Occupier; // 使用第一个占用的标识符
                            var mergedOccupancy = new TimeOccupancy(
                                minStart, 
                                maxEnd, 
                                mergedOccupier, 
                                hasStartCell, 
                                hasEndCell);
                            
                            conflicts[cellId].Add(mergedOccupancy);
                        }
                    }
                    
                    // 只输出每个单元格的汇总信息
                    Debug.LogWarning($"CellSpace {cellId} 检测到 {conflicts[cellId].Count} 个冲突机器人");
                    foreach (var conflict in conflicts[cellId])
                    {
                        Debug.LogWarning($"  与机器人 {conflict.Occupier.RobotId} 在时间 " +
                                    $"{conflict.StartTime:F2}-{conflict.EndTime:F2} 发生冲突");
                    }
                }
            }
            
            return conflicts;
        }
        
        /// <summary>
        /// 将边界路径映射为CellSpace占用
        /// </summary>
        public List<CellSpaceOccupancy> MapConnectionPointPathToCellSpaceOccupancies(
            IndoorSpace indoorSpace, 
            List<ConnectionPoint> connectionPointPath, 
            List<float> times)
        {
            List<CellSpaceOccupancy> occupancies = new List<CellSpaceOccupancy>();
            
            try
            {
                // Validate inputs first
                if (indoorSpace == null)
                {
                    Debug.LogError("Error: IndoorSpace is null");
                    return occupancies;
                }
                
                if (connectionPointPath == null || connectionPointPath.Count < 2)
                {
                    Debug.LogError("Error: ConnectionPointPath is null or too short");
                    return occupancies;
                }
                
                if (times == null || times.Count < connectionPointPath.Count)
                {
                    Debug.LogError("Error: Times array is null or too short");
                    return occupancies;
                }
                
                // 使用字典存储每个CellSpace的入口和出口信息
                Dictionary<string, float?> entryTimes = new Dictionary<string, float?>();
                Dictionary<string, float?> exitTimes = new Dictionary<string, float?>();
                HashSet<string> processedCellIds = new HashSet<string>();
                
                // 确定起点CellSpace
                ConnectionPoint startCP = connectionPointPath[0];
                if (startCP == null) 
                {
                    Debug.LogError("Error: First connection point is null");
                    return occupancies;
                }
                
                // Get CellBoundary from the ConnectionPoint
                var cellBoundary = startCP.CellBoundary;
                if (cellBoundary == null)
                {
                    Debug.LogError($"Error: Connection point {startCP.Id} has no CellBoundary");
                    return occupancies;
                }
                
                // 获取起点CellSpace (source side of the boundary)
                CellSpace startCellSpace = indoorSpace.GetCellSpaceFromId(cellBoundary.Source);
                if (startCellSpace == null)
                {
                    Debug.LogError($"Error: Could not find source CellSpace for boundary {cellBoundary.Id}");
                    return occupancies;
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
                    if (cp == null)
                    {
                        Debug.LogWarning($"Error: Connection point at index {i} is null");
                        continue;
                    }
                    
                    // Get CellBoundary for this connection point
                    var boundary = cp.CellBoundary;
                    if (boundary == null)
                    {
                        Debug.LogWarning($"Error: Connection point {cp.Id} has no CellBoundary");
                        continue;
                    }
                    
                    // Get target CellSpace (depends on current direction)
                    string targetCellId = null;
                    
                    // If we're in the source cell, next cell is target cell
                    if (currentCellId == boundary.Source)
                    {
                        targetCellId = boundary.Target;
                    }
                    // If we're in the target cell, next cell is source cell
                    else if (currentCellId == boundary.Target)
                    {
                        targetCellId = boundary.Source;
                    }
                    // Otherwise, determine based on cell space query
                    else
                    {
                        // 获取Target CellSpace
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
                    
                    // If target cell is identified and different from current
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
                
                // 创建CellSpaceOccupancy对象
                foreach (string cellId in processedCellIds)
                {
                    bool isStartCell = (cellId == startCellId);
                    bool isEndCell = (cellId == currentCellId);
                    
                    float? entryTime = entryTimes.ContainsKey(cellId) ? entryTimes[cellId] : null;
                    float? exitTime = exitTimes.ContainsKey(cellId) ? exitTimes[cellId] : null;
                    
                    if (entryTime.HasValue && exitTime.HasValue)
                    {
                        // 正常CellSpace：有入口和出口时间
                        CellSpaceOccupancy occupancy = new CellSpaceOccupancy(
                            cellId, entryTime.Value, exitTime.Value, isStartCell, isEndCell);
                        occupancies.Add(occupancy);
                    }
                    else if (entryTime.HasValue)
                    {
                        // 终点CellSpace：只有入口时间，使用最后的时间点作为退出时间
                        CellSpaceOccupancy occupancy = new CellSpaceOccupancy(
                            cellId, entryTime.Value, times[times.Count - 1], isStartCell, isEndCell);
                        occupancies.Add(occupancy);
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
            
            return occupancies;
        }
    }
    
    /// <summary>
    /// CellSpace占用信息
    /// </summary>
    public class CellSpaceOccupancy
    {
        public string CellId { get; }
        public float EntryTime { get; }
        public float ExitTime { get; }
        public bool IsStartCell { get; }
        public bool IsEndCell { get; }
        
        public CellSpaceOccupancy(string cellId, float entryTime, float exitTime, bool isStartCell = false, bool isEndCell = false)
        {
            CellId = cellId;
            EntryTime = entryTime;
            ExitTime = exitTime;
            IsStartCell = isStartCell;
            IsEndCell = isEndCell;
        }
    }
}
