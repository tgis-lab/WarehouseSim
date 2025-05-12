using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using NetTopologySuite.Geometries;
using Map;

namespace CentralControl
{
    /// <summary>
    /// CellSpace冲突检测器
    /// </summary>
    public class CellSpaceConflictDetector
    {
        /// <summary>
        /// 检测路径与已有占用的冲突
        /// </summary>
        public Dictionary<string, List<TimeOccupancy>> DetectConflicts(
            DynamicOccupancyLayer layer, 
            IndoorSpace indoorSpace, 
            List<ConnectionPoint> connectionPointPath, 
            List<float> times, 
            string robotId,
            string source = "Unknown")
        {
            if (connectionPointPath == null || connectionPointPath.Count < 2 || times == null || times.Count < connectionPointPath.Count)
            {
                Debug.LogWarning($"[{source}] Invalid boundary path or times");
                return new Dictionary<string, List<TimeOccupancy>>();
            }
            
            // 检查层和空间索引
            if (layer == null || indoorSpace == null)
            {
                Debug.LogWarning($"[{source}] Missing layer or indoorSpace");
                return new Dictionary<string, List<TimeOccupancy>>();
            }
            
            // 收集所有冲突
            Dictionary<string, List<TimeOccupancy>> conflicts = new Dictionary<string, List<TimeOccupancy>>();
            
            try
            {
                // 构建CellSpace路径及其占用时间
                var cellSpaceOccupancies = MapConnectionPointPathToCellSpaceOccupancies(layer, indoorSpace, connectionPointPath, times);
                
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
                        // 合并冲突
                        var mergedConflicts = MergeConflictsByRobot(cellConflicts, isStartCell, isEndCell);
                        conflicts[cellId] = mergedConflicts;
                        
                        // 添加来源信息到日志输出
                        Debug.LogWarning($"[{source}] CellSpace {cellId} 检测到 {conflicts[cellId].Count} 个冲突机器人");
                        foreach (var conflict in conflicts[cellId])
                        {
                            Debug.LogWarning($"[{source}]   与机器人 {conflict.Occupier.RobotId} 在时间 " +
                                        $"{conflict.StartTime:F2}-{conflict.EndTime:F2} 发生冲突");
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"[{source}] Error detecting conflicts: {ex.Message}\n{ex.StackTrace}");
            }
            
            return conflicts;
        }
        
        /// <summary>
        /// 将边界路径映射为CellSpace占用
        /// </summary>
        public List<CellSpaceOccupancy> MapConnectionPointPathToCellSpaceOccupancies(
            DynamicOccupancyLayer layer,
            IndoorSpace indoorSpace, 
            List<ConnectionPoint> connectionPointPath, 
            List<float> times)
        {
            List<CellSpaceOccupancy> occupancies = new List<CellSpaceOccupancy>();
            
            try
            {
                // 验证输入
                if (layer == null || indoorSpace == null)
                {
                    Debug.LogError("Error: Layer or IndoorSpace is null");
                    return occupancies;
                }
                
                if (connectionPointPath == null || connectionPointPath.Count < 2)
                {
                    Debug.LogError("Error: ConnectionPoint path is invalid");
                    return occupancies;
                }
                
                if (times == null || times.Count < connectionPointPath.Count)
                {
                    Debug.LogError($"Error: Times list length ({times?.Count ?? 0}) less than path length ({connectionPointPath.Count})");
                    return occupancies;
                }
                
                // 获取路径经过的所有CellSpace
                var pathCells = layer.GetPathIntersectingCells(connectionPointPath);
                
                if (pathCells == null || pathCells.Count == 0)
                {
                    Debug.LogWarning("No cells found intersecting with the path");
                    return occupancies;
                }
                
                // 跟踪每个CellSpace的进入和退出时间
                Dictionary<string, float> entryTimes = new Dictionary<string, float>();
                Dictionary<string, float> exitTimes = new Dictionary<string, float>();
                HashSet<string> processedCellIds = new HashSet<string>();
                
                // 确定起点CellSpace
                ConnectionPoint startCP = connectionPointPath[0];
                if (startCP == null) 
                {
                    Debug.LogError("Error: First connection point is null");
                    return occupancies;
                }
                
                var startCellId = GetStartCellId(indoorSpace, layer, startCP);
                if (string.IsNullOrEmpty(startCellId))
                {
                    Debug.LogError("Error: Could not determine start cell");
                    return occupancies;
                }
                
                // 记录起点CellSpace的入口时间
                float startTime = times[0];
                entryTimes[startCellId] = startTime;
                processedCellIds.Add(startCellId);
                
                // 沿路径遍历所有ConnectionPoint
                string currentCellId = startCellId;
                
                for (int i = 0; i < connectionPointPath.Count - 1; i++)
                {
                    ConnectionPoint cp = connectionPointPath[i];
                    ConnectionPoint nextCP = connectionPointPath[i + 1];
                    
                    if (cp?.CellBoundary == null || nextCP?.CellBoundary == null)
                    {
                        continue;
                    }
                    
                    // 确定下一个单元格
                    string targetCellId = DetermineNextCell(indoorSpace, cp, nextCP, currentCellId);
                    
                    if (!string.IsNullOrEmpty(targetCellId) && targetCellId != currentCellId)
                    {
                        // 设置当前CellSpace的出口时间
                        exitTimes[currentCellId] = times[i + 1];
                        
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
                    
                    if (entryTimes.TryGetValue(cellId, out float entryTime) && 
                        exitTimes.TryGetValue(cellId, out float exitTime))
                    {
                        // 确保出口时间大于入口时间
                        if (exitTime <= entryTime)
                        {
                            Debug.LogWarning($"修正Cell {cellId}的时间: 入口={entryTime}, 出口={exitTime}");
                            exitTime = entryTime + 0.001f; // 增加一个微小差值
                        }
                        
                        CellSpaceOccupancy occupancy = new CellSpaceOccupancy(
                            cellId, entryTime, exitTime, isStartCell, isEndCell);
                            
                        occupancies.Add(occupancy);
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"Error mapping path to occupancies: {ex.Message}\n{ex.StackTrace}");
            }
            
            return occupancies;
        }
        
        /// <summary>
        /// 按机器人ID合并冲突
        /// </summary>
        private List<TimeOccupancy> MergeConflictsByRobot(
            List<TimeOccupancy> conflicts, 
            bool isStartCell = false, 
            bool isEndCell = false)
        {
            if (conflicts == null || conflicts.Count == 0)
                return new List<TimeOccupancy>();
                
            // 按机器人ID分组
            var groupedConflicts = conflicts.GroupBy(c => c.Occupier.RobotId);
            
            // 合并同一机器人的多个冲突
            List<TimeOccupancy> result = new List<TimeOccupancy>();
            
            foreach (var group in groupedConflicts)
            {
                string robotId = group.Key;
                var robotConflicts = group.ToList();
                
                if (robotConflicts.Count == 1)
                {
                    // 只有一个冲突，直接添加
                    result.Add(robotConflicts[0]);
                }
                else
                {
                    // 合并多个冲突
                    float minStartTime = robotConflicts.Min(c => c.StartTime);
                    float maxEndTime = robotConflicts.Max(c => c.EndTime);
                    
                    var firstConflict = robotConflicts[0];
                    var mergedOccupancy = new TimeOccupancy(
                        minStartTime,
                        maxEndTime,
                        firstConflict.Occupier,
                        isStartCell,
                        isEndCell);
                        
                    result.Add(mergedOccupancy);
                }
            }
            
            return result;
        }
        
        /// <summary>
        /// 确定起点CellSpace ID
        /// </summary>
        private string GetStartCellId(IndoorSpace indoorSpace, DynamicOccupancyLayer layer, ConnectionPoint startCP)
        {
            if (startCP?.CellBoundary == null)
                return null;
                
            // 首先从边界找出起点CellSpace
            var sourceCellId = startCP.CellBoundary.Source;
            var targetCellId = startCP.CellBoundary.Target;
            
            if (!string.IsNullOrEmpty(sourceCellId))
                return sourceCellId;
                
            if (!string.IsNullOrEmpty(targetCellId))
                return targetCellId;
            
            // 如果边界没有提供足够信息，使用空间查询
            if (startCP.Point is Point startPoint)
            {
                Vector2 position = new Vector2((float)startPoint.X, (float)startPoint.Y);
                var containingCells = layer.GetPointContainingCells(position);
                
                if (containingCells.Count > 0)
                    return containingCells[0].Id;
            }
            
            return null;
        }
        
        /// <summary>
        /// 确定下一个CellSpace
        /// </summary>
        private string DetermineNextCell(
            IndoorSpace indoorSpace, 
            ConnectionPoint currentCP, 
            ConnectionPoint nextCP, 
            string currentCellId)
        {
            if (currentCP?.CellBoundary == null || nextCP?.CellBoundary == null)
                return null;
                
            // 当前边界的源和目标
            string currentSourceId = currentCP.CellBoundary.Source;
            string currentTargetId = currentCP.CellBoundary.Target;
            
            // 下一个边界的源和目标
            string nextSourceId = nextCP.CellBoundary.Source;
            string nextTargetId = nextCP.CellBoundary.Target;
            
            // 如果当前单元格是当前边界的源，则下一个是目标
            if (currentCellId == currentSourceId)
                return currentTargetId;
                
            // 如果当前单元格是当前边界的目标，则下一个是源
            if (currentCellId == currentTargetId)
                return currentSourceId;
                
            // 如果下一个边界的源或目标与当前边界匹配，选择另一个
            if (nextSourceId == currentSourceId || nextSourceId == currentTargetId)
                return nextTargetId;
                
            if (nextTargetId == currentSourceId || nextTargetId == currentTargetId)
                return nextSourceId;
                
            // 无法确定，返回null
            return null;
        }
    }
    
    /// <summary>
    /// CellSpace占用信息类
    /// </summary>
    public class CellSpaceOccupancy
    {
        public string CellId { get; private set; }
        public float EntryTime { get; private set; }
        public float ExitTime { get; private set; }
        public bool IsStartCell { get; private set; }
        public bool IsEndCell { get; private set; }
        
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