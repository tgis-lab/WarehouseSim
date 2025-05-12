using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using NetTopologySuite.Geometries;

namespace Map
{
    /// <summary>
    /// 动态占用层 - 管理多个CellSpace的时间占用情况，支持多楼层结构
    /// 作为空间索引和时间索引的统一接口
    /// </summary>
    public class DynamicOccupancyLayer
    {
        private IndoorSpace indoorSpace;
        private Dictionary<string, TemporalIndex> cellTimeIndices;
        private SpatialIndex spatialIndex;
        private float windowDuration;
        private float lastCleanupTime;
        private bool enable3D;

        /// <summary>
        /// 创建新的动态占用层
        /// </summary>
        public DynamicOccupancyLayer(IndoorSpace indoorSpace, float windowDuration, float initialTime, bool enable3D = false)
        {
            this.indoorSpace = indoorSpace;
            this.windowDuration = windowDuration;
            this.lastCleanupTime = initialTime;
            this.enable3D = enable3D;
            
            // 初始化时间索引
            cellTimeIndices = new Dictionary<string, TemporalIndex>();
            
            // 初始化空间索引
            spatialIndex = new SpatialIndex(enable3D);
            
            // 如果提供了IndoorSpace，初始化索引
            if (indoorSpace != null && indoorSpace.CellSpaces != null)
            {
                foreach (var cellSpace in indoorSpace.CellSpaces)
                {
                    // 添加到时间索引
                    cellTimeIndices[cellSpace.Id] = new TemporalIndex();
                    
                    // 添加到空间索引
                    spatialIndex.Insert(cellSpace);
                }
            }
        }
        
        /// <summary>
        /// 默认构造函数
        /// </summary>
        public DynamicOccupancyLayer()
        {
            this.windowDuration = 60f;
            this.lastCleanupTime = Time.time;
            this.enable3D = false;
            
            cellTimeIndices = new Dictionary<string, TemporalIndex>();
            spatialIndex = new SpatialIndex(false);
        }
        
        /// <summary>
        /// 设置是否启用3D多楼层模式
        /// </summary>
        public void SetEnable3D(bool enable)
        {
            if (this.enable3D != enable)
            {
                this.enable3D = enable;
                spatialIndex.SetEnable3D(enable);
            }
        }
        
        /// <summary>
        /// 获取是否启用3D多楼层模式
        /// </summary>
        public bool IsEnable3D()
        {
            return enable3D;
        }

        /// <summary>
        /// 获取指定CellSpace的时间索引，如果不存在则创建
        /// </summary>
        private TemporalIndex GetCellTimeIndex(string cellId)
        {
            if (string.IsNullOrEmpty(cellId))
                return null;
                
            if (!cellTimeIndices.TryGetValue(cellId, out var timeIndex))
            {
                timeIndex = new TemporalIndex();
                cellTimeIndices[cellId] = timeIndex;
            }
            
            return timeIndex;
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
        /// 检查指定的占用是否与同一CellSpace中已有的占用冲突
        /// </summary>
        public List<TimeOccupancy> CheckCellConflict(
            string cellId, 
            float startTime, 
            float endTime, 
            string robotId,
            bool isStartCell = false,
            bool isEndCell = false)
        {
            // 获取时间索引
            var timeIndex = GetCellTimeIndex(cellId);
            if (timeIndex == null)
                return new List<TimeOccupancy>();
            
            // 查询时间段内的所有占用
            var existingOccupancies = timeIndex.GetOccupancies(startTime, endTime);
            
            // 过滤掉来自同一机器人的占用
            return existingOccupancies.Where(o => o.Occupier.RobotId != robotId).ToList();
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
            float cutoffTime = currentTime - windowDuration;
            foreach (var timeIndex in cellTimeIndices.Values)
            {
                timeIndex.RemoveIntervalsBefore(cutoffTime);
            }
        }

        /// <summary>
        /// 清除指定时间之前的过期占用
        /// </summary>
        public void PruneExpiredOccupancies(float cutoffTime)
        {
            // 清理CellSpace时间索引
            foreach (var timeIndex in cellTimeIndices.Values)
            {
                timeIndex.RemoveIntervalsBefore(cutoffTime);
            }
            
            // 更新最后清理时间
            lastCleanupTime = Time.time;
        }

        /// <summary>
        /// 更新实时占用
        /// </summary>
        public void UpdateRealTimeOccupancy(string cellId, string robotId, float currentTime)
        {
            var timeIndex = GetCellTimeIndex(cellId);
            if (timeIndex != null)
            {
                timeIndex.UpdateCurrentOccupancy(robotId, currentTime);
            }
        }

        /// <summary>
        /// 获取指定CellSpace在时间范围内的所有占用记录
        /// 兼容性方法，与GetOccupancies功能相同
        /// </summary>
        public List<TimeOccupancy> GetTimeOccupanciesForCellSpace(string cellId, float startTime, float endTime)
        {
            return GetOccupancies(cellId, startTime, endTime);
        }

        /// <summary>
        /// 获取指定CellSpace在时间段内的所有占用记录
        /// </summary>
        public List<TimeOccupancy> GetOccupancies(string cellId, float startTime, float endTime)
        {
            var timeIndex = GetCellTimeIndex(cellId);
            if (timeIndex == null)
                return new List<TimeOccupancy>();
                
            return timeIndex.GetOccupancies(startTime, endTime);
        }

        /// <summary>
        /// 合并另一个占用层
        /// </summary>
        public void MergeLayer(DynamicOccupancyLayer other, float startTime, float endTime)
        {
            if (other == null)
                return;
                
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
        
        #region Spatial Queries
        
        /// <summary>
        /// 查询与ConnectionPoint路径相交的所有CellSpace
        /// </summary>
        public List<CellSpace> GetPathIntersectingCells(List<ConnectionPoint> connectionPointPath)
        {
            if (connectionPointPath == null || connectionPointPath.Count < 2)
                return new List<CellSpace>();
                
            // 提取路径几何信息
            var pathInfo = ExtractPathGeometry(connectionPointPath);
            
            // 使用SpatialIndex查询相交的CellSpace
            if (enable3D && pathInfo.Floors.Count > 0)
            {
                return spatialIndex.QueryPath(pathInfo.PathPoints, pathInfo.Floors.ToArray());
            }
            else
            {
                // 降级为2D查询
                var path2D = pathInfo.PathPoints.Select(p => new Vector2(p.x, p.y)).ToList();
                return spatialIndex.QueryPath(path2D);
            }
        }
        
        /// <summary>
        /// 查询包含点的所有CellSpace
        /// </summary>
        public List<CellSpace> GetPointContainingCells(Vector2 point)
        {
            return spatialIndex.QueryPoint(point);
        }
        
        /// <summary>
        /// 查询包含3D点的所有CellSpace
        /// </summary>
        public List<CellSpace> GetPointContainingCells(Vector3 point, string floor = null)
        {
            if (enable3D && !string.IsNullOrEmpty(floor))
            {
                return spatialIndex.QueryPoint(point, floor);
            }
            else
            {
                return spatialIndex.QueryPoint(new Vector2(point.x, point.y));
            }
        }
        
        /// <summary>
        /// 查询与边界框相交的所有CellSpace
        /// </summary>
        public List<CellSpace> GetIntersectingCells(Envelope envelope, params string[] floors)
        {
            if (enable3D && floors != null && floors.Length > 0)
            {
                return spatialIndex.QueryIntersect(envelope, floors);
            }
            else
            {
                return spatialIndex.QueryIntersect(envelope);
            }
        }
        
        #endregion
        
        #region 路径处理方法
        
        /// <summary>
        /// 查询路径占用冲突
        /// </summary>
        public Dictionary<string, List<TimeOccupancy>> QueryPathOccupancyConflicts(
            List<ConnectionPoint> connectionPointPath, 
            List<float> times, 
            string robotId)
        {
            if (connectionPointPath == null || connectionPointPath.Count < 2 || 
                times == null || times.Count < connectionPointPath.Count)
            {
                Debug.LogWarning("Invalid path or times for conflict query");
                return new Dictionary<string, List<TimeOccupancy>>();
            }
            
            Dictionary<string, List<TimeOccupancy>> conflicts = new Dictionary<string, List<TimeOccupancy>>();
            
            try
            {
                // 计算路径经过的CellSpace及其时间窗口
                var pathOccupancies = CalculatePathCellOccupancies(connectionPointPath, times);
                
                // 检查每个CellSpace的冲突
                foreach (var cellOccupancy in pathOccupancies)
                {
                    // 检查冲突
                    var cellConflicts = CheckCellConflict(
                        cellOccupancy.CellId, 
                        cellOccupancy.EntryTime, 
                        cellOccupancy.ExitTime, 
                        robotId,
                        cellOccupancy.IsStartCell,
                        cellOccupancy.IsEndCell);
                    
                    // 如果有冲突，按机器人ID合并并保存
                    if (cellConflicts.Count > 0)
                    {
                        var mergedConflicts = MergeConflictsByRobot(cellConflicts);
                        conflicts[cellOccupancy.CellId] = mergedConflicts;
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"Error querying path conflicts: {ex.Message}\n{ex.StackTrace}");
            }
            
            return conflicts;
        }
        
        /// <summary>
        /// 记录路径占用
        /// </summary>
        public void RecordPathOccupancy(
            List<ConnectionPoint> connectionPointPath, 
            List<float> times, 
            string robotId, 
            string planId = null)
        {
            if (connectionPointPath == null || connectionPointPath.Count < 2 || 
                times == null || times.Count < connectionPointPath.Count)
            {
                Debug.LogWarning("Invalid path or times for recording occupancy");
                return;
            }
            
            try
            {
                // 计算路径经过的CellSpace及其时间窗口
                var pathOccupancies = CalculatePathCellOccupancies(connectionPointPath, times);
                
                // 记录每个CellSpace的占用
                foreach (var cellOccupancy in pathOccupancies)
                {
                    SetDynamicOccupancy(
                        cellOccupancy.CellId,
                        cellOccupancy.EntryTime,
                        cellOccupancy.ExitTime,
                        robotId,
                        planId,
                        cellOccupancy.IsStartCell,
                        cellOccupancy.IsEndCell);
                }
                
                Debug.Log($"Recorded path occupancy for robot {robotId}, affecting {pathOccupancies.Count} cells");
            }
            catch (Exception ex)
            {
                Debug.LogError($"Error recording path occupancy: {ex.Message}\n{ex.StackTrace}");
            }
        }
        
        /// <summary>
        /// 计算路径经过的CellSpace及其时间窗口
        /// </summary>
        private List<CellOccupancy> CalculatePathCellOccupancies(
            List<ConnectionPoint> connectionPointPath, 
            List<float> times)
        {
            List<CellOccupancy> result = new List<CellOccupancy>();
            
            try
            {
                // 获取路径经过的所有CellSpace
                var pathCells = GetPathIntersectingCells(connectionPointPath);
                
                if (pathCells.Count == 0)
                {
                    Debug.LogWarning("No cells found for path");
                    return result;
                }
                
                // 存储每个CellSpace的入口和出口时间
                Dictionary<string, float> entryTimes = new Dictionary<string, float>();
                Dictionary<string, float> exitTimes = new Dictionary<string, float>();
                HashSet<string> processedCellIds = new HashSet<string>();
                
                // 处理路径点
                for (int i = 0; i < connectionPointPath.Count; i++)
                {
                    ConnectionPoint cp = connectionPointPath[i];
                    float pointTime = times[i];
                    
                    if (cp?.Point is Point point)
                    {
                        // 获取点的坐标
                        Vector3 position = new Vector3((float)point.X, (float)point.Y, 0);
                        
                        // 如果启用了3D，获取点的楼层
                        string floor = null;
                        if (enable3D)
                        {
                            floor = GetConnectionPointFloor(cp);
                        }
                        
                        // 查询包含此点的CellSpace
                        List<CellSpace> containingCells;
                        if (enable3D && !string.IsNullOrEmpty(floor))
                        {
                            containingCells = GetPointContainingCells(position, floor);
                        }
                        else
                        {
                            containingCells = GetPointContainingCells(new Vector2(position.x, position.y));
                        }
                        
                        // 如果没有直接包含点的CellSpace，使用路径相交的CellSpace
                        if (containingCells.Count == 0)
                        {
                            containingCells = pathCells;
                        }
                        
                        // 更新每个CellSpace的时间
                        foreach (var cell in containingCells)
                        {
                            if (!entryTimes.ContainsKey(cell.Id) || pointTime < entryTimes[cell.Id])
                            {
                                entryTimes[cell.Id] = pointTime;
                            }
                            
                            if (!exitTimes.ContainsKey(cell.Id) || pointTime > exitTimes[cell.Id])
                            {
                                exitTimes[cell.Id] = pointTime;
                            }
                            
                            processedCellIds.Add(cell.Id);
                        }
                    }
                }
                
                // 处理路径线段
                for (int i = 0; i < connectionPointPath.Count - 1; i++)
                {
                    ConnectionPoint cp1 = connectionPointPath[i];
                    ConnectionPoint cp2 = connectionPointPath[i + 1];
                    
                    if (cp1?.Point is Point point1 && cp2?.Point is Point point2)
                    {
                        // 获取线段端点和时间
                        Vector3 p1 = new Vector3((float)point1.X, (float)point1.Y, 0);
                        Vector3 p2 = new Vector3((float)point2.X, (float)point2.Y, 0);
                        float t1 = times[i];
                        float t2 = times[i + 1];
                        
                        // 获取线段的楼层信息
                        List<string> segmentFloors = new List<string>();
                        if (enable3D)
                        {
                            string floor1 = GetConnectionPointFloor(cp1);
                            string floor2 = GetConnectionPointFloor(cp2);
                            
                            if (!string.IsNullOrEmpty(floor1))
                                segmentFloors.Add(floor1);
                                
                            if (!string.IsNullOrEmpty(floor2) && floor1 != floor2)
                                segmentFloors.Add(floor2);
                        }
                        
                        // 查询与线段相交的CellSpace
                        List<CellSpace> intersectingCells;
                        if (enable3D && segmentFloors.Count > 0)
                        {
                            intersectingCells = spatialIndex.QueryPath(
                                new List<Vector3> { p1, p2 }, 
                                segmentFloors.ToArray());
                        }
                        else
                        {
                            intersectingCells = spatialIndex.QueryPath(
                                new List<Vector2> { new Vector2(p1.x, p1.y), new Vector2(p2.x, p2.y) });
                        }
                        
                        // 更新每个相交CellSpace的时间
                        foreach (var cell in intersectingCells)
                        {
                            if (!entryTimes.ContainsKey(cell.Id) || t1 < entryTimes[cell.Id])
                            {
                                entryTimes[cell.Id] = t1;
                            }
                            
                            if (!exitTimes.ContainsKey(cell.Id) || t2 > exitTimes[cell.Id])
                            {
                                exitTimes[cell.Id] = t2;
                            }
                            
                            processedCellIds.Add(cell.Id);
                        }
                    }
                }
                
                // 确定起点和终点CellSpace
                string startCellId = null;
                string endCellId = null;
                
                if (connectionPointPath.Count > 0)
                {
                    // 起点
                    ConnectionPoint startCP = connectionPointPath[0];
                    if (startCP?.Point is Point startPoint)
                    {
                        Vector3 startPos = new Vector3((float)startPoint.X, (float)startPoint.Y, 0);
                        string startFloor = enable3D ? GetConnectionPointFloor(startCP) : null;
                        
                        var startCells = enable3D && !string.IsNullOrEmpty(startFloor) 
                            ? GetPointContainingCells(startPos, startFloor)
                            : GetPointContainingCells(new Vector2(startPos.x, startPos.y));
                            
                        if (startCells.Count > 0)
                        {
                            startCellId = startCells[0].Id;
                        }
                    }
                    
                    // 终点
                    ConnectionPoint endCP = connectionPointPath[connectionPointPath.Count - 1];
                    if (endCP?.Point is Point endPoint)
                    {
                        Vector3 endPos = new Vector3((float)endPoint.X, (float)endPoint.Y, 0);
                        string endFloor = enable3D ? GetConnectionPointFloor(endCP) : null;
                        
                        var endCells = enable3D && !string.IsNullOrEmpty(endFloor)
                            ? GetPointContainingCells(endPos, endFloor)
                            : GetPointContainingCells(new Vector2(endPos.x, endPos.y));
                            
                        if (endCells.Count > 0)
                        {
                            endCellId = endCells[0].Id;
                        }
                    }
                }
                
                // 创建CellOccupancy对象
                foreach (string cellId in processedCellIds)
                {
                    if (entryTimes.TryGetValue(cellId, out float entryTime) && 
                        exitTimes.TryGetValue(cellId, out float exitTime))
                    {
                        // 确保出口时间大于入口时间
                        if (exitTime <= entryTime)
                        {
                            exitTime = entryTime + 0.001f; // 增加一个微小差值
                        }
                        
                        var cellOccupancy = new CellOccupancy
                        {
                            CellId = cellId,
                            EntryTime = entryTime,
                            ExitTime = exitTime,
                            IsStartCell = (cellId == startCellId),
                            IsEndCell = (cellId == endCellId)
                        };
                        
                        result.Add(cellOccupancy);
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"Error calculating path cell occupancies: {ex.Message}\n{ex.StackTrace}");
            }
            
            return result;
        }
        
        /// <summary>
        /// 从ConnectionPoint提取路径的几何信息
        /// </summary>
        private (List<Vector3> PathPoints, HashSet<string> Floors) ExtractPathGeometry(List<ConnectionPoint> connectionPointPath)
        {
            List<Vector3> pathPoints = new List<Vector3>();
            HashSet<string> floors = new HashSet<string>();
            
            // 跟踪当前ConnectionPoint所在楼层
            string currentFloor = null;
            
            foreach (var cp in connectionPointPath)
            {
                if (cp?.Point is Point point)
                {
                    // 确定当前ConnectionPoint的楼层
                    if (enable3D)
                    {
                        string pointFloor = GetConnectionPointFloor(cp);
                        
                        // 如果找到了有效的楼层
                        if (!string.IsNullOrEmpty(pointFloor))
                        {
                            currentFloor = pointFloor;
                            floors.Add(currentFloor);
                        }
                        
                        // 如果是垂直ConnectionPoint (连接不同楼层)
                        if (IsVerticalConnectionPoint(cp))
                        {
                            // 获取连接的两个楼层
                            var connectedFloors = GetConnectedFloors(cp);
                            foreach (var floor in connectedFloors)
                            {
                                floors.Add(floor);
                            }
                        }
                    }
                    
                    // 添加路径点
                    pathPoints.Add(new Vector3((float)point.X, (float)point.Y, 0));
                }
            }
            
            return (pathPoints, floors);
        }
        
        /// <summary>
        /// 按机器人ID合并冲突
        /// </summary>
        private List<TimeOccupancy> MergeConflictsByRobot(List<TimeOccupancy> conflicts)
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
                        false,  // 合并后不再标记为起点/终点
                        false);
                        
                    result.Add(mergedOccupancy);
                }
            }
            
            return result;
        }
        
        #endregion
        
        #region Floor Handling
        
        /// <summary>
        /// 获取ConnectionPoint所在的楼层
        /// </summary>
        private string GetConnectionPointFloor(ConnectionPoint cp)
        {
            // 添加空检查
            if (cp == null)
            {
                Debug.LogWarning("GetConnectionPointFloor: ConnectionPoint is null");
                return "1F"; // 返回默认楼层
            }

            // 获取ConnectionPoint关联的CellBoundary
            CellBoundary cellBoundary = cp.CellBoundary;
            if (cellBoundary == null)
            {
                Debug.LogWarning($"GetConnectionPointFloor: CellBoundary is null for ConnectionPoint {cp.Id}");
                return "1F"; // 返回默认楼层
            }

            string sourceFloor = "1F";
            string targetFloor = "1F";
            bool sourceFound = false;
            bool targetFound = false;

            // 安全地获取Source和Target CellSpace的楼层
            try
            {
                if (cellBoundary.Source != null)
                {
                    CellSpace sourceCellSpace = indoorSpace.GetCellSpaceFromId(cellBoundary.Source);
                    if (sourceCellSpace != null)
                    {
                        sourceFloor = GetCellSpaceFloor(sourceCellSpace);
                        sourceFound = true;
                    }
                }
                
                if (cellBoundary.Target != null)
                {
                    CellSpace targetCellSpace = indoorSpace.GetCellSpaceFromId(cellBoundary.Target);
                    if (targetCellSpace != null)
                    {
                        targetFloor = GetCellSpaceFloor(targetCellSpace);
                        targetFound = true;
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"Error getting floor for ConnectionPoint {cp.Id}: {ex.Message}");
                return "1F"; // 出错时返回默认楼层
            }

            // 如果找到了源和目标，检查是否是垂直连接
            if (sourceFound && targetFound && sourceFloor != targetFloor)
            {
                // 是垂直连接，返回源楼层（调用者需要处理垂直连接的特殊情况）
                return sourceFloor;
            }
            
            // 返回找到的楼层，优先返回源楼层
            return sourceFound ? sourceFloor : (targetFound ? targetFloor : "1F");
        }
        
        /// <summary>
        /// 获取CellSpace所在的楼层
        /// </summary>
        private string GetCellSpaceFloor(CellSpace cellSpace)
        {
            if (cellSpace == null)
                return null;
                
            string locatedFloor = cellSpace.GetLocatedFloor();
            if (!string.IsNullOrEmpty(locatedFloor))
                return locatedFloor;
            
            return null;
        }
        
        /// <summary>
        /// 判断是否为垂直连接点(连接不同楼层的ConnectionPoint)
        /// </summary>
        private bool IsVerticalConnectionPoint(ConnectionPoint cp)
        {
            if (cp?.CellBoundary == null)
                return false;
                
            // 检查CellBoundary是否连接两个不同楼层的CellSpace
            string sourceFloor = null;
            string targetFloor = null;
            
            if (cp.CellBoundary.Source != null)
            {
                var sourceCellSpace = indoorSpace.GetCellSpaceFromId(cp.CellBoundary.Source);
                sourceFloor = GetCellSpaceFloor(sourceCellSpace);
            }
                
            if (cp.CellBoundary.Target != null)
            {
                var targetCellSpace = indoorSpace.GetCellSpaceFromId(cp.CellBoundary.Target);
                targetFloor = GetCellSpaceFloor(targetCellSpace);
            }
                
            // 如果源和目标都有楼层且不同，则为垂直连接
            if (!string.IsNullOrEmpty(sourceFloor) && !string.IsNullOrEmpty(targetFloor) && 
                sourceFloor != targetFloor)
            {
                return true;
            }
            
            // 检查CellBoundary的transitionType是否为"Vertical"
            if (cp.CellBoundary.Properties != null && 
                cp.CellBoundary.Properties.TryGetValue("transitionType", out var transitionType) && 
                transitionType != null && 
                transitionType.ToString().Equals("Vertical", StringComparison.OrdinalIgnoreCase))
            {
                return true;
            }
            
            return false;
        }
        
        /// <summary>
        /// 获取ConnectionPoint连接的楼层列表
        /// </summary>
        private List<string> GetConnectedFloors(ConnectionPoint cp)
        {
            List<string> floors = new List<string>();
            
            if (cp?.CellBoundary == null)
                return floors;
                
            // 添加源CellSpace的楼层
            if (cp.CellBoundary.Source != null)
            {
                var sourceCellSpace = indoorSpace.GetCellSpaceFromId(cp.CellBoundary.Source);
                string sourceFloor = GetCellSpaceFloor(sourceCellSpace);
                if (!string.IsNullOrEmpty(sourceFloor))
                    floors.Add(sourceFloor);
            }
            
            // 添加目标CellSpace的楼层
            if (cp.CellBoundary.Target != null)
            {
                var targetCellSpace = indoorSpace.GetCellSpaceFromId(cp.CellBoundary.Target);
                string targetFloor = GetCellSpaceFloor(targetCellSpace);
                if (!string.IsNullOrEmpty(targetFloor) && !floors.Contains(targetFloor))
                    floors.Add(targetFloor);
            }
            
            // 如果是垂直连接，可能涉及更多楼层
            if (floors.Count > 1)
            {
                var sourceCellSpace = indoorSpace.GetCellSpaceFromId(cp.CellBoundary.Source);
                var targetCellSpace = indoorSpace.GetCellSpaceFromId(cp.CellBoundary.Target);

                if (sourceCellSpace?.IsVerticalConnection() == true || targetCellSpace?.IsVerticalConnection() == true)
                {
                    // 如果是电梯或楼梯，获取所有连接的楼层
                    if (sourceCellSpace?.IsVerticalConnection() == true)
                    {
                        var connected = sourceCellSpace.GetConnectedFloors();
                        foreach (var floor in connected)
                        {
                            if (!floors.Contains(floor))
                                floors.Add(floor);
                        }
                    }
                    
                    if (targetCellSpace?.IsVerticalConnection() == true)
                    {
                        var connected = targetCellSpace.GetConnectedFloors();
                        foreach (var floor in connected)
                        {
                            if (!floors.Contains(floor))
                                floors.Add(floor);
                        }
                    }
                }
            }
            
            return floors;
        }
        
        /// <summary>
        /// 检查字符串是否为有效的楼层ID
        /// </summary>
        private bool IsValidFloorId(string id)
        {
            if (string.IsNullOrEmpty(id))
                return false;
                
            // 地下楼层格式 (B1, B1F 等)
            if (id.StartsWith("B") && id.Length > 1)
            {
                string numPart = id.Substring(1).Replace("F", "").Replace("f", "");
                return int.TryParse(numPart, out _);
            }
            
            // 地上楼层格式 (1F, 1 等)
            string levelPart = id.Replace("F", "").Replace("f", "");
            return int.TryParse(levelPart, out _);
        }
        
        #endregion
        
        #region CellOccupancy
        
        /// <summary>
        /// CellSpace占用信息类
        /// </summary>
        private class CellOccupancy
        {
            public string CellId { get; set; }
            public float EntryTime { get; set; }
            public float ExitTime { get; set; }
            public bool IsStartCell { get; set; }
            public bool IsEndCell { get; set; }
        }
        
        #endregion
    }
}