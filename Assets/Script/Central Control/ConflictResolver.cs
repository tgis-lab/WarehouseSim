using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Map;

namespace CentralControl
{
    /// <summary>
    /// 冲突消解策略实现
    /// </summary>
    public class ConflictResolver
    {
        private DynamicOccupancyLayer globalOccupancyLayer;
        private CellSpaceConflictDetector conflictDetector;
        private IndoorSpace indoorSpace;
        
        // 时间偏移增量参数（可调整）
        private float timeShiftIncrement = 0.1f;
        // 最大尝试次数
        private int maxShiftAttempts = 10;
        
        public ConflictResolver(
            DynamicOccupancyLayer globalOccupancyLayer, 
            CellSpaceConflictDetector conflictDetector,
            IndoorSpace indoorSpace)
        {
            this.globalOccupancyLayer = globalOccupancyLayer;
            this.conflictDetector = conflictDetector;
            this.indoorSpace = indoorSpace;
        }
        
        /// <summary>
        /// 时间偏移策略 - 整体平移路径的时间维度以避免冲突
        /// </summary>
        /// <returns>结果元组 (是否成功, 新的时间列表, 时间偏移量)</returns>
        public (bool success, List<float> newTimes, float appliedShift) ResolveConflictByTimeShift(
            string robotId, 
            List<ConnectionPoint> path, 
            List<float> times,
            Dictionary<string, List<TimeOccupancy>> conflicts,
            float maxAcceptableDelay = 10.0f)
        {
            // 参数验证
            if (path == null || path.Count < 2 || times == null || times.Count != path.Count)
            {
                Debug.LogError("无效的路径或时间数据");
                return (false, null, 0);
            }
            
            // 检查冲突字典是否为null
            if (conflicts == null)
            {
                Debug.LogError("冲突字典为null");
                return (false, null, 0);
            }
            
            // 检查依赖项
            if (globalOccupancyLayer == null)
            {
                Debug.LogError("全局占用层为null");
                return (false, null, 0);
            }
            
            if (conflictDetector == null)
            {
                Debug.LogError("冲突检测器为null");
                return (false, null, 0);
            }
            
            if (indoorSpace == null)
            {
                Debug.LogError("室内空间为null");
                return (false, null, 0);
            }
            
            // 当前尝试的时间偏移
            float currentShift = 0;
            
            // 尝试不同的时间偏移直到找到无冲突的解决方案或达到最大尝试次数
            for (int attempt = 0; attempt < maxShiftAttempts; attempt++)
            {
                // 计算下一个尝试的时间偏移
                // 使用自适应增量：针对不同冲突程度调整增量大小
                if (attempt > 0)
                {
                    // 计算冲突的总持续时间作为调整依据
                    float totalConflictDuration = CalculateTotalConflictDuration(conflicts);
                    float adaptiveIncrement = Math.Max(timeShiftIncrement, totalConflictDuration * 0.5f);
                    
                    // 增加当前偏移量
                    currentShift += adaptiveIncrement;
                    
                    // 检查是否超过最大可接受延迟
                    if (currentShift > maxAcceptableDelay)
                    {
                        Debug.LogWarning($"时间偏移超过最大可接受值: {currentShift}s > {maxAcceptableDelay}s");
                        return (false, null, currentShift);
                    }
                }
                
                // 应用时间偏移创建新的时间列表
                List<float> shiftedTimes = ApplyTimeShift(times, currentShift);
                
                // 使用CellSpaceConflictDetector检查偏移后的路径是否仍有冲突
                Dictionary<string, List<TimeOccupancy>> newConflicts = conflictDetector.DetectConflicts(
                    globalOccupancyLayer,
                    indoorSpace,
                    path,
                    shiftedTimes,
                    robotId, 
                    "TimeShiftVerification");
                
                // 如果没有冲突，返回成功和偏移后的时间
                if (newConflicts.Count == 0)
                {
                    Debug.Log($"成功通过时间偏移解决冲突: 偏移量 = {currentShift}s");
                    return (true, shiftedTimes, currentShift);
                }
                
                // 如果仍有冲突，记录信息并继续尝试
                Debug.Log($"时间偏移尝试 {attempt+1}/{maxShiftAttempts}: 偏移量 = {currentShift}s, 仍有 {newConflicts.Count} 个冲突");
            }
            
            // 达到最大尝试次数仍未解决冲突
            Debug.LogWarning($"无法通过时间偏移解决冲突，已达到最大尝试次数: {maxShiftAttempts}");
            return (false, null, currentShift);
        }
        
        /// <summary>
        /// 对时间列表应用时间偏移
        /// </summary>
        private List<float> ApplyTimeShift(List<float> times, float shift)
        {
            return times.Select(t => t + shift).ToList();
        }
        
        /// <summary>
        /// 计算冲突的总持续时间，用于自适应时间偏移增量
        /// </summary>
        private float CalculateTotalConflictDuration(Dictionary<string, List<TimeOccupancy>> conflicts)
        {
            float totalDuration = 0;
            
            foreach (var cellConflicts in conflicts.Values)
            {
                foreach (var conflict in cellConflicts)
                {
                    totalDuration += (conflict.EndTime - conflict.StartTime);
                }
            }
            
            return totalDuration;
        }

        /// <summary> 
        /// 速度调整策略 - 通过调整速度来避免冲突
        /// </summary>
        public (bool success, List<float> newTimes, List<float> newSpeeds, bool hasChanges) ResolveConflictBySpeedAdjustment(
            string robotId,
            List<ConnectionPoint> path,
            List<float> times,
            Dictionary<string, List<TimeOccupancy>> conflicts,
            float defaultSpeed = 1.0f,
            float maxSpeedAdjustmentRatio = 1.5f,
            float minSpeedAdjustmentRatio = 0.5f)
        {
            // 参数验证
            if (path == null || path.Count < 2 || times == null || times.Count != path.Count)
            {
                Debug.LogError("无效的路径或时间数据");
                return (false, null, null, false);
            }
            
            if (conflicts == null || conflicts.Count == 0)
            {
                // 没有冲突，直接返回原始数据
                return (true, new List<float>(times), new List<float>(), false);
            }
            
            // 创建新的时间列表副本
            List<float> newTimes = new List<float>(times);
            
            // 创建默认速度列表
            List<float> speeds = new List<float>();
            for (int i = 0; i < path.Count; i++)
            {
                speeds.Add(defaultSpeed);
            }
            
            List<float> newSpeeds = new List<float>(speeds);
            bool hasChanges = false;
            
            // 识别冲突区域及其对应的路径段
            Dictionary<int, (string cellId, float conflictStartTime, float conflictEndTime)> conflictSegments = 
                IdentifyConflictSegments(path, times, conflicts);
            
            if (conflictSegments.Count == 0)
            {
                Debug.LogWarning("无法确定冲突区域对应的路径段");
                return (false, null, null, false);
            }
            
            // 对每个冲突段进行速度调整
            foreach (var kvp in conflictSegments)
            {
                int segmentIndex = kvp.Key;
                string cellId = kvp.Value.cellId;
                float conflictStartTime = kvp.Value.conflictStartTime;
                float conflictEndTime = kvp.Value.conflictEndTime;
                
                // 确定需要调整的路径段范围
                // 我们希望提前几个段开始调整速度，以便有足够的时间加速或减速
                int adjustmentStartIndex = Math.Max(0, segmentIndex - 2);
                int adjustmentEndIndex = Math.Min(path.Count - 1, segmentIndex + 2);
                
                // 尝试不同的速度调整方案
                (bool adjusted, List<float> adjustedTimes, List<float> adjustedSpeeds) = 
                    TrySpeedAdjustment(
                        path, 
                        newTimes, 
                        newSpeeds, 
                        adjustmentStartIndex, 
                        adjustmentEndIndex, 
                        cellId,
                        conflictStartTime, 
                        conflictEndTime, 
                        defaultSpeed, 
                        maxSpeedAdjustmentRatio, 
                        minSpeedAdjustmentRatio);
                
                if (adjusted)
                {
                    // 更新时间和速度
                    for (int i = adjustmentStartIndex; i <= adjustmentEndIndex && i < adjustedTimes.Count; i++)
                    {
                        newTimes[i] = adjustedTimes[i - adjustmentStartIndex];
                        newSpeeds[i] = adjustedSpeeds[i - adjustmentStartIndex];
                    }
                    
                    // 更新后续所有点的时间
                    float timeDiff = 0;
                    if (adjustmentEndIndex < newTimes.Count - 1)
                    {
                        timeDiff = newTimes[adjustmentEndIndex] - times[adjustmentEndIndex];
                        for (int i = adjustmentEndIndex + 1; i < newTimes.Count; i++)
                        {
                            newTimes[i] = times[i] + timeDiff;
                        }
                    }
                    
                    hasChanges = true;
                    
                    // 检查调整后是否仍有冲突
                    Dictionary<string, List<TimeOccupancy>> newConflicts = conflictDetector.DetectConflicts(
                        globalOccupancyLayer,
                        indoorSpace,
                        path,
                        newTimes,
                        robotId, 
                        "SpeedAdjustmentVerification");
                    
                    // 如果仍有冲突，继续调整；否则返回调整后的结果
                    if (newConflicts.Count == 0)
                    {
                        Debug.Log($"成功通过速度调整解决冲突: 调整区间 [{adjustmentStartIndex}-{adjustmentEndIndex}]");
                        return (true, newTimes, newSpeeds, true);
                    }
                    
                    Debug.Log($"速度调整后仍有冲突，继续尝试其他冲突段");
                    conflicts = newConflicts;
                }
            }
            
            // 如果所有冲突段都尝试过了，但仍有未解决的冲突
            if (hasChanges)
            {
                Dictionary<string, List<TimeOccupancy>> finalConflicts = conflictDetector.DetectConflicts(
                    globalOccupancyLayer,
                    indoorSpace,
                    path,
                    newTimes,
                    robotId);
                
                if (finalConflicts.Count == 0)
                {
                    Debug.Log("所有冲突都通过速度调整成功解决");
                    return (true, newTimes, newSpeeds, true);
                }
            }
            
            Debug.LogWarning("无法通过速度调整完全解决冲突");
            return (false, hasChanges ? newTimes : null, hasChanges ? newSpeeds : null, hasChanges);
        }

        /// <summary>
        /// 识别冲突区域对应的路径段
        /// </summary>
        private Dictionary<int, (string cellId, float conflictStartTime, float conflictEndTime)> IdentifyConflictSegments(
            List<ConnectionPoint> path,
            List<float> times,
            Dictionary<string, List<TimeOccupancy>> conflicts)
        {
            Dictionary<int, (string cellId, float conflictStartTime, float conflictEndTime)> conflictSegments = 
                new Dictionary<int, (string cellId, float conflictStartTime, float conflictEndTime)>();
            
            // 获取路径上每个点对应的CellSpace
            var pathCells = new List<string>();
            for (int i = 0; i < path.Count; i++)
            {
                ConnectionPoint cp = path[i];
                string sourceCellId = cp.CellBoundary.Source;
                string targetCellId = cp.CellBoundary.Target;
                
                // 对于路径上的第一个点，考虑起点单元格
                if (i == 0)
                {
                    pathCells.Add(sourceCellId);
                }
                
                pathCells.Add(targetCellId);
            }
            
            // 对于每个冲突的CellSpace，找到对应的路径段
            foreach (var kvp in conflicts)
            {
                string cellId = kvp.Key;
                var cellConflicts = kvp.Value;
                
                // 跳过没有具体冲突的单元格
                if (cellConflicts.Count == 0)
                    continue;
                
                // 找到该单元格在路径中的位置
                for (int i = 0; i < pathCells.Count; i++)
                {
                    if (pathCells[i] == cellId)
                    {
                        // 确定进入和离开这个单元格的时间
                        float entryTime = (i == 0) ? times[0] : times[i - 1];
                        float exitTime = (i >= times.Count) ? times[times.Count - 1] : times[i];
                        
                        // 对该单元格的所有冲突，找出最早的开始时间和最晚的结束时间
                        float earliestConflictStart = float.MaxValue;
                        float latestConflictEnd = float.MinValue;
                        
                        foreach (var conflict in cellConflicts)
                        {
                            earliestConflictStart = Mathf.Min(earliestConflictStart, conflict.StartTime);
                            latestConflictEnd = Mathf.Max(latestConflictEnd, conflict.EndTime);
                        }
                        
                        // 记录冲突段
                        // 我们使用路径点的索引，而不是CellSpace的索引
                        int segmentIndex = Mathf.Min(i, path.Count - 1);
                        
                        if (!conflictSegments.ContainsKey(segmentIndex))
                        {
                            conflictSegments.Add(segmentIndex, (cellId, earliestConflictStart, latestConflictEnd));
                        }
                        else
                        {
                            // 如果已经有这个段的冲突记录，扩展冲突时间范围
                            var existing = conflictSegments[segmentIndex];
                            earliestConflictStart = Mathf.Min(earliestConflictStart, existing.conflictStartTime);
                            latestConflictEnd = Mathf.Max(latestConflictEnd, existing.conflictEndTime);
                            conflictSegments[segmentIndex] = (cellId, earliestConflictStart, latestConflictEnd);
                        }
                    }
                }
            }
            
            return conflictSegments;
        }

        /// <summary>
        /// 尝试速度调整方案
        /// </summary>
        private (bool success, List<float> adjustedTimes, List<float> adjustedSpeeds) TrySpeedAdjustment(
            List<ConnectionPoint> path,
            List<float> times,
            List<float> speeds,
            int startIndex,
            int endIndex,
            string conflictCellId,
            float conflictStartTime,
            float conflictEndTime,
            float defaultSpeed,
            float maxSpeedRatio,
            float minSpeedRatio)
        {
            // 提取需要调整的段
            int segmentCount = endIndex - startIndex + 1;
            List<float> segmentTimes = new List<float>();
            List<float> segmentSpeeds = new List<float>();
            
            for (int i = startIndex; i <= endIndex && i < times.Count; i++)
            {
                segmentTimes.Add(times[i]);
                segmentSpeeds.Add(speeds[i]);
            }
            
            // 计算当前到达冲突区域的时间
            float currentArrivalTime = segmentTimes[segmentTimes.Count - 1];
            
            // 首先尝试的策略方向取决于当前位置
            bool tryAccelerationFirst = currentArrivalTime < conflictStartTime;
            
            // 尝试第一种策略（基于当前位置决定是加速还是减速）
            var result = TrySingleDirectionAdjustment(
                path, segmentTimes, segmentSpeeds, startIndex, 
                conflictStartTime, conflictEndTime, 
                tryAccelerationFirst, // 如果在冲突前，先尝试加速
                defaultSpeed, maxSpeedRatio, minSpeedRatio);
            
            if (result.success)
            {
                // 第一种策略成功
                return result;
            }
            
            // 如果第一种策略失败，尝试相反方向
            Debug.Log($"首选{(tryAccelerationFirst ? "加速" : "减速")}策略失败，尝试{(tryAccelerationFirst ? "减速" : "加速")}");
            
            return TrySingleDirectionAdjustment(
                path, segmentTimes, segmentSpeeds, startIndex, 
                conflictStartTime, conflictEndTime, 
                !tryAccelerationFirst, // 尝试相反的策略
                defaultSpeed, maxSpeedRatio, minSpeedRatio);
        }

        /// <summary>
        /// 尝试单一方向的速度调整（加速或减速）
        /// </summary>
        private (bool success, List<float> adjustedTimes, List<float> adjustedSpeeds) TrySingleDirectionAdjustment(
            List<ConnectionPoint> path,
            List<float> segmentTimes,
            List<float> segmentSpeeds,
            int startIndex,
            float conflictStartTime,
            float conflictEndTime,
            bool accelerate, // true为加速，false为减速
            float defaultSpeed,
            float maxSpeedRatio,
            float minSpeedRatio)
        {
            float targetArrivalTime;
            
            if (accelerate)
            {
                // 加速策略：目标是在冲突开始前通过
                targetArrivalTime = conflictStartTime - 0.5f; // 提前0.5秒通过
                
                // 检查是否已经太迟，无法通过加速解决
                if (segmentTimes[0] + (targetArrivalTime - segmentTimes[0]) / maxSpeedRatio > conflictStartTime)
                {
                    Debug.Log($"即使最大加速也无法在冲突前通过，当前开始时间: {segmentTimes[0]:F2}，" +
                            $"冲突开始: {conflictStartTime:F2}，最快可能到达: {segmentTimes[0] + (targetArrivalTime - segmentTimes[0]) / maxSpeedRatio:F2}");
                    return (false, segmentTimes, segmentSpeeds);
                }
            }
            else
            {
                // 减速策略：目标是在冲突结束后通过
                targetArrivalTime = conflictEndTime + 0.5f; // 延后0.5秒通过
                
                // 检查减速是否有意义（例如，如果已经会在冲突结束后到达）
                if (segmentTimes[segmentTimes.Count - 1] > conflictEndTime)
                {
                    Debug.Log($"机器人已经会在冲突结束后到达，无需减速");
                    return (false, segmentTimes, segmentSpeeds);
                }
            }
            
            // 计算需要的平均速度比例调整
            float originalDuration = segmentTimes[segmentTimes.Count - 1] - segmentTimes[0];
            float targetDuration = targetArrivalTime - segmentTimes[0];
            float speedRatio = originalDuration / targetDuration;
            
            Debug.Log($"尝试{(accelerate ? "加速" : "减速")}策略: 原始持续时间={originalDuration:F2}s, " +
                    $"目标持续时间={targetDuration:F2}s, 速度比例={speedRatio:F2}");
            
            // 检查是否在可接受的范围内
            if (accelerate && speedRatio > maxSpeedRatio)
            {
                Debug.Log($"所需加速比例 {speedRatio:F2} 超过最大允许值 {maxSpeedRatio:F2}");
                return (false, segmentTimes, segmentSpeeds);
            }
            else if (!accelerate && speedRatio < minSpeedRatio)
            {
                Debug.Log($"所需减速比例 {speedRatio:F2} 低于最小允许值 {minSpeedRatio:F2}");
                return (false, segmentTimes, segmentSpeeds);
            }
            
            // 应用速度调整
            List<float> adjustedTimes = new List<float>();
            List<float> adjustedSpeeds = new List<float>();
            
            // 第一个时间点保持不变
            adjustedTimes.Add(segmentTimes[0]);
            
            // 调整速度并重新计算时间
            for (int i = 0; i < segmentSpeeds.Count; i++)
            {
                // 调整速度
                float newSpeed = segmentSpeeds[i] * speedRatio;
                adjustedSpeeds.Add(newSpeed);
                
                // 计算新的到达时间
                if (i > 0)
                {
                    float distance = Vector3.Distance(
                        new Vector3((float)path[startIndex + i].Point.Coordinate.X, 0, (float)path[startIndex + i].Point.Coordinate.Y),
                        new Vector3((float)path[startIndex + i - 1].Point.Coordinate.X, 0, (float)path[startIndex + i - 1].Point.Coordinate.Y));
                    
                    float segmentTime = distance / newSpeed;
                    float newTime = adjustedTimes[i - 1] + segmentTime;
                    adjustedTimes.Add(newTime);
                }
            }
            
            // 验证调整后的时间是否符合目标
            float adjustedArrivalTime = adjustedTimes[adjustedTimes.Count - 1];
            
            if (accelerate && adjustedArrivalTime >= conflictStartTime)
            {
                Debug.Log($"加速失败: 调整后到达时间 {adjustedArrivalTime:F2} >= 冲突开始时间 {conflictStartTime:F2}");
                return (false, segmentTimes, segmentSpeeds);
            }
            else if (!accelerate && adjustedArrivalTime <= conflictEndTime)
            {
                Debug.Log($"减速失败: 调整后到达时间 {adjustedArrivalTime:F2} <= 冲突结束时间 {conflictEndTime:F2}");
                return (false, segmentTimes, segmentSpeeds);
            }
            
            Debug.Log($"{(accelerate ? "加速" : "减速")}调整成功: 调整前到达时间={segmentTimes[segmentTimes.Count-1]:F2}, " +
                    $"调整后到达时间={adjustedArrivalTime:F2}, 速度比例={speedRatio:F2}");
            
            // 调整成功
            return (true, adjustedTimes, adjustedSpeeds);
        }
    }
}