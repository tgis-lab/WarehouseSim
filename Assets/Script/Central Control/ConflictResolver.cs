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
        private float timeShiftIncrement = 0.2f;
        // 最大尝试次数
        private int maxShiftAttempts = 50;
        
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
            float maxAcceptableDelay = 30.0f)
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
                    robotId);
                
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
    }
}