using System;
using System.Collections.Generic;
using UnityEngine;
using Map;

namespace CentralControl
{
    /// <summary>
    /// 边界冲突检测器
    /// </summary>
    public class BoundaryConflictDetector
    {
        // 当前可以保持简单，因为主要检测在CentralController进行
        
        /// <summary>
        /// 检测路径冲突
        /// </summary>
        public bool DetectConflicts(DynamicOccupancyLayer layer, List<string> boundaryPath, 
                                   List<float> times, string robotId, string planId = null)
        {
            if (boundaryPath == null || boundaryPath.Count < 2 || times == null || times.Count < boundaryPath.Count)
            {
                Debug.LogWarning("无效的路径或时间数据");
                return false;
            }
            
            bool hasConflict = false;
            
            // 检查每个边界的冲突
            for (int i = 0; i < boundaryPath.Count; i++)
            {
                string boundaryId = boundaryPath[i];
                float entryTime = times[i];
                float exitTime = (i < boundaryPath.Count - 1) ? times[i + 1] : entryTime + 1.0f; // 最后一个点估计1秒
                
                // 检查边界冲突
                var conflicts = layer.CheckBoundaryConflict(boundaryId, entryTime, exitTime, robotId);
                
                if (conflicts.Count > 0)
                {
                    hasConflict = true;
                    Debug.LogWarning($"在边界 {boundaryId} 检测到 {conflicts.Count} 个冲突");
                    
                    foreach (var conflict in conflicts)
                    {
                        Debug.LogWarning($"  与机器人 {conflict.Occupier.RobotId} 在时间 " +
                                        $"{conflict.StartTime:F2}-{conflict.EndTime:F2} 发生冲突");
                    }
                }
            }
            
            return hasConflict;
        }
        
        /// <summary>
        /// 预留路径占用
        /// </summary>
        public void ReservePath(DynamicOccupancyLayer layer, List<string> boundaryPath, 
                               List<float> times, string robotId, string planId = null)
        {
            if (string.IsNullOrEmpty(planId))
            {
                planId = $"Plan_{robotId}_{DateTime.Now.Ticks}";
            }
            
            // 清除之前的计划
            layer.ClearPlannedOccupancy(robotId, planId, Time.time);
            
            // 添加新的计划占用
            for (int i = 0; i < boundaryPath.Count - 1; i++)
            {
                string boundaryId = boundaryPath[i];
                float entryTime = times[i];
                float exitTime = times[i + 1];
                
                layer.SetBoundaryPlannedOccupancy(boundaryId, entryTime, exitTime, robotId, planId);
            }
            
            // 最后一个边界点
            if (boundaryPath.Count > 0)
            {
                string lastBoundaryId = boundaryPath[boundaryPath.Count - 1];
                float lastEntryTime = times[times.Count - 1];
                float lastExitTime = lastEntryTime + 1.0f; // 估计停留1秒
                
                layer.SetBoundaryPlannedOccupancy(lastBoundaryId, lastEntryTime, lastExitTime, robotId, planId);
            }
        }
    }
}