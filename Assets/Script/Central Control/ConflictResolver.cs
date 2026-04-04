using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Map;

namespace CentralControl
{
    /// <summary>
    /// Conflict resolution strategy implementation
    /// </summary>
    public class ConflictResolver
    {
        private DynamicOccupancyLayer globalOccupancyLayer;
        private CellSpaceConflictDetector conflictDetector;
        private IndoorSpace indoorSpace;
        
        // Time shift increment parameter (adjustable)
        private float timeShiftIncrement = 0.1f;
        // Maximum number of attempts
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
        /// Time Shift Strategy - Shifts the time dimension of the entire path to avoid conflicts
        /// </summary>
        /// <returns>Result tuple (success, new times list, time offset)</returns>
        public (bool success, List<float> newTimes, float appliedShift) ResolveConflictByTimeShift(
            string robotId, 
            List<ConnectionPoint> path, 
            List<float> times,
            Dictionary<string, List<TimeOccupancy>> conflicts,
            float maxAcceptableDelay = 10.0f)
        {
            // Parameter validation
            if (path == null || path.Count < 2 || times == null || times.Count != path.Count)
            {
                Debug.LogError("Invalid path or time data");
                return (false, null, 0);
            }
            
            // Check if conflicts dictionary is null
            if (conflicts == null)
            {
                Debug.LogError("Conflicts dictionary is null");
                return (false, null, 0);
            }
            
            // Check dependencies
            if (globalOccupancyLayer == null)
            {
                Debug.LogError("Global occupancy layer is null");
                return (false, null, 0);
            }
            
            if (conflictDetector == null)
            {
                Debug.LogError("Conflict detector is null");
                return (false, null, 0);
            }
            
            if (indoorSpace == null)
            {
                Debug.LogError("Indoor space is null");
                return (false, null, 0);
            }
            
            // Current time shift attempt
            float currentShift = 0;
            
            // Try different time shifts until a conflict-free solution is found or max attempts reached
            for (int attempt = 0; attempt < maxShiftAttempts; attempt++)
            {
                // Calculate next time shift attempt
                // Use adaptive increment: adjust increment size for different conflict levels
                if (attempt > 0)
                {
                    // Calculate total conflict duration as basis for adjustment
                    float totalConflictDuration = CalculateTotalConflictDuration(conflicts);
                    float adaptiveIncrement = Math.Max(timeShiftIncrement, totalConflictDuration * 0.5f);
                    
                    // Increase current offset
                    currentShift += adaptiveIncrement;
                    
                    // Check if max acceptable delay exceeded
                    if (currentShift > maxAcceptableDelay)
                    {
                        Debug.LogWarning($"Time shift exceeds maximum acceptable value: {currentShift}s > {maxAcceptableDelay}s");
                        return (false, null, currentShift);
                    }
                }
                
                // Apply time shift to create new times list
                List<float> shiftedTimes = ApplyTimeShift(times, currentShift);
                
                // Use CellSpaceConflictDetector to check if shifted path still has conflicts
                Dictionary<string, List<TimeOccupancy>> newConflicts = conflictDetector.DetectConflicts(
                    globalOccupancyLayer,
                    indoorSpace,
                    path,
                    shiftedTimes,
                    robotId, 
                    "TimeShiftVerification");
                
                // If no conflicts, return success and shifted times
                if (newConflicts.Count == 0)
                {
                    Debug.Log($"Successfully resolved conflict by time shift: offset = {currentShift}s");
                    return (true, shiftedTimes, currentShift);
                }
                
                // If conflicts still exist, log info and continue trying
                Debug.Log($"Time shift attempt {attempt+1}/{maxShiftAttempts}: offset = {currentShift}s, {newConflicts.Count} conflicts remaining");
            }
            
            // Max attempts reached without resolving conflict
            Debug.LogWarning($"Unable to resolve conflict by time shift, max attempts reached: {maxShiftAttempts}");
            return (false, null, currentShift);
        }
        
        /// <summary>
        /// Apply time shift to times list
        /// </summary>
        private List<float> ApplyTimeShift(List<float> times, float shift)
        {
            return times.Select(t => t + shift).ToList();
        }
        
        /// <summary>
        /// Calculates total conflict duration for adaptive time shift increment
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
        /// Speed Adjustment Strategy - Adjusts speed to avoid conflicts
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
            // Parameter validation
            if (path == null || path.Count < 2 || times == null || times.Count != path.Count)
            {
                Debug.LogError("Invalid path or time data");
                return (false, null, null, false);
            }
            
            if (conflicts == null || conflicts.Count == 0)
            {
                // No conflicts, return original data directly
                return (true, new List<float>(times), new List<float>(), false);
            }
            
            // Create copy of new times list
            List<float> newTimes = new List<float>(times);
            
            // Create default speeds list
            List<float> speeds = new List<float>();
            for (int i = 0; i < path.Count; i++)
            {
                speeds.Add(defaultSpeed);
            }
            
            List<float> newSpeeds = new List<float>(speeds);
            bool hasChanges = false;
            
            // Identify conflict areas and corresponding path segments
            Dictionary<int, (string cellId, float conflictStartTime, float conflictEndTime)> conflictSegments = 
                IdentifyConflictSegments(path, times, conflicts);
            
            if (conflictSegments.Count == 0)
            {
                Debug.LogWarning("Unable to determine path segment corresponding to conflict area");
                return (false, null, null, false);
            }
            
            // Adjust speed for each conflict segment
            foreach (var kvp in conflictSegments)
            {
                int segmentIndex = kvp.Key;
                string cellId = kvp.Value.cellId;
                float conflictStartTime = kvp.Value.conflictStartTime;
                float conflictEndTime = kvp.Value.conflictEndTime;
                
                // Determine range of path segments to adjust
                // We want to start adjusting speed a few segments early for enough acceleration/deceleration time
                int adjustmentStartIndex = Math.Max(0, segmentIndex - 2);
                int adjustmentEndIndex = Math.Min(path.Count - 1, segmentIndex + 2);
                
                // Try different speed adjustment schemes
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
                    // Update times and speeds
                    for (int i = adjustmentStartIndex; i <= adjustmentEndIndex && i < adjustedTimes.Count; i++)
                    {
                        newTimes[i] = adjustedTimes[i - adjustmentStartIndex];
                        newSpeeds[i] = adjustedSpeeds[i - adjustmentStartIndex];
                    }
                    
                    // Update times for all subsequent points
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
                    
                    // Check if conflicts still exist after adjustment
                    Dictionary<string, List<TimeOccupancy>> newConflicts = conflictDetector.DetectConflicts(
                        globalOccupancyLayer,
                        indoorSpace,
                        path,
                        newTimes,
                        robotId, 
                        "SpeedAdjustmentVerification");
                    
                    // If conflicts still exist, continue adjusting; otherwise return adjusted result
                    if (newConflicts.Count == 0)
                    {
                        Debug.Log($"Successfully resolved conflict by speed adjustment: range [{adjustmentStartIndex}-{adjustmentEndIndex}]");
                        return (true, newTimes, newSpeeds, true);
                    }
                    
                    Debug.Log($"Conflicts still exist after speed adjustment, continuing with other segments");
                    conflicts = newConflicts;
                }
            }
            
            // If all conflict segments tried but unresolved conflicts remain
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
                    Debug.Log("All conflicts successfully resolved by speed adjustment");
                    return (true, newTimes, newSpeeds, true);
                }
            }
            
            Debug.LogWarning("Unable to fully resolve conflicts by speed adjustment");
            return (false, hasChanges ? newTimes : null, hasChanges ? newSpeeds : null, hasChanges);
        }

        /// <summary>
        /// Identify path segment corresponding to conflict area
        /// </summary>
        private Dictionary<int, (string cellId, float conflictStartTime, float conflictEndTime)> IdentifyConflictSegments(
            List<ConnectionPoint> path,
            List<float> times,
            Dictionary<string, List<TimeOccupancy>> conflicts)
        {
            Dictionary<int, (string cellId, float conflictStartTime, float conflictEndTime)> conflictSegments = 
                new Dictionary<int, (string cellId, float conflictStartTime, float conflictEndTime)>();
            
            // Get corresponding CellSpace for each path point
            var pathCells = new List<string>();
            for (int i = 0; i < path.Count; i++)
            {
                ConnectionPoint cp = path[i];
                string sourceCellId = cp.CellBoundary.Source;
                string targetCellId = cp.CellBoundary.Target;
                
                // For first path point, consider start cell
                if (i == 0)
                {
                    pathCells.Add(sourceCellId);
                }
                
                pathCells.Add(targetCellId);
            }
            
            // For each conflicting CellSpace, find the corresponding path segment
            foreach (var kvp in conflicts)
            {
                string cellId = kvp.Key;
                var cellConflicts = kvp.Value;
                
                // Skip cells without specific conflicts
                if (cellConflicts.Count == 0)
                    continue;
                
                // Find cell position in path
                for (int i = 0; i < pathCells.Count; i++)
                {
                    if (pathCells[i] == cellId)
                    {
                        // Determine entry and exit times for this cell
                        float entryTime = (i == 0) ? times[0] : times[i - 1];
                        float exitTime = (i >= times.Count) ? times[times.Count - 1] : times[i];
                        
                        // Find earliest start and latest end for all conflicts in this cell
                        float earliestConflictStart = float.MaxValue;
                        float latestConflictEnd = float.MinValue;
                        
                        foreach (var conflict in cellConflicts)
                        {
                            earliestConflictStart = Mathf.Min(earliestConflictStart, conflict.StartTime);
                            latestConflictEnd = Mathf.Max(latestConflictEnd, conflict.EndTime);
                        }
                        
                        // Record conflict segment
                        // We use path point indices instead of CellSpace indices
                        int segmentIndex = Mathf.Min(i, path.Count - 1);
                        
                        if (!conflictSegments.ContainsKey(segmentIndex))
                        {
                            conflictSegments.Add(segmentIndex, (cellId, earliestConflictStart, latestConflictEnd));
                        }
                        else
                        {
                            // If conflict record exists for this segment, extend conflict time range
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
        /// Try speed adjustment schemes
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
            // Extract segment to be adjusted
            int segmentCount = endIndex - startIndex + 1;
            List<float> segmentTimes = new List<float>();
            List<float> segmentSpeeds = new List<float>();
            
            for (int i = startIndex; i <= endIndex && i < times.Count; i++)
            {
                segmentTimes.Add(times[i]);
                segmentSpeeds.Add(speeds[i]);
            }

            if (segmentTimes.Count == 0)
            {
                return (false, times, speeds);
            }

            // Calculate current arrival time at conflict area
            float currentArrivalTime = segmentTimes[segmentTimes.Count - 1];
            
            // First strategy attempt direction depends on current position
            bool tryAccelerationFirst = currentArrivalTime < conflictStartTime;
            
            // Try first strategy (accelerate or decelerate based on current position)
            var result = TrySingleDirectionAdjustment(
                path, segmentTimes, segmentSpeeds, startIndex, 
                conflictStartTime, conflictEndTime, 
                tryAccelerationFirst, // If before conflict, try acceleration first
                defaultSpeed, maxSpeedRatio, minSpeedRatio);
            
            if (result.success)
            {
                // First strategy succeeded
                return result;
            }
            
            // If first strategy fails, try opposite direction
            Debug.Log($"Primary {(tryAccelerationFirst ? "acceleration" : "deceleration")} strategy failed, trying {(tryAccelerationFirst ? "deceleration" : "acceleration")}");
            
            return TrySingleDirectionAdjustment(
                path, segmentTimes, segmentSpeeds, startIndex, 
                conflictStartTime, conflictEndTime, 
                !tryAccelerationFirst, // Try opposite strategy
                defaultSpeed, maxSpeedRatio, minSpeedRatio);
        }

        /// <summary>
        /// Try single-direction speed adjustment (acceleration or deceleration)
        /// </summary>
        private (bool success, List<float> adjustedTimes, List<float> adjustedSpeeds) TrySingleDirectionAdjustment(
            List<ConnectionPoint> path,
            List<float> segmentTimes,
            List<float> segmentSpeeds,
            int startIndex,
            float conflictStartTime,
            float conflictEndTime,
            bool accelerate, // true for acceleration, false for deceleration
            float defaultSpeed,
            float maxSpeedRatio,
            float minSpeedRatio)
        {
            float targetArrivalTime;
            
            if (accelerate)
            {
                // Acceleration strategy: goal is to pass before conflict starts
                targetArrivalTime = conflictStartTime - 0.5f; // Pass 0.5s early
                
                // Check if too late for acceleration to resolve
                if (segmentTimes[0] + (targetArrivalTime - segmentTimes[0]) / maxSpeedRatio > conflictStartTime)
                {
                    Debug.Log($"Unable to pass before conflict even with max acceleration, current start: {segmentTimes[0]:F2}, " +
                            $"conflict start: {conflictStartTime:F2}, earliest arrival: {segmentTimes[0] + (targetArrivalTime - segmentTimes[0]) / maxSpeedRatio:F2}");
                    return (false, segmentTimes, segmentSpeeds);
                }
            }
            else
            {
                // Deceleration strategy: goal is to pass after conflict ends
                targetArrivalTime = conflictEndTime + 0.5f; // Pass 0.5s late
                
                // Check if deceleration is meaningful (e.g., if already arriving after conflict)
                if (segmentTimes[segmentTimes.Count - 1] > conflictEndTime)
                {
                    Debug.Log($"Robot will already arrive after conflict, no deceleration needed");
                    return (false, segmentTimes, segmentSpeeds);
                }
            }
            
            // Calculate required average speed ratio adjustment
            float originalDuration = segmentTimes[segmentTimes.Count - 1] - segmentTimes[0];
            float targetDuration = targetArrivalTime - segmentTimes[0];

            // Guard against zero/negative durations that would produce NaN/Infinity
            if (originalDuration <= 0f)
            {
                Debug.LogWarning($"Original duration is {originalDuration:F4}s (segment has no travel time), cannot adjust speed");
                return (false, segmentTimes, segmentSpeeds);
            }
            if (targetDuration <= 0f)
            {
                Debug.LogWarning($"Target duration is {targetDuration:F4}s (start time already at or past target), cannot adjust speed");
                return (false, segmentTimes, segmentSpeeds);
            }

            float speedRatio = originalDuration / targetDuration;
            
            Debug.Log($"Trying {(accelerate ? "acceleration" : "deceleration")} strategy: original duration={originalDuration:F2}s, " +
                    $"target duration={targetDuration:F2}s, speed ratio={speedRatio:F2}");
            
            // Check if within acceptable range
            if (accelerate && speedRatio > maxSpeedRatio)
            {
                Debug.Log($"Required acceleration ratio {speedRatio:F2} exceeds max allowed {maxSpeedRatio:F2}");
                return (false, segmentTimes, segmentSpeeds);
            }
            else if (!accelerate && speedRatio < minSpeedRatio)
            {
                Debug.Log($"Required deceleration ratio {speedRatio:F2} below min allowed {minSpeedRatio:F2}");
                return (false, segmentTimes, segmentSpeeds);
            }
            
            // Apply speed adjustment
            List<float> adjustedTimes = new List<float>();
            List<float> adjustedSpeeds = new List<float>();
            
            // First time point remains unchanged
            adjustedTimes.Add(segmentTimes[0]);
            
            // Adjust speeds and recalculate times
            for (int i = 0; i < segmentSpeeds.Count; i++)
            {
                // Adjust speed
                float newSpeed = segmentSpeeds[i] * speedRatio;
                adjustedSpeeds.Add(newSpeed);

                // Calculate new arrival time
                if (i > 0)
                {
                    float distance = Vector3.Distance(
                        new Vector3((float)path[startIndex + i].Point.Coordinate.X, 0, (float)path[startIndex + i].Point.Coordinate.Y),
                        new Vector3((float)path[startIndex + i - 1].Point.Coordinate.X, 0, (float)path[startIndex + i - 1].Point.Coordinate.Y));

                    // Guard against zero/near-zero speed that would produce Infinity
                    if (newSpeed <= 1e-6f)
                    {
                        Debug.LogWarning($"Adjusted speed is effectively zero ({newSpeed:F6}) at segment {i}, aborting speed adjustment");
                        return (false, segmentTimes, segmentSpeeds);
                    }

                    float segmentTime = distance / newSpeed;
                    float newTime = adjustedTimes[i - 1] + segmentTime;
                    adjustedTimes.Add(newTime);
                }
            }
            
            // Verify adjusted time meets target
            float adjustedArrivalTime = adjustedTimes[adjustedTimes.Count - 1];
            
            if (accelerate && adjustedArrivalTime >= conflictStartTime)
            {
                Debug.Log($"Acceleration failed: adjusted arrival {adjustedArrivalTime:F2} >= conflict start {conflictStartTime:F2}");
                return (false, segmentTimes, segmentSpeeds);
            }
            else if (!accelerate && adjustedArrivalTime <= conflictEndTime)
            {
                Debug.Log($"Deceleration failed: adjusted arrival {adjustedArrivalTime:F2} <= conflict end {conflictEndTime:F2}");
                return (false, segmentTimes, segmentSpeeds);
            }
            
            Debug.Log($"{(accelerate ? "Acceleration" : "Deceleration")} adjustment successful: prior arrival={segmentTimes[segmentTimes.Count-1]:F2}, " +
                    $"adjusted arrival={adjustedArrivalTime:F2}, speed ratio={speedRatio:F2}");
            
            // Adjustment successful
            return (true, adjustedTimes, adjustedSpeeds);
        }
    }
}