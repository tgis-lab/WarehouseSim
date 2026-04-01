using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using NetTopologySuite.Geometries;
using Map;

namespace CentralControl
{
    public class CellSpaceConflictDetector
    {
        public Dictionary<string, List<TimeOccupancy>> DetectConflicts(
            DynamicOccupancyLayer layer,
            IndoorSpace indoorSpace,
            List<ConnectionPoint> connectionPointPath,
            List<float> times,
            string robotId,
            string source = "Unknown",
            OccupancyType[] excludeTypes = null)
        {
            if (connectionPointPath == null || connectionPointPath.Count < 2 || times == null || times.Count < connectionPointPath.Count)
            {
                Debug.LogWarning($"[{source}] Invalid boundary path or times");
                return new Dictionary<string, List<TimeOccupancy>>();
            }
            
            if (layer == null || indoorSpace == null)
            {
                Debug.LogWarning($"[{source}] Missing layer or indoorSpace");
                return new Dictionary<string, List<TimeOccupancy>>();
            }
            
            Dictionary<string, List<TimeOccupancy>> conflicts = new Dictionary<string, List<TimeOccupancy>>();
            
            try
            {
                var cellSpaceOccupancies = MapConnectionPointPathToCellSpaceOccupancies(layer, indoorSpace, connectionPointPath, times);
                
                foreach (var occupancy in cellSpaceOccupancies)
                {
                    string cellId = occupancy.CellId;
                    float entryTime = occupancy.EntryTime;
                    float exitTime = occupancy.ExitTime;
                    bool isStartCell = occupancy.IsStartCell;
                    bool isEndCell = occupancy.IsEndCell;
                    
                    var cellConflicts = excludeTypes != null && excludeTypes.Length > 0
                        ? layer.CheckCellConflict(cellId, entryTime, exitTime, robotId, isStartCell, isEndCell, excludeTypes)
                        : layer.CheckCellConflict(cellId, entryTime, exitTime, robotId, isStartCell, isEndCell);
                    
                    if (cellConflicts.Count > 0)
                    {
                        var mergedConflicts = MergeConflictsByRobot(cellConflicts, isStartCell, isEndCell);
                        conflicts[cellId] = mergedConflicts;

                        Debug.Log($"[{source}] Detected {mergedConflicts.Count} conflicts in cell {cellId} for robot {robotId}");
                        foreach (var conflict in conflicts[cellId])
                        {
                            Debug.LogWarning($"[{source}] Conflict in cell {cellId} for robot {robotId}: " +
                                $"Start={conflict.StartTime}, End={conflict.EndTime}, Occupier={conflict.Occupier.RobotId}");
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
        
        public List<CellSpaceOccupancy> MapConnectionPointPathToCellSpaceOccupancies(
            DynamicOccupancyLayer layer,
            IndoorSpace indoorSpace, 
            List<ConnectionPoint> connectionPointPath, 
            List<float> times)
        {
            List<CellSpaceOccupancy> occupancies = new List<CellSpaceOccupancy>();
            
            try
            {
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
                
                var pathCells = layer.GetPathIntersectingCells(connectionPointPath);
                
                if (pathCells == null || pathCells.Count == 0)
                {
                    Debug.LogWarning("No cells found intersecting with the path");
                    return occupancies;
                }
                
                Dictionary<string, float> entryTimes = new Dictionary<string, float>();
                Dictionary<string, float> exitTimes = new Dictionary<string, float>();
                HashSet<string> processedCellIds = new HashSet<string>();
                
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
                
                float startTime = times[0];
                entryTimes[startCellId] = startTime;
                processedCellIds.Add(startCellId);
                
                string currentCellId = startCellId;
                
                for (int i = 0; i < connectionPointPath.Count - 1; i++)
                {
                    ConnectionPoint cp = connectionPointPath[i];
                    ConnectionPoint nextCP = connectionPointPath[i + 1];
                    
                    if (cp?.CellBoundary == null || nextCP?.CellBoundary == null)
                    {
                        continue;
                    }
                    
                    string targetCellId = DetermineNextCell(indoorSpace, cp, nextCP, currentCellId);
                    
                    if (!string.IsNullOrEmpty(targetCellId) && targetCellId != currentCellId)
                    {
                        exitTimes[currentCellId] = times[i + 1];
                        
                        if (!entryTimes.ContainsKey(targetCellId))
                        {
                            entryTimes[targetCellId] = times[i];
                            processedCellIds.Add(targetCellId);
                        }
                        
                        currentCellId = targetCellId;
                    }
                }
                
                if (!string.IsNullOrEmpty(currentCellId))
                {
                    exitTimes[currentCellId] = times[times.Count - 1];
                }
                
                foreach (string cellId in processedCellIds)
                {
                    bool isStartCell = (cellId == startCellId);
                    bool isEndCell = (cellId == currentCellId);
                    
                    if (entryTimes.TryGetValue(cellId, out float entryTime) && 
                        exitTimes.TryGetValue(cellId, out float exitTime))
                    {
                        if (exitTime <= entryTime)
                        {
                            exitTime = entryTime + 0.001f;
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
        
        private List<TimeOccupancy> MergeConflictsByRobot(
            List<TimeOccupancy> conflicts, 
            bool isStartCell = false, 
            bool isEndCell = false)
        {
            if (conflicts == null || conflicts.Count == 0)
                return new List<TimeOccupancy>();
                
            var groupedConflicts = conflicts.GroupBy(c => c.Occupier.RobotId);
            
            List<TimeOccupancy> result = new List<TimeOccupancy>();
            
            foreach (var group in groupedConflicts)
            {
                string robotId = group.Key;
                var robotConflicts = group.ToList();
                
                if (robotConflicts.Count == 1)
                {
                    result.Add(robotConflicts[0]);
                }
                else
                {
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
        private string GetStartCellId(IndoorSpace indoorSpace, DynamicOccupancyLayer layer, ConnectionPoint startCP)
        {
            if (startCP?.CellBoundary == null)
                return null;
                
            var sourceCellId = startCP.CellBoundary.Source;
            var targetCellId = startCP.CellBoundary.Target;
            
            if (!string.IsNullOrEmpty(sourceCellId))
                return sourceCellId;
                
            if (!string.IsNullOrEmpty(targetCellId))
                return targetCellId;
            
            if (startCP.Point is Point startPoint)
            {
                Vector2 position = new Vector2((float)startPoint.X, (float)startPoint.Y);
                var containingCells = layer.GetPointContainingCells(position);
                
                if (containingCells.Count > 0)
                    return containingCells[0].Id;
            }
            
            return null;
        }
        
        private string DetermineNextCell(
            IndoorSpace indoorSpace, 
            ConnectionPoint currentCP, 
            ConnectionPoint nextCP, 
            string currentCellId)
        {
            if (currentCP?.CellBoundary == null || nextCP?.CellBoundary == null)
                return null;
                
            string currentSourceId = currentCP.CellBoundary.Source;
            string currentTargetId = currentCP.CellBoundary.Target;
            
            string nextSourceId = nextCP.CellBoundary.Source;
            string nextTargetId = nextCP.CellBoundary.Target;
            
            if (currentCellId == currentSourceId)
                return currentTargetId;
                
            if (currentCellId == currentTargetId)
                return currentSourceId;
                
            if (nextSourceId == currentSourceId || nextSourceId == currentTargetId)
                return nextTargetId;
                
            if (nextTargetId == currentSourceId || nextTargetId == currentTargetId)
                return nextSourceId;
                
            return null;
        }
    }
    
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