using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using NetTopologySuite.Geometries;

namespace Map
{
    /// <summary>
    /// Dynamic Occupancy Layer - Manages time-based occupancy for multiple CellSpaces with multi-floor support.
    /// Serves as a unified interface for spatial and temporal indexing.
    /// </summary>
    public class DynamicOccupancyLayer
    {
        private IndoorSpace indoorSpace;
        private Dictionary<string, TemporalIndex> cellTimeIndices;
        private SpatialIndex spatialIndex;
        private float windowDuration;
        private float lastCleanupTime;
        private bool enable3D;
        
        // Version management:
        // - Single-step mutations use Interlocked.Increment (lock-free, atomic)
        // - Compound mutations (AtomicSwapPath) use lock(_versionLock) for multi-step atomicity
        private long _version = 0;
        private readonly object _versionLock = new object();
        
        /// <summary>
        /// Current version of this layer. Used for optimistic locking during path commits.
        /// </summary>
        public long Version => System.Threading.Interlocked.Read(ref _version);
        
        /// <summary>
        /// Increments the version number. Called after any modification to the layer.
        /// </summary>
        private void IncrementVersion()
        {
            System.Threading.Interlocked.Increment(ref _version);
        }

        /// <summary>
        /// Creates a new dynamic occupancy layer
        /// </summary>
        public DynamicOccupancyLayer(IndoorSpace indoorSpace, float windowDuration, float initialTime, bool enable3D = false)
        {
            this.indoorSpace = indoorSpace;
            this.windowDuration = windowDuration;
            this.lastCleanupTime = initialTime;
            this.enable3D = enable3D;
            
            // Initialize temporal index
            cellTimeIndices = new Dictionary<string, TemporalIndex>();
            
            // Initialize spatial index
            spatialIndex = new SpatialIndex(enable3D);
            
            // If IndoorSpace is provided, initialize indices
            if (indoorSpace != null && indoorSpace.CellSpaces != null)
            {
                foreach (var cellSpace in indoorSpace.CellSpaces)
                {
                    // Add to temporal index
                    cellTimeIndices[cellSpace.Id] = new TemporalIndex();
                    
                    // Add to spatial index
                    spatialIndex.Insert(cellSpace);
                }
            }
        }
        
        /// <summary>
        /// Default constructor
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
        /// Sets whether to enable 3D multi-floor mode
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
        /// Gets whether 3D multi-floor mode is enabled
        /// </summary>
        public bool IsEnable3D()
        {
            return enable3D;
        }

        /// <summary>
        /// Gets the temporal index for the specified CellSpace, creating one if it doesn't exist
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
        /// Clears occupancies for the specified robot and plan.
        /// Also clears realtime occupancies for the same robot from currentTime onwards
        /// to prevent phantom conflicts after replan.
        /// </summary>
        public void ClearPlannedOccupancy(string robotId, string planId, float currentTime)
        {
            // Only clear occupancies from current time onwards, preserve past history
            foreach (var entry in cellTimeIndices)
            {
                string cellId = entry.Key;
                var timeIndex = entry.Value;

                var occupancies = timeIndex.QueryOverlapping(currentTime, float.MaxValue);
                foreach (var occupancy in occupancies)
                {
                    // Clear planned occupancies matching this robot and plan
                    if (occupancy.Occupier.RobotId == robotId &&
                        (string.IsNullOrEmpty(planId) || occupancy.Occupier.PlanId == planId))
                    {
                        timeIndex.RemoveInterval(occupancy);
                    }
                    // Also clear realtime occupancies for this robot to prevent phantom conflicts
                    // Realtime occupancies have PlanId = null and Type = Realtime, so they are
                    // never matched by the PlanId filter above when planId is specified.
                    else if (occupancy.Occupier.RobotId == robotId &&
                             occupancy.Occupier.Type == OccupancyType.Realtime &&
                             occupancy.EndTime >= currentTime)
                    {
                        timeIndex.RemoveInterval(occupancy);
                    }
                }
            }
            IncrementVersion();
        }

        /// <summary>
        /// Clears all occupancies within the specified time window
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
            IncrementVersion();
        }

        /// <summary>
        /// Checks if the specified occupancy conflicts with existing occupancies in the same CellSpace
        /// </summary>
        public List<TimeOccupancy> CheckCellConflict(
            string cellId, 
            float startTime, 
            float endTime, 
            string robotId,
            bool isStartCell = false,
            bool isEndCell = false)
        {
            // Get temporal index
            var timeIndex = GetCellTimeIndex(cellId);
            if (timeIndex == null)
                return new List<TimeOccupancy>();
            
            // Query all occupancies within the time range
            var existingOccupancies = timeIndex.GetOccupancies(startTime, endTime);
            
            // Filter out occupancies from the same robot
            return existingOccupancies.Where(o => o.Occupier.RobotId != robotId).ToList();
        }

        /// <summary>
        /// Checks cell conflict and captures the layer version at query time.
        /// Used for optimistic locking: the caller can later verify the version
        /// hasn't changed before committing a path.
        /// </summary>
        public (List<TimeOccupancy> conflicts, long capturedVersion) CheckCellConflictWithVersion(
            string cellId, float startTime, float endTime, string robotId)
        {
            long version = Version;
            var conflicts = CheckCellConflict(cellId, startTime, endTime, robotId);
            return (conflicts, version);
        }

        /// <summary>
        /// Sets dynamic occupancy for a CellSpace
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
            SetDynamicOccupancyInternal(cellId, startTime, endTime, robotId, planId, isStartCell, isEndCell);
            IncrementVersion();
        }

        /// <summary>
        /// Internal version without version increment — for batch operations
        /// that call IncrementVersion once at the end.
        /// </summary>
        private void SetDynamicOccupancyInternal(
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
                var type = string.IsNullOrEmpty(planId) ? OccupancyType.Realtime : OccupancyType.Planned;
                var occupier = new OccupancyIdentifier(robotId, planId, type);
                var occupancy = new TimeOccupancy(startTime, endTime, occupier, isStartCell, isEndCell);
                timeIndex.AddInterval(occupancy);
            }
        }

        /// <summary>
        /// Updates the end time of an existing realtime occupancy for a robot in a cell,
        /// or creates a new realtime occupancy if none exists.
        /// </summary>
        public void UpdateOccupancyEndTime(
            string cellId,
            string robotId,
            float newEndTime,
            float entryTime,
            bool isStartCell = false,
            bool isEndCell = false)
        {
            var timeIndex = GetCellTimeIndex(cellId);
            if (timeIndex == null) return;

            var existing = timeIndex.FindInterval(o =>
                o.Occupier.RobotId == robotId &&
                o.Occupier.Type == OccupancyType.Realtime &&
                Mathf.Abs(o.StartTime - entryTime) < 1.0f);

            if (existing != null)
            {
                timeIndex.UpdateIntervalEndTime(existing, newEndTime);
            }
            else
            {
                var occupier = new OccupancyIdentifier(robotId, null, OccupancyType.Realtime);
                var occupancy = new TimeOccupancy(entryTime, newEndTime, occupier, isStartCell, isEndCell);
                timeIndex.AddInterval(occupancy);
            }
            IncrementVersion();
        }

        /// <summary>
        /// Periodically cleans up old data
        /// </summary>
        public void UpdateTime(float currentTime)
        {
            if (currentTime - lastCleanupTime > 30f) // Clean up every 30 seconds
            {
                Cleanup(currentTime);
            }
        }

        /// <summary>
        /// Cleans up old data
        /// </summary>
        private void Cleanup(float currentTime)
        {
            lastCleanupTime = currentTime;
            
            // Clean up CellSpace temporal indices
            float cutoffTime = currentTime - windowDuration;
            foreach (var timeIndex in cellTimeIndices.Values)
            {
                timeIndex.RemoveIntervalsBefore(cutoffTime);
            }
        }

        /// <summary>
        /// Clears expired occupancies before the specified time
        /// </summary>
        public void PruneExpiredOccupancies(float cutoffTime)
        {
            // Clean up CellSpace temporal indices
            foreach (var timeIndex in cellTimeIndices.Values)
            {
                timeIndex.RemoveIntervalsBefore(cutoffTime);
            }
            
            // Update last cleanup time
            lastCleanupTime = Time.time;
            IncrementVersion();
        }

        /// <summary>
        /// Updates real-time occupancy
        /// </summary>
        public void UpdateRealTimeOccupancy(string cellId, string robotId, float currentTime)
        {
            var timeIndex = GetCellTimeIndex(cellId);
            if (timeIndex != null)
            {
                timeIndex.UpdateCurrentOccupancy(robotId, currentTime);
                IncrementVersion();
            }
        }

        /// <summary>
        /// Gets all occupancy records for the specified CellSpace within the time range.
        /// Compatibility method, same functionality as GetOccupancies.
        /// </summary>
        public List<TimeOccupancy> GetTimeOccupanciesForCellSpace(string cellId, float startTime, float endTime)
        {
            return GetOccupancies(cellId, startTime, endTime);
        }

        /// <summary>
        /// Gets all occupancy records for the specified CellSpace within the time range
        /// </summary>
        public List<TimeOccupancy> GetOccupancies(string cellId, float startTime, float endTime)
        {
            var timeIndex = GetCellTimeIndex(cellId);
            if (timeIndex == null)
                return new List<TimeOccupancy>();
                
            return timeIndex.GetOccupancies(startTime, endTime);
        }

        /// <summary>
        /// Removes all occupancies belonging to the specified robot from all cells.
        /// Used to clear stale entries before merging updated data from a robot's personal layer.
        /// </summary>
        public void RemoveOccupanciesByRobot(string robotId)
        {
            if (string.IsNullOrEmpty(robotId))
                return;

            foreach (var entry in cellTimeIndices)
            {
                var timeIndex = entry.Value;
                var allOccupancies = timeIndex.GetAllOccupancies();
                foreach (var occupancy in allOccupancies)
                {
                    if (occupancy.Occupier.RobotId == robotId)
                    {
                        timeIndex.RemoveInterval(occupancy);
                    }
                }
            }
            IncrementVersion();
        }

        /// <summary>
        /// Merges another occupancy layer.
        /// If robotId is provided, first removes all existing occupancies for that robot
        /// to prevent ghost accumulation from stale entries.
        /// </summary>
        public void MergeLayer(DynamicOccupancyLayer other, float startTime, float endTime, string robotId = null)
        {
            if (other == null)
                return;

            lock (_versionLock)
            {
                // Clear stale occupancies for this robot before merging new ones
                if (!string.IsNullOrEmpty(robotId))
                {
                    RemoveOccupanciesByRobot(robotId);
                }

                foreach (var cellId in other.cellTimeIndices.Keys)
                {
                    var otherOccupancies = other.GetOccupancies(cellId, startTime, endTime);

                    foreach (var occupancy in otherOccupancies)
                    {
                        // Add to current layer, preserving start/end cell info
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
                _version++; // Single version bump for entire merge operation
            }
        }
        
        #region Intent Layer Methods

        /// <summary>
        /// Queries all occupancies overlapping with the specified time range for a given cell.
        /// Public wrapper around the TemporalIndex QueryOverlapping method.
        /// </summary>
        public List<TimeOccupancy> QueryOverlapping(string cellId, float startTime, float endTime)
        {
            var timeIndex = GetCellTimeIndex(cellId);
            if (timeIndex == null)
                return new List<TimeOccupancy>();

            return timeIndex.QueryOverlapping(startTime, endTime);
        }

        /// <summary>
        /// Sets an intent occupancy for a CellSpace. Intent occupancies represent
        /// a robot's general direction/destination before a concrete path is planned.
        /// </summary>
        public void SetIntentOccupancy(string cellId, float startTime, float endTime, string robotId)
        {
            var timeIndex = GetCellTimeIndex(cellId);
            if (timeIndex != null)
            {
                var occupier = new OccupancyIdentifier(robotId, null, OccupancyType.Intent);
                var occupancy = new TimeOccupancy(startTime, endTime, occupier);
                timeIndex.AddInterval(occupancy);
                IncrementVersion();
            }
        }

        /// <summary>
        /// Clears all intent occupancies for the specified robot across all cells.
        /// </summary>
        public void ClearIntent(string robotId)
        {
            if (string.IsNullOrEmpty(robotId))
                return;

            foreach (var entry in cellTimeIndices)
            {
                entry.Value.RemoveByFilter(o =>
                    o.Occupier.Type == OccupancyType.Intent &&
                    o.Occupier.RobotId == robotId);
            }
            IncrementVersion();
        }

        /// <summary>
        /// Atomically swaps a robot's old planned path and intent with a new planned path.
        /// All operations happen inside a single lock so version increments exactly once.
        /// Returns false if the new path conflicts with existing reservations from other robots.
        /// </summary>
        public bool AtomicSwapPath(
            string robotId,
            string oldPlanId,
            List<string> cellIds,
            List<float> startTimes,
            List<float> endTimes,
            string newPlanId)
        {
            lock (_versionLock)
            {
                // Pre-flight conflict scan: check each new cell reservation against
                // existing occupancies, excluding this robot's own entries.
                // Intent occupancies are soft constraints (cost penalty only) and must not
                // block path commits — consistent with SIPP which excludes them from safe intervals.
                for (int i = 0; i < cellIds.Count; i++)
                {
                    var timeIndex = GetCellTimeIndex(cellIds[i]);
                    if (timeIndex != null)
                    {
                        var conflicts = timeIndex.CheckConflict(
                            startTimes[i], endTimes[i], excludeRobotId: robotId);
                        // Filter out Intent occupancies — they are soft constraints
                        bool hasHardConflict = false;
                        foreach (var c in conflicts)
                        {
                            if (c.Occupier.Type != OccupancyType.Intent)
                            {
                                hasHardConflict = true;
                                break;
                            }
                        }
                        if (hasHardConflict)
                        {
                            return false;
                        }
                    }
                }

                // Clear old planned occupancies (skip if oldPlanId is null)
                if (!string.IsNullOrEmpty(oldPlanId))
                {
                    foreach (var entry in cellTimeIndices)
                    {
                        entry.Value.RemoveByFilter(o =>
                            o.Occupier.RobotId == robotId &&
                            o.Occupier.PlanId == oldPlanId);
                    }
                }

                // Clear intent occupancies for this robot
                foreach (var entry in cellTimeIndices)
                {
                    entry.Value.RemoveByFilter(o =>
                        o.Occupier.Type == OccupancyType.Intent &&
                        o.Occupier.RobotId == robotId);
                }

                // Register new planned occupancies
                for (int i = 0; i < cellIds.Count; i++)
                {
                    var timeIndex = GetCellTimeIndex(cellIds[i]);
                    if (timeIndex != null)
                    {
                        var occupier = new OccupancyIdentifier(robotId, newPlanId, OccupancyType.Planned);
                        var occupancy = new TimeOccupancy(startTimes[i], endTimes[i], occupier);
                        timeIndex.AddInterval(occupancy);
                    }
                }

                // Increment version exactly once
                _version++;
                return true;
            }
        }

        /// <summary>
        /// Checks if the specified occupancy conflicts with existing occupancies in the same CellSpace,
        /// with optional type exclusion filtering.
        /// </summary>
        public List<TimeOccupancy> CheckCellConflict(
            string cellId,
            float startTime,
            float endTime,
            string robotId,
            OccupancyType[] excludeTypes)
        {
            return CheckCellConflict(cellId, startTime, endTime, robotId,
                isStartCell: false, isEndCell: false, excludeTypes: excludeTypes);
        }

        /// <summary>
        /// Checks if the specified occupancy conflicts with existing occupancies in the same CellSpace,
        /// with optional type exclusion filtering. Full overload with all parameters.
        /// </summary>
        public List<TimeOccupancy> CheckCellConflict(
            string cellId,
            float startTime,
            float endTime,
            string robotId,
            bool isStartCell,
            bool isEndCell,
            OccupancyType[] excludeTypes)
        {
            // Get temporal index
            var timeIndex = GetCellTimeIndex(cellId);
            if (timeIndex == null)
                return new List<TimeOccupancy>();

            // Query all occupancies within the time range
            var existingOccupancies = timeIndex.GetOccupancies(startTime, endTime);

            // Filter out occupancies from the same robot
            var result = existingOccupancies.Where(o => o.Occupier.RobotId != robotId);

            // Filter out excluded types
            if (excludeTypes != null && excludeTypes.Length > 0)
            {
                result = result.Where(o => !excludeTypes.Contains(o.Occupier.Type));
            }

            return result.ToList();
        }

        /// <summary>
        /// Checks if a cell has any occupancy of the specified type at the given time.
        /// </summary>
        public bool HasOccupancyOfType(string cellId, float time, OccupancyType type)
        {
            var timeIndex = GetCellTimeIndex(cellId);
            if (timeIndex == null)
                return false;

            var occupancies = timeIndex.QueryOverlapping(time - 0.1f, time + 0.1f);
            return occupancies.Any(o => o.Occupier.Type == type);
        }

        /// <summary>
        /// Checks if a cell has any occupancy of the specified type at the given time,
        /// excluding occupancies from the specified robot.
        /// </summary>
        public bool HasOccupancyOfType(string cellId, float time, OccupancyType type, string excludeRobotId)
        {
            var timeIndex = GetCellTimeIndex(cellId);
            if (timeIndex == null)
                return false;

            var occupancies = timeIndex.QueryOverlapping(time - 0.1f, time + 0.1f);
            return occupancies.Any(o => o.Occupier.Type == type && o.Occupier.RobotId != excludeRobotId);
        }

        #endregion

        #region Spatial Queries
        
        /// <summary>
        /// Queries all CellSpaces that intersect with the ConnectionPoint path
        /// </summary>
        public List<CellSpace> GetPathIntersectingCells(List<ConnectionPoint> connectionPointPath)
        {
            if (connectionPointPath == null || connectionPointPath.Count < 2)
                return new List<CellSpace>();
                
            // Extract path geometry information
            var pathInfo = ExtractPathGeometry(connectionPointPath);
            
            // Use SpatialIndex to query intersecting CellSpaces
            if (enable3D && pathInfo.Floors.Count > 0)
            {
                return spatialIndex.QueryPath(pathInfo.PathPoints, pathInfo.Floors.ToArray());
            }
            else
            {
                // Fallback to 2D query
                var path2D = pathInfo.PathPoints.Select(p => new Vector2(p.x, p.y)).ToList();
                return spatialIndex.QueryPath(path2D);
            }
        }
        
        /// <summary>
        /// Queries all CellSpaces containing the specified point
        /// </summary>
        public List<CellSpace> GetPointContainingCells(Vector2 point)
        {
            return spatialIndex.QueryPoint(point);
        }
        
        /// <summary>
        /// Queries all CellSpaces containing the specified 3D point
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
        /// Queries all CellSpaces intersecting with the specified bounding box
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
        
        #region Path Processing Methods
        
        /// <summary>
        /// Queries path occupancy conflicts
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
                // Calculate CellSpaces the path passes through and their time windows
                var pathOccupancies = CalculatePathCellOccupancies(connectionPointPath, times);
                
                // Check conflicts for each CellSpace
                foreach (var cellOccupancy in pathOccupancies)
                {
                    // Check for conflicts
                    var cellConflicts = CheckCellConflict(
                        cellOccupancy.CellId, 
                        cellOccupancy.EntryTime, 
                        cellOccupancy.ExitTime, 
                        robotId,
                        cellOccupancy.IsStartCell,
                        cellOccupancy.IsEndCell);
                    
                    // If there are conflicts, merge by robot ID and save
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
        /// Records path occupancy
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
                // Calculate CellSpaces the path passes through and their time windows
                var pathOccupancies = CalculatePathCellOccupancies(connectionPointPath, times);
                
                // Record occupancy for each CellSpace (batch — single version bump at end)
                foreach (var cellOccupancy in pathOccupancies)
                {
                    SetDynamicOccupancyInternal(
                        cellOccupancy.CellId,
                        cellOccupancy.EntryTime,
                        cellOccupancy.ExitTime,
                        robotId,
                        planId,
                        cellOccupancy.IsStartCell,
                        cellOccupancy.IsEndCell);
                }
                IncrementVersion();
                
                Debug.Log($"Recorded path occupancy for robot {robotId}, affecting {pathOccupancies.Count} cells");
            }
            catch (Exception ex)
            {
                Debug.LogError($"Error recording path occupancy: {ex.Message}\n{ex.StackTrace}");
            }
        }
        
        /// <summary>
        /// Attempts to commit a path with optimistic locking.
        /// Returns true if commit succeeds (version unchanged), false if conflict detected.
        /// </summary>
        /// <param name="connectionPointPath">The path to commit</param>
        /// <param name="times">Arrival times for each point</param>
        /// <param name="robotId">Robot ID</param>
        /// <param name="planId">Plan ID</param>
        /// <param name="expectedVersion">Version number captured before planning started</param>
        /// <returns>True if commit succeeded, false if version conflict</returns>
        public bool TryCommitPath(
            List<ConnectionPoint> connectionPointPath,
            List<float> times,
            string robotId,
            string planId,
            long expectedVersion)
        {
            lock (_versionLock)
            {
                // Check if version has changed since planning started
                if (Version != expectedVersion)
                {
                    Debug.LogWarning($"[TryCommitPath] Version conflict for Robot {robotId}. Expected: {expectedVersion}, Current: {Version}");
                    return false;
                }
                
                // Version matches - safe to commit
                RecordPathOccupancy(connectionPointPath, times, robotId, planId);
                return true;
            }
        }
        
        /// <summary>
        /// Attempts to commit pre-calculated cell occupancies with optimistic locking.
        /// Alternative method when cell occupancies are already calculated.
        /// </summary>
        public bool TryCommitCellOccupancies(
            List<(string CellId, float StartTime, float EndTime, bool IsStart, bool IsEnd)> occupancies,
            string robotId,
            string planId,
            long expectedVersion)
        {
            lock (_versionLock)
            {
                if (Version != expectedVersion)
                {
                    Debug.LogWarning($"[TryCommitCellOccupancies] Version conflict for Robot {robotId}. Expected: {expectedVersion}, Current: {Version}");
                    return false;
                }
                
                foreach (var occ in occupancies)
                {
                    SetDynamicOccupancy(occ.CellId, occ.StartTime, occ.EndTime, robotId, planId, occ.IsStart, occ.IsEnd);
                }
                return true;
            }
        }
        
        /// <summary>
        /// Calculates the CellSpaces the path passes through and their time windows
        /// </summary>
        private List<CellOccupancy> CalculatePathCellOccupancies(
            List<ConnectionPoint> connectionPointPath, 
            List<float> times)
        {
            List<CellOccupancy> result = new List<CellOccupancy>();
            
            try
            {
                // Get all CellSpaces the path passes through
                var pathCells = GetPathIntersectingCells(connectionPointPath);
                
                if (pathCells.Count == 0)
                {
                    Debug.LogWarning("No cells found for path");
                    return result;
                }
                
                // Store entry and exit times for each CellSpace
                Dictionary<string, float> entryTimes = new Dictionary<string, float>();
                Dictionary<string, float> exitTimes = new Dictionary<string, float>();
                HashSet<string> processedCellIds = new HashSet<string>();
                
                // Process path points
                for (int i = 0; i < connectionPointPath.Count; i++)
                {
                    ConnectionPoint cp = connectionPointPath[i];
                    float pointTime = times[i];
                    
                    if (cp?.Point is Point point)
                    {
                        // Get point coordinates
                        Vector3 position = new Vector3((float)point.X, (float)point.Y, 0);
                        
                        // If 3D is enabled, get point floor
                        string floor = null;
                        if (enable3D)
                        {
                            floor = GetConnectionPointFloor(cp);
                        }
                        
                        // Query CellSpace containing this point
                        List<CellSpace> containingCells;
                        if (enable3D && !string.IsNullOrEmpty(floor))
                        {
                            containingCells = GetPointContainingCells(position, floor);
                        }
                        else
                        {
                            containingCells = GetPointContainingCells(new Vector2(position.x, position.y));
                        }
                        
                        // If no direct containing CellSpace, use path intersecting CellSpaces
                        if (containingCells.Count == 0)
                        {
                            containingCells = pathCells;
                        }
                        
                        // Update times for each CellSpace
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
                
                // Process path segments
                for (int i = 0; i < connectionPointPath.Count - 1; i++)
                {
                    ConnectionPoint cp1 = connectionPointPath[i];
                    ConnectionPoint cp2 = connectionPointPath[i + 1];
                    
                    if (cp1?.Point is Point point1 && cp2?.Point is Point point2)
                    {
                        // Get segment endpoints and times
                        Vector3 p1 = new Vector3((float)point1.X, (float)point1.Y, 0);
                        Vector3 p2 = new Vector3((float)point2.X, (float)point2.Y, 0);
                        float t1 = times[i];
                        float t2 = times[i + 1];
                        
                        // Get segment floor info
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
                        
                        // Query CellSpaces intersecting the segment
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
                        
                        // Update times for each intersecting CellSpace
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
                
                // Determine start and end CellSpace
                string startCellId = null;
                string endCellId = null;
                
                if (connectionPointPath.Count > 0)
                {
                    // Start point
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
                    
                    // End point
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
                
                // Create CellOccupancy objects
                foreach (string cellId in processedCellIds)
                {
                    if (entryTimes.TryGetValue(cellId, out float entryTime) && 
                        exitTimes.TryGetValue(cellId, out float exitTime))
                    {
                        // Ensure exit time is greater than entry time
                        if (exitTime <= entryTime)
                        {
                            exitTime = entryTime + 0.001f; // Add a tiny epsilon
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
        /// Extracts geometric information from a ConnectionPoint path
        /// </summary>
        private (List<Vector3> PathPoints, HashSet<string> Floors) ExtractPathGeometry(List<ConnectionPoint> connectionPointPath)
        {
            List<Vector3> pathPoints = new List<Vector3>();
            HashSet<string> floors = new HashSet<string>();
            
            // Track the floor of the current ConnectionPoint
            string currentFloor = null;
            
            foreach (var cp in connectionPointPath)
            {
                if (cp?.Point is Point point)
                {
                    // Determine floor of current ConnectionPoint
                    if (enable3D)
                    {
                        string pointFloor = GetConnectionPointFloor(cp);
                        
                        // If valid floor found
                        if (!string.IsNullOrEmpty(pointFloor))
                        {
                            currentFloor = pointFloor;
                            floors.Add(currentFloor);
                        }
                        
                        // If vertical ConnectionPoint (connects different floors)
                        if (IsVerticalConnectionPoint(cp))
                        {
                            // Get connected floors
                            var connectedFloors = GetConnectedFloors(cp);
                            foreach (var floor in connectedFloors)
                            {
                                floors.Add(floor);
                            }
                        }
                    }
                    
                    // Add path point
                    pathPoints.Add(new Vector3((float)point.X, (float)point.Y, 0));
                }
            }
            
            return (pathPoints, floors);
        }
        
        /// <summary>
        /// Merges conflicts by robot ID
        /// </summary>
        private List<TimeOccupancy> MergeConflictsByRobot(List<TimeOccupancy> conflicts)
        {
            if (conflicts == null || conflicts.Count == 0)
                return new List<TimeOccupancy>();
                
            // Group by robot ID
            var groupedConflicts = conflicts.GroupBy(c => c.Occupier.RobotId);
            
            // Merge multiple conflicts for same robot
            List<TimeOccupancy> result = new List<TimeOccupancy>();
            
            foreach (var group in groupedConflicts)
            {
                string robotId = group.Key;
                var robotConflicts = group.ToList();
                
                if (robotConflicts.Count == 1)
                {
                    // Single conflict, add directly
                    result.Add(robotConflicts[0]);
                }
                else
                {
                    // Merge multiple conflicts
                    float minStartTime = robotConflicts.Min(c => c.StartTime);
                    float maxEndTime = robotConflicts.Max(c => c.EndTime);
                    
                    var firstConflict = robotConflicts[0];
                    var mergedOccupancy = new TimeOccupancy(
                        minStartTime,
                        maxEndTime,
                        firstConflict.Occupier,
                        false,  // No longer start/end cell after merge
                        false);
                        
                    result.Add(mergedOccupancy);
                }
            }
            
            return result;
        }
        
        #endregion
        
        #region Floor Handling
        
        /// <summary>
        /// Gets the floor where the ConnectionPoint is located
        /// </summary>
        private string GetConnectionPointFloor(ConnectionPoint cp)
        {
            // Add null check
            if (cp == null)
            {
                Debug.LogWarning("GetConnectionPointFloor: ConnectionPoint is null");
                return "1F"; // Return default floor
            }

            // Get CellBoundary associated with ConnectionPoint
            CellBoundary cellBoundary = cp.CellBoundary;
            if (cellBoundary == null)
            {
                Debug.LogWarning($"GetConnectionPointFloor: CellBoundary is null for ConnectionPoint {cp.Id}");
                return "1F"; // Return default floor
            }

            string sourceFloor = "1F";
            string targetFloor = "1F";
            bool sourceFound = false;
            bool targetFound = false;

            // Safely get floor of Source and Target CellSpace
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
                return "1F"; // Return default floor on error
            }

            // If both source and target found, check if it's a vertical connection
            if (sourceFound && targetFound && sourceFloor != targetFloor)
            {
                // It's a vertical connection, return source floor (caller needs to handle special case for vertical connections)
                return sourceFloor;
            }
            
            // Return found floor, prioritize source floor
            return sourceFound ? sourceFloor : (targetFound ? targetFloor : "1F");
        }
        
        /// <summary>
        /// Gets the floor where the CellSpace is located
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
        /// Checks if it is a vertical connection point (ConnectionPoint connecting different floors)
        /// </summary>
        private bool IsVerticalConnectionPoint(ConnectionPoint cp)
        {
            if (cp?.CellBoundary == null)
                return false;
                
            // Check if CellBoundary connects two CellSpaces on different floors
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
                
            // If both source and target have floors and they are different, it is a vertical connection
            if (!string.IsNullOrEmpty(sourceFloor) && !string.IsNullOrEmpty(targetFloor) && 
                sourceFloor != targetFloor)
            {
                return true;
            }
            
            // Check if transitionType property of CellBoundary is "Vertical"
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
        /// Gets the list of floors connected by the ConnectionPoint
        /// </summary>
        private List<string> GetConnectedFloors(ConnectionPoint cp)
        {
            List<string> floors = new List<string>();
            
            if (cp?.CellBoundary == null)
                return floors;
                
            // Add floor of source CellSpace
            if (cp.CellBoundary.Source != null)
            {
                var sourceCellSpace = indoorSpace.GetCellSpaceFromId(cp.CellBoundary.Source);
                string sourceFloor = GetCellSpaceFloor(sourceCellSpace);
                if (!string.IsNullOrEmpty(sourceFloor))
                    floors.Add(sourceFloor);
            }
            
            // Add floor of target CellSpace
            if (cp.CellBoundary.Target != null)
            {
                var targetCellSpace = indoorSpace.GetCellSpaceFromId(cp.CellBoundary.Target);
                string targetFloor = GetCellSpaceFloor(targetCellSpace);
                if (!string.IsNullOrEmpty(targetFloor) && !floors.Contains(targetFloor))
                    floors.Add(targetFloor);
            }
            
            // If it's a vertical connection, it might involve more floors
            if (floors.Count > 1)
            {
                var sourceCellSpace = indoorSpace.GetCellSpaceFromId(cp.CellBoundary.Source);
                var targetCellSpace = indoorSpace.GetCellSpaceFromId(cp.CellBoundary.Target);

                if (sourceCellSpace?.IsVerticalConnection() == true || targetCellSpace?.IsVerticalConnection() == true)
                {
                    // If it's an elevator or staircase, get all connected floors
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
        /// Checks if a string is a valid floor ID
        /// </summary>
        private bool IsValidFloorId(string id)
        {
            if (string.IsNullOrEmpty(id))
                return false;
                
            // Basement floor format (B1, B1F etc.)
            if (id.StartsWith("B") && id.Length > 1)
            {
                string numPart = id.Substring(1).Replace("F", "").Replace("f", "");
                return int.TryParse(numPart, out _);
            }
            
            // Above ground floor format (1F, 1 etc.)
            string levelPart = id.Replace("F", "").Replace("f", "");
            return int.TryParse(levelPart, out _);
        }
        
        #endregion
        
        #region CellOccupancy
        
        /// <summary>
        /// CellSpace occupancy information class
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