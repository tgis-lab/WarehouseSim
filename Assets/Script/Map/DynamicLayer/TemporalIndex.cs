using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using C5;
// Use alias to avoid conflict
using SystemCollections = System.Collections.Generic;

namespace Map
{
    /// <summary>
    /// Time occupancy record, representing occupancy within a specific time period
    /// </summary>
    public class TimeOccupancy : IComparable<TimeOccupancy>
    {
        public float StartTime { get; private set; }
        public float EndTime { get; private set; }
        public OccupancyIdentifier Occupier { get; private set; }
        public Guid Id { get; private set; }
        public bool IsStartCell { get; private set; }
        public bool IsEndCell { get; private set; }

        public TimeOccupancy(float startTime, float endTime, OccupancyIdentifier occupier, bool isStartCell = false, bool isEndCell = false)
        {
            if (endTime < startTime)
            {
                throw new ArgumentException("End time must be greater than or equal to start time");
            }

            StartTime = startTime;
            EndTime = endTime;
            Occupier = occupier;
            Id = Guid.NewGuid();
            IsStartCell = isStartCell;
            IsEndCell = isEndCell;
        }

        // Update end time (for real-time tracking)
        public void UpdateEndTime(float newEndTime)
        {
            if (newEndTime < StartTime)
            {
                throw new ArgumentException("New end time cannot be less than start time");
            }
            EndTime = newEndTime;
        }

        // IComparable interface implementation for C5 collections
        public int CompareTo(TimeOccupancy other)
        {
            if (other == null) return 1;
            
            // Compare start time first
            int startComparison = StartTime.CompareTo(other.StartTime);
            if (startComparison != 0) return startComparison;
            
            // Start times same, compare end times
            int endComparison = EndTime.CompareTo(other.EndTime);
            if (endComparison != 0) return endComparison;
            
            // Times identical, use ID to ensure uniqueness
            return Id.CompareTo(other.Id);
        }
    }

    /// <summary>
    /// Temporal Index - Manages time occupancy for a single location.
    /// Temporal index implemented using the C5 library based on interval tree concepts.
    /// </summary>
    public class TemporalIndex
    {
        // Dictionary storing all occupancies, key is GUID, value is TimeOccupancy
        private readonly HashDictionary<Guid, TimeOccupancy> allOccupancies;
        
        // Occupancy index sorted by start time
        private readonly TreeDictionary<float, HashedLinkedList<TimeOccupancy>> startTimeIndex;
        
        // Occupancy index sorted by end time
        private readonly TreeDictionary<float, HashedLinkedList<TimeOccupancy>> endTimeIndex;
        
        // Ongoing occupancies
        private readonly Dictionary<string, TimeOccupancy> ongoingOccupancies;

        /// <summary>
        /// Create a new temporal index
        /// </summary>
        public TemporalIndex()
        {
            allOccupancies = new HashDictionary<Guid, TimeOccupancy>();
            startTimeIndex = new TreeDictionary<float, HashedLinkedList<TimeOccupancy>>();
            endTimeIndex = new TreeDictionary<float, HashedLinkedList<TimeOccupancy>>();
            ongoingOccupancies = new Dictionary<string, TimeOccupancy>();
        }

        /// <summary>
        /// Add time interval to index
        /// </summary>
        public void AddInterval(TimeOccupancy interval)
        {
            if (interval == null) return;
            
            // Add to all occupancies dictionary
            allOccupancies[interval.Id] = interval;
            
            // Add to start time index
            if (!startTimeIndex.Contains(interval.StartTime))
            {
                startTimeIndex[interval.StartTime] = new HashedLinkedList<TimeOccupancy>();
            }
            startTimeIndex[interval.StartTime].Add(interval);
            
            // Add to end time index
            if (!endTimeIndex.Contains(interval.EndTime))
            {
                endTimeIndex[interval.EndTime] = new HashedLinkedList<TimeOccupancy>();
            }
            endTimeIndex[interval.EndTime].Add(interval);
        }

        /// <summary>
        /// Remove time interval from index
        /// </summary>
        public void RemoveInterval(TimeOccupancy interval)
        {
            if (interval == null) return;
            
            // Remove from all occupancies dictionary
            if (allOccupancies.Contains(interval.Id))
            {
                allOccupancies.Remove(interval.Id);
                
                // Remove from start time index
                if (startTimeIndex.Contains(interval.StartTime))
                {
                    var list = startTimeIndex[interval.StartTime];
                    list.Remove(interval);
                    
                    // If no more occupancies at this time, remove the entry
                    if (list.Count == 0)
                    {
                        startTimeIndex.Remove(interval.StartTime);
                    }
                }
                
                // Remove from end time index
                if (endTimeIndex.Contains(interval.EndTime))
                {
                    var list = endTimeIndex[interval.EndTime];
                    list.Remove(interval);
                    
                    // If no more occupancies at this time, remove the entry
                    if (list.Count == 0)
                    {
                        endTimeIndex.Remove(interval.EndTime);
                    }
                }
            }
        }

        /// <summary>
        /// Query all occupancies overlapping with specified time period
        /// </summary>
        public List<TimeOccupancy> QueryOverlapping(float startTime, float endTime)
        {
            List<TimeOccupancy> result = new List<TimeOccupancy>();
            
            // Check special cases early
            if (startTime > endTime || startTimeIndex.IsEmpty)
            {
                return result;
            }
            
            // Collect all potentially overlapping intervals
            var potentialOverlaps = new SystemCollections.HashSet<TimeOccupancy>();
            
            try
            {
                // Traverse all intervals with start time <= query end time
                var enumerator = startTimeIndex.RangeFrom(startTimeIndex.FindMin().Key).GetEnumerator();
                while (enumerator.MoveNext())
                {
                    var entry = enumerator.Current;
                    if (entry.Key > endTime)
                        break;  // If start time > query end time, can stop
                    
                    foreach (var interval in entry.Value)
                    {
                        potentialOverlaps.Add(interval);
                    }
                }
                
                // Filter out intervals that truly overlap
                foreach (var interval in potentialOverlaps)
                {
                    if (interval.EndTime >= startTime)
                    {
                        result.Add(interval);
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"Error querying time overlap: {ex.Message}");
            }
            
            return result;
        }

        /// <summary>
        /// Check for time conflicts, optionally excluding a robot's occupancy
        /// </summary>
        public List<TimeOccupancy> CheckConflict(
            float startTime, 
            float endTime, 
            string excludeRobotId = null, 
            bool isStartCell = false, 
            bool isEndCell = false)
        {
            // Query overlapping occupancies
            var overlappingOccupancies = QueryOverlapping(startTime, endTime);
            var conflicts = new List<TimeOccupancy>();
            
            foreach (var occupancy in overlappingOccupancies)
            {
                // Exclude occupancy of specific robot
                if (!string.IsNullOrEmpty(excludeRobotId) && occupancy.Occupier.RobotId == excludeRobotId)
                {
                    continue;
                }
                
                // Apply special rules
                
                // Rule 1: If checking start cell, only report conflict in specific cases
                if (isStartCell && !occupancy.IsEndCell)
                {
                    // Allow coexistence at start cell with non-end occupancies
                    continue;
                }
                
                // Rule 2: If checking end cell, only report conflict in specific cases
                if (isEndCell && !occupancy.IsStartCell)
                {
                    // Allow coexistence at end cell with non-start occupancies
                    continue;
                }
                
                // Rule 3: If overlap is a start cell and we are not an end cell, allow coexistence
                if (occupancy.IsStartCell && !isEndCell)
                {
                    continue;
                }
                
                // Rule 4: If overlap is an end cell and we are not a start cell, allow coexistence
                if (occupancy.IsEndCell && !isStartCell)
                {
                    continue;
                }
                
                // Actual conflict if it remains after all rules
                conflicts.Add(occupancy);
            }
            
            return conflicts;
        }

        /// <summary>
        /// Update ongoing occupancy
        /// </summary>
        public void UpdateCurrentOccupancy(string robotId, float currentTime)
        {
            string key = robotId;
            
            if (ongoingOccupancies.TryGetValue(key, out TimeOccupancy current))
            {
                // Remove from index (as end time will change)
                RemoveInterval(current);
                
                // Update end time
                current.UpdateEndTime(currentTime);
                
                // Re-add to index
                AddInterval(current);
            }
            else
            {
                // Create new occupancy record
                var occupier = new OccupancyIdentifier(robotId);
                var newOccupancy = new TimeOccupancy(currentTime, currentTime, occupier);
                ongoingOccupancies[key] = newOccupancy;
                AddInterval(newOccupancy);
            }
        }

        /// <summary>
        /// End current occupancy
        /// </summary>
        public void EndCurrentOccupancy(string robotId, float endTime)
        {
            string key = robotId;
            
            if (ongoingOccupancies.TryGetValue(key, out TimeOccupancy current))
            {
                // Remove from index
                RemoveInterval(current);
                
                // Update end time
                current.UpdateEndTime(endTime);
                
                // Re-add to index
                AddInterval(current);
                
                // Remove from ongoing occupancies
                ongoingOccupancies.Remove(key);
            }
        }

        /// <summary>
        /// Get all time occupancies in this index
        /// </summary>
        public List<TimeOccupancy> GetAllOccupancies()
        {
            return new List<TimeOccupancy>(allOccupancies.Values);
        }

        /// <summary>
        /// Remove all time intervals before a specified time
        /// </summary>
        public void RemoveIntervalsBefore(float cutoffTime)
        {
            // Find all end times earlier than cutoffTime
            var endTimes = endTimeIndex.RangeFrom(float.MinValue).TakeWhile(kvp => kvp.Key <= cutoffTime).ToList();
            
            foreach (var kvp in endTimes)
            {
                float endTime = kvp.Key;
                var intervals = kvp.Value.ToList();
                
                foreach (var interval in intervals)
                {
                    // Remove this time interval
                    RemoveInterval(interval);
                }
            }
        }

        /// <summary>
        /// Get all occupancy records within the specified time range
        /// </summary>
        public List<TimeOccupancy> GetOccupancies(float startTime, float endTime)
        {
            return QueryOverlapping(startTime, endTime);
        }

        /// <summary>
        /// Clean up expired data
        /// </summary>
        public void Cleanup(float currentTime, float windowDuration)
        {
            float cutoffTime = currentTime - windowDuration;
            List<TimeOccupancy> toRemove = new List<TimeOccupancy>();
            
            try
            {
                // Find all occupancies with end time earlier than threshold
                if (!endTimeIndex.IsEmpty)
                {
                    var minKey = endTimeIndex.FindMin().Key;
                    var maxKey = cutoffTime;
                    
                    if (minKey <= maxKey)
                    {
                        var enumerator = endTimeIndex.RangeFrom(minKey).GetEnumerator();
                        while (enumerator.MoveNext())
                        {
                            var entry = enumerator.Current;
                            if (entry.Key > cutoffTime)
                                break;
                                
                            foreach (var interval in entry.Value)
                            {
                                toRemove.Add(interval);
                            }
                        }
                    }
                }
                
                // Remove expired occupancies
                foreach (var interval in toRemove)
                {
                    RemoveInterval(interval);
                }
                
                Debug.Log($"Cleaned up {toRemove.Count} expired occupancy records");
            }
            catch (Exception ex)
            {
                Debug.LogError($"Error cleaning up occupancy records: {ex.Message}");
            }
        }
    }
}