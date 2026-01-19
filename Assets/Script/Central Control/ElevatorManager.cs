using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;
using System.Threading;
using UnityEngine;
using UnityEngine.AI;
using NetTopologySuite.Geometries;
using PathPlanning;
using Map;


namespace CentralControl
{
    /// <summary>
    /// Elevator Manager - Manages the status and scheduling of all elevators
    /// </summary>
    public class ElevatorManager
    {
        // Elevator status class
        public class ElevatorState
        {
            public string ElevatorId { get; private set; }
            public string CurrentFloor { get; set; }
            public string TargetFloor { get; set; }
            public float EarliestAvailableTime { get; set; }
            public bool IsMoving { get; set; }
            public string CurrentUserId { get; set; }
            public List<string> ConnectedFloors { get; private set; }
            public float Speed { get; set; } // Move time per floor

            public ElevatorState(string elevatorId, string initialFloor, List<string> connectedFloors, float speed = 0.5f)
            {
                ElevatorId = elevatorId;
                CurrentFloor = initialFloor;
                TargetFloor = initialFloor;
                EarliestAvailableTime = 0;
                IsMoving = false;
                CurrentUserId = null;
                ConnectedFloors = connectedFloors ?? new List<string>();
                Speed = speed;
            }

            // Calculates the move time from one floor to another
            public float CalculateMoveTime(string fromFloor, string toFloor)
            {
                int fromLevel = ParseFloorLevel(fromFloor);
                int toLevel = ParseFloorLevel(toFloor);
                int floorDifference = Math.Abs(fromLevel - toLevel);
                
                // Base move time + floor distance
                return 2.0f + Speed * floorDifference;
            }
        }

        // Elevator reservation information
        public class ElevatorReservation
        {
            public string ElevatorId { get; set; }
            public string UserId { get; set; }
            public string FromFloor { get; set; }
            public string ToFloor { get; set; }
            public float StartTime { get; set; }
            public float EndTime { get; set; }
            public string PlanId { get; set; }
        }

        private Dictionary<string, ElevatorState> elevators = new Dictionary<string, ElevatorState>();
        private List<ElevatorReservation> reservations = new List<ElevatorReservation>();
        private IndoorSpace indoorSpace;

        public ElevatorManager(IndoorSpace indoorSpace)
        {
            this.indoorSpace = indoorSpace;
            InitializeElevators();
        }

        // Initializes all elevators
        private void InitializeElevators()
        {
            // Get all elevator units from IndoorSpace
            var allElevators = indoorSpace.CellSpaces.Where(cs => cs.IsElevator()).ToList();
            
            foreach (var elevator in allElevators)
            {
                string elevatorId = elevator.Id;
                string initialFloor = "1F"; 
                List<string> connectedFloors = elevator.GetConnectedFloors();
                
                // Create and add elevator state
                elevators[elevatorId] = new ElevatorState(elevatorId, initialFloor, connectedFloors);
                
                Debug.Log($"Initialized elevator {elevatorId} at floor {initialFloor}, connected to {string.Join(", ", connectedFloors)}");
            }
        }

        // Gets the best available elevator and wait time
        public (string elevatorId, float waitTime, float totalTime) GetBestElevator(
            string fromFloor, 
            string toFloor, 
            float requestTime,
            string userId)
        {
            // Find all elevators that can connect these two floors
            var eligibleElevators = elevators.Values
                .Where(e => e.ConnectedFloors.Contains(fromFloor) && e.ConnectedFloors.Contains(toFloor))
                .ToList();
            
            if (eligibleElevators.Count == 0)
            {
                Debug.LogWarning($"No elevator connects floors {fromFloor} and {toFloor}");
                return (null, float.MaxValue, float.MaxValue);
            }
            
            string bestElevatorId = null;
            float minWaitTime = float.MaxValue;
            float minTotalTime = float.MaxValue;
            
            foreach (var elevator in eligibleElevators)
            {
                // Check reservations for this elevator
                var relevantReservations = reservations
                    .Where(r => r.ElevatorId == elevator.ElevatorId && r.EndTime > requestTime)
                    .OrderBy(r => r.StartTime)
                    .ToList();
                
                // Calculate elevator arrival time at fromFloor
                float elevatorArrivalTime = CalculateElevatorArrivalTime(
                    elevator, 
                    fromFloor, 
                    requestTime, 
                    relevantReservations);
                
                // Calculate wait time
                float waitTime = Math.Max(0, elevatorArrivalTime - requestTime);
                
                // Calculate move time from fromFloor to toFloor
                float moveTime = elevator.CalculateMoveTime(fromFloor, toFloor);
                
                // Calculate total time
                float totalTime = waitTime + moveTime;
                
                // Find the best elevator
                if (totalTime < minTotalTime)
                {
                    minTotalTime = totalTime;
                    minWaitTime = waitTime;
                    bestElevatorId = elevator.ElevatorId;
                }
            }
            
            return (bestElevatorId, minWaitTime, minTotalTime);
        }
        
        // Calculates the arrival time of the elevator at the specified floor
        private float CalculateElevatorArrivalTime(
            ElevatorState elevator, 
            string targetFloor, 
            float currentTime,
            List<ElevatorReservation> relevantReservations)
        {
            // If elevator is idle and already on the target floor
            if (!elevator.IsMoving && elevator.CurrentFloor == targetFloor && currentTime >= elevator.EarliestAvailableTime)
            {
                return currentTime;
            }
            
            // If elevator has reservations, wait for it to finish current tasks
            if (relevantReservations.Count > 0)
            {
                // Find the end time of the last reservation
                float lastReservationEnd = relevantReservations.Max(r => r.EndTime);
                
                // Elevator will be at the target floor of the last reservation
                string finalPosition = relevantReservations.Last().ToFloor;
                
                // Calculate move time from final position to target floor
                float moveTimeToTarget = elevator.CalculateMoveTime(finalPosition, targetFloor);
                
                return lastReservationEnd + moveTimeToTarget;
            }
            
            // If elevator is idle but not on the target floor
            if (!elevator.IsMoving && elevator.CurrentFloor != targetFloor && currentTime >= elevator.EarliestAvailableTime)
            {
                // Calculate move time from current floor to target floor
                float moveTime = elevator.CalculateMoveTime(elevator.CurrentFloor, targetFloor);
                return currentTime + moveTime;
            }
            
            // If elevator is currently moving
            if (elevator.IsMoving)
            {
                // Wait for elevator to reach its target position
                float timeToFinishCurrentMove = elevator.EarliestAvailableTime;
                
                // Then move from that position to the requested floor
                float moveTimeToTarget = elevator.CalculateMoveTime(elevator.TargetFloor, targetFloor);
                
                return Math.Max(currentTime, timeToFinishCurrentMove) + moveTimeToTarget;
            }
            
            // Default case: return current time (conservative estimate)
            return currentTime;
        }

        // Reserve elevator use
        public bool ReserveElevator(
            string elevatorId, 
            string userId, 
            string fromFloor, 
            string toFloor, 
            float startTime, 
            float totalTime,
            string planId)
        {
            if (!elevators.ContainsKey(elevatorId))
            {
                Debug.LogError($"Elevator {elevatorId} not found");
                return false;
            }
            
            var elevator = elevators[elevatorId];
            
            // Check if elevator connects these two floors
            if (!elevator.ConnectedFloors.Contains(fromFloor) || !elevator.ConnectedFloors.Contains(toFloor))
            {
                Debug.LogError($"Elevator {elevatorId} does not connect floors {fromFloor} and {toFloor}");
                return false;
            }
            
            // Check for conflicts with existing reservations
            var conflictingReservations = reservations.Where(r => 
                r.ElevatorId == elevatorId && 
                ((r.StartTime <= startTime && r.EndTime > startTime) || 
                (r.StartTime < startTime + totalTime && r.EndTime >= startTime + totalTime) ||
                (r.StartTime >= startTime && r.EndTime <= startTime + totalTime)))
                .ToList();
            
            if (conflictingReservations.Count > 0)
            {
                Debug.LogWarning($"Elevator {elevatorId} already has conflicting reservations at the requested time");
                return false;
            }
            
            // Create new reservation
            var reservation = new ElevatorReservation
            {
                ElevatorId = elevatorId,
                UserId = userId,
                FromFloor = fromFloor,
                ToFloor = toFloor,
                StartTime = startTime,
                EndTime = startTime + totalTime,
                PlanId = planId
            };
            
            reservations.Add(reservation);
            
            // Update elevator status
            elevator.EarliestAvailableTime = startTime + totalTime;
            elevator.TargetFloor = toFloor;
            elevator.CurrentUserId = userId;
            elevator.IsMoving = true;
            
            // Update elevator status at end of schedule
            UpdateElevatorStateAfterReservation(elevatorId, toFloor, startTime + totalTime);
            
            Debug.Log($"Reserved elevator {elevatorId} for user {userId} from {fromFloor} to {toFloor} " +
                    $"at time {startTime}, will be available again at {startTime + totalTime}");
            
            return true;
        }
        
        // Updates elevator status after reservation is completed
        private void UpdateElevatorStateAfterReservation(string elevatorId, string finalFloor, float availableTime)
        {
            if (elevators.ContainsKey(elevatorId))
            {
                var elevator = elevators[elevatorId];
                
                // Set timer to update elevator status when reservation ends
                // In actual implementation, this may require an event or coroutine
                // Simplified here as direct status setting
                elevator.CurrentFloor = finalFloor;
                elevator.TargetFloor = finalFloor;
                elevator.EarliestAvailableTime = availableTime;
                elevator.IsMoving = false;
                elevator.CurrentUserId = null;
            }
        }
        
        // Cancels reservation
        public bool CancelReservation(string userId, string planId)
        {
            var toRemove = reservations.Where(r => r.UserId == userId && r.PlanId == planId).ToList();
            
            foreach (var reservation in toRemove)
            {
                reservations.Remove(reservation);
                
                // If elevator is currently used by this user, reset status
                if (elevators.ContainsKey(reservation.ElevatorId) && 
                    elevators[reservation.ElevatorId].CurrentUserId == userId)
                {
                    elevators[reservation.ElevatorId].CurrentUserId = null;
                    elevators[reservation.ElevatorId].IsMoving = false;
                }
                
                Debug.Log($"Canceled reservation for elevator {reservation.ElevatorId} by user {userId}, plan {planId}");
            }
            
            return toRemove.Count > 0;
        }
        
        // Parses floor level string into an integer (for comparison)
        private static int ParseFloorLevel(string floorLevel)
        {
            if (string.IsNullOrEmpty(floorLevel))
                return 0;
                
            // Handle basement floors, e.g., "B1"
            if (floorLevel.StartsWith("B") || floorLevel.StartsWith("b"))
            {
                if (int.TryParse(floorLevel.Substring(1).Replace("F", "").Replace("f", ""), out int level))
                {
                    return -level;
                }
            }
            // Handle regular floors, e.g., "1F"
            else
            {
                if (int.TryParse(floorLevel.Replace("F", "").Replace("f", ""), out int level))
                {
                    return level;
                }
            }
            
            return 0; // Default value
        }
    }
}