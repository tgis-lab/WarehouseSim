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
    /// 电梯管理器 - 管理所有电梯的状态和调度
    /// </summary>
    public class ElevatorManager
    {
        // 电梯状态类
        public class ElevatorState
        {
            public string ElevatorId { get; private set; }
            public string CurrentFloor { get; set; }
            public string TargetFloor { get; set; }
            public float EarliestAvailableTime { get; set; }
            public bool IsMoving { get; set; }
            public string CurrentUserId { get; set; }
            public List<string> ConnectedFloors { get; private set; }
            public float Speed { get; set; } // 每层楼的移动时间

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

            // 计算从一个楼层到另一个楼层的移动时间
            public float CalculateMoveTime(string fromFloor, string toFloor)
            {
                int fromLevel = ParseFloorLevel(fromFloor);
                int toLevel = ParseFloorLevel(toFloor);
                int floorDifference = Math.Abs(fromLevel - toLevel);
                
                // 基础移动时间 + 楼层距离
                return 2.0f + Speed * floorDifference;
            }
        }

        // 电梯预约信息
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

        // 初始化所有电梯
        private void InitializeElevators()
        {
            // 从室内空间获取所有电梯单元
            var allElevators = indoorSpace.CellSpaces.Where(cs => cs.IsElevator()).ToList();
            
            foreach (var elevator in allElevators)
            {
                string elevatorId = elevator.Id;
                string initialFloor = "1F"; 
                List<string> connectedFloors = elevator.GetConnectedFloors();
                
                // 创建并添加电梯状态
                elevators[elevatorId] = new ElevatorState(elevatorId, initialFloor, connectedFloors);
                
                Debug.Log($"Initialized elevator {elevatorId} at floor {initialFloor}, connected to {string.Join(", ", connectedFloors)}");
            }
        }

        // 获取最佳可用电梯及等待时间
        public (string elevatorId, float waitTime, float totalTime) GetBestElevator(
            string fromFloor, 
            string toFloor, 
            float requestTime,
            string userId)
        {
            // 找出所有能连接这两个楼层的电梯
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
                // 检查该电梯的预约情况
                var relevantReservations = reservations
                    .Where(r => r.ElevatorId == elevator.ElevatorId && r.EndTime > requestTime)
                    .OrderBy(r => r.StartTime)
                    .ToList();
                
                // 计算电梯到达fromFloor的时间
                float elevatorArrivalTime = CalculateElevatorArrivalTime(
                    elevator, 
                    fromFloor, 
                    requestTime, 
                    relevantReservations);
                
                // 计算等待时间
                float waitTime = Math.Max(0, elevatorArrivalTime - requestTime);
                
                // 计算从fromFloor到toFloor的移动时间
                float moveTime = elevator.CalculateMoveTime(fromFloor, toFloor);
                
                // 计算总时间
                float totalTime = waitTime + moveTime;
                
                // 找出最佳电梯
                if (totalTime < minTotalTime)
                {
                    minTotalTime = totalTime;
                    minWaitTime = waitTime;
                    bestElevatorId = elevator.ElevatorId;
                }
            }
            
            return (bestElevatorId, minWaitTime, minTotalTime);
        }
        
        // 计算电梯到达指定楼层的时间
        private float CalculateElevatorArrivalTime(
            ElevatorState elevator, 
            string targetFloor, 
            float currentTime,
            List<ElevatorReservation> relevantReservations)
        {
            // 如果电梯当前空闲且已在目标楼层
            if (!elevator.IsMoving && elevator.CurrentFloor == targetFloor && currentTime >= elevator.EarliestAvailableTime)
            {
                return currentTime;
            }
            
            // 如果电梯当前有预约，需要等待电梯完成当前任务
            if (relevantReservations.Count > 0)
            {
                // 找到最后一个预约的结束时间
                float lastReservationEnd = relevantReservations.Max(r => r.EndTime);
                
                // 电梯将位于最后一个预约的目标楼层
                string finalPosition = relevantReservations.Last().ToFloor;
                
                // 计算从最后位置到目标楼层的时间
                float moveTimeToTarget = elevator.CalculateMoveTime(finalPosition, targetFloor);
                
                return lastReservationEnd + moveTimeToTarget;
            }
            
            // 如果电梯当前空闲但不在目标楼层
            if (!elevator.IsMoving && elevator.CurrentFloor != targetFloor && currentTime >= elevator.EarliestAvailableTime)
            {
                // 计算从当前楼层到目标楼层的时间
                float moveTime = elevator.CalculateMoveTime(elevator.CurrentFloor, targetFloor);
                return currentTime + moveTime;
            }
            
            // 如果电梯当前正在移动
            if (elevator.IsMoving)
            {
                // 等待电梯到达目标位置
                float timeToFinishCurrentMove = elevator.EarliestAvailableTime;
                
                // 然后从该位置移动到请求的楼层
                float moveTimeToTarget = elevator.CalculateMoveTime(elevator.TargetFloor, targetFloor);
                
                return Math.Max(currentTime, timeToFinishCurrentMove) + moveTimeToTarget;
            }
            
            // 默认情况：返回当前时间（保守估计）
            return currentTime;
        }

        // 预约电梯的使用
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
            
            // 检查电梯是否连接这两个楼层
            if (!elevator.ConnectedFloors.Contains(fromFloor) || !elevator.ConnectedFloors.Contains(toFloor))
            {
                Debug.LogError($"Elevator {elevatorId} does not connect floors {fromFloor} and {toFloor}");
                return false;
            }
            
            // 检查是否与现有预约冲突
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
            
            // 创建新预约
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
            
            // 更新电梯状态
            elevator.EarliestAvailableTime = startTime + totalTime;
            elevator.TargetFloor = toFloor;
            elevator.CurrentUserId = userId;
            elevator.IsMoving = true;
            
            // 调度结束时更新电梯状态
            UpdateElevatorStateAfterReservation(elevatorId, toFloor, startTime + totalTime);
            
            Debug.Log($"Reserved elevator {elevatorId} for user {userId} from {fromFloor} to {toFloor} " +
                    $"at time {startTime}, will be available again at {startTime + totalTime}");
            
            return true;
        }
        
        // 在预约完成后更新电梯状态
        private void UpdateElevatorStateAfterReservation(string elevatorId, string finalFloor, float availableTime)
        {
            if (elevators.ContainsKey(elevatorId))
            {
                var elevator = elevators[elevatorId];
                
                // 设置定时器，在预约结束时更新电梯状态
                // 在实际实现中，这可能需要一个事件或协程
                // 这里简化为直接设置状态
                elevator.CurrentFloor = finalFloor;
                elevator.TargetFloor = finalFloor;
                elevator.EarliestAvailableTime = availableTime;
                elevator.IsMoving = false;
                elevator.CurrentUserId = null;
            }
        }
        
        // 取消预约
        public bool CancelReservation(string userId, string planId)
        {
            var toRemove = reservations.Where(r => r.UserId == userId && r.PlanId == planId).ToList();
            
            foreach (var reservation in toRemove)
            {
                reservations.Remove(reservation);
                
                // 如果电梯当前被该用户使用，重置状态
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
        
        // 解析楼层编号为整数（用于比较）
        private static int ParseFloorLevel(string floorLevel)
        {
            if (string.IsNullOrEmpty(floorLevel))
                return 0;
                
            // 处理地下楼层，如"B1"
            if (floorLevel.StartsWith("B") || floorLevel.StartsWith("b"))
            {
                if (int.TryParse(floorLevel.Substring(1).Replace("F", "").Replace("f", ""), out int level))
                {
                    return -level;
                }
            }
            // 处理地上楼层，如"1F"
            else
            {
                if (int.TryParse(floorLevel.Replace("F", "").Replace("f", ""), out int level))
                {
                    return level;
                }
            }
            
            return 0; // 默认值
        }
    }
}