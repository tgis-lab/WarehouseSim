using System;

namespace PathPlanning
{
    public class PathPlanningMetrics
    {
        public string RobotId { get; set; }
        public string PlanId { get; set; }
        public float PlanningTime { get; set; }
        public float ActualPathLength { get; set; }
        public float TravelTime { get; set; }
        public float IdealPathLength { get; set; }
        public DateTime Timestamp { get; set; }
        public int ReplanCount { get; set; }
        public float WaitTime { get; set; }
        public bool IsReplan { get; set; }
        
        public float PathEfficiency => IdealPathLength > 0 && ActualPathLength > 0
            ? IdealPathLength / ActualPathLength
            : 0f;

        /// <summary>
        /// Distance Relative Error: (actual - ideal) / ideal.
        /// Measures how much longer the actual path is compared to the optimal.
        /// </summary>
        public float DistanceRelativeError => IdealPathLength > 0
            ? (ActualPathLength - IdealPathLength) / IdealPathLength
            : 0f;
        
        public PathPlanningMetrics(string robotId, string planId, bool isReplan = false)
        {
            RobotId = robotId;
            PlanId = planId;
            Timestamp = DateTime.Now;
            ReplanCount = 0;
            WaitTime = 0f;
            IsReplan = isReplan;
        }
    }
}