using System;
using UnityEngine;
using CentralControl;

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
        
        public PathPlanningMetrics(string robotId, string planId)
        {
            RobotId = robotId;
            PlanId = planId;
            Timestamp = DateTime.Now;
        }
    }
    
    public class PathPlanningMetricsEventArgs : EventArgs
    {
        public PathPlanningMetrics Metrics { get; private set; }
        
        public PathPlanningMetricsEventArgs(PathPlanningMetrics metrics)
        {
            Metrics = metrics;
        }
    }
}