using System.Collections.Generic;
using Map;

namespace PathPlanning
{
    /// <summary>
    /// Shared result DTO for all waypoint-based planners.
    /// Replaces the (List&lt;ConnectionPoint&gt;, List&lt;float&gt;, List&lt;float&gt;, string, PathPlanningMetrics) tuple.
    /// </summary>
    public sealed class NavigationPathResult
    {
        public List<Waypoint> Path { get; set; }
        public List<float> Times { get; set; }
        public List<float> Speeds { get; set; }
        public string PlanId { get; set; }
        public PathPlanningMetrics Metrics { get; set; }
    }
}
