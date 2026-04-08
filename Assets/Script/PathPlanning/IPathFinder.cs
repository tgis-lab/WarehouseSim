using Map;

namespace PathPlanning
{
    /// <summary>
    /// Common interface for all waypoint-based planners.
    /// </summary>
    public interface IPathFinder
    {
        NavigationPathResult FindPath(
            NavigationGraph graph,
            Waypoint start,
            Waypoint end,
            DynamicOccupancyLayer personalOcc,
            DynamicOccupancyLayer globalOcc,
            string robotId,
            string currentPlanId,
            float startTime,
            float speed,
            float maxAccel,
            float maxDecel);
    }
}
