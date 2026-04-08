using Map;

namespace PathPlanning
{
    /// <summary>
    /// IPathFinder implementation for the TimeSpaceAStar algorithm.
    /// Now delegates directly to the ported TimeSpaceAStar which works natively with
    /// NavigationGraph and Waypoint — no legacy Graph conversion needed.
    /// </summary>
    public class TimeSpaceAStarPathFinder : IPathFinder
    {
        private readonly IndoorSpace indoorSpace;
        private readonly SearchMethod searchMethod;

        public TimeSpaceAStarPathFinder(IndoorSpace indoorSpace, SearchMethod searchMethod)
        {
            this.indoorSpace = indoorSpace;
            this.searchMethod = searchMethod;
        }

        public NavigationPathResult FindPath(
            NavigationGraph graph, Waypoint start, Waypoint end,
            DynamicOccupancyLayer personalOcc, DynamicOccupancyLayer globalOcc,
            string robotId, string currentPlanId,
            float startTime, float speed, float maxAccel, float maxDecel)
        {
            if (start == null || end == null) return null;

            var (path, times, speeds, planId, metrics) = TimeSpaceAStar.TimeSpaceAStarAlgorithm(
                graph, indoorSpace, personalOcc, globalOcc,
                start, end, robotId, currentPlanId,
                startTime, speed, maxAccel, maxDecel, searchMethod);

            if (path == null || path.Count == 0) return null;

            return new NavigationPathResult
            {
                Path = path,
                Times = times,
                Speeds = speeds,
                PlanId = planId,
                Metrics = metrics
            };
        }
    }
}
