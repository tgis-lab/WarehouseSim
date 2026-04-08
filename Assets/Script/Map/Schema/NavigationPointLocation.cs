namespace Map
{
    /// <summary>
    /// Replaces Graph.PointLocation so planner code no longer depends on the legacy graph type.
    /// </summary>
    public enum NavigationPointLocation
    {
        InAisle,
        ApproachingIntersection,
        InIntersection,
        OutIntersection
    }
}
