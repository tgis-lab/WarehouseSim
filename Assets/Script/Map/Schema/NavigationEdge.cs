namespace Map
{
    /// <summary>
    /// Directed edge in the navigation graph.
    /// EnterCellId/ExitCellId are critical for DynamicOccupancyLayer — it needs to know
    /// which CellSpaces a path segment traverses.
    /// </summary>
    public sealed class NavigationEdge
    {
        public string FromId { get; }
        public string ToId { get; }
        public string EnterCellId { get; }
        public string ExitCellId { get; }
        public bool IsVertical { get; }
        public CellBoundary BoundarySource { get; }

        public NavigationEdge(string fromId, string toId, string enterCellId, string exitCellId,
            bool isVertical, CellBoundary boundarySource)
        {
            FromId = fromId;
            ToId = toId;
            EnterCellId = enterCellId;
            ExitCellId = exitCellId;
            IsVertical = isVertical;
            BoundarySource = boundarySource;
        }
    }
}
