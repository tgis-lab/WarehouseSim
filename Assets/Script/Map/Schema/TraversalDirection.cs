namespace Map
{
    /// <summary>
    /// Replaces the ambiguous bool sequence parameter in Graph queries.
    /// Outbound = looking from cell-as-source (sequence=true) — used for start resolution.
    /// Inbound = looking from cell-as-target (sequence=false) — used for goal resolution.
    /// </summary>
    public enum TraversalDirection
    {
        Outbound,
        Inbound
    }
}
