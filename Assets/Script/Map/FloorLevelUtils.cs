namespace Map
{
    /// <summary>
    /// Shared utility for parsing floor level strings into integers.
    /// Consolidates the duplicated ParseFloorLevel logic found across the codebase.
    /// </summary>
    public static class FloorLevelUtils
    {
        /// <summary>
        /// Parses a floor level string into an integer.
        /// "1F" -> 1, "2F" -> 2, "B1" -> -1, "B2" -> -2.
        /// Also handles "F1" format and bare numeric strings like "3".
        /// </summary>
        /// <param name="floorLevel">The floor level string to parse.</param>
        /// <param name="defaultValue">Value returned when parsing fails or input is null/empty.</param>
        /// <returns>The parsed floor level as an integer, or <paramref name="defaultValue"/> on failure.</returns>
        public static int Parse(string floorLevel, int defaultValue = 0)
        {
            if (string.IsNullOrEmpty(floorLevel))
                return defaultValue;

            // Basement floors: B1, B1F, B2, b1, etc.
            if (floorLevel.StartsWith("B") || floorLevel.StartsWith("b"))
            {
                string num = floorLevel.Substring(1).Replace("F", "").Replace("f", "");
                if (int.TryParse(num, out int level))
                {
                    return -level;
                }
            }
            // Above-ground floors: "1F", "2F", "F1", "F2", etc.
            else if (floorLevel.StartsWith("F") || floorLevel.EndsWith("F")
                  || floorLevel.StartsWith("f") || floorLevel.EndsWith("f"))
            {
                string num = floorLevel.Replace("F", "").Replace("f", "");
                if (int.TryParse(num, out int level))
                {
                    return level;
                }
            }

            // Bare numeric string fallback: "1", "2", "-1", etc.
            if (int.TryParse(floorLevel, out int simpleLevel))
            {
                return simpleLevel;
            }

            return defaultValue;
        }

        /// <summary>
        /// Computes the world-space Y position for a given floor level string.
        /// Positive levels (1F, 2F...): (level - 1) * floorHeight + baseHeight
        /// Zero/negative levels (B1 → -1, B2 → -2...): level * floorHeight + baseHeight
        /// </summary>
        /// <param name="floorLevel">Floor level string, e.g. "1F", "2F", "B1".</param>
        /// <param name="floorHeight">Height of each floor in world units.</param>
        /// <param name="baseHeight">Base offset above the computed floor plane.</param>
        /// <returns>World-space Y coordinate for the floor.</returns>
        public static float GetFloorHeight(string floorLevel, float floorHeight = 3.0f, float baseHeight = 0.05f)
        {
            int level = Parse(floorLevel, defaultValue: 1);
            if (level > 0)
            {
                return ((level - 1) * floorHeight) + baseHeight;
            }
            return (level * floorHeight) + baseHeight;
        }
    }
}
