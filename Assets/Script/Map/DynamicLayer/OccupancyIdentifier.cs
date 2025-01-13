using System;

namespace Map
{
    public class OccupancyIdentifier
    {
        public string RobotId { get; private set; }
        public string PlanId { get; private set; }
        public OccupancyType Type { get; private set; }

        public OccupancyIdentifier(string robotId, string planId = null, OccupancyType type = OccupancyType.Realtime)
        {
            if (string.IsNullOrEmpty(robotId))
            {
                throw new ArgumentException("Robot ID cannot be null or empty", nameof(robotId));
            }

            RobotId = robotId;
            PlanId = planId;
            Type = type;

            if (type == OccupancyType.Planned && string.IsNullOrEmpty(planId))
            {
                throw new ArgumentException("Plan ID must be provided for planned occupancy", nameof(planId));
            }
        }

        public override string ToString()
        {
            if (Type == OccupancyType.Realtime)
            {
                return $"Robot_{RobotId}";
            }
            return $"Robot_{RobotId}_Plan_{PlanId}";
        }

        public override bool Equals(object obj)
        {
            if (obj is OccupancyIdentifier other)
            {
                return RobotId == other.RobotId &&
                       PlanId == other.PlanId &&
                       Type == other.Type;
            }
            return false;
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(RobotId, PlanId, Type);
        }
    }

    public enum OccupancyType
    {
        Realtime,
        Planned
    }
}