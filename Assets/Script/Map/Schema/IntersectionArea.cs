using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using NetTopologySuite.Geometries;

namespace Map
{
    public class IntersectionArea
    {
        private string _id;
        private List<Waypoint> _entryPoints;
        private List<Waypoint> _exitPoints;
        private List<CellSpace> _intersectionCells;
        private float _activationDistance;
        private string _floorLevel;

        public string Id { get => _id; private set => _id = value; }
        public List<Waypoint> EntryPoints { get => _entryPoints; }
        public List<Waypoint> ExitPoints { get => _exitPoints; }
        public List<CellSpace> IntersectionCells { get => _intersectionCells; }
        public float ActivationDistance { get => _activationDistance; set => _activationDistance = value; }

        public string FloorLevel { get => _floorLevel; set => _floorLevel = value; }

        private Geometry _intersectionGeometry;
        public Geometry IntersectionGeometry
        {
            get
            {
                if (_intersectionGeometry == null)
                {
                    var geometries = _intersectionCells.Select(cs => cs.Space).ToList();
                    _intersectionGeometry = GeometryFactory.Default.CreateGeometryCollection(geometries.ToArray()).Union();
                }
                return _intersectionGeometry;
            }
        }

        public IntersectionArea(string id)
        {
            _id = id;
            _entryPoints = new List<Waypoint>();
            _exitPoints = new List<Waypoint>();
            _intersectionCells = new List<CellSpace>();
            _activationDistance = 2.0f;
            _floorLevel = "1F";
        }

        public (Vector3 entryDirection, Vector3 suggestedExitDirection) CalculateTraversalDirections(Vector3 robotPosition, Vector3 robotVelocity)
        {
            Waypoint nearestEntryPoint = null;
            float minDistance = float.MaxValue;

            foreach (var entryPoint in _entryPoints)
            {
                Vector3 entryPos = new Vector3(entryPoint.Position.x, robotPosition.y, entryPoint.Position.z);
                float distance = Vector3.Distance(robotPosition, entryPos);

                if (distance < minDistance)
                {
                    minDistance = distance;
                    nearestEntryPoint = entryPoint;
                }
            }

            Vector3 entryDirection = robotVelocity.normalized;
            if (nearestEntryPoint != null)
            {
                Vector3 entryPos = new Vector3(nearestEntryPoint.Position.x, robotPosition.y, nearestEntryPoint.Position.z);
                entryDirection = (robotPosition - entryPos).normalized;
            }

            Vector3 suggestedExitDirection = -entryDirection;
            float bestAlignment = -1f;

            foreach (var exitPoint in _exitPoints)
            {
                Vector3 exitPos = new Vector3(exitPoint.Position.x, robotPosition.y, exitPoint.Position.z);
                Vector3 exitDir = (exitPos - robotPosition).normalized;

                float alignment = Vector3.Dot(entryDirection, exitDir);
                if (alignment > bestAlignment)
                {
                    bestAlignment = alignment;
                    suggestedExitDirection = exitDir;
                }
            }

            return (entryDirection, suggestedExitDirection);
        }

        /// <summary>
        /// Returns cell IDs bordering the intersection.
        /// Uses Waypoint.PrimaryCellId (=Source) for entries and SecondaryCellId (=Target) for exits,
        /// preserving the directional semantics from the old CellBoundary.Source/Target pattern.
        /// </summary>
        public List<string> GetBorderingCellIds()
        {
            List<string> borderingCellIds = new List<string>();

            foreach (var entryPoint in _entryPoints)
            {
                if (!string.IsNullOrEmpty(entryPoint.PrimaryCellId) && !borderingCellIds.Contains(entryPoint.PrimaryCellId))
                    borderingCellIds.Add(entryPoint.PrimaryCellId);
            }

            foreach (var exitPoint in _exitPoints)
            {
                if (!string.IsNullOrEmpty(exitPoint.SecondaryCellId) && !borderingCellIds.Contains(exitPoint.SecondaryCellId))
                    borderingCellIds.Add(exitPoint.SecondaryCellId);
            }

            return borderingCellIds;
        }

        public List<string> GetAllCellIds()
        {
            return _intersectionCells.Select(cell => cell.Id).ToList();
        }

        public void AddEntryPoint(Waypoint point)
        {
            if (!_entryPoints.Any(p => p.Id == point.Id))
                _entryPoints.Add(point);
        }

        public void AddExitPoint(Waypoint point)
        {
            if (!_exitPoints.Any(p => p.Id == point.Id))
                _exitPoints.Add(point);
        }

        public void AddIntersectionCell(CellSpace cell)
        {
            if (!_intersectionCells.Any(c => c.Id == cell.Id))
            {
                _intersectionCells.Add(cell);
                _intersectionGeometry = null;
            }
        }

        public bool Contains(Vector3 position)
        {
            var point = new Point(position.x, position.z);
            return IntersectionGeometry.Contains(point);
        }

        public bool IsNear(Vector3 position, float distance)
        {
            var point = new Point(position.x, position.z);
            return IntersectionGeometry.Distance(point) <= distance;
        }
    }
}
