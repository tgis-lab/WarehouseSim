using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using NetTopologySuite.Geometries;

namespace Map
{
    public class IntersectionArea
    {
        private string _id;
        private List<ConnectionPoint> _entryPoints;
        private List<ConnectionPoint> _exitPoints;
        private List<CellSpace> _intersectionCells;
        private float _activationDistance;
        private string _floorLevel;

        public string Id { get => _id; private set => _id = value; }
        public List<ConnectionPoint> EntryPoints { get => _entryPoints; }
        public List<ConnectionPoint> ExitPoints { get => _exitPoints; }
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
            _entryPoints = new List<ConnectionPoint>();
            _exitPoints = new List<ConnectionPoint>();
            _intersectionCells = new List<CellSpace>();
            _activationDistance = 2.0f;
            _floorLevel = "1F"; // Defaults to floor 1F
        }

        public (Vector3 entryDirection, Vector3 suggestedExitDirection) CalculateTraversalDirections(Vector3 robotPosition, Vector3 robotVelocity)
        {
            // Identify direction robot enters from
            ConnectionPoint nearestEntryPoint = null;
            float minDistance = float.MaxValue;
            
            foreach (var entryPoint in _entryPoints)
            {
                Point entry = (Point) entryPoint.Point;
                Vector3 entryPos = new Vector3((float)entry.X, robotPosition.y, (float)entry.Y);
                float distance = Vector3.Distance(robotPosition, entryPos);
                
                if (distance < minDistance)
                {
                    minDistance = distance;
                    nearestEntryPoint = entryPoint;
                }
            }
            
            // If no entry point found, use current velocity direction
            Vector3 entryDirection = robotVelocity.normalized;
            if (nearestEntryPoint != null)
            {
                Point nearestEntry = (Point) nearestEntryPoint.Point;
                Vector3 entryPos = new Vector3((float)nearestEntry.X, robotPosition.y, (float)nearestEntry.Y);
                entryDirection = (robotPosition - entryPos).normalized;
            }
            
            // Find optimal exit direction (roughly opposite to entry direction)
            Vector3 suggestedExitDirection = -entryDirection;
            float bestAlignment = -1f;
            
            foreach (var exitPoint in _exitPoints)
            {
                Point exit = (Point) exitPoint.Point;
                Vector3 exitPos = new Vector3((float)exit.X, robotPosition.y, (float)exit.Y);
                Vector3 exitDir = (exitPos - robotPosition).normalized;
                
                // Find exit point best aligned with entry direction
                float alignment = Vector3.Dot(entryDirection, exitDir);
                if (alignment > bestAlignment)
                {
                    bestAlignment = alignment;
                    suggestedExitDirection = exitDir;
                }
            }
            
            return (entryDirection, suggestedExitDirection);
        }

        public List<string> GetBorderingCellIds()
        {
            List<string> borderingCellIds = new List<string>();
            
            foreach (var entryPoint in _entryPoints)
            {
                if (!borderingCellIds.Contains(entryPoint.CellBoundary.Source))
                    borderingCellIds.Add(entryPoint.CellBoundary.Source);
            }
            
            foreach (var exitPoint in _exitPoints)
            {
                if (!borderingCellIds.Contains(exitPoint.CellBoundary.Target))
                    borderingCellIds.Add(exitPoint.CellBoundary.Target);
            }
            
            return borderingCellIds;
        }

        public List<string> GetAllCellIds()
        {
            return _intersectionCells.Select(cell => cell.Id).ToList();
        }

        public void AddEntryPoint(ConnectionPoint point)
        {
            if (!_entryPoints.Any(p => p.Id == point.Id))
                _entryPoints.Add(point);
        }

        public void AddExitPoint(ConnectionPoint point)
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