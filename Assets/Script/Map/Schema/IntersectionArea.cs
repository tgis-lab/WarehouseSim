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
        private float _deactivationDistance;
        private Geometry _intersectionGeometry;

        public string Id { get => _id; private set => _id = value; }
        public List<ConnectionPoint> EntryPoints { get => _entryPoints; }
        public List<ConnectionPoint> ExitPoints { get => _exitPoints; }
        public List<CellSpace> IntersectionCells { get => _intersectionCells; }
        public float ActivationDistance { get => _activationDistance; set => _activationDistance = value; }
        public float DeactivationDistance { get => _deactivationDistance; set => _deactivationDistance = value; }
        
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
            _deactivationDistance = 1.5f;
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

        public bool ShouldActivateDWA(Vector3 position)
        {
            return IsNear(position, _activationDistance);
        }

        public bool ShouldDeactivateDWA(Vector3 position)
        {
            return !IsNear(position, _deactivationDistance);
        }
    }
}