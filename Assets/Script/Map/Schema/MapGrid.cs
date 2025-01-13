using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Map
{
    public class MapGrid
    {
        public float width, height, length;
        private float gridSize = 0.5f;
        private Vector3 mapOrigin;
        public int gridWidth, gridLength;

        public MapGrid(float width, float height, float length)
        {
            this.width = width;
            this.height = height;
            this.length = length;
        }

        public void InitializeGrid(IndoorSpace indoorSpace)
        {
            float minX = float.MaxValue, minZ = float.MaxValue;
            float maxX = float.MinValue, maxZ = float.MinValue;

            foreach (var cellSpace in indoorSpace.CellSpaces)
            {
                var envelope = cellSpace.Space.EnvelopeInternal;
                minX = Mathf.Min(minX, (float)envelope.MinX);
                minZ = Mathf.Min(minZ, (float)envelope.MinY);
                maxX = Mathf.Max(maxX, (float)envelope.MaxX);
                maxZ = Mathf.Max(maxZ, (float)envelope.MaxY);
            }

            mapOrigin = new Vector3(minX, 0, minZ);
            gridWidth = Mathf.CeilToInt((maxX - minX) / gridSize);
            gridLength = Mathf.CeilToInt((maxZ - minZ) / gridSize);

            Debug.Log($"Grid dimensions: {gridWidth} x {gridLength} cells");
        }

        public Vector3Int GetGridIndex(Vector3 position)
        {
            int x = Mathf.FloorToInt((position.x - mapOrigin.x) / gridSize);
            int z = Mathf.FloorToInt((position.z - mapOrigin.z) / gridSize);
            return new Vector3Int(x, 0, z);
        }

        public Vector3 GetWorldPosition(Vector3Int gridIndex)
        {
            float x = gridIndex.x * gridSize + mapOrigin.x;
            float z = gridIndex.z * gridSize + mapOrigin.z;
            return new Vector3(x, 0, z);
        }

        public Dictionary<string, Vector3Int> GetCellSpaceGridPositions(IndoorSpace indoorSpace)
        {
            Dictionary<string, Vector3Int> cellSpaceGridPositions = new Dictionary<string, Vector3Int>();
            foreach (var cellSpace in indoorSpace.CellSpaces)
            {
                Vector3 centroid = new Vector3((float)cellSpace.Space.Centroid.X, 0, (float)cellSpace.Space.Centroid.Y);
                cellSpaceGridPositions.Add(cellSpace.Id, GetGridIndex(centroid));
            }
            return cellSpaceGridPositions;
        }
    }
}