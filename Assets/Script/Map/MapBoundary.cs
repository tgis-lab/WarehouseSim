using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Map
{
    public class MapBoundary : MonoBehaviour
    {
        public float mapWidth = 25f;
        public float mapLength = 20f;
        public float wallHeight = 5f;
        public float wallThickness = 0f;

        public void Initialize()
        {
            CreateBoundaryWalls();
        }

        private void CreateBoundaryWalls()
        {
            GameObject wallsParent = new GameObject("BoundaryWalls");
            wallsParent.transform.SetParent(transform);

            CreateWall(new Vector3(mapWidth / 2, wallHeight / 2, 0), new Vector3(mapWidth + wallThickness, wallHeight, wallThickness), "NorthWall", wallsParent.transform);
            CreateWall(new Vector3(mapWidth / 2, wallHeight / 2, mapLength), new Vector3(mapWidth + wallThickness, wallHeight, wallThickness), "SouthWall", wallsParent.transform);
            CreateWall(new Vector3(0, wallHeight / 2, mapLength / 2), new Vector3(wallThickness, wallHeight, mapLength + wallThickness), "WestWall", wallsParent.transform);
            CreateWall(new Vector3(mapWidth, wallHeight / 2, mapLength / 2), new Vector3(wallThickness, wallHeight, mapLength + wallThickness), "EastWall", wallsParent.transform);
        }

        private void CreateWall(Vector3 position, Vector3 scale, string name, Transform parent)
        {
            GameObject wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
            wall.name = name;
            wall.transform.SetParent(parent);
            wall.transform.position = position;
            wall.transform.localScale = scale;

            Renderer renderer = wall.GetComponent<Renderer>();

            wall.GetComponent<MeshRenderer>().material.color = Color.black;

            Rigidbody rb = wall.AddComponent<Rigidbody>();
            rb.isKinematic = true;

            wall.layer = LayerMask.NameToLayer("BoundaryWall");
        }
    }
}
