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
            
            // 将墙父对象也设置为BoundaryWall层
            wallsParent.layer = LayerMask.NameToLayer("BoundaryWall");

            CreateWall(new Vector3(mapWidth / 2, wallHeight / 2, 0), new Vector3(mapWidth + wallThickness, wallHeight, wallThickness), "NorthWall", wallsParent.transform);
            CreateWall(new Vector3(mapWidth / 2, wallHeight / 2, mapLength), new Vector3(mapWidth + wallThickness, wallHeight, wallThickness), "SouthWall", wallsParent.transform);
            CreateWall(new Vector3(0, wallHeight / 2, mapLength / 2), new Vector3(wallThickness, wallHeight, mapLength + wallThickness), "WestWall", wallsParent.transform);
            CreateWall(new Vector3(mapWidth, wallHeight / 2, mapLength / 2), new Vector3(wallThickness, wallHeight, mapLength + wallThickness), "EastWall", wallsParent.transform);
            
            // 创建一个薄的地板来确保NavMesh生成到边界
            CreateFloorBoundary(wallsParent.transform);
        }

        private void CreateWall(Vector3 position, Vector3 scale, string name, Transform parent)
        {
            GameObject wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
            wall.name = name;
            wall.transform.SetParent(parent);
            wall.transform.position = position;
            wall.transform.localScale = scale;

            // 设置层
            wall.layer = LayerMask.NameToLayer("BoundaryWall");

            // 设置材质
            Renderer renderer = wall.GetComponent<Renderer>();
            renderer.material.color = Color.black;

            // 替换BoxCollider为MeshCollider以获得更好的NavMesh边界
            Destroy(wall.GetComponent<BoxCollider>());
            MeshCollider meshCollider = wall.AddComponent<MeshCollider>();
            meshCollider.convex = false; // 非凸碰撞体更适合NavMesh边界

            // 添加静态刚体
            Rigidbody rb = wall.AddComponent<Rigidbody>();
            rb.isKinematic = true;
            rb.useGravity = false;
            
            // 标记为静态物体，这对NavMesh生成很重要
            #if UNITY_EDITOR
            UnityEditor.GameObjectUtility.SetStaticEditorFlags(wall, 
                UnityEditor.StaticEditorFlags.ContributeGI | 
                UnityEditor.StaticEditorFlags.NavigationStatic | 
                UnityEditor.StaticEditorFlags.OccludeeStatic | 
                UnityEditor.StaticEditorFlags.BatchingStatic);
            #endif
        }

        // 新增：创建一个薄的地板边界确保NavMesh延伸到墙壁
        private void CreateFloorBoundary(Transform parent)
        {
            // 创建边缘地板
            GameObject floorBoundary = new GameObject("FloorBoundary");
            floorBoundary.transform.SetParent(parent);
            floorBoundary.transform.position = new Vector3(mapWidth/2, 0.01f, mapLength/2); // 略高于0以避免z-fighting
            
            // 添加地板网格
            MeshFilter meshFilter = floorBoundary.AddComponent<MeshFilter>();
            MeshRenderer meshRenderer = floorBoundary.AddComponent<MeshRenderer>();
            
            // 创建边缘地板网格
            Mesh mesh = new Mesh();
            
            // 创建一个带洞的矩形（外边缘为地图边界，内边缘为实际仓库地板）
            float outerWidth = mapWidth + wallThickness * 2;
            float outerLength = mapLength + wallThickness * 2;
            float innerWidth = mapWidth - 0.1f; // 稍小于实际地图
            float innerLength = mapLength - 0.1f;
            
            Vector3[] vertices = new Vector3[]
            {
                // 外边缘顶点
                new Vector3(-wallThickness, 0, -wallThickness), // 0
                new Vector3(outerWidth - wallThickness, 0, -wallThickness), // 1
                new Vector3(outerWidth - wallThickness, 0, outerLength - wallThickness), // 2
                new Vector3(-wallThickness, 0, outerLength - wallThickness), // 3
                
                // 内边缘顶点
                new Vector3(0.05f, 0, 0.05f), // 4
                new Vector3(innerWidth, 0, 0.05f), // 5
                new Vector3(innerWidth, 0, innerLength), // 6
                new Vector3(0.05f, 0, innerLength) // 7
            };
            
            // 定义外部矩形的三角形
            int[] triangles = new int[]
            {
                // 上边缘
                0, 4, 5,
                0, 5, 1,
                
                // 右边缘
                1, 5, 6,
                1, 6, 2,
                
                // 下边缘
                2, 6, 7,
                2, 7, 3,
                
                // 左边缘
                3, 7, 4,
                3, 4, 0
            };
            
            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.RecalculateNormals();
            
            meshFilter.mesh = mesh;
            
            // 设置材质为半透明
            Material material = new Material(Shader.Find("Standard"));
            material.color = new Color(0.3f, 0.7f, 1f, 0.2f);
            meshRenderer.material = material;
            
            // 添加MeshCollider
            MeshCollider meshCollider = floorBoundary.AddComponent<MeshCollider>();
            meshCollider.sharedMesh = mesh;
            
            // 设置正确的层
            floorBoundary.layer = LayerMask.NameToLayer("GroundLayer");
            
            // 标记为静态物体，这对NavMesh生成很重要
            #if UNITY_EDITOR
            UnityEditor.GameObjectUtility.SetStaticEditorFlags(floorBoundary, 
                UnityEditor.StaticEditorFlags.ContributeGI | 
                UnityEditor.StaticEditorFlags.NavigationStatic | 
                UnityEditor.StaticEditorFlags.BatchingStatic);
            #endif
        }
    }
}
