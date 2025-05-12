using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Map;

namespace Map
{
    public class MapBoundary : MonoBehaviour
    {
        public float wallHeight = 9f;
        public float wallThickness = 0.2f;
        
        [Range(0f, 1f)]
        public float wallTransparency = 0f; 
        
        public Color wallColor = new Color(0.2f, 0.2f, 0.8f);
        public Material boundaryMaterial;
        public float boundaryExtension = 0.2f;
        private float minX, maxX, minZ, maxZ;
        private bool boundaryDetected = false;

        public void Initialize(IndoorSpace indoorSpace = null)
        {
            if (indoorSpace != null)
            {
                DetectBoundaryFromIndoorSpace(indoorSpace);
            }
            else
            {
                MapLoader mapLoader = FindObjectOfType<MapLoader>();
                if (mapLoader != null && mapLoader.indoorSpace != null)
                {
                    DetectBoundaryFromIndoorSpace(mapLoader.indoorSpace);
                }
                else
                {
                    DetectBoundaryFromScene();
                }
            }
            
            if (boundaryDetected)
            {
                CreateBoundaryWalls();
            }
            else
            {
                Debug.LogError("无法检测地图边界。请确保场景中有地图对象或手动设置边界。");
            }
        }
        
        // 从IndoorSpace检测边界
        private void DetectBoundaryFromIndoorSpace(IndoorSpace indoorSpace)
        {
            minX = float.MaxValue;
            maxX = float.MinValue;
            minZ = float.MaxValue;
            maxZ = float.MinValue;
            
            // 遍历所有CellSpace，找出最大和最小的x和z坐标
            foreach (var cellSpace in indoorSpace.CellSpaces)
            {
                var envelope = cellSpace.Space.EnvelopeInternal;
                
                minX = Mathf.Min(minX, (float)envelope.MinX);
                maxX = Mathf.Max(maxX, (float)envelope.MaxX);
                minZ = Mathf.Min(minZ, (float)envelope.MinY); // 注意：NetTopologySuite中Y坐标对应Unity的Z坐标
                maxZ = Mathf.Max(maxZ, (float)envelope.MaxY);
            }
            
            // 扩展边界
            minX -= boundaryExtension;
            maxX += boundaryExtension;
            minZ -= boundaryExtension;
            maxZ += boundaryExtension;
            
            boundaryDetected = true;
            Debug.Log($"从IndoorSpace检测到的地图边界: X({minX}, {maxX}), Z({minZ}, {maxZ})");
        }
        
        // 从场景中的对象检测边界        
        private void DetectBoundaryFromScene()
        {
            minX = float.MaxValue;
            maxX = float.MinValue;
            minZ = float.MaxValue;
            maxZ = float.MinValue;
            
            bool foundObjects = false;
            
            // 查找所有标记为GroundLayer的对象
            GameObject[] groundObjects = GameObject.FindGameObjectsWithTag("GroundLayer");
            if (groundObjects.Length == 0)
            {
                // 如果没有标签，尝试按层查找
                groundObjects = FindObjectsOfType<GameObject>()
                    .Where(obj => obj.layer == LayerMask.NameToLayer("GroundLayer"))
                    .ToArray();
            }
            
            if (groundObjects.Length > 0)
            {
                foundObjects = true;
                foreach (var obj in groundObjects)
                {
                    // 获取对象的边界
                    Renderer renderer = obj.GetComponent<Renderer>();
                    if (renderer != null)
                    {
                        Bounds bounds = renderer.bounds;
                        
                        minX = Mathf.Min(minX, bounds.min.x);
                        maxX = Mathf.Max(maxX, bounds.max.x);
                        minZ = Mathf.Min(minZ, bounds.min.z);
                        maxZ = Mathf.Max(maxZ, bounds.max.z);
                    }
                    else
                    {
                        // 如果没有Renderer，尝试使用对象的位置
                        minX = Mathf.Min(minX, obj.transform.position.x - 0.5f);
                        maxX = Mathf.Max(maxX, obj.transform.position.x + 0.5f);
                        minZ = Mathf.Min(minZ, obj.transform.position.z - 0.5f);
                        maxZ = Mathf.Max(maxZ, obj.transform.position.z + 0.5f);
                    }
                }
            }
            
            // 如果没有找到任何地面对象，尝试查找所有"Floor_"开头的对象
            if (!foundObjects)
            {
                var floorContainers = FindObjectsOfType<Transform>()
                    .Where(t => t.name.StartsWith("Floor_"))
                    .ToArray();
                
                if (floorContainers.Length > 0)
                {
                    foundObjects = true;
                    foreach (var container in floorContainers)
                    {
                        // 遍历容器中的所有子对象
                        foreach (Transform child in container)
                        {
                            Renderer renderer = child.GetComponent<Renderer>();
                            if (renderer != null)
                            {
                                Bounds bounds = renderer.bounds;
                                
                                minX = Mathf.Min(minX, bounds.min.x);
                                maxX = Mathf.Max(maxX, bounds.max.x);
                                minZ = Mathf.Min(minZ, bounds.min.z);
                                maxZ = Mathf.Max(maxZ, bounds.max.z);
                            }
                        }
                    }
                }
            }
            
            if (foundObjects)
            {
                minX -= boundaryExtension;
                maxX += boundaryExtension;
                minZ -= boundaryExtension;
                maxZ += boundaryExtension;
                
                boundaryDetected = true;
            }
        }

        private void CreateBoundaryWalls()
        {
            // 获取地图的宽度和长度
            float mapWidth = maxX - minX;
            float mapLength = maxZ - minZ;
            
            // 创建墙体的父对象
            GameObject wallsParent = new GameObject("BoundaryWalls");
            wallsParent.transform.SetParent(transform);
            
            // 将墙父对象也设置为BoundaryWall层
            wallsParent.layer = LayerMask.NameToLayer("BoundaryWall");
            
            // 创建透明墙体材质
            Material transparentWallMaterial = CreateTransparentMaterial();
            
            // 创建北墙 (Z最大)
            CreateWall(
                new Vector3(minX + mapWidth / 2, wallHeight / 2, maxZ),
                new Vector3(mapWidth + wallThickness, wallHeight, wallThickness),
                "NorthWall",
                wallsParent.transform,
                transparentWallMaterial
            );
            
            // 创建南墙 (Z最小)
            CreateWall(
                new Vector3(minX + mapWidth / 2, wallHeight / 2, minZ),
                new Vector3(mapWidth + wallThickness, wallHeight, wallThickness),
                "SouthWall",
                wallsParent.transform,
                transparentWallMaterial
            );
            
            // 创建西墙 (X最小)
            CreateWall(
                new Vector3(minX, wallHeight / 2, minZ + mapLength / 2),
                new Vector3(wallThickness, wallHeight, mapLength + wallThickness),
                "WestWall",
                wallsParent.transform,
                transparentWallMaterial
            );
            
            // 创建东墙 (X最大)
            CreateWall(
                new Vector3(maxX, wallHeight / 2, minZ + mapLength / 2),
                new Vector3(wallThickness, wallHeight, mapLength + wallThickness),
                "EastWall",
                wallsParent.transform,
                transparentWallMaterial
            );
        }

        // 创建透明材质
        private Material CreateTransparentMaterial()
        {
            // 如果提供了自定义材质，使用它作为基础
            Material material;
            if (boundaryMaterial != null)
            {
                material = new Material(boundaryMaterial);
            }
            else
            {
                material = new Material(Shader.Find("Standard"));
                material.color = new Color(wallColor.r, wallColor.g, wallColor.b, wallTransparency);
            }
            
            // 设置为透明模式
            material.SetFloat("_Mode", 3); // 3 = Transparent
            material.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            material.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            material.SetInt("_ZWrite", 0);
            material.DisableKeyword("_ALPHATEST_ON");
            material.EnableKeyword("_ALPHABLEND_ON");
            material.DisableKeyword("_ALPHAPREMULTIPLY_ON");
            material.renderQueue = 3000;
            
            // 设置最终的Alpha值
            Color color = material.color;
            color.a = wallTransparency;
            material.color = color;
            
            return material;
        }

        private void CreateWall(Vector3 position, Vector3 scale, string name, Transform parent, Material material)
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
            renderer.material = material;
            
            Rigidbody rb = wall.AddComponent<Rigidbody>();
            rb.isKinematic = true;
            rb.useGravity = false;
        }

        private void CreateFloorBoundary(Transform parent, Material material)
        {
            float mapWidth = maxX - minX;
            float mapLength = maxZ - minZ;
            
            GameObject floorBoundary = new GameObject("FloorBoundary");
            floorBoundary.transform.SetParent(parent);
            floorBoundary.transform.position = new Vector3(minX + mapWidth/2, -0.01f, minZ + mapLength/2);
            
            MeshFilter meshFilter = floorBoundary.AddComponent<MeshFilter>();
            MeshRenderer meshRenderer = floorBoundary.AddComponent<MeshRenderer>();
            
            Mesh mesh = new Mesh();
            
            Vector3[] vertices = new Vector3[]
            {
                new Vector3(-mapWidth/2, 0, -mapLength/2),
                new Vector3(mapWidth/2, 0, -mapLength/2),
                new Vector3(mapWidth/2, 0, mapLength/2),
                new Vector3(-mapWidth/2, 0, mapLength/2),
            };
            
            int[] triangles = new int[]
            {
                0, 1, 2,
                0, 2, 3
            };
            
            Vector2[] uv = new Vector2[]
            {
                new Vector2(0, 0),
                new Vector2(1, 0),
                new Vector2(1, 1),
                new Vector2(0, 1)
            };
            
            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.uv = uv;
            mesh.RecalculateNormals();
            
            meshFilter.mesh = mesh;
            meshRenderer.material = material;
            
            MeshCollider meshCollider = floorBoundary.AddComponent<MeshCollider>();
            meshCollider.sharedMesh = mesh;
            
            floorBoundary.layer = LayerMask.NameToLayer("GroundLayer");
        }
        
        public void SetWallTransparency(float transparency)
        {
            wallTransparency = Mathf.Clamp01(transparency);
            
            Transform wallsParent = transform.Find("BoundaryWalls");
            if (wallsParent != null)
            {
                foreach (Transform wall in wallsParent)
                {
                    Renderer renderer = wall.GetComponent<Renderer>();
                    if (renderer != null)
                    {
                        Color color = renderer.material.color;
                        color.a = wallTransparency;
                        renderer.material.color = color;
                    }
                }
            }
        }
    }
}