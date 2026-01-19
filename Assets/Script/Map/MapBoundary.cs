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
                Debug.LogError("Unable to detect map boundary. Ensure there are map objects in the scene or set boundaries manually.");
            }
        }
        
        // Detect boundary from IndoorSpace
        private void DetectBoundaryFromIndoorSpace(IndoorSpace indoorSpace)
        {
            minX = float.MaxValue;
            maxX = float.MinValue;
            minZ = float.MaxValue;
            maxZ = float.MinValue;
            
            // Traverse all CellSpaces to find min/max x and z coordinates
            foreach (var cellSpace in indoorSpace.CellSpaces)
            {
                var envelope = cellSpace.Space.EnvelopeInternal;
                
                minX = Mathf.Min(minX, (float)envelope.MinX);
                maxX = Mathf.Max(maxX, (float)envelope.MaxX);
                minZ = Mathf.Min(minZ, (float)envelope.MinY); // Note: Y coordinate in NetTopologySuite corresponds to Z in Unity
                maxZ = Mathf.Max(maxZ, (float)envelope.MaxY);
            }
            
            // Extend boundary
            minX -= boundaryExtension;
            maxX += boundaryExtension;
            minZ -= boundaryExtension;
            maxZ += boundaryExtension;
            
            boundaryDetected = true;
            Debug.Log($"Map boundary detected from IndoorSpace: X({minX}, {maxX}), Z({minZ}, {maxZ})");
        }
        
        // Detect boundary from objects in the scene        
        private void DetectBoundaryFromScene()
        {
            minX = float.MaxValue;
            maxX = float.MinValue;
            minZ = float.MaxValue;
            maxZ = float.MinValue;
            
            bool foundObjects = false;
            
            // Find all objects tagged as GroundLayer
            GameObject[] groundObjects = GameObject.FindGameObjectsWithTag("GroundLayer");
            if (groundObjects.Length == 0)
            {
                // If no tag, try searching by layer
                groundObjects = FindObjectsOfType<GameObject>()
                    .Where(obj => obj.layer == LayerMask.NameToLayer("GroundLayer"))
                    .ToArray();
            }
            
            if (groundObjects.Length > 0)
            {
                foundObjects = true;
                foreach (var obj in groundObjects)
                {
                    // Get object bounds
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
                        // If no Renderer, try using object position
                        minX = Mathf.Min(minX, obj.transform.position.x - 0.5f);
                        maxX = Mathf.Max(maxX, obj.transform.position.x + 0.5f);
                        minZ = Mathf.Min(minZ, obj.transform.position.z - 0.5f);
                        maxZ = Mathf.Max(maxZ, obj.transform.position.z + 0.5f);
                    }
                }
            }
            
            // If no ground objects found, try searching for objects starting with "Floor_"
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
                        // Traverse all child objects in the container
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
            // Get map width and length
            float mapWidth = maxX - minX;
            float mapLength = maxZ - minZ;
            
            // Create parent object for walls
            GameObject wallsParent = new GameObject("BoundaryWalls");
            wallsParent.transform.SetParent(transform);
            
            // Also set walls parent to BoundaryWall layer
            wallsParent.layer = LayerMask.NameToLayer("BoundaryWall");
            
            // Create transparent wall material
            Material transparentWallMaterial = CreateTransparentMaterial();
            
            // Create North wall (max Z)
            CreateWall(
                new Vector3(minX + mapWidth / 2, wallHeight / 2, maxZ),
                new Vector3(mapWidth + wallThickness, wallHeight, wallThickness),
                "NorthWall",
                wallsParent.transform,
                transparentWallMaterial
            );
            
            // Create South wall (min Z)
            CreateWall(
                new Vector3(minX + mapWidth / 2, wallHeight / 2, minZ),
                new Vector3(mapWidth + wallThickness, wallHeight, wallThickness),
                "SouthWall",
                wallsParent.transform,
                transparentWallMaterial
            );
            
            // Create West wall (min X)
            CreateWall(
                new Vector3(minX, wallHeight / 2, minZ + mapLength / 2),
                new Vector3(wallThickness, wallHeight, mapLength + wallThickness),
                "WestWall",
                wallsParent.transform,
                transparentWallMaterial
            );
            
            // Create East wall (max X)
            CreateWall(
                new Vector3(maxX, wallHeight / 2, minZ + mapLength / 2),
                new Vector3(wallThickness, wallHeight, mapLength + wallThickness),
                "EastWall",
                wallsParent.transform,
                transparentWallMaterial
            );
        }


        // Create transparent material
        private Material CreateTransparentMaterial()
        {
            // If custom material provided, use it as base
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
            
            // Set to transparent mode
            material.SetFloat("_Mode", 3); // 3 = Transparent
            material.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            material.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            material.SetInt("_ZWrite", 0);
            material.DisableKeyword("_ALPHATEST_ON");
            material.EnableKeyword("_ALPHABLEND_ON");
            material.DisableKeyword("_ALPHAPREMULTIPLY_ON");
            material.renderQueue = 3000;
            
            // Set final alpha value
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

            // Set layer
            wall.layer = LayerMask.NameToLayer("BoundaryWall");

            // Set material
            Renderer renderer = wall.GetComponent<Renderer>();
            renderer.material = material;
            
            Rigidbody rb = wall.AddComponent<Rigidbody>();
            rb.isKinematic = true;
            rb.useGravity = false;
        }

        // Create floor boundary
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