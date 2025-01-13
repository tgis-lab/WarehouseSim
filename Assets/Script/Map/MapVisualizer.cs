using UnityEngine;
using System;
using System.Collections.Generic;
using NetTopologySuite.Geometries;
using NetTopologySuite.IO;
using Map;

public class MapVisualizer : MonoBehaviour
{
    public Material navigablePolygonMaterial;
    public Material nonNavigablePolygonMaterial;
    public Material pickingPointPolygonMaterial;
    public GameObject shelfPrefab;
    public GameObject planePrefab;

    private GameObject currentStartPointObject;
    private GameObject currentEndPointObject;
    private List<GameObject> pathInstances = new List<GameObject>();

    public void VisualizeMap(IndoorSpace indoorSpace)
    {
        foreach (var cellSpace in indoorSpace.CellSpaces)
        {
            CreateSpaceVisualization(cellSpace.Space as Polygon, cellSpace.IsNavigable(), cellSpace.IsPickingPoint());
        }
    }

    public void CreateSpaceVisualization(Polygon polygon, bool isNavigable, bool isPickingPoint)
    {
        GameObject polygonObject = Instantiate(planePrefab, Vector3.zero, Quaternion.identity, transform);
        polygonObject.name = "Ground";
        polygonObject.layer = LayerMask.NameToLayer("GroundLayer"); 
        polygonObject.transform.position = new Vector3((float)polygon.Centroid.X, 0, (float)polygon.Centroid.Y);
        polygonObject.transform.localScale = new Vector3(0.1f, 1f, 0.1f);

        MeshRenderer renderer = polygonObject.GetComponent<MeshRenderer>();
        if (isPickingPoint)
        {
            renderer.material = pickingPointPolygonMaterial;
        }
        else
        {
            renderer.material = isNavigable ? navigablePolygonMaterial : nonNavigablePolygonMaterial;
        }
        
        if (!isNavigable)
        {
            GameObject shelfObject = Instantiate(shelfPrefab, Vector3.zero, Quaternion.identity, transform);
            shelfObject.name = "Shelf";
            shelfObject.layer = LayerMask.NameToLayer("CollisionLayer");
            shelfObject.transform.position = new Vector3((float)polygon.Centroid.X, 0f, (float)polygon.Centroid.Y);
            shelfObject.transform.localScale = new Vector3(1f, 1f, 1f);
        }
    }
}
