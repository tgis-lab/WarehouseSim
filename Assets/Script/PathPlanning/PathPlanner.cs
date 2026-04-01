using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Map;
using NetTopologySuite.Geometries;
using NetTopologySuite.IO;
using CentralControl;

namespace PathPlanning
{
    public enum SearchAlgorithm
    {
        BFS,
        Astar_Basic,
        TimeSpaceAStar,
        AnytimeSIPP,
        CellSpaceAStar  // CellSpace NRG baseline
    }

    public enum SearchMethod
    {
        Euclidean_Distance,
        Manhattan_Distance,
        NotApplicable
    }

    public class PathPlanner
    {
        private Graph graph;
        private IndoorSpace indoorSpace;
        private DynamicOccupancyLayer personalOccupancyLayer;
        private DynamicOccupancyLayer globalOccupancyLayer;
        private SearchMethod searchMethod;
        private SearchAlgorithm searchAlgorithm;
        private string robotId;
        private float maxAcceleration;
        private float maxDeceleration;
        private Map.CellSpaceGraph cellSpaceGraph;
        public RoutePoint startPoint;
        public RoutePoint endPoint;
        public float speed;

        public void SetCellSpaceGraph(Map.CellSpaceGraph graph)
        {
            this.cellSpaceGraph = graph;
        }
        
        public PathPlanner(
            Graph graph, 
            IndoorSpace indoorSpace,
            SearchAlgorithm algorithm, 
            SearchMethod method, 
            string robotId, 
            float maxAcceleration, 
            float maxDeceleration)
        {
            this.graph = graph;
            this.indoorSpace = indoorSpace;
            this.searchAlgorithm = algorithm;
            this.searchMethod = method;
            this.robotId = robotId;
            this.maxAcceleration = maxAcceleration;
            this.maxDeceleration = maxDeceleration;
        }

        public (List<ConnectionPoint> path, List<float> times, List<float> speeds, string planId, PathPlanningMetrics metrics) FindPathWithDynamicObstacles(
            DynamicOccupancyLayer personalOccupancyLayer, 
            DynamicOccupancyLayer globalOccupancyLayer,
            RoutePoint startPoint, 
            RoutePoint endPoint, 
            string robotId,
            string currentPlanId,
            float startTime, 
            float speed)
        {
            switch (this.searchAlgorithm)
            {
                case SearchAlgorithm.TimeSpaceAStar:
                    return TimeSpaceAStar.TimeSpaceAStarAlgorithm(
                        this.graph, 
                        this.indoorSpace,
                        personalOccupancyLayer,
                        globalOccupancyLayer, 
                        startPoint, 
                        endPoint, 
                        robotId, 
                        currentPlanId,
                        startTime, 
                        speed, 
                        this.maxAcceleration, 
                        this.maxDeceleration, 
                        this.searchMethod);
                
                case SearchAlgorithm.AnytimeSIPP:
                    return AnytimeSIPP.SIPPAlgorithm(
                        this.graph, 
                        this.indoorSpace,
                        personalOccupancyLayer,
                        globalOccupancyLayer, 
                        startPoint, 
                        endPoint, 
                        robotId, 
                        currentPlanId,
                        startTime, 
                        speed, 
                        this.maxAcceleration, 
                        this.maxDeceleration, 
                        this.searchMethod);
                
                case SearchAlgorithm.CellSpaceAStar:
                    if (cellSpaceGraph == null)
                    {
                        Debug.LogError("CellSpaceGraph not set for CellSpaceAStar algorithm");
                        return (null, null, null, null, null);
                    }
                    return cellSpaceGraph.FindPathForRobot(
                        startPoint, endPoint, robotId, startTime, speed);

                case SearchAlgorithm.Astar_Basic:
                    var basicResult = Astar_Basic.AstarAlgorithm(
                        this.graph, startPoint, endPoint, speed, this.searchMethod);
                    if (basicResult.path == null || basicResult.path.Count == 0)
                        return (null, null, null, null, null);
                    var basicTimes = new List<float>();
                    float t = startTime;
                    basicTimes.Add(t);
                    for (int i = 1; i < basicResult.path.Count; i++)
                    {
                        float d = (float)basicResult.path[i - 1].Point.Distance(basicResult.path[i].Point);
                        t += d / speed;
                        basicTimes.Add(t);
                    }
                    string basicPlanId = $"BasicPlan_{robotId}_{System.DateTime.Now.Ticks}";
                    var basicMetrics = new PathPlanningMetrics(robotId, basicPlanId);
                    return (basicResult.path, basicTimes, basicResult.speeds, basicPlanId, basicMetrics);

                default:
                    Debug.LogError($"Unknown search algorithm: {this.searchAlgorithm}");
                    return (null, null, null, null, null);
            }
        }
    }
}