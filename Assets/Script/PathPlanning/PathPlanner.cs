using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Map;

namespace PathPlanning
{
    public enum SearchAlgorithm
    {
        BFS, 
        Astar_Basic,
        Astar_Traffic
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
        public RoutePoint startPoint;
        public RoutePoint endPoint;
        public float speed;
        
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

        public (List<ConnectionPoint> path, List<float> times, List<float> speeds, string planId) FindPathWithDynamicObstacles(
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
                case SearchAlgorithm.Astar_Traffic:
                    return Astar_Traffic.AstarAlgorithm(
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
                // case SearchAlgorithm.Astar_Basic:
                //     return Astar_Basic.AstarAlgorithm(this.graph, startPoint, endPoint, speed, this.searchMethod);
                // case SearchAlgorithm.BFS:
                //     return BFS.BFSAlgorithm(this.graph, startPoint, endPoint, speed, this.searchMethod);
                default:
                    return (null, null, null, null);
            }
        }
    }
}
