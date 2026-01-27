# WarehouseSim

A Unity3D-based simulation platform for multi-robot systems in warehouse environments. This project provides a comprehensive framework for simulating, testing, and visualizing autonomous guided vehicles (AGVs) performing order fulfillment tasks in indoor warehouse settings.

## Overview

WarehouseSim enables researchers and developers to:
- Model warehouse environments using an **IndoorGML 2.0-inspired spatial data model**
- Simulate multi-floor warehouse operations with time-space path planning
- Deploy and coordinate multiple robots with collision avoidance and deadlock detection
- Test robot-picker collaboration scenarios at picking stations
- Analyze system performance with comprehensive metrics collection

The core of this platform is a **robust spatial data model** that represents indoor spaces as navigable and non-navigable cells with connectivity, supporting complex multi-floor coordination scenarios.

## System Architecture

The system is organized into five core modules:

### 1. Map System (Data Model Core)

The map system models warehouse environments using a spatial data model inspired by **IndoorGML 2.0**:

- **Indoor Space Model**: Represents warehouses as a collection of CellSpaces (spatial units) connected by CellBoundaries, with geometric and semantic properties
- **Navigation Graph**: ConnectionPoints and RoutePoints form a directed graph for path planning
- **Multi-Floor Support**: Native support for multi-level warehouses with vertical connections (elevators, stairs)
- **Dynamic Occupancy Layer**: Time-space representation tracking robot occupancy over time, enabling conflict-free path planning
- **Spatial Indexing**: R-tree and interval tree structures for efficient spatial and temporal queries

### 2. Path Planning System

Provides multiple algorithms for computing collision-free paths in the time-space domain:

- **AnytimeSIPP**: Safe Interval Path Planning algorithm that plans around reserved time intervals
- **Time-Space A***: Alternative planner with explicit time dimension consideration
- **Traffic Rules**: Speed constraints based on cell types (intersections, picking points) and movement patterns
- **Conflict Resolution**: Automatic resolution through time-shifting or speed adjustment strategies
- **Multi-Floor Routing**: Optimal routes across floors considering vertical connection availability

### 3. Robot Management System

Handles the lifecycle and coordination of the robot fleet:

- **Robot Controller**: Individual robot behavior with state machine (Idle, Moving, Queuing, AdvancingInQueue, PickingPending, Picking, EmergencyStopped)
- **Robot Manager**: Fleet coordination, spawning, and system-wide state monitoring
- **Collision Recovery**: Physics-based detection with emergency stop, collision history tracking, and automatic replanning
- **Personal Occupancy Layer**: Each robot maintains reservations merged into the global layer

### 4. Order Management System

Manages order generation, assignment, and tracking:

- **Order Generation**: Configurable order generation with multi-floor support
- **Order Assignment**: Intelligent weighted assignment based on distance, robot state, and spatial clustering
- **Queue Management**: Handles order backlog when robots are busy

### 5. Picker Coordination System

Coordinates human pickers working alongside robots:

- **Picker Controller**: Manages picker movement and task execution at picking stations
- **Picking Queue Manager**: FIFO queue management at each picking point with automatic advancement
- **Robot-Picker Synchronization**: Coordinates robot arrivals with picker availability

### Supporting Systems

- **Event Manager**: Centralized event bus for decoupled inter-component communication
- **Deadlock Detector**: Monitors and resolves circular waiting conditions
- **Metrics Collector**: Tracks performance metrics and generates CSV reports

## Technology Stack

| Component | Technology |
|-----------|------------|
| Game Engine | Unity 2021.3 LTS |
| Geometry Library | NetTopologySuite |
| Spatial Index | RBush |
| Temporal Index | C5 Collections |
| Serialization | Newtonsoft.Json |
| Navigation | Unity NavMesh |

## Getting Started

### Prerequisites
- Unity 2021.3 LTS or later
- NuGet for Unity package manager

### Quick Start

1. **Open the project** in Unity Hub
2. **Load the simulation scene**: `Assets/Scenes/warehouse_with_collision.unity`
3. **Review map configuration**: Map JSON files in `Assets/Script/Map/` define the warehouse layout (CellSpaces, CellBoundaries, floor levels)
4. **Configure simulation parameters** in `CentralController` Inspector:
   - `robotCount` - Number of robots in the fleet
   - `orderGenerationInterval` - Time between order spawns
   - `selectedAlgorithm` - Path planning algorithm (AnytimeSIPP / TimeSpaceAStar)
   - `enableCollisionDetection` - Enable/disable collision system
5. **Press Play** to run the simulation

### Creating Custom Maps

Maps are defined in JSON format following the IndoorGML 2.0-inspired structure:
- Define CellSpaces with geometric boundaries and semantic properties
- Specify CellBoundaries for connectivity
- Assign floor levels and function types (aisle, picking point, etc.)
- Place in `Assets/Script/Map/` directory

For detailed data model specifications, refer to the published journal paper.

## Project Structure

```
Assets/
├── Script/
│   ├── Map/                              # Data Model & Spatial System
│   │   ├── MapLoader.cs                 # JSON parser
│   │   ├── Schema/                      # Data model classes
│   │   │   ├── IndoorSpace.cs          # Core spatial model
│   │   │   ├── CellSpace.cs            # Spatial units
│   │   │   ├── CellBoundary.cs         # Connectivity
│   │   │   ├── ConnectionPoint.cs      # Graph nodes
│   │   │   └── Graph.cs                # Navigation graph
│   │   ├── DynamicLayer/
│   │   │   └── DynamicOccupancyLayer.cs # Time-space tracking
│   │   └── *.json                       # Map definitions
│   ├── PathPlanning/                    # Path Planning Algorithms
│   │   ├── AnytimeSIPP.cs              # SIPP algorithm
│   │   ├── Astar_Traffic_Completed.cs  # Time-space A*
│   │   ├── TrafficRulesKnowledge.cs    # Speed constraints
│   │   └── ConflictResolver.cs         # Conflict resolution
│   ├── Central Control/                 # Coordination & Management
│   │   ├── CentralController.cs        # Main orchestrator
│   │   ├── EventManager.cs             # Event bus
│   │   ├── DeadlockDetector.cs         # Deadlock detection
│   │   ├── SimulationMetricsCollector.cs # Performance metrics
│   │   ├── Robot/
│   │   │   ├── RobotController.cs      # Robot behavior
│   │   │   ├── RobotManager.cs         # Fleet management
│   │   │   └── CollisionRecoveryManager.cs # Collision handling
│   │   ├── Order/
│   │   │   ├── OrderManager.cs         # Order lifecycle
│   │   │   └── OrderAssignmentSelector.cs # Assignment strategy
│   │   └── Picker/
│   │       ├── PickerController.cs     # Picker behavior
│   │       └── PickingQueueManager.cs  # Queue coordination
│   └── ...
├── Prefabs/                             # Reusable game objects
├── Materials/                           # Visual materials
└── Scenes/                              # Unity scenes
```

## Simulation Metrics

The system automatically collects and exports performance metrics to `Application.persistentDataPath`:

| Metric File | Content |
|-------------|---------|
| `collision_report_*.csv` | Collision events, timestamps, robot pairs, locations |
| `path_planning_metrics_*.csv` | Planning time, path length, replans, success rates |

Reports are auto-saved every 60 seconds and on application quit.

## Key Features

- **IndoorGML 2.0-Inspired Data Model**: Robust spatial representation with CellSpaces and CellBoundaries
- **Time-Space Path Planning**: Proactive conflict avoidance through temporal reservations
- **Optimistic Concurrency**: Multiple robots plan paths simultaneously with version control
- **Deadlock Detection & Resolution**: Automatic detection and breaking of circular waits
- **Queue Coordination**: FIFO queues at picking points with automatic advancement
- **Collision Recovery**: Emergency stop, history tracking, and intelligent replanning
- **Multi-Floor Support**: Native support for elevators and stairs in path planning
- **Comprehensive Metrics**: Automated performance data collection and CSV export

## Version History

- **v2.0** - Stability improvements, queue system, deadlock detection, comprehensive metrics collection
- **v1.0** - Initial IndoorGML 2.0-inspired data model and multi-robot coordination

## License

This project is for research and educational purposes.

## Acknowledgments

This simulation platform was developed for research in multi-robot coordination, indoor spatial modeling, and warehouse automation. The spatial data model design is inspired by IndoorGML 2.0.

For detailed information about the data model and algorithms, please refer to the associated journal publication.
