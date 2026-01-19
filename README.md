# WarehouseSim

A Unity3D-based simulation platform for multi-robot systems in warehouse environments. This project provides a comprehensive framework for simulating, testing, and visualizing autonomous guided vehicles (AGVs) performing order fulfillment tasks in indoor warehouse settings.

## Overview

WarehouseSim enables researchers and developers to:
- Simulate multi-floor warehouse environments with realistic spatial layouts
- Deploy and coordinate multiple robots for order picking and delivery
- Test and compare different path planning algorithms
- Analyze system performance under various operational scenarios

## System Architecture

The system is organized into five core modules:

### 1. Map System

The map system models the warehouse environment using concepts inspired by IndoorGML:

- **Indoor Space Model**: Represents the warehouse as a collection of navigable cells (aisles, picking points, packing areas) connected by boundaries
- **Multi-Floor Support**: Full support for multi-level warehouses with elevators and staircases as vertical connections
- **Dynamic Occupancy Layer**: Tracks robot positions over time using a time-space representation, enabling conflict-free path planning
- **Spatial Indexing**: Efficient spatial queries for collision detection and path validation

### 2. Robot Management System

Handles the lifecycle and coordination of all robots in the simulation:

- **Robot Controller**: Manages individual robot behavior including movement, order handling, and state transitions
- **Robot Manager**: Coordinates the robot fleet, handles spawning, and monitors overall system state
- **Collision Handling**: Physics-based collision detection with emergency stop and re-planning capabilities
- **Multi-Floor Navigation**: Supports robot movement across floors via elevators and stairs

### 3. Order Management System

Manages the generation, assignment, and tracking of customer orders:

- **Order Generation**: Configurable order generation with support for multi-floor scenarios
- **Order Assignment**: Intelligent assignment of orders to available robots based on proximity and workload
- **Queue Management**: Handles order backlog when all robots are busy

### 4. Picker Management System

Coordinates human pickers who work alongside robots:

- **Picker Controller**: Manages picker movement and task execution at picking stations
- **Robot-Picker Coordination**: Synchronizes robot arrivals with picker availability
- **Model Synchronization**: Keeps picker states consistent across the simulation

### 5. Path Planning System

Provides multiple algorithms for computing collision-free paths:

- **Time-Space Path Planning**: Plans paths that avoid conflicts both in space and time
- **Traffic Rules**: Enforces speed limits in different areas (intersections, picking points) and handles turning speed adjustments
- **Conflict Resolution**: Automatically resolves path conflicts through time-shifting or speed adjustment strategies
- **Multi-Floor Routing**: Finds optimal routes across floors considering elevator/stair availability

## Technology Stack

| Component | Technology |
|-----------|------------|
| Game Engine | Unity3D |
| Geometry Library | NetTopologySuite |
| Spatial Index | RBush |
| Serialization | Newtonsoft.Json |
| Navigation | Unity NavMesh |

## Getting Started

1. Open the project in Unity (2021.3 LTS or later recommended)
2. Load the main simulation scene
3. Configure your warehouse map via JSON file
4. Adjust simulation parameters in the Inspector
5. Press Play to run the simulation

## Project Structure

```
Assets/
├── Script/
│   ├── Central Control/     # Core coordination logic
│   │   ├── Robot/           # Robot management
│   │   ├── Order/           # Order management
│   │   └── Picker/          # Picker management
│   ├── Map/                  # Map system
│   │   ├── Schema/          # Data structures (IndoorSpace, Graph, etc.)
│   │   └── DynamicLayer/    # Time-space occupancy tracking
│   └── PathPlanning/        # Path planning algorithms
├── Prefabs/                  # Reusable game objects
├── Materials/                # Visual materials
└── Scenes/                   # Unity scenes
```

## License

This project is for research and educational purposes.

## Acknowledgments

This simulation platform was developed for research in multi-robot coordination and warehouse automation.
