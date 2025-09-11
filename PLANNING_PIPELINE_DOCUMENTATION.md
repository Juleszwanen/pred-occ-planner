# Planning Pipeline Documentation

## Overview

This document explains how the planning pipeline works in the pred-occ-planner repository when launching the simulation with:

```bash
roslaunch plan_manager sim_fkpcp_4_case_4.launch
```

## System Architecture

The planning system implements a **decentralized multi-agent trajectory planning framework** using **Spatiotemporal Occupancy Grid Maps (SOGM)** for collision avoidance in dynamic environments.

### High-Level Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Environment   │    │   Individual    │    │  Visualization  │
│   Simulation    │◄──►│  Drone Planner  │◄──►│   & Control     │
│                 │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         │              ┌─────────────────┐              │
         └─────────────►│ Multi-Agent     │◄─────────────┘
                        │ Coordination    │
                        └─────────────────┘
```

## Launch File Analysis: sim_fkpcp_4_case_4.launch

### Configuration Parameters
- **Map Size**: 16×16×7 meters
- **Number of Robots**: 4 UAVs
- **Obstacles**: 20 dynamic obstacles
- **Map Mode**: 0 (pure dynamic obstacles)

### Drone Initial Positions and Goals

| Drone | Start Position | Goal Position | Orientation |
|-------|----------------|---------------|-------------|
| UAV0  | (-8, 0, 1)     | (8, 0, 1)     | Default     |
| UAV1  | (8, 0, 1)      | (-8, 0, 1)    | 180° turn   |
| UAV2  | (0, 8, 1)      | (0, -8, 1)    | -90° turn   |
| UAV3  | (0, -8, 1)     | (0, 8, 1)     | 90° turn    |

This creates a **crossing scenario** where drones must navigate through each other's paths.

## Component Breakdown

### 1. Environment Simulation (`simulator_fake.launch`)

**Purpose**: Creates the dynamic environment with moving obstacles

**Key Components**:
- **Map Generator Node** (`map_generator/dynamic_forest_seq`)
  - Generates dynamic cylindrical obstacles
  - Publishes global point cloud (`/map_generator/global_cloud`)
  - Publishes ground truth state (`/ground_truth_state`)
  - Obstacle properties:
    - Radius: 0.7-2.5m
    - Height: 0.8-2.0m
    - Velocity: up to 0.1 m/s
    - Update rate: 10 Hz

### 2. Individual Drone Planning Stack (`vis_drone_fake_perception.xml`)

Each drone runs an independent planning stack consisting of:

#### A. Main Planner Node (`fake_baseline`)
**Executable**: `plan_manager/fake_baseline`
**Configuration**: `config/sim_fake.yaml`

**Core Functionality**:
- **Spatiotemporal Occupancy Grid Mapping**: Creates 4D occupancy maps (x,y,z,time)
- **Risk-aware Path Planning**: Uses Hybrid A* with risk assessment
- **Multi-agent Coordination**: Shares trajectories to avoid inter-drone collisions

**Key Algorithms**:
1. **FakeParticleRiskVoxel**: Implements spatiotemporal occupancy mapping
2. **FakeRiskHybridAstar**: Risk-aware path search algorithm
3. **BezierOpt**: Trajectory optimization for smooth paths
4. **ParticleATC**: Air traffic control for multi-agent coordination

#### B. Trajectory Server (`bezier_traj_server`)
**Purpose**: Converts planned trajectories into executable commands

**Functions**:
- Receives Bezier trajectories from planner
- Generates position commands for flight controller
- Handles trajectory timing and execution
- Provides replan triggers

#### C. Command Conversion (`poscmd_2_odom`)
**Purpose**: Simulates drone dynamics

**Functions**:
- Converts position commands to odometry
- Simulates drone response to commands
- Publishes pose and odometry topics

#### D. Visualization Components
- **Trajectory Colorization**: Visualizes trajectory with velocity colors
- **Drone Visualization**: 3D drone model in RViz
- **Odometry Visualization**: Shows drone pose and trajectory

### 3. Multi-Agent Coordination

**Communication Mechanism**:
- Drones broadcast their planned trajectories via `/broadcast_traj` topic
- Each drone receives other drones' trajectories
- Trajectories are projected onto the spatiotemporal occupancy grid
- Collision avoidance is achieved through trajectory deconfliction

**Coordination Algorithm**:
1. Each drone plans independently using SOGM
2. Planned trajectories are shared with other drones
3. Other drones' trajectories are treated as dynamic obstacles
4. If collision risk is detected, replanning is triggered
5. Priority-based resolution ensures deadlock avoidance

## Data Flow Diagram

```
Environment Data Flow:
Map Generator → Global Point Cloud → Individual Drone Planners

Individual Drone Planning Flow:
Local Sensors → Occupancy Grid → Path Planning → Trajectory Optimization → Execution

Multi-Agent Coordination Flow:
Drone A Trajectory → Broadcast → Drone B Planner → Collision Check → Replan if needed
```

## Planning Algorithm Details

### Spatiotemporal Occupancy Grid Mapping (SOGM)

**Purpose**: Predicts future occupancy of space considering dynamic obstacles

**Key Features**:
- **4D Representation**: (x, y, z, time)
- **Particle Filter**: Tracks moving obstacles
- **Risk Assessment**: Probabilistic occupancy prediction
- **Multi-resolution**: Different resolutions for space and time

**Parameters** (from `sim_fake.yaml`):
- Spatial resolution: 0.15m
- Time resolution: 0.2s
- Prediction horizon: 3 time steps
- Risk threshold: 0.5

### Hybrid A* Search

**Purpose**: Finds kinodynamically feasible paths in SOGM

**Features**:
- **Kinodynamic Constraints**: Respects velocity/acceleration limits
- **Risk-aware Cost**: Incorporates collision probability
- **Time-optimal**: Minimizes trajectory duration
- **Heuristic Guidance**: Uses 3D Euclidean distance

**Parameters**:
- Max velocity: 2 m/s
- Max acceleration: 6 m/s²
- Safety margin: 0.6m
- Search resolution: 0.15m

### Trajectory Optimization

**Purpose**: Smooths path and ensures dynamic feasibility

**Method**: Bezier curve optimization with constraints
- **Smoothness**: Minimizes jerk and acceleration
- **Collision Avoidance**: Maintains safe distance from obstacles
- **Dynamic Limits**: Enforces velocity/acceleration constraints

## Execution Workflow

### 1. Initialization Phase
1. Launch environment simulator with dynamic obstacles
2. Initialize 4 drone planning stacks with different start/goal positions
3. Start RViz for visualization
4. Each drone initializes its SOGM and planner

### 2. Planning Phase (Triggered by user)
1. User clicks "2D Nav Goal" in RViz to trigger planning
2. Each drone simultaneously:
   - Updates its local SOGM with environment data
   - Incorporates other drones' broadcasted trajectories
   - Runs Hybrid A* search to find initial path
   - Optimizes path using Bezier trajectory optimization
   - Broadcasts its planned trajectory

### 3. Execution Phase
1. Trajectory server converts plans to position commands
2. Simulated dynamics execute the commands
3. Continuous monitoring for collision risks
4. Replanning triggered if:
   - Obstacles move into planned path
   - Other drones create collision risk
   - Execution deviates from plan

### 4. Coordination Loop
- Continuous trajectory broadcasting
- Real-time collision checking
- Automatic replanning when needed
- Priority-based conflict resolution

## Key Configuration Parameters

From `config/sim_fake.yaml`:

### Planning Parameters
- **Goal tolerance**: 1.0m
- **Replan tolerance**: 1.0m
- **Replan duration**: 0.1s
- **Max planning horizon**: 5.0s

### Safety Parameters
- **Clearance**: 0.45m (for collision checking)
- **Safety margin**: 0.6m (for planning)
- **Drone size**: 0.4×0.4×0.45m

### Multi-Agent Parameters
- **Number of robots**: 4
- **Replan risk rate**: 0.0 (immediate replanning on risk)

## Visualization and Monitoring

### RViz Display Elements
- **Global Map**: Shows environment obstacles
- **Drone Models**: 3D visualization of each drone
- **Planned Trajectories**: Color-coded by velocity
- **Occupancy Grid**: Visualizes spatiotemporal occupancy
- **Goal Markers**: Shows target positions

### Topics for Monitoring
- `/map_generator/global_cloud`: Environment point cloud
- `/uavX/planner/trajectory`: Individual planned trajectories
- `/uavX/mavros/local_position/pose`: Drone poses
- `/broadcast_traj`: Shared trajectory information

## Conclusion

The sim_fkpcp_4_case_4.launch creates a challenging 4-drone crossing scenario that demonstrates the effectiveness of the spatiotemporal occupancy grid-based planning approach. The system successfully handles:

1. **Dynamic Environment**: Moving obstacles with uncertain motion
2. **Multi-Agent Coordination**: Decentralized collision avoidance
3. **Real-time Planning**: Continuous replanning and adaptation
4. **Safety Guarantees**: Probabilistic collision avoidance

This makes it an excellent testbed for evaluating multi-agent planning algorithms in complex dynamic environments.