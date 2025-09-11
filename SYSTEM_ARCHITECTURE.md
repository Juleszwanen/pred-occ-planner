# System Architecture Diagram

## Planning Pipeline Flow for sim_fkpcp_4_case_4.launch

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                            SIMULATION ENVIRONMENT                               │
│  ┌─────────────────────────────────────────────────────────────────────────────┤
│  │  Map Generator (dynamic_forest_seq)                                        │
│  │  • Creates 16x16x7m environment with 20 dynamic obstacles                  │
│  │  • Publishes /map_generator/global_cloud (10 Hz)                          │
│  │  • Publishes /ground_truth_state                                          │
│  └─────────────────────────────────────────────────────────────────────────────┤
└─────────────────────────────────────────────────────────────────────────────────┘
                                        │
                                        ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│                           INDIVIDUAL DRONE STACKS (x4)                         │
│                                                                                 │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐              │
│  │    UAV0     │  │    UAV1     │  │    UAV2     │  │    UAV3     │              │
│  │(-8,0,1)→    │  │(8,0,1)→     │  │(0,8,1)→     │  │(0,-8,1)→    │              │
│  │  (8,0,1)    │  │ (-8,0,1)    │  │ (0,-8,1)    │  │  (0,8,1)    │              │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘              │
│         │                 │                 │                 │                 │
│  ┌──────▼──────────────────▼──────────────────▼──────────────────▼──────────┐    │
│  │                    PER-DRONE PLANNING STACK                              │    │
│  │                                                                          │    │
│  │  ┌─────────────────────────────────────────────────────────────────┐     │    │
│  │  │  Main Planner (fake_baseline)                                   │     │    │
│  │  │  ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐    │     │    │
│  │  │  │ SOGM Builder    │ │ Hybrid A* Search│ │ Trajectory Opt  │    │     │    │
│  │  │  │ (4D occupancy)  │ │ (risk-aware)    │ │ (Bezier curves) │    │     │    │
│  │  │  └─────────────────┘ └─────────────────┘ └─────────────────┘    │     │    │
│  │  └─────────────────────────────────────────────────────────────────┘     │    │
│  │  ┌─────────────────────────────────────────────────────────────────┐     │    │
│  │  │  Trajectory Server (bezier_traj_server)                         │     │    │
│  │  │  • Converts trajectories to position commands                   │     │    │
│  │  │  • Handles timing and execution                                 │     │    │
│  │  └─────────────────────────────────────────────────────────────────┘     │    │
│  │  ┌─────────────────────────────────────────────────────────────────┐     │    │
│  │  │  Dynamics Simulator (poscmd_2_odom)                             │     │    │
│  │  │  • Simulates drone response to commands                         │     │    │
│  │  │  • Publishes pose and odometry                                  │     │    │
│  │  └─────────────────────────────────────────────────────────────────┘     │    │
│  └──────────────────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────────────────┘
                                        │
                                        ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│                        MULTI-AGENT COORDINATION                                │
│  ┌─────────────────────────────────────────────────────────────────────────────┤
│  │  Trajectory Broadcasting (/broadcast_traj)                                 │
│  │  • Each drone shares its planned trajectory                                │
│  │  • Other drones incorporate shared trajectories as dynamic obstacles      │
│  │  • Collision detection triggers replanning                                │
│  │  • Priority-based conflict resolution                                     │
│  └─────────────────────────────────────────────────────────────────────────────┤
└─────────────────────────────────────────────────────────────────────────────────┘
                                        │
                                        ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│                           VISUALIZATION & MONITORING                           │
│  ┌─────────────────────────────────────────────────────────────────────────────┤
│  │  RViz Visualization                                                        │
│  │  • 3D environment view with obstacles                                     │
│  │  • Drone models and trajectories                                          │
│  │  • Spatiotemporal occupancy grids                                         │
│  │  • 2D Nav Goal trigger for planning                                       │
│  └─────────────────────────────────────────────────────────────────────────────┤
└─────────────────────────────────────────────────────────────────────────────────┘
```

## Data Flow Diagram

```
Environment Data:
Map Generator → Point Cloud → [Drone 0,1,2,3] → SOGM → Path Planning

Coordination Data:
Drone 0 Trajectory ──┐
Drone 1 Trajectory ──┼─→ /broadcast_traj ─→ All Drones → Collision Check
Drone 2 Trajectory ──┤
Drone 3 Trajectory ──┘

Execution Data:
Plan → Trajectory Server → Position Commands → Dynamics → Pose/Odom → Replanning Loop
```

## Key Interactions

### 1. Environment to Planning
- **Map Generator** continuously publishes obstacle point cloud
- Each **drone planner** subscribes to global point cloud
- **SOGM** processes point cloud into 4D occupancy grid
- **Dynamic obstacles** are tracked and predicted

### 2. Planning to Coordination  
- Each drone plans independently using **Hybrid A***
- Planned trajectories are broadcast via **ROS topics**
- Other drones receive and integrate trajectories as **dynamic obstacles**
- **Collision detection** triggers replanning when needed

### 3. Planning to Execution
- **Trajectory optimization** produces smooth Bezier curves
- **Trajectory server** converts to position commands
- **Dynamics simulator** executes commands and updates pose
- **Continuous monitoring** checks for deviations

### 4. User Interaction
- **RViz 2D Nav Goal** triggers the planning process
- **Visualization** shows real-time status of all components
- **Parameter files** allow configuration without code changes

## Critical Components Communication

```
┌─────────────┐    /map_generator/global_cloud    ┌─────────────┐
│Map Generator├─────────────────────────────────►│Drone Planner│
└─────────────┘                                  └─────────────┘
                                                        │
     ┌─────────────┐      /broadcast_traj              │
     │Other Drones │◄────────────────────────────────┬─┘
     └─────────────┘                                 │
                                                     │ /planner/trajectory
     ┌─────────────┐    /controller/pos_cmd          ▼
     │Dynamics Sim │◄─────────────────────────┌─────────────┐
     └─────────────┘                          │Traj Server  │
            │                                 └─────────────┘
            │ /mavros/local_position/pose
            ▼
     ┌─────────────┐
     │Visualization│
     └─────────────┘
```