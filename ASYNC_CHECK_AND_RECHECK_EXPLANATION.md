# Asynchronous Check-and-Recheck Method Implementation Guide

This document provides a detailed, pedagogical explanation of how Mader's asynchronous check-and-recheck method is implemented in this repository. It is designed to help you understand the code structure and replicate a similar architecture in your own project.

---

## Table of Contents

1. [Overview of the Repository and Algorithm](#1-overview-of-the-repository-and-algorithm)
2. [ROS Node and Communication Architecture](#2-ros-node-and-communication-architecture)
3. [Threading and Callback Model](#3-threading-and-callback-model)
4. [Detailed Implementation of Check-and-Recheck](#4-detailed-implementation-of-check-and-recheck)
5. [Asynchronous Inter-Agent Communication](#5-asynchronous-inter-agent-communication)
6. [Data Structures and State Tracking](#6-data-structures-and-state-tracking)
7. [Summary and How to Replicate This Structure](#7-summary-and-how-to-replicate-this-structure)

---

## 1. Overview of the Repository and Algorithm

### 1.1 High-Level Architecture

This repository implements a **decentralized multi-agent trajectory planning framework** for micro aerial vehicles (MAVs) in dynamic environments. The key innovation is the use of **Spatiotemporal Occupancy Grid Maps (SOGM)** combined with an asynchronous collision avoidance mechanism inspired by MADER.

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                           OVERALL SYSTEM ARCHITECTURE                           │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│  ┌────────────────────┐  ┌────────────────────┐  ┌────────────────────┐         │
│  │   Map Generator    │  │    UAV 0 Stack     │  │  Visualization     │         │
│  │  (Environment)     │  │   (Planning +      │  │  (RViz)            │         │
│  │                    │  │    Execution)      │  │                    │         │
│  └─────────┬──────────┘  └──────────┬─────────┘  └────────────────────┘         │
│            │                        │                                           │
│            │  /map_generator/       │  /broadcast_traj                          │
│            │   global_cloud         │  (shared trajectories)                    │
│            │                        ▼                                           │
│            │             ┌─────────────────────┐                                │
│            └────────────►│  UAV 1, 2, 3...     │                                │
│                          │  (Identical stacks) │                                │
│                          └─────────────────────┘                                │
│                                                                                 │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Where MADER's Check-and-Recheck Logic Lives

The asynchronous check-and-recheck logic is primarily implemented in these packages and classes:

| Package | Main Classes | Purpose |
|---------|-------------|---------|
| `traj_coordinator` | `MADER`, `RMADER`, `ParticleATC` | Core collision avoidance and trajectory deconfliction |
| `plan_manager` | `FiniteStateMachineFake`, `FakeBaselinePlanner` | Finite state machine and planning orchestration |
| `plan_env` | `FakeParticleRiskVoxel`, `MapBase` | Environment representation with multi-agent awareness |

### 1.3 Algorithm Summary

The algorithm follows this high-level flow:

1. **Plan**: Each agent independently computes a trajectory using Hybrid A* and Bezier optimization
2. **Check**: After optimization, check if the new trajectory is collision-free with known neighbor trajectories
3. **Broadcast**: If safe, broadcast the trajectory to all other agents
4. **Re-check**: If new trajectory information arrives during checking, invalidate and replan
5. **Execute**: Execute the validated trajectory while monitoring for new information

---

## 2. ROS Node and Communication Architecture

### 2.1 Per-Drone Node Structure

Each drone runs an identical software stack in its own ROS namespace (e.g., `/uav0/`, `/uav1/`):

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                      SINGLE DRONE NODE STRUCTURE (per UAV)                      │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│  ┌───────────────────────────────────────────────────────────────────────────┐  │
│  │                      PLANNER NODE (fake_baseline)                         │  │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐            │  │
│  │  │   FSM Manager   │  │  Baseline       │  │  Collision      │            │  │
│  │  │                 │  │  Planner        │  │  Avoider        │            │  │
│  │  │ (nh1_)          │  │  (nh2_)         │  │  (nh3_)         │            │  │
│  │  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘            │  │
│  │           │                    │                    │                     │  │
│  │           │ ros::CallbackQueue │ ros::CallbackQueue │ ros::CallbackQueue  │  │
│  │           │ (custom_queue1)    │ (custom_queue2)    │ (custom_queue3)     │  │
│  │           │                    │                    │                     │  │
│  │           ▼                    ▼                    ▼                     │  │
│  │  ┌─────────────────────────────────────────────────────────────────────┐  │  │
│  │  │              AsyncSpinner Pool (Multiple threads)                   │  │  │
│  │  │  spinner1(3) + spinner2(1) + spinner3(2) + spinner4(3)              │  │  │
│  │  └─────────────────────────────────────────────────────────────────────┘  │  │
│  └───────────────────────────────────────────────────────────────────────────┘  │
│                                                                                 │
│  ┌───────────────────────────────────────────────────────────────────────────┐  │
│  │                   TRAJECTORY SERVER (bezier_traj_server)                  │  │
│  │  • Converts Bezier trajectory to position commands                        │  │
│  │  • Runs with AsyncSpinner(2)                                              │  │
│  └───────────────────────────────────────────────────────────────────────────┘  │
│                                                                                 │
│  ┌───────────────────────────────────────────────────────────────────────────┐  │
│  │                   DYNAMICS SIMULATOR (poscmd_2_odom)                      │  │
│  │  • Simulates drone response to position commands                          │  │
│  └───────────────────────────────────────────────────────────────────────────┘  │
│                                                                                 │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Topic/Service Overview

#### Global Topics (Shared across all drones)

| Topic | Type | Publisher | Subscribers | Purpose |
|-------|------|-----------|-------------|---------|
| `/broadcast_traj` | `traj_utils/BezierTraj` | Each drone's planner | All other drones' collision avoiders | **Primary channel for trajectory sharing** |
| `/map_generator/global_cloud` | `sensor_msgs/PointCloud2` | Map Generator | All planners | Environment obstacle data |
| `/ground_truth_state` | `visualization_msgs/MarkerArray` | Map Generator | All planners | Ground truth dynamic obstacle states |
| `/traj_start_trigger` | `geometry_msgs/PoseStamped` | RViz/User | All drones | Trigger planning execution |

#### Per-Drone Topics (within `/uavX/` namespace)

| Topic | Type | Publisher | Subscriber | Purpose |
|-------|------|-----------|------------|---------|
| `planner/trajectory` | `traj_utils/BezierTraj` | FSM | Trajectory Server | Local trajectory for execution |
| `mavros/local_position/pose` | `geometry_msgs/PoseStamped` | Dynamics Sim | Planner, Traj Server | Current pose feedback |
| `controller/pos_cmd` | `quadrotor_msgs/PositionCommand` | Traj Server | Dynamics Sim | Position commands |

### 2.3 Communication Flow for Check-and-Recheck

```
Agent A plans trajectory  ──────────────────►  isSafeAfterOpt() called
                                                      │
                                                      ▼
                                          ┌──────────────────────┐
                                          │  is_checking_ = true │
                                          └──────────┬───────────┘
                                                     │
     ┌───────────────────────────────────────────────┼───────────────────────────┐
     │                                               │                           │
     ▼                                               ▼                           ▼
Agent B broadcasts          (Meanwhile)         Collision Check         Agent C broadcasts
trajectory via              trajectoryCallback()  with known             trajectory via
/broadcast_traj             is called            neighbor trajs          /broadcast_traj
     │                           │                    │                       │
     │                           ▼                    │                       │
     │                 have_received_traj_            │                       │
     │                 while_checking_ = true         │                       │
     │                           │                    │                       │
     └───────────────────────────┼────────────────────┼───────────────────────┘
                                 │                    │
                                 ▼                    ▼
                          ┌──────────────────────────────────┐
                          │ After check: isSafeAfterChk()    │
                          │ Returns false if flag was set    │
                          │ → Triggers replan                │
                          └──────────────────────────────────┘
```

---

## 3. Threading and Callback Model

### 3.1 How Asynchrony Is Implemented

The system achieves asynchrony through **multiple callback queues** with **dedicated AsyncSpinners**. This is a key ROS1 pattern for concurrent callback processing.

#### File: `plan_manager/src/fake_planner_node.cpp`

```cpp
int main(int argc, char **argv) {
  ros::init(argc, argv, "planner_node");
  
  // Create 4 separate NodeHandles, each with its own callback queue
  ros::NodeHandle nh1("~");  // FSM
  ros::NodeHandle nh2("~");  // Planner
  ros::NodeHandle nh3("~");  // Coordinator (collision avoider)
  ros::NodeHandle nh4("~");  // Map

  // Create separate callback queues
  ros::CallbackQueue custom_queue1;
  ros::CallbackQueue custom_queue2;
  ros::CallbackQueue custom_queue3;
  ros::CallbackQueue custom_queue4;

  // Assign queues to NodeHandles
  nh1.setCallbackQueue(&custom_queue1);
  nh2.setCallbackQueue(&custom_queue2);
  nh3.setCallbackQueue(&custom_queue3);
  nh4.setCallbackQueue(&custom_queue4);

  FiniteStateMachineFake plan_manager(nh1, nh2, nh3, nh4);
  plan_manager.run();

  // Start AsyncSpinners - each processes its own queue with dedicated threads
  ros::AsyncSpinner spinner1(3, &custom_queue1);  // 3 threads for FSM
  ros::AsyncSpinner spinner2(1, &custom_queue2);  // 1 thread for planner
  ros::AsyncSpinner spinner3(2, &custom_queue3);  // 2 threads for coordinator
  ros::AsyncSpinner spinner4(3, &custom_queue4);  // 3 threads for map

  spinner1.start();
  spinner2.start();
  spinner3.start();
  spinner4.start();

  ros::waitForShutdown();
  return 0;
}
```

### 3.2 Thread Assignment Table

| Component | NodeHandle | Callback Queue | AsyncSpinner | Threads | Callbacks Processed |
|-----------|------------|----------------|--------------|---------|---------------------|
| FSM | `nh1_` | `custom_queue1` | `spinner1` | 3 | `FSMCallback`, `TriggerCallback`, `PoseCallback` |
| Planner | `nh2_` | `custom_queue2` | `spinner2` | 1 | Planner-specific callbacks |
| Coordinator | `nh3_` | `custom_queue3` | `spinner3` | 2 | **`trajectoryCallback`** (key for check-and-recheck) |
| Map | `nh4_` | `custom_queue4` | `spinner4` | 3 | `cloudCallback`, `odomCallback`, `groundTruthStateCallback` |

### 3.3 Why This Threading Model Matters

The key insight is:

> **The trajectory callback (`trajectoryCallback`) runs on a different thread than the planning loop.**

This means:
1. The planner can be in the middle of checking a trajectory (`isSafeAfterOpt()`)
2. While simultaneously, a neighbor's trajectory arrives and triggers `trajectoryCallback()`
3. The callback sets `have_received_traj_while_checking_ = true`
4. When the check completes, `isSafeAfterChk()` detects this flag and returns `false`
5. This forces a re-check or replan

### 3.4 Callback Thread Analysis

#### Coordinator's trajectoryCallback (runs on spinner3 threads)

File: `traj_coordinator/src/mader.cpp`

```cpp
void MADER::trajectoryCallback(const traj_utils::BezierTraj::ConstPtr &traj_msg) {
  int id = traj_msg->drone_id;

  // Filter out ego trajectories
  if (id == drone_id_) {
    return;
  }

  // *** KEY CHECK-AND-RECHECK LOGIC ***
  if (is_checking_) {
    have_received_traj_while_checking_ = true;  // <-- Flag set asynchronously
    ROS_INFO("[CA|A%i] trajectory received during checking", id);
  } else {
    have_received_traj_while_optimizing_ = false;
  }

  // Update swarm_trajs_ buffer with received trajectory
  // ... (trajectory parsing and storage)
}
```

#### FSM Timer Callback (runs on spinner1 threads)

File: `plan_manager/src/fake_plan_manager.cpp`

```cpp
void FiniteStateMachineFake::FSMCallback(const ros::TimerEvent& event) {
  switch (status_) {
    case FSM_STATUS::REPLAN:
      // ... planning happens here
      bool is_success_ = planner_->replan(start_time, pos, vel, acc, goal_pos_);
      // replan() internally calls isSafeAfterOpt() which sets is_checking_ = true
      // ...
  }
}
```

### 3.5 Data Protection Between Threads

**Important Note**: The current implementation does **not** use explicit mutexes or locks for the shared flags. This works because:

1. The flags are simple boolean types (`bool`), which are typically atomic on most platforms
2. The worst case of a race condition is a conservative behavior (extra replan)
3. The `is_checking_` flag acts as a lightweight synchronization mechanism

For a more robust implementation, consider adding:

```cpp
// Example of thread-safe flag handling (not in current code, but recommended)
std::atomic<bool> is_checking_;
std::atomic<bool> have_received_traj_while_checking_;
std::mutex swarm_trajs_mutex_;
```

---

## 4. Detailed Implementation of Check-and-Recheck

### 4.1 Key Classes and Files

| Class | File | Purpose |
|-------|------|---------|
| `MADER` | `traj_coordinator/include/traj_coordinator/mader.hpp` | Base check-and-recheck implementation |
| `RMADER` | `traj_coordinator/include/traj_coordinator/rmader.hpp` | Robust MADER with delay check |
| `ParticleATC` | `traj_coordinator/include/traj_coordinator/particle.hpp` | Particle-based variant |
| `FakeBaselinePlanner` | `plan_manager/include/plan_manager/baseline_fake.h` | Orchestrates the planning pipeline |

### 4.2 Data Structures

#### SwarmTraj Structure (Neighbor trajectory representation)

```cpp
// File: traj_coordinator/include/traj_coordinator/mader.hpp
struct SwarmTraj {
  int    id;             // Drone ID
  double duration;       // Trajectory duration
  double time_received;  // ROS time when received
  double time_start;     // ROS time when trajectory starts
  double time_end;       // ROS time when trajectory ends
  Traj   traj;           // Bernstein::Bezier trajectory
};
```

#### MADER Class Members

```cpp
// File: traj_coordinator/include/traj_coordinator/mader.hpp
class MADER {
 protected:
  // ROS
  ros::NodeHandle nh_;
  ros::Subscriber swarm_sub_;

  // Trajectory storage
  std::vector<SwarmTraj> swarm_trajs_;        // Buffer of neighbor trajectories
  std::map<int, int>     drone_id_to_index_;  // Map drone ID to buffer index
  std::map<int, int>     index_to_drone_id_;  // Reverse mapping

  // *** CHECK-AND-RECHECK FLAGS ***
  bool is_planner_initialized_;
  bool have_received_traj_while_checking_;    // <-- Key flag
  bool have_received_traj_while_optimizing_;  // <-- Key flag
  bool is_traj_safe_;
  bool is_checking_;                          // <-- Key flag

  // Configuration
  int    drone_id_;
  int    num_robots_;
  double drone_size_x_, drone_size_y_, drone_size_z_;

  // Collision checking
  Eigen::Vector3d             ego_size_;
  Eigen::Matrix<double, 3, 8> ego_cube_;  // 8 vertices of ego bounding box
  separator::Separator *separator_solver_;  // Linear separability solver
};
```

### 4.3 Lifecycle of a Planning Iteration

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                        PLANNING ITERATION LIFECYCLE                             │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│  1. FSM enters REPLAN state                                                     │
│     │                                                                           │
│     ▼                                                                           │
│  2. FakeBaselinePlanner::replan() is called                                     │
│     │                                                                           │
│     ├──► 2a. Path search (Hybrid A*)                                            │
│     │                                                                           │
│     ├──► 2b. Corridor generation (FIRI)                                         │
│     │                                                                           │
│     ├──► 2c. Trajectory optimization (BezierOpt)                                │
│     │                                                                           │
│     ▼                                                                           │
│  3. collision_avoider_->isSafeAfterOpt(traj)  ◄─────────────────────────────┐   │
│     │                                                                       │   │
│     │  ┌───────────────────────────────────────────────────────────────┐   │   │
│     │  │ Inside isSafeAfterOpt():                                      │   │   │
│     │  │   is_checking_ = true;                                        │   │   │
│     │  │   for each neighbor trajectory:                               │   │   │
│     │  │     - Build convex hull of ego trajectory                     │   │   │
│     │  │     - Build convex hull of neighbor trajectory                │   │   │
│     │  │     - Check linear separability (separator_solver_)           │   │   │
│     │  │   is_checking_ = false;                                       │   │   │
│     │  │   return collision_free;                                      │   │   │
│     │  └───────────────────────────────────────────────────────────────┘   │   │
│     │                                                                       │   │
│     │  (Meanwhile, on another thread)                                       │   │
│     │  ┌───────────────────────────────────────────────────────────────┐   │   │
│     │  │ trajectoryCallback() may be called:                           │   │   │
│     │  │   if (is_checking_):                                          │   │   │
│     │  │     have_received_traj_while_checking_ = true; ────────────────┼───┘   │
│     │  │   store received trajectory                                   │       │
│     │  └───────────────────────────────────────────────────────────────┘       │
│     │                                                                           │
│     ▼                                                                           │
│  4. If collision detected → return false → FSM triggers new planning            │
│     │                                                                           │
│     ▼                                                                           │
│  5. (Optional) collision_avoider_->isSafeAfterChk()                             │
│     │  - Checks if have_received_traj_while_checking_ was set                   │
│     │  - If true: invalidates the trajectory → replan                           │
│     │                                                                           │
│     ▼                                                                           │
│  6. If all checks pass:                                                         │
│     │  - prev_traj_start_time_ = traj_start_time_;                              │
│     │  - traj_ = traj;                                                          │
│     │  - return true;                                                           │
│     │                                                                           │
│     ▼                                                                           │
│  7. FSM publishes trajectory via publishTrajectory()                            │
│     │  - Publishes to local trajectory topic                                    │
│     │  - Broadcasts to /broadcast_traj for other agents                         │
│     │                                                                           │
│     ▼                                                                           │
│  8. FSM transitions to EXEC_TRAJ state                                          │
│                                                                                 │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### 4.4 Core Functions Implementation

#### isSafeAfterOpt() - The Check Step

File: `traj_coordinator/src/mader.cpp`

```cpp
bool MADER::isSafeAfterOpt(const Traj &traj) {
  is_checking_ = true;  // <-- Signal that checking has begun
  
  // Get ego trajectory control points
  Eigen::MatrixXd cpts;
  traj.getCtrlPoints(cpts);

  std::vector<Eigen::Vector3d> pointsA;  // Ego trajectory convex hull
  std::vector<Eigen::Vector3d> pointsB;  // Neighbor trajectory convex hull

  // Build Minkowski sum of ego trajectory + ego bounding box
  loadVertices(pointsA, cpts);

  // Check against each neighbor's trajectory
  for (auto &traj : swarm_trajs_) {
    if (traj.id == drone_id_) continue;

    double t0 = ros::Time::now().toSec();
    Eigen::Vector3d n_k;
    double d_k;
    pointsB.clear();

    // Only check if neighbor trajectory is active
    if (traj.time_start < t0 && t0 < traj.time_end) {
      Eigen::MatrixXd cpts = traj.traj.getCtrlPoints();

      // Remove passed control points for efficiency
      int order = traj.traj.getOrder() + 1;
      int piece_idx = traj.traj.locatePiece(t0 - traj.time_start);
      cpts = cpts.bottomRows(cpts.rows() - piece_idx * order);

      loadVertices(pointsB, cpts);

      // Check linear separability using QP solver
      if (!separator_solver_->solveModel(n_k, d_k, pointsA, pointsB)) {
        ROS_WARN("[CA] Drone %i will collide with drone %i", drone_id_, traj.id);
        is_checking_ = false;
        return false;  // <-- Collision detected
      }
    }
  }
  
  is_checking_ = false;  // <-- Signal that checking has ended
  return true;  // <-- Safe
}
```

#### isSafeAfterChk() - The Re-check Step

File: `traj_coordinator/include/traj_coordinator/mader.hpp`

```cpp
bool isSafeAfterChk() {
  if (have_received_traj_while_checking_) {
    have_received_traj_while_checking_ = false;  // Reset flag
    ROS_INFO("[CA] Committed during checking");
    return false;  // <-- Force replan
  }
  return true;
}
```

#### Robust MADER Delay Check

File: `traj_coordinator/src/rmader.cpp`

```cpp
bool RMADER::isSafeAfterDelayCheck(const Bernstein::Bezier &traj) {
  double t = 0.0;
  double dc_start = ros::Time::now().toSec();

  // Continuously check for the delay period (delta_dc_, typically 50ms)
  while (t < delta_dc_) {
    t = ros::Time::now().toSec() - dc_start;
    bool is_safe = collisionCheck(traj);
    if (!is_safe) {
      return false;
    }
  }
  ROS_WARN("[CA] Drone %i passed the delay check", drone_id_);
  return true;
}
```

### 4.5 Sequence Diagram: Two Agents Interacting

```
   Agent A                        ROS Network                       Agent B
      │                               │                                │
      │  ┌─────────────────┐          │                                │
      │  │ Start planning  │          │                                │
      │  └────────┬────────┘          │                                │
      │           │                   │                                │
      │           ▼                   │                                │
      │  ┌─────────────────┐          │         ┌─────────────────┐    │
      │  │ Hybrid A*       │          │         │ Start planning  │    │
      │  │ search          │          │         └────────┬────────┘    │
      │  └────────┬────────┘          │                  │             │
      │           │                   │                  ▼             │
      │           ▼                   │         ┌─────────────────┐    │
      │  ┌─────────────────┐          │         │ isSafeAfterOpt  │    │
      │  │ Bezier          │          │         │ is_checking_=T  │    │
      │  │ optimization    │          │         └────────┬────────┘    │
      │  └────────┬────────┘          │                  │             │
      │           │                   │                  │             │
      │           ▼                   │                  │             │
      │  ┌─────────────────┐          │                  │             │
      │  │ isSafeAfterOpt  │          │                  │             │
      │  │ is_checking_=T  │          │                  │             │
      │  └────────┬────────┘          │                  │             │
      │           │                   │                  │             │
      │           │ (checking)        │                  │ (checking)  │
      │           │                   │                  │             │
      │           ▼                   │                  ▼             │
      │  ┌─────────────────┐          │         ┌─────────────────┐    │
      │  │ Check passes    │          │         │ Check passes    │    │
      │  │ is_checking_=F  │          │         │ is_checking_=F  │    │
      │  └────────┬────────┘          │         └────────┬────────┘    │
      │           │                   │                  │             │
      │           ▼                   │                  ▼             │
      │  ┌─────────────────┐          │         ┌─────────────────┐    │
      │  │ Publish traj    │───────►  │  ◄──────│ Publish traj    │    │
      │  │ /broadcast_traj │          │         │ /broadcast_traj │    │
      │  └────────┬────────┘          │         └────────┬────────┘    │
      │           │                   │                  │             │
      │           │                   │                  │             │
      │  ┌────────▼────────┐          │         ┌────────▼────────┐    │
      │  │ trajectoryCallback         │         │ trajectoryCallback   │
      │  │ receives B's traj│ ◄───────│─────────│ receives A's traj│   │
      │  │ (from B)         │         │         │ (from A)         │   │
      │  └────────┬────────┘          │         └────────┬────────┘    │
      │           │                   │                  │             │
      │           ▼                   │                  ▼             │
      │  ┌─────────────────┐          │         ┌─────────────────┐    │
      │  │ Update          │          │         │ Update          │    │
      │  │ swarm_trajs_    │          │         │ swarm_trajs_    │    │
      │  └─────────────────┘          │         └─────────────────┘    │
      │                               │                                │
      │       ┌──────────────────────────────────────────┐             │
      │       │  Next planning cycle will consider the   │             │
      │       │  other agent's trajectory as an obstacle │             │
      │       └──────────────────────────────────────────┘             │
      │                               │                                │
```

---

## 5. Asynchronous Inter-Agent Communication

### 5.1 How Information About Other Agents Is Obtained

Each agent subscribes to the global `/broadcast_traj` topic:

```cpp
// File: traj_coordinator/src/mader.cpp (or particles.cpp)
void MADER::init() {
  // Subscribe to trajectory broadcast
  swarm_sub_ = nh_.subscribe("/broadcast_traj", 1, &MADER::trajectoryCallback, this);
  // ...
}
```

### 5.2 Message Structure

File: `traj_utils/msg/BezierTraj.msg` (implied from code)

```
int32      drone_id    # Source agent ID
int32      traj_id     # Trajectory sequence number
time       start_time  # When trajectory execution begins
time       pub_time    # When message was published
int32      order       # Bezier polynomial order
float64[]  duration    # Duration of each piece
Point[]    cpts        # Control points
```

### 5.3 Message Frequency

- **Publishing**: Trajectories are broadcast whenever a new plan is successfully validated
- **Typical frequency**: Depends on replanning rate (configured via `fsm/replan_duration`)
- **In the default config**: Replanning every ~0.1 seconds when in EXEC_TRAJ state

### 5.4 Handling Asynchronous Arrival

```cpp
// File: traj_coordinator/src/mader.cpp
void MADER::trajectoryCallback(const traj_utils::BezierTraj::ConstPtr &traj_msg) {
  int id = traj_msg->drone_id;

  // 1. Filter out own trajectories
  if (id == drone_id_) {
    return;
  }

  // 2. Set flag if currently checking (KEY ASYNC LOGIC)
  if (is_checking_) {
    have_received_traj_while_checking_ = true;
    ROS_INFO("[CA|A%i] trajectory received during checking", id);
  }

  // 3. Parse trajectory from message
  int n_seg = traj_msg->duration.size();
  int order = traj_msg->order + 1;

  SwarmTraj traj;
  traj.id = traj_msg->drone_id;
  traj.time_received = ros::Time::now().toSec();
  traj.time_start = traj_msg->start_time.toSec();
  // ... (parse durations and control points)

  // 4. Update or create entry in swarm_trajs_ buffer
  auto obs_ptr = std::find_if(swarm_trajs_.begin(), swarm_trajs_.end(),
                              [=](const SwarmTraj &tmp) { return tmp.id == traj.id; });

  if (obs_ptr != swarm_trajs_.end()) {
    *obs_ptr = traj;  // Update existing
    ROS_INFO("[CA|A%i] traj updated", traj.id);
  } else {
    swarm_trajs_.push_back(traj);  // Create new
    ROS_INFO("[CA|A%i] traj created", traj.id);
  }
}
```

### 5.5 Avoiding Stale Data

The code handles stale trajectories by:

1. **Time-based validity**: Only trajectories whose time interval overlaps with the planning horizon are considered
2. **Lazy cleanup**: Old trajectories are filtered out during collision checking:

```cpp
// Inside isSafeAfterOpt()
if (traj.time_start < t0 && t0 < traj.time_end) {
  // Only check active trajectories
}
```

### 5.6 What Makes This "Asynchronous"

The approach is asynchronous because:

1. **No global clock synchronization**: Agents operate on their own local clocks
2. **No coordination protocol**: Agents don't wait for acknowledgments or barriers
3. **Best-effort communication**: Trajectories are broadcast without guaranteed delivery order
4. **Reactive behavior**: Agents react to received information without blocking
5. **Conservative failure mode**: When uncertain (flag set), agents choose to replan

This differs from synchronous approaches where all agents would:
- Wait at a barrier before planning
- Exchange information in lock-step rounds
- Require consensus before proceeding

---

## 6. Data Structures and State Tracking

### 6.1 Key Data Members and Their Purposes

#### In MADER/ParticleATC Class

| Member | Type | Purpose | Written By | Read By |
|--------|------|---------|------------|---------|
| `swarm_trajs_` | `std::vector<SwarmTraj>` | Buffer of neighbor trajectories | `trajectoryCallback()` | `isSafeAfterOpt()`, `getObstaclePoints()` |
| `is_checking_` | `bool` | Flag indicating collision check in progress | `isSafeAfterOpt()` | `trajectoryCallback()` |
| `have_received_traj_while_checking_` | `bool` | Flag indicating new data arrived during check | `trajectoryCallback()` | `isSafeAfterChk()` |
| `ego_cube_` | `Eigen::Matrix<double, 3, 8>` | 8 vertices of ego bounding box | `init()` | `loadVertices()` |
| `separator_solver_` | `separator::Separator*` | QP solver for linear separability | `init()` | `isSafeAfterOpt()` |

#### In FiniteStateMachineFake Class

| Member | Type | Purpose | Written By | Read By |
|--------|------|---------|------------|---------|
| `status_` | `FSM_STATUS` | Current state machine state | `FSMChangeState()` | `FSMCallback()` |
| `traj_start_time_` | `ros::Time` | Start time of current trajectory | `FSMCallback()` | `checkTimeLapse()`, `publishTrajectory()` |
| `planner_` | `FakeBaselinePlanner::Ptr` | The actual planner instance | `run()` | `FSMCallback()` |

#### In FakeBaselinePlanner Class

| Member | Type | Purpose | Written By | Read By |
|--------|------|---------|------------|---------|
| `collision_avoider_` | `ParticleATC::Ptr` | Multi-agent coordination module | `init()` | `replan()`, `isTrajSafe()` |
| `traj_` | `Bernstein::Bezier` | Current validated trajectory | `replan()` | `getTrajectory()`, `getPos()`, `getVel()`, `getAcc()` |
| `prev_traj_start_time_` | `double` | Start time of last committed trajectory | `replan()` | `getPos()`, `isPrevTrajFinished()` |

### 6.2 State Update Flow

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                           DATA UPDATE FLOW                                      │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│  External Event                    Internal State Change                        │
│  ──────────────                    ─────────────────────                        │
│                                                                                 │
│  [Pose callback]                   odom_pos_, odom_vel_ updated                 │
│       │                                   │                                     │
│       └──────────────────────────────────►│                                     │
│                                           ▼                                     │
│  [FSM Timer fires]                 status_ transitions                          │
│       │                                   │                                     │
│       │    ┌──────────────────────────────┘                                     │
│       │    │                                                                    │
│       │    ▼                                                                    │
│       │  [REPLAN state]                                                         │
│       │    │                                                                    │
│       │    ├──► planner_->replan() called                                       │
│       │    │         │                                                          │
│       │    │         ├──► collision_avoider_->isSafeAfterOpt()                  │
│       │    │         │         │                                                │
│       │    │         │         ├──► is_checking_ = true                         │
│       │    │         │         │                                                │
│       │    │         │         │    ┌─────────────────────────┐                 │
│       │    │         │         │    │ trajectoryCallback()    │                 │
│       │    │         │         │◄───│ runs on another thread  │                 │
│       │    │         │         │    │ sets have_received...   │                 │
│       │    │         │         │    └─────────────────────────┘                 │
│       │    │         │         │                                                │
│       │    │         │         ├──► is_checking_ = false                        │
│       │    │         │         │                                                │
│       │    │         │         └──► returns bool                                │
│       │    │         │                                                          │
│       │    │         ├──► if success: traj_ = new_traj                          │
│       │    │         │                prev_traj_start_time_ = t                 │
│       │    │         │                                                          │
│       │    │         └──► returns bool                                          │
│       │    │                                                                    │
│       │    └──► if success: publishTrajectory()                                 │
│       │              │                                                          │
│       │              └──► broadcast_traj_pub_.publish(msg)                      │
│       │                         │                                               │
│       │                         │  [Other agents receive]                       │
│       │                         └──► Their trajectoryCallback() runs            │
│       │                                  └──► Their swarm_trajs_ updated        │
│       │                                                                         │
│       └──────────────────────────────────────────────────────────────────────►  │
│                                                                                 │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### 6.3 Consistency Mechanisms

#### 1. Flag-Based Synchronization

```cpp
// The is_checking_ flag acts as a lightweight synchronization mechanism
if (is_checking_) {
    have_received_traj_while_checking_ = true;
}
```

#### 2. Time-Based Validity

```cpp
// Trajectories are only considered if time-valid
if (traj.time_start < t0 && t0 < traj.time_end) {
    // Use this trajectory
}
```

#### 3. Replace-on-Update Policy

```cpp
// When updating, replace entirely rather than merge
auto obs_ptr = std::find_if(swarm_trajs_.begin(), swarm_trajs_.end(),
                            [=](const SwarmTraj &tmp) { return tmp.id == traj.id; });
if (obs_ptr != swarm_trajs_.end()) {
    *obs_ptr = traj;  // Complete replacement
}
```

### 6.4 Thread Safety Notes

**Current Implementation Observations:**

1. The flags (`is_checking_`, `have_received_traj_while_checking_`) are accessed without explicit locks
2. The `swarm_trajs_` vector is modified in callbacks and read in planning functions
3. On most platforms, simple boolean reads/writes are atomic, so the flag pattern works

**Recommendations for Production Code:**

```cpp
// Recommended additions for thread safety
class MADER {
 protected:
  std::atomic<bool> is_checking_{false};
  std::atomic<bool> have_received_traj_while_checking_{false};
  std::mutex swarm_trajs_mutex_;

  // In trajectoryCallback():
  {
    std::lock_guard<std::mutex> lock(swarm_trajs_mutex_);
    // Update swarm_trajs_
  }

  // In isSafeAfterOpt():
  {
    std::lock_guard<std::mutex> lock(swarm_trajs_mutex_);
    // Read swarm_trajs_
  }
};
```

---

## 7. Summary and How to Replicate This Structure

### 7.1 Key Takeaways

1. **Decentralized Architecture**: Each agent runs identical software in its own namespace
2. **Asynchronous Communication**: Agents broadcast trajectories without waiting for acknowledgments
3. **Flag-Based Coordination**: Simple boolean flags detect concurrent updates
4. **Multiple Callback Queues**: ROS1's AsyncSpinner with separate queues enables true parallelism
5. **Conservative Failure Mode**: When in doubt, replan rather than risk collision

### 7.2 Step-by-Step Guide to Replicate

#### Step 1: Set Up Multi-Threaded ROS Node

```cpp
// your_planner_node.cpp
int main(int argc, char **argv) {
  ros::init(argc, argv, "planner_node");

  // Create separate NodeHandles for different components
  ros::NodeHandle nh_fsm("~");
  ros::NodeHandle nh_planner("~");
  ros::NodeHandle nh_coordinator("~");

  // Create separate callback queues
  ros::CallbackQueue queue_fsm, queue_planner, queue_coordinator;
  nh_fsm.setCallbackQueue(&queue_fsm);
  nh_planner.setCallbackQueue(&queue_planner);
  nh_coordinator.setCallbackQueue(&queue_coordinator);

  // Initialize your planner
  MyPlanner planner(nh_fsm, nh_planner, nh_coordinator);
  planner.init();

  // Start async spinners
  ros::AsyncSpinner spinner_fsm(2, &queue_fsm);
  ros::AsyncSpinner spinner_planner(1, &queue_planner);
  ros::AsyncSpinner spinner_coordinator(2, &queue_coordinator);

  spinner_fsm.start();
  spinner_planner.start();
  spinner_coordinator.start();

  ros::waitForShutdown();
  return 0;
}
```

#### Step 2: Implement Coordinator Class

```cpp
// coordinator.hpp
class Coordinator {
 public:
  void init(ros::NodeHandle &nh) {
    nh.param("drone_id", drone_id_, 0);
    nh.param("num_robots", num_robots_, 4);

    traj_sub_ = nh.subscribe("/broadcast_traj", 10,
                             &Coordinator::trajectoryCallback, this);

    swarm_trajs_.reserve(num_robots_ - 1);
    is_checking_ = false;
    have_received_traj_while_checking_ = false;
  }

  void trajectoryCallback(const YourTrajMsg::ConstPtr &msg) {
    if (msg->drone_id == drone_id_) return;

    // KEY: Set flag if checking
    if (is_checking_.load()) {
      have_received_traj_while_checking_.store(true);
    }

    // Update trajectory buffer
    std::lock_guard<std::mutex> lock(mutex_);
    updateSwarmTrajectory(msg);
  }

  bool isSafeAfterOpt(const YourTrajectory &traj) {
    is_checking_.store(true);

    bool safe = true;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto &neighbor_traj : swarm_trajs_) {
        if (!checkCollision(traj, neighbor_traj)) {
          safe = false;
          break;
        }
      }
    }

    is_checking_.store(false);
    return safe;
  }

  bool isSafeAfterChk() {
    if (have_received_traj_while_checking_.exchange(false)) {
      return false;  // Force replan
    }
    return true;
  }

 private:
  int drone_id_;
  int num_robots_;
  ros::Subscriber traj_sub_;

  std::atomic<bool> is_checking_;
  std::atomic<bool> have_received_traj_while_checking_;
  std::mutex mutex_;
  std::vector<NeighborTraj> swarm_trajs_;
};
```

#### Step 3: Implement FSM with Planning Loop

```cpp
// fsm.cpp
void FSM::planCallback(const ros::TimerEvent &event) {
  switch (state_) {
    case REPLAN:
      // 1. Compute trajectory
      YourTrajectory new_traj = planner_->plan(start, goal);

      // 2. Check against known neighbors
      if (!coordinator_->isSafeAfterOpt(new_traj)) {
        ROS_WARN("Collision detected, replanning...");
        break;  // Stay in REPLAN
      }

      // 3. Check if new data arrived during check
      if (!coordinator_->isSafeAfterChk()) {
        ROS_WARN("New trajectory received during check, replanning...");
        break;  // Stay in REPLAN
      }

      // 4. Commit and broadcast
      current_traj_ = new_traj;
      publishTrajectory(new_traj);
      broadcastTrajectory(new_traj);

      state_ = EXECUTING;
      break;

    case EXECUTING:
      // Monitor and trigger replan if needed
      if (shouldReplan()) {
        state_ = REPLAN;
      }
      break;
  }
}
```

#### Step 4: Set Up Launch File for Multi-Agent

```xml
<!-- multi_agent.launch -->
<launch>
  <arg name="num_robots" default="4" />

  <group ns="robot0">
    <include file="$(find your_pkg)/launch/single_robot.launch">
      <arg name="drone_id" value="0" />
      <arg name="num_robots" value="$(arg num_robots)" />
    </include>
  </group>

  <group ns="robot1">
    <include file="$(find your_pkg)/launch/single_robot.launch">
      <arg name="drone_id" value="1" />
      <arg name="num_robots" value="$(arg num_robots)" />
    </include>
  </group>

  <!-- Add more robots as needed -->
</launch>
```

### 7.3 Common Pitfalls to Avoid

1. **Don't use a single callback queue**: This serializes all callbacks and defeats the async purpose
2. **Don't forget to set `is_checking_` back to false**: Always use RAII or try-finally patterns
3. **Don't assume message order**: ROS doesn't guarantee message ordering across topics
4. **Don't skip the re-check step**: It's the key to handling race conditions
5. **Don't use blocking operations in callbacks**: Keep callbacks short; offload work to main loop

### 7.4 Testing Your Implementation

1. **Single agent test**: Verify planning works without coordination
2. **Two agent test**: Create crossing scenario, verify no collisions
3. **Race condition test**: Add artificial delays in `isSafeAfterOpt()`, verify re-check triggers
4. **Scalability test**: Increase agent count, monitor CPU usage and message latency

### 7.5 Further Reading

- [MADER Paper](https://arxiv.org/abs/2010.11061): Original algorithm description
- [Robust MADER Paper](https://arxiv.org/abs/2109.04927): Delay check extension
- [ROS AsyncSpinner Documentation](http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning)
- [Planning Pipeline Documentation](PLANNING_PIPELINE_DOCUMENTATION.md): Detailed explanation of the planning system
- [System Architecture Diagram](SYSTEM_ARCHITECTURE.md): Visual representation of component interactions
- [Quick Reference Guide](QUICK_REFERENCE.md): Commands, topics, and troubleshooting tips

---

## Appendix A: File Reference

| File | Purpose |
|------|---------|
| `plan_manager/src/fake_planner_node.cpp` | Main node entry point, sets up threads |
| `plan_manager/src/fake_plan_manager.cpp` | FSM implementation |
| `plan_manager/include/plan_manager/fake_plan_manager.h` | FSM class definition |
| `plan_manager/src/baseline_fake.cpp` | Planner implementation |
| `plan_manager/include/plan_manager/baseline_fake.h` | Planner class definition |
| `traj_coordinator/src/mader.cpp` | MADER check-and-recheck implementation |
| `traj_coordinator/include/traj_coordinator/mader.hpp` | MADER class definition |
| `traj_coordinator/src/rmader.cpp` | Robust MADER with delay check |
| `traj_coordinator/include/traj_coordinator/rmader.hpp` | RMADER class definition |
| `traj_coordinator/src/particles.cpp` | ParticleATC implementation |
| `traj_coordinator/include/traj_coordinator/particle.hpp` | ParticleATC class definition |
| `plan_manager/config/sim_fake.yaml` | Configuration parameters |
| `plan_manager/launch/sim_fkpcp_4_case_4.launch` | 4-drone launch file |
| `plan_manager/launch/simulator/vis_drone_fake_perception.xml` | Per-drone stack |

---

## Appendix B: Configuration Parameters

Key parameters affecting the check-and-recheck behavior (from `config/sim_fake.yaml`):

```yaml
fsm:
  replan_duration: 0.1        # How often to attempt replanning (seconds)
  replan_start_time: 0.02     # Look-ahead time for replan start position
  colli_check_duration: 0.2   # Duration to check trajectory safety

swarm:
  num_robots: 4               # Total number of agents
  drone_size_x: 0.4           # Agent bounding box dimensions
  drone_size_y: 0.4
  drone_size_z: 0.45
  replan_risk_rate: 0.00      # Risk threshold for replanning
```

---

*This document was generated to explain the asynchronous check-and-recheck implementation in the pred-occ-planner repository. For questions or clarifications, please refer to the source code or open an issue.*
