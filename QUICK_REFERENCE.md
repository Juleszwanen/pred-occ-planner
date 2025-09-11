# Quick Reference Guide

## Quick Start

```bash
# Build the workspace
catkin build

# Source the workspace
source devel/setup.bash

# Launch the 4-drone simulation
roslaunch plan_manager sim_fkpcp_4_case_4.launch

# Trigger planning (in RViz)
# 1. Click "2D Nav Goal" tool
# 2. Click anywhere in the map
# 3. Drones will start planning and moving
```

## Key Commands and Topics

### Launch Files
```bash
# 4-drone crossing scenario
roslaunch plan_manager sim_fkpcp_4_case_4.launch

# Other available scenarios
roslaunch plan_manager sim_fkpcp_4_case_2.launch  # Different obstacle config
roslaunch plan_manager sim_fkpcp_4_case_3.launch  # Different obstacle config
roslaunch plan_manager sim_fkpcp_6_case_0.launch  # 6-drone scenario
roslaunch plan_manager sim_fkpcp_8_case_1.launch  # 8-drone scenario
```

### Important Topics

#### Environment
- `/map_generator/global_cloud` - Global environment point cloud
- `/ground_truth_state` - Ground truth obstacle states

#### Individual Drones (replace X with 0,1,2,3)
- `/uavX/planner/trajectory` - Planned trajectory
- `/uavX/mavros/local_position/pose` - Current drone pose
- `/uavX/mavros/local_position/odom` - Current drone odometry
- `/uavX/controller/pos_cmd` - Position commands

#### Multi-Agent Coordination
- `/broadcast_traj` - Shared trajectory information between drones

### Monitoring Commands

```bash
# Monitor drone positions
rostopic echo /uav0/mavros/local_position/pose

# Monitor planned trajectories
rostopic echo /uav0/planner/trajectory

# Monitor environment obstacles
rostopic echo /map_generator/global_cloud

# Monitor coordination messages
rostopic echo /broadcast_traj

# View all active topics
rostopic list

# View node graph
rqt_graph
```

## Configuration Files

### Main Configuration
- `plan_manager/config/sim_fake.yaml` - Main planning parameters
- `plan_manager/launch/sim_fkpcp_4_case_4.launch` - Launch configuration

### Key Parameters to Modify

#### In `sim_fake.yaml`:
```yaml
# Planning speed and aggressiveness
search:
  max_vel: 2          # Maximum velocity (m/s)
  max_acc: 6          # Maximum acceleration (m/s²)
  margin: 0.6         # Safety margin (m)

# Replanning behavior  
fsm:
  replan_tolerance: 1.0    # Distance to trigger replan (m)
  replan_duration: 0.1     # Replanning frequency (s)

# Multi-agent settings
swarm:
  num_robots: 4           # Number of drones
  drone_size_x: 0.4       # Drone dimensions (m)
```

#### In launch file:
```xml
<!-- Environment size -->
<arg name="map_size_x" value="16" />
<arg name="map_size_y" value="16" />
<arg name="map_size_z" value="7" />

<!-- Number of obstacles -->
<arg name="obs_num" default="20" />

<!-- Drone start/goal positions -->
<arg name="init_x" value="-8" />
<arg name="goal_x" value="8" />
```

## Troubleshooting

### Common Issues

1. **Drones not moving after clicking 2D Nav Goal**
   - Check if all planners are initialized: `rostopic echo /uav0/planner/trajectory`
   - Ensure environment is loaded: `rostopic echo /map_generator/global_cloud`

2. **Build errors**
   - Install OSQP dependency: `sudo apt install libosqp-dev`
   - Update submodules: `git submodule update --init --recursive`

3. **RViz crashes or slow performance**
   - Reduce point cloud density in map generator parameters
   - Disable some visualization elements

4. **Drones colliding**
   - Increase safety margin in `sim_fake.yaml`
   - Check multi-agent coordination: `rostopic echo /broadcast_traj`

### Debugging Commands

```bash
# Check if all nodes are running
rosnode list | grep -E "(uav|map_generator|rviz)"

# Monitor planning status
rostopic echo /uav0/planner/trajectory --noarr

# Check for error messages
rosrun rqt_console rqt_console

# Monitor computation time
rostopic hz /uav0/planner/trajectory
```

## Performance Tuning

### For Better Performance
- Reduce map resolution in `sim_fake.yaml`:
  ```yaml
  map:
    resolution: 0.20  # Increase from 0.15
  ```

### For More Challenging Scenarios
- Increase number of obstacles in launch file
- Reduce safety margins
- Add more drones
- Make obstacles move faster

### For Smoother Trajectories
- Increase trajectory optimization iterations
- Reduce replanning frequency
- Increase planning horizon

## Extension Points

### Adding New Scenarios
1. Copy existing launch file
2. Modify drone start/goal positions
3. Adjust environment parameters
4. Test with different obstacle configurations

### Modifying Planning Behavior
1. Edit parameters in `sim_fake.yaml`
2. Modify source code in `plan_manager/src/`
3. Rebuild with `catkin build plan_manager`

### Custom Obstacle Patterns
1. Modify `map_generator` parameters in launch file
2. Create custom obstacle motion patterns
3. Adjust obstacle shapes and sizes