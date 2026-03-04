# robot_master

A ROS 2 workspace for an autonomous mobile robot featuring custom FGO-based localization, graph-based global planning, a holonomic local planner, and a full Nav2 navigation stack.

---

## Repository Structure

| Package | Description |
|---|---|
| `robot_bringup` | Top-level launch files for the full system |
| `robot_description` | URDF / xacro robot model |
| `saye_description` | URDF model for the Saye robot variant |
| `robot_fgo_localization` | Custom Factor Graph Optimization (FGO) localization node |
| `robot_localization` | Additional localization utilities and sensor fusion helpers |
| `robot_navigation` | Nav2 configuration, maps, behavior trees, and launch files |
| `robot_custom_local_planner` | Custom DWA-style local planner for holonomic (mecanum/omni) robots |
| `robot_custom_behaviour_tree_nodes` | Custom BT action/condition nodes (battery check, emergency stop, charge station) |
| `robot_custom_behaviour_executor` | Executor for custom behavior trees |
| `robot_custom_layers` | Custom Nav2 costmap layer (`ProhibitedLayer`) |
| `robot_graph_based_global_planner` | Graph-based global planner that routes through a JSON waypoint graph |
| `robot_database` | Persistent data storage (e.g. saved poses, waypoint records) |
| `robot_interfaces` | Custom ROS 2 message and service definitions |
| `dual_laser_merger` | Merges two SICK LiDAR scans into a single `/scan` topic |
| `bt_visualizer_pkg` | Behavior tree visualization tooling |
| `robot_teleop` | Teleoperation node |
| `robot_gazebo` | Gazebo simulation world and launch files |
| `navigation2` | Upstream Nav2 stack (vendored/patched) |

---

## Localization

### `robot_fgo_localization`

The primary localization method is a custom **Factor Graph Optimization (FGO)** node built on **GTSAM** with incremental solving via **iSAM2**.

#### How It Works

The node maintains a pose graph where each odometry step creates a new node. Sensor measurements are fused as probabilistic factors:

| Factor | Type | Topic |
|---|---|---|
| Odometry | `BetweenFactor` (graph backbone) | `/odometry` |
| IMU | Yaw `PriorFactor` | `/imu` |
| LiDAR (ICP scan match) | Position `PriorFactor` | `/scan` |
| GPS *(optional)* | XY `PriorFactor` | `/gps/fix` |

#### Scan Matching

LiDAR is matched against the loaded occupancy map using **ICP (Iterative Closest Point)**. The scan is automatically transformed from the laser frame to `base_footprint` via TF before matching.

Key ICP parameters (tunable in `fgo_params.yaml`):

```yaml
sensors.lidar.icp_max_correspondence_dist: 0.5   # meters
sensors.lidar.icp_max_iterations:          50
sensors.lidar.icp_voxel_leaf_size:         0.05  # meters
sensors.lidar.min_fitness:                 0.30  # lower = stricter
```

#### TF Tree

```
map
 └── odom          (published by FGO node at 50 Hz)
      └── base_footprint  (raw odometry passthrough)
```

#### Configuration: `fgo_params.yaml`

```yaml
# Enable/disable sensors individually
sensors.odometry.enabled: true
sensors.imu.enabled:      true
sensors.lidar.enabled:    true
sensors.gps.enabled:      false   # set true for outdoor use

# iSAM2 solver
isam2_relinearize_threshold: 0.1
isam2_relinearize_skip:      10
```

For **GPS** (outdoor), set `sensors.gps.enabled: true` and configure the datum (reference lat/lon treated as map origin):

```yaml
sensors.gps.datum_lat: 48.123456
sensors.gps.datum_lon: 11.654321
sensors.gps.noise_x:   2.0   # meters, typical GPS horizontal error
```

#### Initialization

- On startup the node waits 5 seconds for a map and an `/initialpose` message.
- If none arrives it auto-starts at the map origin `(0, 0, 0)`.
- Sending a `2D Pose Estimate` in RViz resets the graph to the new pose.

---

## Navigation

The navigation stack is built on **Nav2** and configured in `robot_navigation/config/robot_move_base.yaml`.

### Global Planner — `robot_graph_based_global_planner`

A custom **graph-based planner** that routes through a predefined JSON waypoint graph instead of searching a full costmap grid.

- Graph file: `robot_navigation/graphs/robot_map_graph.json`
- Finds the closest graph node to the start and goal (within `max_node_search_radius: 5.0 m`).
- Interpolates poses along edges at `interpolation_resolution: 0.1 m`.

Plugin registration:
```
robot_graph_based_global_planner::GraphBasedPlanner
```

### Local Planner — `robot_custom_local_planner`

A custom **DWA-style holonomic controller** (`CustomLocalPlanner`) designed for mecanum/omni-wheel robots (full Vx, Vy, ω control).

**Algorithm:**
1. Prune the global plan to a local window (`prune_plan_distance: 3.0 m`).
2. Sample a 3-D velocity space (Vx × Vy × ω) — default 7 × 7 × 9 = 441 trajectories per cycle.
3. Simulate each trajectory forward (`sim_time: 2.0 s`).
4. Score by weighted sum of: path alignment, goal distance, and obstacle cost.
5. Execute the lowest-cost trajectory.

Key parameters:

```yaml
max_linear_vel_x:  0.5   # m/s
max_linear_vel_y:  0.5   # m/s (lateral strafe)
max_angular_vel:   1.0   # rad/s
controller_frequency: 20.0 Hz
```

Plugin registration:
```
robot_custom_local_planner::CustomLocalPlanner
```

### Costmaps

Both local and global costmaps observe two **SICK LiDAR** sources:

| Source | Topic |
|---|---|
| Front LiDAR | `/sick_lidar0/scan` |
| Rear LiDAR | `/sick_lidar1/scan` |

**Local costmap** (3 × 3 m rolling window, 0.06 m resolution):
- `StaticLayer` + `VoxelLayer` + `ProhibitedLayer` + `InflationLayer`

**Global costmap** (full map, 0.06 m resolution):
- `StaticLayer` + `ObstacleLayer` + `ProhibitedLayer` + `InflationLayer`

`ProhibitedLayer` is a custom plugin (`robot_custom_layers`) that marks user-defined forbidden zones with lethal cost.

### Collision Monitor

An additional **collision monitor** sits between the planner and the hardware, subscribing to `cmd_vel_smoothed` and publishing `cmd_vel`. It uses a footprint-based **approach** polygon with `time_before_collision: 1.2 s` to stop the robot before contact.

### Behavior Trees

Custom BT nodes registered with `bt_navigator`:

| Node | Purpose |
|---|---|
| `battery_check_action_bt_node` | Check battery level before/during navigation |
| `get_charge_station_pose_action_bt_node` | Retrieve docking station pose |
| `emergency_stop_action_bt_node` | Trigger immediate stop |

Default BT: `navigate_w_replanning_simple.xml`

---

## Launch

```bash
# Full robot bringup (hardware + localization + navigation)
ros2 launch robot_bringup robot_bringup.launch.py

# Navigation only (assumes localization is already running)
ros2 launch robot_bringup robot_navigation.launch.py

# SLAM with slam_toolbox
ros2 launch robot_bringup robot_slam.launch.py

# SLAM with Cartographer
ros2 launch robot_bringup robot_cartographer.launch.py

# Gazebo simulation
ros2 launch robot_gazebo <world_launch_file>
```

---

## Dependencies

- ROS 2 Humble (or later)
- Nav2
- GTSAM (for FGO localization)
- PCL (for ICP scan matching)
- BehaviorTree.CPP v4
- SICK scan_unified driver (for `/sick_lidar0` and `/sick_lidar1`)

Install Python dependencies:
```bash
pip install -r requirements.txt
```
