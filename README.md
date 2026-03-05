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

### `factor_graph_optimization`

The primary localization method is a custom **Factor Graph Optimization (FGO)** node built on **GTSAM** with incremental solving via **iSAM2**.

#### How It Works

The node maintains a pose graph where each odometry keyframe creates a new node. Three state variables are maintained per node: pose **X**, velocity **V**, and IMU bias **B**. Sensor measurements are fused as probabilistic factors:

| Factor | Type | Topic | States constrained |
|---|---|---|---|
| Odometry | `BetweenFactor<Pose3>` (graph backbone) | `/odometry` | X(k) → X(k+1) |
| IMU | `CombinedImuFactor` (pre-integrated) | `/imu` | X, V, B at both keyframe ends |
| LiDAR (NDT scan match) | `PriorFactor<Pose3>` | `/scan_match_pose` | X(nearest keyframe) |

#### Keyframe Strategy

A new graph node is created when the robot moves more than **0.02 m** (translation) **or** rotates more than **0.02 rad** since the last keyframe. The optimization timer runs at **50 Hz** and processes all keyframes accumulated since the last cycle.

#### Scan Matching

LiDAR matching is handled by a separate `scan_matcher_node` using **NDT (Normal Distributions Transform)**. The scan arrives pre-transformed into `base_footprint` frame. The resulting pose is published to `/scan_match_pose` with the NDT fitness score stored in `covariance[34]`.

Scan priors applied to the graph include two quality filters:

1. **Age gate** — poses older than `max_scan_age_sec` (default 1.0 s) are discarded to prevent stale scans from polluting the graph after a pause.
2. **Fitness gate** — poses with fitness score above `icp_fitness_score_threshold` (default 0.5) are discarded (higher score = worse match).
3. **Rotation gate** — if the batch contains any rotation keyframe (`batch_dyaw > rotation_gate_rad`, default 0.02 rad), all scan priors in that batch are skipped. NDT yaw is unreliable during in-place rotation.
4. **Adaptive noise** — accepted scan priors use `σ *= (1 + fitness_noise_scale × fitness_score)`, so noisier matches contribute a weaker constraint.

Key NDT parameters (in `fgo_params.yaml`):

```yaml
scan_matcher:
  type:                    "NDT"
  max_iterations:          50
  max_correspondence_dist: 1.0       # meters
  transformation_epsilon:  1.0e-6
noise.lidar:
  icp_fitness_score_threshold: 0.5   # discard if score > this
  max_scan_age_sec:            1.0   # discard if older than this (sec)
  rotation_gate_rad:           0.02  # skip if batch |dyaw| > this (rad)
  fitness_noise_scale:         5.0   # adaptive noise multiplier
```

#### IMU Pre-integration

IMU samples are buffered and pre-integrated per keyframe interval using GTSAM's `CombinedImuFactor`. This factor jointly optimizes pose, velocity, and accelerometer/gyroscope bias. If the IMU sensor frame differs from `base_footprint`, the node performs a one-time static TF lookup at startup and rotates all measurements into the robot body frame automatically.

#### TF Tree

```
map
 └── odom            (published by FGO node, re-published every cycle at 50 Hz)
      └── base_footprint   (raw odometry passthrough, published every odom message)
```

#### Configuration: `fgo_params.yaml`

```yaml
# Enable/disable sensors individually
sensors:
  enable_odom:  true
  enable_imu:   true
  enable_lidar: true

# Frame IDs
frames:
  map_frame:   "map"
  odom_frame:  "odom"
  base_frame:  "base_footprint"
  imu_frame:   "base_footprint"   # change if IMU has a different TF frame

# iSAM2 solver
isam2:
  relinearize_threshold: 0.1
  relinearize_skip:      1
  optimization_rate_hz:  50.0

# Keyframe thresholds
keyframe:
  translation_threshold: 0.02   # meters
  rotation_threshold:    0.02   # radians
```

#### Initialization

- On startup the node initializes the graph at the pose defined by `initial_pose` in `fgo_params.yaml` (default: map origin `(0, 0, 0)`).
- Sending a **2D Pose Estimate** in RViz resets the graph to the new pose.

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

# Navigation
ros2 launch robot_bringup robot_navigation.launch.py
```

---

## Dependencies

- ROS 2 Jazzy
- Nav2
- GTSAM (for FGO localization)
- PCL (for ICP scan matching)
- BehaviorTree.CPP
- SICK scan_unified driver (for `/sick_lidar0` and `/sick_lidar1`)

Install Python dependencies:
```bash
pip install -r requirements.txt
```
