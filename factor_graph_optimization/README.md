<div align="center">

# 📍 Factor Graph Optimization (FGO) Localization

**A ROS 2 sensor-fusion localization system combining Wheel Odometry, IMU, and LiDAR scan matching inside an incremental GTSAM factor graph.**

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue?style=flat-square&logo=ros)](https://docs.ros.org/en/jazzy/)
[![GTSAM](https://img.shields.io/badge/GTSAM-4.x-orange?style=flat-square)](https://gtsam.org/)
[![C++](https://img.shields.io/badge/C++-17-green?style=flat-square&logo=cplusplus)](https://en.cppreference.com/)
[![PCL](https://img.shields.io/badge/PCL-NDT%2FICP-purple?style=flat-square)](https://pointclouds.org/)

</div>

---

## 📑 Table of Contents

1. [What does this package do?](#1-what-does-this-package-do)
2. [Why Factor Graphs?](#2-why-factor-graphs)
3. [System Architecture](#3-system-architecture)
4. [The Two ROS 2 Nodes](#4-the-two-ros-2-nodes)
5. [Data Flow — Step by Step](#5-data-flow--step-by-step)
6. [Factor Graph Deep Dive](#6-factor-graph-deep-dive)
7. [IMU Preintegration](#7-imu-preintegration)
8. [LiDAR Scan Matching (NDT / ICP)](#8-lidar-scan-matching-ndt--icp)
9. [iSAM2 — Incremental Solver](#9-isam2--incremental-solver)
10. [TF Frame Tree](#10-tf-frame-tree)
11. [Topics — Full Reference](#11-topics--full-reference)
12. [Parameter Reference](#12-parameter-reference)
13. [Launch & Build](#13-launch--build)
14. [Tuning Guide](#14-tuning-guide)
15. [Benchmark Results](#15-benchmark-results)
16. [Troubleshooting](#16-troubleshooting)

---

## 1. What does this package do?

This package replaces both **AMCL** (map→odom localization) and **robot_localization EKF** (odom→base_footprint) with a single, unified **Factor Graph Optimization** pipeline.

```
📦 Traditional ROS 2 Nav2 Stack
   AMCL         →  publishes map→odom TF
   robot_localization EKF  →  fuses odom + IMU, publishes odom→base TF

📦 This Package
   FGO Node     →  does BOTH of the above, using full probabilistic batch optimization
```

> **Key insight:** EKF fuses sensors *sequentially* and cannot revise past estimates.
> A factor graph *jointly* optimizes all sensor measurements over a sliding time window,
> so a good LiDAR fix can pull earlier poses into alignment — something an EKF can never do.

---

## 2. Why Factor Graphs?

A **factor graph** is a mathematical structure that encodes the relationships between variables (robot poses, velocities, IMU biases) and measurements (odometry, IMU, LiDAR).

```
 Variables:    X₀ ── X₁ ── X₂ ── X₃ ── ...  (robot poses)
               V₀    V₁    V₂    V₃          (velocities)
               B₀    B₁    B₂    B₃          (IMU biases)

 Factors (measurements that constrain variables):
   ■ PriorFactor(X₀)         ← initial pose
   ■ BetweenFactor(Xᵢ, Xᵢ₊₁) ← odometry delta
   ■ CombinedImuFactor(...)   ← IMU preintegration
   ■ PriorFactor(Xⱼ)         ← LiDAR scan match result
```

The solver finds the **maximum a posteriori (MAP)** estimate — the set of variable values that best explains all measurements simultaneously.

$$\hat{X} = \underset{X}{\arg\min} \sum_{i} \| h_i(X) - z_i \|^2_{\Sigma_i}$$

Where $h_i$ is the factor's measurement model, $z_i$ is the actual measurement, and $\Sigma_i$ is the noise covariance.

---

## 3. System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           ROBOT HARDWARE / GAZEBO                           │
│                                                                             │
│   /imu (100 Hz)        /odometry (50 Hz)         /scan (10 Hz)             │
│       │                      │                       │                      │
└───────┼──────────────────────┼───────────────────────┼──────────────────────┘
        │                      │                       │
        ▼                      ▼                       ▼
┌───────────────────────────────────┐   ┌──────────────────────────────────┐
│         fgo_node                  │   │       scan_matcher_node          │
│                                   │   │                                  │
│  ┌──────────┐  ┌──────────┐       │   │  /map ──► MapBuilder             │
│  │ imu_buf_ │  │ odom_buf_│       │   │  /scan ──► LaserProjection       │
│  └────┬─────┘  └────┬─────┘       │   │                │                 │
│       │             │             │   │          VoxelGrid filter        │
│       │    ┌────────┘             │   │                │                 │
│       │    ▼                      │   │      NDT / ICP  match()          │
│       │  KeyframeSelector         │   │          [async thread]          │
│       │    │ keyframe?            │   │                │                 │
│       │    ▼                      │   │       fitness score gate         │
│       │  scan_buf_ ◄──────────────┼───┤◄── /scan_match_pose             │
│       │             /scan_match_  │   │    /scan_match/fitness_score     │
│       │              pose         │   └──────────────────────────────────┘
│       │                           │
│       ▼   (every 1/50 s)          │
│  ┌─────────────────────────┐      │
│  │   optimizationStep()   │      │
│  │                         │      │
│  │  1. Drain all 3 buffers │      │
│  │  2. BetweenFactors(odom)│      │
│  │  3. CombinedImuFactors  │      │
│  │  4. PriorFactors(scan)  │      │
│  │  5. iSAM2.update()      │      │
│  │  6. Extract estimates   │      │
│  └────────────┬────────────┘      │
│               │                   │
│    ┌──────────┴──────────┐        │
│    ▼                     ▼        │
│  map→odom TF         /fgo/odometry│
│  odom→base TF        /fgo/path    │
└───────────────────────────────────┘
```

---

## 4. The Two ROS 2 Nodes

### `fgo_node` — The Brain

| Aspect | Detail |
|--------|--------|
| **Binary** | `src/fgo_main.cpp` → `fgo_node` executable |
| **Class** | `FgoNode` in `include/factor_graph_optimization/fgo_node.hpp` |
| **Executor** | `rclcpp::spin()` — SingleThreadedExecutor |
| **Core object** | `GraphManager` owns iSAM2, factor graph, IMU preintegrator |
| **Key mechanism** | Wall timer fires at `optimization_rate_hz` (default 50 Hz) |

**FgoNode owns three thread-safe sensor buffers:**

```cpp
SensorBuffer<OdomSample>                                    odom_buf_;
SensorBuffer<ImuSample>                                     imu_buf_;
SensorBuffer<geometry_msgs::msg::PoseWithCovarianceStamped> scan_buf_;
```

Sensor callbacks (fast, non-blocking) push samples in. The optimization timer drains them all at once and feeds them to iSAM2.

---

### `scan_matcher_node` — The Eyes

| Aspect | Detail |
|--------|--------|
| **Binary** | `src/scan_matcher_main.cpp` → `scan_matcher_node` executable |
| **Class** | `ScanMatcherNode` in `include/factor_graph_optimization/scan_matcher.hpp` |
| **Executor** | `rclcpp::spin()` — SingleThreadedExecutor |
| **Algorithm** | NDT or ICP (selectable via YAML) |
| **Threading** | `match()` is offloaded to `std::thread` — executor never blocked |

**Why a separate node?**
- LiDAR matching can crash on malformed scans. Separation means `fgo_node` continues operating with odom+IMU even if `scan_matcher_node` dies.
- Both nodes have `respawn=True` in the launch file — automatic recovery.
- Clean decoupling: their only communication is via the `/scan_match_pose` topic.

---

## 5. Data Flow — Step by Step

### Phase 1 — Sensor Collection (real-time callbacks)

```
/imu message arrives (100 Hz)
  └─► imuCallback()
        ├─ rotate measurements from imu_frame → base_frame  (R_imu_to_base_)
        └─ push ImuSample into imu_buf_  (capped at max_pending_imu=2000)

/odometry message arrives (50 Hz)
  └─► odomCallback()
        ├─ publish odom→base_footprint TF  (raw, no correction)
        ├─ KeyframeSelector::checkAndUpdate()
        │     triggers if: Δpos > 2cm  OR  Δyaw > 0.02rad  OR  Δt > 2s
        └─ push OdomSample into odom_buf_  (capped at max_pending_odom=500)

/scan_match_pose message arrives
  └─► scanPoseCallback()
        ├─ age gate: discard if older than max_scan_age_sec=1.0s
        └─ push PoseWithCovarianceStamped into scan_buf_  (capped at 10)
```

### Phase 2 — Scan Matching (async, in scan_matcher_node)

```
/scan message arrives (10 Hz)
  └─► scanCallback()
        ├─ [lock map_mutex_]  snapshot current_map pointer
        ├─ [lock fgo_pose_mutex_]  check has_fgo_pose_
        ├─ LaserProjection: LaserScan → PointCloud2 → PCL
        ├─ VoxelGrid downsample (scan_voxel_leaf_size=0.05m)
        ├─ buildInitialGuess()  →  Eigen::Matrix4f from current FGO pose
        ├─ matching_in_progress_.exchange(true)  →  skip if busy
        └─ std::thread{
               match(source, map, guess, result)   ← NDT/ICP, 5-50ms
               publish fitness_score
               if fitness < threshold:
                   publish PoseWithCovarianceStamped on /scan_match_pose
               matching_in_progress_ = false
           }.detach()
```

### Phase 3 — Factor Graph Optimization (50 Hz timer callback)

```
optimizationStep() fires (every 20ms)
  │
  ├─ Drain odom_buf_ → local_odom
  ├─ Drain imu_buf_  → local_imu
  ├─ Drain scan_buf_ → local_scan
  │
  ├─ [lock graph_mutex_]
  │
  ├─ if local_odom.empty():
  │     republish cached map→odom TF (prevents Nav2 stale TF timeout)
  │     return
  │
  └─ GraphManager::step(local_odom, local_imu, local_scan)
         │
         ├─ For each odom keyframe i:
         │     compute delta Pose3 from previous keyframe
         │     add BetweenFactor<Pose3>(X(i), X(i+1), delta, odom_noise)
         │     insert X(i+1) initial value
         │
         ├─ ImuPreintegrator::addFactors(...)
         │     For each keyframe interval [from_key, to_key]:
         │       if IMU samples span the interval:
         │           integrate measurements → CombinedImuFactor
         │       else:
         │           PriorFactor on V and B (fallback)
         │
         ├─ For each scan match:
         │     find nearest keyframe by timestamp
         │     check rotation gate (|dyaw| < 0.015 rad)
         │     add PriorFactor<Pose3>(X(j), scan_pose, scan_noise)
         │
         ├─ isam2_->update(new_factors_, new_values_)
         │
         ├─ Extract: optimized_pose_, optimized_velocity_, optimized_bias_
         │
         ├─ trimOldestKeys()  (if graph_max_size=2000 exceeded)
         │
         └─ Publish:
               map→odom TF  (stamped with sensor time of last odom)
               /fgo/odometry  (optimized pose, sensor-time stamp)
               /fgo/path  (history, capped at max_path_length=10000)
```

---

## 6. Factor Graph Deep Dive

### Variable Keys

GTSAM uses **Symbol** keys to identify variables. The shorthand naming convention is:

```cpp
using gtsam::symbol_shorthand::X;  // X(k) = Pose3    of keyframe k
using gtsam::symbol_shorthand::V;  // V(k) = Vector3  velocity of keyframe k
using gtsam::symbol_shorthand::B;  // B(k) = ConstantBias  IMU bias of keyframe k
```

At startup: `X(0)`, `V(0)`, `B(0)` are created with tight `PriorFactor` constraints.

### Factor Types

```
┌───────────────────────────────┬────────────────────────────────────────────┐
│ Factor                        │ What it represents                         │
├───────────────────────────────┼────────────────────────────────────────────┤
│ PriorFactor<Pose3>(X(0))      │ Initial pose from YAML / /initialpose      │
│ PriorFactor<Vector3>(V(0))    │ Initial velocity (approx. at rest)         │
│ PriorFactor<ConstantBias>(B(0))│ Initial IMU bias uncertainty              │
├───────────────────────────────┼────────────────────────────────────────────┤
│ BetweenFactor<Pose3>          │ Relative pose change from wheel odometry   │
│  (X(k), X(k+1), Δpose)        │ Noise: diagonal σ = [0.05, 0.05, 0.01,    │
│                               │        0.01, 0.01, 0.05] meters/radians   │
├───────────────────────────────┼────────────────────────────────────────────┤
│ CombinedImuFactor             │ IMU preintegration over [t_k, t_{k+1}]    │
│  (X(k),V(k), X(k+1),V(k+1),  │ Encodes gravity, bias drift, acceleration │
│   B(k), B(k+1), preint)       │                                            │
├───────────────────────────────┼────────────────────────────────────────────┤
│ PriorFactor<Pose3>(X(j))      │ Absolute pose from NDT/ICP scan match     │
│                               │ Noise: adaptive σ ∝ fitness score         │
└───────────────────────────────┴────────────────────────────────────────────┘
```

### Noise Model — Adaptive LiDAR Covariance

The scan matcher doesn't use fixed noise. It scales covariance based on match quality:

```
σ_effective = σ_base × (1 + fitness_noise_scale × fitness_score)

With fitness_noise_scale=5.0:
  fitness=0.01 → σ × 1.05   (excellent match, high trust)
  fitness=0.40 → σ × 3.0    (poor match, low trust)
  fitness>0.50 → REJECTED    (not added to graph at all)
```

This means a blurry, uncertain scan match still goes into the graph — but with large covariance, so iSAM2 trusts it less.

### Sliding Window Marginalization

The graph can grow forever. To prevent memory unbounded growth:

```yaml
graph_max_size: 2000
```

When `key_` exceeds 2000, `trimOldestKeys()` calls:
```cpp
isam2_->marginalizeLeaves(keys_to_marginalize, &marginal_factors);
```

This converts the oldest `X(k), V(k), B(k)` into **marginal factors** on their neighbors, preserving all the information they carried without keeping the full variables in memory.

---

## 7. IMU Preintegration

Raw IMU runs at 100 Hz. Between two consecutive keyframes (which might be 0.1–2 s apart), there could be 10–200 IMU samples. We can't add 200 factors — we preintegrate them into **one** `CombinedImuFactor`.

### Preintegration Mathematics

Between keyframes at times $t_k$ and $t_{k+1}$, the preintegrated relative motion (PIM) is:

$$\Delta R_{k,k+1} = \prod_{i \in [k,k+1]} \text{Exp}\left((\tilde{\omega}_i - b_i^g) \Delta t\right)$$

$$\Delta v_{k,k+1} = \sum_{i} \Delta R_{k,i} \cdot (\tilde{a}_i - b_i^a) \Delta t$$

$$\Delta p_{k,k+1} = \sum_{i} \left[ \Delta v_{k,i} \Delta t + \frac{1}{2} \Delta R_{k,i} \cdot (\tilde{a}_i - b_i^a) \Delta t^2 \right]$$

Where $\tilde{\omega}$ and $\tilde{a}$ are raw gyro/accel measurements, $b^g$ and $b^a$ are the bias estimates.

The `CombinedImuFactor` in GTSAM encodes this as a single 15-DOF factor connecting pose+velocity+bias at both endpoints.

### Noise Parameters

```yaml
imu:
  accel_sigma:        0.02     # white noise   — how noisy each raw sample is
  gyro_sigma:         0.002    # white noise
  accel_bias_sigma:   0.0001   # bias drift    — how much bias changes per second
  gyro_bias_sigma:    0.0001   # bias drift
  integration_sigma:  0.0001   # discretisation error
  gravity:            9.81
```

> **Rule of thumb:** `accel_sigma` should match your IMU datasheet's "noise density". Too small → IMU overconfident → ignores odom. Too large → IMU ignored entirely.

### Fallback When IMU is Missing

If no IMU samples span a keyframe interval (sensor dropout, IMU disabled), the system adds **fallback priors** instead:

```cpp
PriorFactor<Vector3>(V(to_key), v_seed, Isotropic::Sigma(3, 0.5))
PriorFactor<ConstantBias>(B(to_key), b_seed, Diagonal::Sigmas(...))
```

This keeps every `V(k)` and `B(k)` node constrained so iSAM2 doesn't encounter a singular information matrix.

### Partial Coverage Warning

If IMU data covers less than 80% of a keyframe interval, the system warns:
```
[ImuPreintegrator] Partial IMU coverage 65% (0.130s of 0.200s expected) for interval [12->13].
```

---

## 8. LiDAR Scan Matching (NDT / ICP)

### NDT — Normal Distributions Transform

NDT divides the **reference map** into a voxel grid. Each occupied voxel stores the **mean** and **covariance** of all points inside it (a Gaussian blob). The source scan is then aligned by maximizing the likelihood of each source point falling inside the correct Gaussian.

```
Map cloud (0.05m voxels)          Source scan (0.05m voxels)
┌──────────────────────────┐      (from current sensor reading)
│  Voxel [i,j]:             │
│    mean  = (x̄, ȳ, z̄)     │  NDT finds transform T* that minimizes:
│    cov   = Σ             │
│                           │   Σ  -exp( -½ (T·pₛ - μᵥ)ᵀ Σᵥ⁻¹ (T·pₛ - μᵥ) )
└──────────────────────────┘
```

**Key tuning parameters:**
```yaml
ndt_resolution:  0.30    # voxel cell size for the NDT grid
                         # Rule: ~6× map_voxel_leaf_size (6 × 0.05 = 0.30)
                         # Too small → cells empty, matching fails
                         # Too large → walls merge, gradient blurry

ndt_step_size:   0.10    # gradient descent step size
                         # Too large → overshoots local minimum
                         # Too small → slow convergence, needs more iterations

max_iterations:  80      # increase if robot faces ambiguous environments
```

### ICP — Iterative Closest Point

ICP iteratively:
1. Finds the nearest neighbor in the map for each source point
2. Computes the optimal rigid transform that minimizes mean-squared distance
3. Applies the transform and repeats

ICP is simpler but less robust to initial guess errors than NDT.

### Async Architecture — Zero Executor Blocking

NDT/ICP takes 5–50 ms per scan. If this ran synchronously in the callback, IMU messages would queue behind it.

**Solution:** `scanCallback` does only the fast work (conversion, downsampling, initial guess), then immediately launches a `std::thread` and returns:

```cpp
std::thread([this, src, map, guess, stamp]() mutable {
    double fitness = matcher_->match(src, map, guess, result);
    pub_fitness_score_->publish(score);        // always publish
    if (fitness < threshold)
        pub_scan_pose_->publish(result);       // conditionally publish
    matching_in_progress_.store(false);        // release slot
}).detach();
```

`std::atomic<bool> matching_in_progress_` prevents a second match from launching while the first is still running.

### Fitness Score — Match Quality Monitor

Every scan match publishes a quality score on `/scan_match/fitness_score` (`std_msgs/Float64`).

```
Low  score (< 0.1) → excellent match    → tight noise covariance → graph trusts it
Mid  score (0.1–0.5) → acceptable match → inflated noise covariance
High score (> 0.5) → rejected           → NOT published to /scan_match_pose
```

Monitor it live:
```bash
ros2 topic echo /scan_match/fitness_score
```

### Rotation Gate

NDT is unreliable during in-place rotation (features blur between scan frames). The FGO node rejects any scan match associated with a keyframe where `|Δyaw| > 0.015 rad`:

```yaml
lidar.gating.rotation_gate_rad: 0.015  # ~0.86 degrees
```

---

## 9. iSAM2 — Incremental Solver

**iSAM2** (Incremental Smoothing and Mapping 2) is the optimizer underneath. Rather than re-solving the entire graph from scratch on every tick, it:

1. Identifies which variables changed (affected by new factors)
2. **Re-linearizes only those variables** using the Bayes tree structure
3. Does a partial back-substitution

The result is near-constant-time updates even as the graph grows.

### iSAM2 Configuration

```yaml
isam2:
  relinearize_threshold: 0.1   # re-linearize a variable if its delta exceeds this
  relinearize_skip:      1     # check every update (1 = no skipping)
  factorization:         "CHOLESKY"   # CHOLESKY (fast) or QR (more stable)
```

> Use `factorization: "QR"` if you see iSAM2 throwing near-singular-matrix warnings.  
> QR is ~30% slower but numerically more robust.

### Error Recovery

If iSAM2 throws an exception (e.g., due to a catastrophically bad scan match), `GraphManager::step()` returns `false`, rolls back `key_`, and `FgoNode` skips publishing that tick. The next tick re-tries with fresh sensor data.

---

## 10. TF Frame Tree

```
map
 └─► odom          (published by fgo_node, updated every optimization step)
       └─► base_footprint   (published by fgo_node via odomCallback, raw odometry)
                └─► imu_link       (from URDF / robot_state_publisher)
                └─► base_laser_link (from URDF)
                └─► kinect_v2      (from URDF)
```

### map → odom derivation

```
T(map→odom) = T(map→base)_optimized  ×  T(odom→base)_keyframe_anchor⁻¹
```

The **keyframe anchor** is critical: it's the raw odometry pose at the moment of the last optimization. This ensures that `base_footprint` dead-reckons correctly between optimization ticks via the `odom→base_footprint` chain.

### Timestamp Policy

| TF | Stamp | Reason |
|----|-------|--------|
| `map→odom` (new result) | `lastConsumedOdomStamp()` | Matches sensor epoch of the iSAM2 result |
| `map→odom` (re-publish, no new keyframe) | `now()` | Prevents Nav2 stale-TF timeout |
| `odom→base_footprint` | `msg->header.stamp` | Raw sensor timestamp, unmodified |

---

## 11. Topics — Full Reference

### fgo_node Subscriptions

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/odometry` | `nav_msgs/Odometry` | 50 Hz | Wheel odometry |
| `/imu` | `sensor_msgs/Imu` | 100 Hz | IMU data (optional) |
| `/scan_match_pose` | `geometry_msgs/PoseWithCovarianceStamped` | ~10 Hz | LiDAR match result |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | on-demand | RViz pose reset |

### fgo_node Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/fgo/odometry` | `nav_msgs/Odometry` | Optimized pose, sensor-time stamped |
| `/fgo/path` | `nav_msgs/Path` | Full trajectory history (capped at 10 000 poses) |
| TF `map→odom` | — | Main localization output for Nav2 |
| TF `odom→base_footprint` | — | Raw odometry integration |

### scan_matcher_node Subscriptions

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | transient_local | 2D occupancy map |
| `/scan` | `sensor_msgs/LaserScan` | default | Live laser scans |
| `/fgo/odometry` | `nav_msgs/Odometry` | default | Initial guess for NDT/ICP |

### scan_matcher_node Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/scan_match_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Accepted match poses with adaptive covariance |
| `/scan_match/fitness_score` | `std_msgs/Float64` | Raw quality score for every attempt |

---

## 12. Parameter Reference

### fgo_node Parameters

#### Sensor Toggles

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sensors.enable_odom` | `true` | Enable wheel odometry |
| `sensors.enable_imu` | `true` | Enable IMU preintegration |
| `sensors.enable_lidar` | `true` | Enable scan-match corrections |

#### Keyframe Strategy

| Parameter | Default | Description |
|-----------|---------|-------------|
| `keyframe.translation_threshold` | `0.02 m` | Min translation to create a keyframe |
| `keyframe.rotation_threshold` | `0.02 rad` | Min rotation to create a keyframe |
| `keyframe.max_time_sec` | `2.0 s` | Force keyframe even if robot is stationary |

> **Why keyframes?** Adding every 50 Hz odom sample to the graph would create 3000 variables per minute. Keyframes reduce this to ~10–50 per minute, keeping iSAM2 fast without losing accuracy.

#### Odometry Noise

```yaml
noise.odometry:
  x:     0.05   # σ in metres — how much you trust the wheel encoder in X
  y:     0.05
  z:     0.01   # 2D robot: Z is constrained
  roll:  0.01   # 2D robot: roll is constrained
  pitch: 0.01
  yaw:   0.05
```

#### iSAM2

| Parameter | Default | Description |
|-----------|---------|-------------|
| `isam2.relinearize_threshold` | `0.1` | Delta norm threshold for re-linearization |
| `isam2.relinearize_skip` | `1` | Check every N updates |
| `isam2.factorization` | `"CHOLESKY"` | `"CHOLESKY"` or `"QR"` |

#### Buffer Caps

| Parameter | Default | Description |
|-----------|---------|-------------|
| `node.max_pending_imu` | `2000` | ~20s of 100 Hz IMU at max |
| `node.max_pending_odom` | `500` | Keyframe buffer |
| `node.max_pending_scans` | `10` | Scan match buffer |
| `node.graph_max_size` | `2000` | Marginalize oldest keys above this |
| `node.max_path_length` | `10000` | Cap `/fgo/path` pose history |

---

### scan_matcher_node Parameters

#### NDT Tuning

| Parameter | Default | Description |
|-----------|---------|-------------|
| `scan_matcher.ndt_resolution` | `0.30 m` | NDT voxel cell size. Rule: ~6× `map_voxel_leaf_size` |
| `scan_matcher.ndt_step_size` | `0.10 m` | Gradient descent step. Smaller = more stable |
| `scan_matcher.max_iterations` | `80` | Max NDT iterations |
| `scan_matcher.fitness_score_threshold` | `0.5` | Reject matches above this score |
| `scan_matcher.fitness_noise_scale` | `5.0` | Covariance inflation factor |

#### Voxel Downsampling

| Parameter | Default | Description |
|-----------|---------|-------------|
| `scan_matcher.map_voxel_leaf_size` | `0.05 m` | Map cloud downsampling |
| `scan_matcher.scan_voxel_leaf_size` | `0.05 m` | Scan cloud downsampling (match map density) |

> **Critical relationship:** `ndt_resolution` ÷ `map_voxel_leaf_size` should be 5–8.  
> Too small a ratio → empty NDT cells → matching fails.

---

## 13. Launch & Build

### Build

```bash
cd ~/ros2_ws
colcon build --packages-select factor_graph_optimization \
             --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Launch (Standalone)

```bash
# Simulation (Gazebo)
ros2 launch factor_graph_optimization fgo.launch.py use_sim_time:=true

# Real Robot
ros2 launch factor_graph_optimization fgo.launch.py use_sim_time:=false
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `true` | Use `/clock` topic (Gazebo). Set `false` on real robot |

> ⚠️ **Warning:** Setting `use_sim_time:=false` on a Gazebo-based system causes `TF_OLD_DATA` warnings because sensor timestamps carry sim time while `now()` returns wall clock. Always match this flag to your setup.

### Dependencies

```xml
<depend>rclcpp</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
<depend>sensor_msgs</depend>
<depend>std_msgs</depend>
<depend>tf2</depend>
<depend>tf2_ros</depend>
<depend>tf2_geometry_msgs</depend>
<depend>laser_geometry</depend>
<depend>pcl_ros</depend>
<depend>pcl_conversions</depend>
<!-- GTSAM must be installed separately -->
```

Install GTSAM:
```bash
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

---

## 14. Tuning Guide

### I want better accuracy

1. **Reduce voxel sizes** (start from 0.05 m, try 0.03 m):
   ```yaml
   map_voxel_leaf_size:  0.03
   scan_voxel_leaf_size: 0.03
   ndt_resolution:       0.18   # keep ~6× map_voxel_leaf_size
   ```

2. **Reduce odometry noise** if your encoders are high-quality:
   ```yaml
   noise.odometry.x:   0.02
   noise.odometry.yaw: 0.02
   ```

3. **Trust LiDAR more** (reduce lidar noise):
   ```yaml
   noise.lidar.x:   0.05
   noise.lidar.yaw: 0.05
   ```

### ATE RMSE diverges over time

- NDT initial guess probably failing in featureless corridors.
- Try `ndt_resolution: 0.5` (larger capture basin) + higher `max_iterations: 100`.
- Or switch to ICP:
  ```yaml
  scan_matcher.type: "ICP"
  ```

### High CPU usage

1. Reduce optimization rate: `optimization_rate_hz: 20.0`
2. Reduce NDT iterations: `max_iterations: 40`
3. Increase voxel sizes: `map_voxel_leaf_size: 0.10`
4. Increase keyframe thresholds: `translation_threshold: 0.05`

### "Waiting for initial FGO pose" in logs

`scan_matcher_node` starts before `fgo_node` publishes its first `/fgo/odometry`. Normal at startup — disappears within 1–2 seconds. If it persists, check that `sensors.enable_odom: true`.

### iSAM2 exception in logs

Usually caused by a catastrophically wrong scan match creating a conflicting factor. The system auto-recovers by rolling back the key counter. If frequent:
- Lower `fitness_score_threshold` to 0.3 (stricter gate)
- Switch factorization to `"QR"` for better numerical stability

---

## 15. Benchmark Results

Performance measured via `localization_benchmark.py` against ground truth in Gazebo:

| Metric | Value (n≈500) | Notes |
|--------|---------------|-------|
| **Position Error** | ~0.19 m | Instantaneous |
| **ATE RMSE** | ~0.21 m | Accumulated Trajectory Error |
| **Yaw Error** | < 1.0° | Consistently excellent |
| **Trend** | ↓ decreasing | System still converging at n=500 |

> ATE RMSE decreases as the graph accumulates more scan match corrections — the longer the robot runs, the better it localizes.

---

## 16. Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `TF_OLD_DATA` for `odom` frame | `use_sim_time` mismatch | Set `use_sim_time:=true` for Gazebo |
| `Waiting for map...` | `/map` not published yet | Ensure map server is running before scan_matcher_node |
| `Scan match discarded: fitness=X.XX` | Poor match quality | Tune `ndt_resolution`, check map quality |
| Nav2 can't find a path | `map→odom` TF not being published | Check that `tf.publish_map_to_odom: true` |
| Robot pose jumps on startup | Initial pose wrong | Set `initial_pose` in YAML or use RViz `2D Pose Estimate` |
| High ATE that diverges | Scan matcher failing | Monitor `/scan_match/fitness_score`; tune NDT params |
| `[ImuPreintegrator] Partial IMU coverage` | IMU lagging behind keyframes | Increase `max_pending_imu` or check IMU publish rate |
| Memory grows unboundedly | `graph_max_size: 0` | Set to a finite value (default 2000) |

---

<div align="center">

**Built with [GTSAM](https://gtsam.org/) · [PCL](https://pointclouds.org/) · [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)**

</div>
