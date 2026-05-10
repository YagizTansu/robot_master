#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# entrypoint_fgo.sh
# Container: fgo_sim  |  ROS_DOMAIN_ID=2
#
# Launch order:
#   1. robot_bringup_sim.launch.py  — Gazebo + robot state publisher
#   2. robot_navigation.launch.py   — FGO + Map Server + Nav2 + RViz + Database
#                                     (enable_vda5050:=false — MQTT not needed in sim)
#   3. localization_benchmark       — FGO benchmark dashboard
#
# Benchmark results → /ros2_ws/benchmark_results/
# ─────────────────────────────────────────────────────────────────────────────
set -e

# ── Source ROS and workspace ──────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# ── Ensure benchmark results directory exists ─────────────────────────────────
mkdir -p "${BENCHMARK_RESULTS_DIR:-/ros2_ws/benchmark_results}"

# ── X11 display safety check ──────────────────────────────────────────────────
export DISPLAY="${DISPLAY:-:0}"

echo "======================================================"
echo " fgo_sim container starting"
echo " ROS_DOMAIN_ID = ${ROS_DOMAIN_ID}"
echo " DISPLAY       = ${DISPLAY}"
echo " Results dir   = ${BENCHMARK_RESULTS_DIR}"
echo "======================================================"

# ── 1. Simulation bringup (Gazebo + robot) ────────────────────────────────────
echo "[1/3] Launching Gazebo + robot bringup ..."
ros2 launch robot_bringup robot_bringup_sim.launch.py &
BRINGUP_PID=$!

# Wait for Gazebo clock
echo "      Waiting for /clock ..."
until ros2 topic hz /clock --window 5 2>/dev/null | grep -q "average rate"; do
    sleep 3
done
echo "      Gazebo ready."
sleep 10 # Extra pause to ensure all sim nodes are fully up
# ── 2. Full navigation stack (FGO + Map Server + Nav2 + RViz + Database) ──────
echo "[2/3] Launching robot_navigation (FGO + Nav2 + RViz) ..."
ros2 launch robot_bringup robot_navigation.launch.py \
    enable_vda5050:=false &
NAV_PID=$!

# # Brief pause so FGO and map server can initialise before benchmark starts
# sleep 8

# # ── 3. Benchmark node (ros2 run ile) ──────────────────────────────────────────
# echo "[3/3] Launching FGO benchmark (ros2 run) ..."
# ros2 run robot_gazebo localization_benchmark.py \
#     --ros-args \
#     -p use_sim_time:=true \
#     -p estimated_topic:=/fgo/odometry \
#     -p algorithm_name:=FGO \
#     -p csv_output_dir:="${BENCHMARK_RESULTS_DIR:-/ros2_ws/benchmark_results}" &
# BENCH_PID=$!

echo "======================================================"
echo " All nodes started.  PIDs:"
echo "   bringup   : ${BRINGUP_PID}"
echo "   navigation: ${NAV_PID}"
echo "   benchmark : ${BENCH_PID}"
echo " Ctrl-C to stop all."
echo "======================================================"

# ── Wait for any child process to exit, then kill the rest ───────────────────
wait -n 2>/dev/null || true
echo "A process exited — shutting down fgo_sim ..."
kill ${BRINGUP_PID} ${NAV_PID} 2> /dev/null || true
wait
