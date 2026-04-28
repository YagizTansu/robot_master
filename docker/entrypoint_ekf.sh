#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# entrypoint_ekf.sh
# Container: ekf_sim  |  ROS_DOMAIN_ID=1
#
# Launch order:
#   1. robot_bringup.launch.py      — Gazebo + robot state publisher
#   2. robot_navigation_amcl_ekf    — Map server + AMCL + EKF
#   3. localization_benchmark_amcl  — amcl_pose_to_odom + benchmark dashboard
#   4. RViz with comparison config
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
echo " ekf_sim container starting"
echo " ROS_DOMAIN_ID = ${ROS_DOMAIN_ID}"
echo " DISPLAY       = ${DISPLAY}"
echo " Results dir   = ${BENCHMARK_RESULTS_DIR}"
echo "======================================================"

# ── 1. Simulation bringup (Gazebo + robot) ────────────────────────────────────
echo "[1/4] Launching Gazebo + robot bringup ..."
ros2 launch robot_bringup robot_bringup.launch.py &
BRINGUP_PID=$!

# Wait for Gazebo to be up (clock topic indicates sim is running)
echo "      Waiting for /clock ..."
until ros2 topic hz /clock --window 5 2>/dev/null | grep -q "average rate"; do
    sleep 4
done
echo "      Gazebo ready."
sleep 5
# ── 2. AMCL + EKF localisation ────────────────────────────────────────────────
echo "[2/4] Launching AMCL + EKF localisation ..."
ros2 launch robot_bringup robot_navigation_amcl_ekf.launch.py &
AMCL_EKF_PID=$!

# Brief pause so AMCL can activate before benchmark starts
# sleep 5

# # ── 3. Benchmark node ─────────────────────────────────────────────────────────
# echo "[3/4] Launching AMCL benchmark ..."
# ros2 launch robot_gazebo localization_benchmark_amcl.launch.py \
#     csv_output_dir:="${BENCHMARK_RESULTS_DIR:-/ros2_ws/benchmark_results}" &
# BENCH_PID=$!

echo "======================================================"
echo " All nodes started.  PIDs:"
echo "   bringup   : ${BRINGUP_PID}"
echo "   amcl+ekf  : ${AMCL_EKF_PID}"
echo " Ctrl-C to stop all."
echo "======================================================"

# ── Wait for any child process to exit, then kill the rest ───────────────────
wait -n 2>/dev/null || true
echo "A process exited — shutting down ekf_sim ..."
kill ${BRINGUP_PID} ${AMCL_EKF_PID} 2>/dev/null || true
wait
