# ─────────────────────────────────────────────────────────────────────────────
# robot_master — Localization Benchmark Base Image
#
# Build once, run forever.  Source code is NOT copied into the image.
# The workspace is bind-mounted at runtime → code changes require only
# `colcon build` on the host and `docker compose restart <service>`.
#
# Build:
#   cd docker/
#   docker compose build
# ─────────────────────────────────────────────────────────────────────────────
ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop-full

ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive

# ── System dependencies ───────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    # ROS 2 / Nav2 stack
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-xacro \
    # Gazebo Harmonic + ROS bridge
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-interfaces \
    # Sensor / PCL / LiDAR
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-laser-geometry \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-tf2-sensor-msgs \
    # Behaviour Trees
    ros-${ROS_DISTRO}-behaviortree-cpp \
    ros-${ROS_DISTRO}-nav2-behavior-tree \
    # VDA5050 / MQTT adapter deps
    ros-${ROS_DISTRO}-lifecycle-msgs \
    # C++ third-party
    libgtsam-dev \
    libpcl-dev \
    nlohmann-json3-dev \
    libgeographiclib-dev \
    # Python GUI (BT visualizer, curses dashboard)
    python3-tk \
    python3-numpy \
    python3-scipy \
    # Display / X11 forwarding
    libgl1 \
    libglib2.0-0 \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# ── Python pip packages ───────────────────────────────────────────────────────
RUN pip3 install --break-system-packages \
    xacro \
    graphviz \
    requests

# ── rosdep initialise (skip if already done in base image) ───────────────────
RUN rosdep init 2>/dev/null || true && rosdep update

# ── NVIDIA / EGL for Gazebo hardware rendering ───────────────────────────────
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility
ENV __NV_PRIME_RENDER_OFFLOAD=1
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia

# ── Runtime defaults ──────────────────────────────────────────────────────────
ENV ROS_DOMAIN_ID=0
WORKDIR /ros2_ws

# Source ROS in every shell
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

CMD ["/bin/bash"]
