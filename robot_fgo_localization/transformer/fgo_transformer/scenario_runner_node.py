#!/usr/bin/env python3
"""
Scenario Runner Node
====================
Runs a predefined sequence of kinematic stress scenarios to intentionally
generate extreme feature values for Transformer training data diversity.

Unlike graph_traversal_node (spatial coverage), each scenario here is
designed to push specific features to their extremes:

  Scenario          Target feature(s)
  ─────────────     ──────────────────────────────────────────────────
  jerk_burst        jerk, accel_norm_dev   (sudden accel / brake)
  spin_in_place     gyro_z, angular_vel, gyro_z_std  (pure rotation)
  slalom            slip_metric  (gyro_z vs angular_vel divergence)
  sustained_drive   linear_vel, low jerk   (stable high-speed run)
  stop_and_go       alternating jerk bursts, accel_norm_std
  slow_creep        near-zero vel / angular_vel  (low-end coverage)

Execution model
---------------
Each scenario publishes ``/cmd_vel`` directly (bypassing Nav2's path
smoother so extremes are actually reached).  Between scenarios the node
sends a NavigateToPose goal to return the robot to a known safe position.

Parameters
----------
reset_node_id       Graph node ID to return to between scenarios
                    (default: "node_0_0" — centre of the map)
graph_file          Path to robot_map_graph.json  (for reset goals)
repeat_cycles       How many full scenario cycles to run   (default: 3)
cmd_vel_topic       Topic to publish cmd_vel on             (default: /cmd_vel)
use_sim_time        Use /clock                              (default: true)
"""

import json
import math
import os
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from std_msgs.msg import String


# ─────────────────────────────────────────────────────────────────────────────
# Scenario definitions
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class ScenarioStep:
    """A single timed cmd_vel command within a scenario."""
    linear_x:  float
    angular_z: float
    duration_sec: float


@dataclass
class Scenario:
    name:        str
    description: str
    steps:       List[ScenarioStep]


def _make_scenarios() -> List[Scenario]:
    """
    Returns the full scenario list.  Each scenario is a sequence of timed
    velocity commands followed by an implicit full-stop.
    """
    return [
        # ── 1. Jerk burst ──────────────────────────────────────────────────
        # Rapid acceleration then abrupt stop — high jerk, accel_norm_dev.
        Scenario(
            name="jerk_burst",
            description="Rapid accel then abrupt stop — targets jerk & accel_norm_dev",
            steps=[
                ScenarioStep(0.0,  0.0, 0.5),   # static baseline
                ScenarioStep(0.6,  0.0, 0.3),   # ramp up
                ScenarioStep(0.8,  0.0, 1.0),   # sustained speed
                ScenarioStep(0.0,  0.0, 0.1),   # abrupt stop  ← high jerk
                ScenarioStep(0.0,  0.0, 1.0),   # settle
                ScenarioStep(-0.5, 0.0, 0.3),   # reverse burst
                ScenarioStep(0.0,  0.0, 0.1),   # abrupt stop
                ScenarioStep(0.0,  0.0, 1.5),   # long settle
            ],
        ),

        # ── 2. Spin in place ──────────────────────────────────────────────
        # Pure rotation: high gyro_z, angular_vel, gyro_z_std.
        Scenario(
            name="spin_in_place",
            description="Full rotation in place — targets gyro_z & angular_vel",
            steps=[
                ScenarioStep(0.0,  0.3, 1.0),   # slow spin start
                ScenarioStep(0.0,  0.6, 2.0),   # medium spin
                ScenarioStep(0.0,  0.9, 2.0),   # fast spin  ← peak gyro_z
                ScenarioStep(0.0,  0.6, 1.0),   # decelerate
                ScenarioStep(0.0,  0.0, 0.5),   # stop
                ScenarioStep(0.0, -0.5, 2.0),   # reverse direction
                ScenarioStep(0.0, -0.9, 2.0),   # fast reverse spin
                ScenarioStep(0.0,  0.0, 1.0),   # stop
            ],
        ),

        # ── 3. Slalom ─────────────────────────────────────────────────────
        # Move forward while oscillating — high slip_metric
        # (IMU angular rate diverges from wheel encoder angular velocity).
        Scenario(
            name="slalom",
            description="Sinusoidal yaw while moving — targets slip_metric",
            steps=[
                ScenarioStep(0.4,  0.5, 0.8),   # forward-left arc
                ScenarioStep(0.4, -0.5, 0.8),   # forward-right arc
                ScenarioStep(0.4,  0.6, 0.8),
                ScenarioStep(0.4, -0.6, 0.8),
                ScenarioStep(0.4,  0.8, 0.6),   # tighter arcs
                ScenarioStep(0.4, -0.8, 0.6),
                ScenarioStep(0.4,  0.8, 0.6),
                ScenarioStep(0.4, -0.8, 0.6),
                ScenarioStep(0.0,  0.0, 1.0),   # stop
            ],
        ),

        # ── 4. Sustained high-speed drive ────────────────────────────────
        # Smooth straight run — high linear_vel, low jerk, good fitness_score.
        Scenario(
            name="sustained_drive",
            description="Straight sustained drive — targets linear_vel low-jerk region",
            steps=[
                ScenarioStep(0.2,  0.0, 0.5),   # gentle ramp
                ScenarioStep(0.5,  0.0, 0.5),
                ScenarioStep(0.8,  0.0, 4.0),   # ← sustained peak velocity
                ScenarioStep(0.5,  0.0, 0.5),   # smooth decel
                ScenarioStep(0.0,  0.0, 1.0),
            ],
        ),

        # ── 5. Stop-and-go ────────────────────────────────────────────────
        # Repeated start/stop cycles — high accel_norm_std, jerk bursts.
        Scenario(
            name="stop_and_go",
            description="Repeated stop-start — targets accel_norm_std",
            steps=[
                ScenarioStep(0.5, 0.0, 0.4),
                ScenarioStep(0.0, 0.0, 0.3),
                ScenarioStep(0.5, 0.0, 0.4),
                ScenarioStep(0.0, 0.0, 0.3),
                ScenarioStep(0.5, 0.0, 0.4),
                ScenarioStep(0.0, 0.0, 0.3),
                ScenarioStep(0.6, 0.0, 0.4),
                ScenarioStep(0.0, 0.0, 0.3),
                ScenarioStep(0.6, 0.0, 0.4),
                ScenarioStep(0.0, 0.0, 1.0),
            ],
        ),

        # ── 6. Slow creep ─────────────────────────────────────────────────
        # Very slow movement — fills the near-zero region in all features.
        Scenario(
            name="slow_creep",
            description="Very slow movement — fills near-zero region of all features",
            steps=[
                ScenarioStep(0.05, 0.0,  1.0),
                ScenarioStep(0.05, 0.05, 2.0),
                ScenarioStep(0.0,  0.05, 2.0),
                ScenarioStep(0.05, 0.0,  1.0),
                ScenarioStep(0.0,  0.0,  1.0),
            ],
        ),
    ]


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _load_node(graph_file: str, node_id: str) -> Optional[Dict]:
    if not graph_file or not os.path.isfile(graph_file):
        return None
    with open(graph_file, "r") as f:
        data = json.load(f)
    nodes = {n["id"]: n for n in data["graph"]["nodes"]}
    return nodes.get(node_id)


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 Node
# ─────────────────────────────────────────────────────────────────────────────

# Runner states
_S_WAIT_NAV2   = "WAIT_NAV2"
_S_RESETTING   = "RESETTING"
_S_RUNNING     = "RUNNING"
_S_STEP_PAUSE  = "STEP_PAUSE"
_S_DONE        = "DONE"


class ScenarioRunnerNode(Node):

    def __init__(self) -> None:
        super().__init__("scenario_runner")

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter("reset_node_id",   "node_0_0")
        self.declare_parameter("graph_file",      "")
        self.declare_parameter("repeat_cycles",   3)
        self.declare_parameter("cmd_vel_topic",   "/cmd_vel")

        reset_id        = str(self.get_parameter("reset_node_id").value).strip()
        graph_file      = str(self.get_parameter("graph_file").value).strip()
        self._cycles    = int(self.get_parameter("repeat_cycles").value)
        cmd_vel_topic   = str(self.get_parameter("cmd_vel_topic").value).strip()

        self._reset_node = _load_node(graph_file, reset_id)
        if self._reset_node is None:
            self.get_logger().warn(
                f"reset_node_id '{reset_id}' not found — "
                "will skip reset step between scenarios"
            )

        # ── Scenario list ───────────────────────────────────────────────────
        self._scenarios: List[Scenario] = _make_scenarios()
        self._scenario_idx = 0
        self._step_idx     = 0
        self._cycle        = 0

        # ── Publisher / subscriber ──────────────────────────────────────────
        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._status_pub = self.create_publisher(
            String, "/data_collection/scenario_status", 10
        )

        # ── Nav2 action client (for reset moves) ───────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ── Internal state ──────────────────────────────────────────────────
        self._state = _S_WAIT_NAV2
        self._step_deadline: Optional[float] = None   # wall-clock deadline

        # ── Main control timer (10 Hz) ──────────────────────────────────────
        self._timer = self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f"\n{'='*60}\n"
            f"  ScenarioRunnerNode ready\n"
            f"{'='*60}\n"
            f"  Scenarios : {[s.name for s in self._scenarios]}\n"
            f"  Cycles    : {self._cycles}\n"
            f"  Reset pos : "
            f"{'({x:.2f}, {y:.2f})'.format(**self._reset_node) if self._reset_node else 'disabled'}\n"
            f"{'='*60}"
        )

    # ── Main control loop ─────────────────────────────────────────────────────

    def _tick(self) -> None:
        """Called at 10 Hz — drives the scenario state machine."""
        if self._state == _S_WAIT_NAV2:
            self._state_wait_nav2()

        elif self._state == _S_RESETTING:
            pass  # handled via Nav2 callbacks

        elif self._state == _S_RUNNING:
            self._state_running()

        elif self._state == _S_STEP_PAUSE:
            self._state_step_pause()

        elif self._state == _S_DONE:
            pass  # nothing left to do

    # ── State handlers ────────────────────────────────────────────────────────

    def _state_wait_nav2(self) -> None:
        if self._nav_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().info("Nav2 ready — starting scenario runner")
            self._start_next_scenario_or_reset()
        else:
            self.get_logger().info("Waiting for Nav2 navigate_to_pose…", throttle_duration_sec=5.0)

    def _state_running(self) -> None:
        scenario = self._scenarios[self._scenario_idx]

        # Check if current step has expired
        if self._step_deadline is not None and time.monotonic() >= self._step_deadline:
            self._stop()
            self._step_idx += 1

            if self._step_idx >= len(scenario.steps):
                # Scenario finished
                self.get_logger().info(
                    f"  ✓ Scenario '{scenario.name}' complete"
                )
                self._step_idx = 0
                self._scenario_idx += 1

                if self._scenario_idx >= len(self._scenarios):
                    # Full cycle done
                    self._cycle += 1
                    self._scenario_idx = 0
                    if self._cycle >= self._cycles:
                        self.get_logger().info(
                            f"All {self._cycles} cycle(s) complete. ScenarioRunner DONE."
                        )
                        self._publish_status("DONE")
                        self._state = _S_DONE
                        return
                    self.get_logger().info(
                        f"Cycle {self._cycle}/{self._cycles} done — starting next cycle"
                    )

                self._start_next_scenario_or_reset()
                return

            # Execute next step
            self._execute_step(scenario.steps[self._step_idx])

        elif self._step_deadline is None:
            # First call into this scenario step
            self._execute_step(scenario.steps[self._step_idx])

    def _state_step_pause(self) -> None:
        if self._step_deadline is not None and time.monotonic() >= self._step_deadline:
            self._state = _S_RUNNING
            self._step_deadline = None

    # ── Scenario / step helpers ───────────────────────────────────────────────

    def _start_next_scenario_or_reset(self) -> None:
        scenario = self._scenarios[self._scenario_idx]
        self.get_logger().info(
            f"\n[Cycle {self._cycle+1}/{self._cycles}] "
            f"Scenario: {scenario.name} — {scenario.description}"
        )
        self._publish_status(
            "START",
            scenario=scenario.name,
            cycle=self._cycle + 1,
        )

        if self._reset_node:
            self._send_reset_goal()
        else:
            self._state = _S_RUNNING
            self._step_deadline = None

    def _execute_step(self, step: ScenarioStep) -> None:
        twist = Twist()
        twist.linear.x  = step.linear_x
        twist.angular.z = step.angular_z
        self._cmd_pub.publish(twist)
        self._step_deadline = time.monotonic() + step.duration_sec

    def _stop(self) -> None:
        self._cmd_pub.publish(Twist())   # zero velocity

    # ── Reset navigation ──────────────────────────────────────────────────────

    def _send_reset_goal(self) -> None:
        self._state = _S_RESETTING
        rn = self._reset_node

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id    = "map"
        goal.pose.header.stamp       = self.get_clock().now().to_msg()
        goal.pose.pose.position.x    = float(rn["x"])
        goal.pose.pose.position.y    = float(rn["y"])
        goal.pose.pose.position.z    = 0.0
        goal.pose.pose.orientation   = _yaw_to_quat(0.0)

        self.get_logger().info(
            f"  → Resetting to ({rn['x']:.2f}, {rn['y']:.2f})"
        )
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_reset_accepted)

    def _on_reset_accepted(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Reset goal rejected — proceeding anyway")
            self._state = _S_RUNNING
            self._step_deadline = None
            return
        handle.get_result_async().add_done_callback(self._on_reset_result)

    def _on_reset_result(self, future) -> None:
        result = future.result()
        if result.status != 4:
            self.get_logger().warn(
                f"Reset navigation ended with status {result.status} — proceeding"
            )
        self._state = _S_RUNNING
        self._step_deadline = None

    # ── Status publisher ──────────────────────────────────────────────────────

    def _publish_status(self, event: str, **kwargs: Any) -> None:
        msg = String()
        msg.data = json.dumps({"event": event, **kwargs})
        self._status_pub.publish(msg)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScenarioRunnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
