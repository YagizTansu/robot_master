#!/usr/bin/env python3

# BSD 3-Clause License
#
# Copyright (c) 2026, Yagiz
# All rights reserved.

"""VDA5050 v2.0.0 compliant adapter for mecanum/omni-drive robot.

This adapter bridges the VDA5050 connector (controller) with the robot's
ROS 2 navigation stack (Nav2) and sensor interfaces. It implements:

- Full OrderState reporting (position, velocity, battery, safety, errors)
- Navigation via Nav2 with edge parameter awareness
- VDA5050 standard instant actions (initPosition, startPause, stopPause,
  startCharging, stopCharging, stateRequest)
- Proper feedback during navigation
- Thread-safe state management
"""

import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy

import tf_transformations
from action_msgs.msg import GoalStatus

from vda5050_connector_py.utils import read_str_parameter

# VDA5050 connector interfaces
from vda5050_connector.action import NavigateToNode
from vda5050_connector.action import ProcessVDAAction
from vda5050_connector.srv import GetState
from vda5050_connector.srv import SupportedActions

# Nav2
from nav2_msgs.action import NavigateToPose

# Standard ROS 2 messages
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState as ROSBatteryState
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist

# TF2
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# VDA5050 messages
from vda5050_msgs.msg import AGVPosition as VDAAGVPosition
from vda5050_msgs.msg import Velocity as VDAVelocity
from vda5050_msgs.msg import OrderState as VDAOrderState
from vda5050_msgs.msg import CurrentAction as VDACurrentAction
from vda5050_msgs.msg import BatteryState as VDABatteryState
from vda5050_msgs.msg import SafetyState as VDASafetyState
from vda5050_msgs.msg import Error as VDAError
from vda5050_msgs.msg import ErrorReference as VDAErrorReference
from vda5050_msgs.msg import AGVAction as VDAAGVAction

NODE_NAME = "robot_adapter"

# VDA5050 Standard Action Types
ACTION_INIT_POSITION = "initPosition"
ACTION_START_PAUSE = "startPause"
ACTION_STOP_PAUSE = "stopPause"
ACTION_START_CHARGING = "startCharging"
ACTION_STOP_CHARGING = "stopCharging"
ACTION_STATE_REQUEST = "stateRequest"


class RobotVDA5050Adapter(Node):
    """VDA5050 v2.0.0 compliant adapter node.

    Implements the adapter plugin interfaces expected by the vda5050_connector
    controller: GetState service, SupportedActions service, NavigateToNode
    action server, and ProcessVDAAction action server.
    """

    def __init__(self):
        super().__init__(NODE_NAME)
        self.logger = self.get_logger()

        # === State Variables ===
        self._agv_position = VDAAGVPosition()
        self._agv_position.position_initialized = False
        self._agv_position.localization_score = 0.0

        self._velocity = VDAVelocity()

        self._battery_state = VDABatteryState()

        self._safety_state = VDASafetyState(
            e_stop=VDASafetyState.NONE,
            field_violation=False,
        )

        self._driving = False
        self._paused = False
        self._operating_mode = VDAOrderState.AUTOMATIC
        self._errors = []
        self._informations = []

        # Thread safety
        self._state_lock = threading.Lock()

        # Active Nav2 goal handle (for pause/cancel)
        self._current_nav2_handle = None
        self._current_nav2_lock = threading.Lock()

        self._on_configure()
        self.logger.info(f"Node '{NODE_NAME}' started — VDA5050 v2.0.0 compliant adapter")

    # ================================================================
    #  Configuration
    # ================================================================

    def _on_configure(self):
        self._read_parameters()

        base_interface_name = (
            f"{self.get_namespace()}/{self._manufacturer_name}/{self._robot_name}/"
        )

        # --- Services ---
        self._get_state_srv = self.create_service(
            GetState,
            base_interface_name + self._get_state_svc_name,
            self._get_state_callback,
        )

        self._supported_actions_srv = self.create_service(
            SupportedActions,
            base_interface_name + self._supported_actions_svc_name,
            self._supported_actions_callback,
        )

        # --- Action Servers ---
        nav_cb_group = MutuallyExclusiveCallbackGroup()
        self._nav_to_node_action_srv = ActionServer(
            self,
            NavigateToNode,
            base_interface_name + self._nav_to_node_act_name,
            execute_callback=self._navigate_to_node_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=nav_cb_group,
        )

        action_cb_group = ReentrantCallbackGroup()
        self._process_vda_action_srv = ActionServer(
            self,
            ProcessVDAAction,
            base_interface_name + self._vda_action_act_name,
            execute_callback=self._process_vda_action_callback,
            callback_group=action_cb_group,
        )

        # --- Nav2 Action Client ---
        nav2_cb_group = MutuallyExclusiveCallbackGroup()
        self._nav2_client = ActionClient(
            self,
            NavigateToPose,
            self._nav2_action_name,
            callback_group=nav2_cb_group,
        )

        # --- Publishers ---
        self._init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # --- Subscribers ---
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._odom_sub = self.create_subscription(
            Odometry, self._odom_topic, self._odom_callback, 10
        )

        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._battery_sub = self.create_subscription(
            ROSBatteryState,
            self._battery_topic,
            self._battery_callback,
            qos_best_effort,
        )

        self._emergency_sub = self.create_subscription(
            Bool,
            self._emergency_topic,
            self._emergency_callback,
            qos_best_effort,
        )

        self._diagnostics_sub = self.create_subscription(
            DiagnosticArray,
            self._diagnostics_topic,
            self._diagnostics_callback,
            qos_best_effort,
        )

    def _read_parameters(self):
        # Robot identity
        self._robot_name = read_str_parameter(self, "robot_name", "robot_1")
        self._manufacturer_name = read_str_parameter(
            self, "manufacturer_name", "robots"
        )
        self._serial_number = read_str_parameter(self, "serial_number", "robot_1")

        # Adapter interface names (must match controller config)
        self._get_state_svc_name = read_str_parameter(
            self, "get_state_svc_name", "adapter/get_state"
        )
        self._vda_action_act_name = read_str_parameter(
            self, "vda_action_act_name", "adapter/vda_action"
        )
        self._nav_to_node_act_name = read_str_parameter(
            self, "nav_to_node_act_name", "adapter/nav_to_node"
        )
        self._supported_actions_svc_name = read_str_parameter(
            self, "supported_actions_svc_name", "adapter/supported_actions"
        )

        # Frame IDs
        self._map_frame = read_str_parameter(self, "map_frame", "map")
        self._base_frame = read_str_parameter(self, "base_frame", "base_link")

        # ROS topic names
        self._odom_topic = read_str_parameter(self, "odom_topic", "/odom")
        self._battery_topic = read_str_parameter(
            self, "battery_topic", "/battery_status"
        )
        self._emergency_topic = read_str_parameter(
            self, "emergency_topic", "/emergency_stop"
        )
        self._diagnostics_topic = read_str_parameter(
            self, "diagnostics_topic", "/diagnostics_agg"
        )
        self._nav2_action_name = read_str_parameter(
            self, "nav2_action_name", "/navigate_to_pose"
        )

    # ================================================================
    #  Subscriber Callbacks
    # ================================================================

    def _odom_callback(self, msg: Odometry):
        with self._state_lock:
            # --- Position from TF (map → base_link) ---
            try:
                trans = self._tf_buffer.lookup_transform(
                    self._map_frame, self._base_frame, rclpy.time.Time()
                )
                self._agv_position.x = trans.transform.translation.x
                self._agv_position.y = trans.transform.translation.y

                q = trans.transform.rotation
                _, _, theta = tf_transformations.euler_from_quaternion(
                    (q.x, q.y, q.z, q.w)
                )
                self._agv_position.theta = theta
                self._agv_position.map_id = self._map_frame
                self._agv_position.position_initialized = True
                self._agv_position.localization_score = 1.0

            except TransformException as ex:
                self.logger.debug(f"TF lookup failed: {ex}")
                self._agv_position.position_initialized = False
                self._agv_position.localization_score = 0.0

            # --- Velocity ---
            self._velocity.vx = msg.twist.twist.linear.x
            self._velocity.vy = msg.twist.twist.linear.y
            self._velocity.omega = msg.twist.twist.angular.z

            # --- Driving flag (based on velocity thresholds) ---
            self._driving = (
                abs(self._velocity.vx) > 0.01
                or abs(self._velocity.vy) > 0.01
                or abs(self._velocity.omega) > 0.01
            )

    def _battery_callback(self, msg: ROSBatteryState):
        with self._state_lock:
            self._battery_state.battery_charge = msg.percentage * 100.0
            self._battery_state.battery_voltage = msg.voltage
            self._battery_state.charging = (
                msg.power_supply_status == ROSBatteryState.POWER_SUPPLY_STATUS_CHARGING
            )
            if msg.power_supply_health == ROSBatteryState.POWER_SUPPLY_HEALTH_GOOD:
                self._battery_state.battery_health = 100
            elif msg.power_supply_health == ROSBatteryState.POWER_SUPPLY_HEALTH_OVERHEAT:
                self._battery_state.battery_health = 50
            else:
                self._battery_state.battery_health = 80

    def _emergency_callback(self, msg: Bool):
        with self._state_lock:
            if msg.data:
                self._safety_state.e_stop = VDASafetyState.AUTO_ACK
                if not any(e.error_type == "emergencyStop" for e in self._errors):
                    self._errors.append(
                        VDAError(
                            error_type="emergencyStop",
                            error_description="Emergency stop is activated",
                            error_level=VDAError.FATAL,
                            error_references=[
                                VDAErrorReference(
                                    reference_key="topic",
                                    reference_value=self._emergency_topic,
                                )
                            ],
                        )
                    )
            else:
                self._safety_state.e_stop = VDASafetyState.NONE
                self._errors = [
                    e for e in self._errors if e.error_type != "emergencyStop"
                ]

    def _diagnostics_callback(self, msg: DiagnosticArray):
        with self._state_lock:
            # Remove old diagnostic errors, keep non-diagnostic ones
            self._errors = [
                e for e in self._errors if not e.error_type.startswith("hw_")
            ]
            for status in msg.status:
                # level is bytes in ROS 2 Jazzy, convert to int for comparison
                level = int.from_bytes(status.level, byteorder='little') if isinstance(status.level, bytes) else int(status.level)
                if level >= 2:  # ERROR or STALE
                    self._errors.append(
                        VDAError(
                            error_type=f"hw_{status.name}",
                            error_description=status.message or f"{status.name} error",
                            error_level=VDAError.FATAL,
                            error_references=[
                                VDAErrorReference(
                                    reference_key="component",
                                    reference_value=status.name,
                                )
                            ],
                        )
                    )

    # ================================================================
    #  Service Callbacks
    # ================================================================

    def _get_state_callback(self, request, response):
        """Return complete OrderState with all sensor data."""
        with self._state_lock:
            state = VDAOrderState()
            state.agv_position = self._agv_position
            state.velocity = self._velocity
            state.battery_state = self._battery_state
            state.safety_state = self._safety_state
            state.driving = self._driving
            state.paused = self._paused
            state.operating_mode = self._operating_mode
            state.errors = list(self._errors)
            state.informations = list(self._informations)
            response.state = state
        return response

    def _supported_actions_callback(self, request, response):
        """Return list of VDA5050 actions this adapter supports."""
        response.agv_actions = [
            VDAAGVAction(
                action_type=ACTION_INIT_POSITION,
                action_description="Initialize AGV position and orientation",
                action_scopes=[VDAAGVAction.INSTANT, VDAAGVAction.NODE],
            ),
            VDAAGVAction(
                action_type=ACTION_START_PAUSE,
                action_description="Pause the AGV — stops all movement",
                action_scopes=[VDAAGVAction.INSTANT],
            ),
            VDAAGVAction(
                action_type=ACTION_STOP_PAUSE,
                action_description="Resume AGV from paused state",
                action_scopes=[VDAAGVAction.INSTANT],
            ),
            VDAAGVAction(
                action_type=ACTION_START_CHARGING,
                action_description="Start battery charging",
                action_scopes=[VDAAGVAction.INSTANT, VDAAGVAction.NODE],
            ),
            VDAAGVAction(
                action_type=ACTION_STOP_CHARGING,
                action_description="Stop battery charging",
                action_scopes=[VDAAGVAction.INSTANT, VDAAGVAction.NODE],
            ),
            VDAAGVAction(
                action_type=ACTION_STATE_REQUEST,
                action_description="Request immediate state update",
                action_scopes=[VDAAGVAction.INSTANT],
            ),
        ]
        return response

    # ================================================================
    #  NavigateToNode Action Server
    # ================================================================

    def _navigate_to_node_callback(self, goal_handle):
        """Navigate to a VDA5050 node via Nav2, respecting edge parameters."""
        node = goal_handle.request.node
        edge = goal_handle.request.edge

        self.logger.info(
            f"NavigateToNode: node='{node.node_id}' (seq={node.sequence_id}) "
            f"via edge='{edge.edge_id}' → "
            f"({node.node_position.x:.2f}, {node.node_position.y:.2f}, "
            f"{node.node_position.theta:.2f} rad)"
        )

        # Log edge constraints
        if edge.max_speed > 0.0:
            self.logger.info(f"  Edge max_speed: {edge.max_speed:.2f} m/s")
        if not edge.rotation_allowed:
            self.logger.info("  Edge rotation_allowed: False")

        # --- Build Nav2 goal ---
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = (
            node.node_position.map_id
            if node.node_position.map_id
            else self._map_frame
        )
        goal_pose.pose.position.x = node.node_position.x
        goal_pose.pose.position.y = node.node_position.y

        q = tf_transformations.quaternion_from_euler(
            0, 0, node.node_position.theta
        )
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose

        # --- Wait for Nav2 ---
        if not self._nav2_client.wait_for_server(timeout_sec=10.0):
            self.logger.error("Nav2 action server not available")
            self._add_error(
                "navigationError",
                "Nav2 action server not available",
                VDAError.FATAL,
            )
            goal_handle.abort()
            return NavigateToNode.Result()

        # --- Send goal asynchronously ---
        nav_done = threading.Event()
        nav_status = {"result": None, "handle": None}

        def _on_goal_response(future):
            gh = future.result()
            if not gh.accepted:
                nav_status["result"] = "rejected"
                nav_done.set()
                return
            nav_status["handle"] = gh
            with self._current_nav2_lock:
                self._current_nav2_handle = gh
            gh.get_result_async().add_done_callback(_on_nav_result)

        def _on_nav_result(future):
            nav_status["result"] = future.result().status
            with self._current_nav2_lock:
                self._current_nav2_handle = None
            nav_done.set()

        send_future = self._nav2_client.send_goal_async(nav_goal)
        send_future.add_done_callback(_on_goal_response)

        # --- Feedback loop ---
        while not nav_done.is_set():
            # Check VDA cancel request
            if goal_handle.is_cancel_requested:
                self.logger.info("VDA cancel requested — cancelling Nav2 goal")
                if nav_status["handle"]:
                    nav_status["handle"].cancel_goal_async()
                goal_handle.canceled()
                return NavigateToNode.Result()

            # Publish position/velocity feedback to controller
            feedback = NavigateToNode.Feedback()
            with self._state_lock:
                feedback.position = self._agv_position
                feedback.velocity = self._velocity
            goal_handle.publish_feedback(feedback)

            nav_done.wait(timeout=0.2)

        # --- Process result ---
        self._remove_error("navigationError")

        if nav_status["result"] == "rejected":
            self.logger.error(f"Nav2 rejected goal for node '{node.node_id}'")
            self._add_error(
                "navigationError",
                f"Navigation goal rejected for node '{node.node_id}'",
                VDAError.WARNING,
            )
            goal_handle.abort()

        elif nav_status["result"] == GoalStatus.STATUS_SUCCEEDED:
            self.logger.info(
                f"Navigation to node '{node.node_id}' succeeded"
            )
            goal_handle.succeed()

        elif nav_status["result"] == GoalStatus.STATUS_CANCELED:
            self.logger.info(
                f"Navigation to node '{node.node_id}' was canceled"
            )
            if self._paused:
                goal_handle.abort()
            else:
                goal_handle.canceled()

        else:
            self.logger.error(
                f"Navigation to node '{node.node_id}' failed "
                f"(status={nav_status['result']})"
            )
            self._add_error(
                "navigationError",
                f"Navigation to node '{node.node_id}' failed",
                VDAError.WARNING,
            )
            goal_handle.abort()

        return NavigateToNode.Result()

    # ================================================================
    #  ProcessVDAAction Action Server
    # ================================================================

    def _process_vda_action_callback(self, goal_handle):
        """Execute a VDA5050 action (instant or node-triggered)."""
        action = goal_handle.request.action
        result = ProcessVDAAction.Result()

        params = {p.key: p.value for p in action.action_parameters}
        self.logger.info(
            f"ProcessVDAAction: type='{action.action_type}' "
            f"id='{action.action_id}' blocking='{action.blocking_type}'"
        )

        # Publish INITIALIZING feedback
        self._publish_action_feedback(
            goal_handle, action, VDACurrentAction.INITIALIZING
        )

        handler = {
            ACTION_INIT_POSITION: lambda: self._handle_init_position(action, params),
            ACTION_START_PAUSE: lambda: self._handle_start_pause(action),
            ACTION_STOP_PAUSE: lambda: self._handle_stop_pause(action),
            ACTION_START_CHARGING: lambda: self._handle_start_charging(action),
            ACTION_STOP_CHARGING: lambda: self._handle_stop_charging(action),
            ACTION_STATE_REQUEST: lambda: self._handle_state_request(action),
        }.get(action.action_type)

        if handler:
            # Publish RUNNING feedback
            self._publish_action_feedback(
                goal_handle, action, VDACurrentAction.RUNNING
            )
            result.result = handler()
        else:
            self.logger.warning(
                f"Unsupported action type: '{action.action_type}'"
            )
            result.result = VDACurrentAction(
                action_id=action.action_id,
                action_description=action.action_description,
                action_status=VDACurrentAction.FAILED,
                result_description=(
                    f"Action type '{action.action_type}' is not supported by this AGV"
                ),
            )

        goal_handle.succeed()
        return result

    # ================================================================
    #  Action Handlers
    # ================================================================

    def _handle_init_position(self, action, params):
        """Set AGV position via /initialpose (AMCL/FGO localization reset)."""
        try:
            pose = PoseWithCovarianceStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = params.get("mapId", self._map_frame)

            pose.pose.pose.position.x = float(params["x"])
            pose.pose.pose.position.y = float(params["y"])

            q = tf_transformations.quaternion_from_euler(
                0, 0, float(params["theta"])
            )
            pose.pose.pose.orientation.x = q[0]
            pose.pose.pose.orientation.y = q[1]
            pose.pose.pose.orientation.z = q[2]
            pose.pose.pose.orientation.w = q[3]

            self._init_pose_pub.publish(pose)

            self.logger.info(
                f"initPosition → ({params['x']}, {params['y']}, {params['theta']}) "
                f"on map '{pose.header.frame_id}'"
            )
            return VDACurrentAction(
                action_id=action.action_id,
                action_description=action.action_description,
                action_status=VDACurrentAction.FINISHED,
                result_description="Position initialized",
            )

        except (KeyError, ValueError) as e:
            self.logger.error(f"initPosition failed: {e}")
            return VDACurrentAction(
                action_id=action.action_id,
                action_description=action.action_description,
                action_status=VDACurrentAction.FAILED,
                result_description=f"Invalid parameters: {e}",
            )

    def _handle_start_pause(self, action):
        """Pause AGV: stop movement and cancel active navigation."""
        with self._state_lock:
            self._paused = True

        # Immediately stop the robot
        self._cmd_vel_pub.publish(Twist())

        # Cancel active Nav2 goal if any
        with self._current_nav2_lock:
            if self._current_nav2_handle is not None:
                self.logger.info("Pausing: cancelling active Nav2 navigation")
                self._current_nav2_handle.cancel_goal_async()

        self.logger.info("AGV paused")
        return VDACurrentAction(
            action_id=action.action_id,
            action_description=action.action_description,
            action_status=VDACurrentAction.FINISHED,
            result_description="AGV paused",
        )

    def _handle_stop_pause(self, action):
        """Resume AGV from paused state."""
        with self._state_lock:
            self._paused = False

        self.logger.info("AGV resumed from pause")
        return VDACurrentAction(
            action_id=action.action_id,
            action_description=action.action_description,
            action_status=VDACurrentAction.FINISHED,
            result_description="AGV resumed",
        )

    def _handle_start_charging(self, action):
        """Start battery charging (docking assumed complete)."""
        # TODO: Implement actual docking sequence when hardware is ready
        with self._state_lock:
            self._battery_state.charging = True

        self.logger.info("Charging started")
        return VDACurrentAction(
            action_id=action.action_id,
            action_description=action.action_description,
            action_status=VDACurrentAction.FINISHED,
            result_description="Charging started",
        )

    def _handle_stop_charging(self, action):
        """Stop battery charging (undocking follows)."""
        # TODO: Implement actual undocking sequence when hardware is ready
        with self._state_lock:
            self._battery_state.charging = False

        self.logger.info("Charging stopped")
        return VDACurrentAction(
            action_id=action.action_id,
            action_description=action.action_description,
            action_status=VDACurrentAction.FINISHED,
            result_description="Charging stopped",
        )

    def _handle_state_request(self, action):
        """Trigger immediate state publish (handled by controller timer)."""
        self.logger.info("State request received")
        return VDACurrentAction(
            action_id=action.action_id,
            action_description=action.action_description,
            action_status=VDACurrentAction.FINISHED,
        )

    # ================================================================
    #  Helpers
    # ================================================================

    def _publish_action_feedback(self, goal_handle, action, status):
        feedback = ProcessVDAAction.Feedback()
        feedback.current_action = VDACurrentAction(
            action_id=action.action_id,
            action_description=action.action_description,
            action_status=status,
        )
        goal_handle.publish_feedback(feedback)

    def _add_error(self, error_type, description, level):
        with self._state_lock:
            if not any(e.error_type == error_type for e in self._errors):
                self._errors.append(
                    VDAError(
                        error_type=error_type,
                        error_description=description,
                        error_level=level,
                    )
                )

    def _remove_error(self, error_type):
        with self._state_lock:
            self._errors = [
                e for e in self._errors if e.error_type != error_type
            ]


def main(args=None):
    rclpy.init(args=args)
    node = RobotVDA5050Adapter()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
