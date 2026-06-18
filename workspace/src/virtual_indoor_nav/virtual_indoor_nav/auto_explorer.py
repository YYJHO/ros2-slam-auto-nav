#!/usr/bin/env python3
import math
from collections import deque
from enum import Enum
from typing import Deque, List, Optional, Set, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener


GridCell = Tuple[int, int]
FrontierGoal = Tuple[float, float, GridCell]


class ExplorerState(Enum):
    WAITING = "waiting_for_map"
    INITIAL_SPIN = "initial_spin"
    EXPLORING = "exploring"
    DONE = "exploration_complete"


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def goal_status_to_text(status: int) -> str:
    labels = {
        GoalStatus.STATUS_UNKNOWN: "unknown",
        GoalStatus.STATUS_ACCEPTED: "accepted",
        GoalStatus.STATUS_EXECUTING: "executing",
        GoalStatus.STATUS_CANCELING: "canceling",
        GoalStatus.STATUS_SUCCEEDED: "succeeded",
        GoalStatus.STATUS_CANCELED: "canceled",
        GoalStatus.STATUS_ABORTED: "aborted",
    }
    return labels.get(status, str(status))


class AutoExplorer(Node):
    def __init__(self) -> None:
        super().__init__("auto_explorer")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("free_threshold", 20)
        self.declare_parameter("unknown_value", -1)
        self.declare_parameter("initial_spin_duration", 10.0)
        self.declare_parameter("spin_angular_speed", 0.5)
        self.declare_parameter("exploration_speed", 0.3)
        self.declare_parameter("goal_tolerance", 0.4)
        self.declare_parameter("frontier_min_area", 0.05)
        self.declare_parameter("no_frontier_streak_limit", 15)
        self.declare_parameter("frontier_refresh_interval", 2.0)
        self.declare_parameter("stuck_timeout", 15.0)
        self.declare_parameter("stuck_distance", 0.1)
        self.declare_parameter("recovery_duration", 2.0)
        self.declare_parameter("max_recovery_attempts", 3)
        self.declare_parameter("front_obstacle_distance", 0.5)
        self.declare_parameter("max_exploration_time", 300.0)
        self.declare_parameter("angle_threshold", 0.2)
        self.declare_parameter("obstacle_avoid_angular", 0.6)
        self.declare_parameter("control_rate", 10.0)
        self.declare_parameter("laser_max_range", 12.0)
        self.declare_parameter("score_weight_info", 0.45)
        self.declare_parameter("score_weight_dist", 0.25)
        self.declare_parameter("score_weight_size", 0.20)
        self.declare_parameter("score_weight_history", 0.10)
        self.declare_parameter("stagnation_coverage_threshold", 0.03)
        self.declare_parameter("stagnation_window", 15)
        self.declare_parameter("min_coverage_for_done", 0.85)
        self.declare_parameter("max_unknown_ratio_for_done", 0.03)
        self.declare_parameter("goal_safety_radius", 0.30)
        self.declare_parameter("nav_failure_backoff", 1.0)
        self.declare_parameter("max_goal_failures", 2)
        self.declare_parameter("goal_history_size", 5)
        self.declare_parameter("verification_required_clear_rounds", 3)
        self.declare_parameter("verification_round_interval", 2.0)
        self.declare_parameter("robot_width", 0.44)
        self.declare_parameter("narrow_gap_multiplier", 1.5)
        self.declare_parameter("narrow_pass_speed", 0.15)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.free_threshold = int(self.get_parameter("free_threshold").value)
        self.unknown_value = int(self.get_parameter("unknown_value").value)
        self.initial_spin_duration = float(
            self.get_parameter("initial_spin_duration").value
        )
        self.spin_angular_speed = float(self.get_parameter("spin_angular_speed").value)
        self.exploration_speed = float(self.get_parameter("exploration_speed").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.frontier_min_area = float(self.get_parameter("frontier_min_area").value)
        self.no_frontier_streak_limit = int(
            self.get_parameter("no_frontier_streak_limit").value
        )
        self.frontier_refresh_interval = float(
            self.get_parameter("frontier_refresh_interval").value
        )
        self.stuck_timeout = float(self.get_parameter("stuck_timeout").value)
        self.stuck_distance = float(self.get_parameter("stuck_distance").value)
        self.recovery_duration = float(self.get_parameter("recovery_duration").value)
        self.max_recovery_attempts = int(
            self.get_parameter("max_recovery_attempts").value
        )
        self.front_obstacle_distance = float(
            self.get_parameter("front_obstacle_distance").value
        )
        self.max_exploration_time = float(self.get_parameter("max_exploration_time").value)
        self.angle_threshold = float(self.get_parameter("angle_threshold").value)
        self.obstacle_avoid_angular = float(
            self.get_parameter("obstacle_avoid_angular").value
        )
        control_rate = float(self.get_parameter("control_rate").value)
        self.laser_max_range = float(self.get_parameter("laser_max_range").value)
        self.score_weight_info = float(self.get_parameter("score_weight_info").value)
        self.score_weight_dist = float(self.get_parameter("score_weight_dist").value)
        self.score_weight_size = float(self.get_parameter("score_weight_size").value)
        self.score_weight_history = float(
            self.get_parameter("score_weight_history").value
        )
        self.stagnation_coverage_threshold = float(
            self.get_parameter("stagnation_coverage_threshold").value
        )
        self.stagnation_window = int(self.get_parameter("stagnation_window").value)
        self.min_coverage_for_done = float(
            self.get_parameter("min_coverage_for_done").value
        )
        self.max_unknown_ratio_for_done = float(
            self.get_parameter("max_unknown_ratio_for_done").value
        )
        self.goal_safety_radius = float(
            self.get_parameter("goal_safety_radius").value
        )
        self.nav_failure_backoff = float(
            self.get_parameter("nav_failure_backoff").value
        )
        self.max_goal_failures = int(self.get_parameter("max_goal_failures").value)
        goal_history_size = int(self.get_parameter("goal_history_size").value)
        self.verification_required_clear_rounds = int(
            self.get_parameter("verification_required_clear_rounds").value
        )
        self.verification_round_interval = float(
            self.get_parameter("verification_round_interval").value
        )
        self.robot_width = float(self.get_parameter("robot_width").value)
        self.narrow_gap_multiplier = float(
            self.get_parameter("narrow_gap_multiplier").value
        )
        self.narrow_pass_speed = float(self.get_parameter("narrow_pass_speed").value)

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(OccupancyGrid, "/map", self._map_callback, map_qos)
        self.create_subscription(LaserScan, "/scan", self._scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/exploration_status", 10)
        self.progress_pub = self.create_publisher(Float32, "/exploration_progress", 10)
        self.coverage_pub = self.create_publisher(Float32, "/exploration_coverage", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Nav2 action client for frontier navigation (replaces PID cmd_vel)
        self.nav_action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._nav_goal_active = False
        self._nav_goal_handle = None
        self._nav_goal_kind = "frontier"
        self._forced_goal_active = False
        self._forced_goal_pending = False
        self._last_nav_failure_time = None

        self.state = ExplorerState.WAITING
        self.latest_map: Optional[OccupancyGrid] = None
        self.latest_scan: Optional[LaserScan] = None
        self.frontier_goals: List[FrontierGoal] = []
        self.current_goal: Optional[FrontierGoal] = None
        self.blocked_goals: Set[GridCell] = set()
        self.goal_failure_counts = {}
        self.no_frontier_streak = 0
        self.initial_unknown_count: Optional[int] = None
        self.coverage = 0.0
        self.active_area_cells = 0
        self.explored_cells = 0
        self.coverage_history: Deque[float] = deque(maxlen=max(1, self.stagnation_window))
        self.goal_history: Deque[float] = deque(maxlen=max(1, goal_history_size))

        self.start_time = self.get_clock().now()
        self.spin_start_time = None
        self.last_progress_pose: Optional[Tuple[float, float]] = None
        self.last_progress_time = self.get_clock().now()
        self.recovering = False
        self.recovery_start_time = None
        self.recovery_attempts = 0
        self.last_status = ""

        # 3-round verification: must see 0 frontiers for N consecutive rounds
        self.clear_frontier_rounds = 0
        self.required_clear_rounds = self.verification_required_clear_rounds
        self.done_stop_published = False  # only publish stop once when DONE

        self.create_timer(1.0 / max(control_rate, 1.0), self._motion_loop)
        self.create_timer(
            max(self.frontier_refresh_interval, 0.2),
            self._frontier_detection_loop,
        )
        self._publish_status("waiting_for_map")

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg
        if self.initial_unknown_count is None:
            self.initial_unknown_count = sum(
                1 for value in msg.data if value == self.unknown_value
            )
        if self.state == ExplorerState.WAITING:
            self.state = ExplorerState.INITIAL_SPIN
            self.spin_start_time = self.get_clock().now()
            self._publish_status("map_received_initial_spin")

    def _scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _motion_loop(self) -> None:
        if self.state == ExplorerState.DONE:
            # Publish stop exactly once, then stay silent so Nav2 can drive
            if not self.done_stop_published:
                self._publish_stop()
                self.done_stop_published = True
            return

        self._publish_progress()

        if self.state == ExplorerState.WAITING:
            self._publish_stop()
            return

        if self.state == ExplorerState.INITIAL_SPIN:
            self._run_initial_spin()
            return

        if self.state != ExplorerState.EXPLORING:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > self.max_exploration_time:
            unknown_ratio = 1.0 - self.coverage
            self._complete_exploration(
                f"exploration_incomplete: timeout "
                f"coverage={self.coverage*100:.0f}% "
                f"unknown_ratio={unknown_ratio*100:.1f}% "
                f"frontiers_remaining={len(self.frontier_goals)}"
            )
            return

        # During EXPLORING, Nav2 drives the robot to frontier goals.
        # Only pick a new goal if Nav2 is not already handling one.
        if not self._nav_goal_active and not self._forced_goal_pending:
            pose = self._lookup_pose()
            if pose is not None and self.frontier_goals:
                self._pick_next_goal(pose)

    def _frontier_detection_loop(self) -> None:
        if self.state != ExplorerState.EXPLORING or self.latest_map is None:
            return

        pose = self._lookup_pose()
        if pose is None:
            return

        self._update_coverage(pose)

        frontiers = self._detect_frontiers(pose)
        if frontiers:
            # Found frontiers → reset all verification counters
            self.frontier_goals = frontiers
            self.no_frontier_streak = 0
            self.clear_frontier_rounds = 0
            self._publish_status(
                f"found_{len(frontiers)}_frontiers clear_rounds_reset"
            )
            self._check_stagnation()
            return

        # No frontiers detected this round
        self.frontier_goals = []
        self.no_frontier_streak += 1
        self._publish_status(
            f"no_frontiers_streak_{self.no_frontier_streak}/{self.no_frontier_streak_limit}"
        )

        # --- 3-round verification logic ---
        unknown_ratio = 1.0 - self.coverage
        needs_more_exploration = (
            self.coverage < self.min_coverage_for_done
            or unknown_ratio > self.max_unknown_ratio_for_done
        )

        if needs_more_exploration:
            # Coverage or unknown ratio is still insufficient → try forced exploration first
            if self._trigger_forced_exploration():
                self.no_frontier_streak = 0
                self.clear_frontier_rounds = 0
                self._publish_status("forced_explore_triggered clear_rounds_reset")
                return

            # Forced exploration failed → increment clear round
            self.clear_frontier_rounds += 1
            self._publish_status(
                f"frontier_clear_round {self.clear_frontier_rounds}/{self.required_clear_rounds} "
                f"(coverage={self.coverage*100:.0f}% "
                f"unknown_ratio={unknown_ratio*100:.1f}%)"
            )
        else:
            # Coverage already above threshold
            self.clear_frontier_rounds += 1
            self._publish_status(
                f"frontier_clear_round {self.clear_frontier_rounds}/{self.required_clear_rounds} "
                f"(coverage={self.coverage*100:.0f}% >= {self.min_coverage_for_done*100:.0f}%)"
            )

        # Only complete after N consecutive clear rounds and strict coverage checks.
        can_complete = (
            self.clear_frontier_rounds >= self.required_clear_rounds
            and self.coverage >= self.min_coverage_for_done
            and unknown_ratio <= self.max_unknown_ratio_for_done
            and not self._nav_goal_active
            and not self._forced_goal_active
            and not self._forced_goal_pending
        )
        if can_complete:
            self._publish_status(
                f"all_clear_rounds_passed {self.clear_frontier_rounds}/{self.required_clear_rounds}"
            )
            self._complete_exploration("exploration_complete")
        elif self.clear_frontier_rounds >= self.required_clear_rounds:
            self._publish_status(
                f"exploration_incomplete_waiting coverage={self.coverage*100:.1f}% "
                f"unknown_ratio={unknown_ratio*100:.1f}%"
            )

    def _run_initial_spin(self) -> None:
        if self.spin_start_time is None:
            self.spin_start_time = self.get_clock().now()

        elapsed = (self.get_clock().now() - self.spin_start_time).nanoseconds / 1e9
        if elapsed < self.initial_spin_duration:
            cmd = Twist()
            cmd.angular.z = self.spin_angular_speed
            self.cmd_pub.publish(cmd)
            self._publish_status(
                f"initial_spin {elapsed:.1f}/{self.initial_spin_duration:.1f}s"
            )
            return

        self._publish_stop()
        self.spin_start_time = None
        self.state = ExplorerState.EXPLORING
        self._publish_status("initial_spin_done_exploring")

    def _lookup_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
        return float(translation.x), float(translation.y), yaw

    def _pick_next_goal(self, pose: Tuple[float, float, float]) -> None:
        """Pop the best frontier and send it as a NavigateToPose action goal."""
        if not self.frontier_goals:
            return

        if not self.nav_action_client.wait_for_server(timeout_sec=1.0):
            self._publish_status("nav_action_server_unavailable retry_next_cycle")
            return

        robot_x, robot_y, _ = pose
        if not self._nav_failure_backoff_elapsed():
            return

        self.current_goal = self.frontier_goals.pop(0)
        goal_x, goal_y, _goal_cell = self.current_goal
        self.goal_history.append(
            math.atan2(goal_y - robot_y, goal_x - robot_x)
        )

        # Build PoseStamped for Nav2
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        qx, qy, qz, qw = yaw_to_quaternion(0.0)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self._nav_goal_active = True
        self._nav_goal_kind = "frontier"
        self._publish_status(
            f"sending_nav2_goal x={goal_x:.2f} y={goal_y:.2f}"
        )
        send_future = self.nav_action_client.send_goal_async(
            goal, feedback_callback=self._nav_feedback_callback
        )
        send_future.add_done_callback(self._nav_goal_response_callback)

    # ── Nav2 action callbacks ────────────────────────────────────────────

    def _nav_goal_response_callback(self, future) -> None:
        """Handle Nav2 goal acceptance/rejection."""
        self._nav_goal_active = False
        if self._nav_goal_kind == "forced":
            self._forced_goal_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Nav2 goal send failed: {exc}")
            self._handle_nav_failure()
            return

        if goal_handle is None or not goal_handle.accepted:
            self._publish_status("nav2_goal_rejected")
            self._handle_nav_failure()
            return

        self._nav_goal_handle = goal_handle
        self._nav_goal_active = True
        if self._nav_goal_kind == "forced":
            self._forced_goal_active = True
        self._publish_status("nav2_goal_accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)

    def _nav_result_callback(self, future) -> None:
        """Handle Nav2 goal completion."""
        self._nav_goal_active = False
        self._forced_goal_active = False
        self._forced_goal_pending = False
        self._nav_goal_handle = None
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Nav2 result failed: {exc}")
            self._handle_nav_failure()
            return

        status_text = goal_status_to_text(result.status)
        if status_text == "succeeded":
            self.current_goal = None
            self.last_progress_pose = None
            self._nav_goal_kind = "frontier"
            self._publish_status("nav2_frontier_reached")
            # Immediately try next frontier if available
            pose = self._lookup_pose()
            if pose is not None and self.frontier_goals:
                self._pick_next_goal(pose)
        else:
            self._publish_status(f"nav2_goal_{status_text}")
            self._handle_nav_failure()

    def _nav_feedback_callback(self, feedback_msg) -> None:
        """Nav2 distance feedback — for logging only."""
        feedback = feedback_msg.feedback
        distance = getattr(feedback, "distance_remaining", float("nan"))
        if not math.isnan(distance):
            self.get_logger().debug(f"Nav2 distance remaining: {distance:.2f} m")

    def _handle_nav_failure(self) -> None:
        """Mark current goal as blocked and try the next one."""
        self._last_nav_failure_time = self.get_clock().now()
        if self.current_goal is not None:
            cell = self.current_goal[2]
            self.goal_failure_counts[cell] = self.goal_failure_counts.get(cell, 0) + 1
            if self.goal_failure_counts[cell] >= self.max_goal_failures:
                self.blocked_goals.add(cell)
        self.current_goal = None
        self._nav_goal_kind = "frontier"
        # Try next frontier
        pose = self._lookup_pose()
        if pose is not None and self.frontier_goals:
            self._pick_next_goal(pose)
            return

        if pose is not None and self.latest_map is not None:
            self._publish_status("nav_failed_no_candidates_rechecking_frontiers")
            self._update_coverage(pose)
            frontiers = self._detect_frontiers(pose)
            if frontiers:
                self.frontier_goals = frontiers
                self.no_frontier_streak = 0
                self.clear_frontier_rounds = 0
                self._pick_next_goal(pose)

    def _nav_failure_backoff_elapsed(self) -> bool:
        if self._last_nav_failure_time is None:
            return True
        elapsed = (
            self.get_clock().now() - self._last_nav_failure_time
        ).nanoseconds / 1e9
        if elapsed >= self.nav_failure_backoff:
            return True
        self._publish_status(
            f"nav_failure_backoff {elapsed:.1f}/{self.nav_failure_backoff:.1f}s"
        )
        return False

    # ── Legacy PID navigation (kept for INITIAL_SPIN only, unused in EXPLORING) ──

    def _navigate_to_goal(self, pose: Tuple[float, float, float]) -> bool:
        assert self.current_goal is not None
        robot_x, robot_y, yaw = pose
        goal_x, goal_y, _goal_cell = self.current_goal
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        distance = math.hypot(dx, dy)

        if distance < self.goal_tolerance:
            return True

        if self._is_stuck(robot_x, robot_y):
            self.recovering = True
            self.recovery_start_time = self.get_clock().now()
            self._publish_status(f"recovering attempt={self.recovery_attempts + 1}")
            return False

        target_angle = math.atan2(dy, dx)
        angle_error = normalize_angle(target_angle - yaw)
        cmd = Twist()

        if abs(angle_error) > self.angle_threshold:
            cmd.angular.z = clamp(angle_error * 1.5, -0.8, 0.8)
        elif self._front_obstacle_close():
            gap = self._find_narrowest_gap()
            if gap is not None and self._gap_wide_enough(gap):
                gap_angle, _gap_width = gap
                cmd.linear.x = self.narrow_pass_speed
                cmd.angular.z = clamp(gap_angle * 2.0, -0.8, 0.8)
            else:
                cmd.angular.z = self.obstacle_avoid_angular
        else:
            cmd.linear.x = min(self.exploration_speed, distance * 0.5)
            cmd.angular.z = clamp(angle_error * 0.5, -0.3, 0.3)

        self.cmd_pub.publish(cmd)
        return False

    def _run_recovery(self) -> None:
        if self.recovery_start_time is None:
            self.recovery_start_time = self.get_clock().now()

        elapsed = (self.get_clock().now() - self.recovery_start_time).nanoseconds / 1e9
        if elapsed < self.recovery_duration:
            cmd = Twist()
            cmd.linear.x = -0.05
            cmd.angular.z = 0.7
            self.cmd_pub.publish(cmd)
            return

        self.recovering = False
        self.recovery_start_time = None
        self.recovery_attempts += 1
        self.last_progress_pose = None
        self.last_progress_time = self.get_clock().now()

        if self.recovery_attempts >= self.max_recovery_attempts:
            if self.current_goal is not None:
                self.blocked_goals.add(self.current_goal[2])
            self.current_goal = None
            self.recovery_attempts = 0
            self._publish_stop()
            self._publish_status("frontier_blocked_selecting_next")
            return

        self._publish_status(f"recovery_done retry_navigation attempt={self.recovery_attempts}")

    def _detect_frontiers(self, pose: Tuple[float, float, float]) -> List[FrontierGoal]:
        if self.latest_map is None:
            return []

        grid = self.latest_map
        width = grid.info.width
        height = grid.info.height
        resolution = float(grid.info.resolution)
        if width == 0 or height == 0 or resolution <= 0.0:
            return []

        reachable = self._reachable_free_cells_from_pose(grid, pose)
        if not reachable:
            return []

        frontier_cells = []
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if (x, y) not in reachable:
                    continue
                value = grid.data[y * width + x]
                if not (0 <= value <= self.free_threshold):
                    continue
                if (
                    grid.data[y * width + x + 1] == self.unknown_value
                    or grid.data[y * width + x - 1] == self.unknown_value
                    or grid.data[(y + 1) * width + x] == self.unknown_value
                    or grid.data[(y - 1) * width + x] == self.unknown_value
                ):
                    frontier_cells.append((x, y))

        components = self._connected_components(set(frontier_cells))
        min_cells = max(3, int(self.frontier_min_area / (resolution * resolution)))
        raw_candidates = []
        for component in components:
            if len(component) < min_cells:
                continue
            centroid = self._choose_centroid_cell(component)
            if centroid not in reachable or not self._is_cell_safe_for_robot(
                grid, centroid
            ):
                centroid = self._nearest_safe_reachable_cell(
                    grid, centroid, reachable, component
                )
                if centroid is None:
                    continue
            if centroid in self.blocked_goals:
                continue
            world_x, world_y = self._grid_to_world(centroid, grid)
            raw_candidates.append((world_x, world_y, centroid, len(component)))

        if not raw_candidates:
            return []

        gains = [
            self._estimate_information_gain((candidate[0], candidate[1]), grid)
            for candidate in raw_candidates
        ]
        max_gain = max(gains) if gains else 1
        max_size = max(candidate[3] for candidate in raw_candidates)
        max_distance = max(
            math.hypot(candidate[0] - pose[0], candidate[1] - pose[1])
            for candidate in raw_candidates
        )
        scored = []
        for candidate, gain in zip(raw_candidates, gains):
            goal = (candidate[0], candidate[1], candidate[2])
            score = self._score_frontier(
                goal,
                candidate[3],
                gain,
                max(max_gain, 1),
                max(max_size, 1),
                max(max_distance, 1e-6),
                pose,
            )
            scored.append((score, goal))
        scored.sort(key=lambda item: item[0], reverse=True)
        return [goal for _score, goal in scored]

    def _score_frontier(
        self,
        goal: FrontierGoal,
        cell_count: int,
        info_gain: int,
        max_info_gain: int,
        max_size: int,
        max_distance: float,
        pose: Tuple[float, float, float],
    ) -> float:
        robot_x, robot_y, _ = pose
        distance = math.hypot(goal[0] - robot_x, goal[1] - robot_y)
        info_score = info_gain / float(max_info_gain)
        distance_score = 1.0 - clamp(distance / max_distance, 0.0, 1.0)
        size_score = cell_count / float(max_size)
        history_penalty = self._history_penalty(goal, pose)
        return (
            self.score_weight_info * info_score
            + self.score_weight_dist * distance_score
            + self.score_weight_size * size_score
            - self.score_weight_history * history_penalty
        )

    def _history_penalty(
        self, goal: FrontierGoal, pose: Tuple[float, float, float]
    ) -> float:
        if not self.goal_history:
            return 0.0

        angle = math.atan2(goal[1] - pose[1], goal[0] - pose[0])
        min_diff = min(abs(normalize_angle(angle - history)) for history in self.goal_history)
        return 1.0 - clamp(min_diff / math.pi, 0.0, 1.0)

    def _estimate_information_gain(
        self, world_xy: Tuple[float, float], grid: OccupancyGrid
    ) -> int:
        cell = self._world_to_grid(world_xy, grid)
        if cell is None:
            return 0

        width = grid.info.width
        height = grid.info.height
        resolution = float(grid.info.resolution)
        radius_cells = max(1, int(self.laser_max_range / max(resolution, 1e-6)))
        cx, cy = cell
        gain = 0
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                gx = cx + dx
                gy = cy + dy
                if gx < 0 or gy < 0 or gx >= width or gy >= height:
                    continue
                if grid.data[gy * width + gx] != self.unknown_value:
                    continue
                if self._has_free_neighbor(grid, gx, gy, width, height):
                    gain += 1
        return gain

    def _connected_components(self, cells: Set[GridCell]) -> List[List[GridCell]]:
        remaining = set(cells)
        components: List[List[GridCell]] = []
        neighbor_offsets = (
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1),
        )

        while remaining:
            start = remaining.pop()
            component = [start]
            queue: Deque[GridCell] = deque([start])
            while queue:
                x, y = queue.popleft()
                for dx, dy in neighbor_offsets:
                    neighbor = (x + dx, y + dy)
                    if neighbor in remaining:
                        remaining.remove(neighbor)
                        component.append(neighbor)
                        queue.append(neighbor)
            components.append(component)
        return components

    def _choose_centroid_cell(self, cells: List[GridCell]) -> GridCell:
        avg_x = sum(cell[0] for cell in cells) / len(cells)
        avg_y = sum(cell[1] for cell in cells) / len(cells)
        return min(cells, key=lambda cell: (cell[0] - avg_x) ** 2 + (cell[1] - avg_y) ** 2)

    def _grid_to_world(self, cell: GridCell, grid: OccupancyGrid) -> Tuple[float, float]:
        x, y = cell
        origin = grid.info.origin.position
        resolution = float(grid.info.resolution)
        return (
            float(origin.x + (x + 0.5) * resolution),
            float(origin.y + (y + 0.5) * resolution),
        )

    def _world_to_grid(
        self, world_xy: Tuple[float, float], grid: OccupancyGrid
    ) -> Optional[GridCell]:
        origin = grid.info.origin.position
        resolution = float(grid.info.resolution)
        if resolution <= 0.0:
            return None
        x = int((world_xy[0] - origin.x) / resolution)
        y = int((world_xy[1] - origin.y) / resolution)
        if x < 0 or y < 0 or x >= grid.info.width or y >= grid.info.height:
            return None
        return x, y

    def _reachable_free_cells_from_pose(
        self, grid: OccupancyGrid, pose: Tuple[float, float, float]
    ) -> Set[GridCell]:
        start = self._world_to_grid((pose[0], pose[1]), grid)
        if start is None:
            return set()
        return self._reachable_free_cells_from_cell(grid, start)

    def _reachable_free_cells_from_cell(
        self, grid: OccupancyGrid, start: GridCell
    ) -> Set[GridCell]:
        width = grid.info.width
        height = grid.info.height
        if start[0] < 0 or start[1] < 0 or start[0] >= width or start[1] >= height:
            return set()

        if not self._is_free_cell(grid, start):
            nearest = self._nearest_free_cell(grid, start)
            if nearest is None:
                return set()
            start = nearest

        visited: Set[GridCell] = {start}
        queue: Deque[GridCell] = deque([start])
        while queue:
            x, y = queue.popleft()
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                neighbor = (x + dx, y + dy)
                if neighbor in visited:
                    continue
                if not self._is_free_cell(grid, neighbor):
                    continue
                visited.add(neighbor)
                queue.append(neighbor)
        return visited

    def _nearest_free_cell(
        self, grid: OccupancyGrid, start: GridCell
    ) -> Optional[GridCell]:
        width = grid.info.width
        height = grid.info.height
        best_cell = None
        best_dist = float("inf")
        sx, sy = start
        for y in range(height):
            for x in range(width):
                cell = (x, y)
                if not self._is_free_cell(grid, cell):
                    continue
                dist = (x - sx) ** 2 + (y - sy) ** 2
                if dist < best_dist:
                    best_dist = dist
                    best_cell = cell
        return best_cell

    def _is_free_cell(self, grid: OccupancyGrid, cell: GridCell) -> bool:
        x, y = cell
        if x < 0 or y < 0 or x >= grid.info.width or y >= grid.info.height:
            return False
        value = grid.data[y * grid.info.width + x]
        return 0 <= value <= self.free_threshold

    def _is_cell_safe_for_robot(
        self, grid: OccupancyGrid, cell: GridCell, safety_radius_m: Optional[float] = None
    ) -> bool:
        if not self._is_free_cell(grid, cell):
            return False
        radius_m = self.goal_safety_radius if safety_radius_m is None else safety_radius_m
        resolution = float(grid.info.resolution)
        if resolution <= 0.0:
            return False
        radius_cells = max(1, int(math.ceil(radius_m / resolution)))
        cx, cy = cell
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                nx = cx + dx
                ny = cy + dy
                if nx < 0 or ny < 0 or nx >= grid.info.width or ny >= grid.info.height:
                    return False
                value = grid.data[ny * grid.info.width + nx]
                if value >= self.free_threshold + 1 and value != self.unknown_value:
                    return False
        return True

    def _nearest_safe_reachable_cell(
        self,
        grid: OccupancyGrid,
        target: GridCell,
        reachable: Set[GridCell],
        candidates: Optional[List[GridCell]] = None,
    ) -> Optional[GridCell]:
        search_cells = candidates if candidates is not None else list(reachable)
        tx, ty = target
        best_cell = None
        best_dist = float("inf")
        for cell in search_cells:
            if cell not in reachable or cell in self.blocked_goals:
                continue
            if not self._is_cell_safe_for_robot(grid, cell):
                continue
            dist = (cell[0] - tx) ** 2 + (cell[1] - ty) ** 2
            if dist < best_dist:
                best_dist = dist
                best_cell = cell
        return best_cell

    def _has_free_neighbor(
        self, grid: OccupancyGrid, x: int, y: int, width: int, height: int
    ) -> bool:
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx = x + dx
            ny = y + dy
            if nx < 0 or ny < 0 or nx >= width or ny >= height:
                continue
            value = grid.data[ny * width + nx]
            if 0 <= value <= self.free_threshold:
                return True
        return False

    def _update_coverage(self, pose: Tuple[float, float, float]) -> None:
        if self.latest_map is None:
            return

        active_area, explored = self._compute_active_area(pose)
        self.active_area_cells = active_area
        self.explored_cells = explored
        self.coverage = explored / float(max(active_area, 1))
        self.coverage_history.append(self.coverage)
        self._publish_coverage()
        self._publish_status(
            "coverage "
            f"{self.coverage * 100.0:.1f}% "
            f"explored={self.explored_cells} active={self.active_area_cells}"
        )

    def _compute_active_area(self, pose: Tuple[float, float, float]) -> Tuple[int, int]:
        assert self.latest_map is not None
        grid = self.latest_map
        width = grid.info.width
        height = grid.info.height
        active_cells: Set[GridCell] = set()
        explored = 0
        free_cells: List[GridCell] = []
        for y in range(height):
            for x in range(width):
                value = grid.data[y * width + x]
                if 0 <= value <= self.free_threshold:
                    cell = (x, y)
                    free_cells.append(cell)
                    active_cells.add(cell)
                    explored += 1

        if not free_cells:
            return self._compute_active_area_bfs_fallback(pose)

        for x, y in free_cells:
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx = x + dx
                ny = y + dy
                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    continue
                if grid.data[ny * width + nx] == self.unknown_value:
                    active_cells.add((nx, ny))

        if not active_cells:
            return self._compute_active_area_bfs_fallback(pose)
        return len(active_cells), explored

    def _compute_active_area_bfs_fallback(
        self, pose: Tuple[float, float, float]
    ) -> Tuple[int, int]:
        assert self.latest_map is not None
        grid = self.latest_map
        width = grid.info.width
        height = grid.info.height
        start = self._world_to_grid((pose[0], pose[1]), grid)
        if start is None:
            return 0, 0

        if not self._is_free_cell(grid, start):
            nearest = self._nearest_free_cell(grid, start)
            if nearest is None:
                return 0, 0
            start = nearest

        visited = {start}
        queue: Deque[GridCell] = deque([start])
        active_area = 0
        explored = 0
        while queue:
            x, y = queue.popleft()
            value = grid.data[y * width + x]
            if value == self.unknown_value or 0 <= value <= self.free_threshold:
                active_area += 1
                if 0 <= value <= self.free_threshold:
                    explored += 1
            else:
                continue

            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx = x + dx
                ny = y + dy
                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    continue
                neighbor = (nx, ny)
                if neighbor in visited:
                    continue
                neighbor_value = grid.data[ny * width + nx]
                if neighbor_value >= self.free_threshold + 1:
                    continue
                visited.add(neighbor)
                queue.append(neighbor)
        return active_area, explored

    def _check_stagnation(self) -> None:
        if len(self.coverage_history) < self.stagnation_window:
            return
        recent_max = max(self.coverage_history)
        recent_min = min(self.coverage_history)
        if (
            recent_max - recent_min < self.stagnation_coverage_threshold
            and self.coverage < self.min_coverage_for_done
            and not self._nav_goal_active
            and not self._forced_goal_pending
        ):
            if self._trigger_forced_exploration():
                self.coverage_history.clear()
                self.no_frontier_streak = 0
                self.clear_frontier_rounds = 0

    def _trigger_forced_exploration(self) -> bool:
        if self.latest_map is None or self._nav_goal_active or self._forced_goal_pending:
            return False
        if not self._nav_failure_backoff_elapsed():
            return False

        grid = self.latest_map
        width = grid.info.width
        height = grid.info.height
        pose = self._lookup_pose()
        if pose is None:
            return False
        reachable = self._reachable_free_cells_from_pose(grid, pose)
        if not reachable:
            return False

        boundary_free = []
        for x, y in reachable:
            if x <= 0 or y <= 0 or x >= width - 1 or y >= height - 1:
                continue
            if not self._is_cell_safe_for_robot(grid, (x, y)):
                continue
            if self._has_unknown_neighbor(grid, x, y, width, height):
                boundary_free.append((x, y))

        clusters = self._connected_components(set(boundary_free))
        clusters = [cluster for cluster in clusters if cluster]
        if not clusters:
            return False

        clusters.sort(key=len, reverse=True)
        centroid = None
        largest_size = 0
        for cluster in clusters:
            largest_size = max(largest_size, len(cluster))
            candidate = self._choose_centroid_cell(cluster)
            if (
                candidate in reachable
                and candidate not in self.blocked_goals
                and self._is_cell_safe_for_robot(grid, candidate)
            ):
                centroid = candidate
                largest_size = len(cluster)
                break
            centroid = self._nearest_safe_reachable_cell(
                grid, candidate, reachable, cluster
            )
            if centroid is not None:
                largest_size = len(cluster)
                break
        if centroid is None:
            return False

        world_x, world_y = self._grid_to_world(centroid, grid)
        self.current_goal = (world_x, world_y, centroid)
        self.frontier_goals = []
        self.recovering = False
        self.recovery_attempts = 0
        self.last_progress_pose = None
        self._publish_status(f"forced_explore boundary_free_cluster={largest_size}cells")

        # Send forced exploration as Nav2 goal
        if not self.nav_action_client.wait_for_server(timeout_sec=1.0):
            self._publish_status("forced_explore_nav_action_server_unavailable")
            return False

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = world_x
        goal.pose.pose.position.y = world_y
        qx, qy, qz, qw = yaw_to_quaternion(0.0)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        self._nav_goal_active = True
        self._forced_goal_pending = True
        self._nav_goal_kind = "forced"
        send_future = self.nav_action_client.send_goal_async(goal)
        send_future.add_done_callback(self._nav_goal_response_callback)

        return True

    def _has_unknown_neighbor(
        self, grid: OccupancyGrid, x: int, y: int, width: int, height: int
    ) -> bool:
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx = x + dx
            ny = y + dy
            if nx < 0 or ny < 0 or nx >= width or ny >= height:
                continue
            if grid.data[ny * width + nx] == self.unknown_value:
                return True
        return False

    def _is_stuck(self, robot_x: float, robot_y: float) -> bool:
        if self.last_progress_pose is None:
            self.last_progress_pose = (robot_x, robot_y)
            self.last_progress_time = self.get_clock().now()
            return False

        last_x, last_y = self.last_progress_pose
        moved = math.hypot(robot_x - last_x, robot_y - last_y)
        now = self.get_clock().now()
        elapsed = (now - self.last_progress_time).nanoseconds / 1e9
        if moved >= self.stuck_distance:
            self.last_progress_pose = (robot_x, robot_y)
            self.last_progress_time = now
            return False
        return elapsed > self.stuck_timeout

    def _front_obstacle_close(self) -> bool:
        if self.latest_scan is None or not self.latest_scan.ranges:
            return False

        front_ranges = []
        for index, distance in enumerate(self.latest_scan.ranges):
            if math.isinf(distance) or math.isnan(distance):
                continue
            angle = self.latest_scan.angle_min + index * self.latest_scan.angle_increment
            if abs(angle) <= math.radians(25.0):
                front_ranges.append(distance)
        return bool(front_ranges) and min(front_ranges) < self.front_obstacle_distance

    def _find_narrowest_gap(self) -> Optional[Tuple[float, float]]:
        if self.latest_scan is None or not self.latest_scan.ranges:
            return None

        gaps = []
        ranges = self.latest_scan.ranges
        for index in range(len(ranges) - 1):
            r1 = ranges[index]
            r2 = ranges[index + 1]
            if math.isinf(r1) or math.isinf(r2) or math.isnan(r1) or math.isnan(r2):
                continue
            angle1 = self.latest_scan.angle_min + index * self.latest_scan.angle_increment
            angle2 = self.latest_scan.angle_min + (index + 1) * self.latest_scan.angle_increment
            if abs((angle1 + angle2) * 0.5) > math.radians(60.0):
                continue
            gap_width = math.sqrt(max(0.0, r1 * r1 + r2 * r2 - 2.0 * r1 * r2 * math.cos(angle2 - angle1)))
            if gap_width > self.robot_width:
                gaps.append(((angle1 + angle2) * 0.5, gap_width))

        if not gaps:
            return None
        return max(gaps, key=lambda gap: gap[1])

    def _gap_wide_enough(self, gap: Tuple[float, float]) -> bool:
        _angle, width = gap
        return width >= self.robot_width * self.narrow_gap_multiplier

    def _publish_progress(self) -> None:
        msg = Float32()
        if self.latest_map is None or self.initial_unknown_count in (None, 0):
            msg.data = 0.0
        else:
            unknown = sum(1 for value in self.latest_map.data if value == self.unknown_value)
            explored = max(0, self.initial_unknown_count - unknown)
            msg.data = clamp(explored / float(self.initial_unknown_count), 0.0, 1.0)
        self.progress_pub.publish(msg)

    def _publish_coverage(self) -> None:
        msg = Float32()
        msg.data = float(clamp(self.coverage, 0.0, 1.0))
        self.coverage_pub.publish(msg)

    def _publish_status(self, text: str) -> None:
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _complete_exploration(self, text: str) -> None:
        self.state = ExplorerState.DONE
        self.frontier_goals = []
        self.current_goal = None
        self.recovering = False
        self.done_stop_published = False  # allow one final stop publish

        # Cancel any active Nav2 goal
        if self._nav_goal_handle is not None:
            cancel_future = self._nav_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda _f: None)
            self._nav_goal_handle = None
        self._nav_goal_active = False
        self._forced_goal_active = False
        self._forced_goal_pending = False
        self._nav_goal_kind = "frontier"

        self._publish_stop()
        self.done_stop_published = True   # prevent continuous zero-vel publishing
        self._publish_progress()
        self._publish_status(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutoExplorer()
    try:
        rclpy.spin(node)
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
