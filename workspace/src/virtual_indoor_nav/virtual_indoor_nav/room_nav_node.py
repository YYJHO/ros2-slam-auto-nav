#!/usr/bin/env python3
import math
from collections import deque
from pathlib import Path
import os
from typing import Any, Deque, Dict, List, Optional, Tuple

from action_msgs.msg import GoalStatus
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


GridCell = Tuple[int, int]
RoomRecord = Dict[str, Any]


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


class RoomNavNode(Node):
    def __init__(self) -> None:
        super().__init__("room_nav_node")
        default_rooms_file = os.environ.get(
            "VIRTUAL_INDOOR_NAV_ROOMS_FILE",
            str(Path(__file__).resolve().parents[4] / "runtime" / "rooms.yaml"),
        )
        self.declare_parameter("rooms_file", default_rooms_file)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("command_topic", "/room_command")
        self.declare_parameter("status_topic", "/room_status")
        self.declare_parameter("navigation_action", "navigate_to_pose")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("free_threshold", 20)
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("room_detection_inflation_radius", 0.35)
        self.declare_parameter("room_min_area_m2", 3.0)
        self.declare_parameter("goal_snap_radius", 2.0)
        self.declare_parameter("goal_snap_step", 0.5)
        self.declare_parameter("goal_safety_radius", 0.22)
        self.declare_parameter("final_goal_safety_radius", 0.35)
        self.declare_parameter("robot_snap_radius", 0.5)
        self.declare_parameter("route_waypoint_spacing", 0.9)
        self.declare_parameter("route_waypoint_min_distance", 1.8)
        self.declare_parameter("route_waypoint_max_count", 20)

        self.rooms_file = Path(self.get_parameter("rooms_file").value)
        self.map_frame = self.get_parameter("map_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.command_topic = self.get_parameter("command_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.navigation_action = self.get_parameter("navigation_action").value
        self.map_topic = self.get_parameter("map_topic").value
        self.free_threshold = int(self.get_parameter("free_threshold").value)
        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self.room_detection_inflation_radius = float(
            self.get_parameter("room_detection_inflation_radius").value
        )
        self.room_min_area_m2 = float(self.get_parameter("room_min_area_m2").value)
        self.goal_snap_radius = float(self.get_parameter("goal_snap_radius").value)
        self.goal_snap_step = float(self.get_parameter("goal_snap_step").value)
        self.goal_safety_radius = float(
            self.get_parameter("goal_safety_radius").value
        )
        self.final_goal_safety_radius = float(
            self.get_parameter("final_goal_safety_radius").value
        )
        self.robot_snap_radius = float(
            self.get_parameter("robot_snap_radius").value
        )
        self.route_waypoint_spacing = float(
            self.get_parameter("route_waypoint_spacing").value
        )
        self.route_waypoint_min_distance = float(
            self.get_parameter("route_waypoint_min_distance").value
        )
        self.route_waypoint_max_count = int(
            self.get_parameter("route_waypoint_max_count").value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, NavigateToPose, self.navigation_action)

        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.command_sub = self.create_subscription(
            String, self.command_topic, self.command_callback, 10
        )
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self._map_callback, map_qos
        )

        self._goal_handle = None
        self._goal_pending = False
        self._cancel_requested = False
        self._active_goal_room: Optional[str] = None
        self._active_goal_final: Optional[Tuple[float, float, float]] = None
        self._active_route: Deque[Tuple[float, float, float]] = deque()
        self._active_route_total = 0
        self._last_feedback_log_time = 0.0
        self._last_feedback_distance: Optional[float] = None
        self._latest_map: Optional[OccupancyGrid] = None
        self._exploration_active = False
        self._exploration_queue: Deque[str] = deque()
        self._exploration_results: List[Tuple[str, str]] = []
        self.rooms: Dict[str, RoomRecord] = {}
        self._load_rooms()
        self._publish_status(
            "room_nav_node ready. Commands: save <name>, goto <name>, delete <name>, "
            "rename <old_name> <new_name>, set <name> <x> <y> [yaw], "
            "list, auto_rooms, explore_rooms, cancel"
        )

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

    def _load_rooms(self) -> None:
        if not self.rooms_file.exists():
            self.rooms_file.parent.mkdir(parents=True, exist_ok=True)
            self.rooms_file.write_text("rooms: []\n", encoding="utf-8")
        data = yaml.safe_load(self.rooms_file.read_text(encoding="utf-8")) or {}
        self.rooms = {}
        for item in data.get("rooms", []):
            name = item.get("name")
            if not name:
                continue
            room: RoomRecord = {
                "x": float(item.get("x", 0.0)),
                "y": float(item.get("y", 0.0)),
                "yaw": float(item.get("yaw", 0.0)),
                "source": item.get("source", "manual"),
            }
            for field in ("area_m2", "cell_count", "explore_order"):
                if field in item:
                    room[field] = item[field]
            self.rooms[name] = room

    def _save_rooms(self) -> None:
        serialized_rooms = []
        for name, room in sorted(self.rooms.items(), key=lambda item: item[0]):
            payload: RoomRecord = {"name": name, "x": room["x"], "y": room["y"], "yaw": room["yaw"]}
            if room.get("source"):
                payload["source"] = room["source"]
            if "area_m2" in room:
                payload["area_m2"] = round(float(room["area_m2"]), 3)
            if "cell_count" in room:
                payload["cell_count"] = int(room["cell_count"])
            if "explore_order" in room:
                payload["explore_order"] = int(room["explore_order"])
            serialized_rooms.append(payload)

        payload = {"rooms": serialized_rooms}
        self.rooms_file.parent.mkdir(parents=True, exist_ok=True)
        self.rooms_file.write_text(
            yaml.safe_dump(payload, allow_unicode=True, sort_keys=False),
            encoding="utf-8",
        )

    def _lookup_current_pose(self) -> RoomRecord:
        transform = self.tf_buffer.lookup_transform(
            self.map_frame,
            self.robot_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=1.0),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
        return {"x": float(translation.x), "y": float(translation.y), "yaw": float(yaw)}

    def command_callback(self, msg: String) -> None:
        command = msg.data.strip()
        if not command:
            self._publish_status("Ignored empty room command.")
            return

        if command == "list":
            self._list_rooms()
            return

        if command == "cancel":
            self._cancel_goal()
            return

        if command == "auto_rooms":
            self._auto_detect_rooms()
            return

        if command == "explore_rooms":
            self._explore_rooms()
            return

        if command.startswith("save "):
            self._save_current_room(command[5:].strip())
            return

        if command.startswith("goto "):
            self._goto_room(command[5:].strip())
            return

        if command.startswith("delete "):
            self._delete_room(command[7:].strip())
            return

        if command.startswith("rename "):
            self._rename_room(command[7:].strip())
            return

        if command.startswith("set "):
            self._set_room(command[4:].strip())
            return

        self._publish_status(f"Unknown room command: {command}")

    def _list_rooms(self) -> None:
        if not self.rooms:
            self._publish_status("Known rooms: (empty)")
            return
        items = []
        for name in sorted(self.rooms):
            source = self.rooms[name].get("source", "manual")
            items.append(f"{name}[{source}]")
        self._publish_status(f"Known rooms: {', '.join(items)}")

    def _save_current_room(self, room_name: str) -> None:
        if not room_name:
            self._publish_status("Room name is empty.")
            return
        try:
            pose = self._lookup_current_pose()
        except TransformException as exc:
            self._publish_status(f"Failed to save room {room_name}: {exc}")
            return
        pose["source"] = "manual"
        self.rooms[room_name] = pose
        self._save_rooms()
        self._publish_status(
            f"Saved room {room_name} at x={pose['x']:.2f}, y={pose['y']:.2f}, yaw={pose['yaw']:.2f}"
        )

    def _delete_room(self, room_name: str) -> None:
        if room_name not in self.rooms:
            self._publish_status(f"Room not found: {room_name}")
            return
        del self.rooms[room_name]
        self._save_rooms()
        self._publish_status(f"Deleted room: {room_name}")

    def _rename_room(self, args: str) -> None:
        parts = args.split(maxsplit=1)
        if len(parts) != 2:
            self._publish_status("Usage: rename <old_name> <new_name>")
            return

        old_name, new_name = parts[0].strip(), parts[1].strip()
        if not old_name or not new_name:
            self._publish_status("Usage: rename <old_name> <new_name>")
            return
        if old_name not in self.rooms:
            self._publish_status(f"Room not found: {old_name}")
            return
        if new_name in self.rooms:
            self._publish_status(f"Room already exists: {new_name}")
            return

        self.rooms[new_name] = self.rooms.pop(old_name)
        self._save_rooms()
        self._publish_status(f"Renamed {old_name} to {new_name}")

    def _set_room(self, args: str) -> None:
        """set <name> <x> <y> [yaw] — add/update room from map click coordinates."""
        parts = args.split()
        if len(parts) < 3:
            self._publish_status("Usage: set <name> <x> <y> [yaw]")
            return

        name = parts[0].strip()
        try:
            x = float(parts[1])
            y = float(parts[2])
            yaw = float(parts[3]) if len(parts) >= 4 else 0.0
        except ValueError:
            self._publish_status("Invalid coordinates. Usage: set <name> <x> <y> [yaw]")
            return

        if not name:
            self._publish_status("Room name cannot be empty.")
            return

        room: RoomRecord = {
            "x": x,
            "y": y,
            "yaw": yaw,
            "source": "map_click",
        }

        if name in self.rooms:
            self._publish_status(
                f"Updating room {name}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
            )
        else:
            self._publish_status(
                f"Set room {name} at x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
            )

        self.rooms[name] = room
        self._save_rooms()

    def _goto_room(self, room_name: str) -> None:
        # Reload rooms to pick up any changes from App set command
        self._load_rooms()
        if room_name not in self.rooms:
            self._publish_status(f"Room not found: {room_name}")
            return
        if self._exploration_queue:
            self._publish_status("Exploration is in progress. Send cancel before goto.")
            return
        if self._goal_handle is not None or self._goal_pending:
            self._publish_status("A navigation goal is already active.")
            return
        self._send_navigation_goal(room_name)

    def _send_navigation_goal(self, room_name: str) -> None:
        room_pose = self.rooms[room_name]

        ok, goal_x, goal_y, waypoints = self._prepare_route_to_goal(
            room_name, room_pose["x"], room_pose["y"]
        )
        if not ok:
            self._publish_status(
                f"Navigation failed: room target ({room_pose['x']:.2f}, "
                f"{room_pose['y']:.2f}) is not on reachable free space."
            )
            if self._exploration_active:
                self._register_navigation_result(room_name, "unreachable")
            return

        self._active_goal_room = room_name
        self._active_goal_final = (goal_x, goal_y, float(room_pose["yaw"]))
        self._active_route = deque(waypoints)
        self._active_route_total = len(waypoints)
        self._send_next_route_goal()

    def _send_next_route_goal(self) -> None:
        if self._active_goal_final is None:
            self._publish_status("Navigation failed: no active route target.")
            return

        self._publish_status("Waiting for navigate_to_pose action server...")
        if not self.action_client.wait_for_server(timeout_sec=30.0):
            self._publish_status(
                "Navigation failed: navigate_to_pose action server is not available."
            )
            room_name = self._active_goal_room or "(unknown)"
            self._goal_pending = False
            self._register_navigation_result(room_name, "server_unavailable")
            return

        if self._active_route:
            goal_x, goal_y, yaw = self._active_route.popleft()
            route_index = self._active_route_total - len(self._active_route)
            route_label = f" waypoint {route_index}/{self._active_route_total}"
        else:
            goal_x, goal_y, yaw = self._active_goal_final
            route_label = ""

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        room_name = self._active_goal_room or "(unknown)"
        self._publish_status(f"Sending navigation goal to room: {room_name}{route_label}")
        self._goal_pending = True
        self._last_feedback_log_time = 0.0
        self._last_feedback_distance = None
        send_goal_future = self.action_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        room_name = self._active_goal_room or "(unknown)"
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._goal_pending = False
            self._goal_handle = None
            self._register_navigation_result(room_name, f"send_failed:{exc}")
            return
        if goal_handle is None or not goal_handle.accepted:
            self._goal_pending = False
            self._goal_handle = None
            self._register_navigation_result(room_name, "rejected")
            return
        self._goal_pending = False
        self._goal_handle = goal_handle
        if self._cancel_requested:
            self._publish_status(f"Cancel requested for room: {room_name}")
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)
            return
        self._publish_status(f"Navigation goal accepted for room: {room_name}")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future) -> None:
        room_name = self._active_goal_room or "(unknown)"
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self._goal_pending = False
            self._goal_handle = None
            self._register_navigation_result(room_name, f"failed:{exc}")
            return
        status_text = goal_status_to_text(result.status)
        self._goal_pending = False
        self._goal_handle = None
        if status_text == "succeeded" and self._active_route:
            self._send_next_route_goal()
            return
        self._register_navigation_result(room_name, status_text)

    def _register_navigation_result(self, room_name: str, status_text: str) -> None:
        is_exploring = self._exploration_active
        self._goal_pending = False
        self._cancel_requested = False
        self._active_goal_room = None
        self._active_goal_final = None
        self._active_route.clear()
        self._active_route_total = 0
        self._publish_status(
            f"Navigation finished for room {room_name}, status={status_text}"
        )
        if not is_exploring:
            return

        self._exploration_results.append((room_name, status_text))
        if self._exploration_queue:
            self._send_next_exploration_goal()
            return

        succeeded = [name for name, status in self._exploration_results if status == "succeeded"]
        failed = [name for name, status in self._exploration_results if status != "succeeded"]
        summary = f"Exploration completed. Visited: {', '.join(succeeded) if succeeded else '(none)'}"
        if failed:
            summary += f". Failed: {', '.join(failed)}"
        self._exploration_active = False
        self._exploration_results = []
        self._publish_status(summary)

    def _prepare_route_to_goal(
        self, room_name: str, room_x: float, room_y: float
    ) -> Tuple[bool, float, float, List[Tuple[float, float, float]]]:
        """Validate the target on a Nav2-like safe grid and build route waypoints."""
        if self._latest_map is None:
            self._publish_status(
                "Navigation failed: no /map available for target validation."
            )
            return False, room_x, room_y, []

        grid = self._latest_map
        width = grid.info.width
        height = grid.info.height
        resolution = float(grid.info.resolution)
        if width == 0 or height == 0 or resolution <= 0.0:
            self._publish_status(
                "Navigation failed: /map metadata invalid."
            )
            return False, room_x, room_y, []

        target_cell = self._world_to_grid(room_x, room_y, grid)
        if target_cell is None:
            self._publish_status(
                f"Navigation failed: room target ({room_x:.2f},{room_y:.2f}) "
                "is outside the current map."
            )
            return False, room_x, room_y, []

        try:
            current_pose = self._lookup_current_pose()
        except TransformException as exc:
            self._publish_status(f"Navigation failed: current pose unavailable: {exc}")
            return False, room_x, room_y, []

        robot_cell = self._world_to_grid(
            float(current_pose["x"]), float(current_pose["y"]), grid
        )
        if robot_cell is None:
            self._publish_status("Navigation failed: robot is outside the current map.")
            return False, room_x, room_y, []

        safe_cells = self._safe_free_cells(grid, self.goal_safety_radius)
        nearest_robot = self._nearest_cell_from_set(
            grid, robot_cell, safe_cells, self.robot_snap_radius
        )
        if nearest_robot is None:
            self._publish_status(
                "Navigation failed: no Nav2-safe free space near robot pose."
            )
            return False, room_x, room_y, []
        robot_cell = nearest_robot

        parents = self._reachable_safe_cells_with_parents(safe_cells, robot_cell)
        reachable = set(parents)
        if not reachable:
            self._publish_status(
                "Navigation failed: no reachable Nav2-safe free space from robot pose."
            )
            return False, room_x, room_y, []

        snapped_cell = self._nearest_reachable_safe_cell(
            grid,
            target_cell,
            reachable,
            self.goal_snap_radius,
            self.goal_snap_step,
            self.final_goal_safety_radius,
        )
        if snapped_cell is None:
            self._publish_status(
                f"Navigation failed: room target ({room_x:.2f},{room_y:.2f}) "
                "is not reachable on Nav2-safe map."
            )
            return False, room_x, room_y, []

        snapped_x, snapped_y = self._grid_to_world(snapped_cell, grid)
        offset = math.hypot(snapped_x - room_x, snapped_y - room_y)
        if offset > resolution:
            if room_name in self.rooms:
                self.rooms[room_name]["x"] = snapped_x
                self.rooms[room_name]["y"] = snapped_y
                self._save_rooms()
            else:
                self._publish_status(
                    f"Warning: room '{room_name}' not in self.rooms during snap."
                )

            self._publish_status(
                f"Snapped target from ({room_x:.2f},{room_y:.2f}) "
                f"to reachable safe cell ({snapped_x:.2f},{snapped_y:.2f}) "
                f"offset={offset:.2f}m"
            )

        path_cells = self._reconstruct_path(parents, robot_cell, snapped_cell)
        waypoints = self._route_waypoints_from_path(
            grid, path_cells, float(current_pose["x"]), float(current_pose["y"])
        )
        return True, snapped_x, snapped_y, waypoints

    def _world_to_grid(
        self, world_x: float, world_y: float, grid: OccupancyGrid
    ) -> Optional[GridCell]:
        resolution = float(grid.info.resolution)
        if resolution <= 0.0:
            return None
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        gx = math.floor((world_x - origin_x) / resolution)
        gy = math.floor((world_y - origin_y) / resolution)
        if gx < 0 or gy < 0 or gx >= grid.info.width or gy >= grid.info.height:
            return None
        return gx, gy

    def _is_free_cell(self, grid: OccupancyGrid, cell: GridCell) -> bool:
        x, y = cell
        if x < 0 or y < 0 or x >= grid.info.width or y >= grid.info.height:
            return False
        value = grid.data[y * grid.info.width + x]
        return 0 <= value <= self.free_threshold

    def _nearest_free_cell(
        self, grid: OccupancyGrid, start: GridCell
    ) -> Optional[GridCell]:
        width = grid.info.width
        height = grid.info.height
        sx, sy = start
        best_cell = None
        best_dist = float("inf")
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

    def _nearest_cell_from_set(
        self,
        grid: OccupancyGrid,
        start: GridCell,
        cells: set[GridCell],
        max_radius_m: Optional[float] = None,
    ) -> Optional[GridCell]:
        if not cells:
            return None
        if start in cells:
            return start
        sx, sy = start
        best_cell = min(
            cells,
            key=lambda cell: (cell[0] - sx) ** 2 + (cell[1] - sy) ** 2,
        )
        if max_radius_m is not None:
            resolution = float(grid.info.resolution)
            max_radius_cells = max_radius_m / resolution if resolution > 0.0 else 0.0
            distance_cells = math.hypot(best_cell[0] - sx, best_cell[1] - sy)
            if distance_cells > max_radius_cells:
                return None
        return best_cell

    def _safe_free_cells(self, grid: OccupancyGrid, safety_radius_m: float) -> set[GridCell]:
        resolution = float(grid.info.resolution)
        if resolution <= 0.0:
            return set()
        free_cells = {
            (x, y)
            for y in range(grid.info.height)
            for x in range(grid.info.width)
            if self._is_free_cell(grid, (x, y))
        }
        radius_cells = max(0, int(math.ceil(safety_radius_m / resolution)))
        return self._filter_inflated_free_cells(
            free_cells, grid.info.width, grid.info.height, radius_cells
        )

    def _reachable_free_cells(
        self, grid: OccupancyGrid, start: GridCell
    ) -> set[GridCell]:
        if not self._is_free_cell(grid, start):
            nearest = self._nearest_free_cell(grid, start)
            if nearest is None:
                return set()
            start = nearest

        visited = {start}
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

    def _reachable_safe_cells_with_parents(
        self, safe_cells: set[GridCell], start: GridCell
    ) -> Dict[GridCell, Optional[GridCell]]:
        if start not in safe_cells:
            return {}

        parents: Dict[GridCell, Optional[GridCell]] = {start: None}
        queue: Deque[GridCell] = deque([start])
        while queue:
            x, y = queue.popleft()
            for dx, dy in (
                (1, 0),
                (-1, 0),
                (0, 1),
                (0, -1),
                (1, 1),
                (1, -1),
                (-1, 1),
                (-1, -1),
            ):
                neighbor = (x + dx, y + dy)
                if neighbor in parents or neighbor not in safe_cells:
                    continue
                parents[neighbor] = (x, y)
                queue.append(neighbor)
        return parents

    def _reconstruct_path(
        self,
        parents: Dict[GridCell, Optional[GridCell]],
        start: GridCell,
        goal: GridCell,
    ) -> List[GridCell]:
        if goal not in parents:
            return []
        path = [goal]
        current = goal
        while current != start:
            parent = parents.get(current)
            if parent is None:
                break
            path.append(parent)
            current = parent
        path.reverse()
        return path

    def _route_waypoints_from_path(
        self,
        grid: OccupancyGrid,
        path: List[GridCell],
        current_x: float,
        current_y: float,
    ) -> List[Tuple[float, float, float]]:
        if not path:
            return []

        goal_x, goal_y = self._grid_to_world(path[-1], grid)
        direct_distance = math.hypot(goal_x - current_x, goal_y - current_y)
        if direct_distance < self.route_waypoint_min_distance:
            return []

        spacing = max(self.route_waypoint_spacing, float(grid.info.resolution))
        waypoints: List[Tuple[float, float, float]] = []
        last_x = current_x
        last_y = current_y
        min_goal_gap = max(0.6, spacing * 0.75)

        for cell in path[1:-1]:
            world_x, world_y = self._grid_to_world(cell, grid)
            if math.hypot(world_x - last_x, world_y - last_y) < spacing:
                continue
            if math.hypot(goal_x - world_x, goal_y - world_y) < min_goal_gap:
                continue
            yaw = math.atan2(world_y - last_y, world_x - last_x)
            waypoints.append((world_x, world_y, yaw))
            last_x = world_x
            last_y = world_y

        max_waypoints = max(1, self.route_waypoint_max_count)
        if len(waypoints) > max_waypoints:
            step = max(1, math.ceil(len(waypoints) / max_waypoints))
            waypoints = waypoints[::step]
        if waypoints:
            self._publish_status(
                f"Route split into {len(waypoints)} waypoint(s) before final goal."
            )
        return waypoints

    def _is_cell_safe_for_robot(self, grid: OccupancyGrid, cell: GridCell) -> bool:
        if not self._is_free_cell(grid, cell):
            return False
        return self._is_cell_safe_with_radius(grid, cell, self.goal_safety_radius)

    def _is_cell_safe_with_radius(
        self, grid: OccupancyGrid, cell: GridCell, safety_radius_m: float
    ) -> bool:
        resolution = float(grid.info.resolution)
        if resolution <= 0.0:
            return False
        radius_cells = max(1, int(math.ceil(safety_radius_m / resolution)))
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
                if value < 0 or value >= self.occupied_threshold:
                    return False
        return True

    def _nearest_reachable_safe_cell(
        self,
        grid: OccupancyGrid,
        target: GridCell,
        reachable: set[GridCell],
        max_radius_m: float,
        step_m: float,
        safety_radius_m: Optional[float] = None,
    ) -> Optional[GridCell]:
        resolution = float(grid.info.resolution)
        if resolution <= 0.0:
            return None
        safety_radius = self.goal_safety_radius if safety_radius_m is None else safety_radius_m
        step_m = max(step_m, resolution)
        radius_m = step_m
        tx, ty = target
        while radius_m <= max_radius_m + 1e-9:
            radius_cells = max(1, int(math.ceil(radius_m / resolution)))
            best_cell = None
            best_dist = float("inf")
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    if dx * dx + dy * dy > radius_cells * radius_cells:
                        continue
                    cell = (tx + dx, ty + dy)
                    if cell not in reachable:
                        continue
                    if not self._is_cell_safe_with_radius(grid, cell, safety_radius):
                        continue
                    dist = dx * dx + dy * dy
                    if dist < best_dist:
                        best_dist = dist
                        best_cell = cell
            if best_cell is not None:
                return best_cell
            radius_m += step_m
        return None

    def _feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        distance = getattr(feedback, "distance_remaining", float("nan"))
        if math.isnan(distance):
            return
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if (
            now_sec - self._last_feedback_log_time < 1.0
            and self._last_feedback_distance is not None
            and abs(distance - self._last_feedback_distance) < 0.25
        ):
            return
        self._last_feedback_log_time = now_sec
        self._last_feedback_distance = distance
        self.get_logger().info(f"Distance remaining: {distance:.2f} m")

    def _cancel_goal(self) -> None:
        self._exploration_active = False
        self._exploration_queue.clear()
        self._exploration_results = []
        if self._goal_pending:
            self._cancel_requested = True
            self._publish_status("Cancel requested for pending navigation goal.")
            return
        if self._goal_handle is None:
            self._active_goal_room = None
            self._active_goal_final = None
            self._active_route.clear()
            self._active_route_total = 0
            self._publish_status("No active navigation goal.")
            return
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_done_callback)

    def _cancel_done_callback(self, future) -> None:
        try:
            future.result()
        except Exception as exc:  # noqa: BLE001
            self._publish_status(f"Cancel failed: {exc}")
            return
        self._goal_pending = False
        self._cancel_requested = False
        self._goal_handle = None
        self._active_goal_room = None
        self._active_goal_final = None
        self._active_route.clear()
        self._active_route_total = 0
        self._publish_status("Active navigation goal cancelled.")

    def _auto_detect_rooms(self) -> None:
        if self._goal_handle is not None:
            self._publish_status("Cannot detect rooms while navigation is active.")
            return
        try:
            detected = self._detect_rooms_from_map()
        except RuntimeError as exc:
            self._publish_status(str(exc))
            return

        self._remove_auto_rooms()
        detected_names = []
        for index, room in enumerate(detected, start=1):
            room["source"] = "auto"
            room["explore_order"] = index
            self.rooms[room["name"]] = room
            detected_names.append(room["name"])
        self._save_rooms()

        if detected_names:
            self._publish_status(
                f"Auto-detected {len(detected_names)} rooms: {', '.join(detected_names)}"
            )
        else:
            self._publish_status("No rooms were detected from the current map.")

    def _explore_rooms(self) -> None:
        if self._goal_handle is not None or self._exploration_queue:
            self._publish_status("A navigation task is already active.")
            return
        if not self.rooms:
            self._auto_detect_rooms()
        auto_detected_names = [name for name, room in self.rooms.items() if room.get("source") == "auto"]
        if not auto_detected_names:
            self._publish_status("No rooms available for exploration. Run auto_rooms first.")
            return
        ordered_names = self._build_exploration_order(auto_detected_names)
        if not ordered_names:
            self._publish_status("No reachable room order could be generated.")
            return
        self._exploration_queue = deque(ordered_names)
        self._exploration_active = True
        self._exploration_results = []
        self._publish_status(f"Exploration order: {', '.join(ordered_names)}")
        self._send_next_exploration_goal()

    def _send_next_exploration_goal(self) -> None:
        if not self._exploration_queue:
            return
        next_room = self._exploration_queue.popleft()
        self._send_navigation_goal(next_room)

    def _remove_auto_rooms(self) -> None:
        auto_names = [name for name, room in self.rooms.items() if room.get("source") == "auto"]
        for name in auto_names:
            del self.rooms[name]

    def _build_exploration_order(self, room_names: List[str]) -> List[str]:
        remaining = list(room_names)
        try:
            current_pose = self._lookup_current_pose()
            current_x = float(current_pose["x"])
            current_y = float(current_pose["y"])
        except TransformException:
            current_x = sum(self.rooms[name]["x"] for name in remaining) / len(remaining)
            current_y = sum(self.rooms[name]["y"] for name in remaining) / len(remaining)

        ordered: List[str] = []
        while remaining:
            next_name = min(
                remaining,
                key=lambda name: (self.rooms[name]["x"] - current_x) ** 2
                + (self.rooms[name]["y"] - current_y) ** 2,
            )
            ordered.append(next_name)
            current_x = float(self.rooms[next_name]["x"])
            current_y = float(self.rooms[next_name]["y"])
            remaining.remove(next_name)
        return ordered

    def _detect_rooms_from_map(self) -> List[RoomRecord]:
        if self._latest_map is None:
            raise RuntimeError("No /map message received yet. Wait for map_server or slam_toolbox.")

        grid = self._latest_map
        width = grid.info.width
        height = grid.info.height
        resolution = float(grid.info.resolution)
        if width == 0 or height == 0 or resolution <= 0.0:
            raise RuntimeError("Current map metadata is invalid.")

        free_cells = {
            (x, y)
            for y in range(height)
            for x in range(width)
            if 0 <= grid.data[y * width + x] <= self.free_threshold
        }
        if not free_cells:
            raise RuntimeError("Current map does not contain free cells yet.")

        inflation_radius_cells = max(
            0, int(round(self.room_detection_inflation_radius / resolution))
        )
        inflated_free_cells = self._filter_inflated_free_cells(
            free_cells, width, height, inflation_radius_cells
        )
        components = self._find_connected_components(inflated_free_cells, width, height)

        min_cells = max(1, int(self.room_min_area_m2 / (resolution * resolution)))
        room_candidates = []
        for cells in components:
            if len(cells) < min_cells:
                continue
            representative = self._choose_representative_cell(cells)
            world_x, world_y = self._grid_to_world(representative, grid)
            room_candidates.append(
                {
                    "cell": representative,
                    "x": world_x,
                    "y": world_y,
                    "yaw": 0.0,
                    "cell_count": len(cells),
                    "area_m2": len(cells) * resolution * resolution,
                }
            )

        named_candidates = self._assign_auto_names(room_candidates)
        return sorted(named_candidates, key=lambda item: item["name"])

    def _filter_inflated_free_cells(
        self,
        free_cells: set[GridCell],
        width: int,
        height: int,
        inflation_radius_cells: int,
    ) -> set[GridCell]:
        if inflation_radius_cells <= 0:
            return set(free_cells)

        offsets = []
        radius_sq = inflation_radius_cells * inflation_radius_cells
        for dy in range(-inflation_radius_cells, inflation_radius_cells + 1):
            for dx in range(-inflation_radius_cells, inflation_radius_cells + 1):
                if dx * dx + dy * dy <= radius_sq:
                    offsets.append((dx, dy))

        filtered: set[GridCell] = set()
        for x, y in free_cells:
            keep = True
            for dx, dy in offsets:
                nx = x + dx
                ny = y + dy
                if nx < 0 or ny < 0 or nx >= width or ny >= height or (nx, ny) not in free_cells:
                    keep = False
                    break
            if keep:
                filtered.add((x, y))
        return filtered

    def _find_connected_components(
        self, cells: set[GridCell], width: int, height: int
    ) -> List[List[GridCell]]:
        del width
        del height
        remaining = set(cells)
        components: List[List[GridCell]] = []
        neighbor_offsets = ((1, 0), (-1, 0), (0, 1), (0, -1))

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

    def _choose_representative_cell(self, cells: List[GridCell]) -> GridCell:
        avg_x = sum(cell[0] for cell in cells) / len(cells)
        avg_y = sum(cell[1] for cell in cells) / len(cells)
        return min(cells, key=lambda cell: (cell[0] - avg_x) ** 2 + (cell[1] - avg_y) ** 2)

    def _assign_auto_names(self, candidates: List[RoomRecord]) -> List[RoomRecord]:
        if not candidates:
            return []

        xs = [float(candidate["x"]) for candidate in candidates]
        ys = [float(candidate["y"]) for candidate in candidates]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        width = max(max_x - min_x, 1e-6)
        height = max(max_y - min_y, 1e-6)

        bucket_counts: Dict[str, int] = {}
        existing_names = set(self.rooms)
        named = []
        for candidate in sorted(candidates, key=lambda item: (-float(item["y"]), float(item["x"]))):
            horizontal = "西" if candidate["x"] < min_x + width / 3.0 else "东" if candidate["x"] > max_x - width / 3.0 else "中"
            vertical = "南" if candidate["y"] < min_y + height / 3.0 else "北" if candidate["y"] > max_y - height / 3.0 else "中"
            base_name = self._build_directional_name(vertical, horizontal)
            bucket_counts[base_name] = bucket_counts.get(base_name, 0) + 1
            suffix = bucket_counts[base_name]
            candidate_name = base_name if suffix == 1 else f"{base_name}{suffix}"
            unique_name = self._ensure_unique_room_name(candidate_name, existing_names)
            existing_names.add(unique_name)
            candidate["name"] = unique_name
            named.append(candidate)
        return named

    def _build_directional_name(self, vertical: str, horizontal: str) -> str:
        mapping = {
            ("北", "西"): "西北房间",
            ("北", "中"): "北侧房间",
            ("北", "东"): "东北房间",
            ("中", "西"): "西侧房间",
            ("中", "中"): "中央区域",
            ("中", "东"): "东侧房间",
            ("南", "西"): "西南房间",
            ("南", "中"): "南侧房间",
            ("南", "东"): "东南房间",
        }
        return mapping[(vertical, horizontal)]

    def _ensure_unique_room_name(self, base_name: str, existing_names: set[str]) -> str:
        if base_name not in existing_names:
            return base_name
        suffix = 2
        while f"{base_name}{suffix}" in existing_names:
            suffix += 1
        return f"{base_name}{suffix}"

    def _grid_to_world(self, cell: GridCell, grid: OccupancyGrid) -> Tuple[float, float]:
        x, y = cell
        origin = grid.info.origin.position
        resolution = float(grid.info.resolution)
        world_x = float(origin.x + (x + 0.5) * resolution)
        world_y = float(origin.y + (y + 0.5) * resolution)
        return world_x, world_y


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoomNavNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
