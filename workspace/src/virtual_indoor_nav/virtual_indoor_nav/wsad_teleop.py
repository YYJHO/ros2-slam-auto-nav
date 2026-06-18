#!/usr/bin/env python3
import os
import sys
from typing import Set

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

try:
    import tkinter as tk
except ImportError:  # pragma: no cover
    tk = None


HELP_TEXT = """
WSAD realtime teleop
-------------------
Hold w : move forward
Hold s : move backward
Hold a : rotate left
Hold d : rotate right
Release: stop the corresponding motion
x      : stop immediately
q / e  : increase / decrease both linear and angular speed
""".strip()


class WsadTeleop(Node):
    def __init__(self) -> None:
        super().__init__("wsad_teleop")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.linear_speed = 0.25
        self.angular_speed = 0.8
        self.active_keys: Set[str] = set()
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.create_timer(0.05, self._publish_current_twist)
        self.get_logger().info("WSAD teleop ready. Publishing to /cmd_vel")

    def publish_twist(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)

    def stop(self) -> None:
        self.active_keys.clear()
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.publish_twist(0.0, 0.0)

    def on_key_press(self, key: str) -> bool:
        key = key.lower()
        if key in {"w", "a", "s", "d"}:
            self.active_keys.add(key)
            self._update_motion_from_keys()
            return True

        if key == "x":
            self.stop()
            return True

        if key == "q":
            self.linear_speed *= 1.1
            self.angular_speed *= 1.1
            self.get_logger().info(
                f"Speed up: linear={self.linear_speed:.2f} m/s, "
                f"angular={self.angular_speed:.2f} rad/s"
            )
            self._update_motion_from_keys()
            return True

        if key == "e":
            self.linear_speed *= 0.9
            self.angular_speed *= 0.9
            self.get_logger().info(
                f"Speed down: linear={self.linear_speed:.2f} m/s, "
                f"angular={self.angular_speed:.2f} rad/s"
            )
            self._update_motion_from_keys()
            return True

        return False

    def on_key_release(self, key: str) -> bool:
        key = key.lower()
        if key not in {"w", "a", "s", "d"}:
            return False
        self.active_keys.discard(key)
        self._update_motion_from_keys()
        return True

    def _update_motion_from_keys(self) -> None:
        if "w" in self.active_keys and "s" not in self.active_keys:
            self.current_linear = self.linear_speed
        elif "s" in self.active_keys and "w" not in self.active_keys:
            self.current_linear = -self.linear_speed
        else:
            self.current_linear = 0.0

        if "a" in self.active_keys and "d" not in self.active_keys:
            self.current_angular = self.angular_speed
        elif "d" in self.active_keys and "a" not in self.active_keys:
            self.current_angular = -self.angular_speed
        else:
            self.current_angular = 0.0

        self.publish_twist(self.current_linear, self.current_angular)

    def _publish_current_twist(self) -> None:
        self.publish_twist(self.current_linear, self.current_angular)


class TeleopWindow:
    def __init__(self, node: WsadTeleop) -> None:
        if tk is None:
            raise RuntimeError("Tkinter is not available.")
        self.node = node
        self.root = tk.Tk()
        self.root.title("WSAD Teleop")
        self.root.geometry("420x240")
        self.root.resizable(False, False)

        self.status_var = tk.StringVar()
        self.speed_var = tk.StringVar()
        self._refresh_labels()

        title = tk.Label(
            self.root,
            text="Realtime WSAD Teleop",
            font=("Ubuntu", 16, "bold"),
        )
        title.pack(pady=(16, 8))

        help_label = tk.Label(
            self.root,
            text=HELP_TEXT,
            justify="left",
            font=("Ubuntu Mono", 11),
        )
        help_label.pack(padx=16, anchor="w")

        self.status_label = tk.Label(
            self.root,
            textvariable=self.status_var,
            font=("Ubuntu", 12),
        )
        self.status_label.pack(pady=(12, 4))

        self.speed_label = tk.Label(
            self.root,
            textvariable=self.speed_var,
            font=("Ubuntu Mono", 11),
        )
        self.speed_label.pack()

        hint_label = tk.Label(
            self.root,
            text="Click this window once to focus keyboard input.",
            font=("Ubuntu", 10),
        )
        hint_label.pack(pady=(10, 0))

        self.root.bind("<KeyPress>", self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.focus_force()
        self.root.after(20, self._spin_ros_once)

    def _on_key_press(self, event) -> None:
        if self.node.on_key_press(event.keysym):
            self._refresh_labels()

    def _on_key_release(self, event) -> None:
        if self.node.on_key_release(event.keysym):
            self._refresh_labels()

    def _refresh_labels(self) -> None:
        active = "".join(sorted(self.node.active_keys)) or "(none)"
        self.status_var.set(
            f"Active keys: {active} | "
            f"linear={self.node.current_linear:.2f} m/s | "
            f"angular={self.node.current_angular:.2f} rad/s"
        )
        self.speed_var.set(
            f"Base speed: linear={self.node.linear_speed:.2f} m/s, "
            f"angular={self.node.angular_speed:.2f} rad/s"
        )

    def _spin_ros_once(self) -> None:
        if not rclpy.ok():
            self.root.quit()
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.root.after(20, self._spin_ros_once)

    def _on_close(self) -> None:
        self.node.stop()
        self.root.quit()

    def run(self) -> None:
        self.root.mainloop()


def main(args=None) -> None:
    if tk is None or not os.environ.get("DISPLAY"):
        print("Realtime key release control requires a desktop session with Tkinter.")
        print("Run this on the Ubuntu desktop where Gazebo/RViz are open.")
        sys.exit(1)

    rclpy.init(args=args)
    node = WsadTeleop()
    window = TeleopWindow(node)
    try:
        window.run()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
