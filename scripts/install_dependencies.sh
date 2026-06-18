#!/usr/bin/env bash
set -euo pipefail

if ! command -v sudo >/dev/null 2>&1; then
  echo "sudo is required to install system packages." >&2
  exit 1
fi

if [ -f /etc/os-release ]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  if [ "${ID:-}" != "ubuntu" ] || [ "${VERSION_ID:-}" != "22.04" ]; then
    echo "Warning: this project is tested on Ubuntu 22.04; detected ${PRETTY_NAME:-unknown}." >&2
  fi
fi

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "Warning: ROS 2 Humble is not installed at /opt/ros/humble/setup.bash." >&2
  echo "Install ROS 2 Humble first, then rerun this script." >&2
fi

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-lxml \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-map-server \
  ros-humble-nav2-amcl \
  ros-humble-nav2-simple-commander \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-teleop-twist-keyboard \
  ros-humble-rviz2 \
  python3-pil \
  python3-pil.imagetk

echo
echo "Dependency installation finished."
echo "Next steps:"
echo "  bash scripts/check_system.sh"
echo "  bash scripts/build_workspace.sh"
