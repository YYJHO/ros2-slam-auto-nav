#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="${PROJECT_ROOT}/workspace"

# Force ROS/Gazebo Python tools to use the system interpreter instead of a
# possibly active conda or custom Python environment.
unset PYTHONHOME
export PATH="/usr/bin:/bin:/usr/sbin:/sbin:${PATH}"

source /opt/ros/humble/setup.bash

# Isolate this simulator from other ROS 2 systems running on the workstation.
export ROS_DOMAIN_ID="${VIRTUAL_INDOOR_NAV_ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

export AMENT_PREFIX_PATH="${WORKSPACE_DIR}/install/virtual_indoor_nav:${AMENT_PREFIX_PATH}"
export COLCON_PREFIX_PATH="${WORKSPACE_DIR}/install:${COLCON_PREFIX_PATH}"

if [ -f "${WORKSPACE_DIR}/install/local_setup.bash" ]; then
  source "${WORKSPACE_DIR}/install/local_setup.bash"
else
  echo "Workspace has not been built yet: ${WORKSPACE_DIR}/install/local_setup.bash" >&2
  echo "Run: bash ${PROJECT_ROOT}/scripts/build_workspace.sh" >&2
  exit 1
fi
