#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="${PROJECT_ROOT}/workspace"

source /opt/ros/humble/setup.bash

cd "${WORKSPACE_DIR}"
colcon build --symlink-install

chmod +x "${PROJECT_ROOT}/scripts/source_workspace.sh"

echo
echo "Build completed."
echo "Source with:"
echo "bash ${PROJECT_ROOT}/scripts/source_workspace.sh"
