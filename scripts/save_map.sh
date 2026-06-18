#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MAP_DIR="${PROJECT_ROOT}/runtime/maps"
MAP_BASENAME="${MAP_DIR}/generated_map"

mkdir -p "${MAP_DIR}"

source "${PROJECT_ROOT}/scripts/source_workspace.sh"

ros2 run nav2_map_server map_saver_cli -f "${MAP_BASENAME}"
