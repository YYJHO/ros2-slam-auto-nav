#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MAP_YAML="${PROJECT_ROOT}/runtime/maps/generated_map.yaml"
MAP_PGM="${PROJECT_ROOT}/runtime/maps/generated_map.pgm"

if [ ! -f "${MAP_YAML}" ] || [ ! -f "${MAP_PGM}" ]; then
  echo "Saved map not found." >&2
  echo "Run mapping first, then save a map:" >&2
  echo "  bash scripts/run_auto_mapping.sh" >&2
  echo "  bash scripts/save_map.sh" >&2
  exit 1
fi

source "${PROJECT_ROOT}/scripts/source_workspace.sh"

ros2 launch virtual_indoor_nav navigation.launch.py gui:=true rviz:=true
