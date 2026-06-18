#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="${PROJECT_ROOT}/workspace"
MAP_YAML="${PROJECT_ROOT}/runtime/maps/generated_map.yaml"
MAP_PGM="${PROJECT_ROOT}/runtime/maps/generated_map.pgm"
ROOMS_FILE="${PROJECT_ROOT}/runtime/rooms.yaml"

failures=0
warnings=0

pass() {
  printf '[PASS] %s\n' "$1"
}

warn() {
  warnings=$((warnings + 1))
  printf '[WARN] %s\n' "$1"
}

fail() {
  failures=$((failures + 1))
  printf '[FAIL] %s\n' "$1"
}

check_file() {
  local path="$1"
  local label="$2"
  if [ -f "$path" ]; then
    pass "$label: $path"
  else
    fail "$label missing: $path"
  fi
}

check_dir() {
  local path="$1"
  local label="$2"
  if [ -d "$path" ]; then
    pass "$label: $path"
  else
    fail "$label missing: $path"
  fi
}

printf 'Virtual indoor navigation system check\n'
printf 'Project: %s\n\n' "${PROJECT_ROOT}"

check_dir "${PROJECT_ROOT}" "project root"
check_dir "${WORKSPACE_DIR}/src/virtual_indoor_nav" "package source"
check_file "${WORKSPACE_DIR}/src/virtual_indoor_nav/package.xml" "package.xml"
check_file "${WORKSPACE_DIR}/src/virtual_indoor_nav/setup.py" "setup.py"
check_file "${WORKSPACE_DIR}/src/virtual_indoor_nav/launch/mapping.launch.py" "mapping launch"
check_file "${WORKSPACE_DIR}/src/virtual_indoor_nav/launch/auto_mapping.launch.py" "auto mapping launch"
check_file "${WORKSPACE_DIR}/src/virtual_indoor_nav/launch/navigation.launch.py" "navigation launch"
check_file "${WORKSPACE_DIR}/src/virtual_indoor_nav/urdf/indoor_bot.urdf.xacro" "robot model"
check_file "${WORKSPACE_DIR}/src/virtual_indoor_nav/worlds/apartment.world" "Gazebo world"
check_file "${WORKSPACE_DIR}/src/virtual_indoor_nav/config/nav2_params.yaml" "Nav2 params"
check_file "${WORKSPACE_DIR}/src/virtual_indoor_nav/virtual_indoor_nav/auto_explorer.py" "auto explorer node"
check_file "${PROJECT_ROOT}/scripts/run_auto_mapping.sh" "auto mapping script"

if grep -q '_motion_loop' "${WORKSPACE_DIR}/src/virtual_indoor_nav/virtual_indoor_nav/auto_explorer.py" \
  && grep -q '_frontier_detection_loop' "${WORKSPACE_DIR}/src/virtual_indoor_nav/virtual_indoor_nav/auto_explorer.py" \
  && grep -q 'no_frontier_streak_limit' "${WORKSPACE_DIR}/src/virtual_indoor_nav/virtual_indoor_nav/auto_explorer.py" \
  && grep -q 'frontier_refresh_interval' "${WORKSPACE_DIR}/src/virtual_indoor_nav/virtual_indoor_nav/auto_explorer.py" \
  && grep -q '_estimate_information_gain' "${WORKSPACE_DIR}/src/virtual_indoor_nav/virtual_indoor_nav/auto_explorer.py" \
  && grep -q '_trigger_forced_exploration' "${WORKSPACE_DIR}/src/virtual_indoor_nav/virtual_indoor_nav/auto_explorer.py" \
  && grep -q '/exploration_coverage' "${WORKSPACE_DIR}/src/virtual_indoor_nav/virtual_indoor_nav/auto_explorer.py"; then
  pass "auto explorer v3.0 scoring/coverage architecture is present"
else
  fail "auto explorer v3.0 scoring/coverage architecture is missing"
fi

if grep -q 'max_vel_x: 0.30' "${WORKSPACE_DIR}/src/virtual_indoor_nav/config/nav2_params.yaml" \
  && grep -q 'sim_time: 2.0' "${WORKSPACE_DIR}/src/virtual_indoor_nav/config/nav2_params.yaml" \
  && grep -q 'inflation_radius: 0.25' "${WORKSPACE_DIR}/src/virtual_indoor_nav/config/nav2_params.yaml" \
  && grep -q 'inflation_radius: 0.30' "${WORKSPACE_DIR}/src/virtual_indoor_nav/config/nav2_params.yaml" \
  && grep -q 'obstacle_max_range: 8.0' "${WORKSPACE_DIR}/src/virtual_indoor_nav/config/nav2_params.yaml"; then
  pass "Nav2 v3.0 narrow-passage tuning is present"
else
  fail "Nav2 v3.0 narrow-passage tuning is missing"
fi

printf '\nROS environment\n'
if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
  pass "ROS 2 Humble setup found"
else
  fail "ROS 2 Humble setup not found at /opt/ros/humble/setup.bash"
fi

if [ -f "${WORKSPACE_DIR}/install/local_setup.bash" ]; then
  set +u
  # shellcheck disable=SC1091
  source "${PROJECT_ROOT}/scripts/source_workspace.sh"
  set -u
  pass "workspace install setup found"
else
  warn "workspace has not been built yet: ${WORKSPACE_DIR}/install/local_setup.bash"
fi

if command -v ros2 >/dev/null 2>&1; then
  pass "ros2 command is available: $(command -v ros2)"
else
  fail "ros2 command is not available"
fi

if command -v colcon >/dev/null 2>&1; then
  pass "colcon command is available: $(command -v colcon)"
else
  fail "colcon command is not available"
fi

if command -v gazebo >/dev/null 2>&1; then
  pass "gazebo command is available: $(command -v gazebo)"
else
  warn "gazebo command is not available in PATH"
fi

printf '\nROS packages\n'
required_packages=(
  gazebo_ros
  nav2_bringup
  nav2_map_server
  slam_toolbox
  robot_localization
  robot_state_publisher
  sensor_msgs
  xacro
  rviz2
)

if command -v ros2 >/dev/null 2>&1; then
  for package in "${required_packages[@]}"; do
    if ros2 pkg prefix "$package" >/dev/null 2>&1; then
      pass "ROS package available: ${package}"
    else
      fail "ROS package missing: ${package}"
    fi
  done

  if ros2 pkg prefix virtual_indoor_nav >/dev/null 2>&1; then
    pass "project package available: virtual_indoor_nav"
    if ros2 pkg executables virtual_indoor_nav | grep -q '^virtual_indoor_nav auto_explorer$'; then
      pass "project executable available: auto_explorer"
    else
      fail "project executable missing: auto_explorer"
    fi
  else
    warn "project package not discoverable yet; run scripts/build_workspace.sh"
  fi
fi

printf '\nRuntime files\n'
if [ -f "${MAP_YAML}" ] && [ -f "${MAP_PGM}" ]; then
  pass "saved map files exist"
elif [ -f "${MAP_YAML}" ] || [ -f "${MAP_PGM}" ]; then
  warn "only one saved map file exists; expected both generated_map.yaml and generated_map.pgm"
else
  warn "saved map files not found yet; run mapping and scripts/save_map.sh"
fi

if [ -f "${ROOMS_FILE}" ]; then
  pass "rooms file exists: ${ROOMS_FILE}"
else
  warn "rooms file not found yet; room_nav_node will create it on first run"
fi

printf '\nProcess state\n'
if pgrep -x gzserver >/dev/null 2>&1 || pgrep -x gzclient >/dev/null 2>&1; then
  warn "Gazebo is currently running; use scripts/cleanup_gazebo.sh before restarting modes"
else
  pass "no leftover gzserver/gzclient processes detected"
fi

if [ -n "${DISPLAY:-}" ]; then
  pass "DISPLAY is set: ${DISPLAY}"
else
  warn "DISPLAY is not set; Gazebo/RViz/control center GUI windows cannot open here"
fi

printf '\nSummary: %d failure(s), %d warning(s)\n' "${failures}" "${warnings}"
if [ "${failures}" -gt 0 ]; then
  exit 1
fi
