#!/usr/bin/env bash
set -e

# Keep restarts deterministic. Gazebo alone is not enough: failed launches can
# leave Nav2 lifecycle managers and project Python nodes in the ROS graph.
patterns=(
  "gzserver"
  "gzclient"
  "controller_server"
  "planner_server"
  "behavior_server"
  "bt_navigator"
  "lifecycle_manager"
  "async_slam_toolbox_node"
  "ekf_node"
  "robot_state_publisher"
  "rviz2"
  "spawn_entity.py"
  "auto_explorer"
  "room_nav_node"
  "app_system.launch.py"
)

for pattern in "${patterns[@]}"; do
  pkill -f "${pattern}" 2>/dev/null || true
done

sleep 1

for pattern in "${patterns[@]}"; do
  pkill -9 -f "${pattern}" 2>/dev/null || true
done

for _ in 1 2 3 4 5; do
  running=false
  for pattern in "${patterns[@]}"; do
    if pgrep -f "${pattern}" >/dev/null 2>&1; then
      running=true
      break
    fi
  done
  if [ "${running}" = false ]; then
    break
  fi
  sleep 1
done

echo "ROS/Gazebo cleanup finished."
echo "Stopped leftover Gazebo, SLAM, Nav2, RViz, explorer, and room navigation processes."
