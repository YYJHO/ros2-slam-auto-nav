# Architecture

This project is a ROS 2 Humble simulation stack for indoor mapping and room
navigation. It is designed around a small differential-drive robot in Gazebo
Classic, with SLAM, localization, planning, and a Python control center.

## Runtime Modes

### Mapping

Entry point:

```bash
bash scripts/run_mapping.sh
```

Main launch file:

```text
workspace/src/virtual_indoor_nav/launch/mapping.launch.py
```

Mapping starts Gazebo, the robot model, sensor publishers, EKF, `slam_toolbox`,
and RViz. The robot can be driven with:

```bash
bash scripts/run_wsad_teleop.sh
```

Maps are saved with:

```bash
bash scripts/save_map.sh
```

### Automatic Mapping

Entry point:

```bash
bash scripts/run_auto_mapping.sh
```

Main launch file:

```text
workspace/src/virtual_indoor_nav/launch/auto_mapping.launch.py
```

This mode adds `auto_explorer`, which detects frontier goals from the map and
publishes exploration status for the control center.

### Navigation

Entry point:

```bash
bash scripts/run_navigation.sh
```

Main launch file:

```text
workspace/src/virtual_indoor_nav/launch/navigation.launch.py
```

Navigation loads the saved map from `runtime/maps/`, starts AMCL and Nav2, and
uses `room_nav_node` for named room goals.

### Control Center

Entry point:

```bash
bash scripts/run_control_center.sh
```

The control center wraps the common workflows:

- Build workspace.
- Clean old Gazebo/Nav2 processes.
- Start mapping or automatic mapping.
- Save maps.
- Save, rename, delete, and navigate to named room goals.
- Export diagnostics.

## Main ROS Topics

| Topic | Purpose |
| --- | --- |
| `/scan` | 2D LiDAR input for SLAM and obstacle layers. |
| `/odom` | Gazebo differential-drive odometry. |
| `/imu/data` | Simulated IMU data. |
| `/odometry/filtered` | EKF output for navigation. |
| `/map` | Occupancy grid from SLAM or map server. |
| `/room_command` | String commands consumed by `room_nav_node`. |
| `/room_status` | Room navigation status messages. |
| `/exploration_coverage` | Exploration progress from `auto_explorer`. |

## Important Files

| File | Role |
| --- | --- |
| `workspace/src/virtual_indoor_nav/worlds/apartment.world` | Gazebo indoor world. |
| `workspace/src/virtual_indoor_nav/urdf/indoor_bot.urdf.xacro` | Robot model, sensors, and Gazebo plugins. |
| `workspace/src/virtual_indoor_nav/config/nav2_params.yaml` | Nav2 controller, planner, costmap, and behavior tuning. |
| `workspace/src/virtual_indoor_nav/config/slam_toolbox.yaml` | SLAM configuration. |
| `workspace/src/virtual_indoor_nav/config/ekf.yaml` | Robot localization EKF configuration. |
| `workspace/src/virtual_indoor_nav/virtual_indoor_nav/auto_explorer.py` | Frontier-based automatic exploration node. |
| `workspace/src/virtual_indoor_nav/virtual_indoor_nav/room_nav_node.py` | Named room goal management. |
| `workspace/src/virtual_indoor_nav/virtual_indoor_nav/control_center.py` | Tkinter desktop control center. |

## Runtime Data

Runtime outputs are intentionally ignored by Git:

```text
runtime/maps/generated_map.yaml
runtime/maps/generated_map.pgm
runtime/rooms.yaml
runtime/diagnostics/
workspace/build/
workspace/install/
workspace/log/
```

Keep only source files, documentation, launch files, parameters, models, and
example assets in commits.
