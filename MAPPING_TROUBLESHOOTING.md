# 建图失败排查清单

这个清单用于排查下面几类问题：

- Gazebo 里机器人会动，但 RViz 里机器人不跟随
- RViz 有激光，但地图不继续长出来
- `/map`、`/scan`、`TF` 某一层断了

建议每次都先从项目根目录重新启动：

```bash
bash scripts/cleanup_gazebo.sh
bash scripts/build_workspace.sh
bash scripts/run_mapping.sh
```

另开一个终端：

```bash
bash scripts/run_wsad_teleop.sh
```

再开第三个终端做排查：

```bash
bash scripts/source_workspace.sh
```

## 1. 先看关键话题在不在

```bash
ros2 topic list | grep -E "^/scan$|^/imu/data$|^/odom$|^/odometry/filtered$|^/map$|^/tf$|^/tf_static$"
```

正常时至少应看到：

- `/scan`
- `/imu/data`
- `/odom`
- `/odometry/filtered`
- `/tf`
- `/tf_static`

建图开始后还应看到：

- `/map`

如果 `/scan` 或 `/odom` 不存在，先不要看 RViz，先回头查 Gazebo 传感器和底盘插件。

## 2. 看激光有没有数据

```bash
ros2 topic echo /scan --once
```

正常时应该能看到：

- `angle_min`
- `angle_max`
- `ranges`

如果这里卡住不出数据：

- 说明 Gazebo 的激光插件没有正常发布
- 重点检查 `indoor_bot.urdf.xacro` 里的 `laser_link` 和 `libgazebo_ros_ray_sensor.so`

## 3. 看轮式里程计有没有数据

```bash
ros2 topic echo /odom --once
```

正常时应该能看到：

- `pose`
- `twist`
- `header.frame_id: odom`
- `child_frame_id: base_footprint` 或接近含义的底盘坐标

如果这里没有数据：

- 说明 Gazebo 的差速驱动插件没正常工作
- 重点检查 `indoor_bot.urdf.xacro` 里的 `libgazebo_ros_diff_drive.so`

## 4. 看 EKF 融合后的里程计

```bash
ros2 topic echo /odometry/filtered --once
```

正常时应该有输出。

如果 `/odom` 有，但 `/odometry/filtered` 没有：

- 说明 EKF 没吃到输入
- 重点检查 `config/ekf.yaml`

当前项目里，EKF 应该订阅：

- `odom0: /odom`
- `imu0: /imu/data`

## 5. 看 TF 是否连通

先看 `odom -> base_footprint`：

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

正常时数值应持续变化，机器人移动时 `Translation` 会变化。

再看 `map -> odom`：

```bash
ros2 run tf2_ros tf2_echo map odom
```

正常时：

- 建图启动后能查到这个 TF
- 机器人移动过程中这个变换可能会小幅调整

如果 `odom -> base_footprint` 查不到：

- RViz 中机器人通常会卡死不动
- 说明底盘 odom 或 EKF TF 没发出来

如果 `map -> odom` 查不到：

- 地图通常不会正常增长
- 说明 `slam_toolbox` 没正常工作

## 6. 看 slam_toolbox 是否真的在发布地图

```bash
ros2 topic echo /map --once
```

正常时应该看到：

- `info.width`
- `info.height`
- `resolution`
- `data`

如果 `/scan` 和 TF 都正常，但 `/map` 一直没有：

- 说明 `slam_toolbox` 没启动成功或报错

再看节点列表：

```bash
ros2 node list | grep slam
```

正常时应看到类似：

- `/slam_toolbox`

## 7. 看 RViz 里机器人不跟随时该查什么

这是你现在最接近的问题。

优先执行：

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map base_footprint
```

判断方法：

- 如果 Gazebo 里机器人在动，但这两个 TF 数值不变，问题在 TF 链
- 如果 TF 在变，但 RViz 机器人不动，问题更可能在 RViz Fixed Frame 或显示项

再确认 RViz：

- Fixed Frame 应该设成 `map`
- Displays 里至少要启用 `TF`、`LaserScan`、`Map`、`RobotModel`

如果你怀疑是 RViz 自己卡住，直接重开：

```bash
bash scripts/run_mapping.sh
```

## 8. 直接看节点日志

如果你已经在 `run_mapping.sh` 的终端里，重点看有没有这些报错：

- `No transform`
- `Message Filter dropping message`
- `Lookup would require extrapolation`
- `Failed to compute odom pose`
- `Failed to create map update`

也可以单独查最近日志：

```bash
ls -t workspace/log | head
```

## 9. 一条命令快速看系统是否活着

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /odometry/filtered
```

正常时：

- `/scan` 应持续刷新
- `/odom` 应持续刷新
- `/odometry/filtered` 应持续刷新

如果 `/scan` 有频率、`/odom` 有频率，但 `/odometry/filtered` 没频率：

- 基本可以直接定位为 EKF 配置或输入不匹配

## 10. 当前项目最常见的故障定位

### 现象 1

Gazebo 里机器人会动，RViz 里机器人不动。

优先检查：

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic echo /odometry/filtered --once
```

大概率原因：

- EKF 没有发布 `odom -> base_footprint`
- `/odom` 没接上 EKF

### 现象 2

RViz 有激光，但地图不长。

优先检查：

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /map --once
```

大概率原因：

- `slam_toolbox` 没正常启动
- TF 不完整，`slam_toolbox` 无法解算

### 现象 3

地图只长局部，或者机器人过不了门。

优先检查：

- Gazebo 世界里门洞是否真的比机器人宽
- 障碍物是否堵在门口

世界文件在：

```text
workspace/src/virtual_indoor_nav/worlds/apartment.world
```

## 11. 如果你要把结果发给我

最有用的是把下面四条命令的输出贴出来：

```bash
ros2 topic list | grep -E "^/scan$|^/imu/data$|^/odom$|^/odometry/filtered$|^/map$|^/tf$|^/tf_static$"
ros2 topic echo /odom --once
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_footprint
```
