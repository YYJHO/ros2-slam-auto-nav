# 安装与操作手册

这份手册只保留当前项目最常用、最容易成功的流程。

适用环境：

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic

项目目录：

```text
release-virtual-indoor-nav
```

---

## 1. 先知道这几个目录

```text
release-virtual-indoor-nav
├── scripts/                  你平时直接运行的脚本
├── runtime/maps/             保存地图
├── runtime/rooms.yaml        保存房间名字和目标点
└── workspace/src/virtual_indoor_nav
    ├── worlds/               Gazebo 世界
    ├── urdf/                 机器人模型
    ├── config/               SLAM / Nav2 / EKF 参数
    ├── launch/               启动文件
    └── virtual_indoor_nav/   Python 节点代码
```

如果你要改房间结构，改：

```text
workspace/src/virtual_indoor_nav/worlds/apartment.world
```

如果你要改机器人和 Gazebo 底盘插件，改：

```text
workspace/src/virtual_indoor_nav/urdf/indoor_bot.urdf.xacro
```

如果你要改实时键盘控制，改：

```text
workspace/src/virtual_indoor_nav/virtual_indoor_nav/wsad_teleop.py
```

---

## 2. 第一次先装依赖

执行：

```bash
bash scripts/install_dependencies.sh
```

这一步会安装：

- Gazebo ROS 插件
- Nav2
- slam_toolbox
- robot_localization
- xacro
- rviz2

装完后可以简单检查：

```bash
source /opt/ros/humble/setup.bash
ros2 pkg list | grep -E "gazebo_ros|nav2_bringup|slam_toolbox|robot_localization"
```

正常时应能看到这些包名。

---

## 3. 每次改完代码后怎么重新构建

执行：

```bash
bash scripts/build_workspace.sh
```

正常时你会看到：

```text
Starting >>> virtual_indoor_nav
Finished <<< virtual_indoor_nav
Summary: 1 package finished
```

---

## 4. 每次启动前先清理旧 Gazebo

执行：

```bash
bash scripts/cleanup_gazebo.sh
```

这一步是为了避免：

- 端口占用
- 旧 Gazebo 进程残留
- 你明明改了文件但画面还是旧的

---

## 5. 项目环境怎么加载

执行：

```bash
bash scripts/source_workspace.sh
```

这一步很重要，原因是你机器上有 `(base)` conda 环境，直接裸跑 `ros2` 或 `python3` 很容易串环境。

如果你想确认环境是否加载成功：

```bash
bash scripts/source_workspace.sh
ros2 pkg prefix virtual_indoor_nav
```

正常时会返回类似：

```text
workspace/install/virtual_indoor_nav
```

---

## 6. 最短成功路径

如果你只是想完整跑通一次，按这个顺序做：

```bash
bash scripts/cleanup_gazebo.sh
bash scripts/check_system.sh
bash scripts/build_workspace.sh
bash scripts/run_auto_mapping.sh
```

另开一个终端：

```bash
bash scripts/run_wsad_teleop.sh
```

建图完成后：

```bash
bash scripts/save_map.sh
bash scripts/run_navigation.sh
```

---

## 7. 可视化控制台

如果你不想手动记脚本，可以直接打开桌面控制台：

```bash
bash scripts/run_control_center.sh
```

这个界面里已经集成了这些按钮：

- 系统健康检查
- 构建工作区
- 清理 Gazebo
- 开始模拟建图
- 打开实时控制
- 保存地图
- 开始导航
- 保存当前位置为房间
- 自动识别房间
- 导航到选中房间
- 自动探索所有房间

你还可以在这个界面里：

- 直接在主界面里按住 `W/A/S/D` 控制机器人
- 直接点击方向按钮控制机器人
- 输入房间名字并保存
- 查看当前已保存的房间列表
- 预览已经保存的地图
- 查看日志输出

推荐优先使用这个主界面内置控制，不必另外开独立控制窗口。

如果你更喜欢图形界面，后面的很多命令都可以直接在这个控制台里点按钮完成。

---

## 8. 建图模式

### 启动建图

执行：

```bash
bash scripts/run_mapping.sh
```

它会启动：

- Gazebo
- 机器人
- 激光雷达
- IMU
- 里程计
- EKF
- slam_toolbox
- RViz

### 建图成功时你应该看到什么

- Gazebo 里机器人能动
- RViz 里能看到机器人
- RViz 里能看到激光点
- 机器人移动时地图会逐步长出来

### 当前项目里关键链路

当前已统一成下面这条链：

- Gazebo 差速驱动发布 `/odom`
- EKF 读取 `/odom` 和 `/imu/data`
- slam_toolbox 使用 `/scan`

如果这几层断了，地图就不会正常长。

---

## 9. 实时键盘控制

推荐方式：

先打开控制台：

```bash
bash scripts/run_control_center.sh
```

然后：

- 先点一下主界面空白位置
- 按住 `W/A/S/D` 控制机器人
- 松开立即停止
- 也可以直接用界面上的方向按钮

### 备选方式：独立控制窗口

执行：

```bash
bash scripts/run_wsad_teleop.sh
```

这个脚本现在会打开一个小窗口，不是旧式纯终端控制。

注意：

- 先点一下那个控制窗口，让它获得键盘焦点
- 按住键才会动
- 松开键会立刻停

按键规则：

- `W`：按住前进
- `S`：按住后退
- `A`：按住左转
- `D`：按住右转
- `W + A`：前进并左转
- `W + D`：前进并右转
- `X`：立即停车
- `Q`：速度增加 10%
- `E`：速度降低 10%

如果窗口打不开，通常是：

- 你当前不是桌面会话
- 没有图形界面
- `DISPLAY` 环境不存在

---

## 10. 保存地图

地图看起来完整后执行：

```bash
bash scripts/save_map.sh
```

会生成：

```text
runtime/maps/generated_map.yaml
runtime/maps/generated_map.pgm
```

如果这两个文件没出现，说明地图还没真正保存成功。

---

## 11. 导航模式

执行：

```bash
bash scripts/run_navigation.sh
```

它会启动：

- Gazebo
- 机器人
- Nav2
- AMCL
- 房间导航节点
- RViz

默认读取这张地图：

```text
runtime/maps/generated_map.yaml
```

如果你想换一张地图：

```bash
bash scripts/source_workspace.sh
ros2 launch virtual_indoor_nav navigation.launch.py map:=/你的地图路径.yaml
```

---

## 12. 房间命令

房间命令统一发到：

```text
/room_command
```

项目已经提供脚本，不需要手打 `ros2 topic pub`。

### 手动保存一个房间

```bash
bash scripts/send_room_command.sh "save 8号房间"
```

### 查看当前房间列表

```bash
bash scripts/send_room_command.sh "list"
```

### 删除一个房间

```bash
bash scripts/send_room_command.sh "delete 8号房间"
```

### 去某个房间

```bash
bash scripts/send_room_command.sh "goto 8号房间"
```

### 取消当前导航

```bash
bash scripts/send_room_command.sh "cancel"
```

---

## 13. 自动命名房间

执行：

```bash
bash scripts/send_room_command.sh "auto_rooms"
```

它会：

- 读取当前 `/map`
- 自动找可通行的大连通区域
- 生成默认中文房间名
- 写入 `runtime/rooms.yaml`

默认命名类似：

- `西北房间`
- `北侧房间`
- `东南房间`

房间数据保存在：

```text
runtime/rooms.yaml
```

---

## 14. 自动探索所有房间

执行：

```bash
bash scripts/send_room_command.sh "explore_rooms"
```

它会：

1. 如果还没有自动房间数据，先尝试自动识别
2. 生成一条巡航顺序
3. 逐个房间下发导航目标
4. 走完整个已识别区域

中途停止：

```bash
bash scripts/send_room_command.sh "cancel"
```

---

## 15. 你现在最常用的命令

### 重新开始一轮建图

```bash
bash scripts/cleanup_gazebo.sh
bash scripts/build_workspace.sh
bash scripts/run_mapping.sh
```

### 打开独立控制窗口

```bash
bash scripts/run_wsad_teleop.sh
```

### 打开可视化控制台

```bash
bash scripts/run_control_center.sh
```

### 保存地图

```bash
bash scripts/save_map.sh
```

### 启动导航

```bash
bash scripts/run_navigation.sh
```

### 自动识别房间

```bash
bash scripts/send_room_command.sh "auto_rooms"
```

### 自动巡航所有房间

```bash
bash scripts/send_room_command.sh "explore_rooms"
```

---

## 16. 常见问题

### 1. Gazebo 里机器人动了，但 RViz 机器人不跟随

先查：

```bash
bash scripts/source_workspace.sh
ros2 topic echo /odom --once
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map base_footprint
```

如果这里断了，优先看：

- Gazebo 底盘插件
- EKF 配置
- slam_toolbox 日志

### 2. RViz 有激光，但地图不长

先查：

```bash
bash scripts/source_workspace.sh
ros2 topic echo /scan --once
ros2 topic echo /map --once
```

### 3. 明明改了世界文件，但 Gazebo 画面没变化

通常是旧进程还在。先执行：

```bash
bash scripts/cleanup_gazebo.sh
```

再重新启动。

### 4. `ros2 pkg prefix virtual_indoor_nav` 找不到包

先执行：

```bash
bash scripts/source_workspace.sh
```

### 5. `spawn_entity.py` 报 `No module named 'lxml'`

执行：

```bash
bash scripts/install_dependencies.sh
bash scripts/source_workspace.sh
python3 -c "import lxml; print(lxml.__file__)"
```

### 6. 想看完整排查清单

看这个文件：

```text
MAPPING_TROUBLESHOOTING.md
```

---

## 17. 推荐实际操作顺序

第一次完整跑通，按下面顺序来：

1. 安装依赖
2. 构建工作区
3. 清理 Gazebo
4. 启动建图
5. 打开可视化控制台并直接用主界面控制机器人
6. 开着机器人把所有房间走一遍
7. 保存地图
8. 启动导航
9. 运行 `auto_rooms`
10. 运行 `explore_rooms`

如果你以后继续让我接着做，最有用的开场信息是：

- 你现在已经跑到哪一步
- 当前目录是不是项目仓库根目录
- 下一步你最想改什么



