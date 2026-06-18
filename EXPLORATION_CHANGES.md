# 自动探索修复说明

本次修复针对 Frontier Exploration 提前结束、局部震荡、未知区域覆盖不完整的问题。

## 代码改动

- `workspace/src/virtual_indoor_nav/virtual_indoor_nav/auto_explorer.py`
  - 使用两个独立定时循环：运动控制循环和前沿检测循环解耦，避免导航动作阻塞地图刷新。
  - 前沿聚类后使用综合评分排序目标：
    - 信息增益：候选点传感器范围内可打开的未知边界数量。
    - 距离代价：避免过远目标，但不再只按最近点选择。
    - 前沿面积：优先较大的连续未知边界。
    - 历史方向惩罚：减少在同一方向反复震荡。
  - 增加 `blocked_goals`，多次恢复失败的目标不再重复选择。
  - 增加覆盖率计算并发布 `/exploration_coverage`，日志输出 `coverage xx.x% explored=N active=M`。
  - 增加停滞检测：覆盖率在窗口内提升低于阈值且未达到完成覆盖率时，触发强制探索。
  - 强制探索目标现在选择”已知 free 且相邻 unknown”的边界单元格，而不是直接把目标放到 unknown 单元格。
  - **新增 3 轮无 frontier 验证机制**：
    - 每轮检测到 zero frontiers → `clear_frontier_rounds += 1`
    - 发现新 frontier 或 forced_exploration 成功 → `clear_frontier_rounds = 0`（清零）
    - 连续 3 轮无 frontier → 才判定 exploration_complete
    - 日志输出 `frontier_clear_round N/3`

## 配置改动

- `workspace/src/virtual_indoor_nav/config/exploration_params.yaml`
  - 新增 `verification_required_clear_rounds: 3` — 连续无前沿轮数要求
  - 新增 `verification_round_interval: 2.0` — 每轮检测间隔（秒）
  - 集中管理自动探索参数，包括前沿最小面积、目标评分权重、覆盖率完成阈值、恢复策略和窄通道速度。

- `workspace/src/virtual_indoor_nav/config/nav2_exploration_costmap_template.yaml`
  - 提供探索场景下的 Nav2 costmap 推荐模板。

- `workspace/src/virtual_indoor_nav/launch/auto_mapping.launch.py`
  - 自动加载 `exploration_params.yaml`。

- **新增** `workspace/src/virtual_indoor_nav/launch/app_system.launch.py`
  - 统一启动文件：一次 launch 启动 Gazebo + SLAM + Nav2 + auto_explorer + room_nav_node + RViz
  - Nav2 与 SLAM 同一会话内运行（在线 SLAM 地图，不切 AMCL）
  - auto_explorer 和 room_nav_node 延迟启动以等待 Nav2 lifecycle 就绪

## 预期效果

- 机器人不会因为短时间无前沿就立即结束探索。
- 连续 3 轮验证确保探索完整性（约额外 6 秒确认时间）。
- 目标选择更偏向能打开未知空间的区域，减少只追最近点造成的局部循环。
- 狭窄通道不容易被 inflation 过度封死。
- 当前覆盖率、有效探索区域和已探索单元数可通过日志和 topic 实时观察。

## 常用调参方向

- 未知区域仍很多但过早完成：提高 `min_coverage_for_done`，或提高 `verification_required_clear_rounds`。
- 前沿噪声太多：提高 `frontier_min_area` 到 `0.10` 或 `0.20`。
- 小房间/窄通道漏探索：降低 `frontier_min_area` 到 `0.02`，并保持 local inflation 不高于 `0.25`。
- 机器人在局部来回震荡：提高 `score_weight_history`，或降低 `score_weight_dist`。
