# Task5 使用说明（中文）

本任务在 Kinova Gen3 上实现“末端执行器轴对齐”的运动规划。通过 STOMP 算法生成避障轨迹，并在中间时刻对末端某个轴与世界目标轴的对齐进行约束或惩罚。同时，终点位置与姿态可分别配置为“严格锁定”或“仅位置锁定”。

## 运行入口

- `RunTask5_Compare.m`：对比运行，两次 STOMP（无约束与有约束）叠加显示差异，并打印末端到目标位置的距离。

运行方式：在 MATLAB 当前目录为项目根目录时，直接运行上述脚本即可。

## 关键文件

- `Task5/helperSTOMP.m`：STOMP 规划主流程（Task5 覆盖版本）。
- `Task5/stompTrajCost.m`：代价函数，含避障和姿态惩罚；支持角度阈值铰链惩罚 `angle_hinge`。
- `Task5/RunTask5_Compare.m`：场景设置、IK 端点处理、可视化与结果输出。

## 主要参数（在脚本顶部）

- `task5_enable`：是否开启姿态约束/惩罚。
- `task5_keep_axis`：选择对齐的 EE 轴，取值 `'x'|'y'|'z'`。
- `task5_world_axis`：世界目标轴向量（3x1），会归一化使用。
- `task5_penalty_metric`：姿态惩罚度量，推荐 `'angle_hinge'`（阈值内不罚，超出用铰链平方惩罚）。
- `task5_angle_threshold_deg`：角度阈值（度）。
- `task5_penalty_weight`：姿态惩罚权重，权重大对齐更严格。
- `task5_strict_goal_lock`：是否严格锁定目标位置（true 不自动挪动目标）。

STOMP 分辨率与采样：
- `stomp_nDiscretize`（离散点数量）、`stomp_nPaths`（采样路径数量）、`stomp_eta`（步长系数）、`voxel_size_override`（体素尺寸）等在脚本顶部可调。

## 起点与终点的设置（做法二：终点方向对齐并锁定）

- 起点位置：取机器人 `homeConfiguration` 下的 EE 位置 `startPos`。
- 起点姿态：根据 `task5_keep_axis` 与 `task5_world_axis` 构造 `R_start`，用 IK 锁定为起点关节。
- 终点位置：`goalPos`（按场景选择或手动指定）。
- 终点姿态：根据对齐规则构造 `R_goal`，并在 IK 中使用位置+姿态全权重 `[1 1 1 1 1 1]` 锁定（保证终点 EE 轴与世界轴一致）。
- IK 求解器已增强迭代与容忍度，确保终点位置与姿态更精确。

若需仅锁定终点位置、放开终点姿态：把权重改为 `[1 1 1 0 0 0]`。

## 可视化增强

- 图中清晰标注：
  - 无约束路径（红线）与有约束路径（蓝线）。
  - 目标位置红点（`Goal position`）。
  - 末端选定轴的箭头（采样时刻，颜色随轴变）。
  - 世界目标轴箭头（颜色自动映射）。
- 图例已清理：只显示上述核心对象，避免出现大量 `dataX` 条目。

## 结果与输出

- 对比脚本会打印末端到目标的最终距离（单位 cm），并在距离大于 1 cm 时告警。
- 叠加绘制两条 EE 路径，便于观察约束带来的差异。
- 会保存两条轨迹：`Theta_Task5_Compare_Before.mat` 与 `Theta_Task5_Compare_After.mat`。

## 常见问题与建议

- 末端未能到达目标位置：
  - 检查 `task5_strict_goal_lock` 是否过于严格导致目标在障碍空间不可达；可暂时设为 `false` 允许脚本微调目标至自由空间。
  - 适度增加 `stomp_nDiscretize`/`stomp_nPaths`，提高轨迹质量。
  - 提高 IK 求解迭代步数或降低容忍度（脚本已增强，仍可继续调小）。
- 姿态对齐不够紧：
  - 降低 `task5_angle_threshold_deg`（如 8→5）。
  - 增大 `task5_penalty_weight`（如 300→900）。
- 更换对齐轴或世界轴：
  - 改 `task5_keep_axis` 与 `task5_world_axis`，脚本会自动重算起点/终点姿态与可视化箭头。

## 复现实验（建议流程）

1. 运行 `RunTask5_Compare.m`，观察两条路径与打印的末端距离。
2. 若距离或对齐不满意：调 `task5_angle_threshold_deg` 与 `task5_penalty_weight`，必要时增大采样与离散点。
3. 确认终点姿态是否需要锁定：
   - 需要锁定：保持 `[1 1 1 1 1 1]`（做法二）。
   - 只锁位置：改为 `[1 1 1 0 0 0]`（做法一）。

## 备注

- 障碍物与体素环境由 Task4 的辅助方法构建，场景通过 `scenario_id` 选择。
- 代码风格尽量保持与原仓库一致，未对无关模块做改动。