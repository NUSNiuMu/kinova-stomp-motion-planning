# Kinova STOMP Motion Planning

## 项目概览
- MATLAB Live Script `KINOVA_STOMP_Path_Planning.mlx` 演示使用 STOMP（`Stochastic Trajectory Optimization for Motion Planning`）为 7 关节机械臂生成无碰撞轨迹。
- 代码以课程 EE5112 Project 2 为基础，包含环境建模、路径优化、碰撞检测与可视化记录等环节。
- 主入口 `RunLiveScript.m` 直接调用 Live Script，辅助函数位于 `EE5112_Project_2_Collision-Avoidance-Code_Incomplete/`。

## 环境依赖
- MATLAB R2022b 及以上（已测试函数均使用 Robotics System Toolbox API）。
- Robotics System Toolbox、Optimization Toolbox、Image Processing Toolbox（用于 SDF 与视频导出）。
- 建议在具有 GPU 加速的机器上运行以获得更顺畅的可视化效果。

## 快速上手
1. 在 MATLAB 当前路径设为仓库根目录或 `EE5112_Project_2_Collision-Avoidance-Code_Incomplete/`。
2. 运行 `RunLiveScript.m` 打开并执行 Live Script，逐节观察机器人模型、环境及 STOMP 规划过程。
3. 规划完成后脚本会导出训练过程与最终轨迹视频 (`KinvaGen3_Training.avi`, `KinvaGen3_wEEConY3.avi`) 以及优化后的轨迹参数 (`Theta_nDisc*_nPaths_*.mat`)。

## 目录速览
- `KINOVA_STOMP_Path_Planning.mlx`：主 Live Script，集成机器人建模、环境初始化、STOMP 调用。
- `RunLiveScript.m`：便捷入口，直接运行 Live Script。
- `helperCreateObstaclesKINOVA.m`：构建体素化障碍与签距场。
- `helperInitialVisualizerKINOVA.m`：绘制初始姿态、终点目标与障碍。
- `helperSTOMP.m`：STOMP 主循环、渐进更新、碰撞检测与结果导出。
- `stomp*.m`：STOMP 相关子函数（采样、代价、概率、梯度、关节球体等）。
- `Theta_nDisc*_nPaths_*.mat`：示例轨迹结果。

## Task 1：机器人模型与目标姿态
- 目标：加载 Kinova/KUKA 机械臂模型，设定初始与期望末端位姿，并用逆运动学获得关节角。
- 当前进度：`KINOVA_STOMP_Path_Planning.mlx` 已完成配置，默认使用 `loadrobot('kukaIiwa14','DataFormat','column')` 与 `inverseKinematics` 求解，终点姿态保存在 `poseFinal` 与 `finalRobotJConfig`，可按需求更换模型或位姿。

## Task 2：碰撞环境建模
- 目标：按 STOMP 论文构建体素世界以及签距场，提供障碍物的碰撞代价查询。
- 当前进度：`helperCreateObstaclesKINOVA.m` 已实现体素化立方体障碍、`sEDT_3d` 求 signed EDT，并对外暴露 `voxel_world` 结构；可扩展 `world` 列表叠加更多障碍并同步更新体素占据。

## Task 3：STOMP 轨迹优化内环
- 目标：实现采样扰动、局部代价评估、软最小概率加权、梯度估计与平滑更新。
- 当前进度：`helperSTOMP.m` 已填充 TODO 代码，调用 `stompSamples`、`stompTrajCost`、`stompDTheta` 等函数完成完整迭代流程，迭代停止条件包含最小代价收敛、最大 50 次迭代与零梯度保护；仍建议根据实验调参 `eta`、`convergenceThreshold` 与 `nPaths`。

## Task 4：碰撞校验与结果记录
- 目标：在优化结束后验证整条轨迹的碰撞状态，并输出可视化/视频/数据文件。
- 当前进度：`helperSTOMP.m` 已对每个离散点调用 `checkCollision` 记录碰撞对，提供动画播放与两段视频写入；同时保存轨迹矩阵 `theta` 至 `Theta_nDisc*_nPaths_*.mat`，可进一步拓展成性能统计或日志记录。

## Task 5：末端执行器姿态约束
- 目标：在规划时新增末端执行器姿态约束，使其选定轴（如 y 轴）始终与世界坐标指定方向对齐，并展示加入约束前后的规划差异。
- 当前进度：尚未实现。需在 STOMP 代价函数或更新步骤中加入约束项（例如姿态误差惩罚），或对初末姿态重新设定以匹配期望方向，并更新可视化对比结果。

## 下一步建议
- 若需替换真实 Kinova Gen3 模型，调整 `robot_name` 及末端执行器名称，并确认关节限幅。
- 根据课程要求补充实验报告、性能指标或与 MPC 等方法对比的章节。
- 可在 README 保持任务状态最新，便于团队同步。
