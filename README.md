## EE5112 Project 2 AY 25/26

### 使用 STOMP 的机器人机械臂避障路径规划

---

## 📘 注意事项

- 本项目占课程评估（CA）的 **30%**。

  - 报告和代码占 **20%**，
  - 演示占 **10%**。
- 评分将基于：

  - 代码的正确性、质量、效率、实际考量以及新增功能或新颖实现；
  - 设计与规划结果的挑战性与创新性；
  - 演示的清晰度、有效性与视觉呈现；
  - 你对相关定理与算法的理解。
    请在报告与演示中突出以上任一点。
- 请保持诚实，不要做虚假陈述。
- 这是一个小组项目，但每位成员都应有贡献并展示个人部分。个人在演示中的表现将单独评估。
- 将提供一个**不完整的示例代码**，供小组在其上进行扩展。

---

## 🧩 任务说明

### **任务 1**

完善提供的示例代码，使得可运行 MATLAB 实时脚本
`KINOVA_STOMP_Path_Planning.mlx`，在原始避障设置下成功规划无碰撞路径并显示动画。

以下文件是不完整的：

```

helperSTOMP.m

updateJointsWorldPosition.m

stompDTheta.m

stompSamples.m

stompObstacleCost.m

```

---

### **任务 2**

选择一个**不同的机器人机械臂**，并在类似于任务 1 的场景中为其规划路径。

- 在 MATLAB 函数 [`loadrobot()`](https://www.mathworks.com/help/robotics/ref/loadrobot.html) 的帮助页中可以找到自带的机器人模型列表。
- 你也可以加载外部机器人模型。可参考 MoveIt 支持的机器人列表：👉 [https://moveit.ros.org/robots/](https://moveit.ros.org/robots/)
- MATLAB 函数 `importrobot()` 允许通过 URDF 文件导入外部机器人。
  若要在 MATLAB 中可视化，还需指定网格文件路径。

例如，可使用 Kuka KR210 （一个 6 自由度旋转关节机械臂）的 xacro 与 mesh 文件。
可用以下 ROS 命令将 xacro 转换为 URDF：

```bash
rosrun xacro xacro model.xacro > model.urdf
```

此后，余下的任务均基于你选择的机器人进行。

---

### **任务 3**

使用**基于扭转理论的指数积（Product of Exponential, PoE）公式**编写正向运动学程序，

替换 MATLAB 内置的 `getTransform()` 函数。

* 可使用教材（ *Modern Robotics* ）提供的 `FKinSpace.m` 等代码（在 [GitHub](https://github.com/NxRLab/ModernRobotics/tree/master/packages/MATLAB) 上可获取）。
* 你需要根据所选机器人的几何参数确定每个关节的扭转（twist），

  这可通过检查相应 URDF 文件或各链接的齐次变换矩阵获得（参见 §1.1）。
* 在报告和演示中应清楚说明如何确定这些扭转向量。
* 注意代码效率的实现与集成。

---

### **任务 4**

创建你自己的 **避障场景** ，例如：

* 添加其他障碍物，
* 使用不同的起始与目标配置。

展示路径规划结果时，可叠加中间姿态或使用动画。

场景的难度与新颖性将影响评分。

⚠️ 不要使用一开始就无碰撞的“平凡”初始化路径。

---

### **任务 5**

在前述任务基础上，进一步为机器人末端执行器添加 **方向约束** ：

使其 *y* 轴在从初始位置到目标位置的移动过程中保持竖直（例如搬运咖啡杯的任务）。

* 你也可以选择 *x* 或 *z* 轴，并将其与世界坐标系中选定的轴对齐。
* 展示添加约束前后的路径规划差异。
* 由于代码中已固定初末姿态，你可调整末端执行器的初末方向以符合所选约束方向，便于规划。

---

## 🧠 备注

* 必须理解整个代码逻辑才能调试并完成所有任务。
* 可根据需要修改给定代码，例如：

  * 采用自定义数据格式；
  * 根据所选机器人最大伸展范围调整障碍物位置；
  * 改进计算效率或结构。
* 可自由使用外部函数、库或代码片段。
* 任何额外贡献（如更高效的计算、添加 GUI 交互界面、修复 bug、改进 STOMP 算法等）

  都会纳入评分，请在报告和演示中明确指出。

---

## 📦 提交要求

* 提交  **PDF 报告与 MATLAB 代码** 。
* 所有文件放入同一文件夹并压缩为一个 zip。
* 每组仅提交 1 个 zip 文件。

命名规范：

```
Group<number>_Report_Code.zip
```

提交截止：🕔 **2025 年 11 月 13 日 17:00**

建议：

* 为每个任务单独建文件夹，或建立清晰结构以便复现全部结果。
* 无需包含 *Modern Robotics* 教材原始代码（若未修改）。
* 附上 `readme.txt` 说明文件是良好实践。
* 所有任务均可使用 PoE 实现。

---

## 🎤 演示说明

* 时间：**2025 年 11 月 13 日（星期四）18:00 – 21:00**
* 地点：**E1-06-07**
* 每组 10 分钟：
  * 7 分钟演示
  * 3 分钟问答

说明：

* 可播放预录视频进行展示，随后进行现场 Q&A（所有成员须到场）。
* 每位成员需讲解小组工作的一部分。
* 演示顺序按小组编号（如需调整，请联系助教）。
* 可使用个人笔电或教室电脑。
* 整个演示将被录制（现场进行）。

---

## ⚙️ 1.1 技术提示 (Technical Tips)

* 示例代码依赖  **MATLAB Robotics System Toolbox** 。安装时请确保选中。
* 获取机器人 body 名称时，以下写法速度更快：
  ```matlab
  robot.BodyNames{k}           % 快
  robot.Bodies{k}.Name         % 慢
  ```
* 某些机器人（如 UR 系列）起始链接为固定关节，请确保规划的关节角对应实际可动关节。
* 注意部分 URDF 文件中 `base_link` 原点与基础坐标系不同，应保持引用一致。

示例：

```matlab
getTransform(robot, robot.homeConfiguration, robot.BodyNames{2})              % 基于 base frame
getTransform(robot, robot.homeConfiguration, robot.BodyNames{2}, 'base_link') % 可能不同
```

* 获取几何参数示例：
  ```matlab
  robot = loadrobot('kinovaGen3', 'DataFormat','column');
  show(robot);
  robot.Bodies{1}.Joint.JointToParentTransform
  ```
* 也可通过 URDF 文件查看几何信息，可用以下命令查找本地路径：
  ```matlab
  [model, robotData] = loadrobot('kinovaGen3');
  robotData.FilePath
  ```
* 仅需计算一次螺旋轴（screw axes），其取决于初始配置。更新关节时直接使用 PoE 公式，无需重新计算。
* 计算螺旋轴方法：
  1. 在零位（home configuration）使用 `getTransform()` 获取每个关节的旋转轴与位置向量，用叉积计算；
  2. 或在任意非零角度下求姿态，用矩阵对数映射得到 twist，再归一化获得螺旋轴。
* **末端执行器方向约束（任务 5）**
  * 方法 1：选取 *y* 轴竖直的旋转矩阵，并将惩罚项加入代价函数。
  * 方法 2：惩罚末端执行器 *y* 轴方向 ŷₛᵦ 与世界坐标 *z* 轴方向 ẑₛ 的差异。

    代价可取残差向量的 L₁ 范数：

    $$
    [

    |ŷ_{sb} - ẑ_s| *1 = \sum_i |ŷ* {sb}(i) - ẑ_s(i)|

    ]
    $$

    MATLAB 计算方式：`norm(y_sb - z_s, 1)`
* 多元高斯采样函数 `mvnrnd` 需要  **Statistics and Machine Learning Toolbox** 。
* 关节角度初始化影响较大，应处理 θ ∈ [−π, π] 的不连续性。
* MATLAB 提供的球面线性插值函数 `slerp()` 仅适用于四元数表示的旋转。

---

> 📩 Lecturer: **Dr Lin Zhao**  ([elezhli@nus.edu.sg](mailto:elezhli@nus.edu.sg))
>
> School of Electrical and Computer Engineering, National University of Singapore
