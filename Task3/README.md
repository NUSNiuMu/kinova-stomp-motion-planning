# Task 3: Product of Exponentials (PoE) 正向运动学实现

## 📌 任务要求
使用基于扭转理论的指数积（Product of Exponential, PoE）公式编写正向运动学程序，替换MATLAB内置的`getTransform()`函数。

## ✅ 实现状态
**已完成并优化** ✨

---

## 🔧 核心实现

### 主文件：`updateJointsWorldPosition.m`

#### 1. **PoE公式应用**
```matlab
T_i = exp([S_1]θ_1) · exp([S_2]θ_2) · ... · exp([S_i]θ_i) · M_i
```

其中：
- `S_i`: 第i个关节的空间螺旋轴 (6×1向量)
- `θ_i`: 第i个关节角度
- `M_i`: home configuration时第i个body的变换矩阵

#### 2. **螺旋轴计算方法**
```matlab
% 使用几何雅可比矩阵提取螺旋轴
Jspace = geometricJacobian(robot_struct, homeConfig, bodyName);
S_i = Jspace(:, i);  % 第i列对应第i个关节的螺旋轴
```

#### 3. **指数映射（Rodrigues公式）**
对于旋转关节：
- **旋转部分**: R = I + sin(θ)[ω] + (1-cos(θ))[ω]²
- **平移部分**: p = G·v，其中 G = I·θ + (1-cos(θ))[ω] + (θ-sin(θ))[ω]²

#### 4. **性能优化**
- 使用`persistent`变量缓存螺旋轴和M矩阵
- 仅在机器人首次调用时计算一次
- 后续调用直接使用缓存，显著提升效率

---

## 📂 文件说明

### 主要代码文件

| 文件名 | 功能 | 状态 |
|--------|------|------|
| `updateJointsWorldPosition.m` | PoE正向运动学核心实现 | ✅ 完成+优化 |
| `helperSTOMP.m` | STOMP主算法循环 | ✅ 完成+优化 |
| `stompDTheta.m` | 梯度估计计算 | ✅ 完成 |
| `stompSamples.m` | 轨迹采样 | ✅ 完成 |
| `stompObstacleCost.m` | 障碍物代价函数 | ✅ 完成+优化 |
| `stompTrajCost.m` | 轨迹总代价计算 | ✅ 完成+优化 |

### 辅助文件
- `helperCreateObstaclesKINOVA.m` - 障碍物环境构建
- `helperInitialVisualizerKINOVA.m` - 可视化初始化
- `stompRobotSphere.m` - 机器人球形包络
- `sEDT_3d.m` - 3D距离场计算

---

## 🎯 关键技术点

### 1. **螺旋轴的物理意义**
螺旋轴 S = [ω; v] 包含：
- `ω`: 旋转轴方向（单位向量）
- `v`: 线速度分量（考虑旋转中心位置）

### 2. **为何仍需getTransform()**
在`computePoEParameters()`函数中使用一次`getTransform()`来获取M_i矩阵是**合理且必要的**：
- M_i表示home configuration时各body的初始姿态
- 这是PoE公式的基准配置
- 符合教材建议："仅需计算一次螺旋轴"

### 3. **数值稳定性处理**
```matlab
if omegaNorm < 1e-9  % 处理移动关节
    R = eye(3);
    p = v * theta;
else
    omega = omega / omegaNorm;  % 重归一化
    ...
end
```

---

## 🚀 使用方法

### 运行主程序
```matlab
% 在MATLAB中运行
run('RunLiveScript.m')
% 或直接打开并运行
open('KINOVA_STOMP_Path_Planning.mlx')
```

### 参数配置
在`helperSTOMP.m`中可调整：
```matlab
nDiscretize = 20;  % 轨迹离散化点数
nPaths = 20;       % 采样路径数量
convergenceThreshold = 0.1;  % 收敛阈值
```

---

## 📊 验证结果

### 语法检查
```
✅ updateJointsWorldPosition.m - 无错误
✅ helperSTOMP.m - 无错误  
✅ stompTrajCost.m - 无错误
⚠️ stompObstacleCost.m - 1个警告（误报，可忽略）
```

### 功能测试
- [x] PoE公式正确计算正向运动学
- [x] 与getTransform()结果一致
- [x] 缓存机制正常工作
- [x] STOMP算法收敛
- [x] 路径规划成功

---

## 📈 优化亮点

1. **代码文档完善**
   - 详细的函数注释
   - 清晰的实现说明
   - 数学公式说明

2. **错误处理增强**
   - 完善的异常捕获
   - 有意义的警告信息
   - 安全的fallback机制

3. **代码结构优化**
   - 消除冗余代码
   - 改进可读性
   - 统一代码风格

4. **性能提升**
   - 缓存机制避免重复计算
   - 高效的矩阵运算
   - 合理的数据结构

---

## 📚 参考资料

- **教材**: *Modern Robotics: Mechanics, Planning, and Control*
- **PoE公式**: Chapter 4 - Forward Kinematics
- **STOMP算法**: ICRA 2011 Paper

---

## 👥 开发团队

**项目**: EE5112 Human Robot Interaction - Project 2  
**日期**: 2025年11月6日  
**优化状态**: ✅ 完成

---

## 📌 下一步

准备进行：
- **Task 4**: 创建自定义避障场景
- **Task 5**: 添加末端执行器方向约束

**当前Task 3已完全就绪！** 🎉
