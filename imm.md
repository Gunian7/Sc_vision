# IMM 原理与当前工程实现说明

本文说明 `Sc_vision` 中的 `IMM`（Interacting Multiple Model，交互式多模型）如何工作、为什么适合小陀螺旋转状态估计，以及当前代码里如何接入 `Tracker`。

---

## 1. 为什么需要 IMM

单一模型（单 KF/EKF）通常假设目标运动模式在一段时间内近似固定，例如“近似匀速转动”。

但实战中的小陀螺并不是始终满足同一种旋转模型，常见情况包括：

- 慢速转动
- 近似匀速快速转动
- 明显加减速的变速转动

如果只使用单模型，模式切换时往往会出现：

- 角速度估计滞后
- `yaw` 跟踪抖动
- 变速阶段响应不够及时

IMM 的核心思想是：

- 同时维护多个旋转模型并行估计
- 每一帧根据观测结果更新各模型概率
- 用模型概率加权融合得到最终状态

这样能在“稳态”和“突变”之间取得更好的平衡。

---

## 2. IMM 的标准流程（算法原理）

设模型数量为 $M$，每个模型有状态 $x_i, P_i$，模型概率为 $\mu_i$，模型转移矩阵为 $\Pi=[\pi_{ij}]$。

每一帧的流程为：

1. **交互混合（Interaction / Mixing）**
   - 计算模型 $j$ 的先验归一化常数：
     $$c_j = \sum_{i=1}^{M} \pi_{ij}\mu_i$$
   - 计算从模型 $i \rightarrow j$ 的混合权重：
     $$\mu_{i|j} = \frac{\pi_{ij}\mu_i}{c_j}$$
   - 用 $\mu_{i|j}$ 混合出模型 $j$ 的初始状态与协方差。

2. **各模型独立预测/更新（Model-matched KF）**
   - 每个模型使用自己的状态转移矩阵与过程噪声做预测
   - 每个模型独立用观测做更新，得到创新及其协方差

3. **模型概率更新（Bayes）**
   - 计算每个模型的似然 $\Lambda_j$
   - 更新后验模型概率：
     $$\mu_j^+ = \frac{c_j\Lambda_j}{\sum_k c_k\Lambda_k}$$

4. **融合输出（Combination）**
   - 最终融合状态：
     $$\hat{x}=\sum_{j=1}^{M}\mu_j^+ x_j$$
   - 对角度变量需要单独处理环绕问题，避免直接线性平均导致跳变

---

## 3. 当前工程中的 IMM 实现

### 3.1 文件位置

- `tasks/auto_aim/imm.hpp`
- `tasks/auto_aim/imm.cpp`

类名：`auto_aim::SpinIMM`

### 3.2 状态定义

当前 IMM 只负责**旋转相关状态**，采用 3 维状态：

- `yaw`：目标航向角
- `w`：角速度
- `alpha`：角加速度

即：

$$x=[yaw,\;w,\;\alpha]^T$$

注意：

- 这里的 IMM 关注的是**旋转维度**
- 目标的线速度仍保留在主 EKF 状态中
- **线速度与旋转状态分类解耦**

### 3.3 三个并行模型

`SpinIMM` 中维护 3 个旋转模型，对应当前代码里的三种 `SpinModel`：

1. `slow`
2. `constant`
3. `variable`

它们的含义分别是：

- `slow`：低角速度、接近慢转
- `constant`：角速度较高且相对稳定
- `variable`：存在明显角加速度，处于变速旋转阶段

实现中三个模型的主要差异体现在：

- 状态转移矩阵 `F`
- 过程噪声 `Q`
- 对 `alpha` 的衰减方式

其中 `variable` 模型允许更明显的角速度变化，因此通常具有更激进的过程噪声设置。

### 3.4 模型转移矩阵

默认模型转移矩阵在 `Tracker` 构造阶段通过 `SpinIMM::Params` 配置：

$$
\Pi=
\begin{bmatrix}
0.93 & 0.05 & 0.02\\
0.04 & 0.93 & 0.03\\
0.03 & 0.07 & 0.90
\end{bmatrix}
$$

含义：

- 对角线较大：模型具有较强自保持性
- 非对角线较小：允许在不同旋转模式之间切换，但不会频繁抖动

### 3.5 角度处理

`yaw` 是圆周变量，不能直接做普通线性平均。

当前实现对角度专门处理：

- 使用 `normalize_angle()` 保持角度在 $[-\pi,\pi]$
- 融合时用 `sin/cos` 加权，再通过 `atan2` 恢复角度

这是 IMM 应用于角度状态时的关键实现细节。

---

## 4. 如何接入 Tracker（当前实现）

接入函数为：

- `Tracker::update_motion_state()`  
- 位置：`tasks/auto_aim/tracker.cpp`

### 4.1 当前逻辑

每帧大致流程如下：

1. 准备融合状态
   - 默认使用 `target.ekf_x()` 作为状态来源
   - 若 `IMM` 已启用且已初始化，则使用 `spin_imm_.state()` 作为融合后的旋转状态来源

2. 若 `IMM` 已启用
   - 读取三模型概率：
     - `model_prob_slow`
     - `model_prob_constant`
     - `model_prob_variable`
   - 读取各模型的 `w / alpha`
   - 取概率最大的模型，直接映射为：
     - `SpinModel::slow`
     - `SpinModel::constant`
     - `SpinModel::variable`

3. 若 `IMM` 未启用，但 `motion_state_enable` 为真
   - 使用 fallback 规则做简单旋转分类：
     - 若 `|alpha| >= motion_dw_high`，判为 `variable`
     - 否则若 `|w| >= motion_w_low`，判为 `constant`
     - 否则判为 `slow`

4. 从融合状态中提取：
   - `w`
   - `alpha`
   - `vx, vy, vz`

5. 单独计算线速度模长：
   $$
   linear\_speed = \sqrt{vx^2 + vy^2 + vz^2}
   $$

6. 将结果写回 `Target`
   - `target.set_spin_state(spin_state)`
   - `target.set_linear_speed(linear_speed)`
   - `target.set_imm_output(w, alpha)`

### 4.2 当前设计要点

这里最重要的一点是：

- **旋转状态分类只由角速度/角加速度相关信息决定**
- **线速度不再参与旋转类别枚举**
- **平移信息通过 `linear_speed` 单独输出**

也就是说，当前工程已经不再使用旧的“把平移和旋转混在一起”的复合 `MotionState` 设计。

---

## 5. Tracker 可配置参数

当前与旋转状态分类直接相关的配置项包括：

- `motion_state_enable`
- `enable_imm`
- `motion_w_low`
- `motion_dw_high`

含义如下：

- `motion_state_enable`
  - 是否启用旋转状态分类逻辑
- `enable_imm`
  - 是否启用 IMM 三模型融合
- `motion_w_low`
  - fallback 模式下的角速度阈值，`|w|` 超过后判为 `constant`
- `motion_dw_high`
  - fallback 模式下的角加速度阈值，`|alpha|` 超过后判为 `variable`

此外 IMM 本身也支持以下参数配置：

- `imm_transition`
- `imm_r_yaw`
- `imm_q_slow`
- `imm_q_constant`
- `imm_q_variable`
- `imm_alpha_decay_slow`
- `imm_alpha_decay_constant`
- `imm_dt_min`
- `imm_dt_max`

---

## 6. 输出到日志/可视化的数据

当前 `Tracker::update_motion_state()` 会输出一组与 IMM 相关的调试量，便于回放分析：

- `imm_enabled`
- `spin_state`
- `linear_speed`
- `fused_w`
- `fused_alpha`
- `ekf_w`
- `ekf_alpha`
- `imm_dw_lpf`
- `model_prob_slow`
- `model_prob_constant`
- `model_prob_variable`
- `model_w_slow`
- `model_w_constant`
- `model_w_variable`
- `model_alpha_slow`
- `model_alpha_constant`
- `model_alpha_variable`

这些量可用于判断：

- 模型切换是否稳定
- `IMM` 输出是否比裸 EKF 更平滑
- `variable` 是否只在明显加减速阶段出现
- 线速度与旋转状态是否已经实现良好解耦

---

## 7. 与“自适应 Kalman”关系

当前实现的重点是 **多模型切换**，不是纯粹的“在线调整 Q/R”的自适应 KF。

两者关注点不同：

- IMM 主要解决：**运动模式切换**
  - 慢转 / 匀速转 / 变速转
- 自适应 KF 主要解决：**噪声统计变化**
  - 某一阶段观测更抖
  - 某一阶段模型误差更大

工程上常见做法是：

- 先用 IMM 解决模式切换问题
- 再视需要在各子模型内部做轻量自适应

当前仓库属于前者。

---

## 8. 调参建议

建议按以下优先级调参：

1. **先调三模型分类行为**
   - `motion_w_low`
   - `motion_dw_high`

2. **再调 IMM 本体参数**
   - `imm_transition`
   - `imm_r_yaw`
   - `imm_q_slow`
   - `imm_q_constant`
   - `imm_q_variable`
   - `imm_alpha_decay_slow`
   - `imm_alpha_decay_constant`

3. **最后结合日志回放观察效果**
   - `fused_w` 是否比 EKF 原始角速度更稳
   - 模型概率是否在合理区间切换
   - `variable` 是否只在确实变速时占主导
   - `linear_speed` 是否能稳定反映平移强度

---

## 9. 当前实现边界

当前版本是较小侵入的工程实现，特点是：

- 只对旋转维度引入 IMM
- 保留主 EKF 的平移状态
- 通过 `SpinModel + linear_speed` 输出给后级模块使用

这意味着：

- 它不是完整的“全状态 IMM”
- 它的优势集中在小陀螺旋转状态识别与角速度估计
- 平移与旋转的语义已经解耦，便于后续模块独立使用

---

## 10. 一句话总结

本项目当前的 IMM 实现本质上是：

- 用 3 个旋转模型并行估计 `yaw / w / alpha`
- 用模型概率直接输出三种旋转状态：`slow / constant / variable`
- 将线速度作为独立量 `linear_speed` 单独输出

从而避免旧的复合 `MotionState` 设计，把“旋转分类”和“平移强度”分开处理。
