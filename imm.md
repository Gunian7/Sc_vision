# IMM 原理与当前工程实现说明

本文说明 `Sc_vision` 中新增的 `IMM`（Interacting Multiple Model，交互式多模型）是如何工作的、为什么适合变速小陀螺，以及当前代码里是怎么接入 `Tracker/Planner` 的。

---

## 1. 为什么需要 IMM

单一模型（单 KF/EKF）通常默认目标运动模式固定，例如“近似匀速转动”。
但实战中小陀螺会在不同模式间切换：

- 慢速转
- 匀速快速转
- 明显加减速（变速）

单模型在模式切换时会出现滞后或抖动。IMM 的核心思想是：

- 同时维护多个模型并行估计
- 每一帧根据观测结果给每个模型分配概率
- 用概率加权融合得到最终状态

这样能兼顾“稳态”和“突变”场景。

---

## 2. IMM 的标准流程（算法原理）

设模型数量为 $M$，每个模型有状态 $x_i, P_i$，模型概率为 $\mu_i$，模型转移矩阵为 $\Pi=[\pi_{ij}]$。

每一帧的流程：

1. **交互混合（Interaction / Mixing）**
   - 先计算模型 $j$ 的先验权重归一化常数：
     $$c_j = \sum_{i=1}^{M} \pi_{ij}\mu_i$$
   - 计算从模型 $i\rightarrow j$ 的混合权重：
     $$\mu_{i|j} = \frac{\pi_{ij}\mu_i}{c_j}$$
   - 用 $\mu_{i|j}$ 混合出模型 $j$ 的初始状态与协方差。

2. **各模型独立预测/更新（Model-matched KF）**
   - 每个模型用自己的状态转移矩阵与过程噪声做预测。
   - 用测量做更新，得到创新及其协方差。

3. **模型概率更新（Bayes）**
   - 计算每个模型的似然 $\Lambda_j$（通常高斯）：
     $$\Lambda_j \propto \exp\left(-\frac{1}{2}\nu_j^T S_j^{-1}\nu_j\right)/\sqrt{|S_j|}$$
   - 更新后验概率：
     $$\mu_j^+ = \frac{c_j\Lambda_j}{\sum_k c_k\Lambda_k}$$

4. **融合输出（Combination）**
   - 最终状态：
     $$\hat{x}=\sum_{j=1}^{M}\mu_j^+ x_j$$
   - 对角度类变量需做环绕处理（$[-\pi,\pi]$）。

---

## 3. 当前工程中的 IMM 实现（代码对应）

### 3.1 文件位置

- `tasks/auto_aim/imm.hpp`
- `tasks/auto_aim/imm.cpp`

类名：`auto_aim::SpinIMM`

### 3.2 状态定义

当前实现采用 3 维状态：

- `yaw`：目标航向角
- `w`：角速度
- `alpha`：角加速度（用于变速判定）

即：$x=[yaw,\,w,\,alpha]^T$。

### 3.3 三个并行模型

`SpinIMM::Mode`：

1. `slow_spin`
2. `constant_spin`
3. `variable_spin`

实现中三个模型的差异主要体现在：

- 状态转移矩阵 `F`（`variable_spin` 含 $0.5dt^2$ 与 $dt$ 的加速度项）
- 过程噪声 `Q`（`variable_spin` 设得更大，允许更剧烈变化）

### 3.4 模型转移矩阵

在 `SpinIMM::SpinIMM()` 中：

$$
\Pi=
\begin{bmatrix}
0.93 & 0.05 & 0.02\\
0.04 & 0.93 & 0.03\\
0.03 & 0.07 & 0.90
\end{bmatrix}
$$

含义：

- 对角线较大：模型有“自保持”倾向
- 非对角线较小：允许切换，但不会过于频繁

### 3.5 角度处理

`yaw` 是圆周变量，直接线性平均会出错。
当前实现使用：

- `normalize_angle()` 保持角度在 $[-\pi,\pi]$
- 混合时用 `sin/cos` 加权后 `atan2` 恢复角度（见 `blend_state()`）

这是 IMM 用于角度状态时的关键细节。

---

## 4. 如何接入 Tracker（当前实现）

接入函数：`Tracker::update_motion_state()`（`tasks/auto_aim/tracker.cpp`）

每帧逻辑：

1. 从 `target.ekf_x()` 取观测：
   - `yaw_measure = ekf_x[6]`
   - 平移速度 `v = hypot(ekf_x[1], ekf_x[3])`

2. 更新 IMM：
   - 首帧 `reset()`
   - 后续 `spin_imm_.update(yaw_measure, dt)`

3. 由 IMM 输出得到：
   - `w = imm.w`
   - `alpha = imm.alpha`

4. 计算角加速度指标（用于第 6 类“变速小陀螺”）：
   - 先差分 $dw/dt$
   - 再低通滤波：`imm_dw_lpf_ = 0.7*old + 0.3*new`

5. 依据阈值分类六状态：
   - `static_state`
   - `translate`
   - `spin_slow_inplace`
   - `move_slow_spin`
   - `spin_fast_inplace`
   - `spin_variable`

6. 写回 `Target`：
   - `target.set_motion_state(state)`
   - `target.set_imm_output(w, alpha)`

### 4.1 Tracker 可配置参数

在构造中支持读取：

- `motion_state_enable`
- `motion_v_enter`
- `motion_v_exit`
- `motion_w_low`
- `motion_w_high`
- `motion_dw_high`

对应“是否启用、平移判定、慢/快旋转分界、变速分界”。

---

## 5. 如何影响 Planner（当前实现）

位置：`tasks/auto_aim/planner/planner.cpp` 的 `Planner::plan(std::optional<Target>, ...)`

当前逻辑：

1. 用 `target->imm_w()`（若无则回退 `ekf_x()[7]`）判断高低速档：
   - 高速：`high_speed_delay_time`
   - 低速：`low_speed_delay_time`

2. 根据六状态再叠加额外 delay：
   - `spin_fast_inplace`：`+ imm_fast_spin_extra_delay`
   - `spin_variable`：`+ imm_variable_spin_extra_delay`

可配置参数：

- `imm_fast_spin_extra_delay`（默认 `0.015`）
- `imm_variable_spin_extra_delay`（默认 `0.03`）

这样做的目标是：在高风险状态（快速转、变速）下更保守，降低误打和相位错位。

---

## 6. 与“自适应 Kalman”关系

当前实现重点是 **多模型切换**（IMM），不是纯粹“在线改 Q/R”的自适应 KF。

- IMM 解决：模式切换（慢转 / 匀速 / 变速）
- 自适应 KF 解决：噪声强弱变化

工程上常见最优实践是：

- IMM 作为主框架
- 每个子模型内部再做轻量自适应（后续可加）

---

## 7. 调参建议（按优先级）

1. **先调六状态阈值**
   - `motion_w_low`, `motion_w_high`, `motion_dw_high`
2. **再调额外 delay**
   - `imm_fast_spin_extra_delay`, `imm_variable_spin_extra_delay`
3. **最后调 IMM 内部参数**
   - 转移矩阵 `transition_`
   - 三模型 `Q` 对角线

建议先日志回放看：

- `imm_w` 是否比 `ekf_x()[7]` 平滑且响应足够快
- `spin_variable` 是否只在明显加减速阶段出现
- 开火时误差是否因额外 delay 降低

---

## 8. 当前实现边界与后续可增强点

当前版本是“最小可用”实现，优点是侵入小、可快速上线验证；后续可增强：

1. 把 `mode_probability` 打到可视化日志（便于看模式切换是否合理）
2. 增加 `motion_state_min_hold`（防止频繁抖动切态）
3. 在 `Shooter` 侧直接使用 `motion_state` 做开火门控（例如 `spin_variable` 短暂抑制）
4. 在每个子模型内部引入轻量自适应 `Q/R`

---

## 9. 一句话总结

本项目的 IMM 实现本质是：

- 用 3 个旋转模型并行估计 `yaw/w/alpha`
- 通过模型概率在线选择“当前更像哪种旋转状态”
- 将结果映射到六状态并反馈给 `Planner` 做 delay 策略

从而提升变速小陀螺场景下的稳定性与实战命中一致性。
