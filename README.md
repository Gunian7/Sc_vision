# Phoenix Vision Module
### 概述：
本项目为杭州电子科技大学Phoenix战队26赛季小弹丸兵种视觉自瞄模块，基于同济大学SP战队25赛季视觉系统进行优化和重构，适配小弹丸兵种的特殊需求。核心功能包括装甲板识别、坐标解算、状态估计、击打点选择和弹道解算，目标实现高命中率和短击杀时间，既满足中期和完整形态考核的性能要求而添加应试性参数接口，也满足赛场实战的鲁棒性和适应性需求。

## 1. 功能简介和工作流

### 1.1 功能简介

自动瞄准（AutoAim）是视觉组核心功能之一，定义为**针对移动装甲板目标的自动瞄准与自动火控软件**。操作手切换至自瞄模式后，系统接管云台控制权和发射机构控制权，通过对敌方运动轨迹的预测和弹道解算，控制云台追踪目标并判断最优开火时机，目标是实现**短击杀时间**（面对 300HP 步兵 2m 处，7 rad/s 约 8s，14 rad/s 约 10s）和**高命中率**（国赛不低于 30%）。

### 工作流

整体数据流如下：

```
相机（图像 + 时间戳）
        ↓
    Detector（装甲板识别）
        ↓ 装甲板像素坐标 + 类别
    Solver（坐标解算）
        ↓ 装甲板世界坐标 + 姿态
    Tracker（状态估计器）
        ↓ 整车运动状态（EKF 滤波后）
    Aimer / Planner（决策器）
        ↓ yaw / pitch 指令 + 开火决策
   C 板（下位机控制器）
        ↓
    云台电机 + 发射机构
```

各模块职责：

| 模块 | 类 | 功能 |
|---|---|---|
| 识别器 | `Detector` / `Yolo` | 对图像进行装甲板检测，输出四点像素坐标和装甲板编号 |
| 坐标解算 | `Solver` | 利用相机内参和手眼标定结果，将像素坐标转换为世界坐标系下的位姿 |
| 状态估计 | `Tracker` + `Target` | 对整车运动状态进行 EKF 滤波，输出旋转中心坐标、角速度、装甲板半径等 |
| 决策器 | `Aimer` / `Planner` | 预测未来击打时刻装甲板位置，选择最优击打点，进行弹道解算，输出云台角度指令 |
| 开火控制 | `Shooter` | 根据位置误差和发弹延迟判断开火时机 |

---

## 1.2 重要算法原理阐述及推导

### 一、整车状态估计器（EKF）

**状态量定义**（11 维）：

$$\mathbf{x} = [x,\ \dot{x},\ y,\ \dot{y},\ z,\ \dot{z},\ \alpha,\ \omega,\ r,\ l,\ h]^T$$

其中：
- $(x, y, z)$：整车旋转中心世界坐标（m）
- $(\dot{x}, \dot{y}, \dot{z})$：旋转中心速度
- $\alpha$：当前装甲板角度（rad）
- $\omega$：角速度（rad/s），即"小陀螺转速"
- $r$：装甲板半径（第一块），即旋转中心到装甲板的水平距离
- $l = r_2 - r_1$：长短轴差（4 块装甲板机器人用）
- $h = z_2 - z_1$：上下高度差

**预测模型**（匀速运动假设）：

状态转移矩阵 $F$ 对位置-速度对采用匀速模型，即每个维度 $[p, v]$ 满足：

$$p_{k+1} = p_k + v_k \cdot dt, \quad v_{k+1} = v_k$$

过程噪声采用分段白噪声模型（Piecewise White Noise），对位置分量和角度分量分别设置方差 $v_1$（普通目标：100）和 $v_2$（普通目标：400）。前哨站因角速度稳定，方差更小（$v_1=10,\ v_2=0.1$）。

**观测量**（4 维）：

$$\mathbf{z} = [yaw,\ pitch,\ distance,\ \alpha_{armor}]^T$$

由 `Solver` 提供的装甲板世界坐标转换而来，观测噪声矩阵 $R$ 依据装甲板与旋转中心的夹角和距离动态调整，远距离和大角度时噪声更大：

$$R_{distance} = \log(|distance| + 1) + 1, \quad R_{angle} = \frac{\log(|\delta_{angle}| + 1)}{200} + 0.09$$

**EKF 更新**（使用 Joseph 稳定公式）：

$$K = P H^T (H P H^T + R)^{-1}$$
$$P = (I - KH)P(I - KH)^T + KRK^T$$
$$\mathbf{x} \leftarrow \mathbf{x} \oplus K(\mathbf{z} - h(\mathbf{x}))$$

其中 $\oplus$ 为自定义加法（角度量需取模限制到 $[-\pi, \pi]$）。使用 NIS（Normalized Innovation Squared）进行卡方检验，监控滤波器一致性：

$$\text{NIS} = \nu^T S^{-1} \nu, \quad S = HPH^T + R$$

---

### 二、弹道解算

不考虑空气阻力，子弹在竖直平面内做抛体运动。设初速度 $v_0$，水平距离 $d$，目标高度差 $h$，仰角为 $\theta$：

$$h = d \tan\theta - \frac{g d^2}{2 v_0^2 \cos^2\theta}$$

利用 $\sec^2\theta = 1 + \tan^2\theta$，整理为关于 $\tan\theta$ 的二次方程，令 $a = \dfrac{gd^2}{2v_0^2}$：

$$a(1 + \tan^2\theta) - d\tan\theta + (a + h - d) \cdot \frac{d}{a} \to a\tan^2\theta - d\tan\theta + (a + h) = 0$$

解得：

$$\tan\theta = \frac{d \pm \sqrt{d^2 - 4a(a + h)}}{2a}$$

取飞行时间较短的解（仰角较小，低弹道），子弹飞行时间为：

$$t_{fly} = \frac{d}{v_0 \cos\theta}$$

---

### 三、迭代弹道修正

由于预测目标位置时需要知道 $t_{fly}$，而计算 $t_{fly}$ 又需要目标位置，采用**定点迭代**求解（最多 10 次，收敛条件 $|\Delta t_{fly}| < 1\ \text{ms}$）：

1. 以初始瞄准点计算 $t_{fly}^{(0)}$
2. 用 $t_{fly}^{(k)}$ 预测未来时刻目标位置，重新选择击打点，计算新的 $t_{fly}^{(k+1)}$
3. 若 $|t_{fly}^{(k+1)} - t_{fly}^{(k)}| < 0.001\ \text{s}$，则判定收敛，退出迭代

---

### 四、小陀螺击打点选择（`choose_aim_point`）

小陀螺模式下装甲板周期性出现和消失。以旋转中心为参考，装甲板相对于旋转中心连线方向的偏角记为 $\delta$，定义：

- **coming_angle**（进入角）：装甲板从"不可见"进入"可打击区域"的角度阈值
- **leaving_angle**（离开角）：装甲板即将离开"可打击区域"时的保护角度阈值

选取规则：满足以下两个条件时，选择该装甲板作为击打目标：

$$|\delta| \leq \text{coming\_angle}$$


进一步地，`coming_angle` 和 `leaving_angle` 根据角速度 $|\omega|$ 进行线性衰减，角速度越高则窗口越窄：

$$\theta(|\omega|) = \begin{cases} \theta_{low} & |\omega| < \omega_1 \\ \theta_{high} + \dfrac{|\omega| - \omega_1}{\omega_2 - \omega_1}(\theta_{end} - \theta_{high}) & \omega_1 \leq |\omega| < \omega_2 \\ \theta_{end} & |\omega| \geq \omega_2 \end{cases}$$

前哨站（角速度恒为 $0.8\pi$ rad/s）使用固定参数，不参与动态调整。

---

### 五、装甲板跳变检测与 pitch 修正

当装甲板 ID 发生切换时，计算前后装甲板高度差 $\Delta z$：
- $|\Delta z| \geq \text{jump\_z\_threshold}$：记录跳变方向（up/down）和时刻
- 使用指数移动平均（EMA，系数 $\alpha$）平滑高度观测，减少噪声干扰：

$$z_{avg,k} = \alpha \cdot z_k + (1 - \alpha) \cdot z_{avg,k-1}$$

跳变发生后，在时间窗口 $[0,\ T_{up/down}]$ 内给 pitch 输出叠加修正量：

$$\text{pitch}_{send} = \text{pitch}_{trajectory} + \Delta\text{pitch}_{jump}$$

其中向下跳变（高→低）叠加正修正 `jump_pitch_up`，向上跳变（低→高）叠加负修正 `jump_pitch_down`。

---

### 六、跳变延迟开火（Jump Fire Cooldown）

装甲板跳变后，EKF 状态尚未收敛，此时不应立即开火。延迟时间根据角速度线性插值：

$$t_{cooldown}(|\omega|) = \begin{cases} t_{min} & |\omega| \leq \omega_{start} \\ t_{min} + \dfrac{|\omega| - \omega_{start}}{\omega_{end} - \omega_{start}} (t_{max} - t_{min}) & \omega_{start} < |\omega| < \omega_{end} \\ t_{max} & |\omega| \geq \omega_{end} \end{cases}$$

前哨站使用固定冷却时间，不随角速度变化。

---


## 4 算法接口介绍及说明

### Tracker（状态估计器）

```cpp
// 输入：装甲板列表 + 时间戳
// 输出：目标列表（含 EKF 状态）
std::list<Target> Tracker::track(
    std::list<Armor> & armors,
    std::chrono::steady_clock::time_point t,
    bool use_enemy_color = true);
```

**关键 YAML 参数：**

| 参数 | 含义 | 示例值 |
|---|---|---|
| `min_detect_count` | 目标确认所需最少帧数 | — |
| `max_temp_lost_count` | 允许最多丢失帧数 | — |
| `jump_z_threshold` | 触发跳变的高度差阈值（m） | 0.03 |
| `jump_confirm_count` | 跳变确认帧数 | 1 |
| `jump_avg_alpha` | 高度滑动平均系数（EMA） | 0.8 |
| `jump_fire_cooldown_min` | 跳变后发弹冷却最小值（s） | 0.05 |
| `jump_fire_cooldown_max` | 跳变后发弹冷却最大值（s） | 0.20 |
| `jump_fire_cooldown_speed_start` | 冷却插值起始角速度（rad/s） | 3.0 |
| `jump_fire_cooldown_speed_end` | 冷却插值终止角速度（rad/s） | 10.0 |
| `outpost_jump_fire_cooldown` | 前哨站固定冷却时间（s） | 0.08 |
| `jump_min_interval` | 两次跳变最小间隔（s） | 0.1 |

---

### Aimer（瞄准决策器）

```cpp
// 输入：目标列表 + 时间戳 + 弹速
// 输出：云台控制指令（yaw, pitch, control, shoot）
io::Command Aimer::aim(
    std::list<Target> targets,
    std::chrono::steady_clock::time_point timestamp,
    double bullet_speed,
    bool to_now = true);
```

**关键 YAML 参数：**

| 参数 | 含义 |
|---|---|
| `yaw_offset` / `pitch_offset` | 安装角偏差补偿（degree） |
| `comming_angle` | 低速进入角（degree） |
| `leaving_angle` | 低速保护离开角（degree） |
| `comming_angle_high` | 高速衰减区间起始进入角（degree） |
| `comming_end_angle` | 最高速时进入角最小值（degree） |
| `leaving_angle_high` / `leaving_end_angle` | 对应 leaving 衰减参数（degree） |
| `speed_angle` | 线性衰减起始角速度（rad/s） |
| `speed_angle_max` | 线性衰减终止角速度（rad/s） |
| `outpost_comming_angle` / `outpost_leaving_angle` | 前哨站专用角度（degree） |
| `decision_speed` | 高低速判断阈值（rad/s） |
| `high_speed_delay_time` / `low_speed_delay_time` | 预测时间偏移量（s） |
| `jump_pitch_up` / `jump_pitch_down` | 跳变 pitch 修正量（degree） |
| `jump_pitch_up_duration` / `jump_pitch_down_duration` | 修正有效时间窗口（s） |

---

### Target（单目标状态）

```cpp
// 获取 EKF 状态向量（11维）
// [x, vx, y, vy, z, vz, alpha, omega, r, l, h]
Eigen::VectorXd Target::ekf_x() const;

// 获取所有装甲板的世界坐标（xyza，a 为角度）
std::vector<Eigen::Vector4d> Target::armor_xyza_list() const;

// 跳变状态查询
bool Target::has_jump_time() const;
int  Target::last_jump_dir() const;   // -1=向下跳（高→低）  +1=向上跳（低→高）
bool Target::in_jump_fire_cooldown(std::chrono::steady_clock::time_point t) const;
```

---

## 5 算法实际运行结果

### 轨迹跟随效果

使用 PlotJuggler 记录云台 yaw 实际值与指令值的对比：
- 跟随单块装甲板时，稳态误差 < 0.01 rad
- 装甲板切换（小陀螺跳变）时，具备提前减速能力，有效减少超调和滞后现象

### 量化指标（25 赛季国赛）

| 指标 | 数值 |
|---|---|
| 最高命中率 | 39.6%（赛场 MVP 场次） |
| 正常命中率 | ≥ 30% |
| 击杀时间（300HP，2m，7 rad/s） | ~8 s |
| 击杀时间（300HP，2m，14 rad/s） | ~10 s |

### 调试工具与监控数据

通过 NoMachine 远程桌面 + PlotJuggler 实时监控以下数据：

| 数据字段 | 含义 |
|---|---|
| `w` | 目标角速度（用于区分高低速模式） |
| `residual_yaw` / `residual_pitch` | EKF 观测残差（判断滤波器健康状态） |
| `nis` | 归一化新息平方（卡方检验指标） |
| `shoot` | 开火信号 |
| `cmd_yaw` vs `gimbal_yaw` | 云台跟随误差 |

### Debug 可视化（`standard_mpc_se`）

图像叠加显示内容：
- **蓝色框**：识别到的装甲板
- **红色框**：当前击打目标预击打位置（`aim_center`）
- **"Up" 标签**（橙色）：处于 pitch 向上跳变修正窗口内时显示，用于调试 `jump_pitch_up_duration`
- **"Down" 标签**（红色）：处于 pitch 向下跳变修正窗口内时显示，用于调试 `jump_pitch_down_duration`


# 本次更新： 主要优化点说明 


**1. 线性衰减角度阈值**

高转速下装甲板停留时间短，需要更保守的进入角度；通过 `speed_angle` 和 `speed_angle_max` 定义线性衰减区间，避免硬切换导致的跳变抖动。相比固定阈值方案，在高转速场景（> 7 rad/s）下减少了因窗口过宽导致的错误击打装甲板问题。

**2. 动态跳变冷却**

低速时跳变后状态收敛快，冷却时间可短（约 50ms）；高速时收敛慢，需要更长冷却（约 200ms）。线性插值使参数具有物理直觉，便于根据实际效果调整端点值。

**3. 迭代弹道修正**

静态弹道解算在距离较远（> 4m）、飞行时间较长时误差显著。迭代修正将连续两次飞行时间差压缩至 1ms 以内，保证了中远距离打击精度。实测 10 次以内必然收敛。

**4. 装甲板跳变 pitch 修正**

传统方案在装甲板高度发生跳变时，由于 EKF 的延迟性，云台 pitch 会跟随估计值短暂偏离真实高度。通过检测跳变方向并在短时间窗口内叠加经验补偿量，可以减少跳变瞬间的脱靶率。

**5. 前哨站新模型：运行时高度自动标定**

前哨站三块装甲板在结构上高度不等（约 ±0.1m），但 EKF 状态向量中只有一个整车中心高度 $z$，若三块板统一使用同一高度，会导致 pitch 解算误差。

传统方案直接使用固定偏移或忽略该差异。本项目在前哨站目标初始化后，**前 2.5s 内对每块装甲板的观测高度分别采样**（前哨站约 2.5s 转一圈，可覆盖全部三块板）。采样结束后对三组均值排序，自动赋予高度偏移量：

$$\text{height\_offsets} = [-0.1\text{m},\ 0.0\text{m},\ +0.1\text{m}]$$（按高度从低到高分配给对应 ID）

标定完成后，装甲板坐标计算时叠加各自的偏移：

$$z_{id} = z_{center} + \text{height\_offsets}[id]$$

跳变检测也直接使用已标定偏移差（而非 EKF 估计的浮动高度），避免了估计误差引入的误判：

$$\Delta z = \text{height\_offsets}[\text{new\_id}] - \text{height\_offsets}[\text{last\_id}]$$

此外，前哨站收敛后对角速度进行钳位（$\omega$ 强制为 $\pm 2.51\ \text{rad/s}$），防止 EKF 噪声导致角速度漂移，保证预测精度。

6. 深圳大学RobotPilots视觉RP24识别模型嵌入，但未实际测试过，实际效果有待验证。