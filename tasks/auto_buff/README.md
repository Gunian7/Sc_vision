# auto_buff 功能文档

`tasks/auto_buff` 实现能量机关打符链路：从相机图像中识别扇叶关键点，估算能量机关在世界坐标系中的位姿，跟踪小符/大符运动状态，并输出云台控制与发射指令。

主链路调用顺序通常为：

1. `Buff_Detector::detect()` / `detect_24()`：图像推理与扇叶整理，得到 `PowerRune` 观测。
2. `Solver::solve()`：使用 PnP 与外参/IMU 姿态，将观测补齐为世界坐标系下的中心、扇叶中心和姿态。
3. `SmallTarget::get_target()` 或 `BigTarget::get_target()`：用 EKF 维护目标状态并处理丢失、角度跳变和旋转方向。
4. `Aimer::aim()` 或 `Aimer::mpc_aim()`：预测未来击打点，加入弹道补偿，输出 `io::Command` 或 `auto_aim::Plan`。

## 文件职责

| 文件 | 作用 |
| --- | --- |
| `buff_type.hpp/.cpp` | 定义扇叶、能量机关观测结构和目标扇叶排序逻辑。 |
| `yolo11_buff.hpp/.cpp` | 封装 OpenVINO YOLO11 关键点模型推理、后处理和调试绘制。 |
| `buff_detector.hpp/.cpp` | 将模型输出转换为 `FanBlade` / `PowerRune`，估算 R 标中心并维护检测丢失状态；大符提供多候选入口。 |
| `buff_solver.hpp/.cpp` | 根据相机内参、外参和云台姿态做 PnP 位姿解算与坐标变换。 |
| `buff_target.hpp/.cpp` | 定义小符/大符目标跟踪器，用扩展卡尔曼滤波预测旋转状态。 |
| `buff_aimer.hpp/.cpp` | 根据目标预测、弹道模型和配置偏置生成瞄准角与开火逻辑。 |
| `buff_predict.hpp` | 旧版/备用角度预测器与 XYZ 滤波器，当前未加入 `CMakeLists.txt` 的 `auto_buff` 编译源。 |
| `CMakeLists.txt` | 构建 `auto_buff_obj` 和 `auto_buff` 静态库，链接 `auto_aim`、OpenVINO 和 Ceres。 |

## 数据结构逻辑

### `FanBlade`

`FanBlade` 表示单片扇叶观测：

- `center`：扇叶中心。当前由 YOLO 输出的 `kpt[4]` 传入。
- `points`：保存模型输出关键点。实际使用中包含 6 个关键点：前 4 个用于 PnP，`points[4]` 是扇叶中心，`points[5]` 用于估算 R 标中心。
- `angle`：以 R 标中心为原点，当前扇叶相对目标扇叶的角度。
- `type`：`_target`、`_light`、`_unlight`，表示待击打扇叶、已亮扇叶和未亮扇叶。

### `PowerRune`

`PowerRune` 表示一帧能量机关观测和解算结果：

- `r_center`：图像中的 R 标中心。
- `fanblades`：固定整理为 5 个扇叶槽位，从目标扇叶开始按顺时针角度排列，缺失槽位填 `_unlight`。
- `light_num`：本帧识别到的亮扇叶数量。
- `xyz_in_world` / `ypd_in_world`：R 标中心在世界坐标系下的直角坐标和球坐标。
- `blade_xyz_in_world` / `blade_ypd_in_world`：目标扇叶中心在世界坐标系下的位置。
- `ypr_in_world`：buff 坐标系相对世界坐标系的 yaw/pitch/roll。

构造函数会结合上一帧 `last_powerrune` 判断目标扇叶：

- 大符类型下，若同时识别到两片合法亮扇叶，直接选择离屏幕中心最近的一片作为目标。
- 只有一个扇叶时，该扇叶直接作为目标。
- 扇叶数量不变时，选取与上一帧目标中心最近的扇叶作为目标。
- 扇叶数量增加 1 时，选取与上一帧所有已亮扇叶距离最远的新增扇叶作为目标。
- 其他数量变化视为无法求解，设置 `unsolvable_`。

之后它会以目标扇叶为角度零点，对扇叶排序，并按 5 等分角度填充缺失扇叶。

## 图像检测逻辑

### `YOLO11_BUFF`

构造时从 YAML 的 `model` 字段读取 OpenVINO 模型路径，编译为 CPU 模型，并建立推理请求。

`get_onecandidatebox()` 用于当前主流程：

1. 用 `fill_tensor_data_image()` 做 letterbox 缩放、BGR 到 RGB、归一化和 HWC 到 CHW 数据填充。
2. 执行 `infer_request.infer()`。
3. 在输出列中寻找置信度最高的候选框。
4. 若置信度高于 `ConfidenceThreshold = 0.7`，解析矩形框和 6 个关键点。
5. 在图像上绘制框、关键点编号和 FPS，并返回单个 `Object`。

`get_multicandidateboxes()` 用于多扇叶候选：

1. 用 `fill_tensor_data_image()` 做与单候选一致的 letterbox、通道转换、归一化和数据填充。
2. 解析所有高于阈值的候选框和关键点。
3. 使用 `cv::dnn::NMSBoxes()` 按 `IouThreshold = 0.4` 去重。
4. 返回多个 `Object`，同时绘制调试框和 FPS。

### `Buff_Detector`

`Buff_Detector` 持有 `YOLO11_BUFF MODE_`、跟踪状态 `status_`、丢失计数 `lose_` 和上一帧 `last_powerrune_`。

主要接口：

- `detect()`：调用 `get_onecandidatebox()`，只保留最高置信度扇叶，适合当前主程序默认流程。
- `detect_big()`：调用 `get_multicandidateboxes()`，保留多个候选扇叶，并使用大符目标选择策略；当两片亮起时优先锁定离屏幕中心最近的一片。
- `detect_24()`：调用 `get_multicandidateboxes()`，保留多个候选扇叶。
- `detect_debug()`：先多候选检测，再按传入向量 `v` 从候选中筛选调试用扇叶。

R 标中心估算在 `get_r_center()` 中完成：

1. 对每个扇叶取 `points[4]` 和 `points[5]`，用 `(point6 - point5) * 1.4 + point5` 粗估 R 标方向，并对所有扇叶求平均。
2. `handle_img()` 将原图灰度化、阈值二值化，再用 5x5 矩形核膨胀。
3. 以粗估中心为圆心生成 mask，只保留附近区域。
4. 查找轮廓，用最小外接旋转矩形的长宽比和到粗估中心的距离共同评分，选取得分最小的轮廓中心作为 `r_center`。

丢失处理：

- 检测为空或 `PowerRune` 无法求解时调用 `handle_lose()`。
- `lose_` 达到 `LOSE_MAX = 20` 后清空上一帧观测。
- 当前实现每次丢失都会把 `status_` 置为 `TEM_LOSE`。

## 位姿解算逻辑

`Solver` 从配置读取：

- `camera_matrix`、`distort_coeffs`：相机内参和畸变。
- `R_camera2gimbal`、`t_camera2gimbal`：相机到云台坐标系外参。
- `R_gimbal2imubody`：云台与 IMU body 的安装关系。

`set_R_gimbal2world()` 接收 IMU 四元数，计算 `R_gimbal2world_`，供每帧 PnP 结果转换到世界坐标系。

`solve()` 的核心步骤：

1. 若输入 `PowerRune` 为空则直接返回。
2. 取目标扇叶前 4 个图像关键点，与 `OBJECT_POINTS` 前 4 个 buff 坐标系 3D 点匹配。
3. 使用 `cv::solvePnP(..., cv::SOLVEPNP_IPPE)` 求 `R_buff2camera` 和 `t_buff2camera`。
4. 将 R 标中心 `(0,0,0)` 和目标扇叶中心 `(0,0,0.7)` 从 buff 坐标系变换到 camera、gimbal、world 坐标系。
5. 写回 `PowerRune` 的 `xyz_in_world`、`ypd_in_world`、`blade_xyz_in_world`、`blade_ypd_in_world` 和 `ypr_in_world`。

调试接口：

- `point_buff2pixel()`：把 buff 坐标系点用最近一次 PnP 结果投影到像素。
- `reproject_buff()`：给定世界坐标、yaw 和 roll，反算到相机坐标并重投影所有 `OBJECT_POINTS`。

## 目标跟踪逻辑

### 通用 `Target`

`Target` 是抽象基类，维护 EKF 公共变量和状态标志：

- `first_in_`：是否需要重新初始化。
- `unsolvable_`：当前目标是否可用于瞄准。
- `lasttime_`：上一帧滤波时间。
- `voter`：根据 roll 角变化投票判断旋转方向，正数视为顺时针，负数视为逆时针。

`point_buff2world()` 根据 EKF 状态把 buff 坐标系内的点变换到世界坐标系。状态中的 R 标中心以球坐标保存，buff 姿态使用 yaw 和 roll，pitch 固定为 0。

### `SmallTarget`

小符使用近似恒定角速度模型，角速度固定为 `SMALL_W = pi / 3`，方向由 `Voter` 决定。

状态向量共 7 维：

```text
[R_yaw, R_v_yaw, R_pitch, R_dis, yaw, roll, spd]
```

处理流程：

1. `get_target()` 收到空观测时标记不可解并累计丢失次数。
2. 首帧或重置后调用 `init()`，用 `PowerRune` 的 R 标球坐标、buff yaw/roll 和固定角速度初始化 EKF。
3. 丢失超过 6 帧时重置 `first_in_`。
4. `update()` 先处理 roll 的 2pi/5 扇叶跳变，再根据 roll 变化投票旋转方向。
5. 调用 `predict(nowtime - lasttime_)` 做状态预测。
6. 用两组观测更新 EKF：
   - R 标中心球坐标和 buff roll。
   - 目标扇叶中心球坐标，使用 `h_jacobian()` 构造非线性观测雅可比。
7. 若估计角速度偏离 `SMALL_W` 超过约 10 度/秒，认为发散并重置。

### `BigTarget`

大符使用正弦变速模型，角速度形式为：

```text
spd = a * sin(w * t + fi) + 2.09 - a
```

状态向量共 10 维：

```text
[R_yaw, R_v_yaw, R_pitch, R_dis, yaw, roll, spd, a, w, fi]
```

处理流程与小符类似，但预测模型不同：

1. `init()` 用经验初值初始化 `spd=1.1775`、`a=0.9125`、`w=1.942`、`fi=0`。
2. `predict()` 根据正弦速度积分更新 roll，并更新瞬时速度。
3. `update()` 同样分两次用 R 标观测和扇叶中心观测更新 EKF。
4. 将 EKF 估计速度加入 `RansacSineFitter`，用拟合出的正弦速度 `fit_spd_` 辅助后续预测。
5. 若 `a` 或 `w` 超出经验范围，认为滤波发散并触发重新初始化。

## 瞄准与开火逻辑

`Aimer` 从配置读取：

- `yaw_offset`、`pitch_offset`：角度偏置，配置中以 degree 表示，内部转 rad。
- `fire_gap_time`：连续开火最小间隔。
- `predict_time`：额外预测时间。

### `get_send_angle()`

这是普通瞄准和 MPC 瞄准共用的核心角度计算：

1. 先按 `predict_time` 将目标预测到未来。
2. 取 buff 坐标系下 `(0,0,0.7)` 的目标扇叶中心，转换为世界坐标。
3. 用 `tools::Trajectory` 根据弹速、水平距离和高度计算弹道俯仰角和飞行时间。
4. 再按第一轮弹道飞行时间继续预测目标。
5. 重新计算弹道，若两次飞行时间误差超过 0.01s，则认为解算不可靠。
6. 输出 `yaw = atan2(y, x) + yaw_offset`，`pitch = trajectory.pitch + pitch_offset`。

### `aim()`

输出 `io::Command`：

- 目标不可解时返回空控制命令。
- 弹速小于 10m/s 时回退为 24m/s。
- 预测时间为图像时间戳到当前时间的延迟加 `predict_time_`。
- 若本次 yaw/pitch 与上次差异超过 5 度，认为可能切换了扇叶，短期内不发送控制或不开火。
- 连续多次大跳变后允许重新控制，避免长期卡死。
- 未切换扇叶且超过 `fire_gap_time_` 才置 `shoot = true`。

### `mpc_aim()`

输出 `auto_aim::Plan`，角度逻辑与 `aim()` 相同，但在 `plan.control` 为真时额外估算 yaw/pitch 的速度和加速度，供云台 MPC 接口使用：

- 第一次进入控制时速度、加速度置 0。
- 后续用当前预测角和负时间预测角计算角速度。
- 用当前云台状态 `gs` 与预测角计算角加速度。
- 开火字段为 `plan.fire`。

## 配置依赖

当前 `auto_buff` 直接使用的配置项包括：

- 模型：`model`
- 相机内参：`camera_matrix`、`distort_coeffs`
- 坐标外参：`R_gimbal2imubody`、`R_camera2gimbal`、`t_camera2gimbal`
- 瞄准偏置：`yaw_offset`、`pitch_offset`
- 开火与预测：`fire_gap_time`、`predict_time`
- 弹速：上层传入 `cboard.bullet_speed` 或 `GimbalState::bullet_speed`，低于阈值时内部回退。

## 当前主程序接入方式

在 `src/standard.cpp` 中，进入 `small_buff` 或 `big_buff` 模式后：

1. 更新 `buff_solver` 的云台到世界坐标旋转。
2. 小符调用 `buff_detector.detect(img)` 获取单扇叶观测，大符调用 `buff_detector.detect_big(img)` 获取多候选观测。
3. 调用 `buff_solver.solve(power_runes)` 写入世界坐标信息。
4. 小符模式使用 `SmallTarget`，大符模式使用 `BigTarget`。
5. 为避免瞄准预测修改长期跟踪器状态，上层会复制一份 target，再传入 `buff_aimer.aim()`。
6. 将输出命令发送到下位机。

`src/auto_buff_debug.cpp` 和 `src/auto_buff_debug_mpc.cpp` 分别用于普通命令和 MPC 计划的打符调试。

## 注意事项

- `detect()` 当前只使用最高置信度候选，因此 `PowerRune` 多扇叶补全能力主要在 `detect_24()` 路径中体现。
- `Solver::solve()` 当前实际 PnP 只使用目标扇叶前 4 个关键点，`r_center` 会参与后续观测与调试，但没有参与 PnP 求解。
- `buff_predict.hpp` 中的预测器未被当前 `auto_buff` 库编译，维护主链路时应优先查看 `buff_target.hpp/.cpp`。
- `Aimer` 会在 `get_send_angle()` 内多次调用 `target.predict()`，所以上层传入的是 target 副本，避免瞄准阶段推进主跟踪器状态。
