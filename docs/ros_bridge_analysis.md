# Sc_vision 图源与位姿源 ROS 接入分析

## 1. 图源入口与线程模型

### 1.1 相机抽象
- 抽象接口：`io::CameraBase`，定义于 `io/camera.hpp`，核心接口是 `read(cv::Mat&, steady_clock::time_point&)`。
- 门面类：`io::Camera`，定义于 `io/camera.cpp`，按配置中的 `camera_name` 选择后端并统一对外输出 `cv::Mat + 时间戳`。

### 1.2 现有后端
- MindVision：`io/mindvision/mindvision.cpp`
- Hikrobot：`io/hikrobot/hikrobot.cpp`
- USBCamera（OpenCV V4L）：`io/usbcamera/usbcamera.cpp`

### 1.3 线程模型
- 采集线程持续取图，并写入线程安全队列（`ThreadSafeQueue<CameraData>`）。
- 业务线程通过 `read()` 读取最新帧。
- 工业相机/USB 路径均带守护逻辑（掉线后重连/重启采集）。

## 2. 位姿源入口与时间对齐

### 2.1 位姿来源
- 主来源：`io::CBoard`（`io/cboard.cpp`），串口接收下位机姿态/弹速等信息。
- 可选来源：`io::DM_IMU`（`io/dm_imu/dm_imu.hpp`），主要用于部分测试程序。

### 2.2 时间对齐方式
- 业务常用模式为：相机读帧时刻 `t`，姿态取 `imu_at(t - 1ms)`。
- `CBoard::imu_at()` 使用队列内前后样本做四元数插值（slerp）。
- 现状是本地单调时钟对齐，不是 ROS Header 时间语义。

## 3. 现有非 ROS 端到端链路

1. 相机后端取图（MindVision/Hik/USB）  
2. `io::Camera::read()` 输出 `cv::Mat + t`  
3. `q = cboard.imu_at(t - 1ms)`  
4. `solver.set_R_gimbal2world(q)`  
5. 检测/跟踪/解算输出目标状态  
6. 生成 `io::Command`  
7. `CBoard::send()` 下发到电控

关键主流程入口：
- `src/standard.cpp`
- `src/mt_standard.cpp`
- `src/sentry.cpp`

## 4. ROS 接入最小接口建议

### 4.1 图像输入
- Topic：`sensor_msgs/msg/Image`
- 建议命名：`/image_for_auto_aim`、`/image_for_remote_shoot`
- 要求：`header.stamp` 与姿态时钟统一，否则多源融合会漂移。

### 4.2 位姿输入
- 推荐 Topic：`sensor_msgs/msg/Imu`（或 `geometry_msgs/msg/PoseStamped`）
- 必须明确 `frame_id`，并在 tf2 中固定 `base_link -> gimbal -> camera_optical`。

### 4.3 控制输出
- 建议新增自定义消息（或过渡期用现有字段拼装）：
  - `yaw`
  - `pitch`
  - `yaw_vel`
  - `pitch_vel`
  - `shoot`
  - `control_mode`

### 4.4 状态输出
- 弹速建议单独话题（`std_msgs/Float32`）或并入反馈消息。
- 模式状态建议单独话题（`std_msgs/UInt8`）。

## 5. 接入实施边界

### 一期（最小侵入）
- 保留 Sc_vision 核心算法，先做 ROS 桥接层：
  - 图像改为 ROS 订阅输入（替代直接相机读取）
  - 姿态改为 ROS 订阅输入（替代直接串口读取）
  - 控制命令改为 ROS 发布输出（替代直接串口发送）

### 二期（体系收敛）
- 将时间对齐统一到 ROS 时间轴与消息同步策略（message_filters / 时间戳统一来源）。
- 将多入口可执行（standard/mt/sentry）逐步收敛为可配置 ROS 节点组合。
