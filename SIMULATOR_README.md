# 🎯 Simulator Vision Pipeline - 模拟器视觉流程

本文档说明如何将 **RoboMaster Simulator** (Rust/Bevy) 与 **Sc_vision** (C++/OpenVINO) 对接，使用独立的 `simulator` 可执行程序运行完整的视觉算法仿真。

## 📋 架构概览

```
┌─────────────────────────────────────────────────────────┐
│                   RoboMaster Simulator (Rust)            │
│                                                          │
│  Bevy 渲染引擎                                           │
│      │                                                   │
│      ▼                                                   │
│  ROS2 Capture Plugin  (ros2/capture.rs)                  │
│      │                                                   │
│      └──→ POSIX 共享内存  (/simulator_frame)             │
│            布局:                                          │
│            offset  0:  width      (u32)                  │
│            offset  4:  height     (u32)                  │
│            offset  8:  timestamp  (i64, ns)              │
│            offset 16: gimbal_yaw  (f32, rad)             │
│            offset 20: gimbal_pitch (f32, rad)            │
│            offset 24: gimbal_roll  (f32, rad)            │
│            offset 28: bullet_speed (f32, m/s)            │
│            offset 32: mode        (i32)                  │
│            offset 36: pixel data  (RGB8, w×h×3)          │
│                                                          │
└──────────────────────────┬──────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────┐
│                   Sc_vision (C++)                        │
│                                                          │
│  simulator (独立可执行程序，不依赖任何物理设备)           │
│      │                                                   │
│      ├── io::SimulatorCamera  ← 读取共享内存图像          │
│      │       cv::Mat (RGB8, w×h)                         │
│      │                                                   │
│      ├── io::SimulatorCBoard  ← 读取共享内存 gimbal 数据  │
│      │       ├── imu_at()    → Eigen::Quaterniond        │
│      │       ├── bullet_speed → float (m/s)              │
│      │       ├── mode        → io::Mode                  │
│      │       └── send()      → no-op (不下发)             │
│      │                                                   │
│      ├── YOLO detector (OpenVINO)                        │
│      ├── Tracker + Solver                                │
│      ├── Planner + Aimer                                 │
│      └── reprojection 可视化窗口                          │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

## 🚀 快速开始

### 前置要求

| 工具 | 版本 | 说明 |
|------|------|------|
| Rust | ≥1.75 | 运行模拟器 |
| C++17 编译器 | GCC ≥9 / Clang ≥10 | 编译 Sc_vision |
| OpenCV | ≥4.5 | 图像处理 |
| Eigen3 | ≥3.4 | 矩阵运算 |
| OpenVINO | 2024.6 | YOLO 推理后端 |

### 第一步：启动模拟器

```bash
# 进入模拟器目录
cd /home/setsuna/RM/AutoAim/Simulator

# 编译并运行模拟器（默认 1280×1024）
cargo run --release

# 如果需要自动截图模式（用于测试，完成后自动退出）
SC_SIM_AUTOCAPTURE=1 cargo run --release
```

模拟器启动后，你将看到：
- 一个 3D 场景，包含两辆步兵机器人
- 默认第一人称视角（跟随本地机器人云台）
- 左上角显示发射统计

### 第二步：编译 simulator 程序

**在另一个终端**中：

```bash
cd /home/setsuna/RM/AutoAim/Sc_vision

# 编译 simulator 可执行文件
colcon build --packages-select sc_vision
```

### 第三步：启动仿真视觉处理

```bash
# 运行仿真程序（从共享内存同时读取图像 + IMU/模式/弹速）
./install/sc_vision/lib/sc_vision/simulator configs/standard3.yaml
```

### 第四步：观察结果

- 程序自动打开名为 "reprojection" 的窗口，显示 YOLO 检测结果
  - **绿色边框**：重投影的装甲板
  - **红色边框**：Aimer 计算的瞄准点
  - **青色圆点**：Tracker EKF 估计的装甲板中心
- 终端每帧输出 YOLO/Tracker/Aimer 的耗时
- 可以使用 **F2** 在模拟器中截图

## 🎮 模拟器控制

### 本地机器人 (蓝方)

| 功能 | 按键 |
|------|------|
| 移动 | `W` `A` `S` `D` |
| 底盘旋转 | `Q` `E` |
| 云台俯仰/偏航 | `↑` `↓` `←` `→` |
| 发射弹丸 | `Space` |
| 视角切换 | `F3` (自由/第一人称/第三人称) |

### 敌方机器人 (红方)

| 功能 | 按键 |
|------|------|
| 移动 | `I` `J` `K` `L` |
| 底盘旋转 | `U` `O` |
| 云台旋转 | `F` `V` `C` `B` |

> **建议**：将敌方机器人移动到视野中，然后切换到第一人称视角（按 `F3`），观察检测效果。

## 🔧 数据流详解

### 共享内存布局

| 偏移 | 大小 | 字段 | 说明 |
|------|------|------|------|
| 0 | 4 | width | 图像宽度 (uint32_t) |
| 4 | 4 | height | 图像高度 (uint32_t) |
| 8 | 8 | timestamp_ns | Unix 纪元纳秒 (int64_t) |
| 16 | 4 | gimbal_yaw | 云台偏航 (float, rad) |
| 20 | 4 | gimbal_pitch | 云台俯仰 (float, rad) |
| 24 | 4 | gimbal_roll | 云台横滚 (float, rad) |
| 28 | 4 | bullet_speed | 弹速 (float, m/s) |
| 32 | 4 | mode | 工作模式 (int32_t) |
| 36+ | N | pixel_data | 像素数据 (RGB8, 逐行) |

### mode 枚举值

| 值 | 模式 | 说明 |
|----|------|------|
| 0 | idle | 空闲 |
| 1 | auto_aim | 自瞄 |
| 2 | small_buff | 小能量机关 |
| 3 | big_buff | 大能量机关 |
| 4 | outpost | 前哨站 |

### Simulator CBoard 数据流

`src/simulator.cpp` 中：

```cpp
// 1. 每秒 60 帧从共享内存读取 1280×1024 RGB 图像
camera->read(img, t);

// 2. 读取 gimbal yaw/pitch/roll → Eigen 四元数
q = cboard->imu_at(t - 1ms);

// 3. 同时更新 bullet_speed 和 mode
double speed = cboard->bullet_speed;  // 共用体从 shm 同步
io::Mode m   = cboard->mode;

// 4. 指令下发 → no-op（仿真模式不连接物理下位机）
cboard->send(command);  // 空操作
```

## 📊 性能

- **模拟器渲染 FPS**：≈60fps (取决于 GPU)
- **共享内存读取延迟**：<1μs (本地 mmap + 无锁读取)
- **Sc_vision 处理**：
  - YOLO 推理：~5-15ms (取决于模型和 OpenVINO 后端)
  - Tracker + Solver：<1ms
  - 总帧延迟：~10-30ms

## 🐛 常见问题

### Q: 找不到共享内存 /simulator_frame

**原因**：模拟器未启动或尚未创建共享内存。

**解决**：
```bash
# 检查共享内存是否存在
ls -la /dev/shm/simulator_frame

# 先启动模拟器
cd /home/setsuna/RM/AutoAim/Simulator
cargo run --release
# 等待显示 3D 场景后再启动 simulator
```

### Q: 共享内存数据异常

```bash
# 清理旧共享内存（需要 root 权限）
sudo rm -f /dev/shm/simulator_frame
sudo rm -f /dev/shm/sem.simulator_sem

# 重新启动模拟器
cargo run --release
```

### Q: 编译失败

确保 CMake 能找到所有依赖：
```bash
# 确认 OpenVINO 环境
source /opt/intel/openvino_2024.6.0/setupvars.sh

# 重新编译
colcon build --packages-select sc_vision
```

## 📁 相关文件清单

| 文件 | 角色 |
|------|------|
| **Simulator (Rust)** | |
| `src/util/shmem.rs` | 共享内存写入器 |
| `src/ros2/capture.rs` | 每帧写入图像 + gimbal 数据到 shm |
| `src/ros2/plugin.rs` | `update_shm_gimbal_state()` 更新 gimbal 状态 |
| **Sc_vision (C++)** | |
| `src/simulator.cpp` | **独立仿真主程序**（本文档的核心） |
| `io/simulator/simulator.hpp` | `ShmLayout` 结构定义 + `read_gimbal_*` 函数声明 |
| `io/simulator/simulator.cpp` | 共享内存读取函数实现 |
| `io/simulator/cboard.hpp` | `SimulatorCBoard : CBoard` 类声明 |
| `io/simulator/cboard.cpp` | `SimulatorCBoard` 实现：`imu_at()`, `send()` |

> **注意**：`simulator` 是完全独立的可执行程序，不修改也不依赖 `standard.cpp`、`standard_mpc_se.cpp` 等物理设备程序。两者可以共存，互不影响。
