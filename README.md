# Phoenix Vision Module
## 0. 概述：
本项目为杭州电子科技大学Phoenix战队26赛季视觉自瞄模块，核心功能包括装甲板识别、坐标解算、状态估计、击打点选择和弹道解算，目标实现高命中率和短击杀时间，既满足中期和完整形态考核的性能要求而添加应试性参数接口，也满足赛场实战的鲁棒性和适应性需求。

## 1. 功能简介和工作流

###  功能简介

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
## 2. 快速上手 (Quickstart)
见 [Quickstart.md](Quickstart.md)

### 2.1 运行环境提示（ROS 2 + 相机测试）

若你在运行测试程序（如 `camera_test`）时遇到以下现象：

- 报错缺少 ROS 相关动态库（如 `libament_index_cpp.so`）
- 程序有 FPS 输出但没有 `imshow` 窗口

请注意：

1. `source /opt/ros/<distro>/setup.*` 只对**当前终端会话**生效，开新终端需要重新加载；
2. `camera_test` 只有在传入 `--display`（或 `-d`）时才会弹窗显示图像。

项目根目录已提供脚本 `run_camera_test.sh`（当前按 jazzy 环境配置）用于一键运行相机测试：
- 自动清理常见 ROS/colcon 污染变量
- 自动 source `/opt/ros/jazzy/setup.zsh`
- 如存在 `io/ros2/install` overlay 则自动叠加
- 默认附带 `--display`

使用方式：

```bash
chmod +x run_camera_test.sh
./run_camera_test.sh
```

## 3. 代码结构

```
sp_vision_25
├── assets         // 包含demo素材、网络权重等
│   └── ...
├── calibration    // 标定相关程序
│   ├── calibrate_camera.cpp             // 相机内参标定程序
│   ├── calibrate_handeye.cpp            // 手眼标定程序
│   ├── calibrate_robotworld_handeye.cpp // 手眼标定程序（同时计算标定板位置）
│   └── capture.cpp                      // 相机标定数据采集程序
├── CMakeLists.txt // CMake配置文件
├── configs        // 每台机器人的YAML配置文件
│   └── ...
├── io             // 硬件抽象层，见3.4软件架构
│   └── ...
├── src            // 应用层，见3.4软件架构
│   └── ...
├── tasks          // 功能层，见3.4软件架构
│   ├── auto_aim       // 自瞄相关算法实现
│   │   └── ...
│   ├── auto_buff      // 打符相关算法实现
│   │   └── ...
│   └── omniperception // 全向感知相关算法实现
│   │   └── ...
├── tests
│   ├── auto_aim_test.cpp         // 自瞄录制视频测试程序
│   ├── auto_buff_test.cpp        // 打符录制视频测试程序
│   ├── camera_detect_test.cpp    // 识别器测试程序（工业相机）
│   ├── camera_test.cpp           // 相机测试程序
│   ├── camera_thread_test.cpp    // 相机线程测试程序
│   ├── cboard_test.cpp           // C板测试程序
│   ├── detector_video_test.cpp   // 识别器测试程序（视频）
│   ├── dm_test.cpp               // 达妙IMU测试程序
│   ├── fire_test.cpp             // 开火测试程序
│   ├── gimbal_response_test.cpp  // 云台响应测试程序
│   ├── gimbal_test.cpp           // 云台通信测试程序
│   ├── handeye_test.cpp          // 手眼标定测试程序
│   ├── minimum_vision_system.cpp // 最小视觉系统测试程序
│   ├── multi_usbcamera_test.cpp  // 多USB摄像头测试程序
│   ├── planner_test_offline.cpp  // 规划器测试程序（离线）
│   ├── planner_test.cpp          // 规划器测试程序（实车）
│   ├── publish_test.cpp          // ROS发送测试程序
│   ├── subscribe_test.cpp        // ROS接收测试程序
│   ├── topic_loop_test.cpp       // ROS话题循环测试程序
│   ├── usbcamera_detect_test.cpp // 识别器测试程序（USB相机）
│   ├── usbcamera_test.cpp        // USB相机测试程序
│   └── ...
└── tools          // 工具层，见3.4软件架构
    ├── crc.hpp                    // CRC校验
    ├── exiter.hpp                 // 退出检测
    ├── extended_kalman_filter.hpp // 扩展卡尔曼滤波器
    ├── img_tools.hpp              // 图像处理工具
    ├── logger.hpp                 // 日志记录器
    ├── math_tools.hpp             // 数学工具
    ├── plotter.hpp                // 曲线图绘制工具
    ├── recorder.hpp               // 视频录制器
    ├── thread_safe_queue.hpp      // 线程安全队列
    ├── trajectory.hpp             // 弹道解算
    ├── yaml.hpp                   // YAML配置文件解析器
    └── ...
```

## TODO
1. 职责分层，planner拆分，让其只负责决策，然后aimer和shooter分别负责云台控制和开火控制，解耦决策和执行；
2. 开火判定重构，或许可以改成根据“几何+散布”阈值（根据实际情况决定）
3. pnp重投影误差约束的因子图优化（慎重）  
