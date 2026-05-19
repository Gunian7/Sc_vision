# Ember
## 0. 概述：

本项目为杭州电子科技大学Phoenix战队26赛季视觉自瞄模块，不依赖ROS，在参考多支队伍的开源后进行了重构和改进（缝合怪），同时添加了IMM对旋转状态进行单独建模，提升了变速小陀螺的适应性，测试中使用6mm镜头近距离(4m)内能保持较好的跟踪和击打性能，通过有限状态机进行火控，实际测试中基本2转每秒至少40%以上的命中率（保守估计）。

## 1. 前言
main分支主要在步兵上使用，如果需要对应兵种请根据分支去进行拉取。
需要注意的是由于自瞄在哨兵上与导航共用虚拟串口进行通信，因此其会有ROS依赖，需要进行一些适配，详见对应分支的README。

---
## 2. 快速上手 (Quickstart)
见 [Quickstart.md](Quickstart.md)
（没写完）

## 3. 代码结构
```
Ember
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

## Latest Updates
1. 针对旋转状态的运动建模
2. 根据转速的有限状态机击打选板逻辑
3. 新前哨站模型（带针对前哨站的开火窗口火控）
4. 打通打符链路

## TODO
1. 职责分层，planner拆分，让其只负责决策，然后aimer和shooter分别负责云台控制和开火控制，解耦决策和执行；
2. 开火判定重构，或许可以改成根据“几何+散布”阈值（根据实际情况决定）
   
## 实验性（未加入）
#### （ 1 ）角点像素重投影 ESEKF  
1. 初始化使用pnp解算器的结果，满足三个条件
- 连续 2~3 帧角点稳定
- PnP 解变化连续
- 重投影误差足够小
再正式初始化 target。
2.  加重投影误差筛选
单帧 PnP 后，把解再投影回去，检查四个角点误差：
- 平均 reprojection error 太大 → 丢弃
- 某个点误差异常大 → 丢弃
3. 用运动连续性选解
IPPE常见问题不是没有解，而是有多个接近解，
有上一帧预测状态，就可以按这些规则选：

- 哪个解离上一帧中心更近
- 哪个解 yaw 更连续
- 哪个解投影后的角点顺序更合理
- 哪个解对应的 z 深度更合理
- 
4. 远距离、小面积时不要太信初始化
当装甲板在图像里太小时：
- 角点亚像素误差会被放大
- 单帧 pose 抖动很严重
这时可能更适合：
- 提高初始化阈值
- 放宽“先观测几帧再建轨”的条件
- 或者只用粗初始化，不立刻强依赖 yaw/r
5. 初始化后先弱信任，再逐步收紧（理论上是这样但是实际实现还没有思路）
也就是：
初始化时给较大的 P0
让后续角点重投影更新慢慢把状态拉回来
这比“单帧 PnP 一下就把初始状态定死”更安全。

---
#### （ 2 ） pnp重投影误差约束的因子图优化（慎重）（参考吉大开源） 
#### （ 3 ） 仿真环境搭建（相机图源读取，其他不改变）
1. 不需要ROS依赖，无需发布/订阅话题，直接在本地读取视频或相机图像进行处理，实时显示结果，仿真正常运行，只是输出相机源作为识别模型的输入，其他模块不变；
2. 支持打符测试/大符和小符；对了，前哨站需要使用新的模型（目前还没有修改）

---
维护人：陈位恺

特别鸣谢及参考：
1. [武汉科技大学 崇实战队](https://github.com/WUST-RM/awakening)
2. [同济大学 sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25)
3. [深圳大学 RobotDetectionModel](https://github.com/broalantaps/RobotDetectionModel)
4. [君瞄 rm_vision](https://github.com/chenjunnn/rm_vision)
5. [吉林大学 JLU_VISION](https://github.com/Fskaaaaaaaa/jlu_vision_26)