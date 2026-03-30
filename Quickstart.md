# 快速开始 (Quickstart)

# 安装依赖
在终端中运行setup.bash脚本安装依赖：
（注意ROS版本）
```bash
bash setup.bash
```

## 编译 (Build)

```bash
cmake -B build
cmake --build build -j$(nproc)
```

如果只需编译某个特定目标（例如步兵主程序），可以加 `--target`：

```bash
cmake --build build --target standard_mpc_se -j$(nproc)
```
io部分需要单独colcon build编译，不然找不到对应的serial：（如果还有报错请移步询问ai）
```bash
cd io
colcon build --symlink-install
```

## 运行 (Run)

### 主程序

**当前标准步兵自瞄程序为 `standard_mpc_se`**，使用串口 CBoard 通信，支持多线程推理和完整火控逻辑,需要注意的是.yaml文件众多，但是不一定都是对的，需要自己辨别，一般来说，standard.yaml是正常的可以直接使用：

```bash
./build/standard_mpc_se configs/standard3.yaml
```

> 配置文件按机器人选择，`standard3.yaml` / `standard4.yaml` 等对应不同机器人。

#### 可选识别模型：RP24-DetectionModel

本仓库已支持将 `RobotDetectionModel`（RP24）作为可选识别模型接入。

1. 将导出的模型放到仓库 `assets/`（示例：`assets/rp24_0526.onnx`）。  
2. 在 yaml 中增加模型路径并切换模型名：

```yaml
yolo_name: rp24
rp24_model_path: assets/rp24_0526.onnx
```

3. 运行方式不变，例如：

```bash
./build/standard_mpc_se configs/standard4.yaml
```

> 说明：`rp24` 复用了当前 YOLOv5 的关键点解码流程，适用于输出格式为“8关键点 + 置信度 + 颜色分类 + 数字分类”的模型。

---

### 单元测试

#### 相机测试
验证相机是否能正常出图：
```bash
./build/camera_test --config-path=configs/sentry_blue.yaml --display
```

#### 通信测试
验证与 C 板通讯（打印欧拉角 + 弹速）：
```bash
./build/cboard_test configs/standard3.yaml
```

#### 自瞄离线录像测试
用录制好的视频和 IMU 数据测试自瞄效果（无需相机和 C 板）：
```bash
./build/auto_aim_test
```

---

### 识别模块测试

以下三个程序均只测试识别模块（Detector + YOLO），**不涉及追踪和火控**。

#### MindVision 工业相机识别测试（`camera_detect_test`）
使用 MindVision 工业相机实时采图并运行识别（包含 Detector 与 YOLO）：
```bash
./build/camera_detect_test configs/right.yaml
# 显示识别画面（需要有显示器）（哨兵）
./build/camera_detect_test configs/right.yaml --display
./build/camera_detect_test configs/standard3.yaml --display  
```
> 可加 `--tradition=true` 切换为传统识别方法。

#### 离线视频识别测试（`detector_video_test`）
读取本地 `.avi` 视频文件，逐帧运行识别，**无需相机**，适合调参和验证模型：
```bash
./build/detector_video_test --config-path=configs/sentry_blue.yaml <视频路径>
```
> 可通过 `--start-index` / `--end-index` 指定视频起止帧。

#### USB 摄像头识别测试（`usbcamera_detect_test`）
使用 USB 摄像头实时采图并运行 YOLO 识别，需使用含 `image_width` / `usb_exposure` 等 USB 相机字段的 yaml（如 `uav.yaml`）：
```bash
./build/usbcamera_detect_test configs/uav.yaml --name=video0 --display
```
> `--name` 指定设备名，默认 `video0`。**不可使用 MindVision 相机的 yaml（如 `sentry_blue.yaml`）**，否则会报 `image_width not found`。

---

### 识别 + 追踪 + 瞄准测试（无需下位机）

#### 相机追踪测试（`camera_track_test`）
使用 MindVision 相机实时采图，运行完整的 YOLO → Tracker → Aimer 链路，**不需要 CBoard 或串口**。
IMU 姿态固定为单位四元数（等效云台水平静止），弹速从 yaml 中读取 `bullet_speed` 字段。
```bash
./build/camera_track_test configs/sentry_blue.yaml
```
- 窗口显示追踪重投影（黄色框）
- 终端打印追踪状态、瞄准角度、是否触发射击
- PlotJuggler 可实时接收 `target_x/y/z/w`、`cmd_yaw/pitch`、`shoot` 等数据
> 可通过 `--speed=15.0` 覆盖 yaml 中的弹速。

---

### 打符（Buff）测试

#### 在线实时打符（独立程序）

适合单独调试打符链路（`Detector -> Solver -> Target -> Aimer`）：

```bash
./build/auto_buff_debug configs/standard3.yaml
```

若需要 MPC 版本调试：

```bash
./build/auto_buff_debug_mpc configs/standard3.yaml
```

> `auto_buff_debug` 源码当前默认是小符目标（`SmallTarget`）。如需固定调大符，可切换为 `BigTarget` 分支后重新编译。

#### 在线模式切换运行（推荐实车）

`mt_standard` 同时包含自瞄与打符流程，按下位机模式切换：
- `auto_aim`：自瞄
- `small_buff`：小符
- `big_buff`：大符

```bash
./build/mt_standard configs/standard3.yaml
```
#### 云台响应测试（无需相机/下位机），发送步兵云台信号，观察响应曲线，修改mode即可改变相应轨迹：
```
./build/gimbal_response_test configs/standard4.yaml --signal-mode=step --axis yaw
```

#### 离线录像打符测试（无需相机/下位机）

`auto_buff_test` 使用录像回放，输入为同名 `avi/txt`：

```bash
./build/auto_buff_test --config-path=configs/standard3.yaml records/buff_demo
```

上面命令会读取：
- `records/buff_demo.avi`
- `records/buff_demo.txt`

可选截取帧区间：

```bash
./build/auto_buff_test --config-path=configs/standard3.yaml --start-index=100 --end-index=1200 records/buff_demo
```

## 开机自启 (Autostart)

本项目使用 `watchdog.sh` 作为守护进程，由 systemd 服务在开机时拉起。

### 步骤 1：确保脚本有执行权限

```bash
chmod +x /home/setsuna/RM/AutoAim/Sc_vision/watchdog.sh
```

### 步骤 2：确认 watchdog.sh 配置正确

`watchdog.sh` 中关键配置项：

```bash
BIN_PATH="./build/standard_mpc_se"       # 运行的可执行文件
CONFIG_PATH="configs/standard3.yaml"      # 配置文件路径
```

ROS 2 环境变量（如需要）已在脚本开头自动 source：
```bash
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
```
> 如使用其他 ROS 版本，修改路径即可。

### 步骤 3：重载并启动 systemd 服务

修改过 `watchdog.sh` 或服务文件后，需要重载：

```bash
sudo systemctl daemon-reload
sudo systemctl restart sp_vision.service
```

验证运行状态：
```bash
sudo systemctl status sp_vision.service
```

### 步骤 4：查看实时日志

```bash
journalctl -u sp_vision.service -f
```

### 步骤 5：停止自启程序

```bash
sudo systemctl stop sp_vision.service
# 或直接杀进程
pkill -f watchdog.sh
pkill -f standard_mpc_se
```

---

## 修改代码后重新编译

```bash
cmake --build build --target standard_mpc_se -j$(nproc)
```

然后重启服务使其生效：
```bash
sudo systemctl restart sp_vision.service
```

## Impovements in progress
- 动态调整ROI大小以适应目标距离变化（较重要）
- 引入自适应增益以提升不同速度下的跟随性能
- 