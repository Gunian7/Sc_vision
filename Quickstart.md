# 快速开始 (Quickstart)

## 1. 安装依赖
在终端中运行setup.bash脚本安装依赖：
（注意ROS和OpenVINO版本，确保与系统兼容,如果下载太慢或者不方便，直接在本机下载然后通过scp传过去）
```bash
bash setup.bash
```

## 2. 编译 (Build)

```bash
cmake -B build
cmake --build build -j$(nproc)
```

如果只需编译某个特定目标（例如自瞄主程序），可以加 `--target`：

```bash
cmake --build build --target standard_mpc_se -j$(nproc)
```
io部分可能需要单独colcon build编译，不然找不到对应的serial：(理论上除了哨兵分支，需要在io/ros单独source，别的部分都没有ROS的依赖)
（如果还有报错请移步询问ai，可能需要安装串口库，届时请直接安装对应发行版依赖，例如 jazzy：
`sudo apt install ros-jazzy-serial ros-jazzy-ros2-serial-driver`）
```bash
cd io
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install
```
3. 授予串口执行权限
```bash
sudo usermod -a -G dialout $USER
```
获取端口 ID（serial, idVendor, idProduct）
```bash
udevadm info -a -n /dev/ttyACM0 | grep -E '({serial}|{idVendor}|{idProduct})'
```
将 /dev/ttyACM0 替换为实际设备名。
## 3. 运行 (Run)

### 主程序

**当前自瞄程序为 `standard_mpc_se`**，使用串口 CBoard 通信，支持多线程推理和完整火控逻辑,需要注意的是.yaml文件众多，但是不一定都是对的，需要自己辨别，一般来说，standard.yaml是正常的可以直接使用：

```bash
./build/standard_mpc_se configs/standard.yaml
```
 提醒：
 1. 模型目前yolov5实测效果最好，推理速度一般在10ms左右，精度较好，建议低曝光，高增益，需要注意，曝光可能需要根据环境照度值去调整，过高过低都会导致识别效果变差。识别是最重要的一环！！！

---


### 单元测试

#### 相机测试
验证相机是否能正常出图：
```bash
./build/camera_test config-path=configs/standard.yaml --display
```
> 注意：`camera_test` **只有在传入 `--display`（或 `-d`）时才会调用 `imshow` 弹窗**。  
> 不加该参数时程序只会在终端打印 FPS，看起来像“没有窗口”。

#### 通信测试
验证与 C 板通讯（打印欧拉角 + 弹速 + mode）：
```bash
./build/cboard_test 
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
./build/camera_detect_test configs/standard.yaml

> 可加 `--tradition=true` 切换为传统识别方法。
```
#### 离线视频识别测试（`detector_video_test`）
读取本地 `.avi` 视频文件，逐帧运行识别，**无需相机**，适合调参和验证模型：
```bash
./build/detector_video_test --config-path=configs/standard.yaml <视频路径>
```
> 可通过 `--start-index` / `--end-index` 指定视频起止帧。

#### USB 摄像头识别测试（`usbcamera_detect_test`）
使用 USB 摄像头实时采图并运行 YOLO 识别，需使用含 `image_width` / `usb_exposure` 等 USB 相机字段的 yaml（如 `uav.yaml`）：
```bash
./build/usbcamera_detect_test configs/standard.yaml --name=video0 --display
```
> `--name` 指定设备名，默认 `video0`。

---

### 识别 + 追踪 + 瞄准测试（无需下位机）

#### 相机追踪测试（`camera_track_test`）
使用 MindVision 相机实时采图，运行完整的 YOLO → Tracker → Aimer 链路，**不需要 CBoard 或串口**。
IMU 姿态固定为单位四元数（等效云台水平静止），弹速从 yaml 中读取 `bullet_speed` 字段。
```bash
./build/camera_track_test configs/standard.yaml
```
- 窗口显示追踪重投影（黄色框）
- 终端打印追踪状态、瞄准角度、是否触发射击
- PlotJuggler 可实时接收 `target_x/y/z/w`、`cmd_yaw/pitch`、`shoot` 等数据
>支持命令行传参覆盖 如 `--speed=15.0` 覆盖 yaml 中的弹速。

---

### 打符（Buff）测试

#### 在线实时打符（独立程序）

适合单独调试打符链路（`Detector -> Solver -> Target -> Aimer`）：

```bash
./build/auto_buff_debug configs/standard.yaml
```

若需要 MPC 版本调试：

```bash
./build/standard_mpc_se configs/standard.yaml
```

> 根据下位机传的mode决定对应流程

```bash
./build/mt_standard configs/standard.yaml
```
#### 云台响应测试（无需相机/下位机）：
发送步兵云台信号，观察响应曲线，修改mode即可改变相应轨迹：
```
./build/gimbal_response_test configs/standard.yaml --signal-mode=step --axis yaw
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

本项目使用 `watchdog.sh` 作为守护进程，由 systemd 服务在开机时拉起。当没有设备的时候不断轮询直到找到设备，然后进入主程序

### 步骤 1：确保脚本有执行权限（注意路径要修改为实际路径）

```bash
chmod +x /home/setsuna/RM/AutoAim/Sc_vision/watchdog.sh
（改到你对应的程序）
```

### 步骤 2：确认 watchdog.sh 配置正确

`watchdog.sh` 中关键配置项：

```bash
BIN_PATH="./build/standard_mpc_se"       # 运行的可执行文件
CONFIG_PATH="configs/standard.yaml"      # 配置文件路径
```

ROS 2 环境变量（如需要）建议在脚本开头按本机发行版自动 source（当前机器环境为humble）：
```bash
if [ -f /opt/ros/humble/setup.zsh ]; then
    source /opt/ros/humble/setup.zsh
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi
```
> 如使用其他 ROS 版本，修改路径即可。

### 步骤 3：创建 systemd 服务文件

在系统中创建一个 systemd unit，用于在开机时启动 `watchdog.sh` 并由 systemd 管理其重启与日志。

1. 使用编辑器创建服务文件：

```bash
sudo nano /etc/systemd/system/Sc_vision.service
```

2. 将下面内容粘贴进去并保存（根据你的路径修改 `User`、`WorkingDirectory`、`ExecStart`）：

```ini
[Unit]
Description=Sc_vision Auto Aim Watchdog
After=network.target

[Service]
Type=simple
User=setsuna
WorkingDirectory=/home/setsuna/RM/AutoAim/Sc_vision
ExecStart=/bin/bash /home/setsuna/RM/AutoAim/Sc_vision/watchdog.sh
Restart=always
RestartSec=5
Environment=HOME=/home/setsuna
# 可选：从 /etc/default/Sc_vision.env 加载环境变量（如 LD_LIBRARY_PATH）
EnvironmentFile=-/etc/default/Sc_vision.env

[Install]
WantedBy=multi-user.target
```

说明：
- `Type=simple` 假定 `watchdog.sh` 在前台运行并不 fork；如果脚本会后台 fork，请改为 `Type=forking` 并使用 `PIDFile=`。
- `EnvironmentFile` 前的 `-` 表示文件不存在时忽略（方便可选配置）。
- 注意将

3. 重载 systemd 配置并启动服务：

```bash
# 重载 systemd 配置
sudo systemctl daemon-reload
# 启动并设置开机自启
sudo systemctl enable --now Sc_vision.service
```

4. 查看服务状态与日志：

```bash
# 查看服务状态
sudo systemctl status Sc_vision.service
# 实时查看日志输出
sudo journalctl -u Sc_vision.service -f
```

5. 停止或重启服务：

```bash
# 停止服务
sudo systemctl stop Sc_vision.service
# 重启服务
sudo systemctl restart Sc_vision.service
```

如果你需要直接杀掉进程作为最后手段：

```bash
pkill -f watchdog.sh
pkill -f standard_mpc_se
```

---

