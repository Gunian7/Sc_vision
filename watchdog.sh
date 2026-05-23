#!/bin/bash

# 确保脚本在错误时不会意外终止整个 shell，但这里我们需要循环，所以不用 set -e
# 动态获取当前脚本所在目录作为工作目录，适配不同环境
WORK_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Source ROS 2 environment (humble)  记得修改为你的 ROS 2 版本路径
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "没有找到所需的ROS环境，请检查路径"
fi
BIN_PATH="./build/standard_mpc_se"
CONFIG_PATH="configs/standard.yaml" # 显式指定配置文件路径

# 从配置文件中提取 C 板设备路径（如果 yaml 解析不可用则使用默认值）
CBOARD_DEVICE="/dev/ttyACM0"

cd "$WORK_DIR" || { echo "Cannot find work directory $WORK_DIR"; exit 1; }

# 确保日志目录存在
mkdir -p logs

echo "Starting watchdog for $BIN_PATH..."

while true; do
    # 检查可执行文件是否存在
    if [ ! -f "$BIN_PATH" ]; then
        echo "Error: Executable $BIN_PATH not found!"
        sleep 5
        continue
    fi

    # ---- 等待 C 板设备就绪 ----
    while [ ! -e "$CBOARD_DEVICE" ]; do
        echo "[Watchdog] Waiting for CBoard device $CBOARD_DEVICE ..."
        sleep 2
    done

    # 启动程序
    echo "------------------------------------------------"
    echo "Launching process at $(date) with config: $CONFIG_PATH"
    echo "------------------------------------------------"
    
    $BIN_PATH "$CONFIG_PATH"
    
    # 记录退出码
    EXIT_CODE=$?
    
    # 获取当前时间
    TIMESTAMP=$(date "+%Y-%m-%d %H:%M:%S")
    echo "[Watchdog] Process exited with code $EXIT_CODE at $TIMESTAMP"

    # 根据退出码分析原因
    if [ $EXIT_CODE -eq 0 ]; then
        echo "[Watchdog] Info: Process exited normally."
    elif [ $EXIT_CODE -eq 139 ]; then
        echo "[Watchdog] ERROR: Segmentation fault (Core Dumped). This might be due to memory access violation."
    elif [ $EXIT_CODE -eq 134 ]; then
        echo "[Watchdog] ERROR: Aborted (Core Dumped). This typically happens when an assertion fails or a library calls abort()."
        echo "[Watchdog] Check if GUI functions (imshow/waitKey) are called in a headless environment."
    elif [ $EXIT_CODE -eq 127 ]; then
        echo "[Watchdog] ERROR: Command not found or missing shared libraries."
    else
        echo "[Watchdog] Warning: Process exited with non-zero code. Possible crash or manual termination."
    fi

    echo "[Watchdog] Restarting camera/application sequence initiated..."

    # 防止疯狂重启（例如配置错误导致启动即崩）
    sleep 2
done
