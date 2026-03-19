#!/bin/bash
# 本脚本用于对 standard_mpc_se 进行压力测试，反复清理和编译，以检测系统在高负载下的稳定性。
# 配置
ITERATIONS=50
PROJECT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BUILD_DIR="${PROJECT_DIR}/build"
LOG_FILE="${PROJECT_DIR}/stress_test_build.log"

# 初始化环境
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

echo "开始压力测试: 将清理并重新编译 standard_mpc_se $ITERATIONS 次。" | tee $LOG_FILE
echo "此脚本用于检测小电脑在高负载下是否会出现死机、重启或编译错误。" | tee -a $LOG_FILE
echo "开始时间: $(date)" | tee -a $LOG_FILE

# 确保 build 目录存在
mkdir -p "$BUILD_DIR"

for i in $(seq 1 $ITERATIONS); do
    echo "===================================" | tee -a $LOG_FILE
    echo "第 $i / $ITERATIONS 次编译循环" | tee -a $LOG_FILE
    echo "时间戳: $(date)" | tee -a $LOG_FILE
    
    cd "$BUILD_DIR" || { echo "无法进入 build 目录"; exit 1; }
    
    # 清理旧文件以强制重新编译 (产生CPU和IO负载)
    echo "正在清理 (make clean)..."
    make clean > /dev/null 2>&1
    
    # 重新生成 Makefile
    echo "正在配置 (cmake)..."
    cmake .. > /dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "❌ 第 $i 次循环 CMake 配置失败！" | tee -a $LOG_FILE
        exit 1
    fi
    
    # 执行多线程编译 (降低为 -j2 以避免内存峰值导致的编译器崩溃)
    echo "正在编译 (make -j2)..."
    make standard_mpc_se -j2 > /dev/null
    
    if [ $? -eq 0 ]; then
        echo "✅ 第 $i 次循环 编译成功。" | tee -a $LOG_FILE
    else
        echo "❌ 第 $i 次循环 编译失败！" | tee -a $LOG_FILE
        echo "可能原因：系统过热、内存溢出(OOM)或硬件不稳定。" | tee -a $LOG_FILE
        exit 1
    fi
    
    # 简单的温度检查 (如果安装了 sensors)
    if command -v sensors &> /dev/null; then
        TEMP=$(sensors | grep -E "Package id 0|Core 0" | head -n 1 | awk '{print $4}')
        echo "当前CPU温度: $TEMP" | tee -a $LOG_FILE
    fi
    
    # 稍微暂停，让日志可读，但保持负载
    sleep 1
done

echo "===================================" | tee -a $LOG_FILE
echo "🎉 压力测试全部完成！系统似乎经受住了考验。" | tee -a $LOG_FILE
echo "日志已保存至: $LOG_FILE"
