#!/bin/bash
echo ">>> APT update" && sleep 1
sudo apt update
echo -e "\n\n>>> wget eigen3 ceres pip" && sleep 1
sudo apt install wget libeigen3-dev libceres-dev python3-pip -y
sudo apt install -y \
    git \
    g++ \
    cmake \
    can-utils \
    libopencv-dev \
    libfmt-dev \
    libeigen3-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    libusb-1.0-0-dev \
    nlohmann-json3-dev \
    openssh-server \
    screen
echo -e "\n\n>>> MindVision SDK" && sleep 1
if [ -d /usr/include/mindvision/ ]; then
    echo "mindvision-sdk already installed"
else
    echo ">>> start install mindvision-sdk"
    mkdir mindvision-sdk
    wget https://www.mindvision.com.cn/wp-content/uploads/2023/08/linuxSDK_V2.1.0.37.tar.gz -O mindvision-sdk/SDK.tar.gz
    tar -zxvf mindvision-sdk/SDK.tar.gz --directory=mindvision-sdk
    sed -i 's/usr\/include/usr\/include\/mindvision/g' mindvision-sdk/install.sh
    sed -i 17a\\"mkdir -p /usr/include/mindvision" mindvision-sdk/install.sh
    cd mindvision-sdk && sudo bash ./install.sh && cd ..
    echo ">>> successfully install mindvision-sdk"
    rm -r mindvision-sdk
fi
echo -e "\n\n>>> OpenVINO" && sleep 1
echo "Installing OpenVINO Runtime (offline package)..."
# create target folder
if [ ! -d /opt/intel ]; then
        sudo mkdir -p /opt/intel
fi

cd /tmp || exit 1
# Download OpenVINO runtime archive (Ubuntu 22.04 x86_64). If you need a different
# distro/arch, replace the URL accordingly.
OPENVINO_TGZ=openvino_2024.6.0.tgz
if [ ! -f "$OPENVINO_TGZ" ]; then
    echo "Downloading OpenVINO runtime..."
    curl -L \
        https://storage.openvinotoolkit.org/repositories/openvino/packages/2024.6/linux/l_openvino_toolkit_ubuntu22_2024.6.0.17404.4c0f47d2335_x86_64.tgz \
        --output "$OPENVINO_TGZ"
fi

if [ ! -f "$OPENVINO_TGZ" ]; then
    echo "Failed to download OpenVINO archive ($OPENVINO_TGZ)"
else
    echo "Extracting OpenVINO..."
    tar -xf "$OPENVINO_TGZ"
    # determine extracted dir
    DIRNAME=$(tar -tf "$OPENVINO_TGZ" | head -1 | cut -f1 -d"/")
    if [ -d "$DIRNAME" ]; then
        echo "Moving OpenVINO to /opt/intel/openvino_2024.6.0"
        sudo mv "$DIRNAME" /opt/intel/openvino_2024.6.0
        # Install OS-level dependencies provided by OpenVINO
        if [ -f /opt/intel/openvino_2024.6.0/install_dependencies/install_openvino_dependencies.sh ]; then
            echo "Installing OpenVINO system dependencies..."
            sudo -E /opt/intel/openvino_2024.6.0/install_dependencies/install_openvino_dependencies.sh
        else
            echo "OpenVINO dependency installer not found; please run it manually in /opt/intel/openvino_2024.6.0"
        fi
    else
        echo "Extraction failed or unexpected package layout; please inspect /tmp"
    fi
fi

cd - >/dev/null || true
echo -e "\n\n>>> ROS 2" && sleep 1
if ! command -v ros2 &> /dev/null
then
    echo "ROS is not installed, start install ROS2"
    wget http://fishros.com/install -O fishros && . fishros
else
    echo "ROS installed, skip install ROS2"
fi

echo -e "\n\n>>> rosdepc" && sleep 1
if command -v rosdep &> /dev/null 
then
    echo "rosdepc is not installed, start install rosdepc"
    wget http://fishros.com/install -O fishros && . fishros
else
    echo "rosdepc installed, skip install rosdepc"
fi
rosdepc update

sudo apt install ros-humble-serial-driver
sudo apt install ros-humble-asio-cmake-module

cd "$(dirname "$0")"
cd ..
git submodule init
git submodule update