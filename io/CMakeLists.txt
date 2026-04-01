cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)
find_package(serial_driver REQUIRED)

add_subdirectory(serial)

# 创建目标 io
add_library(io STATIC 
    hikrobot/hikrobot.cpp    
    mindvision/mindvision.cpp  
    usbcamera/usbcamera.cpp  
    camera.cpp
    cboard.cpp
    dm_imu/dm_imu.cpp
    gimbal/gimbal.cpp
    serial_phoenix/src/serial.cpp
)

# hikrobot
target_include_directories(io PUBLIC hikrobot/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC hikrobot/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC hikrobot/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

# mindvision
target_include_directories(io PUBLIC mindvision/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC mindvision/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC mindvision/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(io MvCameraControl MVSDK usb-1.0 yaml-cpp serial fmt::fmt spdlog::spdlog tools)

# serial_driver 是 ament 包，需要用 ament_target_dependencies 设置 include 和链接
ament_target_dependencies(io serial_driver)

################## 检查 ROS 环境 ##################
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(sp_msgs QUIET)
find_package(communicate_2025 QUIET)

# 尝试添加 ROS2 相关代码。如果完整 ROS 环境（包括 sp_msgs 等）可用，按常规处理；
# 否则当用户启用了 BUILD_STANDARD_MPC_SE 或者本地生成的 communicate_2025 可用时，
# 仍然尝试把 ros2 源加入到 io 目标，并且只把已找到的 ROS 依赖加入 ament_target_dependencies，
# 以减少因为缺少某些包而完全跳过 ros2 源编译的情况。
set(HAVE_ROS_FULL FALSE)
if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND rosidl_typesupport_cpp_FOUND AND sp_msgs_FOUND)
  set(HAVE_ROS_FULL TRUE)
endif()

# 检查本地 communicate_2025（由 io/ros2 colcon 构建并安装到 io/ros2/install）是否存在
set(COMMUNICATE_LOCAL_DIR "${CMAKE_SOURCE_DIR}/io/ros2/install/communicate_2025")
if(EXISTS "${COMMUNICATE_LOCAL_DIR}/include/communicate_2025")
  set(HAVE_COMMUNICATE_LOCAL TRUE)
else()
  set(HAVE_COMMUNICATE_LOCAL FALSE)
endif()

if(HAVE_ROS_FULL)
  message(STATUS "ROS2 environment found, compiling ROS2-related code.")
  target_sources(io PRIVATE ros2/publish2nav.cpp ros2/ros2.cpp ros2/subscribe2nav.cpp)
  include_directories(${std_msgs_INCLUDE_DIRS})
  ament_target_dependencies(io rclcpp std_msgs sp_msgs)
else()
  if(BUILD_STANDARD_MPC_SE OR HAVE_COMMUNICATE_LOCAL)
    message(STATUS "Partial ROS support: adding ros2 sources into io (BUILD_STANDARD_MPC_SE or local communicate_2025 detected).")
    # 如果本地 communicate 可用，加入其 include 路径以找到生成的消息头
    if(HAVE_COMMUNICATE_LOCAL)
      message(STATUS "Adding local communicate_2025 include: ${COMMUNICATE_LOCAL_DIR}/include/communicate_2025")
      include_directories("${COMMUNICATE_LOCAL_DIR}/include/communicate_2025")
    endif()
    # 把 ros2 源加入 io；如果系统中找到了 rclcpp/std_msgs 等，会在下面加入依赖
    target_sources(io PRIVATE ros2/publish2nav.cpp ros2/ros2.cpp ros2/subscribe2nav.cpp)

    # 有条件地把已找到的 ROS 包加入依赖，避免找不到某个包导致失败
    set(_ros_deps)
    if(rclcpp_FOUND)
      list(APPEND _ros_deps rclcpp)
    endif()
    if(std_msgs_FOUND)
      list(APPEND _ros_deps std_msgs)
    endif()
    if(sp_msgs_FOUND)
      list(APPEND _ros_deps sp_msgs)
    endif()
    if(communicate_2025_FOUND)
      list(APPEND _ros_deps communicate_2025)
    endif()
    if(_ros_deps)
      ament_target_dependencies(io ${_ros_deps})
    else()
      message(WARNING "ROS headers/libraries not found; ros2 sources added but may fail to compile without rclcpp/std_msgs installed.")
    endif()
  else()
    message(WARNING "ROS2 not found, skipping ROS2 specific code.")
  endif()
endif()
