cmake_minimum_required(VERSION 3.16)
project(io)
find_package(yaml-cpp REQUIRED)
find_package(serial_driver REQUIRED)

add_subdirectory(serial)

# 创建目标 io
add_library(io STATIC 
    board_types.cpp
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

################## Hero 专用：ROS 图像静态库 io_image_from_ros（不经由 io::Camera）##################
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(sensor_msgs QUIET)
find_package(cv_bridge QUIET)

if(ament_cmake_FOUND AND rclcpp_FOUND AND sensor_msgs_FOUND AND cv_bridge_FOUND)
    message(STATUS "io: io_image_from_ros target available (Hero)")
    add_library(io_image_from_ros STATIC image_from_ros/image_from_ros.cpp)
    target_include_directories(io_image_from_ros PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
    target_link_libraries(io_image_from_ros tools fmt::fmt spdlog::spdlog)
    ament_target_dependencies(io_image_from_ros rclcpp sensor_msgs cv_bridge)
else()
    message(STATUS "io: io_image_from_ros skipped (need rclcpp, sensor_msgs, cv_bridge)")
endif()

################## Hero：Sc_vision 专用 ROS 板卡适配（依赖 HERO_MSGS_PACKAGE + rclcpp；勿依赖 hero_common）##################
# HERO_MSGS_PACKAGE 在顶层 Sc_vision/CMakeLists.txt 中定义；只改该处一行即可切换消息包。
find_package(${HERO_MSGS_PACKAGE} QUIET)
if(ament_cmake_FOUND AND rclcpp_FOUND AND ${HERO_MSGS_PACKAGE}_FOUND)
    message(STATUS "io: io_hero_ros_board target available (HeroRosBoard, standalone Sc_vision)")
    add_library(io_hero_ros_board STATIC
        hero_ros_board/hero_ros_board.cpp
        hero_ros_board/hero_ros_command_pub.cpp)
    target_include_directories(io_hero_ros_board PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
    ament_target_dependencies(io_hero_ros_board rclcpp ${HERO_MSGS_PACKAGE})
    target_link_libraries(io_hero_ros_board Eigen3::Eigen yaml-cpp tools fmt::fmt spdlog::spdlog)
else()
    message(STATUS "io: io_hero_ros_board skipped (need rclcpp + ${HERO_MSGS_PACKAGE})")
endif()

################## ROS：publish2nav / subscribe2nav（额外依赖 sp_msgs）##################
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(sp_msgs QUIET)

if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND rosidl_typesupport_cpp_FOUND AND sp_msgs_FOUND)
    message(STATUS "io: ros2 publish2nav / subscribe2nav enabled")
    target_sources(io PRIVATE ros2/publish2nav.cpp ros2/ros2.cpp ros2/subscribe2nav.cpp)
    include_directories(${std_msgs_INCLUDE_DIRS})
    ament_target_dependencies(io rclcpp std_msgs sp_msgs)
else()
    message(STATUS "io: ros2 publish2nav skipped (sp_msgs or deps missing)")
endif()
