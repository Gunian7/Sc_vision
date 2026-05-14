cmake_minimum_required(VERSION 3.16.3)

project(sp_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # 聚合体省略尾部 initializer 在工程中大量使用；-Wmissing-field-initializers 噪声远大于收益
  add_compile_options(-Wno-missing-field-initializers)
endif()
add_link_options(-Wl,--no-as-needed)
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

if(EXISTS "/usr/lib/x86_64-linux-gnu/cmake/fmt/fmt-config.cmake")
  set(fmt_DIR "/usr/lib/x86_64-linux-gnu/cmake/fmt" CACHE PATH "System fmt config" FORCE)
endif()

if(EXISTS "/usr/lib/x86_64-linux-gnu/cmake/spdlog/spdlogConfig.cmake")
  set(spdlog_DIR "/usr/lib/x86_64-linux-gnu/cmake/spdlog" CACHE PATH "System spdlog config" FORCE)
endif()

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED CONFIG)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED CONFIG)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(CURL REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
find_package(OpenVINO REQUIRED)
find_package(fastcdr REQUIRED)

# 自定义 ROS 消息包：与 io/hero_ros_board 中 #include "<包名>/msg/..." 一致；整栈默认 hero_interfaces。只改下一行，或 cmake -DHERO_MSGS_PACKAGE=...
set(HERO_MSGS_PACKAGE hero_interfaces CACHE STRING "ament 自定义消息包名")

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})
link_libraries(CURL::libcurl)


add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
add_subdirectory(tasks/auto_buff)
add_subdirectory(tasks/omniperception)

####################src###################
add_executable(standard src/standard.cpp)
add_executable(mt_standard src/mt_standard.cpp)
add_executable(standard_mpc src/standard_mpc.cpp)
add_executable(auto_aim_debug_mpc src/auto_aim_debug_mpc.cpp)
add_executable(mt_auto_aim_debug src/mt_auto_aim_debug.cpp)
add_executable(auto_buff_debug src/auto_buff_debug.cpp)
add_executable(auto_buff_debug_mpc src/auto_buff_debug_mpc.cpp)
add_executable(uav src/uav.cpp)
add_executable(uav_debug src/uav_debug.cpp)
add_executable(standard_mpc_se src/standard_mpc_se.cpp)

target_link_libraries(standard ${OpenCV_LIBS} yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(mt_standard ${OpenCV_LIBS} yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(standard_mpc ${OpenCV_LIBS} yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(auto_aim_debug_mpc ${OpenCV_LIBS} yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(mt_auto_aim_debug ${OpenCV_LIBS} yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(auto_buff_debug ${OpenCV_LIBS} yaml-cpp auto_buff tools io ${CERES_LIBRARIES})
target_link_libraries(auto_buff_debug_mpc ${OpenCV_LIBS} yaml-cpp auto_buff tools io ${CERES_LIBRARIES})
target_link_libraries(uav ${OpenCV_LIBS} yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(uav_debug ${OpenCV_LIBS} yaml-cpp auto_aim tools io)
target_link_libraries(standard_mpc_se ${OpenCV_LIBS} yaml-cpp auto_aim auto_buff tools io)

##################calibration################
add_executable(capture calibration/capture.cpp)
add_executable(calibrate_camera calibration/calibrate_camera.cpp)
add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
add_executable(calibrate_robotworld_handeye calibration/calibrate_robotworld_handeye.cpp)
add_executable(split_video calibration/split_video.cpp)

target_link_libraries(capture ${OpenCV_LIBS} tools io)
target_link_libraries(calibrate_camera ${OpenCV_LIBS} yaml-cpp tools)
target_link_libraries(calibrate_handeye ${OpenCV_LIBS} yaml-cpp tools)
target_link_libraries(calibrate_robotworld_handeye ${OpenCV_LIBS} yaml-cpp tools)
target_link_libraries(split_video ${OpenCV_LIBS} tools)

##################tests##################
add_executable(auto_aim_test tests/auto_aim_test.cpp)
add_executable(auto_buff_test tests/auto_buff_test.cpp)
target_sources(auto_buff_test PRIVATE $<TARGET_OBJECTS:auto_buff_obj>)
add_executable(camera_detect_test tests/camera_detect_test.cpp)
add_executable(camera_track_test tests/camera_track_test.cpp)
add_executable(camera_test tests/camera_test.cpp)
add_executable(camera_thread_test tests/camera_thread_test.cpp)
add_executable(cboard_test tests/cboard_test.cpp)
add_executable(fire_test tests/fire_test.cpp)
add_executable(detector_video_test tests/detector_video_test.cpp)
add_executable(gimbal_response_test tests/gimbal_response_test.cpp)
add_executable(multi_usbcamera_test tests/multi_usbcamera_test.cpp)
add_executable(usbcamera_detect_test tests/usbcamera_detect_test.cpp)
add_executable(usbcamera_test tests/usbcamera_test.cpp)
add_executable(handeye_test tests/handeye_test.cpp)
add_executable(dm_test tests/dm_test.cpp)
add_executable(minimum_vision_system tests/minimum_vision_system.cpp)

target_link_libraries(auto_aim_test ${OpenCV_LIBS} yaml-cpp tools io auto_aim)
target_link_libraries(auto_buff_test ${OpenCV_LIBS} yaml-cpp auto_aim openvino::runtime ${CERES_LIBRARIES} tools io)
target_link_libraries(camera_detect_test ${OpenCV_LIBS} yaml-cpp tools auto_aim io)
target_link_libraries(camera_track_test ${OpenCV_LIBS} yaml-cpp nlohmann_json::nlohmann_json tools auto_aim io)
target_link_libraries(camera_test ${OpenCV_LIBS} tools io)
target_link_libraries(camera_thread_test ${OpenCV_LIBS} auto_aim tools io)
target_link_libraries(cboard_test ${OpenCV_LIBS} tools io)
target_link_libraries(fire_test ${OpenCV_LIBS} tools io)
target_link_libraries(detector_video_test ${OpenCV_LIBS} yaml-cpp tools auto_aim)
target_link_libraries(gimbal_response_test ${OpenCV_LIBS} yaml-cpp tools io)
target_link_libraries(multi_usbcamera_test ${OpenCV_LIBS} tools io)
target_link_libraries(usbcamera_detect_test ${OpenCV_LIBS} yaml-cpp tools io auto_aim)
target_link_libraries(usbcamera_test ${OpenCV_LIBS} yaml-cpp tools io)
target_link_libraries(handeye_test ${OpenCV_LIBS} yaml-cpp tools io auto_aim)
target_link_libraries(dm_test ${OpenCV_LIBS} yaml-cpp tools io)
target_link_libraries(minimum_vision_system ${OpenCV_LIBS} yaml-cpp tools io auto_aim)

add_executable(tune_exposure tools/tune_exposure.cpp)
target_link_libraries(tune_exposure ${OpenCV_LIBS} yaml-cpp tools io auto_aim)

add_executable(gimbal_test tests/gimbal_test.cpp)
target_link_libraries(gimbal_test ${OpenCV_LIBS} yaml-cpp auto_aim tools io)

add_executable(planner_test tests/planner_test.cpp)
target_link_libraries(planner_test ${OpenCV_LIBS} yaml-cpp auto_aim tools io)

add_executable(planner_test_offline tests/planner_test_offline.cpp)
target_link_libraries(planner_test_offline ${OpenCV_LIBS} yaml-cpp auto_aim tools io)

# 检查 ROS 环境
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(sensor_msgs QUIET)
find_package(cv_bridge QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(sp_msgs QUIET)
find_package(${HERO_MSGS_PACKAGE} QUIET)

# Hero：ROS 图像 + HeroRosBoard；hero_tf_test / hero_ros_test 依赖自定义消息包 + rclcpp
if(TARGET io_hero_ros_board AND ament_cmake_FOUND AND rclcpp_FOUND AND ${HERO_MSGS_PACKAGE}_FOUND)
    add_executable(hero_tf_test src/hero_tf_test.cpp)
    target_link_libraries(hero_tf_test hero_solver_core io_hero_ros_board tools)
    ament_target_dependencies(hero_tf_test rclcpp ${HERO_MSGS_PACKAGE})

    add_executable(hero_ros_test src/hero_ros_test.cpp)
    target_link_libraries(hero_ros_test yaml-cpp)
    ament_target_dependencies(hero_ros_test rclcpp ${HERO_MSGS_PACKAGE})
endif()

# Hero：ROS 图像话题频率统计
if(ament_cmake_FOUND AND rclcpp_FOUND AND sensor_msgs_FOUND)
    add_executable(hero_camera_ros_test src/hero_camera_ros_test.cpp)
    target_link_libraries(hero_camera_ros_test ${OpenCV_LIBS} yaml-cpp tools)
    ament_target_dependencies(hero_camera_ros_test rclcpp sensor_msgs)
endif()

# Hero：ROS 图像 + HeroRosBoard 订阅自瞄反馈（自定义 Autoaim 等）
if(TARGET io_image_from_ros AND TARGET io_hero_ros_board AND ament_cmake_FOUND AND rclcpp_FOUND
   AND sensor_msgs_FOUND AND cv_bridge_FOUND AND ${HERO_MSGS_PACKAGE}_FOUND)
    add_executable(hero src/hero.cpp)
    target_link_libraries(
        hero
        ${OpenCV_LIBS}
        yaml-cpp
        tools
        io
        io_image_from_ros
        io_hero_ros_board
        hero_solver
        auto_aim
    )
    ament_target_dependencies(hero rclcpp sensor_msgs cv_bridge ${HERO_MSGS_PACKAGE})

    foreach(
        _hero_t
        hero_detectwithpnp_test
        hero_intrinsic_triangulation_test
        hero_observe_test
        hero_withoutboard_test
        hero_full_test)
        add_executable(${_hero_t} src/${_hero_t}.cpp)
        target_link_libraries(
            ${_hero_t}
            ${OpenCV_LIBS}
            yaml-cpp
            tools
            io
            io_image_from_ros
            io_hero_ros_board
            hero_solver
            auto_aim
        )
        ament_target_dependencies(${_hero_t} rclcpp sensor_msgs cv_bridge ${HERO_MSGS_PACKAGE})
    endforeach()
endif()

# 只有在找到 ROS 环境时才会添加 sentry 和 publish_test
if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND rosidl_typesupport_cpp_FOUND AND sp_msgs_FOUND)
    # 编译 sentry 和 publish_test
    add_executable(sentry src/sentry.cpp)
    add_executable(sentry_bp src/sentry_bp.cpp)
    add_executable(sentry_debug src/sentry_debug.cpp)
    add_executable(sentry_multithread src/sentry_multithread.cpp)
    add_executable(publish_test tests/publish_test.cpp)
    add_executable(subscribe_test tests/subscribe_test.cpp)
    add_executable(topic_loop_test tests/topic_loop_test.cpp)

    target_link_libraries(sentry ${OpenCV_LIBS} yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_bp ${OpenCV_LIBS} yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_debug ${OpenCV_LIBS} yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_multithread ${OpenCV_LIBS} yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(publish_test ${OpenCV_LIBS} tools io auto_aim)
    target_link_libraries(subscribe_test ${OpenCV_LIBS} tools io)
    target_link_libraries(topic_loop_test ${OpenCV_LIBS} tools io)
else()
    message(STATUS "ROS2 environment not found, skipping ROS2-related code.")
endif()
