cmake_minimum_required(VERSION 3.16)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")

find_package(OpenVINO REQUIRED)
find_package(Ceres REQUIRED)

set(AUTO_BUFF_SOURCES
    buff_type.cpp
    buff_detector.cpp
    buff_solver.cpp
    buff_aimer.cpp
    buff_target.cpp
    yolo11_buff.cpp
)

add_library(auto_buff_obj OBJECT ${AUTO_BUFF_SOURCES})
target_link_libraries(auto_buff_obj auto_aim openvino::runtime ${CERES_LIBRARIES})

add_library(auto_buff STATIC $<TARGET_OBJECTS:auto_buff_obj>)
target_link_libraries(auto_buff auto_aim openvino::runtime ${CERES_LIBRARIES})