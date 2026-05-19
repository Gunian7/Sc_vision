cmake_minimum_required(VERSION 3.16)

set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_subdirectory(planner/tinympc)

set(AUTO_AIM_SOURCES
    armor.cpp
    classifier.cpp
    detector.cpp
    solver.cpp
    imm.cpp
    aimer.cpp
    target.cpp
    tracker.cpp
    voter.cpp
    shooter.cpp
    yolo.cpp
    yolos/yolov5.cpp
    yolos/infer_request_pool.cpp
    yolos/yolov8.cpp
    yolos/yolo11.cpp
    multithread/commandgener.cpp
    multithread/mt_detector.cpp
    planner/planner.cpp
)

set(RKNN_VENDOR_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/yolos/rknn_threadpool/include")
set(RKNN_VENDOR_RGA_HINT "${RKNN_VENDOR_INCLUDE_DIR}/3rdparty/rga/RK3588/lib/Linux/aarch64")
set(HAS_RKNN FALSE)

if(DEFINED RKNN_INCLUDE_DIR AND EXISTS "${RKNN_INCLUDE_DIR}/rknn_api.h" AND DEFINED RKNN_LIBRARY AND EXISTS "${RKNN_LIBRARY}")
  set(HAS_RKNN TRUE)
  list(APPEND AUTO_AIM_SOURCES
    yolos/yolov5_rknn.cpp
    yolos/rknn_threadpool/src/preprocess.cc
    yolos/rknn_threadpool/src/postprocess.cc
    yolos/rknn_threadpool/src/rkYolov5s.cc
  )
endif()

add_library(auto_aim OBJECT ${AUTO_AIM_SOURCES})

if(HAS_RKNN)
  target_include_directories(auto_aim PRIVATE
    ${RKNN_VENDOR_INCLUDE_DIR}
    ${RKNN_VENDOR_INCLUDE_DIR}/3rdparty/rga/RK3588/include
  )

  find_library(RGA_LIBRARY rga
    PATHS /lib /usr/lib /lib/aarch64-linux-gnu /usr/lib/aarch64-linux-gnu
    HINTS ${RKNN_VENDOR_RGA_HINT}
  )
  if(NOT RGA_LIBRARY)
    message(FATAL_ERROR "RGA library not found (librga.so).")
  endif()

  target_link_libraries(auto_aim io openvino::runtime tinympcstatic ${RKNN_LIBRARY} ${RGA_LIBRARY})
else()
  target_link_libraries(auto_aim io openvino::runtime tinympcstatic)
endif()
