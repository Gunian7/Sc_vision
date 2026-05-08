cmake_minimum_required(VERSION 3.16)

set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_subdirectory(planner/tinympc)

add_library(auto_aim OBJECT 
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

target_link_libraries(auto_aim io openvino::runtime tinympcstatic)

# Hero 专用：坐标链封装。core 仅 YAML/Eigen，无 Solver；apply 依赖 auto_aim（供 hero 链接）
add_library(hero_solver_core STATIC hero_solver.cpp)
target_link_libraries(hero_solver_core PUBLIC yaml-cpp)

add_library(hero_solver STATIC hero_solver_apply.cpp)
target_link_libraries(hero_solver PUBLIC hero_solver_core)