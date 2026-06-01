#include <fmt/core.h>

#include <chrono>
#include <cmath>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/simulator/cboard.hpp"
#include "yaml-cpp/yaml.h"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
    "{help h usage ? |      | 输出命令行参数说明}"
    "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }"
    "{simulator s    |      | 启用 simulator 模式 (从共享内存读取 IMU/模式/弹速) }";

int main(int argc, char* argv[]) {
    cv::CommandLineParser cli(argc, argv, keys);
    auto config_path = cli.get<std::string>(0);
    bool use_simulator = cli.has("simulator");
    if (cli.has("help") || config_path.empty()) {
        cli.printMessage();
        return 0;
    }

    tools::Exiter exiter;
    tools::Plotter plotter;
    tools::Recorder recorder;

    // --- CBoard: 两种实现，统一接口 via lambdas ---
    io::CBoard serial_cboard(config_path);           // real hardware
    io::SimulatorCBoard sim_cboard;                  // simulator

    // Pointers / refs used in the main loop:
    std::function<Eigen::Quaterniond()> get_imu;
    std::function<void(io::Command)>   send_cmd;
    std::function<int()>               get_mode;
    std::function<double()>            get_bullet_speed;

    if (use_simulator) {
        sim_cboard.open();
        tools::logger()->info("Using SimulatorCBoard (shared memory)");
        get_imu = [&]() -> Eigen::Quaterniond {
            Eigen::Quaterniond q = sim_cboard.imu_at();
            return q;
        };
        get_mode = [&]() -> int { return sim_cboard.mode; };
        send_cmd = [&](io::Command cmd) { sim_cboard.send(cmd); };
        get_bullet_speed = [&]() -> double { return sim_cboard.bullet_speed; };
    } else {
        tools::logger()->info("Using CBoard (serial)");
        get_imu = [&]() -> Eigen::Quaterniond {
            return serial_cboard.imu_at(std::chrono::steady_clock::now() - 1ms);
        };
        get_mode = [&]() -> int { return static_cast<int>(serial_cboard.mode); };
        send_cmd = [&](io::Command cmd) { serial_cboard.send(cmd); };
        get_bullet_speed = [&]() -> double { return serial_cboard.bullet_speed; };
    }

    io::Camera camera(config_path);

    auto_aim::YOLO detector(config_path, false);
    auto_aim::Solver solver(config_path);
    auto_aim::Tracker tracker(config_path, solver);
    auto_aim::Planner planner(config_path);
    auto_aim::Aimer aimer(config_path);
    auto_aim::Shooter shooter(config_path);

    auto_buff::Buff_Detector buff_detector(config_path);
    auto_buff::Solver buff_solver(config_path);
    auto_buff::SmallTarget buff_small_target;
    auto_buff::BigTarget buff_big_target;
    auto_buff::Aimer buff_aimer(config_path);

    cv::Mat img;
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point t;
    double bullet_speed = 25.0;

    int mode       = 0;          // 0=idle, 1=auto_aim, 2=small_buff, 3=big_buff, 4=outpost
    int last_mode  = -1;
    int frame_count = 0;
    io::Command last_command = {false, false, 0.0, 0.0, 0.0, 0.0};
    int total_armors = 0;
    int detected_frames = 0;

    while (!exiter.exit()) {
        camera.read(img, t);
        q            = get_imu();
        mode         = get_mode();
        bullet_speed = get_bullet_speed();

        if (last_mode != mode) {
            tools::logger()->info("Switch to mode {}", mode);
            last_mode = mode;
        }

        recorder.record(img, q, t);

        solver.set_R_gimbal2world(q);

        if (mode == io::Mode::small_buff || mode == io::Mode::big_buff) {
            buff_solver.set_R_gimbal2world(q);
            auto power_runes = buff_detector.detect(img);
            buff_solver.solve(power_runes);

            io::Command buff_command{false, false, 0.0, 0.0, 0.0, 0.0};
            if (mode == io::Mode::small_buff) {
                buff_small_target.get_target(power_runes, t);
                auto target_copy = buff_small_target;
                buff_command = buff_aimer.aim(target_copy, t, bullet_speed, true);
            } else {
                buff_big_target.get_target(power_runes, t);
                auto target_copy = buff_big_target;
                buff_command = buff_aimer.aim(target_copy, t, bullet_speed, true);
            }
            send_cmd(buff_command);
            frame_count++;
            continue;
        }

        if (mode != io::Mode::auto_aim && mode != io::Mode::outpost) {
            send_cmd({false, false, 0.0, 0.0, 0.0, 0.0});
            frame_count++;
            continue;
        }

        auto yolo_start    = std::chrono::steady_clock::now();
        auto armors        = detector.detect(img);
        total_armors += armors.size();  // 累加检测到的装甲板
        if (!armors.empty()) {
            detected_frames++;  // 累加检测成功的帧数
        }
        auto tracker_start = std::chrono::steady_clock::now();
        auto targets       = tracker.track(armors, t);
        auto aimer_start   = std::chrono::steady_clock::now();
        auto command       = aimer.aim(targets, t, bullet_speed);
        
        if (!targets.empty()) {
            auto plan = planner.plan(targets.front(), bullet_speed);
            if (plan.control) {
                // 使用 MPC (Planner) 的结果（位置+速度）覆盖 Aimer 的结果
                command.yaw       = plan.yaw;
                command.pitch     = plan.pitch;
                command.yaw_vel   = plan.yaw_vel;
                command.pitch_vel = plan.pitch_vel;
            }
        }

        auto finish        = std::chrono::steady_clock::now();

        if (!targets.empty() && aimer.debug_aim_point.valid
            && std::abs(command.yaw - last_command.yaw) * 57.3 < 2)
        {
            command.shoot = true;
        }

        if (command.control) {
            last_command = command;
        }

        tools::logger()->info(
            "[{}] yolo: {:.1f}ms, tracker: {:.1f}ms, aimer: {:.1f}ms",
            frame_count,
            tools::delta_time(tracker_start, yolo_start) * 1e3,
            tools::delta_time(aimer_start, tracker_start) * 1e3,
            tools::delta_time(finish, aimer_start) * 1e3
        );

        Eigen::Quaterniond gimbal_q = q;
        Eigen::Vector3d ypr         = tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0);
        auto yaw                    = ypr[0];

        tools::draw_text(
            img,
            fmt::format(
                "command is {},{:.2f},{:.2f},shoot:{}",
                command.control,
                command.yaw * 57.3,
                command.pitch * 57.3,
                command.shoot
            ),
            { 10, 60 },
            { 154, 50, 205 }
        );
        tools::draw_text(
            img,
            fmt::format("gimbal yaw{:.2f}", yaw * 57.3),
            { 10, 90 },
            { 255, 255, 255 }
        );

        nlohmann::json data;
        data["armor_num"] = armors.size();
        if (!armors.empty()) {
            const auto& armor      = armors.front();
            data["armor_x"]        = armor.xyz_in_world[0];
            data["armor_y"]        = armor.xyz_in_world[1];
            data["armor_yaw"]      = armor.ypr_in_world[0] * 57.3;
            data["armor_yaw_raw"]  = armor.yaw_raw * 57.3;
            data["armor_center_x"] = armor.center_norm.x;
            data["armor_center_y"] = armor.center_norm.y;
        }

        data["gimbal_yaw"] = yaw * 57.3;
        data["cmd_yaw"]    = command.yaw * 57.3;
        data["shoot"]      = command.shoot;

        if (!targets.empty()) {
            auto target                                  = targets.front();
            std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
            for (const Eigen::Vector4d& xyza: armor_xyza_list) {
                auto image_points =
                    solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
                tools::draw_points(img, image_points, { 0, 255, 0 });
            }
            Eigen::VectorXd x = target.ekf_x();
            std::vector<cv::Point3f> center_pt = {{static_cast<float>(x[0]), static_cast<float>(x[2]), static_cast<float>(x[4])}};
            auto center_img_pts = solver.world2pixel(center_pt);
            if (!center_img_pts.empty()) {
                cv::circle(img, center_img_pts[0], 5, {255, 255, 0}, -1); // Cyan circle
            }
            auto aim_point           = aimer.debug_aim_point;
            Eigen::Vector4d aim_xyza = aim_point.xyza;
            auto image_points =
                solver
                    .reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
            if (aim_point.valid) {
                tools::draw_points(img, image_points, { 0, 0, 255 });
            }

            data["x"]         = x[0];
            data["vx"]        = x[1];
            data["y"]         = x[2];
            data["vy"]        = x[3];
            data["z"]         = x[4];
            data["vz"]        = x[5];
            data["a"]         = x[6] * 57.3;
            data["w"]         = x[7];
            data["r"]         = x[8];
            data["l"]         = x[9];
            data["h"]         = x[10];
            data["last_id"]   = target.last_id;

            auto ekf = target.ekf();

            data["residual_yaw"]        = ekf.data.at("residual_yaw");
            data["residual_pitch"]      = ekf.data.at("residual_pitch");
            data["residual_distance"]   = ekf.data.at("residual_distance");
            data["residual_angle"]      = ekf.data.at("residual_angle");
            data["nis"]                 = ekf.data.at("nis");
            data["nees"]                = ekf.data.at("nees");
            data["nis_fail"]            = ekf.data.at("nis_fail");
            data["nees_fail"]           = ekf.data.at("nees_fail");
            data["recent_nis_failures"] = ekf.data.at("recent_nis_failures");
        }

        plotter.plot(data);

        // cv::resize(img, img, {}, 0.8, 0.8);
        // cv::imshow("reprojection", img);
        // auto key = cv::waitKey(1);
        // if (key == 'q')
        //    break;

        send_cmd(command);
        frame_count++;
    }
        // 在程序结束时输出识别率
    if (frame_count > 0) {
        double avg_armors_per_frame = static_cast<double>(total_armors) / frame_count;
        double detection_rate = 100.0 * detected_frames / frame_count;
        tools::logger()->info(
            "Recognition Summary: Total frames: {}, Total armors detected: {}, "
            "Average armors per frame: {:.2f}, Detection rate: {:.2f}% ({}/{} frames)",
            frame_count, total_armors, avg_armors_per_frame, detection_rate, detected_frames, frame_count
        );
    }

    return 0;
}
