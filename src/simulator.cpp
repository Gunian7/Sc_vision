#include <cstddef>
#include <array>
#include <algorithm>
#include <chrono>
#include <memory>
#include <cmath>
#include <vector>
#include <string>
#include <limits>

#include <fmt/core.h>
#include <fmt/format.h>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "io/simulator/simulator.hpp"
#include "io/simulator/cboard.hpp"
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
    "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char* argv[]) {
    cv::CommandLineParser cli(argc, argv, keys);
    auto config_path = cli.get<std::string>(0);
    if (cli.has("help") || config_path.empty()) {
        cli.printMessage();
        return 0;
    }

    double jump_pitch_up_duration = 0.0;
    double jump_pitch_down_duration = 0.0;
    double decision_speed = 0.0;
    double target_jump_angle_threshold_deg = 5.0;
    double target_stabilize_alpha = 0.3;
    bool enable_target_stabilize = true;
    try {
        auto yaml = YAML::LoadFile(config_path);
        if (yaml["jump_pitch_up_duration"].IsDefined()) {
            jump_pitch_up_duration = yaml["jump_pitch_up_duration"].as<double>();
        }
        if (yaml["jump_pitch_down_duration"].IsDefined()) {
            jump_pitch_down_duration = yaml["jump_pitch_down_duration"].as<double>();
        }
        if (yaml["decision_speed"].IsDefined()) {
            decision_speed = yaml["decision_speed"].as<double>();
        }
        if (yaml["target_jump_angle_threshold_deg"].IsDefined()) {
            target_jump_angle_threshold_deg = yaml["target_jump_angle_threshold_deg"].as<double>();
        }
        if (yaml["target_stabilize_alpha"].IsDefined()) {
            target_stabilize_alpha = yaml["target_stabilize_alpha"].as<double>();
        }
        if (yaml["enable_target_stabilize"].IsDefined()) {
            enable_target_stabilize = yaml["enable_target_stabilize"].as<bool>();
        }
    } catch (const YAML::Exception & e) {
        tools::logger()->warn("Failed to read configuration: {}", e.what());
    }

    tools::logger()->info("[simulator] Starting auto-aim pipeline with simulator input");

    tools::Exiter exiter;
    tools::Plotter plotter;
    tools::Recorder recorder;

    // 创建模拟器数据源（图像 + 下位机数据全部来自共享内存）
    auto camera = std::make_unique<io::SimulatorCamera>();
    auto cboard = std::make_unique<io::SimulatorCBoard>();

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

    auto mode       = io::Mode::idle;
    auto last_mode  = io::Mode::idle;
    int frame_count = 0;
    io::Command last_command = {false, false, 0.0, 0.0, 0.0, 0.0};
    int total_armors = 0;
    int detected_frames = 0;

    while (!exiter.exit()) {
        // 从共享内存读取图像 + IMU/模式/弹速
        camera->read(img, t);
        q    = cboard->imu_at(t - 1ms);
        mode = cboard->mode;

        if (last_mode != mode) {
            tools::logger()->info("Switch to {}", io::MODES[mode].c_str());
            last_mode = mode;
        }

        recorder.record(img, q, t);

        solver.set_R_gimbal2world(q);

        // 能量机关模式
        if (mode == io::Mode::small_buff || mode == io::Mode::big_buff) {
            buff_solver.set_R_gimbal2world(q);
            auto power_runes = buff_detector.detect(img);
            buff_solver.solve(power_runes);

            io::Command buff_command{false, false, 0.0, 0.0, 0.0, 0.0};
            const double active_bullet_speed = cboard->bullet_speed;
            if (mode == io::Mode::small_buff) {
                buff_small_target.get_target(power_runes, t);
                auto target_copy = buff_small_target;
                buff_command = buff_aimer.aim(target_copy, t, active_bullet_speed, true);
            } else {
                buff_big_target.get_target(power_runes, t);
                auto target_copy = buff_big_target;
                buff_command = buff_aimer.aim(target_copy, t, active_bullet_speed, true);
            }
            cboard->send(buff_command);
            frame_count++;
            continue;
        }

        // 自瞄模式 / 前哨站模式
        if (mode != io::Mode::auto_aim && mode != io::Mode::outpost) {
            cboard->send({false, false, 0.0, 0.0, 0.0, 0.0});
            frame_count++;
            continue;
        }

        auto yolo_start    = std::chrono::steady_clock::now();
        auto armors        = detector.detect(img);
        cv::Rect roi_rect;
        bool roi_active = false;
        if (detector.get_debug_roi(roi_rect, roi_active) && roi_active && roi_rect.width > 0 && roi_rect.height > 0) {
            cv::rectangle(img, roi_rect, cv::Scalar(0, 255, 255), 2);
            tools::draw_text(img, "ROI", cv::Point(roi_rect.x, std::max(0, roi_rect.y - 8)), {0, 255, 255});
        }
        total_armors += armors.size();
        if (!armors.empty()) {
            detected_frames++;
        }
        auto tracker_start = std::chrono::steady_clock::now();
        auto targets       = tracker.track(armors, t);
        auto aimer_start   = std::chrono::steady_clock::now();
        double active_bullet_speed = cboard->bullet_speed;
        auto command       = aimer.aim(targets, t, active_bullet_speed);

        const double nan_deg = std::numeric_limits<double>::quiet_NaN();
        double aim_debug_yaw_deg = nan_deg;
        double planner_debug_yaw_deg = nan_deg;
        double plan_target_yaw_deg = nan_deg;
        double plan_yaw_deg = nan_deg;

        if (aimer.debug_aim_point.valid) {
            aim_debug_yaw_deg = aimer.debug_aim_point.xyza[3] * 57.3;
        }

        if (!targets.empty()) {
            auto plan = planner.plan(targets.front(), active_bullet_speed);
            planner_debug_yaw_deg = planner.debug_xyza[3] * 57.3;
            plan_target_yaw_deg = plan.target_yaw * 57.3;
            plan_yaw_deg = plan.yaw * 57.3;
            if (plan.control) {
                command.control   = true;
                command.yaw       = plan.yaw;
                command.pitch     = plan.pitch;
                command.yaw_vel   = plan.yaw_vel;
                command.pitch_vel = plan.pitch_vel;
            }
        }

        auto finish        = std::chrono::steady_clock::now();

        Eigen::Quaterniond gimbal_q = q;
        Eigen::Vector3d ypr         = tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0);
        auto yaw                    = ypr[0];
        auto pitch                  = ypr[1];

        const io::Command prev_command = last_command;

        // 平滑过渡逻辑
        if (enable_target_stabilize && command.control && prev_command.control) {
            double delta_yaw = std::abs(command.yaw - prev_command.yaw) * 57.3;
            double delta_pitch = std::abs(command.pitch - prev_command.pitch) * 57.3;

            if (delta_yaw > target_jump_angle_threshold_deg || delta_pitch > target_jump_angle_threshold_deg) {
                command.yaw = (1.0 - target_stabilize_alpha) * prev_command.yaw + target_stabilize_alpha * command.yaw;
                command.pitch = (1.0 - target_stabilize_alpha) * prev_command.pitch + target_stabilize_alpha * command.pitch;
            }
        }

        command.shoot = shooter.shoot(command, aimer, targets, ypr);

        tools::logger()->info(
            "[{}] yolo: {:.1f}ms, tracker: {:.1f}ms, aimer: {:.1f}ms",
            frame_count,
            tools::delta_time(tracker_start, yolo_start) * 1e3,
            tools::delta_time(aimer_start, tracker_start) * 1e3,
            tools::delta_time(finish, aimer_start) * 1e3
        );

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
            fmt::format("gimbal yaw{:.2f}, pitch{:.2f}", yaw * 57.3, pitch * 57.3),
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
        data["gimbal_pitch"] = pitch * 57.3;
        data["cmd_yaw"]    = command.yaw * 57.3;
        data["cmd_pitch"]  = command.pitch * 57.3;
        data["cmd_yaw_vel"] = command.yaw_vel;
        data["cmd_pitch_vel"] = command.pitch_vel;
        data["cmd_find"]   = command.control;
        data["cmd_fire"]   = command.shoot;
        data["shoot"]      = command.shoot;
        data["bullet_speed"] = cboard->bullet_speed;

        data["aim_debug_yaw_deg"] = aim_debug_yaw_deg;
        data["planner_debug_yaw_deg"] = planner_debug_yaw_deg;
        data["plan_target_yaw_deg"] = plan_target_yaw_deg;
        data["plan_yaw_deg"] = plan_yaw_deg;
        data["delta_aim_vs_gimbal_deg"] = aim_debug_yaw_deg - yaw * 57.3;
        data["delta_plan_target_vs_gimbal_deg"] = plan_target_yaw_deg - yaw * 57.3;
        data["delta_plan_vs_gimbal_deg"] = plan_yaw_deg - yaw * 57.3;
        data["delta_plan_vs_cmd_deg"] = plan_yaw_deg - command.yaw * 57.3;

        if (!targets.empty()) {
            auto target                                  = targets.front();
            std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
            std::array<int, 3> outpost_order{0, 1, 2};
            if (target.name == auto_aim::ArmorName::outpost && armor_xyza_list.size() == 3) {
                std::sort(
                    outpost_order.begin(),
                    outpost_order.end(),
                    [&](int a, int b) { return armor_xyza_list[a][2] < armor_xyza_list[b][2]; }
                );
            }

            bool show_jump_up = false;
            bool show_jump_down = false;
            if (target.has_jump_time()) {
                const double abs_w = std::abs(target.ekf_x()[7]);
                if (abs_w >= decision_speed) {
                    auto age = std::chrono::duration<double>(t - target.last_jump_time()).count();
                    auto dir = target.last_jump_dir();
                    if (dir < 0 && jump_pitch_up_duration > 0.0 && age >= 0.0 && age <= jump_pitch_up_duration) {
                        show_jump_up = true;
                    }
                    if (dir > 0 && jump_pitch_down_duration > 0.0 && age >= 0.0 && age <= jump_pitch_down_duration) {
                        show_jump_down = true;
                    }
                }
            }

            for (std::size_t i = 0; i < armor_xyza_list.size(); ++i) {
                const auto& xyza = armor_xyza_list[i];
                auto image_points =
                    solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
                tools::draw_points(img, image_points, { 0, 255, 0 });

                cv::Point2f center{0.0F, 0.0F};
                for (const auto& pt : image_points) {
                    center.x += pt.x;
                    center.y += pt.y;
                }
                center.x /= static_cast<float>(image_points.size());
                center.y /= static_cast<float>(image_points.size());
                tools::draw_text(img, fmt::format("id:{}", i), center, { 255, 255, 0 });

                if (target.name == auto_aim::ArmorName::outpost && armor_xyza_list.size() == 3) {
                    std::string tag = "middle";
                    if (static_cast<int>(i) == outpost_order[0]) tag = "low";
                    if (static_cast<int>(i) == outpost_order[2]) tag = "high";
                    tools::draw_text(img, tag, cv::Point2f{center.x, center.y + 18.0F}, { 0, 255, 255 });
                }
            }
            Eigen::VectorXd x = target.ekf_x();
            std::vector<cv::Point3f> center_pt = {{static_cast<float>(x[0]), static_cast<float>(x[2]), static_cast<float>(x[4])}};
            auto center_img_pts = solver.world2pixel(center_pt);
            if (!center_img_pts.empty()) {
                cv::circle(img, center_img_pts[0], 5, {255, 255, 0}, -1);
            }
            auto aim_point           = aimer.debug_aim_point;
            Eigen::Vector4d aim_xyza = aim_point.xyza;
            auto image_points =
                solver
                    .reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
            if (aim_point.valid) {
                tools::draw_points(img, image_points, { 0, 0, 255 });

                cv::Point2f aim_center{0.0F, 0.0F};
                for (const auto& pt : image_points) {
                    aim_center.x += pt.x;
                    aim_center.y += pt.y;
                }
                aim_center.x /= static_cast<float>(image_points.size());
                aim_center.y /= static_cast<float>(image_points.size());
                if (show_jump_up) {
                    tools::draw_text(img, "Up", cv::Point2f{aim_center.x, aim_center.y - 14.0F}, { 0, 165, 255 });
                }
                if (show_jump_down) {
                    tools::draw_text(img, "Down", cv::Point2f{aim_center.x, aim_center.y - 14.0F}, { 0, 0, 255 });
                }
                if (command.shoot) {
                    tools::draw_text(img, "fire", cv::Point2f{aim_center.x, aim_center.y + 14.0F}, { 0, 0, 255 });
                }
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

        cv::resize(img, img, {}, 1.0, 1.0);
        cv::imshow("reprojection", img);
        auto key = cv::waitKey(1);
        if (key == 'q')
            break;

        cboard->send(command);
        if (command.control) {
            last_command = command;
        }
        frame_count++;
    }

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
