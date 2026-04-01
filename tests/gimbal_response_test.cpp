#include <cstddef>
#include <array>
#include <algorithm>
#include <chrono>
#include <memory>
#include <cmath>
#include <vector>
#include <string>

#include <fmt/core.h>
#include <fmt/format.h> // Explicit formatting support

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
// #include "io/cboard.hpp"powet
#include "io/ros2/ros2.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{delta-angle a  |          8          | yaw轴delta角}"
  "{circle      c  |         0.2         | delta_angle的切片数}"
  "{signal-mode m  |     triangle_wave   | 发送信号的模式}"
  "{axis        x  |         yaw         | 发送信号的轴}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

double yaw_cal(double t)
{
  double A = 7;
  double T = 4;  // s

  return A * std::sin(2 * M_PI * t / T);  // 31是云台yaw初始角度，单位为度
}

double pitch_cal(double t)
{
  double A = 7;
  double T = 4;  // s

  return A * std::sin(2 * M_PI * t / T + M_PI / 2) + 18;  // 18是云台pitch初始角度，单位为度
}

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  auto delta_angle = cli.get<double>("delta-angle");
  auto circle = cli.get<double>("circle");
  auto signal_mode = cli.get<std::string>("signal-mode");
  auto axis = cli.get<std::string>("axis");
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;

  double cboard_bullet_speed = 21.0;
    bool cboard_use_default_bullet_speed = true;
    bool phoenix_angles_in_degrees = false;
    double imu_yaw_offset_rad = 0.0;
    double imu_pitch_offset_rad = 0.0;
    try {
        auto yaml = YAML::LoadFile(config_path);
        if (yaml["bullet_speed"]) cboard_bullet_speed = yaml["bullet_speed"].as<double>();
        if (yaml["use_default_bullet_speed"]) cboard_use_default_bullet_speed = yaml["use_default_bullet_speed"].as<bool>();
        if (yaml["phoenix_angle_unit"]) {
            auto unit = yaml["phoenix_angle_unit"].as<std::string>();
            for (auto &c: unit) c = static_cast<char>(std::tolower(c));
            phoenix_angles_in_degrees = (unit == "deg" || unit == "degree" || unit == "degrees");
        }
        if (yaml["imu_yaw_offset_deg"]) imu_yaw_offset_rad = yaml["imu_yaw_offset_deg"].as<double>() * M_PI / 180.0;
        if (yaml["imu_pitch_offset_deg"]) imu_pitch_offset_rad = yaml["imu_pitch_offset_deg"].as<double>() * M_PI / 180.0;
        if (yaml["imu_yaw_offset_rad"]) imu_yaw_offset_rad = yaml["imu_yaw_offset_rad"].as<double>();
        if (yaml["imu_pitch_offset_rad"]) imu_pitch_offset_rad = yaml["imu_pitch_offset_rad"].as<double>();
    } catch (const YAML::Exception & e) {
        tools::logger()->warn("Failed to read cboard config from {}: {}", config_path, e.what());
    }

  // io::CBoard cboard(config_path);

   // Initialize ROS2 and IO wrappers (non-blocking): Subscribe to autoaim and publish commands
   rclcpp::init(argc, argv);
   auto nav_sub = std::make_shared<io::Subscribe2Nav>();
   auto pub_node = std::make_shared<io::Publish2Nav>();
   std::thread nav_thread([nav_sub]() { nav_sub->start(); });
   nav_thread.detach();
   std::thread pub_thread([pub_node]() { pub_node->start(); });
   pub_thread.detach();


   auto mode       = io::Mode::idle;
    auto last_mode  = io::Mode::idle;
  auto init_angle = 0;
  double slice = circle * 100;  //切片数=周期*帧率
  auto dangle = delta_angle / slice;
  double cmd_angle = init_angle;

  int axis_index = axis == "yaw" ? 0 : 1;  // 0 for yaw, 1 for pitch

  double error = 0;
  int count = 0;

  io::Command init_command{1, 0, 0, 0};
  // cboard.send(init_command);
  // send command via ROS2 publisher node
  if (pub_node) {
    Eigen::Vector4d out;
    out[0] = init_command.yaw;
    out[1] = init_command.pitch;
    out[2] = 0.0; // reserved
    out[3] = init_command.control ? 1.0 : 0.0; // is_find flag
    pub_node->send_data(out);
  }
  std::this_thread::sleep_for(5s);  //等待云台归零

  io::Command command{0};
  io::Command last_command{0};

  double t = 0;
  auto last_t = t;
  double dt = 0.005;  // 5ms, 模拟200fps

  auto t0 = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    nlohmann::json data;
    auto timestamp = std::chrono::steady_clock::now();

    std::this_thread::sleep_for(1ms);

    Eigen::Quaterniond q;
    // sub
    // try to get latest autoaim data from ROS2; if absent, fall back to identity quaternion
    std::optional<io::AutoaimData> maybe = std::nullopt;
    // Subscribe2Nav exposes get_autoaim_data()
    if (nav_sub) {
        maybe = nav_sub->get_autoaim_data();
        tools::logger()->info("maybe has value: {}", maybe.has_value());
    }

    if (maybe.has_value()) {
        auto ad = maybe.value();
        double yaw = static_cast<double>(ad.high_gimbal_yaw);
        double pitch = static_cast<double>(ad.pitch);
        if (phoenix_angles_in_degrees) {
            constexpr double kDeg2Rad = M_PI / 180.0;
            yaw *= kDeg2Rad;
            pitch *= kDeg2Rad;
        }
        yaw += imu_yaw_offset_rad;
        pitch += imu_pitch_offset_rad;
        Eigen::AngleAxisd yaw_aa(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd pitch_aa(pitch, Eigen::Vector3d::UnitY());
        q = (yaw_aa * pitch_aa).normalized();
        mode = static_cast<io::Mode>(ad.mode);
    } else {
        q = Eigen::Quaterniond::Identity();
    }

    if (last_mode != mode) {
        tools::logger()->info("Switch to {}", io::MODES[mode].c_str());
        last_mode = mode;
    }


    Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0);

    if (signal_mode == "triangle_wave") {
      if (count == slice) {
        cmd_angle = init_angle;
        command = {1, 0, 0, 0};
        if (axis_index == 0)
          command.yaw = cmd_angle / 57.3;
        else
          command.pitch = cmd_angle / 57.3;
        count = 0;

      } else {
        cmd_angle += dangle;
        if (axis_index == 0)
          command.yaw = cmd_angle / 57.3;
        else
          command.pitch = cmd_angle / 57.3;
        count++;
      }

      // cboard.send(command);
      // send command via ROS2 publisher node
      if (pub_node) {
        Eigen::Vector4d out;
        out[0] = command.yaw;
        out[1] = command.pitch;
        out[2] = 0.0; // reserved
        out[3] = command.control ? 1.0 : 0.0; // is_find flag
        pub_node->send_data(out);
      }
      if (axis_index == 0) {
        data["cmd_yaw"] = command.yaw * 57.3;
        data["last_cmd_yaw"] = last_command.yaw * 57.3;
        data["gimbal_yaw"] = eulers[0] * 57.3;
      } else {
        data["cmd_pitch"] = command.pitch * 57.3;
        data["last_cmd_pitch"] = last_command.pitch * 57.3;
        data["gimbal_pitch"] = eulers[1] * 57.3;
      }
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
      last_command = command;
      plotter.plot(data);
      std::this_thread::sleep_for(8ms);  //模拟自瞄100fps
    }

    else if (signal_mode == "step") {
      if (count == 300) {
        cmd_angle += delta_angle;
        count = 0;
      }
      command = {1, 0, tools::limit_rad(cmd_angle / 57.3), 0};
      count++;

      // cboard.send(command);
      // send command via ROS2 publisher node
      if (pub_node) {
        Eigen::Vector4d out;
        out[0] = command.yaw;
        out[1] = command.pitch;
        out[2] = 0.0; // reserved
        out[3] = command.control ? 1.0 : 0.0; // is_find flag
        pub_node->send_data(out);
      }
      data["cmd_yaw"] = command.yaw * 57.3;
      data["last_cmd_yaw"] = last_command.yaw * 57.3;
      data["gimbal_yaw"] = eulers[0] * 57.3;
      last_command = command;
      plotter.plot(data);
      std::this_thread::sleep_for(8ms);  //模拟自瞄100fps
    }

    else if (signal_mode == "circle") {
      std::cout << "t: " << t << std::endl;
      command.yaw = yaw_cal(t) / 57.3;
      command.pitch = pitch_cal(t) / 57.3;
      command.control = 1;
      command.shoot = 0;
      t += dt;
      if (t - last_t > 2) {
        t += 2.4;
        last_t = t;
      }
      // cboard.send(command);
      // send command via ROS2 publisher node
      if (pub_node) {
        Eigen::Vector4d out;
        out[0] = command.yaw;
        out[1] = command.pitch;
        out[2] = 0.0; // reserved
        out[3] = command.control ? 1.0 : 0.0; // is_find flag
        pub_node->send_data(out);
      }

      data["t"] = t;
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
      data["gimbal_yaw"] = eulers[0] * 57.3;
      data["gimbal_pitch"] = eulers[1] * 57.3;
      plotter.plot(data);
      std::this_thread::sleep_for(9ms);
    }
  }
  return 0;
}