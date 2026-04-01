#include <chrono>
#include <cmath>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "communicate_26/msg/autoaim.hpp"
#include "communicate_26/msg/serial_info.hpp"
#include "tools/exiter.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{delta-angle a  |          8          | 角度幅值(度)}"
  "{circle      c  |         4.0         | 正弦/三角波周期(s)}"
  "{signal-mode m  |         sine        | 模式: sine|step|triangle_wave|circle}"
  "{axis        x  |         yaw         | 轴: yaw|pitch}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径(兼容保留)}";

double yaw_cal(double t)
{
  constexpr double A = 7.0;
  constexpr double T = 4.0;
  return A * std::sin(2.0 * M_PI * t / T);
}

double pitch_cal(double t)
{
  constexpr double A = 7.0;
  constexpr double T = 4.0;
  return A * std::sin(2.0 * M_PI * t / T + M_PI / 2.0) + 18.0;
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

  (void)config_path;

  tools::Exiter exiter;
  tools::Plotter plotter;

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("gimbal_response_test_ros");

  // 单节点自发布：直接发布 /shoot_info
  auto shoot_pub =
    node->create_publisher<communicate_26::msg::SerialInfo>("/shoot_info", 10);

  // 同时订阅 /communicate/autoaim
  std::mutex autoaim_mtx;
  communicate_26::msg::Autoaim::SharedPtr last_autoaim = nullptr;
  auto autoaim_sub = node->create_subscription<communicate_26::msg::Autoaim>(
    "/communicate/autoaim", 10,
    [&](communicate_26::msg::Autoaim::UniquePtr msg) {
      std::lock_guard<std::mutex> lk(autoaim_mtx);
      last_autoaim = std::make_shared<communicate_26::msg::Autoaim>(*msg);
    });
  (void)autoaim_sub;

  const int axis_index = (axis == "yaw") ? 0 : 1;
  double period_s = circle > 1e-6 ? circle : 4.0;
  double slice = period_s * 100.0;  // 100Hz
  if (slice < 1.0) {
    slice = 1.0;
  }

  double cmd_angle = 0.0;
  const double dangle = delta_angle / slice;
  int count = 0;

  double cmd_yaw_deg = 0.0;
  double cmd_pitch_deg = 0.0;
  double last_cmd_yaw_deg = 0.0;
  double last_cmd_pitch_deg = 0.0;

  double t = 0.0;
  double last_t = 0.0;
  constexpr double dt = 0.005;
  auto t0 = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    rclcpp::spin_some(node);
    nlohmann::json data;
    const double now_t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    if (signal_mode == "triangle_wave") {
      if (count >= static_cast<int>(slice)) {
        cmd_angle = 0.0;
        count = 0;
      } else {
        cmd_angle += dangle;
        ++count;
      }
      if (axis_index == 0) {
        cmd_yaw_deg = cmd_angle;
        cmd_pitch_deg = 0.0;
      } else {
        cmd_pitch_deg = cmd_angle;
        cmd_yaw_deg = 0.0;
      }
    } else if (signal_mode == "step") {
      if (count >= 300) {
        cmd_angle += delta_angle;
        count = 0;
      }
      ++count;
      if (axis_index == 0) {
        cmd_yaw_deg = cmd_angle;
        cmd_pitch_deg = 0.0;
      } else {
        cmd_pitch_deg = cmd_angle;
        cmd_yaw_deg = 0.0;
      }
    } else if (signal_mode == "circle") {
      cmd_yaw_deg = yaw_cal(t);
      cmd_pitch_deg = pitch_cal(t);
      t += dt;
      if (t - last_t > 2.0) {
        t += 2.4;
        last_t = t;
      }
    } else {  // default: sine
      const double w = 2.0 * M_PI / period_s;
      const double sine = delta_angle * std::sin(w * now_t);
      if (axis_index == 0) {
        cmd_yaw_deg = sine;
        cmd_pitch_deg = 0.0;
      } else {
        cmd_pitch_deg = sine;
        cmd_yaw_deg = 0.0;
      }
    }

    communicate_26::msg::SerialInfo msg;
    msg.yaw = static_cast<float>(cmd_yaw_deg / 57.3);
    msg.pitch = static_cast<float>(cmd_pitch_deg / 57.3);
    msg.is_find.data = 1;
    shoot_pub->publish(msg);

    data["t"] = now_t;
    data["cmd_yaw"] = cmd_yaw_deg;
    data["cmd_pitch"] = cmd_pitch_deg;
    data["last_cmd_yaw"] = last_cmd_yaw_deg;
    data["last_cmd_pitch"] = last_cmd_pitch_deg;
    data["pub_yaw"] = msg.yaw * 57.3;
    data["pub_pitch"] = msg.pitch * 57.3;

    {
      std::lock_guard<std::mutex> lk(autoaim_mtx);
      if (last_autoaim) {
        data["sub_autoaim_yaw"] = static_cast<double>(last_autoaim->high_gimbal_yaw);
        data["sub_autoaim_pitch"] = static_cast<double>(last_autoaim->pitch);
        data["sub_autoaim_mode"] = static_cast<int>(last_autoaim->mode);
        data["sub_autoaim_rune_flag"] = static_cast<int>(last_autoaim->rune_flag);
      }
    }
    plotter.plot(data);

    last_cmd_yaw_deg = cmd_yaw_deg;
    last_cmd_pitch_deg = cmd_pitch_deg;

    std::this_thread::sleep_for(8ms);
  }

  rclcpp::shutdown();
  return 0;
}