/**
 * Hero：ROS 图像话题统计（频率、间隔抖动、分辨率/编码），不跑识别。
 * 用法: hero_camera_ros_test [--topic /image_for_auto_aim]
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <numeric>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "io/hero_ros_yaml_flags.hpp"

#include "tools/exiter.hpp"
#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace {

double percentile(std::deque<double>& sorted, double p)
{
  if (sorted.empty()) {
    return 0.;
  }
  std::sort(sorted.begin(), sorted.end());
  const size_t i = static_cast<size_t>((sorted.size() - 1) * p * 0.01);
  return sorted[i];
}

} // namespace

int main(int argc, char** argv)
{
  const std::string keys =
      "{help h usage ? | | 输出帮助}"
      "{topic t | /image_for_auto_aim | sensor_msgs/Image 话题}"
      "{config c | | 可选 YAML；若含 hero_ros_suppress_rx_stall_log 则抑制接收超时告警}"
      "{suppress_rx_warn s | false | 强制关闭「长时间未收到图像」告警（无需 yaml）}";

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  const std::string topic = cli.get<std::string>("topic");
  const std::string cam_cfg = cli.get<std::string>("config");
  bool suppress_rx_stall_log = cli.get<bool>("suppress_rx_warn");
  if (!cam_cfg.empty()) {
    suppress_rx_stall_log =
        suppress_rx_stall_log || io::yaml_hero_ros_suppress_rx_stall_log(cam_cfg);
  }

  rclcpp::init(argc, argv);
  tools::Exiter exiter;
  auto node = std::make_shared<rclcpp::Node>("hero_camera_ros_test");

  std::uint64_t total = 0;
  std::uint64_t window_count = 0;
  auto window_start = std::chrono::steady_clock::now();
  auto last_cb = std::chrono::steady_clock::now();
  auto last_image_rx = std::chrono::steady_clock::now();
  auto last_stall_warn = std::chrono::steady_clock::now() - std::chrono::hours(1);
  std::deque<double> dt_ms_window;
  constexpr std::size_t kWindowMax = 512;
  std::string last_enc;
  uint32_t last_w = 0, last_h = 0;

  auto sub = node->create_subscription<sensor_msgs::msg::Image>(
      topic,
      rclcpp::SensorDataQoS(),
      [&](const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!msg || msg->data.empty()) {
          return;
        }
        const auto now = std::chrono::steady_clock::now();
        last_image_rx = now;
        if (total > 0) {
          const double ms =
              std::chrono::duration<double, std::milli>(now - last_cb).count();
          dt_ms_window.push_back(ms);
          if (dt_ms_window.size() > kWindowMax) {
            dt_ms_window.pop_front();
          }
        }
        last_cb = now;
        total++;
        window_count++;
        last_enc = msg->encoding;
        last_w = msg->width;
        last_h = msg->height;
      });

  tools::logger()->info("[hero_camera_ros_test] subscribe {}", topic);

  rclcpp::WallRate rate(2.0);
  while (!exiter.exit() && rclcpp::ok()) {
    rclcpp::spin_some(node);
    const auto now = std::chrono::steady_clock::now();
    if (!suppress_rx_stall_log && now - last_image_rx >= 1500ms) {
      if (now - last_stall_warn >= 4000ms) {
        last_stall_warn = now;
        const double idle_s = std::chrono::duration<double>(now - last_image_rx).count();
        tools::logger()->warn(
            "[hero_camera_ros_test] {:.2f}s 未收到有效图像 topic={}（QoS/话题/发布端）",
            idle_s,
            topic.c_str());
      }
    }
    const double win_s = std::chrono::duration<double>(now - window_start).count();
    if (win_s >= 1.0) {
      const double hz = static_cast<double>(window_count) / win_s;
      double mean_ms = 0., std_ms = 0.;
      if (!dt_ms_window.empty()) {
        mean_ms =
            std::accumulate(dt_ms_window.begin(), dt_ms_window.end(), 0.) /
            static_cast<double>(dt_ms_window.size());
        double acc = 0.;
        for (double d : dt_ms_window) {
          acc += (d - mean_ms) * (d - mean_ms);
        }
        std_ms = std::sqrt(acc / static_cast<double>(dt_ms_window.size()));
      }
      auto sorted = dt_ms_window;
      const double p50 = percentile(sorted, 50.);
      sorted = dt_ms_window;
      const double p95 = percentile(sorted, 95.);
      sorted = dt_ms_window;
      const double p99 = percentile(sorted, 99.);
      tools::logger()->info(
          "[hero_camera_ros_test] last1s hz={:.2f} frames={} total={} | "
          "dt_ms mean={:.2f} std={:.2f} p50={:.2f} p95={:.2f} p99={:.2f} | "
          "{}x{} {}",
          hz,
          window_count,
          total,
          mean_ms,
          std_ms,
          p50,
          p95,
          p99,
          last_w,
          last_h,
          last_enc.c_str());
      window_count = 0;
      window_start = now;
    }
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
