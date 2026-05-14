/**
 * Hero：ROS 图像话题统计（频率、间隔抖动、分辨率/编码），不跑识别。
 * 用法: hero_camera_ros_test [--topic /image_for_auto_aim]
 *
 * 设计：单线程 spin_some，订阅与定时器串行执行（无需互斥锁）。
 * hz_avg = 本周期帧数 / 墙钟间隔；inst_hz 为相邻两帧瞬时频率 1000/dt_ms，仅统计均值与 min/max。
 */
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "io/hero_ros_yaml_flags.hpp"

#include "tools/exiter.hpp"
#include "tools/logger.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  const std::string keys =
      "{help h usage ? | | 输出帮助}"
      "{topic t | /image_for_auto_aim | sensor_msgs/Image 话题}"
      "{config c | | 可选 YAML；若含 hero_ros_suppress_rx_stall_log 则抑制接收超时告警}"
      "{suppress_rx_warn s | false | 强制关闭「长时间未收到图像」告警（无需 yaml）}"
      "{qos_depth q | 2 | 订阅 KeepLast 深度（SensorDataQoS = best_effort；实时优先常用 1～2）}";

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  const std::string topic = cli.get<std::string>("topic");
  const std::string cam_cfg = cli.get<std::string>("config");
  bool suppress_rx_stall_log = cli.get<bool>("suppress_rx_warn");
  const int qos_depth = std::max(1, cli.get<int>("qos_depth"));
  if (!cam_cfg.empty()) {
    suppress_rx_stall_log =
        suppress_rx_stall_log || io::yaml_hero_ros_suppress_rx_stall_log(cam_cfg);
  }

  rclcpp::init(argc, argv);
  tools::Exiter exiter;
  auto node = std::make_shared<rclcpp::Node>("hero_camera_ros_test");

  std::uint64_t total = 0;
  std::uint64_t window_count = 0;
  auto last_cb = std::chrono::steady_clock::now();
  auto last_image_rx = std::chrono::steady_clock::now();
  auto last_stall_warn = std::chrono::steady_clock::now() - std::chrono::hours(1);
  auto last_stats_wall = std::chrono::steady_clock::now();
  /// 相邻两帧瞬时频率 Hz（1000/dt_ms），与回调中有效间隔一一对应
  std::vector<double> period_inst_hz;
  period_inst_hz.reserve(2048);
  std::string last_enc;
  uint32_t last_w = 0, last_h = 0;

  rclcpp::QoS image_qos =
      rclcpp::SensorDataQoS().keep_last(static_cast<size_t>(qos_depth));

  auto sub = node->create_subscription<sensor_msgs::msg::Image>(
      topic,
      image_qos,
      [&](const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!msg || msg->data.empty()) {
          return;
        }
        const auto now = std::chrono::steady_clock::now();
        last_image_rx = now;
        if (total > 0) {
          const double dt_ms =
              std::chrono::duration<double, std::milli>(now - last_cb).count();
          if (dt_ms > 1e-6) {
            period_inst_hz.push_back(1000.0 / dt_ms);
          }
        }
        last_cb = now;
        total++;
        window_count++;
        last_enc = msg->encoding;
        last_w = msg->width;
        last_h = msg->height;
      });

  tools::logger()->info(
      "[hero_camera_ros_test] subscribe {} qos_keep_last={}",
      topic,
      qos_depth);

  auto stall_timer = node->create_wall_timer(
      400ms,
      [&]() {
        if (suppress_rx_stall_log) {
          return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (now - last_image_rx < 1500ms) {
          return;
        }
        if (now - last_stall_warn < 4000ms) {
          return;
        }
        last_stall_warn = now;
        const double idle_s =
            std::chrono::duration<double>(now - last_image_rx).count();
        tools::logger()->warn(
            "[hero_camera_ros_test] {:.2f}s 未收到有效图像 topic={}（QoS/话题/发布端）",
            idle_s,
            topic.c_str());
      });

  auto stats_timer = node->create_wall_timer(
      1000ms,
      [&]() {
        const auto now = std::chrono::steady_clock::now();
        const double span_s =
            std::chrono::duration<double>(now - last_stats_wall).count();
        last_stats_wall = now;

        const uint64_t wc = window_count;
        window_count = 0;

        std::vector<double> hz_copy;
        hz_copy.swap(period_inst_hz);

        const uint64_t tot = total;
        const uint32_t lw = last_w;
        const uint32_t lh = last_h;
        const std::string enc = last_enc;

        const double hz_avg =
            (span_s > 1e-6) ? (static_cast<double>(wc) / span_s) : 0.;

        double inst_mean_hz = 0.;
        double inst_min_hz = 0.;
        double inst_max_hz = 0.;
        if (!hz_copy.empty()) {
          inst_mean_hz =
              std::accumulate(hz_copy.begin(), hz_copy.end(), 0.) /
              static_cast<double>(hz_copy.size());
          const auto mm =
              std::minmax_element(hz_copy.begin(), hz_copy.end());
          inst_min_hz = *mm.first;
          inst_max_hz = *mm.second;
        }

        tools::logger()->info(
            "[hero_camera_ros_test] period {:.3f}s hz_avg={:.2f} frames={} total={} | "
            "inst_hz mean={:.2f} min={:.2f} max={:.2f} | "
            "{}x{} {}",
            span_s,
            hz_avg,
            wc,
            tot,
            inst_mean_hz,
            inst_min_hz,
            inst_max_hz,
            lw,
            lh,
            enc.c_str());
      });

  (void)stall_timer;
  (void)stats_timer;

  while (!exiter.exit() && rclcpp::ok()) {
    rclcpp::spin_some(node);
    std::this_thread::yield();
  }

  rclcpp::shutdown();
  return 0;
}
