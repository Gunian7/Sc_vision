#include "image_from_ros.hpp"

#include <chrono>
#include <thread>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <sensor_msgs/image_encodings.hpp>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{

ImageFromRos::ImageFromRos(std::string topic, std::size_t queue_capacity, bool suppress_rx_stall_log)
: topic_(std::move(topic)),
  queue_(queue_capacity),
  suppress_rx_stall_log_(suppress_rx_stall_log)
{
  if (topic_.empty()) {
    throw std::runtime_error("ImageFromRos: ros_image_topic is empty");
  }

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  node_ = std::make_shared<rclcpp::Node>("image_from_ros", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
      topic_,
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::SharedPtr msg) { on_image(std::move(msg)); });

  executor_.add_node(node_);

  const auto t0 = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(stall_mtx_);
    last_good_rx_ = t0;
    last_rx_warn_ = t0 - 86400s;
    last_watchdog_tick_ = t0;
  }

  spin_thread_ = std::thread([this] { spin_loop(); });

  tools::logger()->info("[ImageFromRos] subscribe {}", topic_);
}

ImageFromRos::~ImageFromRos()
{
  quit_.store(true);
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  executor_.remove_node(node_);
  tools::logger()->info("[ImageFromRos] destroyed");
}

void ImageFromRos::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data = queue_.pop();
  img       = std::move(data.img);
  timestamp = data.timestamp;
}

void ImageFromRos::on_image(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!msg || msg->data.empty()) {
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    tools::logger()->warn("[ImageFromRos] cv_bridge: {}", e.what());
    return;
  }

  CameraData data;
  data.img       = cv_ptr->image.clone();
  data.timestamp = std::chrono::steady_clock::now();
  queue_.push(std::move(data));
  {
    std::lock_guard<std::mutex> lock(stall_mtx_);
    last_good_rx_ = data.timestamp;
  }
}

void ImageFromRos::maybe_warn_rx_stall()
{
  if (suppress_rx_stall_log_) {
    return;
  }
  constexpr auto kIdleWarn = 1500ms;
  constexpr auto kRepeatWarn = 4000ms;

  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(stall_mtx_);
  if (now - last_good_rx_ < kIdleWarn) {
    return;
  }
  if (now - last_rx_warn_ < kRepeatWarn) {
    return;
  }
  last_rx_warn_ = now;
  const double idle_s = std::chrono::duration<double>(now - last_good_rx_).count();
  tools::logger()->warn(
      "[ImageFromRos] 已超过 {:.2f}s 未收到有效图像 topic={}（检查发布端 / 话题名 / QoS）",
      idle_s,
      topic_);
}

void ImageFromRos::spin_loop()
{
  while (!quit_.load()) {
    if (rclcpp::ok()) {
      executor_.spin_some();
    }
    bool tick_watchdog = false;
    {
      const auto now = std::chrono::steady_clock::now();
      std::lock_guard<std::mutex> lock(stall_mtx_);
      if (now - last_watchdog_tick_ >= 500ms) {
        last_watchdog_tick_ = now;
        tick_watchdog = true;
      }
    }
    if (tick_watchdog) {
      maybe_warn_rx_stall();
    }
    std::this_thread::sleep_for(500us);
  }
}

}  // namespace io
