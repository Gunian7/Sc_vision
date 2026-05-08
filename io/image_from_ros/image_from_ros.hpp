#ifndef IO__IMAGE_FROM_ROS_HPP
#define IO__IMAGE_FROM_ROS_HPP

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "io/camera.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{

/**
 * 订阅 ROS sensor_msgs/Image，将帧写入队列；read() 语义对齐 MindVision（阻塞 pop）。
 * 进程内需已 rclcpp::init（本类在必要时会以 argc=0 兜底 init，正式入口建议 main 显式 init）。
 */
class ImageFromRos : public CameraBase
{
public:
  /**
   * @param topic 图像话题，默认可与 dual_camera_node 的 auto_aim 输出对齐
   * @param queue_capacity 队列深度；满时丢弃最旧帧（PopWhenFull）
   * @param suppress_rx_stall_log yaml hero_ros_suppress_rx_stall_log：阻塞/低速取图时不刷接收超时 warn
   */
  ImageFromRos(
    std::string topic, std::size_t queue_capacity = 3, bool suppress_rx_stall_log = false);
  ~ImageFromRos() override;

  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
  struct CameraData
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
  };

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg);
  void spin_loop();
  void maybe_warn_rx_stall();

  std::string topic_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  std::atomic<bool> quit_{false};
  std::thread spin_thread_;

  tools::ThreadSafeQueue<CameraData, true> queue_;

  mutable std::mutex stall_mtx_;
  std::chrono::steady_clock::time_point last_good_rx_{};
  std::chrono::steady_clock::time_point last_rx_warn_{};
  std::chrono::steady_clock::time_point last_watchdog_tick_{};
  bool suppress_rx_stall_log_{false};
};

}  // namespace io

#endif  // IO__IMAGE_FROM_ROS_HPP
