#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{@config-path   | configs/sentry.yaml    | yaml配置文件的路径}"
  "{tradition t    |  false                 | 是否使用传统方法识别}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto use_tradition = cli.get<bool>("tradition");

  tools::Exiter exiter;

  io::Camera camera(config_path);
  auto_aim::Detector detector(config_path, true);
  auto_aim::YOLO yolo(config_path, true);

  std::chrono::steady_clock::time_point timestamp;

  int total_frames = 0;
  int detect_frames = 0;

  while (!exiter.exit()) {
    cv::Mat img;
    std::list<auto_aim::Armor> armors;

    camera.read(img, timestamp);

    if (img.empty()) break;

    auto last = std::chrono::steady_clock::now();

    if (use_tradition)
      armors = detector.detect(img);
    else
      armors = yolo.detect(img);

    cv::Rect roi_rect;
    bool roi_active = false;
    if (yolo.get_debug_roi(roi_rect, roi_active)) {
      tools::logger()->info("[ROI] active={}, x={}, y={}, w={}, h={}",
        roi_active, roi_rect.x, roi_rect.y, roi_rect.width, roi_rect.height);
    }

    total_frames++;
    if (!armors.empty()) detect_frames++;

    double detect_rate = total_frames > 0 ? 100.0 * detect_frames / total_frames : 0.0;

    auto now = std::chrono::steady_clock::now();
    auto dt = tools::delta_time(now, last);
    tools::logger()->info("{:.2f} fps  detect_rate={:.1f}%  ({}/{} frames)",
      1 / dt, detect_rate, detect_frames, total_frames);

    auto key = cv::waitKey(33);
    if (key == 'q') break;
  }

  double final_rate = total_frames > 0 ? 100.0 * detect_frames / total_frames : 0.0;
  tools::logger()->info("=== 识别结束: 总识别率 {:.2f}%  ({}/{} 帧) ===",
    final_rate, detect_frames, total_frames);

  return 0;
}