#include "io/cboard.hpp"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                       | 输出命令行参数说明}"
  "{@config-path   | configs/standard.yaml | yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  tools::Exiter exiter;

  io::CBoard cboard(config_path);
  auto last_mode = cboard.mode;

  while (!exiter.exit()) {
    auto timestamp = std::chrono::steady_clock::now();

    std::this_thread::sleep_for(500ms);

    Eigen::Quaterniond q = cboard.imu_at(timestamp);

    Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0) * 57.3;
    tools::logger()->info("z{:.2f} y{:.2f} x{:.2f} degree", eulers[0], eulers[1], eulers[2]);
    tools::logger()->info(
        "bullet speed {:.2f} m/s | yaw_vel {:.3f} | pitch_vel {:.3f} | mode {} ({})",
        cboard.bullet_speed, cboard.yaw_vel, cboard.pitch_vel, static_cast<int>(cboard.mode),
        io::MODES[cboard.mode]);

    if (cboard.mode != last_mode) {
      tools::logger()->warn(
          "mode changed: {} ({}) -> {} ({})", static_cast<int>(last_mode), io::MODES[last_mode],
          static_cast<int>(cboard.mode), io::MODES[cboard.mode]);
      last_mode = cboard.mode;
    }
  }

  return 0;
}
