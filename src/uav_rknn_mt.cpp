#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
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
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/thread_pool.hpp"

const std::string keys =
  "{help h usage ? |                  | 输出命令行参数说明}"
  "{@config-path   | configs/uav.yaml | yaml配置文件路径 }";

using namespace std::chrono_literals;

namespace
{
struct PendingFrame
{
  auto_aim::YOLO::JobId job_id = auto_aim::YOLO::kInvalidJobId;
  int id = 0;
  std::chrono::steady_clock::time_point t;
};
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  try {
    auto yaml = YAML::LoadFile(config_path);
    const auto yolo_name = yaml["yolo_name"].as<std::string>("");
    if (yolo_name != "yolov5_rknn") {
      tools::logger()->error(
        "uav_rknn_mt 需要 yolo_name: yolov5_rknn (当前: {})，否则无法使用 YOLO::submit/wait",
        yolo_name);
      return 1;
    }
  } catch (const std::exception & e) {
    tools::logger()->error("读取配置失败: {}", e.what());
    return 1;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::Camera camera(config_path);
  io::CBoard cboard(config_path);

  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  auto_buff::Buff_Detector buff_detector(config_path);
  auto_buff::Solver buff_solver(config_path);
  auto_buff::SmallTarget buff_small_target;
  auto_buff::BigTarget buff_big_target;
  auto_buff::Aimer buff_aimer(config_path);

  std::atomic<io::Mode> mode{io::Mode::idle};
  io::Mode last_mode = io::Mode::idle;

  std::mutex camera_mtx;
  tools::OrderedQueue frame_queue;

  constexpr int kMaxInflight = 3;

  auto producer_thread = std::thread([&]() {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    int frame_id = 0;
    std::deque<PendingFrame> pending;

    auto drain_pending = [&]() {
      while (!pending.empty() && !exiter.exit()) {
        auto pf = pending.front();
        pending.pop_front();
        (void)yolo.wait(pf.job_id);
      }
    };

    while (!exiter.exit()) {
      const auto m = mode.load();

      if (m != io::Mode::auto_aim && m != io::Mode::outpost) {
        drain_pending();
        std::this_thread::sleep_for(2ms);
        continue;
      }

      {
        std::scoped_lock<std::mutex> lk(camera_mtx);
        camera.read(img, t);
      }

      if (img.empty()) {
        std::this_thread::sleep_for(1ms);
        continue;
      }

      const int id = ++frame_id;
      const auto job_id = yolo.submit(img, id);
      if (job_id == auto_aim::YOLO::kInvalidJobId) {
        continue;
      }

      pending.push_back(PendingFrame{job_id, id, t});

      if (pending.size() >= static_cast<size_t>(kMaxInflight)) {
        auto pf = pending.front();
        pending.pop_front();

        tools::Frame frame;
        frame.id = pf.id;
        frame.img = cv::Mat();
        frame.t = pf.t;
        frame.q = Eigen::Quaterniond::Identity();
        frame.armors = yolo.wait(pf.job_id);

        frame_queue.enqueue(frame);
      }
    }

    // exit: drain to keep internal queues bounded
    // (best-effort, do not block forever)
    // Note: exiter.exit() already true, but wait may still block; keep it minimal.
    while (!pending.empty()) {
      auto pf = pending.front();
      pending.pop_front();
      (void)yolo.wait(pf.job_id);
    }
  });

  while (!exiter.exit()) {
    const io::Mode current_mode = cboard.mode;
    mode.store(current_mode);

    if (last_mode != current_mode) {
      tools::logger()->info("Switch to {}", io::MODES[current_mode]);
      last_mode = current_mode;
    }

    /// 自瞄 / 前哨站
    if (current_mode == io::Mode::auto_aim || current_mode == io::Mode::outpost) {
      tools::Frame frame;
      if (!frame_queue.try_dequeue(frame)) {
        std::this_thread::sleep_for(1ms);
        continue;
      }

      const Eigen::Quaterniond q = cboard.imu_at(frame.t - 1ms);
      // recorder.record(frame.img, q, frame.t);

      solver.set_R_gimbal2world(q);
      const Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

      auto targets = tracker.track(frame.armors, frame.t);

      auto command = aimer.aim(targets, frame.t, cboard.bullet_speed);
      command.shoot = shooter.shoot(command, aimer, targets, ypr);

      cboard.send(command);
    }

    /// 打符
    else if (current_mode == io::Mode::small_buff || current_mode == io::Mode::big_buff) {
      cv::Mat img;
      std::chrono::steady_clock::time_point t;
      {
        std::scoped_lock<std::mutex> lk(camera_mtx);
        camera.read(img, t);
      }

      if (img.empty()) {
        std::this_thread::sleep_for(1ms);
        continue;
      }

      const Eigen::Quaterniond q = cboard.imu_at(t - 1ms);
      // recorder.record(img, q, t);

      buff_solver.set_R_gimbal2world(q);

      auto power_runes = buff_detector.detect(img);
      buff_solver.solve(power_runes);

      io::Command buff_command;
      if (current_mode == io::Mode::small_buff) {
        buff_small_target.get_target(power_runes, t);
        auto target_copy = buff_small_target;
        buff_command = buff_aimer.aim(target_copy, t, cboard.bullet_speed, true);
      } else {
        buff_big_target.get_target(power_runes, t);
        auto target_copy = buff_big_target;
        buff_command = buff_aimer.aim(target_copy, t, cboard.bullet_speed, true);
      }

      cboard.send(buff_command);
    }

    else {
      std::this_thread::sleep_for(2ms);
    }
  }

  if (producer_thread.joinable()) producer_thread.join();
  return 0;
}
