#include <fmt/core.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明 }"
  "{config-path c  | configs/demo.yaml | yaml配置文件的路径}"
  "{start-index s  | 0                 | 视频起始帧下标    }"
  "{end-index e    | 0                 | 视频结束帧下标    }"
  "{@input-path    | assets/demo/demo  | avi和txt文件的路径}";

namespace
{
struct FramePose
{
  double t;
  double w;
  double x;
  double y;
  double z;
};

struct PlaybackControl
{
  int requested_frame = -1;
  double playback_speed = 1.0;
  bool internal_frame_update = false;
  bool internal_speed_update = false;
  bool seek_from_slider = false;
};

int clamp_int(int value, int low, int high)
{
  return std::max(low, std::min(value, high));
}

constexpr double kSegmentResetThresholdS = 0.1;
constexpr int kMaxPlaybackWaitMs = 200;
}  // namespace

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");

  tools::Plotter plotter;
  tools::Exiter exiter;

  auto video_path = fmt::format("{}.avi", input_path);
  auto text_path = fmt::format("{}.txt", input_path);
  cv::VideoCapture video(video_path);
  std::ifstream text(text_path);

  if (!video.isOpened()) {
    tools::logger()->error("无法打开视频文件: {}", video_path);
    return 1;
  }

  if (!text.is_open()) {
    tools::logger()->error("无法打开姿态文件: {}", text_path);
    return 1;
  }

  std::vector<FramePose> records;
  for (FramePose pose; text >> pose.t >> pose.w >> pose.x >> pose.y >> pose.z;) {
    records.push_back(pose);
  }

  if (records.empty()) {
    tools::logger()->error("姿态文件为空或格式错误: {}", text_path);
    return 1;
  }

  auto video_frame_count = static_cast<int>(std::lround(video.get(cv::CAP_PROP_FRAME_COUNT)));
  auto total_frame_count = static_cast<int>(records.size());
  if (video_frame_count > 0) {
    total_frame_count = std::min(total_frame_count, video_frame_count);
  }

  if (total_frame_count <= 0) {
    tools::logger()->error("没有可用的离线帧数据");
    return 1;
  }

  if (video_frame_count > 0 && video_frame_count != static_cast<int>(records.size())) {
    tools::logger()->warn(
      "视频帧数({}) 与 txt 记录数({}) 不一致，将按较小值 {} 处理", video_frame_count, records.size(),
      total_frame_count);
  }

  auto last_frame_index = total_frame_count - 1;
  auto start_frame = clamp_int(start_index, 0, last_frame_index);
  auto end_frame = end_index > 0 ? clamp_int(end_index, 0, last_frame_index) : last_frame_index;

  if (start_frame > end_frame) {
    tools::logger()->error("起始帧 {} 不能大于结束帧 {}", start_frame, end_frame);
    return 1;
  }

  auto video_fps = video.get(cv::CAP_PROP_FPS);
  auto base_delay_ms =
    video_fps > 1e-3 ? std::max(1, static_cast<int>(std::lround(1000.0 / video_fps))) : 30;

  auto_aim::YOLO yolo(config_path);
  auto_aim::Solver solver(config_path);
  auto tracker = std::make_unique<auto_aim::Tracker>(config_path, solver);
  auto aimer = std::make_unique<auto_aim::Aimer>(config_path);

  cv::Mat img;
  auto t0 = std::chrono::steady_clock::now();

  auto_aim::Target last_target;
  io::Command last_command;
  double last_t = -1;

  const std::string window_name = "reprojection";
  const std::string frame_trackbar_name = "frame";
  const std::string speed_trackbar_name = "speed x100";

  PlaybackControl playback_control;
  int frame_trackbar_pos = start_frame;
  int speed_trackbar_pos = 100;

  cv::namedWindow(window_name, cv::WINDOW_NORMAL);
  cv::createTrackbar(
    frame_trackbar_name, window_name, nullptr, last_frame_index,
    [](int pos, void * userdata) {
      auto * control = static_cast<PlaybackControl *>(userdata);
      if (control->internal_frame_update) return;
      control->requested_frame = pos;
      control->seek_from_slider = true;
    },
    &playback_control);
  cv::createTrackbar(
    speed_trackbar_name, window_name, nullptr, 800,
    [](int pos, void * userdata) {
      auto * control = static_cast<PlaybackControl *>(userdata);
      if (control->internal_speed_update) return;
      control->playback_speed = std::max(pos, 1) / 100.0;
    },
    &playback_control);

  auto sync_speed_trackbar = [&](double speed) {
    playback_control.playback_speed = std::max(0.05, std::min(speed, 8.0));
    speed_trackbar_pos = static_cast<int>(std::lround(playback_control.playback_speed * 100.0));
    playback_control.internal_speed_update = true;
    cv::setTrackbarPos(speed_trackbar_name, window_name, speed_trackbar_pos);
    playback_control.internal_speed_update = false;
  };

  auto sync_frame_trackbar = [&](int frame_index) {
    frame_trackbar_pos = frame_index;
    playback_control.internal_frame_update = true;
    cv::setTrackbarPos(frame_trackbar_name, window_name, frame_trackbar_pos);
    playback_control.internal_frame_update = false;
  };

  sync_speed_trackbar(1.0);
  sync_frame_trackbar(start_frame);

  int current_frame = start_frame;
  int displayed_frame = -1;
  bool paused = false;
  bool refresh_frame = true;
  cv::Mat current_display_img;
  bool playback_anchor_valid = false;
  bool playback_anchor_needs_reset = true;
  std::chrono::steady_clock::time_point playback_anchor_wall_time;
  double playback_anchor_record_time = 0.0;
  double last_video_pos_frames = -1.0;
  double last_prev_dt = std::numeric_limits<double>::quiet_NaN();
  double last_next_dt = std::numeric_limits<double>::quiet_NaN();
  double last_process_time_s = 0.0;
  double smoothed_process_time_s = 0.0;
  bool last_frame_started_new_segment = false;

  auto reset_runtime_state = [&]() {
    tracker = std::make_unique<auto_aim::Tracker>(config_path, solver);
    aimer = std::make_unique<auto_aim::Aimer>(config_path);
    last_target = auto_aim::Target{};
    last_command = io::Command{};
    last_t = -1;
  };

  auto reset_playback_clock = [&](int frame_index) {
    frame_index = clamp_int(frame_index, start_frame, end_frame);
    playback_anchor_wall_time = std::chrono::steady_clock::now();
    playback_anchor_record_time = records[frame_index].t;
    playback_anchor_valid = true;
    playback_anchor_needs_reset = false;
  };

  auto process_current_frame = [&]() -> bool {
    if (current_frame > end_frame) return false;

    const auto frame_index = current_frame;
    const auto & record = records[frame_index];
    auto process_start = std::chrono::steady_clock::now();

    last_frame_started_new_segment = false;
    last_prev_dt = std::numeric_limits<double>::quiet_NaN();
    if (frame_index > start_frame) {
      last_prev_dt = record.t - records[frame_index - 1].t;
      if (!std::isfinite(last_prev_dt) || last_prev_dt <= 0.0 || last_prev_dt > kSegmentResetThresholdS) {
        reset_runtime_state();
        playback_anchor_valid = false;
        playback_anchor_needs_reset = true;
        last_frame_started_new_segment = true;
        tools::logger()->warn(
          "检测到时间跳变，frame {} dt={:.3f}s，已重置 Tracker/Aimer 与播放时钟", frame_index,
          last_prev_dt);
      }
    }

    last_next_dt =
      frame_index < end_frame ? records[frame_index + 1].t - record.t
                            : std::numeric_limits<double>::quiet_NaN();

    video.read(img);
    if (img.empty()) return false;
    last_video_pos_frames = video.get(cv::CAP_PROP_POS_FRAMES);

    auto timestamp = t0 + std::chrono::microseconds(static_cast<int64_t>(record.t * 1e6));

    /// 自瞄核心逻辑

    solver.set_R_gimbal2world({record.w, record.x, record.y, record.z});

    auto yolo_start = std::chrono::steady_clock::now();
    auto armors = yolo.detect(img, current_frame);

    auto tracker_start = std::chrono::steady_clock::now();
    auto targets = tracker->track(armors, timestamp);

    auto aimer_start = std::chrono::steady_clock::now();
    auto command = aimer->aim(targets, timestamp, 27, false);

    if (
      !targets.empty() && aimer->debug_aim_point.valid &&
      std::abs(command.yaw - last_command.yaw) * 57.3 < 2)
      command.shoot = true;

    if (command.control) last_command = command;
    /// 调试输出

    auto finish = std::chrono::steady_clock::now();
    last_process_time_s = std::max(0.0, tools::delta_time(finish, process_start));
    if (smoothed_process_time_s <= 0.0) {
      smoothed_process_time_s = last_process_time_s;
    } else {
      smoothed_process_time_s = 0.8 * smoothed_process_time_s + 0.2 * last_process_time_s;
    }

    tools::logger()->info(
      "[{}] yolo: {:.1f}ms, tracker: {:.1f}ms, aimer: {:.1f}ms", current_frame,
      tools::delta_time(tracker_start, yolo_start) * 1e3,
      tools::delta_time(aimer_start, tracker_start) * 1e3,
      tools::delta_time(finish, aimer_start) * 1e3);

    tools::draw_text(
      img,
      fmt::format(
        "command is {},{:.2f},{:.2f},shoot:{}", command.control, command.yaw * 57.3,
        command.pitch * 57.3, command.shoot),
      {10, 60}, {154, 50, 205});

    Eigen::Quaternion gimbal_q = {record.w, record.x, record.y, record.z};
    tools::draw_text(
      img,
      fmt::format(
        "gimbal yaw{:.2f}",
        (tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0) * 57.3)[0]),
      {10, 90}, {255, 255, 255});

    nlohmann::json data;

    // 装甲板原始观测数据
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      const auto & armor = armors.front();
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
      data["armor_center_x"] = armor.center_norm.x;
      data["armor_center_y"] = armor.center_norm.y;
    }

    Eigen::Quaternion q{record.w, record.x, record.y, record.z};
    auto yaw = tools::eulers(q, 2, 1, 0)[0];
    data["gimbal_yaw"] = yaw * 57.3;
    data["cmd_yaw"] = command.yaw * 57.3;
    data["shoot"] = command.shoot;

    if (!targets.empty()) {
      auto target = targets.front();

      if (last_t == -1) {
        last_target = target;
        last_t = record.t;
      }

      std::vector<Eigen::Vector4d> armor_xyza_list;

      // 当前帧target更新后
      armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // aimer瞄准位置
      auto aim_point = aimer->debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid) tools::draw_points(img, image_points, {0, 0, 255});

      // 观测器内部数据
      Eigen::VectorXd x = target.ekf_x();
      data["x"] = x[0];
      data["vx"] = x[1];
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4];
      data["vz"] = x[5];
      data["a"] = x[6] * 57.3;
      data["w"] = x[7];
      data["r"] = x[8];
      data["l"] = x[9];
      data["h"] = x[10];
      data["last_id"] = target.last_id;

      // 卡方检验数据
      data["residual_yaw"] = target.ekf().data.at("residual_yaw");
      data["residual_pitch"] = target.ekf().data.at("residual_pitch");
      data["residual_distance"] = target.ekf().data.at("residual_distance");
      data["residual_angle"] = target.ekf().data.at("residual_angle");
      data["nis"] = target.ekf().data.at("nis");
      data["nees"] = target.ekf().data.at("nees");
      data["nis_fail"] = target.ekf().data.at("nis_fail");
      data["nees_fail"] = target.ekf().data.at("nees_fail");
      data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");
    }

    plotter.plot(data);

    current_display_img = img.clone();
    displayed_frame = frame_index;
    sync_frame_trackbar(displayed_frame);
    current_frame = frame_index + 1;
    refresh_frame = false;
    return true;
  };

  auto seek_to_frame = [&](int target_frame) -> bool {
    target_frame = clamp_int(target_frame, start_frame, end_frame);
    video.set(cv::CAP_PROP_POS_FRAMES, target_frame);
    current_frame = target_frame;
    displayed_frame = target_frame - 1;
    reset_runtime_state();
    playback_anchor_valid = false;
    playback_anchor_needs_reset = true;
    sync_frame_trackbar(target_frame);
    refresh_frame = true;

    if (paused) return process_current_frame();
    return true;
  };

  auto wait_delay_ms = [&](int frame_index) {
    if (paused) return 30;

    if (frame_index < start_frame || frame_index > end_frame) {
      return std::max(1, static_cast<int>(std::lround(base_delay_ms / playback_control.playback_speed)));
    }

    if (!playback_anchor_valid || playback_anchor_needs_reset) return 1;

    auto expected_elapsed_s =
      (records[frame_index].t - playback_anchor_record_time) / playback_control.playback_speed;
    if (!std::isfinite(expected_elapsed_s)) {
      return std::max(1, static_cast<int>(std::lround(base_delay_ms / playback_control.playback_speed)));
    }

    auto actual_elapsed_s =
      tools::delta_time(std::chrono::steady_clock::now(), playback_anchor_wall_time);
    auto predicted_process_time_s =
      smoothed_process_time_s > 0.0 ? smoothed_process_time_s : last_process_time_s;
    auto remaining_ms = static_cast<int>(
      std::lround((expected_elapsed_s - actual_elapsed_s - predicted_process_time_s) * 1e3));
    return clamp_int(remaining_ms, 1, kMaxPlaybackWaitMs);
  };

  tools::logger()->info(
    "离线调试控制: 空格暂停/继续, +/- 调速, a/d 回退/前进1秒, z/c 回退/前进5秒, 拖动frame滑条跳转, q退出；播放节奏按 txt 时间戳调度，跨段自动重置");

  while (!exiter.exit()) {
    if ((refresh_frame || !paused) && !process_current_frame()) break;

    if (current_display_img.empty()) continue;

    auto display_img = current_display_img.clone();
    auto current_record_index = displayed_frame >= 0 ? displayed_frame : start_frame;
    auto next_frame_index =
      current_frame <= end_frame ? current_frame : clamp_int(current_frame, start_frame, end_frame);

    if (playback_anchor_needs_reset && !paused) {
      reset_playback_clock(current_record_index);
    }

    auto current_time = records[current_record_index].t;
    auto scheduled_frame_index = paused ? current_record_index : next_frame_index;
    auto scheduled_time = records[scheduled_frame_index].t;
    auto current_wait_ms = wait_delay_ms(scheduled_frame_index);
    auto dt_prev_text = std::isfinite(last_prev_dt) ? fmt::format("{:.3f}", last_prev_dt) : "-";
    auto dt_next_text = std::isfinite(last_next_dt) ? fmt::format("{:.3f}", last_next_dt) : "-";
    auto decoded_frame_hint =
      last_video_pos_frames >= 0.0 ? std::max(0, static_cast<int>(std::lround(last_video_pos_frames)) - 1) : -1;

    tools::draw_text(
      display_img,
      fmt::format(
        "frame:{}/{}  time:{:.3f}s  next:{}@{:.3f}s  speed:{:.2f}x  wait:{}ms  proc:{:.1f}ms  state:{}",
        displayed_frame, end_frame, current_time, scheduled_frame_index, scheduled_time,
        playback_control.playback_speed, current_wait_ms, smoothed_process_time_s * 1e3,
        paused ? "paused" : "playing"),
      {10, 30}, {0, 255, 255});
    tools::draw_text(
      display_img,
      fmt::format(
        "video_pos:{:.1f} decoded:{}  dt_prev:{}s  dt_next:{}s  segment:{}",
        last_video_pos_frames, decoded_frame_hint, dt_prev_text, dt_next_text,
        last_frame_started_new_segment ? "reset" : "cont"),
      {10, 120}, {0, 255, 255});
    tools::draw_text(
      display_img,
      "space:pause/resume +/-:speed a/d:-/+1s z/c:-/+5s drag frame bar to seek q:quit",
      {10, 150}, {0, 255, 255});

    cv::resize(display_img, display_img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow(window_name, display_img);

    auto key = cv::waitKey(current_wait_ms);

    auto ui_frame_pos = cv::getTrackbarPos(frame_trackbar_name, window_name);
    if (ui_frame_pos != frame_trackbar_pos) {
      playback_control.requested_frame = ui_frame_pos;
      playback_control.seek_from_slider = true;
    }

    auto ui_speed_pos = cv::getTrackbarPos(speed_trackbar_name, window_name);
    if (ui_speed_pos != speed_trackbar_pos) {
      playback_control.playback_speed = std::max(ui_speed_pos, 1) / 100.0;
      speed_trackbar_pos = ui_speed_pos;
      if (!paused && displayed_frame >= start_frame) reset_playback_clock(displayed_frame);
    }

    if (playback_control.requested_frame >= 0) {
      if (playback_control.seek_from_slider) paused = true;
      auto seek_ok = seek_to_frame(playback_control.requested_frame);
      playback_control.requested_frame = -1;
      playback_control.seek_from_slider = false;
      if (!seek_ok) break;
      continue;
    }

    if (key < 0) continue;
    if (key == 'q') break;

    if (key == ' ') {
      paused = !paused;
      if (!paused && displayed_frame >= start_frame) reset_playback_clock(displayed_frame);
      continue;
    }

    if (key == '+' || key == '=') {
      sync_speed_trackbar(playback_control.playback_speed * 1.25);
      if (!paused && displayed_frame >= start_frame) reset_playback_clock(displayed_frame);
      continue;
    }

    if (key == '-') {
      sync_speed_trackbar(playback_control.playback_speed / 1.25);
      if (!paused && displayed_frame >= start_frame) reset_playback_clock(displayed_frame);
      continue;
    }

    auto jump_small = std::max(1, static_cast<int>(std::lround(video_fps > 1e-3 ? video_fps : 30.0)));
    auto jump_large = jump_small * 5;

    if (key == 'a') {
      if (!seek_to_frame(displayed_frame - jump_small)) break;
      continue;
    }

    if (key == 'd') {
      if (!seek_to_frame(displayed_frame + jump_small)) break;
      continue;
    }

    if (key == 'z') {
      if (!seek_to_frame(displayed_frame - jump_large)) break;
      continue;
    }

    if (key == 'c') {
      if (!seek_to_frame(displayed_frame + jump_large)) break;
      continue;
    }
  }

  return 0;
}
