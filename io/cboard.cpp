#include "cboard.hpp"

#include <cstdint>
#include <iostream>

namespace io {
CBoard::CBoard(const std::string& config_path):
    bullet_speed(21.0),
    yaw_vel(0.0),
    pitch_vel(0.0),
    mode(Mode::idle),
    shoot_mode(ShootMode::left_shoot),
    queue_(5000) {
    auto yaml          = YAML::LoadFile(config_path);

    if (yaml["bullet_speed"]) {
        default_bullet_speed_ = yaml["bullet_speed"].as<double>();
        bullet_speed = default_bullet_speed_;
    }

    if (yaml["use_default_bullet_speed"]) {
        use_default_bullet_speed_ = yaml["use_default_bullet_speed"].as<bool>();
    }


    if (yaml["phoenix_angle_unit"]) {
        auto unit = yaml["phoenix_angle_unit"].as<std::string>();
        for (auto& c: unit) c = static_cast<char>(std::tolower(c));
        phoenix_angles_in_degrees_ = (unit == "deg" || unit == "degree" || unit == "degrees");
    }

    if (yaml["imu_yaw_offset_deg"]) {
        imu_yaw_offset_rad_ = yaml["imu_yaw_offset_deg"].as<double>() * M_PI / 180.0;
    }
    if (yaml["imu_pitch_offset_deg"]) {
        imu_pitch_offset_rad_ = yaml["imu_pitch_offset_deg"].as<double>() * M_PI / 180.0;
    }
    if (yaml["imu_yaw_offset_rad"]) {
        imu_yaw_offset_rad_ = yaml["imu_yaw_offset_rad"].as<double>();
    }
    if (yaml["imu_pitch_offset_rad"]) {
        imu_pitch_offset_rad_ = yaml["imu_pitch_offset_rad"].as<double>();
    }
    if (yaml["cboard_debug_log"]) {
        cboard_debug_log_ = yaml["cboard_debug_log"].as<bool>();
    }
    last_debug_log_time_ = std::chrono::steady_clock::now();

    tools::logger()->info("[Cboard] Waiting for q...");

    this->read_buffer_.resize(32);
    this->write_buffer_.resize(32);
    this->serial_ = serial_phoenix::Serial();

    auto code = this->serial_.open(findFirstACMDevice(), nullptr, 32);
    if (!code) {
        tools::logger()->warn("[Cboard] Serial port not opened: {}", static_cast<int>(code.code()));
    }
    this->start();
    // Use default values to prevent blocking startup if serial is silent
    data_ahead_ = { Eigen::Quaterniond::Identity(), std::chrono::steady_clock::now() };
    data_behind_ = data_ahead_;
    tools::logger()->info("[Cboard] Opened.");
}

void CBoard::start() {
    std::thread Link_thread([this] {
        while (true) {
            this->serial_.read(this->read_buffer_);
            uint8_t type = this->read_buffer_[1];

            std::vector<uint8_t> buffer(29);

            std::memcpy(buffer.data(), this->read_buffer_.data() + 2, 29);
            if (type == 0xb0) {
                this->read_fun_1(*(Message_phoenix*)this->read_buffer_.data());
            }
        }
    });
    Link_thread.detach();
}

Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp) {
    if (data_behind_.timestamp < timestamp)
        data_ahead_ = data_behind_;

    while (true) {
        queue_.pop(data_behind_);
        if (data_behind_.timestamp > timestamp)
            break;
        data_ahead_ = data_behind_;
    }

    Eigen::Quaterniond q_a             = data_ahead_.q.normalized();
    Eigen::Quaterniond q_b             = data_behind_.q.normalized();
    auto t_a                           = data_ahead_.timestamp;
    auto t_b                           = data_behind_.timestamp;
    auto t_c                           = timestamp;
    std::chrono::duration<double> t_ab = t_b - t_a;
    std::chrono::duration<double> t_ac = t_c - t_a;

    // 四元数插值
    auto k                 = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

    return q_c;
}

// 存在多写竞争，但是目前只有单线程写入需求
// TODO: 改进为线程安全
void CBoard::send(Command command) {
    Message_phoenix msg;
    msg.header = 's';
    msg.type   = 0xA0;
    GimbalControl msg_body;
    msg_body.find_bools = command.control ? 49 : 48; // '1' or '0'
    msg_body.yaw        = static_cast<float>(command.yaw);
    msg_body.pitch      = static_cast<float>(command.pitch);
    msg_body.yaw_vel    = static_cast<float>(command.yaw_vel);
    msg_body.pitch_vel  = static_cast<float>(command.pitch_vel);
    msg_body.fire_bools = command.shoot ? 49 : 48;   // '1' or '0'
    std::memcpy(msg.data, &msg_body, sizeof(GimbalControl));
    msg.tail = 'e';

    auto code = this->serial_.write(std::move(msg));
    static int fail_count = 0;
    if (!code) {
        int err_code = static_cast<int>(code.code());
        tools::logger()->warn("Serial write failed: {}", err_code);
        
        // Error code 40 (WRITE_FAIL) usually implies a disconnected or broken device.
        // If it happens continuously, we should exit to let watchdog restart the process.
        fail_count++;
        if (fail_count > 5 || err_code == 40) {
            tools::logger()->error("Serial write failed too many times or critical error. Exiting...");
            std::exit(1); 
        }
    } else {
        fail_count = 0;
    }
}

void CBoard::read_fun_1(Message_phoenix& msg) {
    auto timestamp = std::chrono::steady_clock::now();

    Autoaim_s data = reinterpret_cast<Autoaim_s&>(msg.data);
    if (data.mode < MODES.size()) {
        mode = normalize_mcu_mode_for_auto_aim(static_cast<Mode>(data.mode));
    } else if (cboard_debug_log_) {
        tools::logger()->warn(
            "[CBoard] Invalid mode from MCU: {} (keep current {})",
            static_cast<int>(data.mode), io::MODES[mode]);
    }

    double raw_bullet_speed = data.bullet_speed;
    if (use_default_bullet_speed_) {
        this->bullet_speed = default_bullet_speed_;
    } else if (std::isfinite(raw_bullet_speed) && raw_bullet_speed >= 1.0) {
        this->bullet_speed = raw_bullet_speed;
    } else if (cboard_debug_log_) {
        tools::logger()->warn(
            "[CBoard] Invalid bullet speed from MCU: raw={:.3f}. Keep fallback/current v={:.3f}",
            raw_bullet_speed, this->bullet_speed);
    }
    double raw_yaw = data.yaw;
    double raw_pitch = data.pitch;
    double raw_yaw_vel = data.yaw_vel;
    double raw_pitch_vel = data.pitch_vel;
    double yaw = raw_yaw;
    double pitch = raw_pitch;
    double yaw_velocity = raw_yaw_vel;
    double pitch_velocity = raw_pitch_vel;

    // // 记录原始读取值，便于排查数据格式/协议问题
    // tools::logger()->debug("[CBoard] raw angles: yaw={}, pitch={}, degrees_flag={}", yaw, pitch,
    //                       phoenix_angles_in_degrees_);

    // 如果为度单位则转为弧度
    if (phoenix_angles_in_degrees_) {
        constexpr double kDeg2Rad = M_PI / 180.0;
        yaw *= kDeg2Rad;
        pitch *= kDeg2Rad;
        yaw_velocity *= kDeg2Rad;
        pitch_velocity *= kDeg2Rad;
    }

    yaw += imu_yaw_offset_rad_;
    pitch += imu_pitch_offset_rad_;
    yaw_vel = yaw_velocity;
    pitch_vel = pitch_velocity;

    // 合法性检查：排除 NaN/Inf 或极端错误值，避免产生非法四元数
    if (!std::isfinite(yaw) || !std::isfinite(pitch) || std::abs(yaw) > 1e4 || std::abs(pitch) > 1e4) {
        tools::logger()->error("[CBoard] Invalid IMU angles, skipping sample: yaw={}, pitch={}", yaw, pitch);
        return;
    }

    // std::cout << "Yaw: " << yaw << ", Pitch: " << pitch << std::endl;
    Eigen::AngleAxisd yaw_aa(yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitch_aa(pitch, Eigen::Vector3d::UnitY());
    Eigen::Quaterniond q = (yaw_aa * pitch_aa).normalized();
    queue_.push({ q, timestamp });

    if (cboard_debug_log_) {
        auto dt = std::chrono::duration<double>(timestamp - last_debug_log_time_).count();
        if (dt > 0.5) {
            tools::logger()->info(
                "[CBoard] raw(yaw={:.3f}, pitch={:.3f}, yaw_vel={:.3f}, pitch_vel={:.3f}, v={:.3f}) unit={} -> parsed(yaw={:.3f}rad, pitch={:.3f}rad, yaw_vel={:.3f}, pitch_vel={:.3f}, v={:.3f}, source={})",
                raw_yaw, raw_pitch, raw_yaw_vel, raw_pitch_vel, raw_bullet_speed,
                phoenix_angles_in_degrees_ ? "deg" : "rad",
                yaw, pitch, yaw_vel, pitch_vel, this->bullet_speed,
                use_default_bullet_speed_ ? "yaml_default" : "mcu_or_fallback");
            last_debug_log_time_ = timestamp;
        }
    }
}

std::string CBoard::findFirstACMDevice() {
    const std::string dev_path = "/dev/";
    for (const auto& entry: std::filesystem::directory_iterator(dev_path)) {
        if (entry.path().filename().string().find("ttyACM") != std::string::npos) {
            std::cout << "find ACM device: " << entry.path().string() << std::endl;
            return entry.path().string();
        }
    }
    throw std::runtime_error("No /dev/ttyACM* device found.");
}
} // namespace io
