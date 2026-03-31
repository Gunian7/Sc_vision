#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "io/command.hpp"
#include "io/socketcan.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"


#include "io/serial_phoenix_async/serial/serial_session.hpp"

namespace io {
enum Mode { idle, auto_aim, small_buff, big_buff, outpost };
const std::vector<std::string> MODES = { "idle", "auto_aim", "small_buff", "big_buff", "outpost" };

// 哨兵专有
enum ShootMode { left_shoot, right_shoot, both_shoot };
const std::vector<std::string> SHOOT_MODES = { "left_shoot", "right_shoot", "both_shoot" };

#pragma pack(1)

typedef struct Autoaim_s {
    float yaw;
    float pitch;
    uint8_t enemy_team_color;
    uint8_t mode;
    uint8_t rune_flag;
    float bullet_speed;
} Autoaim;

typedef struct GimbalControl_s {
    char find_bools;
    float yaw;
    float pitch;
    float yaw_vel;
    float pitch_vel;
    char fire_bools;
} GimbalControl;

typedef struct Message_phoenix_s {
    uint8_t header;
    uint8_t type;
    uint8_t data[29];
    uint8_t tail;
    
    // 转换为 vector<char>
    std::vector<char> toBytes() const {
        std::vector<char> bytes(sizeof(*this));
        std::memcpy(bytes.data(), this, sizeof(*this));
        return bytes;
    }
    
    // 从 vector<char> 还原（静态方法）
    static Message_phoenix_s fromBytes(const std::vector<char>& bytes) {
        Message_phoenix_s msg;
        if (bytes.size() >= sizeof(msg)) {
            std::memcpy(&msg, bytes.data(), sizeof(msg));
        }
        return msg;
    }
    
    // 从字节流填充（成员方法）
    bool loadFromBytes(const std::vector<char>& bytes) {
        if (bytes.size() < sizeof(*this)) {
            return false;
        }
        std::memcpy(this, bytes.data(), sizeof(*this));
        return true;
    }
    
    // 验证消息有效性
    bool isValid() const {
        return header == 0xAA && tail == 0x55;
    }
    
} Message_phoenix;

#pragma pack()

class CBoard {
public:
    double bullet_speed;
    Mode mode;
    ShootMode shoot_mode;
    double ft_angle; //无人机专有

    CBoard(const std::string& config_path);

    // 启动接收线程
    void start();

    Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

    // 上发下
    void send(Command command);

private:
    struct IMUData {
        Eigen::Quaterniond q;
        std::chrono::steady_clock::time_point timestamp;
    };

    tools::ThreadSafeQueue<IMUData> queue_; // 必须在can_之前初始化，否则存在死锁的可能
    // SocketCAN can_;
    IMUData data_ahead_;
    IMUData data_behind_;

    int quaternion_canid_, bullet_speed_canid_, send_canid_;

    // Phoenix 下位机上报的 yaw/pitch 单位；默认按 rad 解析。
    // 可在 yaml 里配置：phoenix_angle_unit: deg|rad
    bool phoenix_angles_in_degrees_ = false;
    double imu_yaw_offset_rad_ = 0.0;
    double imu_pitch_offset_rad_ = 0.0;
    bool use_default_bullet_speed_ = false;
    double default_bullet_speed_ = 21.0;
    bool cboard_debug_log_ = false;
    std::chrono::steady_clock::time_point last_debug_log_time_;

    std::string findFirstACMDevice();

    // std::string read_yaml(const std::string& config_path);

    void read_fun_1(Message_phoenix& msg);

    asio::io_context io_context_;
    session::serial::SerialSession serial_;
    std::vector<uint8_t> read_buffer_;
    std::vector<uint8_t> write_buffer_;
};

} // namespace io

#endif // IO__CBOARD_HPP