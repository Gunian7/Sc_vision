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
#include "io/mcu_mode.hpp"
#include "io/socketcan.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

#include "io/serial_phoenix/include/serial.hpp"

namespace io {
// 哨兵专有
enum ShootMode { left_shoot, right_shoot, both_shoot };
const std::vector<std::string> SHOOT_MODES = { "left_shoot", "right_shoot", "both_shoot" };

#pragma pack(1)

typedef struct Autoaim_s {
    float yaw;
    float pitch;
    uint8_t enemy_team_color;
    uint8_t mode; 
    //  mode: 0-idle, 1-auto_aim, 2-small_buff, 3-big_buff, 4-outpost
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

    serial_phoenix::Serial serial_;
    std::vector<uint8_t> read_buffer_;
    std::vector<uint8_t> write_buffer_;
};

} // namespace io

#endif // IO__CBOARD_HPP