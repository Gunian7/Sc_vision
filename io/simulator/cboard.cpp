#include "cboard.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>

#include "io/simulator/simulator.hpp"

namespace io
{

static constexpr const char * SHM_NAME = "/simulator_frame";

SimulatorCBoard::SimulatorCBoard()
    : CBoard()  // use the protected default ctor
{
    open_shm();
}

SimulatorCBoard::~SimulatorCBoard()
{
    if (shm_ptr_ != nullptr && shm_ptr_ != MAP_FAILED) {
        ::munmap(shm_ptr_, map_size_);
    }
    if (shm_fd_ >= 0) {
        ::close(shm_fd_);
    }
}

void SimulatorCBoard::open_shm()
{
    // Open shared memory (read-only)
    shm_fd_ = shm_open(SHM_NAME, O_RDONLY, 0);
    if (shm_fd_ < 0) {
        throw std::runtime_error(
            "SimulatorCBoard: failed to open shared memory " + std::string(SHM_NAME) +
            ". Is the simulator running?");
    }

    // Read header to get dimensions (needed for map_size)
    uint32_t hdr[2];
    if (::read(shm_fd_, &hdr, sizeof(hdr)) != static_cast<ssize_t>(sizeof(hdr))) {
        ::close(shm_fd_);
        throw std::runtime_error("SimulatorCBoard: failed to read shm header");
    }
    uint32_t width = hdr[0];
    uint32_t height = hdr[1];

    if (width == 0 || height == 0 || width > 4096 || height > 4096) {
        ::close(shm_fd_);
        throw std::runtime_error(
            "SimulatorCBoard: invalid dimensions in shm header (" +
            std::to_string(width) + "x" + std::to_string(height) + ")");
    }

    map_size_ = ShmLayout::OFFSET_PIXEL_DATA +
                static_cast<std::size_t>(width) * height * 3;

    // Mmap (read-only)
    shm_ptr_ = ::mmap(nullptr, map_size_, PROT_READ, MAP_SHARED, shm_fd_, 0);
    if (shm_ptr_ == MAP_FAILED) {
        ::close(shm_fd_);
        throw std::runtime_error("SimulatorCBoard: mmap failed");
    }

    valid_ = true;
}

Eigen::Quaterniond SimulatorCBoard::imu_at(std::chrono::steady_clock::time_point /*timestamp*/)
{
    if (!valid_) {
        return Eigen::Quaterniond::Identity();
    }

    // Read yaw/pitch/roll (radians) from shared memory
    float yaw   = read_gimbal_yaw(shm_ptr_);
    float pitch = read_gimbal_pitch(shm_ptr_);
    float roll  = read_gimbal_roll(shm_ptr_);

    // Construct quaternion from Euler angles (YXZ order, matching the simulator)
    Eigen::Quaterniond q =
        Eigen::AngleAxisd(static_cast<double>(roll),  Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(static_cast<double>(pitch), Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(static_cast<double>(yaw),   Eigen::Vector3d::UnitY());

    // Update base-class public members so standard.cpp can read them
    bullet_speed = static_cast<double>(read_bullet_speed(shm_ptr_));
    int32_t mode_val = read_mode(shm_ptr_);
    switch (mode_val) {
        case 0:  mode = Mode::idle;      break;
        case 1:  mode = Mode::auto_aim;  break;
        case 2:  mode = Mode::small_buff; break;
        case 3:  mode = Mode::big_buff;  break;
        case 4:  mode = Mode::outpost;   break;
        default: mode = Mode::idle;      break;
    }

    return q;
}

} // namespace io
