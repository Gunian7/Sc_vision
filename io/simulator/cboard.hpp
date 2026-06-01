#ifndef IO__SIMULATOR_CBOARD_HPP
#define IO__SIMULATOR_CBOARD_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <string>

#include "io/cboard.hpp"
#include "io/command.hpp"

namespace io {

/**
 * @brief A CBoard subclass that reads IMU/gimbal data from the shared memory
 *        written by the RoboMaster Simulator instead of physical serial/CAN.
 *
 * Usage:
 *   SimulatorCBoard cboard;   // opens shared memory automatically
 *   Eigen::Quaterniond q = cboard.imu_at(now);  // reads from shm
 *   cboard.send(cmd);        // no-op in simulation
 */
class SimulatorCBoard : public CBoard {
public:
    SimulatorCBoard();
    ~SimulatorCBoard() override;

    /// Override: read from shared memory, ignores timestamp.
    Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp) override;

    /// Override: no-op in simulation.
    void send(Command /*command*/) override {}

private:
    int shm_fd_ = -1;
    void * shm_ptr_ = nullptr;
    std::size_t map_size_ = 0;
    bool valid_ = false;

    void open_shm();
};

} // namespace io

#endif // IO__SIMULATOR_CBOARD_HPP
