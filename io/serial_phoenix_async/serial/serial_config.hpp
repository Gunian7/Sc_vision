#pragma once

#include <asio.hpp>

#include <string>

namespace session::serial {
/**
 * @brief 串口收发方向模式。
 */
enum class DuplexMode {
    full_duplex,
    half_duplex,
};

/**
 * @brief 串口默认读取缓冲区大小。
 */
constexpr std::size_t default_read_buffer_size = 1024;

/**
 * @brief Asio 串口奇偶校验类型别名。
 */
using Parity      = asio::serial_port_base::parity;
/**
 * @brief Asio 串口停止位类型别名。
 */
using StopBits    = asio::serial_port_base::stop_bits;
/**
 * @brief Asio 串口流控类型别名。
 */
using FlowControl = asio::serial_port_base::flow_control;

/**
 * @brief 串口配置项。
 */
struct SerialPortConfig {
    /** @brief 串口设备路径，例如 `/dev/ttyACM0`。 */
    std::string device_path = "/dev/ttyACM0";

    /** @brief 波特率。 */
    unsigned int baud_rate         = 115200;
    /** @brief 字符位数。 */
    unsigned int character_size    = 8;
    /** @brief 奇偶校验模式。 */
    Parity::type parity            = Parity::none;
    /** @brief 停止位模式。 */
    StopBits::type stop_bits       = StopBits::one;
    /** @brief 流控模式。 */
    FlowControl::type flow_control = FlowControl::none;

    /** @brief 串口方向模式。 */
    DuplexMode duplex_mode       = DuplexMode::full_duplex;
    /** @brief 读取缓冲区大小。 */
    std::size_t read_buffer_size = default_read_buffer_size;
};
} // namespace session::serial
