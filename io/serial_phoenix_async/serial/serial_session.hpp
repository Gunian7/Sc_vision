#pragma once

#include "../core/async_transport_session.hpp"
#include "serial_config.hpp"

#include <asio/serial_port.hpp>

#include <memory>
#include <vector>

namespace session::serial {

/**
 * @brief 串口会话，提供字节流式异步收发能力。
 */
class SerialSession:
    public AsyncTransportSession<SerialSession, std::vector<char>, std::vector<char>>,
    public std::enable_shared_from_this<SerialSession> {
public:
    /** @brief 读取回调类型。 */
    using Base        = AsyncTransportSession<SerialSession, std::vector<char>, std::vector<char>>;
    using ReadHandler = Base::ReadHandler;
    using DuplexMode   = session::serial::DuplexMode;

    /**
     * @brief 构造一个串口会话。
     * @param io_context Boost.Asio 运行上下文。
     */
    explicit SerialSession(asio::io_context& io_context):
        Base(io_context),
        socket_(io_context) {}

    /**
     * @brief 创建并打开串口会话。
     * @param io_context Boost.Asio 运行上下文。
     * @param config 串口配置。
     * @param ec 若提供，则返回打开结果。
     */
    static std::shared_ptr<SerialSession> create(
        asio::io_context& io_context,
        const SerialPortConfig& config,
        asio::error_code* ec = nullptr
    ) {
        auto session = std::make_shared<SerialSession>(io_context);
        if (ec) {
            *ec = session->open(config);
        } else {
            asio::error_code ignored_ec = session->open(config);
            (void)ignored_ec;
        }

        return session;
    }

    /**
     * @brief 打开串口并应用配置。
     */
    asio::error_code open(const SerialPortConfig& config) {
        asio::error_code ec;
        socket_.open(config.device_path, ec);
        if (ec) {
            return ec;
        }

        ec = apply_config(config);
        if (ec) {
            asio::error_code close_ec;
            socket_.close(close_ec);
        }

        return ec;
    }

    /**
     * @brief 将串口参数应用到当前会话。
     */
    asio::error_code apply_config(const SerialPortConfig& config) {
        auto& port = socket_;

        asio::error_code ec;
        port.set_option(asio::serial_port_base::baud_rate(config.baud_rate), ec);
        if (ec) {
            return ec;
        }

        port.set_option(asio::serial_port_base::character_size(config.character_size), ec);
        if (ec) {
            return ec;
        }

        port.set_option(asio::serial_port_base::parity(config.parity), ec);
        if (ec) {
            return ec;
        }

        port.set_option(asio::serial_port_base::stop_bits(config.stop_bits), ec);
        if (ec) {
            return ec;
        }

        port.set_option(asio::serial_port_base::flow_control(config.flow_control), ec);
        if (ec) {
            return ec;
        }

        duplex_mode_ = config.duplex_mode;
        read_buffer_.resize(config.read_buffer_size);
        return {};
    }

    /** @brief 判断串口是否已打开。 */
    bool is_open() const {
        return socket_.is_open();
    }

    /** @brief 获取底层串口对象。 */
    asio::serial_port& socket() {
        return socket_;
    }

    /** @brief 获取底层串口对象（只读）。 */
    const asio::serial_port& socket() const {
        return socket_;
    }

    /**
     * @brief 设置串口方向模式。
     */
    void set_duplex_mode(DuplexMode duplex_mode) {
        asio::post(strand(), [this, self = shared_from_this(), duplex_mode]() {
            duplex_mode_ = duplex_mode;
            schedule_operations();
        });
    }

    /**
     * @brief 设置读取缓冲区大小。
     */
    void set_read_buffer_size(std::size_t read_buffer_size) {
        asio::post(strand(), [this, self = shared_from_this(), read_buffer_size]() {
            if (is_read_in_progress() || read_buffer_size == 0U) {
                return;
            }

            read_buffer_.resize(read_buffer_size);
        });
    }

    /** @brief 判断是否允许启动下一次读取。 */
    bool can_start_read() const {
        if (!is_read_enabled() || is_read_in_progress() || is_closed()) {
            return false;
        }

        if (duplex_mode_ == DuplexMode::half_duplex) {
            return !is_write_in_progress() && !has_pending_writes();
        }

        return true;
    }

    /** @brief 判断是否允许启动下一次写入。 */
    bool can_start_write() const {
        if (is_write_in_progress() || !has_pending_writes() || is_closed()) {
            return false;
        }

        if (duplex_mode_ == DuplexMode::half_duplex) {
            return !is_read_in_progress();
        }

        return true;
    }

    /** @brief 启动一次异步读取。 */
    void do_read() {
        mark_read_started();

        socket_.async_read_some(
            asio::buffer(read_buffer_),
            asio::bind_executor(
                strand(),
                [this, self = shared_from_this()](asio::error_code ec, std::size_t bytes_transferred) {
                    mark_read_finished();

                    if (ec) {
                        notify_read(ec, {});
                        close();
                        return;
                    }

                    std::vector<char> data(
                        read_buffer_.begin(),
                        read_buffer_.begin() + static_cast<std::vector<char>::difference_type>(bytes_transferred)
                    );
                    notify_read({}, std::move(data));
                    schedule_operations();
                }
            )
        );
    }

    /** @brief 启动一次异步写入。 */
    void do_write() {
        auto buffer = current_write();
        if (!buffer) {
            return;
        }

        mark_write_started();

        asio::async_write(
            socket_,
            asio::buffer(*buffer),
            asio::bind_executor(
                strand(),
                [this, self = shared_from_this()](asio::error_code ec, std::size_t /*bytes_transferred*/) {
                    mark_write_finished();

                    if (ec) {
                        close();
                        return;
                    }

                    consume_write();
                    schedule_operations();
                }
            )
        );
    }

    /** @brief 关闭底层串口资源。 */
    void close_transport() {
        asio::error_code ec;
        socket_.close(ec);
    }

private:
    asio::serial_port socket_;
    DuplexMode duplex_mode_ = DuplexMode::full_duplex;
    std::vector<char> read_buffer_ = std::vector<char>(default_read_buffer_size);
};

} // namespace session::serial
