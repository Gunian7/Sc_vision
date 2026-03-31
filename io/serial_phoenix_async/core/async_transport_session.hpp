#pragma once

#include <asio.hpp>

#include <deque>
#include <functional>
#include <memory>
#include <utility>

namespace session {
/**
 * @brief 通用异步通信核心，负责 strand 调度、发送队列和会话状态管理。
 *
 * @tparam Derived 具体协议会话类型，采用 CRTP 方式提供协议相关读写实现。
 * @tparam ReadValue 读取回调返回的数据类型。
 * @tparam WriteValue 发送队列中保存的数据类型，默认与读取类型一致。
 */
template<typename Derived, typename ReadValue, typename WriteValue = ReadValue>
class AsyncTransportSession {
public:
    /**
     * @brief 读取完成时触发的回调类型。
     */
    using ReadHandler = std::function<void(asio::error_code, ReadValue)>;

    /**
     * @brief 构造一个异步通信核心。
     * @param io_context Boost.Asio 运行上下文。
     */
    explicit AsyncTransportSession(asio::io_context& io_context):
        strand_(asio::make_strand(io_context)) {}

    /**
     * @brief 设置读取回调。
     * @param read_handler 读取完成后调用的处理函数。
     */
    void set_read_handler(ReadHandler read_handler) {
        asio::post(
            strand_,
            [this,
             self         = derived_shared_from_this(),
             read_handler = std::move(read_handler)]() mutable {
                read_handler_ = std::move(read_handler);
            }
        );
    }

    /**
     * @brief 启动异步读取调度。
     */
    void start_reading() {
        asio::post(strand_, [this, self = derived_shared_from_this()]() {
            read_enabled_ = true;
            schedule_operations();
        });
    }

    /**
     * @brief 停止后续读取调度。
     */
    void stop_reading() {
        asio::post(strand_, [this, self = derived_shared_from_this()]() {
            read_enabled_ = false;
        });
    }

    /**
     * @brief 发送一个待传输的对象。
     * @param message 协议层定义的发送数据。
     */
    void send(WriteValue message) {
        asio::post(
            strand_,
            [this, self = derived_shared_from_this(), message = std::move(message)]() mutable {
                write_queue_.push_back(std::make_shared<WriteValue>(std::move(message)));
                schedule_operations();
            }
        );
    }

    /**
     * @brief 关闭会话并清空待发送队列。
     */
    void close() {
        asio::post(strand_, [this, self = derived_shared_from_this()]() {
            if (closed_) {
                return;
            }

            read_enabled_ = false;
            write_queue_.clear();
            closed_ = true;
            static_cast<Derived*>(this)->close_transport();
        });
    }

protected:
    /**
     * @brief 获取当前 strand。
     */
    asio::strand<asio::io_context::executor_type>& strand() {
        return strand_;
    }

    const asio::strand<asio::io_context::executor_type>& strand() const {
        return strand_;
    }

    /**
     * @brief 判断是否允许继续读取。
     */
    bool is_read_enabled() const {
        return read_enabled_;
    }

    /**
     * @brief 判断读取操作是否正在进行中。
     */
    bool is_read_in_progress() const {
        return read_in_progress_;
    }

    /**
     * @brief 判断写入操作是否正在进行中。
     */
    bool is_write_in_progress() const {
        return write_in_progress_;
    }

    /**
     * @brief 判断会话是否已关闭。
     */
    bool is_closed() const {
        return closed_;
    }

    /**
     * @brief 判断是否还有待发送的数据。
     */
    bool has_pending_writes() const {
        return !write_queue_.empty();
    }

    /**
     * @brief 获取当前待写入的数据。
     */
    std::shared_ptr<WriteValue> current_write() {
        if (write_queue_.empty()) {
            return {};
        }

        return write_queue_.front();
    }

    /**
     * @brief 消耗当前写入队列头部元素。
     */
    void consume_write() {
        if (!write_queue_.empty()) {
            write_queue_.pop_front();
        }
    }

    /**
     * @brief 标记读取已开始。
     */
    void mark_read_started() {
        read_in_progress_ = true;
    }

    /**
     * @brief 标记读取已结束。
     */
    void mark_read_finished() {
        read_in_progress_ = false;
    }

    /**
     * @brief 标记写入已开始。
     */
    void mark_write_started() {
        write_in_progress_ = true;
    }

    /**
     * @brief 标记写入已结束。
     */
    void mark_write_finished() {
        write_in_progress_ = false;
    }

    /**
     * @brief 通知上层读取结果。
     */
    void notify_read(asio::error_code ec, ReadValue value) {
        if (read_handler_) {
            read_handler_(ec, std::move(value));
        }
    }

    /**
     * @brief 触发协议层的读写调度。
     */
    void schedule_operations() {
        auto* derived = static_cast<Derived*>(this);

        if (derived->can_start_write()) {
            derived->do_write();
        }

        if (derived->can_start_read()) {
            derived->do_read();
        }
    }

private:
    auto derived_shared_from_this() {
        return static_cast<Derived*>(this)->shared_from_this();
    }

    asio::strand<asio::io_context::executor_type> strand_;
    ReadHandler read_handler_;
    std::deque<std::shared_ptr<WriteValue>> write_queue_;

    bool read_enabled_      = false;
    bool read_in_progress_  = false;
    bool write_in_progress_ = false;
    bool closed_            = false;
};
} // namespace session
