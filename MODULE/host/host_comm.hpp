#pragma once

#include "frame_parser.hpp"

#include <type_traits>

namespace host_comm {

/// @brief 按消息类型收发原协议；同一实例须串行访问，回调不得重入 process_buffer()。
/// @tparam Messages 提供 Frame、to_tuple() 和 from_tuple() 的消息类型，ID 须互不相同。
template <typename... Messages> class HostComm {
public:
    using Parser = frame::FrameParser<typename Messages::Frame...>;
    using Callbacks = std::tuple<void (*)(const Messages&, void*)...>;

    /// @brief 回调顺序与消息类型顺序一致，上下文由调用方管理。
    explicit HostComm(Callbacks callbacks = {}, void* context = nullptr)
        : m_callbacks(callbacks), m_context(context), m_parser({&dispatch<Messages>...}, this) {}

    HostComm(const HostComm&) = delete;
    HostComm& operator=(const HostComm&) = delete;
    HostComm(HostComm&&) = delete;
    HostComm& operator=(HostComm&&) = delete;

    /// @brief 输入任意分段字节流，返回本次通过校验的帧数。
    std::size_t process_buffer(std::span<const std::uint8_t> bytes) {
        return m_parser.feed_and_parse(bytes);
    }

    /// @brief 丢弃半帧，供接收溢出或 USB 会话切换后调用。
    void reset() noexcept { m_parser.reset(); }

    /// @brief 使用消息布局生成独立帧，可交给 USB 或其他字节流端口发送。
    template <typename Message>
    [[nodiscard]] static constexpr auto build_frame(const Message& message) noexcept {
        static_assert((std::is_same_v<Message, Messages> || ...), "消息未列入 HostComm 类型表");
        return Message::Frame::serialize(message.to_tuple());
    }

private:
    Callbacks m_callbacks;
    void* m_context;
    Parser m_parser;

    template <typename Message>
    static void dispatch(Parser& parser, const typename Message::Frame::DataTuple& data) {
        auto& self = *static_cast<HostComm*>(parser.m_callback_context);
        const auto callback = std::get<void (*)(const Message&, void*)>(self.m_callbacks);
        if (callback != nullptr)
            callback(Message::from_tuple(data), self.m_context);
    }
};

} // namespace host_comm
