#pragma once

#include "host_frame.hpp"

namespace host_comm {

struct VisionPose {
    static constexpr std::uint8_t kId = 0x01;
    using Frame = HostFrame<kId, frame::Field<float>, frame::Field<float>, frame::Field<float>,
                            frame::Field<std::uint32_t>, frame::Field<std::uint32_t>>;

    float x{};
    float y{};
    float yaw{};
    std::uint32_t timestamp_ms{};
    std::uint32_t seq{};

    /// @brief 按线路字段顺序提供数据，不依赖结构体内存布局。
    [[nodiscard]] constexpr Frame::DataTuple to_tuple() const noexcept {
        return {x, y, yaw, timestamp_ms, seq};
    }

    /// @brief 从已验证的数据字段创建消息。
    [[nodiscard]] static constexpr VisionPose from_tuple(const Frame::DataTuple& data) noexcept {
        return std::apply([](auto... values) { return VisionPose{values...}; }, data);
    }
};

static_assert(VisionPose::Frame::size() == 28);

} // namespace host_comm
