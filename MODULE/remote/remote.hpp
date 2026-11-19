#pragma once

#include "rtos_utility.hpp"

#include <algorithm>
#include <array>
#include <cstdint>

namespace remote {

/// @brief i-Bus 遥控器消息中心：由接收任务喂入，其它任务读取摇杆与开关。
class Remote {
public:
    static constexpr float kDeadband = 0.08f;
    static constexpr std::uint32_t kTimeoutMs = 100;

    /// @brief 喂入一帧解析结果，刷新在线时间戳。
    void feed(const std::array<std::uint16_t, 14>& channels) {
        channels_ = channels;
        last_feed_ms_ = rtos::now_ms();
        ++frames_;
    }

    /// @brief 已成功解析的帧数，供调试器观察。
    std::uint32_t frames() const { return frames_; }

    /// @brief 通道 1000~2000 → -1.0~1.0（死区 + 限幅）。
    float axis(int index) const {
        const float a = (static_cast<float>(channels_[index]) - 1500.0f) / 500.0f;
        if (a > -kDeadband && a < kDeadband) return 0.0f;
        return std::clamp(a, -1.0f, 1.0f);
    }

    float right_x() const { return axis(0); }
    float right_y() const { return axis(1); }
    float left_y() const { return axis(2); }
    float left_x() const { return axis(3); }

    std::uint16_t switch_a() const { return channels_[4]; }
    std::uint16_t switch_b() const { return channels_[5]; }
    std::uint16_t switch_c() const { return channels_[6]; }
    std::uint16_t switch_d() const { return channels_[7]; }

    /// @brief 通用通道值（0 基下标）。
    std::uint16_t channel(int index) const { return channels_[index]; }

    /// @brief 启用/禁用遥控器（全自动模式可禁用）。
    void set_enabled(bool enabled) { enabled_ = enabled; }

    /// @brief 启用且最近 kTimeoutMs 内收到过帧则在线。
    bool online() const {
        return enabled_ &&
               static_cast<std::uint32_t>(rtos::now_ms() - last_feed_ms_) < kTimeoutMs;
    }

private:
    // 掉线默认值：摇杆回中、开关归上。
    std::array<std::uint16_t, 14> channels_{
        1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
    std::uint32_t last_feed_ms_{0};
    std::uint32_t frames_{0};
    bool enabled_{true};
};

}  // namespace remote
