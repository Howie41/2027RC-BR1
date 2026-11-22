#pragma once

#include "rtos_utility.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <numbers>

namespace imu {

/// @brief 帧类型。
enum class FrameType : std::uint8_t { None, Gyro, Angle };

/// @brief HWT901B 解析后的数据（仅航向与角速度，机体坐标系 z 轴）。
struct Data {
    float yaw_imu{0.0f};    // 航向 (rad)
    float omega_imu{0.0f};  // 角速度 z (rad/s)
    bool is_online{true};  // 最近 kTimeoutMs 内收到过帧
};

/// @brief HWT901B 流式解析器：逐字节喂入，收齐 11 字节并校验后更新对应字段。
class Parser {
public:
    static constexpr std::uint32_t kTimeoutMs = 50;

    /// @brief 喂入一个字节；解析出完整一帧时更新 data 并返回帧类型。刷新在线时间戳。
    FrameType feed(std::uint8_t byte, Data& data);

private:
    static constexpr std::size_t kFrameLength = 11;
    static constexpr float kDegToRad = std::numbers::pi_v<float> / 180.0f;
    static constexpr float kGyroScale = 2000.0f / 32768.0f * kDegToRad; // LSB → rad/s
    static constexpr float kAngleScale = 180.0f / 32768.0f * kDegToRad; // LSB → rad

    std::array<std::uint8_t, kFrameLength> buffer_{};
    std::size_t length_{0};

    std::uint32_t last_feed_ms_{0};
    
};

inline FrameType Parser::feed(std::uint8_t byte, Data& data) {
    last_feed_ms_ = rtos::now_ms();
    data.is_online = static_cast<std::uint32_t>(rtos::now_ms() - last_feed_ms_) < kTimeoutMs;

    if (length_ == 0 && byte != 0x55) return FrameType::None;
    buffer_[length_++] = byte;
    if (length_ < kFrameLength) return FrameType::None;
    length_ = 0;

    // 校验：前 10 字节之和的低 8 位。
    std::uint8_t sum = 0;
    for (std::size_t i = 0; i < kFrameLength - 1; ++i) {
        sum = static_cast<std::uint8_t>(sum + buffer_[i]);
    }
    if (sum != buffer_[kFrameLength - 1]) return FrameType::None;

    const auto i16 = [this](std::size_t i) {
        return static_cast<std::int16_t>(buffer_[i] | (buffer_[i + 1] << 8));
    };

    switch (buffer_[1]) {
        case 0x52:  // 角速度
            data.omega_imu = static_cast<float>(i16(6)) * kGyroScale;
            return FrameType::Gyro;
        case 0x53:  // 角度
            data.yaw_imu = static_cast<float>(i16(6)) * kAngleScale;
            return FrameType::Angle;
        default:
            return FrameType::None;
    }

}

}  // namespace imu
