#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <numbers>

namespace imu {

/// @brief 帧类型。
enum class FrameType : std::uint8_t { None, Accel, Gyro, Angle };

/// @brief HWT901B 解析后的数据（机体坐标系：x 右、y 前、z 上）。
struct Data {
    float ax{0.0f};  // 加速度 x (m/s²)
    float ay{0.0f};  // 加速度 y (m/s²)
    float az{0.0f};  // 加速度 z (m/s²)
    float wx{0.0f};  // 角速度 x (rad/s)
    float wy{0.0f};  // 角速度 y (rad/s)
    float wz{0.0f};  // 角速度 z (rad/s)
    float roll{0.0f};   // 横滚 (rad)
    float pitch{0.0f};  // 俯仰 (rad)
    float yaw{0.0f};    // 航向 (rad)
};

/// @brief HWT901B 流式解析器：逐字节喂入，收齐 11 字节并校验后更新对应字段。
class Parser {
public:
    /// @brief 喂入一个字节；解析出完整一帧时更新 data 并返回帧类型。
    FrameType parse(std::uint8_t byte, Data& data);

private:
    static constexpr std::size_t kFrameLength = 11;
    static constexpr float kDegToRad = std::numbers::pi_v<float> / 180.0f;
    static constexpr float kAccelScale = 16.0f / 32768.0f * 9.80665f;   // LSB → m/s²
    static constexpr float kGyroScale = 2000.0f / 32768.0f * kDegToRad; // LSB → rad/s
    static constexpr float kAngleScale = 180.0f / 32768.0f * kDegToRad; // LSB → rad

    std::array<std::uint8_t, kFrameLength> buffer_{};
    std::size_t length_{0};
};

inline FrameType Parser::parse(std::uint8_t byte, Data& data) {
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
    case 0x51:  // 加速度
        data.ax = static_cast<float>(i16(2)) * kAccelScale;
        data.ay = static_cast<float>(i16(4)) * kAccelScale;
        data.az = static_cast<float>(i16(6)) * kAccelScale;
        return FrameType::Accel;
    case 0x52:  // 角速度
        data.wx = static_cast<float>(i16(2)) * kGyroScale;
        data.wy = static_cast<float>(i16(4)) * kGyroScale;
        data.wz = static_cast<float>(i16(6)) * kGyroScale;
        return FrameType::Gyro;
    case 0x53:  // 角度
        data.roll = static_cast<float>(i16(2)) * kAngleScale;
        data.pitch = static_cast<float>(i16(4)) * kAngleScale;
        data.yaw = static_cast<float>(i16(6)) * kAngleScale;
        return FrameType::Angle;
    default:
        return FrameType::None;
    }
}

}  // namespace imu
