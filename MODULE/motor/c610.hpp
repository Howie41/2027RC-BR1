#pragma once

#include "can_bus.hpp"
#include "motor_base.hpp"
#include "frame_codec.hpp"
#include "asserter.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <tuple>

namespace motor_detail {

using C610Feedback = frame::Frame<
    frame::Field<std::uint16_t, std::endian::big>,
    frame::Field<std::int16_t, std::endian::big>,
    frame::Field<std::int16_t, std::endian::big>,
    frame::Field<std::uint8_t>, frame::Field<std::uint8_t>>;

using C610MotorOutputs = frame::Frame<
    frame::Field<std::int16_t, std::endian::big>,
    frame::Field<std::int16_t, std::endian::big>,
    frame::Field<std::int16_t, std::endian::big>,
    frame::Field<std::int16_t, std::endian::big>>;

} // namespace motor_detail

class C610Motor;

/** @brief C610 协议设备：统一管理 1～8 号电机，按控制组构建控制命令并分发反馈。 */
class C610 : public CanDevice {
public:
    static constexpr std::uint8_t kMinMotorId = 1;
    static constexpr std::uint8_t kMaxMotorId = 8;
    static constexpr std::size_t kMotorCount = kMaxMotorId - kMinMotorId + 1;
    static constexpr std::size_t kMotorsPerFrame = 4;
    static constexpr std::size_t kControlFrameCount = kMotorCount / kMotorsPerFrame;
    static constexpr std::size_t kCanFrameSize = motor_detail::C610MotorOutputs::size();
    static constexpr std::uint32_t kFeedbackIdBase = 0x200U;
    static constexpr std::uint32_t kTxIdMotors1To4 = 0x200U;
    static constexpr std::uint32_t kTxIdMotors5To8 = 0x1FFU;
    static constexpr float kDefaultReductionRatio = 36.0f;
    static constexpr float kMaxCurrentMa = 10000.0f;
    static constexpr float kMaxProtocolOutput = 10000.0f;

    /** @brief 选择设备负责的固定电机编号组。 */
    enum class ControlGroup : std::uint8_t { Motors1To4, Motors5To8, All };

    /** @brief 固定控制组；同总线混用协议时必须选择互不重叠的组。 */
    explicit C610(CanBus& bus, ControlGroup group = ControlGroup::All)
        : CanDevice(bus, kFeedbackIdBase + (group == ControlGroup::Motors5To8 ? 5U : 1U), false),
          group_(group) {
        RUNTIME_ASSERT(group == ControlGroup::All || group == ControlGroup::Motors1To4 ||
                       group == ControlGroup::Motors5To8);
    }

    /** @brief 电机构造时登记，按电调 ID 保存指针；同指针重复登记成功，拒绝重复 ID。 */
    [[nodiscard]] bool register_motor(C610Motor* motor);

    /** @brief 接收所选控制组的标准帧反馈范围，不要求对应电机已经绑定。 */
    [[nodiscard]] bool match_rx(const FDCAN_RxHeaderTypeDef& header) const override {
        return header.IdType == FDCAN_STANDARD_ID &&
               header.Identifier >= kFeedbackIdBase + firstMotorId() &&
               header.Identifier <= kFeedbackIdBase + lastMotorId();
    }

    /** @brief 根据接收 ID 分发并更新对应电机反馈，未绑定电机的反馈直接忽略。 */
    void on_rx(std::uint32_t rx_id, const std::uint8_t data[kCanFrameSize], std::uint8_t len) override;

    /** @brief 仅发送所选组；未绑定或禁用的槽位置零，全部发送成功返回 true。 */
    [[nodiscard]] bool build_tx(CanBus& bus) override;

private:
    [[nodiscard]] std::uint8_t firstMotorId() const {
        return group_ == ControlGroup::Motors5To8 ? 5 : 1;
    }
    [[nodiscard]] std::uint8_t lastMotorId() const {
        return group_ == ControlGroup::Motors1To4 ? 4 : 8;
    }
    const ControlGroup group_;
    static constexpr std::array<std::uint32_t, kControlFrameCount> kControlTxIds{
        kTxIdMotors1To4, kTxIdMotors5To8};
    static constexpr std::int32_t kEncoderCountsPerTurn = 8192;
    static constexpr std::int32_t kEncoderHalfTurn = kEncoderCountsPerTurn / 2;
    static constexpr float kDegreesPerTurn = 360.0f;
    static constexpr float kEncoderAngleRatio = static_cast<float>(kEncoderCountsPerTurn) / kDegreesPerTurn;
    static constexpr float kRpmToRadiansPerSecond = 0.104719755f;
    std::array<C610Motor*, kMotorCount> m_motors{};
};

/** @brief 单个 C610 电机的状态、配置及输出目标，不继承 CanDevice，不负责 CAN 编解码。 */
class C610Motor : public MotorBase {
public:
    struct OutputCommand {
        float current_ma{0}; // 电流目标，单位 mA，由外部 PID 等控制器计算。
    };

    /** @brief 接收所属 C610 和电调 ID（1～8），构造末尾自动登记，不保存 C610 或 Bus 引用。 */
    C610Motor(C610& controller, std::uint8_t motor_id) : m_id(motor_id) {
        RUNTIME_ASSERT(motor_id >= C610::kMinMotorId && motor_id <= C610::kMaxMotorId);
        set_motor_reduction(C610::kDefaultReductionRatio);
        // 协议未提供温度及转矩工程单位，不能把默认零值当作有效测量。
        m_torque = std::numeric_limits<float>::quiet_NaN();
        m_temperature = std::numeric_limits<float>::quiet_NaN();
        const bool registered = controller.register_motor(this);
        RUNTIME_ASSERT(registered);
    }

    C610Motor(const C610Motor&) = delete;
    C610Motor& operator=(const C610Motor&) = delete;
    C610Motor(C610Motor&&) = delete;
    C610Motor& operator=(C610Motor&&) = delete;

    /** @brief 获取电调设置的 ID（1～8），不是 CAN 反馈 ID。 */
    [[nodiscard]] std::uint8_t id() const noexcept { return m_id; }

    /** @brief 设置减速比与毫安电流上限，不重置反馈。 */
    void init(float reduction_ratio = C610::kDefaultReductionRatio,
              float max_output = C610::kMaxCurrentMa) {
        RUNTIME_ASSERT(std::isfinite(reduction_ratio) && reduction_ratio > 0);
        RUNTIME_ASSERT(std::isfinite(max_output) && max_output >= 0 && max_output <= C610::kMaxCurrentMa);
        set_motor_reduction(reduction_ratio);
        m_max_output_ma = max_output;
        m_output_command.current_ma = std::clamp(m_output_command.current_ma, -max_output, max_output);
    }

    /** @brief 允许输出；实际控制帧由 C610 在总线 periodic 时构建。 */
    void enable() noexcept override { m_enabled = true; }

    /** @brief 禁止输出并清空电流目标，停机仍需继续周期发送零电流。 */
    void disable() noexcept override {
        m_enabled = false;
        m_output_command = {};
    }

    /** @brief 保存限幅后的电流目标；非有限值返回 false 且保留原命令。 */
    [[nodiscard]] bool set_output_command(const OutputCommand& command) noexcept {
        if (!std::isfinite(command.current_ma)) return false;
        m_output_command.current_ma = std::clamp(command.current_ma, -m_max_output_ma, m_max_output_ma);
        return true;
    }

    /** @brief 获取当前保存的电流目标，单位 mA。 */
    [[nodiscard]] const OutputCommand& get_output_command() const noexcept { return m_output_command; }

private:
    friend class C610;

    const std::uint8_t m_id;
    OutputCommand m_output_command{};
    float m_max_output_ma{C610::kMaxCurrentMa};
    bool m_is_encoder_init{false};
    std::uint16_t m_encoder_offset{0};
    std::uint16_t m_last_encoder{0};
    std::int32_t m_round_count{0};
};

inline bool C610::register_motor(C610Motor* motor) {
    if (motor == nullptr) return false;
    if (motor->id() < firstMotorId() || motor->id() > lastMotorId()) return false;
    if (motor->id() < kMinMotorId || motor->id() > kMaxMotorId) return false;
    const auto index = static_cast<std::size_t>(motor->id() - kMinMotorId);
    auto& slot = m_motors[index];
    if (slot == motor) return true;
    if (slot != nullptr) return false;
    slot = motor;
    return true;
}

inline bool C610::build_tx(CanBus& bus) {
    std::array<std::array<std::int16_t, kMotorsPerFrame>, kControlFrameCount> outputs{};
    for (std::size_t i = 0; i < m_motors.size(); ++i) {
        const auto* motor = m_motors[i];
        if (motor == nullptr || !motor->is_enabled()) continue;
        outputs[i / kMotorsPerFrame][i % kMotorsPerFrame] = static_cast<std::int16_t>(
            motor->get_output_command().current_ma * kMaxProtocolOutput / kMaxCurrentMa);
    }

    bool success = true;
    for (std::size_t i = 0; i < kControlFrameCount; ++i) {
        if ((i == 0 && firstMotorId() == 5) || (i == 1 && lastMotorId() == 4)) continue;
        const auto& slots = outputs[i];
        const auto bytes = motor_detail::C610MotorOutputs::serialize(
            std::tuple{slots[0], slots[1], slots[2], slots[3]});
        CanBus::ClassicPack pack{};
        pack.m_id = kControlTxIds[i];
        pack.m_type = CanBus::Type::Standard;
        std::copy(bytes.begin(), bytes.end(), pack.m_data);
        if (bus.add_can_msg(pack) != 0) success = false;
    }
    return success;
}

inline void C610::on_rx(std::uint32_t rx_id, const std::uint8_t data[kCanFrameSize], std::uint8_t len) {
    if (rx_id < kFeedbackIdBase + firstMotorId() || rx_id > kFeedbackIdBase + lastMotorId() ||
        len < motor_detail::C610Feedback::size()) return;
    auto* motor = m_motors[rx_id - kFeedbackIdBase - kMinMotorId];
    if (motor == nullptr) return;
    const auto decoded = motor_detail::C610Feedback::deserialize(
        std::span<const std::uint8_t>{data, motor_detail::C610Feedback::size()});
    if (!decoded) return;
    const auto [encoder, speed, torque, reserved1, reserved2] = *decoded;
    (void)reserved1;
    (void)reserved2;
    if (encoder >= kEncoderCountsPerTurn || !std::isfinite(motor->m_reduction_ratio) ||
        motor->m_reduction_ratio <= 0) return;

    if (motor->m_is_encoder_init) {
        const auto delta = static_cast<std::int32_t>(encoder) - motor->m_last_encoder;
        if (delta < -kEncoderHalfTurn) ++motor->m_round_count;
        else if (delta > kEncoderHalfTurn) --motor->m_round_count;
        const std::int32_t total_encoder = motor->m_round_count * kEncoderCountsPerTurn +
                                           encoder - motor->m_encoder_offset;
        motor->m_raw_sum_pos = static_cast<float>(total_encoder) / kEncoderAngleRatio;
        motor->m_sum_pos = motor->m_raw_sum_pos / motor->m_reduction_ratio;
        motor->m_single_pos = std::fmod(motor->m_sum_pos, kDegreesPerTurn);
        if (motor->m_single_pos < 0.0f) motor->m_single_pos += kDegreesPerTurn;
    } else {
        motor->m_encoder_offset = encoder;
        motor->m_is_encoder_init = true;
    }
    motor->m_raw_single_pos = static_cast<float>(encoder) / kEncoderAngleRatio;
    motor->m_last_encoder = encoder;
    motor->m_raw_speed = static_cast<float>(speed) * kRpmToRadiansPerSecond;
    motor->m_speed = motor->m_raw_speed / motor->m_reduction_ratio;
    // 协议定义为实际输出转矩，但未给出工程单位；只保留原始值，不换算到力矩字段。
    motor->m_raw_torque = static_cast<float>(torque);
    // 末尾两个反馈字节为空字段，力矩及温度工程值保持构造时的 NaN。
    const auto tick = rtos::is_irq_context() ? xTaskGetTickCountFromISR() : rtos::now_tick();
    motor->m_last_feedback_ms = static_cast<std::uint32_t>(tick * portTICK_PERIOD_MS);
}
