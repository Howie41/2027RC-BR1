#pragma once

#include "asserter.hpp"
#include "can_bus.hpp"
#include "dji_motor_frames.hpp"
#include "motor_base.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <tuple>

/** @brief 大疆 C610 / C620 电调型号。同一条总线上的电机按这个字段选择电流换算和反馈解释。 */
enum class C6x0Model : std::uint8_t {
    C610 = 0,
    C620 = 1,
};

/** @brief 某个型号的默认减速比、电流上限和协议满量程。 */
struct C6x0Spec {
    float default_reduction{1.0f};
    float max_current_ma{0.0f};
    float max_protocol_output{0.0f};
};

/** @brief 返回型号对应的协议参数。电流换算分母是这里的满量程，不是 init() 的输出上限。 */
[[nodiscard]] constexpr C6x0Spec c6x0_spec(C6x0Model model) noexcept {
    if (model == C6x0Model::C610) {
        return {.default_reduction = 36.0f, .max_current_ma = 10000.0f, .max_protocol_output = 10000.0f};
    }
    return {.default_reduction = 19.0f, .max_current_ma = 20000.0f, .max_protocol_output = 16384.0f};
}

class C6x0Motor;

/**
 * @brief 一条 CAN 上的 C610 / C620 管理器。
 * @note 每周期只发送一帧 0x200 和一帧 0x1FF。槽位按电机自己的型号换算电流。
 *       同一条总线只创建一个 C6x0Bus，电调 ID 1～8 不能重复。
 */
class C6x0Bus : public CanDevice {
public:
    static constexpr std::uint8_t kMinMotorId = 1;
    static constexpr std::uint8_t kMaxMotorId = 8;
    static constexpr std::size_t kMotorCount = kMaxMotorId - kMinMotorId + 1;
    static constexpr std::size_t kMotorsPerFrame = 4;
    static constexpr std::size_t kControlFrameCount = kMotorCount / kMotorsPerFrame;
    static constexpr std::uint32_t kFeedbackIdBase = 0x200U;
    static constexpr std::uint32_t kTxIdMotors1To4 = 0x200U;
    static constexpr std::uint32_t kTxIdMotors5To8 = 0x1FFU;

    /** @brief 以标准帧 0x201 登记到尚未启动的总线，不操作硬件。 */
    explicit C6x0Bus(CanBus& bus) : CanDevice(bus, kFeedbackIdBase + kMinMotorId, false) {}

    /** @brief 按电调 ID 登记电机。同指针重复登记成功，ID 已被占用时返回 false。 */
    [[nodiscard]] bool register_motor(C6x0Motor* motor);

    /** @brief 匹配 1～8 号电调的标准帧反馈 ID。 */
    [[nodiscard]] bool match_rx(const FDCAN_RxHeaderTypeDef& header) const override {
        return header.IdType == FDCAN_STANDARD_ID &&
               header.Identifier >= kFeedbackIdBase + kMinMotorId &&
               header.Identifier <= kFeedbackIdBase + kMaxMotorId;
    }

    /** @brief 把反馈交给对应槽位。C620 使用第 7 字节温度，C610 使用第 8 字节错误码。 */
    void on_rx(std::uint32_t rx_id, const std::uint8_t data[8], std::uint8_t len) override;

    /** @brief 提交两组电流命令。未绑定或禁用的槽位填 0，两帧都成功才返回 true。 */
    [[nodiscard]] bool build_tx(CanBus& bus) override;

private:
    static constexpr std::array<std::uint32_t, kControlFrameCount> kControlTxIds{
        kTxIdMotors1To4, kTxIdMotors5To8};

    std::array<C6x0Motor*, kMotorCount> m_motors{};
};

/**
 * @brief 一台 C610 或 C620 电机。
 * @note 不负责 CAN 编解码。电流目标单位是 mA，协议换算由型号决定。
 *       C610 的 get_current_temperature() 和 get_current_torque() 返回 NaN。
 */
class C6x0Motor : public MotorBase {
public:
    struct OutputCommand {
        float current_ma{0}; // 电流目标，单位 mA。
    };

    /** @brief 登记到总线。model 决定换算和反馈解释，motor_id 是电调拨码 1～8。 */
    C6x0Motor(C6x0Bus& bus, C6x0Model model, std::uint8_t motor_id);

    C6x0Motor(const C6x0Motor&) = delete;
    C6x0Motor& operator=(const C6x0Motor&) = delete;
    C6x0Motor(C6x0Motor&&) = delete;
    C6x0Motor& operator=(C6x0Motor&&) = delete;

    /** @brief 获取电调拨码，不是 CAN 反馈 ID。 */
    [[nodiscard]] std::uint8_t id() const noexcept { return m_id; }

    /** @brief 获取构造时指定的型号。 */
    [[nodiscard]] C6x0Model model() const noexcept { return m_model; }

    /** @brief 设置减速比和毫安电流上限，不重置反馈。上限不能超过该型号的协议满量程。 */
    void init(float reduction_ratio, float max_output_ma);

    /** @brief 允许输出。控制帧由 C6x0Bus 在总线 periodic 时构建。 */
    void enable() noexcept override { m_enabled = true; }

    /** @brief 禁止输出并清空电流目标。停机后仍要继续周期发送零电流。 */
    void disable() noexcept override {
        m_enabled = false;
        m_output_command = {};
    }

    /** @brief 保存限幅后的电流目标。非有限值返回 false 且保留原命令。 */
    [[nodiscard]] bool set_output_command(const OutputCommand& command) noexcept {
        if (!std::isfinite(command.current_ma)) return false;
        m_output_command.current_ma = std::clamp(command.current_ma, -m_max_output_ma, m_max_output_ma);
        return true;
    }

    /** @brief 获取最近保存的电流目标。 */
    [[nodiscard]] const OutputCommand& get_output_command() const noexcept { return m_output_command; }

    /** @brief 获取反馈里的转矩电流原始值。 */
    [[nodiscard]] std::int16_t get_raw_current() const noexcept {
        return static_cast<std::int16_t>(m_raw_torque);
    }

    /** @brief 获取 C610 最近的错误码，0 表示无异常。C620 保持 0。 */
    [[nodiscard]] std::uint8_t get_error_code() const noexcept { return m_error_code; }

private:
    friend class C6x0Bus;

    /** @brief 按本机型号把毫安目标换成协议 int16。禁用时返回 0。 */
    [[nodiscard]] std::int16_t protocol_current() const noexcept {
        if (!m_enabled) return 0;
        const C6x0Spec spec = c6x0_spec(m_model);
        return static_cast<std::int16_t>(
            m_output_command.current_ma * spec.max_protocol_output / spec.max_current_ma);
    }

    /** @brief 写入一帧反馈。C620 第一帧只记编码器零点。 */
    void apply_feedback(std::uint16_t encoder, std::int16_t speed_rpm, std::int16_t current,
                        std::uint8_t byte6, std::uint8_t byte7);

    void apply_c620_feedback(std::uint16_t encoder, std::int16_t speed_rpm, std::int16_t current,
                             std::uint8_t temperature);
    void apply_c610_feedback(std::uint16_t encoder, std::int16_t speed_rpm, std::int16_t current,
                             std::uint8_t error_code);

    static constexpr std::int32_t kEncoderCountsPerTurn = 8192;
    static constexpr std::int32_t kEncoderHalfTurn = kEncoderCountsPerTurn / 2;
    static constexpr float kDegreesPerTurn = 360.0f;
    static constexpr float kEncoderAngleRatio = static_cast<float>(kEncoderCountsPerTurn) / kDegreesPerTurn;
    static constexpr float kDegreesPerCount = kDegreesPerTurn / kEncoderCountsPerTurn;
    static constexpr float kRpmToRadiansPerSecond = 0.104719755f;
    static constexpr float kCurrentToTorque = 0.2f;

    const std::uint8_t m_id;
    const C6x0Model m_model;
    OutputCommand m_output_command{};
    float m_max_output_ma;
    bool m_is_encoder_init{false};
    std::uint16_t m_encoder_offset{0};
    std::uint16_t m_last_encoder{0};
    std::int32_t m_round_count{0};
    bool m_has_encoder{false};
    std::int64_t m_encoder_total{0};
    std::uint8_t m_error_code{0};
};

inline C6x0Motor::C6x0Motor(C6x0Bus& bus, C6x0Model model, std::uint8_t motor_id)
    : m_id(motor_id), m_model(model), m_max_output_ma(c6x0_spec(model).max_current_ma) {
    RUNTIME_ASSERT(motor_id >= C6x0Bus::kMinMotorId && motor_id <= C6x0Bus::kMaxMotorId);
    set_motor_reduction(c6x0_spec(model).default_reduction);
    if (model == C6x0Model::C610) {
        m_torque = std::numeric_limits<float>::quiet_NaN();
        m_temperature = std::numeric_limits<float>::quiet_NaN();
    }
    const bool registered = bus.register_motor(this);
    RUNTIME_ASSERT(registered);
}

inline void C6x0Motor::init(float reduction_ratio, float max_output_ma) {
    const float max_allowed = c6x0_spec(m_model).max_current_ma;
    RUNTIME_ASSERT(std::isfinite(reduction_ratio) && reduction_ratio > 0.0f);
    RUNTIME_ASSERT(std::isfinite(max_output_ma) && max_output_ma >= 0.0f && max_output_ma <= max_allowed);
    set_motor_reduction(reduction_ratio);
    m_max_output_ma = max_output_ma;
    m_output_command.current_ma = std::clamp(m_output_command.current_ma, -max_output_ma, max_output_ma);
}

inline bool C6x0Bus::register_motor(C6x0Motor* motor) {
    if (motor == nullptr) return false;
    const auto motor_id = motor->id();
    if (motor_id < kMinMotorId || motor_id > kMaxMotorId) return false;
    auto& slot = m_motors[static_cast<std::size_t>(motor_id - kMinMotorId)];
    if (slot == motor) return true;
    if (slot != nullptr) return false;
    slot = motor;
    return true;
}

inline void C6x0Bus::on_rx(std::uint32_t rx_id, const std::uint8_t data[8], std::uint8_t len) {
    constexpr auto kFeedbackSize = motor_detail::DjiMotorFeedback::size();
    if (rx_id < kFeedbackIdBase + kMinMotorId || rx_id > kFeedbackIdBase + kMaxMotorId ||
        len < kFeedbackSize) return;
    auto* motor = m_motors[static_cast<std::size_t>(rx_id - kFeedbackIdBase - kMinMotorId)];
    if (motor == nullptr) return;
    const auto decoded = motor_detail::DjiMotorFeedback::deserialize(
        std::span<const std::uint8_t>{data, kFeedbackSize});
    if (!decoded) return;
    const auto [encoder, speed, current, byte6, byte7] = *decoded;
    motor->apply_feedback(encoder, speed, current, byte6, byte7);
}

inline bool C6x0Bus::build_tx(CanBus& bus) {
    std::array<std::array<std::int16_t, kMotorsPerFrame>, kControlFrameCount> outputs{};
    for (std::size_t i = 0; i < m_motors.size(); ++i) {
        const auto* motor = m_motors[i];
        if (motor == nullptr) continue;
        outputs[i / kMotorsPerFrame][i % kMotorsPerFrame] = motor->protocol_current();
    }

    bool success = true;
    for (std::size_t i = 0; i < kControlFrameCount; ++i) {
        const auto& slots = outputs[i];
        const auto bytes = motor_detail::DjiMotorOutputs::serialize(
            std::tuple{slots[0], slots[1], slots[2], slots[3]});
        CanBus::ClassicPack pack{};
        pack.m_id = kControlTxIds[i];
        pack.m_type = CanBus::Type::Standard;
        std::copy(bytes.begin(), bytes.end(), pack.m_data);
        if (bus.add_can_msg(pack) != 0) success = false;
    }
    return success;
}

inline void C6x0Motor::apply_feedback(std::uint16_t encoder, std::int16_t speed_rpm, std::int16_t current,
                                      std::uint8_t byte6, std::uint8_t byte7) {
    if (encoder >= kEncoderCountsPerTurn || !std::isfinite(m_reduction_ratio) ||
        m_reduction_ratio <= 0.0f) return;

    if (m_model == C6x0Model::C620) {
        apply_c620_feedback(encoder, speed_rpm, current, byte6);
    } else {
        apply_c610_feedback(encoder, speed_rpm, current, byte7);
    }
    const auto tick = rtos::is_irq_context() ? xTaskGetTickCountFromISR() : rtos::now_tick();
    m_last_feedback_ms = static_cast<std::uint32_t>(tick * portTICK_PERIOD_MS);
}

inline void C6x0Motor::apply_c620_feedback(std::uint16_t encoder, std::int16_t speed_rpm,
                                           std::int16_t current, std::uint8_t temperature) {
    if (m_is_encoder_init) {
        const auto delta = static_cast<std::int32_t>(encoder) - m_last_encoder;
        if (delta < -kEncoderHalfTurn) ++m_round_count;
        else if (delta > kEncoderHalfTurn) --m_round_count;
        const std::int32_t total_encoder = m_round_count * kEncoderCountsPerTurn +
                                           encoder - m_encoder_offset;
        m_raw_single_pos = static_cast<float>(encoder) / kEncoderAngleRatio;
        m_raw_sum_pos = static_cast<float>(total_encoder) / kEncoderAngleRatio;
        m_sum_pos = m_raw_sum_pos / m_reduction_ratio;
        m_single_pos = std::fmod(m_sum_pos, kDegreesPerTurn);
        if (m_single_pos < 0.0f) m_single_pos += kDegreesPerTurn;
    } else {
        m_encoder_offset = encoder;
        m_is_encoder_init = true;
    }
    m_last_encoder = encoder;
    m_raw_speed = static_cast<float>(speed_rpm) * kRpmToRadiansPerSecond;
    m_speed = m_raw_speed / m_reduction_ratio;
    // 保留原有反馈换算，不在此次结构调整中新增电流或力矩标定。
    m_raw_torque = static_cast<float>(current);
    m_torque = m_raw_torque * kCurrentToTorque * m_reduction_ratio;
    m_temperature = static_cast<float>(temperature);
}

inline void C6x0Motor::apply_c610_feedback(std::uint16_t encoder, std::int16_t speed_rpm,
                                           std::int16_t raw_current, std::uint8_t error_code) {
    if (m_has_encoder) {
        auto delta = static_cast<std::int32_t>(encoder) - m_last_encoder;
        if (delta < -kEncoderHalfTurn) delta += kEncoderCountsPerTurn;
        else if (delta > kEncoderHalfTurn) delta -= kEncoderCountsPerTurn;
        m_encoder_total += delta;
    }
    m_has_encoder = true;
    m_last_encoder = encoder;
    m_raw_single_pos = static_cast<float>(encoder) * kDegreesPerCount;
    m_raw_sum_pos = static_cast<float>(m_encoder_total) * kDegreesPerCount;
    m_sum_pos = m_raw_sum_pos / m_reduction_ratio;
    m_single_pos = std::fmod(m_sum_pos, kDegreesPerTurn);
    if (m_single_pos < 0.0f) m_single_pos += kDegreesPerTurn;
    m_raw_speed = static_cast<float>(speed_rpm) * kRpmToRadiansPerSecond;
    m_speed = m_raw_speed / m_reduction_ratio;
    m_raw_torque = static_cast<float>(raw_current);
    m_error_code = error_code;
}
