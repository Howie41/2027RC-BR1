#pragma once

#include "c6x0.hpp"
#include "pid_controller.hpp"
#include "static_string.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string_view>
#include <type_traits>

/**
 * @brief 带 PID 的 C6x0 电机。
 * @tparam EnablePositionLoop 启用时为位置速度双环，禁用时只有速度环。
 */
template <bool EnablePositionLoop = false>
class C6x0MotorWithPid : public C6x0Motor {
    struct EmptyPidController {};
    using OptionalPidController = std::conditional_t<
        EnablePositionLoop,
        PidController,
        EmptyPidController
    >;

    template <StaticString Suffix>
    static auto pid_name(std::string_view name) {
        constexpr auto capacity = PidController::kNameSize - 1;
        static_assert(Suffix.size() <= capacity);
        using Name = StaticString<capacity>;
        if (name.empty()) return Name{};

        auto length = std::min(name.size(), capacity - Suffix.size());
        // 截断时保留完整的 UTF-8 字符和环名称后缀。
        // TODO 把这一部分处理写进 StaticString
        while (length > 0 && length < name.size() && (static_cast<unsigned char>(name[length]) & 0xC0) == 0x80) {
            --length;
        }
        Name result{name.substr(0, length)};
        result.try_append(Suffix);
        return result;
    }

public:
    /** @brief 单环模式。 */
    C6x0MotorWithPid(
        C6x0Bus& bus,
        C6x0Model model,
        std::uint8_t motor_id,
        const PidController::Config& speed_config = {}
    ) requires (!EnablePositionLoop)
        : C6x0MotorWithPid(bus, model, std::string_view{}, motor_id, speed_config) {}

    C6x0MotorWithPid(
        C6x0Bus& bus,
        C6x0Model model,
        std::string_view motor_name,
        std::uint8_t motor_id,
        const PidController::Config& speed_config = {}
    ) requires (!EnablePositionLoop)
        : C6x0Motor(bus, model, motor_id),
          m_speed_pid(pid_name<"_speed">(motor_name).c_str(), speed_config) {}

    /** @brief 双环模式。位置目标单位是度，速度环目标单位是 rad/s。 */
    C6x0MotorWithPid(
        C6x0Bus& bus,
        C6x0Model model,
        std::uint8_t motor_id,
        const PidController::Config& speed_config = {},
        const PidController::Config& position_config = {}
    ) requires (EnablePositionLoop)
        : C6x0MotorWithPid(bus, model, std::string_view{}, motor_id, speed_config, position_config) {}

    C6x0MotorWithPid(
        C6x0Bus& bus,
        C6x0Model model,
        std::string_view motor_name,
        std::uint8_t motor_id,
        const PidController::Config& speed_config = {},
        const PidController::Config& position_config = {}
    ) requires (EnablePositionLoop)
        : C6x0Motor(bus, model, motor_id),
          m_speed_pid(pid_name<"_speed">(motor_name).c_str(), speed_config),
          m_position_pid(pid_name<"_pos">(motor_name).c_str(), position_config) {}

    /** @brief 禁用电机、清零电流并重置已启用的 PID。 */
    void disable() noexcept override {
        C6x0Motor::disable();
        reset_pid();
    }

    /** @brief 按当前目标计算电流。单环目标是 rad/s，双环目标是输出端角度。
     * @param speed_feedforward_rad_s 可选速度前馈，来自位置轨迹的一阶导数；本类不做规划。
     */
    void calculate_and_output(float target, float speed_feedforward_rad_s = 0.0F) {
        if (!is_enabled() || !is_online() || !std::isfinite(target) ||
            !std::isfinite(speed_feedforward_rad_s)) {
            clear_output();
            return;
        }

        const float speed = get_current_speed();

        float target_speed = target + speed_feedforward_rad_s;
        if constexpr (EnablePositionLoop) {
            const float position = get_current_sum_pos();
            const float speed_limit = std::abs(m_position_pid.config().max_out.load());
            target_speed = std::clamp(
                m_position_pid.calculate(position, target) + speed_feedforward_rad_s,
                -speed_limit, speed_limit);
        }

        const float current_ma = m_speed_pid.calculate(speed, target_speed);
        (void)set_output_command({.current_ma = current_ma});
    }

    PidController m_speed_pid;
    // no_unique_address 使得空类大小可以优化为 0 字节，并且共享地址
    [[no_unique_address]] OptionalPidController m_position_pid;

private:
    void reset_pid() noexcept {
        m_speed_pid.reset();
        if constexpr (EnablePositionLoop) m_position_pid.reset();
    }

    void clear_output() noexcept {
        (void)set_output_command({.current_ma = 0.0f});
        reset_pid();
    }
};
