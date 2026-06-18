#pragma once

#include <stdint.h>
#include "rudder_calibration.hpp"
#include "servo_command.hpp"

namespace pypilot_steering_signaling {

struct ServoSafetyConfig {
    uint64_t command_timeout_us;
    float max_position_abs;
    float max_speed_abs;
    float max_raw_pwm_abs;
    float rudder_min_deg;
    float rudder_max_deg;
    float rudder_limit_margin_deg;
    bool enforce_rudder_limits;

    ServoSafetyConfig()
        : command_timeout_us(1000000ULL),
          max_position_abs(1.0f),
          max_speed_abs(1.0f),
          max_raw_pwm_abs(1.0f),
          rudder_min_deg(-45.0f),
          rudder_max_deg(45.0f),
          rudder_limit_margin_deg(0.5f),
          enforce_rudder_limits(true) {}
};

enum class ServoSafetyReason {
    None,
    Disabled,
    InvalidValue,
    StaleCommand,
    RudderInvalid,
    RudderPortLimit,
    RudderStarboardLimit
};

struct ServoSafetyResult {
    ServoCommand command;
    ServoSafetyReason reason;
    bool allowed;

    ServoSafetyResult()
        : command(), reason(ServoSafetyReason::None), allowed(false) {}

    ServoSafetyResult(const ServoCommand& output,
                      ServoSafetyReason safety_reason,
                      bool is_allowed)
        : command(output), reason(safety_reason), allowed(is_allowed) {}
};

inline float servo_limit_for_mode(ServoCommandMode mode, const ServoSafetyConfig& config) {
    switch (mode) {
        case ServoCommandMode::Position: return config.max_position_abs;
        case ServoCommandMode::Speed: return config.max_speed_abs;
        case ServoCommandMode::RawPwm: return config.max_raw_pwm_abs;
        case ServoCommandMode::Stop: default: return 0.0f;
    }
}

inline ServoCommand servo_clamped_command(const ServoCommand& command,
                                          const ServoSafetyConfig& config) {
    if (command.mode == ServoCommandMode::Stop) {
        return ServoCommand(ServoCommandMode::Stop, 0.0f, command.timestamp_us, false);
    }
    const float limit = servo_abs(servo_limit_for_mode(command.mode, config));
    ServoCommand out = command;
    out.value = servo_clamp(command.value, -limit, limit);
    return out;
}

inline bool servo_command_drives_port(const ServoCommand& command) {
    return command.enabled && command.value < 0.0f;
}

inline bool servo_command_drives_starboard(const ServoCommand& command) {
    return command.enabled && command.value > 0.0f;
}

inline ServoSafetyResult apply_servo_safety(const ServoCommand& command,
                                            const RudderAngle& rudder,
                                            uint64_t now_us,
                                            const ServoSafetyConfig& config = ServoSafetyConfig()) {
    if (!command.enabled || command.mode == ServoCommandMode::Stop) {
        return ServoSafetyResult(servo_stop_command(now_us), ServoSafetyReason::Disabled, false);
    }
    if (!servo_command_value_valid(command)) {
        return ServoSafetyResult(servo_stop_command(now_us), ServoSafetyReason::InvalidValue, false);
    }
    if (servo_command_is_stale(command, now_us, config.command_timeout_us)) {
        return ServoSafetyResult(servo_stop_command(now_us), ServoSafetyReason::StaleCommand, false);
    }

    ServoCommand out = servo_clamped_command(command, config);

    if (config.enforce_rudder_limits) {
        if (!rudder.valid) {
            return ServoSafetyResult(servo_stop_command(now_us), ServoSafetyReason::RudderInvalid, false);
        }
        const float port_limit = config.rudder_min_deg + config.rudder_limit_margin_deg;
        const float starboard_limit = config.rudder_max_deg - config.rudder_limit_margin_deg;
        if (rudder.angle_deg <= port_limit && servo_command_drives_port(out)) {
            return ServoSafetyResult(servo_stop_command(now_us), ServoSafetyReason::RudderPortLimit, false);
        }
        if (rudder.angle_deg >= starboard_limit && servo_command_drives_starboard(out)) {
            return ServoSafetyResult(servo_stop_command(now_us), ServoSafetyReason::RudderStarboardLimit, false);
        }
    }

    return ServoSafetyResult(out, ServoSafetyReason::None, true);
}

}  // namespace pypilot_steering_signaling
