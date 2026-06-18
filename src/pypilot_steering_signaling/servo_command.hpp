#pragma once

#include <stdint.h>

namespace pypilot_steering_signaling {

enum class ServoCommandMode {
    Stop,
    Position,
    Speed,
    RawPwm
};

enum class ServoCommandSource {
    None,
    Autopilot,
    Manual,
    Remote
};

struct ServoCommand {
    ServoCommandMode mode;
    float value;
    uint64_t timestamp_us;
    bool enabled;

    ServoCommand()
        : mode(ServoCommandMode::Stop), value(0.0f), timestamp_us(0), enabled(false) {}

    ServoCommand(ServoCommandMode command_mode,
                 float command_value,
                 uint64_t time_us,
                 bool enabled_value = true)
        : mode(command_mode), value(command_value), timestamp_us(time_us), enabled(enabled_value) {}
};

struct ServoCommandRequest {
    ServoCommand command;
    ServoCommandSource source;
    bool valid;

    ServoCommandRequest()
        : command(), source(ServoCommandSource::None), valid(false) {}

    ServoCommandRequest(const ServoCommand& command_value,
                        ServoCommandSource source_value,
                        bool valid_value = true)
        : command(command_value), source(source_value), valid(valid_value) {}
};

inline ServoCommandRequest servo_command_request_none(uint64_t now_us) {
    return ServoCommandRequest(ServoCommand(ServoCommandMode::Stop, 0.0f, now_us, false),
                               ServoCommandSource::None,
                               false);
}

inline bool servo_value_is_finite(float value) {
    return value == value && value > -3.4028234e38f && value < 3.4028234e38f;
}

inline float servo_abs(float value) {
    return value < 0.0f ? -value : value;
}

inline float servo_clamp(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

inline ServoCommand servo_stop_command(uint64_t now_us) {
    return ServoCommand(ServoCommandMode::Stop, 0.0f, now_us, false);
}

inline bool servo_command_is_stop(const ServoCommand& command) {
    return command.mode == ServoCommandMode::Stop || !command.enabled;
}

inline bool servo_command_value_valid(const ServoCommand& command) {
    return servo_value_is_finite(command.value);
}

inline bool servo_command_is_stale(const ServoCommand& command,
                                   uint64_t now_us,
                                   uint64_t timeout_us) {
    if (!command.enabled) return true;
    if (command.timestamp_us == 0) return true;
    if (now_us < command.timestamp_us) return false;
    return (now_us - command.timestamp_us) > timeout_us;
}

inline ServoCommandRequest select_servo_command_request(const ServoCommandRequest& autopilot,
                                                        const ServoCommandRequest& manual,
                                                        uint64_t now_us) {
    if (manual.valid) return manual;
    if (autopilot.valid) return autopilot;
    return servo_command_request_none(now_us);
}

}  // namespace pypilot_steering_signaling
