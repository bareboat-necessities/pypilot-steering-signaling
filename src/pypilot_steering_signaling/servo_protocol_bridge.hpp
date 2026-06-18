#pragma once

#include <stdint.h>
#include <pypilot_servo_protocol.hpp>
#include "servo_safety.hpp"

namespace pypilot_steering_signaling {

inline float servo_protocol_normalized_command(const ServoCommand& command) {
    if (!command.enabled || command.mode == ServoCommandMode::Stop) return 0.0f;
    return servo_clamp(command.value, -1.0f, 1.0f);
}

inline pypilot_servo_protocol::RawPacket servo_command_to_protocol_packet(const ServoCommand& command) {
    return pypilot_servo_protocol::encode_packet(
        pypilot_servo_protocol::COMMAND_CODE,
        pypilot_servo_protocol::encode_command(servo_protocol_normalized_command(command)));
}

}  // namespace pypilot_steering_signaling
