#pragma once

#include <stdint.h>
#include <pypilot_servo_protocol.hpp>
#include "servo_safety.hpp"

namespace pypilot_steering_signaling {

struct ServoProtocolCommandPacket {
    pypilot_servo_protocol::RawPacket raw_packet;
    ServoCommand effective_command;
    ServoSafetyReason safety_reason;
    bool allowed;

    ServoProtocolCommandPacket()
        : raw_packet({{0, 0, 0, 0}}), effective_command(),
          safety_reason(ServoSafetyReason::None), allowed(false) {}

    ServoProtocolCommandPacket(const pypilot_servo_protocol::RawPacket& packet_value,
                               const ServoCommand& command_value,
                               ServoSafetyReason reason_value,
                               bool allowed_value)
        : raw_packet(packet_value), effective_command(command_value),
          safety_reason(reason_value), allowed(allowed_value) {}
};

struct ServoFeedback {
    bool has_current_a;
    bool has_voltage_v;
    bool has_controller_temp_c;
    bool has_motor_temp_c;
    bool has_rudder_normalized;
    bool has_flags;
    bool rudder_valid;
    bool synced;
    bool engaged;
    bool faulted;
    float current_a;
    float voltage_v;
    float controller_temp_c;
    float motor_temp_c;
    float rudder_normalized;
    uint16_t flags;
    uint64_t timestamp_us;

    ServoFeedback()
        : has_current_a(false), has_voltage_v(false), has_controller_temp_c(false),
          has_motor_temp_c(false), has_rudder_normalized(false), has_flags(false),
          rudder_valid(false), synced(false), engaged(false), faulted(false),
          current_a(0.0f), voltage_v(0.0f), controller_temp_c(0.0f), motor_temp_c(0.0f),
          rudder_normalized(0.0f), flags(0), timestamp_us(0) {}
};

inline float servo_protocol_normalized_command(const ServoCommand& command) {
    if (!command.enabled || command.mode == ServoCommandMode::Stop) return 0.0f;
    return servo_clamp(command.value, -1.0f, 1.0f);
}

inline pypilot_servo_protocol::RawPacket servo_command_to_protocol_packet(const ServoCommand& command) {
    return pypilot_servo_protocol::encode_packet(
        pypilot_servo_protocol::COMMAND_CODE,
        pypilot_servo_protocol::encode_command(servo_protocol_normalized_command(command)));
}

inline ServoProtocolCommandPacket servo_safety_to_protocol_packet(const ServoCommand& command,
                                                                  const RudderAngle& rudder,
                                                                  uint64_t now_us,
                                                                  const ServoSafetyConfig& config = ServoSafetyConfig()) {
    const ServoSafetyResult safe = apply_servo_safety(command, rudder, now_us, config);
    return ServoProtocolCommandPacket(
        servo_command_to_protocol_packet(safe.command),
        safe.command,
        safe.reason,
        safe.allowed);
}

inline ServoFeedback servo_feedback_from_protocol(const pypilot_servo_protocol::Telemetry& telemetry,
                                                  uint64_t now_us) {
    ServoFeedback feedback;
    feedback.has_current_a = telemetry.has_current;
    feedback.has_voltage_v = telemetry.has_voltage;
    feedback.has_controller_temp_c = telemetry.has_controller_temp;
    feedback.has_motor_temp_c = telemetry.has_motor_temp;
    feedback.has_rudder_normalized = telemetry.has_rudder;
    feedback.has_flags = telemetry.has_flags;
    feedback.rudder_valid = telemetry.rudder_valid;
    feedback.current_a = telemetry.current_a;
    feedback.voltage_v = telemetry.voltage_v;
    feedback.controller_temp_c = telemetry.controller_temp_c;
    feedback.motor_temp_c = telemetry.motor_temp_c;
    feedback.rudder_normalized = telemetry.rudder;
    feedback.flags = telemetry.flags;
    feedback.synced = telemetry.synced();
    feedback.engaged = telemetry.engaged();
    feedback.faulted = telemetry.faulted();
    feedback.timestamp_us = now_us;
    return feedback;
}

}  // namespace pypilot_steering_signaling
