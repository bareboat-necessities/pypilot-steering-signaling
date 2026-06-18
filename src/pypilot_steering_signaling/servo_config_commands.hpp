#pragma once

#include <stdint.h>
#include <pypilot_servo_protocol.hpp>

namespace pypilot_steering_signaling {

struct ServoConfigPacket {
    pypilot_servo_protocol::RawPacket raw_packet;
    uint8_t code;
    uint16_t value;

    ServoConfigPacket()
        : raw_packet({{0, 0, 0, 0}}), code(0), value(0) {}

    ServoConfigPacket(uint8_t code_value, uint16_t payload_value)
        : raw_packet(pypilot_servo_protocol::encode_packet(code_value, payload_value)),
          code(code_value), value(payload_value) {}
};

inline float servo_nonnegative(float value) {
    return value < 0.0f ? 0.0f : value;
}

inline ServoConfigPacket servo_config_packet(uint8_t code, uint16_t value) {
    return ServoConfigPacket(code, value);
}

inline ServoConfigPacket servo_disengage_packet() {
    return servo_config_packet(pypilot_servo_protocol::DISENGAGE_CODE, 0u);
}

inline ServoConfigPacket servo_reset_packet() {
    return servo_config_packet(pypilot_servo_protocol::RESET_CODE, 0u);
}

inline ServoConfigPacket servo_max_current_packet(float amps) {
    return servo_config_packet(
        pypilot_servo_protocol::MAX_CURRENT_CODE,
        pypilot_servo_protocol::encode_centivalue(servo_nonnegative(amps)));
}

inline ServoConfigPacket servo_max_controller_temp_packet(float temp_c) {
    return servo_config_packet(
        pypilot_servo_protocol::MAX_CONTROLLER_TEMP_CODE,
        pypilot_servo_protocol::encode_centivalue(servo_nonnegative(temp_c)));
}

inline ServoConfigPacket servo_max_motor_temp_packet(float temp_c) {
    return servo_config_packet(
        pypilot_servo_protocol::MAX_MOTOR_TEMP_CODE,
        pypilot_servo_protocol::encode_centivalue(servo_nonnegative(temp_c)));
}

inline ServoConfigPacket servo_rudder_range_packet(float normalized_range) {
    return servo_config_packet(
        pypilot_servo_protocol::RUDDER_RANGE_CODE,
        pypilot_servo_protocol::encode_rudder(normalized_range));
}

inline ServoConfigPacket servo_rudder_min_packet(float normalized_min) {
    return servo_config_packet(
        pypilot_servo_protocol::RUDDER_MIN_CODE,
        pypilot_servo_protocol::encode_rudder(normalized_min));
}

inline ServoConfigPacket servo_rudder_max_packet(float normalized_max) {
    return servo_config_packet(
        pypilot_servo_protocol::RUDDER_MAX_CODE,
        pypilot_servo_protocol::encode_rudder(normalized_max));
}

inline ServoConfigPacket servo_max_slew_packet(uint8_t fast, uint8_t slow) {
    return servo_config_packet(
        pypilot_servo_protocol::MAX_SLEW_CODE,
        static_cast<uint16_t>(fast) | static_cast<uint16_t>(static_cast<uint16_t>(slow) << 8));
}

inline ServoConfigPacket servo_clutch_pwm_and_brake_packet(uint8_t pwm, bool brake) {
    return servo_config_packet(
        pypilot_servo_protocol::CLUTCH_PWM_AND_BRAKE_CODE,
        static_cast<uint16_t>(pwm) | static_cast<uint16_t>((brake ? 1u : 0u) << 8));
}

}  // namespace pypilot_steering_signaling
