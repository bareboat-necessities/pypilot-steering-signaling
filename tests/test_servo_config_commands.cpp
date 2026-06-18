#include <cassert>
#include <pypilot_steering_signaling.hpp>

using namespace pypilot_steering_signaling;

static pypilot_servo_protocol::Packet decoded_packet(const pypilot_servo_protocol::RawPacket& raw) {
    pypilot_servo_protocol::Packet packet;
    assert(pypilot_servo_protocol::decode_packet(raw.bytes, packet));
    return packet;
}

static void assert_packet(const ServoConfigPacket& config_packet,
                          uint8_t expected_code,
                          uint16_t expected_value) {
    assert(config_packet.code == expected_code);
    assert(config_packet.value == expected_value);
    pypilot_servo_protocol::Packet decoded = decoded_packet(config_packet.raw_packet);
    assert(decoded.code == expected_code);
    assert(decoded.value == expected_value);
}

int main() {
    assert_packet(servo_disengage_packet(), pypilot_servo_protocol::DISENGAGE_CODE, 0u);
    assert_packet(servo_reset_packet(), pypilot_servo_protocol::RESET_CODE, 0u);

    assert_packet(servo_max_current_packet(12.34f),
                  pypilot_servo_protocol::MAX_CURRENT_CODE,
                  pypilot_servo_protocol::encode_centivalue(12.34f));
    assert_packet(servo_max_current_packet(-5.0f),
                  pypilot_servo_protocol::MAX_CURRENT_CODE,
                  pypilot_servo_protocol::encode_centivalue(0.0f));

    assert_packet(servo_max_controller_temp_packet(70.0f),
                  pypilot_servo_protocol::MAX_CONTROLLER_TEMP_CODE,
                  pypilot_servo_protocol::encode_centivalue(70.0f));
    assert_packet(servo_max_motor_temp_packet(80.0f),
                  pypilot_servo_protocol::MAX_MOTOR_TEMP_CODE,
                  pypilot_servo_protocol::encode_centivalue(80.0f));

    assert_packet(servo_rudder_range_packet(0.42f),
                  pypilot_servo_protocol::RUDDER_RANGE_CODE,
                  pypilot_servo_protocol::encode_rudder(0.42f));
    assert_packet(servo_rudder_min_packet(-0.35f),
                  pypilot_servo_protocol::RUDDER_MIN_CODE,
                  pypilot_servo_protocol::encode_rudder(-0.35f));
    assert_packet(servo_rudder_max_packet(0.35f),
                  pypilot_servo_protocol::RUDDER_MAX_CODE,
                  pypilot_servo_protocol::encode_rudder(0.35f));

    assert_packet(servo_max_slew_packet(3u, 17u),
                  pypilot_servo_protocol::MAX_SLEW_CODE,
                  static_cast<uint16_t>(3u | (17u << 8)));

    assert_packet(servo_clutch_pwm_and_brake_packet(128u, true),
                  pypilot_servo_protocol::CLUTCH_PWM_AND_BRAKE_CODE,
                  static_cast<uint16_t>(128u | (1u << 8)));
    assert_packet(servo_clutch_pwm_and_brake_packet(64u, false),
                  pypilot_servo_protocol::CLUTCH_PWM_AND_BRAKE_CODE,
                  static_cast<uint16_t>(64u));

    return 0;
}
