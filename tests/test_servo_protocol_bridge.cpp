#include <cassert>
#include <cmath>
#include <pypilot_steering_signaling.hpp>

using namespace pypilot_steering_signaling;

static bool nearf(float a, float b, float eps = 0.0001f) {
    return std::fabs(a - b) < eps;
}

static pypilot_servo_protocol::Packet decoded_packet(const pypilot_servo_protocol::RawPacket& raw) {
    pypilot_servo_protocol::Packet packet;
    assert(pypilot_servo_protocol::decode_packet(raw.bytes, packet));
    return packet;
}

int main() {
    ServoSafetyConfig config;
    config.command_timeout_us = 1000000ULL;
    config.max_speed_abs = 0.5f;

    const RudderAngle centered_rudder(0.0f, 1000000ULL, true);

    ServoProtocolCommandPacket allowed = servo_safety_to_protocol_packet(
        ServoCommand(ServoCommandMode::Speed, 0.25f, 1000000ULL, true),
        centered_rudder,
        1200000ULL,
        config);
    assert(allowed.allowed);
    assert(allowed.safety_reason == ServoSafetyReason::None);
    assert(nearf(allowed.effective_command.value, 0.25f));
    pypilot_servo_protocol::Packet allowed_packet = decoded_packet(allowed.raw_packet);
    assert(allowed_packet.code == pypilot_servo_protocol::COMMAND_CODE);
    assert(allowed_packet.value == pypilot_servo_protocol::encode_command(0.25f));

    ServoProtocolCommandPacket clamped = servo_safety_to_protocol_packet(
        ServoCommand(ServoCommandMode::Speed, 2.0f, 1000000ULL, true),
        centered_rudder,
        1200000ULL,
        config);
    assert(clamped.allowed);
    assert(nearf(clamped.effective_command.value, 0.5f));
    pypilot_servo_protocol::Packet clamped_packet = decoded_packet(clamped.raw_packet);
    assert(clamped_packet.value == pypilot_servo_protocol::encode_command(0.5f));

    ServoProtocolCommandPacket stale = servo_safety_to_protocol_packet(
        ServoCommand(ServoCommandMode::Speed, 0.25f, 1000000ULL, true),
        centered_rudder,
        2500000ULL,
        config);
    assert(!stale.allowed);
    assert(stale.safety_reason == ServoSafetyReason::StaleCommand);
    pypilot_servo_protocol::Packet stale_packet = decoded_packet(stale.raw_packet);
    assert(stale_packet.value == pypilot_servo_protocol::encode_command(0.0f));

    pypilot_servo_protocol::Telemetry telemetry;
    pypilot_servo_protocol::Packet current = {pypilot_servo_protocol::CURRENT_CODE,
                                             pypilot_servo_protocol::encode_centivalue(12.34f)};
    pypilot_servo_protocol::Packet voltage = {pypilot_servo_protocol::VOLTAGE_CODE,
                                             pypilot_servo_protocol::encode_centivalue(12.80f)};
    pypilot_servo_protocol::Packet rudder = {pypilot_servo_protocol::RUDDER_SENSE_CODE,
                                            pypilot_servo_protocol::encode_rudder(0.20f)};
    pypilot_servo_protocol::Packet flags = {pypilot_servo_protocol::FLAGS_CODE,
                                           static_cast<uint16_t>(pypilot_servo_protocol::SYNC_FLAG |
                                                                 pypilot_servo_protocol::ENGAGED_FLAG)};
    assert(telemetry.apply(current));
    assert(telemetry.apply(voltage));
    assert(telemetry.apply(rudder));
    assert(telemetry.apply(flags));

    ServoFeedback feedback = servo_feedback_from_protocol(telemetry, 3000000ULL);
    assert(feedback.has_current_a);
    assert(feedback.has_voltage_v);
    assert(feedback.has_rudder_normalized);
    assert(feedback.has_flags);
    assert(feedback.rudder_valid);
    assert(feedback.synced);
    assert(feedback.engaged);
    assert(!feedback.faulted);
    assert(feedback.timestamp_us == 3000000ULL);
    assert(nearf(feedback.current_a, 12.34f, 0.01f));
    assert(nearf(feedback.voltage_v, 12.80f, 0.01f));
    assert(nearf(feedback.rudder_normalized, 0.20f, 0.001f));

    pypilot_servo_protocol::Packet fault_flags = {pypilot_servo_protocol::FLAGS_CODE,
                                                 pypilot_servo_protocol::OVERTEMP_FAULT};
    assert(telemetry.apply(fault_flags));
    ServoFeedback fault_feedback = servo_feedback_from_protocol(telemetry, 4000000ULL);
    assert(fault_feedback.faulted);

    return 0;
}
