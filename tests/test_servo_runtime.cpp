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
    ServoRuntimeConfig config;
    config.output_period_us = 100000ULL;
    config.safety.command_timeout_us = 1000000ULL;
    config.safety.max_speed_abs = 0.5f;
    config.safety.rudder_min_deg = -45.0f;
    config.safety.rudder_max_deg = 45.0f;
    config.safety.rudder_limit_margin_deg = 0.5f;

    ServoRuntime runtime(config);
    assert(!runtime.has_input());
    assert(!runtime.has_feedback());
    assert(!runtime.has_emitted());

    const RudderAngle center(0.0f, 1000000ULL, true);
    ServoRuntimeOutput first = runtime.update_command(
        ServoCommand(ServoCommandMode::Speed, 0.25f, 1000000ULL, true),
        center,
        1000000ULL);
    assert(first.emit);
    assert(first.protocol.allowed);
    assert(first.protocol.safety_reason == ServoSafetyReason::None);
    assert(runtime.has_input());
    assert(runtime.has_emitted());
    assert(runtime.last_emit_us() == 1000000ULL);
    pypilot_servo_protocol::Packet first_packet = decoded_packet(first.protocol.raw_packet);
    assert(first_packet.value == pypilot_servo_protocol::encode_command(0.25f));

    ServoRuntimeOutput held = runtime.update_command(
        ServoCommand(ServoCommandMode::Speed, 0.30f, 1050000ULL, true),
        center,
        1050000ULL);
    assert(!held.emit);
    assert(held.protocol.allowed);
    assert(nearf(held.protocol.effective_command.value, 0.30f));
    assert(runtime.last_emit_us() == 1000000ULL);

    ServoRuntimeOutput second = runtime.update_command(
        ServoCommand(ServoCommandMode::Speed, 2.00f, 1100000ULL, true),
        center,
        1100000ULL);
    assert(second.emit);
    assert(second.protocol.allowed);
    assert(nearf(second.protocol.effective_command.value, 0.5f));
    pypilot_servo_protocol::Packet second_packet = decoded_packet(second.protocol.raw_packet);
    assert(second_packet.value == pypilot_servo_protocol::encode_command(0.5f));
    assert(runtime.last_emit_us() == 1100000ULL);

    ServoRuntimeOutput blocked = runtime.update_command(
        ServoCommand(ServoCommandMode::Speed, 0.20f, 1150000ULL, true),
        RudderAngle(44.6f, 1150000ULL, true),
        1150000ULL);
    assert(blocked.emit);
    assert(!blocked.protocol.allowed);
    assert(blocked.protocol.safety_reason == ServoSafetyReason::RudderStarboardLimit);
    pypilot_servo_protocol::Packet blocked_packet = decoded_packet(blocked.protocol.raw_packet);
    assert(blocked_packet.value == pypilot_servo_protocol::encode_command(0.0f));
    assert(runtime.last_emit_us() == 1150000ULL);

    ServoRuntimeOutput still_blocked = runtime.update_command(
        ServoCommand(ServoCommandMode::Speed, 0.20f, 1160000ULL, true),
        RudderAngle(44.6f, 1160000ULL, true),
        1160000ULL);
    assert(!still_blocked.emit);
    assert(!still_blocked.protocol.allowed);
    assert(runtime.last_emit_us() == 1150000ULL);

    ServoRuntimeOutput blocked_period = runtime.update_command(
        ServoCommand(ServoCommandMode::Speed, 0.20f, 1250000ULL, true),
        RudderAngle(44.6f, 1250000ULL, true),
        1250000ULL);
    assert(blocked_period.emit);
    assert(!blocked_period.protocol.allowed);
    assert(runtime.last_emit_us() == 1250000ULL);

    pypilot_servo_protocol::Telemetry telemetry;
    pypilot_servo_protocol::Packet voltage = {pypilot_servo_protocol::VOLTAGE_CODE,
                                             pypilot_servo_protocol::encode_centivalue(12.80f)};
    pypilot_servo_protocol::Packet flags = {pypilot_servo_protocol::FLAGS_CODE,
                                           static_cast<uint16_t>(pypilot_servo_protocol::SYNC_FLAG |
                                                                 pypilot_servo_protocol::ENGAGED_FLAG)};
    assert(telemetry.apply(voltage));
    assert(telemetry.apply(flags));
    runtime.update_feedback(telemetry, 1300000ULL);
    assert(runtime.has_feedback());
    assert(runtime.latest_feedback().has_voltage_v);
    assert(nearf(runtime.latest_feedback().voltage_v, 12.80f, 0.01f));
    assert(runtime.latest_feedback().synced);
    assert(runtime.latest_feedback().engaged);
    assert(runtime.latest_feedback().timestamp_us == 1300000ULL);

    runtime.reset();
    assert(!runtime.has_input());
    assert(!runtime.has_feedback());
    assert(!runtime.has_emitted());
    assert(runtime.last_emit_us() == 0ULL);

    return 0;
}
