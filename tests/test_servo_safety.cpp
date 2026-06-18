#include <cassert>
#include <cmath>
#include <limits>
#include <pypilot_steering_signaling.hpp>

using namespace pypilot_steering_signaling;

static bool nearf(float a, float b, float eps = 0.0001f) {
    return std::fabs(a - b) < eps;
}

int main() {
    ServoCommand stop;
    assert(stop.mode == ServoCommandMode::Stop);
    assert(!stop.enabled);
    assert(servo_command_is_stop(stop));

    ServoCommand speed(ServoCommandMode::Speed, 0.25f, 1000000ULL, true);
    assert(!servo_command_is_stop(speed));
    assert(!servo_command_is_stale(speed, 1500000ULL, 1000000ULL));
    assert(!servo_command_is_stale(speed, 2000000ULL, 1000000ULL));
    assert(servo_command_is_stale(speed, 2000001ULL, 1000000ULL));

    ServoSafetyConfig config;
    config.command_timeout_us = 1000000ULL;
    config.max_speed_abs = 0.5f;
    config.max_position_abs = 0.75f;
    config.max_raw_pwm_abs = 0.9f;
    config.rudder_min_deg = -45.0f;
    config.rudder_max_deg = 45.0f;
    config.rudder_limit_margin_deg = 0.5f;

    ServoSafetyResult allowed = apply_servo_safety(
        ServoCommand(ServoCommandMode::Speed, 0.25f, 1200000ULL, true),
        RudderAngle(0.0f, 1200000ULL, true),
        1300000ULL,
        config);
    assert(allowed.allowed);
    assert(allowed.reason == ServoSafetyReason::None);
    assert(allowed.command.mode == ServoCommandMode::Speed);
    assert(nearf(allowed.command.value, 0.25f));

    ServoSafetyResult clamped = apply_servo_safety(
        ServoCommand(ServoCommandMode::Speed, 2.0f, 1200000ULL, true),
        RudderAngle(0.0f, 1200000ULL, true),
        1300000ULL,
        config);
    assert(clamped.allowed);
    assert(nearf(clamped.command.value, 0.5f));

    ServoSafetyResult disabled = apply_servo_safety(
        ServoCommand(ServoCommandMode::Speed, 0.2f, 1200000ULL, false),
        RudderAngle(0.0f, 1200000ULL, true),
        1300000ULL,
        config);
    assert(!disabled.allowed);
    assert(disabled.reason == ServoSafetyReason::Disabled);
    assert(disabled.command.mode == ServoCommandMode::Stop);

    ServoSafetyResult stale = apply_servo_safety(
        ServoCommand(ServoCommandMode::Speed, 0.2f, 1000000ULL, true),
        RudderAngle(0.0f, 1000000ULL, true),
        2000001ULL,
        config);
    assert(!stale.allowed);
    assert(stale.reason == ServoSafetyReason::StaleCommand);

    const float nan_value = std::numeric_limits<float>::quiet_NaN();
    ServoSafetyResult invalid_value = apply_servo_safety(
        ServoCommand(ServoCommandMode::Speed, nan_value, 1200000ULL, true),
        RudderAngle(0.0f, 1200000ULL, true),
        1300000ULL,
        config);
    assert(!invalid_value.allowed);
    assert(invalid_value.reason == ServoSafetyReason::InvalidValue);

    ServoSafetyResult invalid_rudder = apply_servo_safety(
        ServoCommand(ServoCommandMode::Speed, 0.2f, 1200000ULL, true),
        RudderAngle(0.0f, 1200000ULL, false),
        1300000ULL,
        config);
    assert(!invalid_rudder.allowed);
    assert(invalid_rudder.reason == ServoSafetyReason::RudderInvalid);

    ServoSafetyResult port_block = apply_servo_safety(
        ServoCommand(ServoCommandMode::Speed, -0.2f, 1200000ULL, true),
        RudderAngle(-44.6f, 1200000ULL, true),
        1300000ULL,
        config);
    assert(!port_block.allowed);
    assert(port_block.reason == ServoSafetyReason::RudderPortLimit);

    ServoSafetyResult port_allows_starboard = apply_servo_safety(
        ServoCommand(ServoCommandMode::Speed, 0.2f, 1200000ULL, true),
        RudderAngle(-44.6f, 1200000ULL, true),
        1300000ULL,
        config);
    assert(port_allows_starboard.allowed);

    ServoSafetyResult starboard_block = apply_servo_safety(
        ServoCommand(ServoCommandMode::Speed, 0.2f, 1200000ULL, true),
        RudderAngle(44.6f, 1200000ULL, true),
        1300000ULL,
        config);
    assert(!starboard_block.allowed);
    assert(starboard_block.reason == ServoSafetyReason::RudderStarboardLimit);

    config.enforce_rudder_limits = false;
    ServoSafetyResult limits_disabled = apply_servo_safety(
        ServoCommand(ServoCommandMode::RawPwm, -2.0f, 1200000ULL, true),
        RudderAngle(0.0f, 1200000ULL, false),
        1300000ULL,
        config);
    assert(limits_disabled.allowed);
    assert(nearf(limits_disabled.command.value, -0.9f));

    return 0;
}
