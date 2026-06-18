#include <cassert>
#include <cmath>
#include <pypilot_steering_signaling.hpp>

using namespace pypilot_steering_signaling;

static bool nearf(float a, float b, float eps = 0.0001f) {
    return std::fabs(a - b) < eps;
}

int main() {
    const RudderCalibration calibration(0.0f, 512.0f, 1024.0f, -45.0f, 45.0f);
    assert(rudder_calibration_is_valid(calibration));

    const RudderAngle center = apply_rudder_calibration(RudderSample(512.0f, 1000ULL), calibration);
    assert(center.valid);
    assert(center.timestamp_us == 1000ULL);
    assert(nearf(center.angle_deg, 0.0f));

    const RudderAngle port = apply_rudder_calibration(RudderSample(0.0f, 1001ULL), calibration);
    assert(port.valid);
    assert(nearf(port.angle_deg, -45.0f));

    const RudderAngle starboard = apply_rudder_calibration(RudderSample(1024.0f, 1002ULL), calibration);
    assert(starboard.valid);
    assert(nearf(starboard.angle_deg, 45.0f));

    const RudderAngle port_mid = apply_rudder_calibration(RudderSample(256.0f, 1003ULL), calibration);
    assert(port_mid.valid);
    assert(nearf(port_mid.angle_deg, -22.5f));

    const RudderAngle starboard_mid = apply_rudder_calibration(RudderSample(768.0f, 1004ULL), calibration);
    assert(starboard_mid.valid);
    assert(nearf(starboard_mid.angle_deg, 22.5f));

    const RudderAngle port_limit = apply_rudder_calibration(RudderSample(-500.0f, 1005ULL), calibration);
    assert(port_limit.valid);
    assert(nearf(port_limit.angle_deg, -45.0f));

    const RudderAngle starboard_limit = apply_rudder_calibration(RudderSample(2000.0f, 1006ULL), calibration);
    assert(starboard_limit.valid);
    assert(nearf(starboard_limit.angle_deg, 45.0f));

    const RudderCalibration asymmetric(100.0f, 300.0f, 700.0f, -35.0f, 45.0f);
    assert(rudder_calibration_is_valid(asymmetric));
    assert(nearf(apply_rudder_calibration(RudderSample(200.0f, 1ULL), asymmetric).angle_deg, -17.5f));
    assert(nearf(apply_rudder_calibration(RudderSample(500.0f, 1ULL), asymmetric).angle_deg, 22.5f));

    float angle_deg = 0.0f;
    assert(calibrated_rudder_angle_deg(RudderSample(768.0f, 2000ULL), calibration, angle_deg));
    assert(nearf(angle_deg, 22.5f));

    const RudderAngle bad_sample = apply_rudder_calibration(RudderSample(512.0f, 2001ULL, false), calibration);
    assert(!bad_sample.valid);
    assert(bad_sample.timestamp_us == 2001ULL);

    assert(!rudder_calibration_is_valid(RudderCalibration(0.0f, 0.0f, 1024.0f, -45.0f, 45.0f)));
    assert(!rudder_calibration_is_valid(RudderCalibration(0.0f, 512.0f, 512.0f, -45.0f, 45.0f)));
    assert(!rudder_calibration_is_valid(RudderCalibration(0.0f, 512.0f, 1024.0f, 0.0f, 45.0f)));
    assert(!rudder_calibration_is_valid(RudderCalibration(0.0f, 512.0f, 1024.0f, -45.0f, 0.0f)));

    return 0;
}
