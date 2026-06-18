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
    assert(nearf(apply_rudder_calibration(RudderSample(512.0f, 1000ULL), calibration).angle_deg, 0.0f));
    return 0;
}
