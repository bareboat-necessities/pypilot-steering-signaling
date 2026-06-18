#include <iostream>
#include <pypilot_steering_signaling.hpp>

int main() {
    const pypilot_steering_signaling::RudderCalibration calibration(
        0.0f, 512.0f, 1024.0f, -45.0f, 45.0f);
    const pypilot_steering_signaling::RudderSample sample(768.0f, 1000000ULL);
    const pypilot_steering_signaling::RudderAngle angle =
        pypilot_steering_signaling::apply_rudder_calibration(sample, calibration);

    if (!angle.valid) {
        std::cerr << "rudder calibration failed\n";
        return 1;
    }

    std::cout << "rudder_angle_deg=" << angle.angle_deg << "\n";
    return 0;
}
