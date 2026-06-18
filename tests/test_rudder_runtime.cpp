#include <cassert>
#include <cmath>
#include <pypilot_steering_signaling.hpp>

using namespace pypilot_steering_signaling;

static bool nearf(float a, float b, float eps = 0.0001f) {
    return std::fabs(a - b) < eps;
}

int main() {
    const RudderCalibration calibration(0.0f, 512.0f, 1024.0f, -45.0f, 45.0f);

    RudderRuntime runtime(calibration);
    assert(runtime.calibration_valid());
    assert(!runtime.has_sample());
    assert(runtime.is_stale(0ULL));
    assert(!runtime.latest().valid);

    runtime.set_timeout_us(2000000ULL);
    assert(runtime.timeout_us() == 2000000ULL);

    RudderAngle first = runtime.update_sample(RudderSample(768.0f, 1000000ULL));
    assert(first.valid);
    assert(nearf(first.angle_deg, 22.5f));
    assert(runtime.has_sample());
    assert(runtime.latest().valid);
    assert(nearf(runtime.latest().angle_deg, 22.5f));

    assert(!runtime.is_stale(1000000ULL));
    assert(!runtime.is_stale(3000000ULL));
    assert(runtime.is_stale(3000001ULL));

    RudderAngle still_valid = runtime.update_time(2500000ULL);
    assert(still_valid.valid);
    assert(nearf(still_valid.angle_deg, 22.5f));

    RudderAngle stale = runtime.update_time(4000000ULL);
    assert(!stale.valid);
    assert(runtime.is_stale(4000000ULL));

    RudderAngle second = runtime.update_sample(RudderSample(256.0f, 5000000ULL));
    assert(second.valid);
    assert(nearf(second.angle_deg, -22.5f));
    assert(!runtime.is_stale(5000000ULL));

    RudderAngle bad_sample = runtime.update_sample(RudderSample(512.0f, 6000000ULL, false));
    assert(!bad_sample.valid);
    assert(runtime.has_sample());
    assert(runtime.is_stale(6000000ULL));

    runtime.update_sample(RudderSample(512.0f, 7000000ULL));
    assert(runtime.latest().valid);
    runtime.reset();
    assert(!runtime.has_sample());
    assert(!runtime.latest().valid);
    assert(runtime.is_stale(8000000ULL));

    runtime.set_calibration(RudderCalibration(100.0f, 300.0f, 700.0f, -35.0f, 45.0f));
    assert(runtime.calibration_valid());
    RudderAngle asymmetric = runtime.update_sample(RudderSample(500.0f, 9000000ULL));
    assert(asymmetric.valid);
    assert(nearf(asymmetric.angle_deg, 22.5f));

    runtime.set_calibration(RudderCalibration(0.0f, 0.0f, 1024.0f, -45.0f, 45.0f));
    assert(!runtime.calibration_valid());
    assert(!runtime.has_sample());
    assert(!runtime.latest().valid);
    RudderAngle invalid_cal = runtime.update_sample(RudderSample(512.0f, 10000000ULL));
    assert(!invalid_cal.valid);

    return 0;
}
