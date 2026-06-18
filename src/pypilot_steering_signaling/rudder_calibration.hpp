#pragma once

#include <stdint.h>

namespace pypilot_steering_signaling {

inline bool rudder_is_finite(float value) {
    return value == value && value > -3.4028234e38f && value < 3.4028234e38f;
}

inline float rudder_clamp(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

struct RudderCalibration {
    float raw_min;
    float raw_center;
    float raw_max;
    float angle_min_deg;
    float angle_max_deg;
    bool valid;

    RudderCalibration()
        : raw_min(0.0f), raw_center(0.0f), raw_max(0.0f),
          angle_min_deg(0.0f), angle_max_deg(0.0f), valid(false) {}

    RudderCalibration(float raw_min_value,
                      float raw_center_value,
                      float raw_max_value,
                      float angle_min_value_deg,
                      float angle_max_value_deg,
                      bool valid_value = true)
        : raw_min(raw_min_value), raw_center(raw_center_value), raw_max(raw_max_value),
          angle_min_deg(angle_min_value_deg), angle_max_deg(angle_max_value_deg),
          valid(valid_value) {}
};

struct RudderSample {
    float raw;
    uint64_t timestamp_us;
    bool valid;

    RudderSample() : raw(0.0f), timestamp_us(0), valid(false) {}
    RudderSample(float raw_value, uint64_t time_us, bool valid_value = true)
        : raw(raw_value), timestamp_us(time_us), valid(valid_value) {}
};

struct RudderAngle {
    float angle_deg;
    uint64_t timestamp_us;
    bool valid;

    RudderAngle() : angle_deg(0.0f), timestamp_us(0), valid(false) {}
    RudderAngle(float angle_value_deg, uint64_t time_us, bool valid_value)
        : angle_deg(angle_value_deg), timestamp_us(time_us), valid(valid_value) {}
};

inline bool rudder_calibration_is_valid(const RudderCalibration& calibration) {
    if (!calibration.valid) return false;
    if (!rudder_is_finite(calibration.raw_min) ||
        !rudder_is_finite(calibration.raw_center) ||
        !rudder_is_finite(calibration.raw_max) ||
        !rudder_is_finite(calibration.angle_min_deg) ||
        !rudder_is_finite(calibration.angle_max_deg)) {
        return false;
    }
    if (!(calibration.raw_min < calibration.raw_center &&
          calibration.raw_center < calibration.raw_max)) {
        return false;
    }
    if (!(calibration.angle_min_deg < 0.0f && calibration.angle_max_deg > 0.0f)) {
        return false;
    }
    return true;
}

inline RudderAngle apply_rudder_calibration(const RudderSample& sample,
                                            const RudderCalibration& calibration) {
    if (!sample.valid || !rudder_is_finite(sample.raw) ||
        !rudder_calibration_is_valid(calibration)) {
        return RudderAngle(0.0f, sample.timestamp_us, false);
    }

    float angle = 0.0f;
    if (sample.raw >= calibration.raw_center) {
        const float upper_span = calibration.raw_max - calibration.raw_center;
        const float ratio = (sample.raw - calibration.raw_center) / upper_span;
        angle = ratio * calibration.angle_max_deg;
    } else {
        const float lower_span = calibration.raw_center - calibration.raw_min;
        const float ratio = (calibration.raw_center - sample.raw) / lower_span;
        angle = -ratio * -calibration.angle_min_deg;
    }

    angle = rudder_clamp(angle, calibration.angle_min_deg, calibration.angle_max_deg);
    return RudderAngle(angle, sample.timestamp_us, true);
}

inline bool calibrated_rudder_angle_deg(const RudderSample& sample,
                                        const RudderCalibration& calibration,
                                        float& angle_deg_out) {
    const RudderAngle angle = apply_rudder_calibration(sample, calibration);
    if (!angle.valid) {
        angle_deg_out = 0.0f;
        return false;
    }
    angle_deg_out = angle.angle_deg;
    return true;
}

}  // namespace pypilot_steering_signaling
