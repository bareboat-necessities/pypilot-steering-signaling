#pragma once

#include <stdint.h>
#include <pypilot_syslib.hpp>
#include "rudder_calibration.hpp"

namespace pypilot_steering_signaling {

struct RudderRuntimeConfig {
    uint64_t stale_timeout_us;

    RudderRuntimeConfig() : stale_timeout_us(8000000ULL) {}
    explicit RudderRuntimeConfig(uint64_t timeout_us) : stale_timeout_us(timeout_us) {}
};

class RudderRuntime {
public:
    RudderRuntime()
        : calibration_(), config_(), latest_(), has_sample_(false), logger_(0) {}

    explicit RudderRuntime(const RudderCalibration& calibration)
        : calibration_(calibration), config_(), latest_(), has_sample_(false), logger_(0) {}

    void set_logger(pypilot_syslib::Logger* logger) { logger_ = logger; }
    pypilot_syslib::Logger* logger() const { return logger_; }

    void reset() {
        latest_ = RudderAngle();
        has_sample_ = false;
    }

    void set_calibration(const RudderCalibration& calibration) {
        calibration_ = calibration;
        if (!rudder_calibration_is_valid(calibration_)) {
            latest_ = RudderAngle();
            has_sample_ = false;
            pypilot_syslib::log_if(logger_, 0ULL,
                                   pypilot_syslib::LogLevel::Error,
                                   pypilot_syslib::LogModule::SteeringSignaling,
                                   pypilot_syslib::LogEvent::RudderCalibrationInvalid,
                                   "invalid rudder calibration");
        }
    }

    const RudderCalibration& calibration() const { return calibration_; }
    bool calibration_valid() const { return rudder_calibration_is_valid(calibration_); }

    void set_timeout_us(uint64_t timeout_us) { config_.stale_timeout_us = timeout_us; }
    uint64_t timeout_us() const { return config_.stale_timeout_us; }

    RudderAngle update_sample(const RudderSample& sample) {
        has_sample_ = true;
        latest_ = apply_rudder_calibration(sample, calibration_);
        return latest_;
    }

    RudderAngle latest() const { return latest_; }
    bool has_sample() const { return has_sample_; }

    bool is_stale(uint64_t now_us) const {
        if (!has_sample_ || !latest_.valid) return true;
        if (now_us < latest_.timestamp_us) return false;
        return (now_us - latest_.timestamp_us) > config_.stale_timeout_us;
    }

    RudderAngle update_time(uint64_t now_us) {
        if (is_stale(now_us)) {
            latest_.valid = false;
        }
        return latest_;
    }

private:
    RudderCalibration calibration_;
    RudderRuntimeConfig config_;
    RudderAngle latest_;
    bool has_sample_;
    pypilot_syslib::Logger* logger_;
};

}  // namespace pypilot_steering_signaling
