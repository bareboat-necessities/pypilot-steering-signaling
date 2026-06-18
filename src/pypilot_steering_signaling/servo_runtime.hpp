#pragma once

#include <stdint.h>
#include "servo_protocol_bridge.hpp"

namespace pypilot_steering_signaling {

struct ServoRuntimeConfig {
    ServoSafetyConfig safety;
    uint64_t output_period_us;
    bool emit_neutral_on_block;

    ServoRuntimeConfig()
        : safety(), output_period_us(100000ULL), emit_neutral_on_block(true) {}
};

struct ServoRuntimeOutput {
    ServoProtocolCommandPacket protocol;
    bool emit;

    ServoRuntimeOutput() : protocol(), emit(false) {}
    ServoRuntimeOutput(const ServoProtocolCommandPacket& packet, bool emit_value)
        : protocol(packet), emit(emit_value) {}
};

class ServoRuntime {
public:
    ServoRuntime()
        : config_(), last_input_(), latest_feedback_(), has_input_(false),
          has_feedback_(false), has_emit_(false), last_emit_us_(0),
          last_sent_allowed_(false) {}

    explicit ServoRuntime(const ServoRuntimeConfig& config)
        : config_(config), last_input_(), latest_feedback_(), has_input_(false),
          has_feedback_(false), has_emit_(false), last_emit_us_(0),
          last_sent_allowed_(false) {}

    void reset() {
        last_input_ = ServoCommand();
        latest_feedback_ = ServoFeedback();
        has_input_ = false;
        has_feedback_ = false;
        has_emit_ = false;
        last_emit_us_ = 0;
        last_sent_allowed_ = false;
    }

    void set_config(const ServoRuntimeConfig& config) { config_ = config; }
    const ServoRuntimeConfig& config() const { return config_; }
    ServoRuntimeConfig& mutable_config() { return config_; }

    bool has_input() const { return has_input_; }
    ServoCommand last_input() const { return last_input_; }

    bool has_feedback() const { return has_feedback_; }
    ServoFeedback latest_feedback() const { return latest_feedback_; }

    uint64_t last_emit_us() const { return last_emit_us_; }
    bool has_emitted() const { return has_emit_; }

    void update_feedback(const ServoFeedback& feedback) {
        latest_feedback_ = feedback;
        has_feedback_ = true;
    }

    void update_feedback(const pypilot_servo_protocol::Telemetry& telemetry, uint64_t now_us) {
        update_feedback(servo_feedback_from_protocol(telemetry, now_us));
    }

    ServoRuntimeOutput update_command(const ServoCommand& command,
                                      const RudderAngle& rudder,
                                      uint64_t now_us) {
        has_input_ = true;
        last_input_ = command;

        const ServoProtocolCommandPacket packet =
            servo_safety_to_protocol_packet(command, rudder, now_us, config_.safety);
        const bool should_emit = should_emit_packet(packet, now_us);
        if (should_emit) {
            has_emit_ = true;
            last_emit_us_ = now_us;
            last_sent_allowed_ = packet.allowed;
        }
        return ServoRuntimeOutput(packet, should_emit);
    }

private:
    bool should_emit_packet(const ServoProtocolCommandPacket& packet, uint64_t now_us) const {
        if (!has_emit_) return true;
        if (config_.emit_neutral_on_block && !packet.allowed && last_sent_allowed_) return true;
        if (config_.output_period_us == 0) return true;
        if (now_us < last_emit_us_) return false;
        return (now_us - last_emit_us_) >= config_.output_period_us;
    }

    ServoRuntimeConfig config_;
    ServoCommand last_input_;
    ServoFeedback latest_feedback_;
    bool has_input_;
    bool has_feedback_;
    bool has_emit_;
    uint64_t last_emit_us_;
    bool last_sent_allowed_;
};

}  // namespace pypilot_steering_signaling
