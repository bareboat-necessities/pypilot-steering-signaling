#pragma once

#include <stdint.h>
#include "servo_protocol_bridge.hpp"

namespace pypilot_steering_signaling {

struct ServoRuntimeConfig {
    ServoSafetyConfig safety;
    uint64_t output_period_us;
    bool emit_neutral_on_block;
    bool emit_on_source_change;
    bool block_on_feedback_fault;

    ServoRuntimeConfig()
        : safety(), output_period_us(100000ULL), emit_neutral_on_block(true),
          emit_on_source_change(true), block_on_feedback_fault(true) {}
};

struct ServoRuntimeOutput {
    ServoProtocolCommandPacket protocol;
    bool emit;
    ServoCommandSource source;

    ServoRuntimeOutput() : protocol(), emit(false), source(ServoCommandSource::None) {}
    ServoRuntimeOutput(const ServoProtocolCommandPacket& packet,
                       bool emit_value,
                       ServoCommandSource source_value = ServoCommandSource::None)
        : protocol(packet), emit(emit_value), source(source_value) {}
};

class ServoRuntime {
public:
    ServoRuntime()
        : config_(), last_input_(), last_source_(ServoCommandSource::None), latest_feedback_(),
          has_input_(false), has_feedback_(false), has_emit_(false), last_emit_us_(0),
          last_sent_allowed_(false), last_sent_source_(ServoCommandSource::None) {}

    explicit ServoRuntime(const ServoRuntimeConfig& config)
        : config_(config), last_input_(), last_source_(ServoCommandSource::None), latest_feedback_(),
          has_input_(false), has_feedback_(false), has_emit_(false), last_emit_us_(0),
          last_sent_allowed_(false), last_sent_source_(ServoCommandSource::None) {}

    void reset() {
        last_input_ = ServoCommand();
        last_source_ = ServoCommandSource::None;
        latest_feedback_ = ServoFeedback();
        has_input_ = false;
        has_feedback_ = false;
        has_emit_ = false;
        last_emit_us_ = 0;
        last_sent_allowed_ = false;
        last_sent_source_ = ServoCommandSource::None;
    }

    void set_config(const ServoRuntimeConfig& config) { config_ = config; }
    const ServoRuntimeConfig& config() const { return config_; }
    ServoRuntimeConfig& mutable_config() { return config_; }

    bool has_input() const { return has_input_; }
    ServoCommand last_input() const { return last_input_; }
    ServoCommandSource last_source() const { return last_source_; }

    bool has_feedback() const { return has_feedback_; }
    ServoFeedback latest_feedback() const { return latest_feedback_; }

    uint64_t last_emit_us() const { return last_emit_us_; }
    bool has_emitted() const { return has_emit_; }
    ServoCommandSource last_sent_source() const { return last_sent_source_; }

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
        return update_command_with_source(command, ServoCommandSource::Autopilot, rudder, now_us);
    }

    ServoRuntimeOutput update_command_request(const ServoCommandRequest& request,
                                             const RudderAngle& rudder,
                                             uint64_t now_us) {
        return update_command_with_source(request.command, request.source, rudder, now_us);
    }

    ServoRuntimeOutput update_command_requests(const ServoCommandRequest& autopilot,
                                              const ServoCommandRequest& manual,
                                              const RudderAngle& rudder,
                                              uint64_t now_us) {
        const ServoCommandRequest selected = select_servo_command_request(autopilot, manual, now_us);
        return update_command_request(selected, rudder, now_us);
    }

private:
    ServoProtocolCommandPacket feedback_blocked_packet(uint64_t now_us) const {
        const ServoCommand stop = servo_stop_command(now_us);
        return ServoProtocolCommandPacket(
            servo_command_to_protocol_packet(stop),
            stop,
            ServoSafetyReason::FeedbackBlocked,
            false);
    }

    ServoRuntimeOutput update_command_with_source(const ServoCommand& command,
                                                  ServoCommandSource source,
                                                  const RudderAngle& rudder,
                                                  uint64_t now_us) {
        has_input_ = true;
        last_input_ = command;
        last_source_ = source;

        ServoProtocolCommandPacket packet =
            servo_safety_to_protocol_packet(command, rudder, now_us, config_.safety);
        if (config_.block_on_feedback_fault && has_feedback_ && latest_feedback_.faulted) {
            packet = feedback_blocked_packet(now_us);
        }

        const bool should_emit = should_emit_packet(packet, source, now_us);
        if (should_emit) {
            has_emit_ = true;
            last_emit_us_ = now_us;
            last_sent_allowed_ = packet.allowed;
            last_sent_source_ = source;
        }
        return ServoRuntimeOutput(packet, should_emit, source);
    }

    bool should_emit_packet(const ServoProtocolCommandPacket& packet,
                            ServoCommandSource source,
                            uint64_t now_us) const {
        if (!has_emit_) return true;
        if (config_.emit_neutral_on_block && !packet.allowed && last_sent_allowed_) return true;
        if (config_.emit_on_source_change && source != last_sent_source_) return true;
        if (config_.output_period_us == 0) return true;
        if (now_us < last_emit_us_) return false;
        return (now_us - last_emit_us_) >= config_.output_period_us;
    }

    ServoRuntimeConfig config_;
    ServoCommand last_input_;
    ServoCommandSource last_source_;
    ServoFeedback latest_feedback_;
    bool has_input_;
    bool has_feedback_;
    bool has_emit_;
    uint64_t last_emit_us_;
    bool last_sent_allowed_;
    ServoCommandSource last_sent_source_;
};

}  // namespace pypilot_steering_signaling
