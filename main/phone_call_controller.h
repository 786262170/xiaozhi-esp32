#pragma once

#include <atomic>
#include <cstdint>

enum class PhoneCallState : uint8_t {
    kIdle,
    kConnecting,
    kConnected,
    kHangingUp,
    kFailed,
};

enum class PhoneHookAction : uint8_t {
    kNone,
    kBegin,
    kHangUp,
};

// Thread-safe lifecycle gate for the phone-style REC/handset control. The
// generation invalidates a slow connection attempt after hang-up or timeout.
class PhoneCallController {
public:
    uint32_t Begin() {
        PhoneCallState expected = PhoneCallState::kIdle;
        if (!state_.compare_exchange_strong(expected, PhoneCallState::kConnecting)) {
            return 0;
        }
        return ++generation_;
    }

    bool MarkConnected(uint32_t generation) {
        if (generation == 0 || generation_.load() != generation) {
            return false;
        }
        PhoneCallState expected = PhoneCallState::kConnecting;
        return state_.compare_exchange_strong(expected, PhoneCallState::kConnected);
    }

    bool BeginHangUp() {
        auto state = state_.load();
        while (state == PhoneCallState::kConnecting || state == PhoneCallState::kConnected) {
            if (state_.compare_exchange_weak(state, PhoneCallState::kHangingUp)) {
                ++generation_;
                return true;
            }
        }
        return false;
    }

    bool Fail(uint32_t generation) {
        if (generation == 0 || generation_.load() != generation) {
            return false;
        }
        auto state = state_.load();
        while (state == PhoneCallState::kConnecting || state == PhoneCallState::kConnected) {
            if (state_.compare_exchange_weak(state, PhoneCallState::kFailed)) {
                ++generation_;
                return true;
            }
        }
        return false;
    }

    void Finish() { state_.store(PhoneCallState::kIdle); }

    PhoneCallState state() const { return state_.load(); }
    uint32_t generation() const { return generation_.load(); }

    bool IsConnecting(uint32_t generation) const {
        return generation != 0 && generation_.load() == generation &&
               state_.load() == PhoneCallState::kConnecting;
    }

    bool IsConnected() const { return state_.load() == PhoneCallState::kConnected; }

    PhoneHookAction ActionForHookState(bool off_hook) const {
        const auto state = state_.load();
        if (off_hook) {
            return state == PhoneCallState::kIdle ? PhoneHookAction::kBegin
                                                  : PhoneHookAction::kNone;
        }
        return state == PhoneCallState::kConnecting || state == PhoneCallState::kConnected
                   ? PhoneHookAction::kHangUp
                   : PhoneHookAction::kNone;
    }

private:
    std::atomic<PhoneCallState> state_{PhoneCallState::kIdle};
    std::atomic<uint32_t> generation_{0};
};
