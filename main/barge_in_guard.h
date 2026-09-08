#ifndef _BARGE_IN_GUARD_H_
#define _BARGE_IN_GUARD_H_

#include <stdint.h>

// A short, non-sliding guard window for local VAD barge-in. The first
// assistant audio frame starts the window; later frames must not extend it.
// GCC/Clang atomic builtins keep the protocol callback and main event task
// synchronized without requiring a lock.
class BargeInGuard {
public:
    void ResetForAssistantTurn() {
        __atomic_store_n(&block_until_ms_, 0, __ATOMIC_RELEASE);
    }

    bool MarkFirstAssistantAudio(uint32_t now_ms, uint32_t guard_ms) {
        uint32_t deadline_ms = now_ms + guard_ms;
        if (deadline_ms == 0) {
            deadline_ms = 1;
        }

        uint32_t expected = 0;
        return __atomic_compare_exchange_n(&block_until_ms_, &expected, deadline_ms,
                                           false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
    }

    bool IsBlocked(uint32_t now_ms) const {
        const uint32_t deadline_ms =
            __atomic_load_n(&block_until_ms_, __ATOMIC_ACQUIRE);
        return deadline_ms != 0 &&
               static_cast<int32_t>(deadline_ms - now_ms) > 0;
    }

private:
    alignas(4) uint32_t block_until_ms_ = 0;
};

#endif  // _BARGE_IN_GUARD_H_
