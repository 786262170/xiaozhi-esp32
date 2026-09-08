#ifndef FALLBACK_PROTOCOL_H
#define FALLBACK_PROTOCOL_H

#include "protocol.h"

#include <atomic>
#include <memory>
#include <string>
#include <vector>

class FallbackProtocol : public Protocol {
public:
    FallbackProtocol() = default;
    ~FallbackProtocol() override;

    void AddCandidate(const std::string& name, std::unique_ptr<Protocol> protocol);

    bool Start() override;
    bool SendAudio(std::unique_ptr<AudioStreamPacket> packet) override;
    bool OpenAudioChannel() override;
    void CloseAudioChannel(bool send_goodbye = true) override;
    bool IsAudioChannelOpened() const override;

private:
    struct Candidate {
        std::string name;
        std::unique_ptr<Protocol> protocol;
        bool started = false;
    };

    std::vector<Candidate> candidates_;
    size_t active_index_ = 0;
    std::atomic<bool> switching_{false};
    std::string deferred_error_;

    Protocol* active() const;
    void BindCallbacks(Protocol* protocol);
    bool StartCandidate(size_t index);
    bool SendText(const std::string& text) override;
};

#endif  // FALLBACK_PROTOCOL_H
