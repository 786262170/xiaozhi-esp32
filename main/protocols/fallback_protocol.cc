#include "fallback_protocol.h"

#include <esp_log.h>

namespace {
constexpr char kTag[] = "Transport";
}

FallbackProtocol::~FallbackProtocol() {
    switching_.store(true);
    for (auto& candidate : candidates_) {
        candidate.protocol->CloseAudioChannel(false);
    }
}

void FallbackProtocol::AddCandidate(const std::string& name, std::unique_ptr<Protocol> protocol) {
    if (protocol == nullptr) {
        return;
    }
    BindCallbacks(protocol.get());
    candidates_.push_back(Candidate{.name = name, .protocol = std::move(protocol)});
}

Protocol* FallbackProtocol::active() const {
    return active_index_ < candidates_.size() ? candidates_[active_index_].protocol.get() : nullptr;
}

void FallbackProtocol::BindCallbacks(Protocol* protocol) {
    protocol->OnIncomingAudio([this, protocol](std::unique_ptr<AudioStreamPacket> packet) {
        if (protocol == active() && on_incoming_audio_) {
            on_incoming_audio_(std::move(packet));
        }
    });
    protocol->OnIncomingJson([this, protocol](const cJSON* root) {
        if (protocol == active() && on_incoming_json_) {
            on_incoming_json_(root);
        }
    });
    protocol->OnAudioChannelClosed([this, protocol]() {
        if (protocol == active() && !switching_.load() && on_audio_channel_closed_) {
            on_audio_channel_closed_();
        }
    });
    protocol->OnNetworkError([this, protocol](const std::string& message) {
        if (protocol != active()) {
            return;
        }
        if (switching_.load()) {
            deferred_error_ = message;
        } else if (on_network_error_) {
            on_network_error_(message);
        }
    });
    protocol->OnDisconnected([this, protocol]() {
        if (protocol == active() && !switching_.load() && on_disconnected_) {
            on_disconnected_();
        }
    });
}

bool FallbackProtocol::StartCandidate(size_t index) {
    if (index >= candidates_.size()) {
        return false;
    }
    active_index_ = index;
    auto& candidate = candidates_[index];
    if (candidate.started) {
        return true;
    }
    ESP_LOGI(kTag, "Starting transport %s", candidate.name.c_str());
    candidate.started = candidate.protocol->Start();
    return candidate.started;
}

bool FallbackProtocol::Start() {
    for (size_t index = 0; index < candidates_.size(); ++index) {
        if (StartCandidate(index)) {
            return true;
        }
        ESP_LOGW(kTag, "Transport %s failed to start", candidates_[index].name.c_str());
    }
    return false;
}

bool FallbackProtocol::OpenAudioChannel() {
    if (IsAudioChannelOpened()) {
        return true;
    }

    switching_.store(true);
    deferred_error_.clear();
    for (size_t index = 0; index < candidates_.size(); ++index) {
        if (!StartCandidate(index)) {
            continue;
        }
        auto& candidate = candidates_[index];
        ESP_LOGI(kTag, "Opening transport %s", candidate.name.c_str());
        if (candidate.protocol->OpenAudioChannel()) {
            server_sample_rate_ = candidate.protocol->server_sample_rate();
            server_frame_duration_ = candidate.protocol->server_frame_duration();
            session_id_ = candidate.protocol->session_id();
            error_occurred_ = false;
            switching_.store(false);
            ESP_LOGI(kTag, "Selected transport %s", candidate.name.c_str());
            if (on_audio_channel_opened_) {
                on_audio_channel_opened_();
            }
            if (on_connected_) {
                on_connected_();
            }
            return true;
        }
        ESP_LOGW(kTag, "Transport %s failed, trying fallback", candidate.name.c_str());
        candidate.protocol->CloseAudioChannel(false);
    }
    switching_.store(false);
    error_occurred_ = true;
    if (on_network_error_ && !deferred_error_.empty()) {
        on_network_error_(deferred_error_);
    }
    return false;
}

void FallbackProtocol::CloseAudioChannel(bool send_goodbye) {
    Protocol* protocol = active();
    if (protocol == nullptr) {
        return;
    }
    const bool was_open = protocol->IsAudioChannelOpened();
    switching_.store(true);
    protocol->CloseAudioChannel(send_goodbye);
    switching_.store(false);
    if (was_open && on_audio_channel_closed_) {
        on_audio_channel_closed_();
    }
}

bool FallbackProtocol::IsAudioChannelOpened() const {
    Protocol* protocol = active();
    return protocol != nullptr && protocol->IsAudioChannelOpened();
}

bool FallbackProtocol::SendAudio(std::unique_ptr<AudioStreamPacket> packet) {
    Protocol* protocol = active();
    return protocol != nullptr && protocol->SendAudio(std::move(packet));
}

bool FallbackProtocol::SendText(const std::string& text) {
    Protocol* protocol = active();
    return protocol != nullptr && protocol->SendTextMessage(text);
}
