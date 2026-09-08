#ifndef WEBRTC_PROTOCOL_H
#define WEBRTC_PROTOCOL_H

#include "protocol.h"
#include "webrtc_audio_bridge.h"

#include <atomic>
#include <memory>
#include <string>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <livekit.h>

class WebRTCProtocol : public Protocol {
public:
    explicit WebRTCProtocol(AudioService& audio_service);
    ~WebRTCProtocol() override;

    bool Start() override;
    bool SendAudio(std::unique_ptr<AudioStreamPacket> packet) override;
    bool OpenAudioChannel() override;
    void CloseAudioChannel(bool send_goodbye = true) override;
    bool IsAudioChannelOpened() const override;

private:
    struct SessionCredentials {
        std::string url;
        std::string token;
        std::string room;
        std::string identity;
        std::string server_identity;
        std::string command_topic;
        std::string event_topic;
        int sample_rate = 16000;
        int channels = 1;
        int frame_duration_ms = 20;
    };

    static constexpr EventBits_t kConnectedEvent = 1 << 0;
    static constexpr EventBits_t kServerReadyEvent = 1 << 1;
    static constexpr EventBits_t kServerHelloEvent = 1 << 2;
    static constexpr EventBits_t kFailedEvent = 1 << 3;
    static constexpr EventBits_t kDisconnectedEvent = 1 << 4;

    AudioService& audio_service_;
    EventGroupHandle_t event_group_ = nullptr;
    std::unique_ptr<WebRTCAudioBridge> audio_bridge_;
    livekit_room_handle_t room_ = nullptr;
    SessionCredentials credentials_;
    std::atomic<bool> closing_{false};
    int connect_timeout_ms_ = 8000;

    bool FetchSessionCredentials();
    bool ParseSessionCredentials(const std::string& response);
    bool SendClientHello();
    void ParseServerHello(const cJSON* root);
    bool SendText(const std::string& text) override;

    static void OnStateChanged(livekit_connection_state_t state, void* context);
    static void OnDataReceived(const livekit_data_received_t* data, void* context);
    static void OnParticipantInfo(const livekit_participant_info_t* info, void* context);
};

#endif  // WEBRTC_PROTOCOL_H
