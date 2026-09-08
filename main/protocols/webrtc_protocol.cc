#include "webrtc_protocol.h"

#include "application.h"
#include "assets/lang_config.h"
#include "board.h"
#include "settings.h"
#include "system_info.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <ctime>

#include <esp_audio_dec_default.h>
#include <esp_audio_enc_default.h>
#include <esp_err.h>
#include <esp_hmac.h>
#include <esp_log.h>
#include <esp_random.h>
#include <psa/crypto.h>
#include <soc/soc_caps.h>

namespace {
constexpr char kTag[] = "WebRTC";
constexpr char kSessionPath[] = "/xiaozhi/rtc/session";
constexpr char kKeyId[] = "pre-secret-key";
constexpr char kRequestBody[] = "{}";

std::string Hex(const uint8_t* data, size_t size) {
    static constexpr char kDigits[] = "0123456789abcdef";
    std::string result(size * 2, '0');
    for (size_t i = 0; i < size; ++i) {
        result[i * 2] = kDigits[data[i] >> 4];
        result[i * 2 + 1] = kDigits[data[i] & 0x0f];
    }
    return result;
}

std::string JsonString(const cJSON* object, const char* key, const std::string& fallback = {}) {
    cJSON* value = cJSON_GetObjectItem(object, key);
    return cJSON_IsString(value) ? value->valuestring : fallback;
}
}  // namespace

WebRTCProtocol::WebRTCProtocol(AudioService& audio_service) : audio_service_(audio_service) {
    event_group_ = xEventGroupCreate();
}

WebRTCProtocol::~WebRTCProtocol() {
    CloseAudioChannel(false);
    if (event_group_ != nullptr) {
        vEventGroupDelete(event_group_);
        event_group_ = nullptr;
    }
}

bool WebRTCProtocol::Start() {
    static bool initialized = false;
    if (initialized) {
        return true;
    }
    const psa_status_t crypto_status = psa_crypto_init();
    if (crypto_status != PSA_SUCCESS) {
        ESP_LOGE(kTag, "Failed to initialize PSA Crypto: %ld", static_cast<long>(crypto_status));
        return false;
    }
    if (livekit_system_init() != LIVEKIT_ERR_NONE) {
        ESP_LOGE(kTag, "Failed to initialize LiveKit system");
        return false;
    }
    esp_audio_enc_register_default();
    esp_audio_dec_register_default();
    initialized = true;
    return true;
}

bool WebRTCProtocol::FetchSessionCredentials() {
    Settings settings("webrtc", false);
    std::string session_url = settings.GetString("session_url");
    connect_timeout_ms_ = settings.GetInt("timeout_ms", 8000);
    connect_timeout_ms_ = std::clamp(connect_timeout_ms_, 3000, 30000);
    if (session_url.empty()) {
        ESP_LOGE(kTag, "WebRTC session URL is empty");
        return false;
    }

    const std::string device_id = SystemInfo::GetMacAddress();
    const std::string client_id = Board::GetInstance().GetUuid();
    const int64_t timestamp = static_cast<int64_t>(std::time(nullptr));
    if (timestamp < 1700000000) {
        ESP_LOGE(kTag, "System time is not synchronized; refusing signed WebRTC bootstrap");
        return false;
    }

    std::array<uint8_t, 16> nonce_bytes{};
    esp_fill_random(nonce_bytes.data(), nonce_bytes.size());
    const std::string nonce = Hex(nonce_bytes.data(), nonce_bytes.size());

    std::array<uint8_t, 32> body_digest{};
    size_t body_digest_size = 0;
    if (psa_hash_compute(PSA_ALG_SHA_256, reinterpret_cast<const uint8_t*>(kRequestBody),
                         strlen(kRequestBody), body_digest.data(), body_digest.size(),
                         &body_digest_size) != PSA_SUCCESS ||
        body_digest_size != body_digest.size()) {
        ESP_LOGE(kTag, "Failed to hash WebRTC session request");
        return false;
    }
    const std::string canonical = std::string("POST\n") + kSessionPath + "\n" + device_id + "\n" +
                                  client_id + "\n" + kKeyId + "\n" + std::to_string(timestamp) +
                                  "\n" + nonce + "\n" + Hex(body_digest.data(), body_digest.size());

#if SOC_HMAC_SUPPORTED
    std::array<uint8_t, 32> signature{};
    esp_err_t hmac_result =
        esp_hmac_calculate(HMAC_KEY0, reinterpret_cast<const uint8_t*>(canonical.data()),
                           canonical.size(), signature.data());
    if (hmac_result != ESP_OK) {
        ESP_LOGE(kTag, "HMAC signing failed: %s", esp_err_to_name(hmac_result));
        return false;
    }
#else
    ESP_LOGE(kTag, "WebRTC bootstrap requires an ESP HMAC peripheral");
    return false;
#endif

    auto network = Board::GetInstance().GetNetwork();
    auto http = network->CreateHttp(0);
    if (http == nullptr) {
        ESP_LOGE(kTag, "Failed to create HTTP client");
        return false;
    }
    http->SetTimeout(connect_timeout_ms_);
    http->SetHeader("Content-Type", "application/json");
    http->SetHeader("Device-Id", device_id);
    http->SetHeader("Client-Id", client_id);
    http->SetHeader("X-RTC-Key-Id", kKeyId);
    http->SetHeader("X-RTC-Timestamp", std::to_string(timestamp));
    http->SetHeader("X-RTC-Nonce", nonce);
#if SOC_HMAC_SUPPORTED
    http->SetHeader("X-RTC-Signature", Hex(signature.data(), signature.size()));
#endif
    http->SetContent(std::string(kRequestBody));

    if (!http->Open("POST", session_url)) {
        ESP_LOGE(kTag, "WebRTC session request failed: 0x%x", http->GetLastError());
        return false;
    }
    const int status = http->GetStatusCode();
    std::string response = http->ReadAll();
    http->Close();
    if (status != 200) {
        ESP_LOGE(kTag, "WebRTC session request returned HTTP %d: %s", status, response.c_str());
        return false;
    }
    return ParseSessionCredentials(response);
}

bool WebRTCProtocol::ParseSessionCredentials(const std::string& response) {
    cJSON* root = cJSON_ParseWithLength(response.data(), response.size());
    if (!cJSON_IsObject(root)) {
        ESP_LOGE(kTag, "Invalid WebRTC session response");
        cJSON_Delete(root);
        return false;
    }

    const std::string provider = JsonString(root, "provider");
    SessionCredentials parsed;
    parsed.url = JsonString(root, "url");
    parsed.token = JsonString(root, "token");
    parsed.room = JsonString(root, "room");
    parsed.identity = JsonString(root, "identity");
    parsed.server_identity = JsonString(root, "server_identity");

    cJSON* topics = cJSON_GetObjectItem(root, "topics");
    if (cJSON_IsObject(topics)) {
        parsed.command_topic = JsonString(topics, "command");
        parsed.event_topic = JsonString(topics, "event");
    }
    cJSON* audio = cJSON_GetObjectItem(root, "audio");
    std::string codec;
    if (cJSON_IsObject(audio)) {
        codec = JsonString(audio, "codec");
        cJSON* sample_rate = cJSON_GetObjectItem(audio, "sample_rate");
        cJSON* channels = cJSON_GetObjectItem(audio, "channels");
        cJSON* frame_duration = cJSON_GetObjectItem(audio, "frame_duration_ms");
        if (cJSON_IsNumber(sample_rate)) {
            parsed.sample_rate = sample_rate->valueint;
        }
        if (cJSON_IsNumber(channels)) {
            parsed.channels = channels->valueint;
        }
        if (cJSON_IsNumber(frame_duration)) {
            parsed.frame_duration_ms = frame_duration->valueint;
        }
    }
    cJSON_Delete(root);

    if (provider != "livekit" || parsed.url.empty() || parsed.token.empty() ||
        parsed.room.empty() || parsed.identity.empty() || parsed.server_identity.empty() ||
        parsed.command_topic.empty() || parsed.event_topic.empty() || codec != "opus" ||
        parsed.sample_rate != 16000 || parsed.channels != 1 || parsed.frame_duration_ms != 20) {
        ESP_LOGE(kTag, "Incomplete or unsupported WebRTC session profile");
        return false;
    }
    credentials_ = std::move(parsed);
    return true;
}

bool WebRTCProtocol::OpenAudioChannel() {
    if (IsAudioChannelOpened()) {
        return true;
    }
    CloseAudioChannel(false);
    error_occurred_ = false;
    closing_.store(false);
    xEventGroupClearBits(event_group_, kConnectedEvent | kServerReadyEvent | kServerHelloEvent |
                                           kFailedEvent | kDisconnectedEvent);

    if (!Start() || !FetchSessionCredentials()) {
        return false;
    }

    audio_bridge_ = std::make_unique<WebRTCAudioBridge>(audio_service_);
    if (!audio_bridge_->Initialize()) {
        audio_bridge_.reset();
        return false;
    }

    livekit_room_options_t room_options = {
        .publish =
            {
                .kind = LIVEKIT_MEDIA_TYPE_AUDIO,
                .audio_encode =
                    {
                        .codec = LIVEKIT_AUDIO_CODEC_OPUS,
                        .sample_rate = static_cast<uint32_t>(credentials_.sample_rate),
                        .channel_count = static_cast<uint8_t>(credentials_.channels),
                    },
                .capturer = audio_bridge_->capturer(),
            },
        .subscribe =
            {
                .kind = LIVEKIT_MEDIA_TYPE_AUDIO,
                .renderer = audio_bridge_->renderer(),
            },
        .on_state_changed = OnStateChanged,
        .on_data_received = OnDataReceived,
        .on_participant_info = OnParticipantInfo,
        .ctx = this,
    };
    if (livekit_room_create(&room_, &room_options) != LIVEKIT_ERR_NONE || room_ == nullptr) {
        ESP_LOGE(kTag, "Failed to create LiveKit room");
        CloseAudioChannel(false);
        return false;
    }
    if (livekit_room_connect(room_, credentials_.url.c_str(), credentials_.token.c_str()) !=
        LIVEKIT_ERR_NONE) {
        ESP_LOGE(kTag, "Failed to start LiveKit room connection");
        CloseAudioChannel(false);
        return false;
    }

    const EventBits_t ready_bits = kConnectedEvent | kServerReadyEvent;
    EventBits_t bits = xEventGroupWaitBits(event_group_, ready_bits | kFailedEvent, pdFALSE,
                                           pdFALSE, pdMS_TO_TICKS(connect_timeout_ms_));
    if ((bits & ready_bits) != ready_bits || (bits & kFailedEvent)) {
        ESP_LOGE(kTag, "LiveKit room or server participant did not become ready");
        CloseAudioChannel(false);
        return false;
    }
    if (!SendClientHello()) {
        CloseAudioChannel(false);
        return false;
    }
    bits = xEventGroupWaitBits(event_group_, kServerHelloEvent | kFailedEvent, pdTRUE, pdFALSE,
                               pdMS_TO_TICKS(connect_timeout_ms_));
    if (!(bits & kServerHelloEvent) || (bits & kFailedEvent)) {
        ESP_LOGE(kTag, "Timed out waiting for WebRTC server hello");
        CloseAudioChannel(false);
        return false;
    }

    last_incoming_time_ = std::chrono::steady_clock::now();
    if (on_audio_channel_opened_) {
        on_audio_channel_opened_();
    }
    if (on_connected_) {
        on_connected_();
    }
    ESP_LOGI(kTag, "WebRTC channel opened: room=%s", credentials_.room.c_str());
    return true;
}

bool WebRTCProtocol::SendClientHello() {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "hello");
    cJSON_AddNumberToObject(root, "version", 1);
    cJSON* features = cJSON_CreateObject();
    cJSON_AddBoolToObject(features, "mcp", true);
    cJSON_AddBoolToObject(features, "phone_hangup", true);
#if CONFIG_DUPLEX_PLAYBACK_CONTROL
    cJSON_AddBoolToObject(features, "playback_control_v1", true);
#endif
#if CONFIG_LISTENER_FEEDBACK
    cJSON_AddBoolToObject(features, "listener_feedback_v1", true);
#endif
#if CONFIG_USE_DEVICE_AEC
    cJSON_AddBoolToObject(features, "aec", true);
#endif
    cJSON_AddItemToObject(root, "features", features);
    AddTextFontCapabilities(root);
    cJSON_AddStringToObject(root, "transport", "webrtc");
    cJSON* audio_params = cJSON_CreateObject();
    cJSON_AddStringToObject(audio_params, "format", "opus");
    cJSON_AddNumberToObject(audio_params, "sample_rate", credentials_.sample_rate);
    cJSON_AddNumberToObject(audio_params, "channels", credentials_.channels);
    cJSON_AddNumberToObject(audio_params, "frame_duration", credentials_.frame_duration_ms);
    cJSON_AddItemToObject(root, "audio_params", audio_params);
    char* json = cJSON_PrintUnformatted(root);
    std::string message = json == nullptr ? std::string() : std::string(json);
    cJSON_free(json);
    cJSON_Delete(root);
    return !message.empty() && SendText(message);
}

bool WebRTCProtocol::SendText(const std::string& text) {
    if (room_ == nullptr || livekit_room_get_state(room_) != LIVEKIT_CONNECTION_STATE_CONNECTED) {
        return false;
    }
    livekit_data_payload_t payload = {
        .bytes = reinterpret_cast<uint8_t*>(const_cast<char*>(text.data())),
        .size = text.size(),
    };
    char* destinations[] = {const_cast<char*>(credentials_.server_identity.c_str())};
    livekit_data_publish_options_t options = {
        .payload = &payload,
        .topic = const_cast<char*>(credentials_.command_topic.c_str()),
        .lossy = false,
        .destination_identities = credentials_.server_identity.empty() ? nullptr : destinations,
        .destination_identities_count = credentials_.server_identity.empty() ? 0 : 1,
    };
    if (livekit_room_publish_data(room_, &options) != LIVEKIT_ERR_NONE) {
        ESP_LOGE(kTag, "Failed to publish WebRTC command");
        return false;
    }
    return true;
}

bool WebRTCProtocol::SendAudio(std::unique_ptr<AudioStreamPacket> packet) {
    (void)packet;
    // LiveKit owns Opus encoding and reads post-AFE PCM from WebRTCAudioBridge.
    return IsAudioChannelOpened();
}

bool WebRTCProtocol::IsAudioChannelOpened() const {
    return room_ != nullptr &&
           livekit_room_get_state(room_) == LIVEKIT_CONNECTION_STATE_CONNECTED &&
           !error_occurred_ && !IsTimeout();
}

void WebRTCProtocol::CloseAudioChannel(bool send_goodbye) {
    if (room_ == nullptr && audio_bridge_ == nullptr) {
        return;
    }
    closing_.store(true);
    if (send_goodbye && room_ != nullptr && !session_id_.empty()) {
        SendText("{\"session_id\":\"" + session_id_ + "\",\"type\":\"goodbye\"}");
    }
    if (room_ != nullptr) {
        livekit_room_close(room_);
        xEventGroupWaitBits(event_group_, kDisconnectedEvent, pdTRUE, pdFALSE, pdMS_TO_TICKS(3000));
        livekit_room_destroy(room_);
        room_ = nullptr;
    }
    audio_bridge_.reset();
    credentials_ = {};
    closing_.store(false);
}

void WebRTCProtocol::OnStateChanged(livekit_connection_state_t state, void* context) {
    auto* protocol = static_cast<WebRTCProtocol*>(context);
    ESP_LOGI(kTag, "Room state changed: %s", livekit_connection_state_str(state));
    switch (state) {
        case LIVEKIT_CONNECTION_STATE_CONNECTED:
            xEventGroupSetBits(protocol->event_group_, kConnectedEvent);
            break;
        case LIVEKIT_CONNECTION_STATE_FAILED: {
            livekit_failure_reason_t reason =
                protocol->room_ == nullptr ? LIVEKIT_FAILURE_REASON_OTHER
                                           : livekit_room_get_failure_reason(protocol->room_);
            ESP_LOGE(kTag, "LiveKit connection failed: %s", livekit_failure_reason_str(reason));
            xEventGroupSetBits(protocol->event_group_, kFailedEvent);
            break;
        }
        case LIVEKIT_CONNECTION_STATE_DISCONNECTED:
            xEventGroupSetBits(protocol->event_group_, kDisconnectedEvent);
            if (!protocol->closing_.load() && protocol->on_audio_channel_closed_) {
                protocol->on_audio_channel_closed_();
            }
            if (!protocol->closing_.load() && protocol->on_disconnected_) {
                protocol->on_disconnected_();
            }
            break;
        default:
            break;
    }
}

void WebRTCProtocol::OnParticipantInfo(const livekit_participant_info_t* info, void* context) {
    if (info == nullptr || info->identity == nullptr) {
        return;
    }
    auto* protocol = static_cast<WebRTCProtocol*>(context);
    const bool expected_server = protocol->credentials_.server_identity == info->identity;
    if (expected_server && info->state == LIVEKIT_PARTICIPANT_STATE_ACTIVE) {
        xEventGroupSetBits(protocol->event_group_, kServerReadyEvent);
    }
}

void WebRTCProtocol::OnDataReceived(const livekit_data_received_t* data, void* context) {
    if (data == nullptr || data->payload.bytes == nullptr || data->payload.size == 0) {
        return;
    }
    auto* protocol = static_cast<WebRTCProtocol*>(context);
    if (data->topic == nullptr || protocol->credentials_.event_topic != data->topic) {
        return;
    }
    if (!protocol->credentials_.server_identity.empty() &&
        (data->sender_identity == nullptr ||
         protocol->credentials_.server_identity != data->sender_identity)) {
        ESP_LOGW(kTag, "Ignoring event from unexpected participant %s",
                 data->sender_identity == nullptr ? "<unknown>" : data->sender_identity);
        return;
    }

    cJSON* root = cJSON_ParseWithLength(reinterpret_cast<const char*>(data->payload.bytes),
                                        data->payload.size);
    if (root == nullptr) {
        ESP_LOGW(kTag, "Ignoring malformed WebRTC event");
        return;
    }
    cJSON* type = cJSON_GetObjectItem(root, "type");
    if (cJSON_IsString(type) && strcmp(type->valuestring, "hello") == 0) {
        protocol->ParseServerHello(root);
    } else if (protocol->on_incoming_json_) {
        protocol->on_incoming_json_(root);
    }
    protocol->last_incoming_time_ = std::chrono::steady_clock::now();
    cJSON_Delete(root);
}

void WebRTCProtocol::ParseServerHello(const cJSON* root) {
    cJSON* transport = cJSON_GetObjectItem(root, "transport");
    if (!cJSON_IsString(transport) || strcmp(transport->valuestring, "webrtc") != 0) {
        ESP_LOGE(kTag, "Invalid WebRTC server hello transport");
        xEventGroupSetBits(event_group_, kFailedEvent);
        return;
    }
    cJSON* session_id = cJSON_GetObjectItem(root, "session_id");
    if (cJSON_IsString(session_id)) {
        session_id_ = session_id->valuestring;
    }
    cJSON* audio_params = cJSON_GetObjectItem(root, "audio_params");
    if (cJSON_IsObject(audio_params)) {
        cJSON* sample_rate = cJSON_GetObjectItem(audio_params, "sample_rate");
        cJSON* frame_duration = cJSON_GetObjectItem(audio_params, "frame_duration");
        if (cJSON_IsNumber(sample_rate)) {
            server_sample_rate_ = sample_rate->valueint;
        }
        if (cJSON_IsNumber(frame_duration)) {
            server_frame_duration_ = frame_duration->valueint;
        }
    }
    xEventGroupSetBits(event_group_, kServerHelloEvent);
}
