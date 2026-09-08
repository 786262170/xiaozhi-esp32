#include "application.h"
#include "assets.h"
#include "assets/lang_config.h"
#include "audio_codec.h"
#include "board.h"
#include "display.h"
#include "fallback_protocol.h"
#include "mcp_server.h"
#include "mqtt_protocol.h"
#include "settings.h"
#include "system_info.h"
#include "text_glyph_payload.h"
#include "websocket_protocol.h"
#if CONFIG_USE_WEBRTC
#include <livekit.h>
#include "webrtc_protocol.h"
#endif

#include <driver/gpio.h>
#include <esp_log.h>
#include <arpa/inet.h>
#include <cJSON.h>
#include <algorithm>
#include <cstring>

#define TAG "Application"

namespace {
constexpr int64_t kPhoneCallTimeoutUs = 8 * 1000 * 1000;
constexpr int64_t kPhoneHangupTimeoutUs = 4 * 1000 * 1000;
// The selected A-version intro contains a 120 ms lead-in and two ringback
// bursts separated by 880 ms. Starting the retry at 4.9 seconds preserves
// the same natural pause after the second burst when the server is still busy.
constexpr int64_t kPhoneRingbackCadenceUs = 4900 * 1000;
constexpr uint32_t kPhoneConnectTaskStackSize = 8192;
// Keep TLS/WebSocket setup below the Opus codec task (priority 2). Otherwise
// the connection handshake can starve local ringback decoding and make the
// second burst stutter even though the embedded Ogg asset is continuous.
constexpr UBaseType_t kPhoneConnectTaskPriority = 1;
#if CONFIG_DUPLEX_PLAYBACK_CONTROL
constexpr size_t kMaxPlaybackOutputIdLength = 128;
constexpr size_t kMaxPlaybackActionLength = 16;
constexpr size_t kMaxPlaybackReasonLength = 64;
#endif
#if CONFIG_LISTENER_FEEDBACK
constexpr size_t kMaxFeedbackClipIdLength = 64;
#endif

extern const char ogg_phone_dial_start[] asm("_binary_phone_dial_ogg_start");
extern const char ogg_phone_dial_end[] asm("_binary_phone_dial_ogg_end");
const std::string_view kPhoneCallingIntroSound{
    static_cast<const char*>(ogg_phone_dial_start),
    static_cast<size_t>(ogg_phone_dial_end - ogg_phone_dial_start)};

#if CONFIG_LISTENER_FEEDBACK
extern const char ogg_neutral_ack_1_start[] asm("_binary_neutral_ack_1_ogg_start");
extern const char ogg_neutral_ack_1_end[] asm("_binary_neutral_ack_1_ogg_end");
extern const char ogg_neutral_ack_2_start[] asm("_binary_neutral_ack_2_ogg_start");
extern const char ogg_neutral_ack_2_end[] asm("_binary_neutral_ack_2_ogg_end");
extern const char ogg_neutral_ack_3_start[] asm("_binary_neutral_ack_3_ogg_start");
extern const char ogg_neutral_ack_3_end[] asm("_binary_neutral_ack_3_ogg_end");

const std::string_view kNeutralAck1{
    static_cast<const char*>(ogg_neutral_ack_1_start),
    static_cast<size_t>(ogg_neutral_ack_1_end - ogg_neutral_ack_1_start)};
const std::string_view kNeutralAck2{
    static_cast<const char*>(ogg_neutral_ack_2_start),
    static_cast<size_t>(ogg_neutral_ack_2_end - ogg_neutral_ack_2_start)};
const std::string_view kNeutralAck3{
    static_cast<const char*>(ogg_neutral_ack_3_start),
    static_cast<size_t>(ogg_neutral_ack_3_end - ogg_neutral_ack_3_start)};

std::string_view ListenerFeedbackSound(const char* clip_id) {
    if (clip_id == nullptr) {
        return {};
    }
    if (strcmp(clip_id, "neutral_ack_1") == 0) {
        return kNeutralAck1;
    }
    if (strcmp(clip_id, "neutral_ack_2") == 0) {
        return kNeutralAck2;
    }
    if (strcmp(clip_id, "neutral_ack_3") == 0) {
        return kNeutralAck3;
    }
    return {};
}
#endif
}  // namespace

#if CONFIG_VOICE_LATENCY_METRICS
static void LogVoiceMetric(const char* event, uint32_t turn_id) {
    ESP_LOGI(TAG, "VOICE_METRIC event=%s turn_id=%lu ts_mono_ms=%lld", event,
             static_cast<unsigned long>(turn_id),
             static_cast<long long>(esp_timer_get_time() / 1000));
}
#endif

Application::Application() {
    event_group_ = xEventGroupCreate();

#if CONFIG_USE_DEVICE_AEC && CONFIG_USE_SERVER_AEC
#error "CONFIG_USE_DEVICE_AEC and CONFIG_USE_SERVER_AEC cannot be enabled at the same time"
#elif CONFIG_USE_DEVICE_AEC
    aec_mode_ = kAecOnDeviceSide;
#elif CONFIG_USE_SERVER_AEC
    aec_mode_ = kAecOnServerSide;
#else
    aec_mode_ = kAecOff;
#endif

    esp_timer_create_args_t clock_timer_args = {.callback =
                                                    [](void* arg) {
                                                        Application* app = (Application*)arg;
                                                        xEventGroupSetBits(app->event_group_,
                                                                           MAIN_EVENT_CLOCK_TICK);
                                                    },
                                                .arg = this,
                                                .dispatch_method = ESP_TIMER_TASK,
                                                .name = "clock_timer",
                                                .skip_unhandled_events = true};
    esp_timer_create(&clock_timer_args, &clock_timer_handle_);
}

Application::~Application() {
    if (clock_timer_handle_ != nullptr) {
        esp_timer_stop(clock_timer_handle_);
        esp_timer_delete(clock_timer_handle_);
    }
    vEventGroupDelete(event_group_);
}

bool Application::SetDeviceState(DeviceState state) { return state_machine_.TransitionTo(state); }

void Application::Initialize() {
    auto& board = Board::GetInstance();
    SetDeviceState(kDeviceStateStarting);

#if CONFIG_USE_WEBRTC
    // Linking LiveKit makes esp_audio_codec allocations use media_lib's SAL.
    // Register its memory and OS adapters before AudioService opens Opus.
    auto livekit_result = livekit_system_init();
    if (livekit_result != LIVEKIT_ERR_NONE) {
        ESP_LOGE(TAG, "Failed to initialize LiveKit media adapters: %d",
                 static_cast<int>(livekit_result));
    }
#endif

    // Setup the display
    auto display = board.GetDisplay();
    display->SetupUI();
    // Print board name/version info
    display->SetChatMessage("system", SystemInfo::GetUserAgent().c_str());

    // Setup the audio service
    auto codec = board.GetAudioCodec();
    audio_service_.Initialize(codec);
    audio_service_.Start();

    AudioServiceCallbacks callbacks;
    callbacks.on_send_queue_available = [this]() {
        xEventGroupSetBits(event_group_, MAIN_EVENT_SEND_AUDIO);
    };
    callbacks.on_wake_word_detected = [this](const std::string& wake_word) {
        xEventGroupSetBits(event_group_, MAIN_EVENT_WAKE_WORD_DETECTED);
    };
    callbacks.on_vad_change = [this](bool speaking) {
        xEventGroupSetBits(event_group_, MAIN_EVENT_VAD_CHANGE);
    };
    callbacks.on_playback_drained = [this]() {
        xEventGroupSetBits(event_group_, MAIN_EVENT_PLAYBACK_DRAINED);
    };
#if CONFIG_DUPLEX_PLAYBACK_CONTROL
    callbacks.on_playback_progress =
        [this](const std::string& output_id, int64_t played_ms, int64_t buffered_ms,
               int64_t device_ts) {
            Schedule([this, output_id, played_ms, buffered_ms, device_ts]() {
                if (protocol_) {
                    protocol_->SendPlaybackProgress(output_id, played_ms, buffered_ms, device_ts);
                }
            });
        };
    callbacks.on_playback_stopped =
        [this](const std::string& output_id, int64_t played_ms, const std::string& reason,
               int64_t device_ts) {
            Schedule([this, output_id, played_ms, reason, device_ts]() {
                if (protocol_) {
                    protocol_->SendPlaybackStopped(output_id, played_ms, reason, device_ts);
                }
            });
        };
#endif
    audio_service_.SetCallbacks(callbacks);

    // Add state change listeners
    state_machine_.AddStateChangeListener([this](DeviceState old_state, DeviceState new_state) {
        xEventGroupSetBits(event_group_, MAIN_EVENT_STATE_CHANGED);
    });

    // Start the clock timer to update the status bar
    esp_timer_start_periodic(clock_timer_handle_, 1000000);

    // Add MCP common tools (only once during initialization)
    auto& mcp_server = McpServer::GetInstance();
    mcp_server.AddCommonTools();
    mcp_server.AddUserOnlyTools();

    // Set network event callback for UI updates and network state handling
    board.SetNetworkEventCallback([this](NetworkEvent event, const std::string& data) {
        auto display = Board::GetInstance().GetDisplay();

        switch (event) {
            case NetworkEvent::Scanning:
                display->ShowNotification(Lang::Strings::SCANNING_WIFI, 30000);
                xEventGroupSetBits(event_group_, MAIN_EVENT_NETWORK_DISCONNECTED);
                break;
            case NetworkEvent::Connecting: {
                if (data.empty()) {
                    // Cellular network - registering without carrier info yet
                    display->SetStatus(Lang::Strings::REGISTERING_NETWORK);
                } else {
                    // WiFi or cellular with carrier info
                    std::string msg = Lang::Strings::CONNECT_TO;
                    msg += data;
                    msg += "...";
                    display->ShowNotification(msg.c_str(), 30000);
                }
                break;
            }
            case NetworkEvent::Connected: {
                std::string msg = Lang::Strings::CONNECTED_TO;
                msg += data;
                display->ShowNotification(msg.c_str(), 30000);
                xEventGroupSetBits(event_group_, MAIN_EVENT_NETWORK_CONNECTED);
                break;
            }
            case NetworkEvent::Disconnected:
                xEventGroupSetBits(event_group_, MAIN_EVENT_NETWORK_DISCONNECTED);
                break;
            case NetworkEvent::WifiConfigModeEnter:
                // WiFi config mode enter is handled by WifiBoard internally
                break;
            case NetworkEvent::WifiConfigModeExit:
                // WiFi config mode exit is handled by WifiBoard internally
                break;
            // Cellular modem specific events
            case NetworkEvent::ModemDetecting:
                display->SetStatus(Lang::Strings::DETECTING_MODULE);
                break;
            case NetworkEvent::ModemErrorNoSim:
                Alert(Lang::Strings::ERROR, Lang::Strings::PIN_ERROR, "warning",
                      Lang::Sounds::OGG_ERR_PIN);
                break;
            case NetworkEvent::ModemErrorRegDenied:
                Alert(Lang::Strings::ERROR, Lang::Strings::REG_ERROR, "warning",
                      Lang::Sounds::OGG_ERR_REG);
                break;
            case NetworkEvent::ModemErrorInitFailed:
                Alert(Lang::Strings::ERROR, Lang::Strings::MODEM_INIT_ERROR, "warning",
                      Lang::Sounds::OGG_EXCLAMATION);
                break;
            case NetworkEvent::ModemErrorTimeout:
                display->SetStatus(Lang::Strings::REGISTERING_NETWORK);
                break;
        }
    });

    // Start network asynchronously
    board.StartNetwork();

    // Update the status bar immediately to show the network state
    display->UpdateStatusBar(true);
}

void Application::Run() {
    // Set the priority of the main task to 10
    vTaskPrioritySet(nullptr, 10);

    const EventBits_t ALL_EVENTS =
        MAIN_EVENT_SCHEDULE | MAIN_EVENT_SEND_AUDIO | MAIN_EVENT_WAKE_WORD_DETECTED |
        MAIN_EVENT_VAD_CHANGE | MAIN_EVENT_CLOCK_TICK | MAIN_EVENT_ERROR |
        MAIN_EVENT_NETWORK_CONNECTED | MAIN_EVENT_NETWORK_DISCONNECTED | MAIN_EVENT_TOGGLE_CHAT |
        MAIN_EVENT_START_LISTENING | MAIN_EVENT_STOP_LISTENING | MAIN_EVENT_ACTIVATION_DONE |
        MAIN_EVENT_STATE_CHANGED | MAIN_EVENT_PLAYBACK_DRAINED | MAIN_EVENT_TOGGLE_PHONE_CALL |
        MAIN_EVENT_PHONE_HOOK_CHANGED;

    while (true) {
        auto bits = xEventGroupWaitBits(event_group_, ALL_EVENTS, pdTRUE, pdFALSE, portMAX_DELAY);

        if (bits & MAIN_EVENT_ERROR) {
            auto call_state = phone_call_controller_.state();
            if (call_state == PhoneCallState::kConnecting ||
                call_state == PhoneCallState::kConnected) {
                FailPhoneCall(phone_call_controller_.generation(), last_error_message_.c_str());
            } else if (call_state == PhoneCallState::kHangingUp ||
                       call_state == PhoneCallState::kFailed) {
                ESP_LOGI(TAG, "Ignoring duplicate network error during phone cleanup");
            } else {
                SetDeviceState(kDeviceStateIdle);
                Alert(Lang::Strings::ERROR, last_error_message_.c_str(), "cancel",
                      Lang::Sounds::OGG_EXCLAMATION);
            }
        }

        if (bits & MAIN_EVENT_NETWORK_CONNECTED) {
            HandleNetworkConnectedEvent();
        }

        if (bits & MAIN_EVENT_NETWORK_DISCONNECTED) {
            HandleNetworkDisconnectedEvent();
        }

        if (bits & MAIN_EVENT_ACTIVATION_DONE) {
            HandleActivationDoneEvent();
        }

        if (bits & MAIN_EVENT_STATE_CHANGED) {
            HandleStateChangedEvent();
        }

        if (bits & MAIN_EVENT_PLAYBACK_DRAINED) {
#if CONFIG_DUPLEX_PLAYBACK_CONTROL
            audio_service_.CompleteRemotePlaybackIfDrained();
#endif
            if (phone_call_controller_.state() == PhoneCallState::kHangingUp &&
                audio_service_.IsPlaybackIdle()) {
                if (phone_hangup_sound_playing_.load()) {
                    FinalizePhoneHangup("local hang-up sound drained");
                } else if (phone_hangup_ready_.load()) {
                    CompletePhoneHangup("farewell playback drained");
                }
            }
#if CONFIG_LOCAL_VAD_BARGE_IN && CONFIG_VOICE_LATENCY_METRICS
            if (barge_in_waiting_for_playback_drain_ && audio_service_.IsPlaybackIdle()) {
                barge_in_waiting_for_playback_drain_ = false;
                LogVoiceMetric("BARGE_IN_PLAYBACK_STOP", voice_metric_turn_id_);
            }
#endif
#if CONFIG_VOICE_LATENCY_METRICS
            if (turn_completion_waiting_for_playback_drain_ && audio_service_.IsPlaybackIdle()) {
                turn_completion_waiting_for_playback_drain_ = false;
                LogVoiceMetric("TURN_COMPLETED", voice_metric_turn_id_);
            }
#endif
            // Deferred listening start (auto mode): the playback queue has
            // drained, so it is now safe to enable voice processing.
            if (pending_listening_start_ && GetDeviceState() == kDeviceStateListening &&
                audio_service_.IsPlaybackIdle()) {
                pending_listening_start_ = false;
                StartListeningAudio();
            }
        }

        if (bits & MAIN_EVENT_TOGGLE_CHAT) {
            HandleToggleChatEvent();
        }

        if (bits & MAIN_EVENT_TOGGLE_PHONE_CALL) {
            HandleTogglePhoneCallEvent();
        }

        if (bits & MAIN_EVENT_PHONE_HOOK_CHANGED) {
            HandlePhoneHookChangedEvent();
        }

        if (bits & MAIN_EVENT_START_LISTENING) {
            HandleStartListeningEvent();
        }

        if (bits & MAIN_EVENT_STOP_LISTENING) {
            HandleStopListeningEvent();
        }

        if (bits & MAIN_EVENT_SEND_AUDIO) {
            while (auto packet = audio_service_.PopPacketFromSendQueue()) {
                if (protocol_ && !protocol_->SendAudio(std::move(packet))) {
                    // Drop the remaining packets. Leaving them in the queue would
                    // stall the Opus codec task (it waits for queue space), which in
                    // turn deadlocks the whole audio input pipeline, as no new
                    // MAIN_EVENT_SEND_AUDIO event would ever be triggered again.
                    while (audio_service_.PopPacketFromSendQueue())
                        ;
                    break;
                }
            }
        }

        if (bits & MAIN_EVENT_WAKE_WORD_DETECTED) {
            HandleWakeWordDetectedEvent();
        }

        if (bits & MAIN_EVENT_VAD_CHANGE) {
            auto state = GetDeviceState();
#if CONFIG_LISTENER_FEEDBACK
            if (audio_service_.IsVoiceDetected() &&
                audio_service_.IsListenerFeedbackPlaying()) {
                ESP_LOGI(TAG, "Near-end voice cancelled listener feedback");
                audio_service_.CancelListenerFeedback();
            }
#endif
#if CONFIG_VOICE_LATENCY_METRICS
            if (state == kDeviceStateListening) {
                if (audio_service_.IsVoiceDetected()) {
                    ++voice_metric_turn_id_;
                    LogVoiceMetric("USER_AUDIO_START", voice_metric_turn_id_);
                } else {
                    LogVoiceMetric("DEVICE_VAD_END", voice_metric_turn_id_);
                }
            }
#endif
#if CONFIG_LOCAL_VAD_BARGE_IN
            if (state == kDeviceStateSpeaking &&
                listening_mode_ == kListeningModeRealtime) {
                if (!audio_service_.IsVoiceDetected()) {
                    local_vad_barge_in_armed_ = true;
                } else if (local_vad_barge_in_armed_ && !aborted_) {
                    const uint32_t now_ms =
                        static_cast<uint32_t>(esp_timer_get_time() / 1000);
                    if (barge_in_guard_.IsBlocked(now_ms)) {
                        ESP_LOGI(TAG, "Local VAD barge-in suppressed during playback guard");
                    } else {
                        local_vad_barge_in_armed_ = false;
#if CONFIG_VOICE_LATENCY_METRICS
                        ++voice_metric_turn_id_;
                        LogVoiceMetric("USER_AUDIO_START", voice_metric_turn_id_);
#endif
#if CONFIG_ML307_LOCAL_VAD_BARGE_IN
                        ESP_LOGI(TAG, "Local VAD barge-in");
#if CONFIG_VOICE_LATENCY_METRICS
                        LogVoiceMetric("BARGE_IN_DETECTED", voice_metric_turn_id_);
                        barge_in_waiting_for_playback_drain_ = true;
                        turn_completion_waiting_for_playback_drain_ = false;
#endif
                        ml307_local_barge_in_pending_ = true;
                        audio_service_.ResetDecoder();
                        AbortSpeaking(kAbortReasonNone);
#if CONFIG_VOICE_LATENCY_METRICS
                        LogVoiceMetric("TURN_CANCELLED", voice_metric_turn_id_);
#endif
                        // Keep the modem uplink gated until the server TTS stop event.
                        // This avoids overlapping MQTT abort and UDP MIPSEND commands.
#else
                        // Wi-Fi full duplex keeps AEC audio flowing upstream. Local VAD
                        // only marks a speech candidate; mode 5 owns the semantic
                        // decision and sends TTS stop only after confirmation.
                        ESP_LOGI(TAG, "Local VAD speech candidate");
#if CONFIG_VOICE_LATENCY_METRICS
                        LogVoiceMetric("LOCAL_SPEECH_CANDIDATE", voice_metric_turn_id_);
#endif
#endif
                    }
                }
            }
#endif
            if (state == kDeviceStateListening) {
                auto led = Board::GetInstance().GetLed();
                led->OnStateChanged();
            }
        }

        if (bits & MAIN_EVENT_SCHEDULE) {
            std::unique_lock<std::mutex> lock(mutex_);
            auto tasks = std::move(main_tasks_);
            lock.unlock();
            for (auto& task : tasks) {
                task();
            }
        }

        if (bits & MAIN_EVENT_CLOCK_TICK) {
            clock_ticks_++;
            HandlePhoneCallClockTick();
            auto display = Board::GetInstance().GetDisplay();
            display->UpdateStatusBar();

            // Print debug info every 10 seconds
            if (clock_ticks_ % 10 == 0) {
                SystemInfo::PrintHeapStats();
                // SystemInfo::PrintTaskList();
                // SystemInfo::PrintTaskCpuUsage(pdMS_TO_TICKS(1000));
            }
        }
    }
}

void Application::HandleNetworkConnectedEvent() {
    ESP_LOGI(TAG, "Network connected");
    auto state = GetDeviceState();

    if (state == kDeviceStateStarting || state == kDeviceStateWifiConfiguring) {
        // Network is ready, start activation
        SetDeviceState(kDeviceStateActivating);
        if (activation_task_handle_ != nullptr) {
            ESP_LOGW(TAG, "Activation task already running");
            return;
        }

        xTaskCreate(
            [](void* arg) {
                Application* app = static_cast<Application*>(arg);
                app->ActivationTask();
                app->activation_task_handle_ = nullptr;
                vTaskDelete(NULL);
            },
            "activation", 4096 * 2, this, 2, &activation_task_handle_);
    }

    // Update the status bar immediately to show the network state
    auto display = Board::GetInstance().GetDisplay();
    display->UpdateStatusBar(true);
}

void Application::HandleNetworkDisconnectedEvent() {
    // Close current conversation when network disconnected
    auto state = GetDeviceState();
    auto call_state = phone_call_controller_.state();
    if (call_state == PhoneCallState::kConnecting ||
        call_state == PhoneCallState::kConnected) {
        FailPhoneCall(phone_call_controller_.generation(), "network disconnected");
    }
    if (state == kDeviceStateConnecting || state == kDeviceStateListening ||
        state == kDeviceStateSpeaking) {
        // OpenAudioChannel may still own the protocol from the phone connection
        // task. Its completion callback will close the stale channel safely.
        if (!phone_connect_task_running_.load()) {
            ESP_LOGI(TAG, "Closing audio channel due to network disconnection");
            protocol_->CloseAudioChannel();
        }
    }

    // Update the status bar immediately to show the network state
    auto display = Board::GetInstance().GetDisplay();
    display->UpdateStatusBar(true);
}

void Application::HandleActivationDoneEvent() {
    ESP_LOGI(TAG, "Activation done");

    SystemInfo::PrintHeapStats();
    SetDeviceState(kDeviceStateIdle);

    has_server_time_ = ota_->HasServerTime();

    auto display = Board::GetInstance().GetDisplay();
    std::string message = std::string(Lang::Strings::VERSION) + ota_->GetCurrentVersion();
    display->ShowNotification(message.c_str());
    display->SetChatMessage("system", "");

    // Release OTA object after activation is complete
    ota_.reset();
    auto& board = Board::GetInstance();
    board.SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);

    Schedule([this]() {
        // Play the success sound to indicate the device is ready
        audio_service_.PlaySound(Lang::Sounds::OGG_SUCCESS);
    });
}

void Application::ActivationTask() {
    // Create OTA object for activation process
    ota_ = std::make_unique<Ota>();

    // Check for new assets version
    CheckAssetsVersion();

    // Check for new firmware version
    CheckNewVersion();

    // Initialize the protocol
    InitializeProtocol();

    // Signal completion to main loop
    xEventGroupSetBits(event_group_, MAIN_EVENT_ACTIVATION_DONE);
}

void Application::CheckAssetsVersion() {
    // Only allow CheckAssetsVersion to be called once
    if (assets_version_checked_) {
        return;
    }
    assets_version_checked_ = true;

    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    auto& assets = Assets::GetInstance();

    if (!assets.partition_valid()) {
        ESP_LOGW(TAG, "Assets partition is disabled for board %s", BOARD_NAME);
        return;
    }

    Settings settings("assets", true);
    // Check if there is a new assets need to be downloaded
    std::string download_url = settings.GetString("download_url");

    if (!download_url.empty()) {
        settings.EraseKey("download_url");

        char message[256];
        snprintf(message, sizeof(message), Lang::Strings::FOUND_NEW_ASSETS, download_url.c_str());
        Alert(Lang::Strings::LOADING_ASSETS, message, "cloud_download", Lang::Sounds::OGG_UPGRADE);

        // Wait for the audio service to be idle for 3 seconds
        vTaskDelay(pdMS_TO_TICKS(3000));
        SetDeviceState(kDeviceStateUpgrading);
        board.SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);
        display->SetChatMessage("system", Lang::Strings::PLEASE_WAIT);

        bool success =
            assets.Download(download_url, [this, display](int progress, size_t speed) -> void {
                char buffer[32];
                snprintf(buffer, sizeof(buffer), "%d%% %uKB/s", progress, speed / 1024);
                Schedule([display, message = std::string(buffer)]() {
                    display->SetChatMessage("system", message.c_str());
                });
            });

        board.SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);
        vTaskDelay(pdMS_TO_TICKS(1000));

        if (!success) {
            Alert(Lang::Strings::ERROR, Lang::Strings::DOWNLOAD_ASSETS_FAILED, "cancel",
                  Lang::Sounds::OGG_EXCLAMATION);
            vTaskDelay(pdMS_TO_TICKS(2000));
            SetDeviceState(kDeviceStateActivating);
            return;
        }
    }

    // Apply assets
    assets.Apply();
    display->SetChatMessage("system", "");
    display->SetEmotion("robot_2");
}

void Application::CheckNewVersion() {
    const int MAX_RETRY = 2;
    int retry_count = 0;
    int retry_delay = 2;  // Initial retry delay in seconds

    auto& board = Board::GetInstance();
    while (true) {
        auto display = board.GetDisplay();
        display->SetStatus(Lang::Strings::CHECKING_NEW_VERSION);

        esp_err_t err = ota_->CheckVersion();
        if (err != ESP_OK) {
            if (ota_->HasCachedProtocolConfig()) {
                ESP_LOGW(TAG,
                         "OTA metadata refresh failed; continuing with cached public protocol "
                         "configuration, code=%d",
                         err);
                return;
            }

            retry_count++;
            if (retry_count >= MAX_RETRY) {
                ESP_LOGE(TAG, "Too many retries, exit version check");
                return;
            }

            char error_message[128];
            snprintf(error_message, sizeof(error_message), "code=%d, url=%s", err,
                     ota_->GetCheckVersionUrl().c_str());
            char buffer[256];
            snprintf(buffer, sizeof(buffer), Lang::Strings::CHECK_NEW_VERSION_FAILED, retry_delay,
                     error_message);
            Alert(Lang::Strings::ERROR, buffer, "cloud_off", Lang::Sounds::OGG_EXCLAMATION);

            ESP_LOGW(TAG, "Check new version failed, retry in %d seconds (%d/%d)", retry_delay,
                     retry_count, MAX_RETRY);
            for (int i = 0; i < retry_delay; i++) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                if (GetDeviceState() == kDeviceStateIdle) {
                    break;
                }
            }
            retry_delay *= 2;  // Double the retry delay
            continue;
        }
        retry_count = 0;
        retry_delay = 2;  // Reset retry delay

        if (ota_->HasNewVersion()) {
            if (UpgradeFirmware(ota_->GetFirmwareUrl(), ota_->GetFirmwareVersion())) {
                return;  // This line will never be reached after reboot
            }
            // If upgrade failed, continue to normal operation
        }

        // No new version, mark the current version as valid
        ota_->MarkCurrentVersionValid();
        if (!ota_->HasActivationCode() && !ota_->HasActivationChallenge()) {
            // Exit the loop if done checking new version
            break;
        }

        display->SetStatus(Lang::Strings::ACTIVATION);
        // Activation code is shown to the user and waiting for the user to input
        if (ota_->HasActivationCode()) {
            ShowActivationCode(ota_->GetActivationCode(), ota_->GetActivationMessage());
        }

        // This will block the loop until the activation is done or timeout
        for (int i = 0; i < 10; ++i) {
            ESP_LOGI(TAG, "Activating... %d/%d", i + 1, 10);
            esp_err_t err = ota_->Activate();
            if (err == ESP_OK) {
                break;
            } else if (err == ESP_ERR_TIMEOUT) {
                vTaskDelay(pdMS_TO_TICKS(3000));
            } else {
                vTaskDelay(pdMS_TO_TICKS(10000));
            }
            if (GetDeviceState() == kDeviceStateIdle) {
                break;
            }
        }
    }
}

void Application::InitializeProtocol() {
    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    auto codec = board.GetAudioCodec();

    display->SetStatus(Lang::Strings::LOADING_PROTOCOL);

    auto transports = std::make_unique<FallbackProtocol>();
    std::vector<std::string> added_transports;
    auto add_transport = [&](const std::string& name) {
        const std::string normalized_name = (name == "mqtt" || name == "udp") ? "mqtt_udp" : name;
        if (std::find(added_transports.begin(), added_transports.end(), normalized_name) !=
            added_transports.end()) {
            return;
        }
#if CONFIG_USE_WEBRTC
        if (normalized_name == "webrtc" && ota_->HasWebRTCConfig()) {
            transports->AddCandidate(normalized_name,
                                     std::make_unique<WebRTCProtocol>(audio_service_));
            added_transports.push_back(normalized_name);
            return;
        }
#endif
        if (normalized_name == "websocket" && ota_->HasWebsocketConfig()) {
            transports->AddCandidate(normalized_name, std::make_unique<WebsocketProtocol>());
            added_transports.push_back(normalized_name);
            return;
        }
        if (normalized_name == "mqtt_udp" && ota_->HasMqttConfig()) {
            transports->AddCandidate(normalized_name, std::make_unique<MqttProtocol>());
            added_transports.push_back(normalized_name);
        }
    };

    for (const auto& transport : ota_->GetTransportOrder()) {
        add_transport(transport);
    }
    if (added_transports.empty()) {
        // Preserve the legacy preference when the OTA server does not publish
        // an explicit transport policy.
        if (ota_->HasMqttConfig()) {
            add_transport("mqtt_udp");
        } else if (ota_->HasWebsocketConfig()) {
            add_transport("websocket");
        }
#if CONFIG_USE_WEBRTC
        else if (ota_->HasWebRTCConfig()) {
            add_transport("webrtc");
        }
#endif
    }
    if (added_transports.empty()) {
        ESP_LOGW(TAG, "No protocol specified in the OTA config, using MQTT");
        transports->AddCandidate("mqtt_udp", std::make_unique<MqttProtocol>());
    }
    protocol_ = std::move(transports);

    protocol_->OnConnected([this]() { DismissAlert(); });

    protocol_->OnNetworkError([this](const std::string& message) {
        last_error_message_ = message;
        xEventGroupSetBits(event_group_, MAIN_EVENT_ERROR);
    });

    protocol_->OnIncomingAudio([this](std::unique_ptr<AudioStreamPacket> packet) {
        const bool phone_audio = phone_call_controller_.IsConnected();
        const bool accept_remote_audio =
            (GetDeviceState() == kDeviceStateSpeaking || phone_audio) && !aborted_.load();
        if (accept_remote_audio) {
            const bool first_phone_audio = phone_audio && StopPhoneRingbackForRemoteAudio();
#if CONFIG_LOCAL_VAD_BARGE_IN
            const uint32_t now_ms =
                static_cast<uint32_t>(esp_timer_get_time() / 1000);
            if (barge_in_guard_.MarkFirstAssistantAudio(
                    now_ms, CONFIG_LOCAL_VAD_BARGE_IN_GUARD_MS)) {
                ESP_LOGI(TAG, "Local VAD barge-in guard started: %d ms",
                         CONFIG_LOCAL_VAD_BARGE_IN_GUARD_MS);
            }
#endif
#if CONFIG_VOICE_LATENCY_METRICS
            if (!device_first_audio_logged_) {
                device_first_audio_logged_ = true;
                LogVoiceMetric("DEVICE_FIRST_AUDIO_RECEIVED", voice_metric_turn_id_);
            }
#endif
            if (first_phone_audio) {
                ESP_LOGI(TAG, "Playing local phone pickup sound before remote greeting");
                audio_service_.PlaySound(Lang::Sounds::OGG_PHONE_CONNECT);
            }
#if CONFIG_DUPLEX_PLAYBACK_CONTROL
            std::string output_id;
            {
                std::lock_guard<std::mutex> lock(playback_protocol_mutex_);
                output_id = active_remote_output_id_;
            }
            if (!output_id.empty()) {
                audio_service_.BeginRemotePlayback(output_id);
                audio_service_.PushRemotePacketToDecodeQueue(std::move(packet), output_id);
            } else {
                audio_service_.PushPacketToDecodeQueue(std::move(packet));
            }
#else
            audio_service_.PushPacketToDecodeQueue(std::move(packet));
#endif
        }
    });

    protocol_->OnAudioChannelOpened([this, codec, &board]() {
        board.SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);
        if (protocol_->server_sample_rate() != codec->output_sample_rate()) {
            ESP_LOGW(TAG,
                     "Server sample rate %d does not match device output sample rate %d, "
                     "resampling may cause distortion",
                     protocol_->server_sample_rate(), codec->output_sample_rate());
        }
    });

    protocol_->OnAudioChannelClosed([this, &board]() {
        board.SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);
        Schedule([this]() {
            auto call_state = phone_call_controller_.state();
            const bool server_hung_up = call_state == PhoneCallState::kConnecting ||
                                        call_state == PhoneCallState::kConnected;
            const bool hangup_interrupted = call_state == PhoneCallState::kHangingUp;
            phone_ringback_active_.store(false);
            phone_hangup_ready_.store(false);
            phone_hangup_sound_playing_.store(false);
            phone_hangup_requested_us_ = 0;
#if CONFIG_DUPLEX_PLAYBACK_CONTROL
            {
                std::lock_guard<std::mutex> lock(playback_protocol_mutex_);
                active_remote_output_id_.clear();
            }
            audio_service_.ResetDecoder();
#endif
            if (server_hung_up || hangup_interrupted) {
#if !CONFIG_DUPLEX_PLAYBACK_CONTROL
                audio_service_.ResetDecoder();
#endif
                audio_service_.PlaySound(Lang::Sounds::OGG_PHONE_HANGUP);
            }
            if (!phone_connect_task_running_.load()) {
                phone_call_controller_.Finish();
            }
            auto display = Board::GetInstance().GetDisplay();
            display->SetChatMessage("system", "");
            SetDeviceState(kDeviceStateIdle);
        });
    });

    protocol_->OnIncomingJson([this, display](const cJSON* root) {
        // Parse JSON data
        auto type = cJSON_GetObjectItem(root, "type");
        if (!cJSON_IsString(type)) {
            ESP_LOGW(TAG, "Incoming JSON message has no type");
            return;
        }
        if (strcmp(type->valuestring, "tts") == 0) {
            auto state = cJSON_GetObjectItem(root, "state");
            if (!cJSON_IsString(state)) {
                return;
            }
            if (strcmp(state->valuestring, "start") == 0) {
#if CONFIG_DUPLEX_PLAYBACK_CONTROL
                std::string output_id;
                auto payload = cJSON_GetObjectItem(root, "payload");
                if (cJSON_IsObject(payload)) {
                    auto output_id_item = cJSON_GetObjectItem(payload, "output_id");
                    if (cJSON_IsString(output_id_item)) {
                        const size_t output_id_length = strnlen(
                            output_id_item->valuestring, kMaxPlaybackOutputIdLength + 1);
                        if (output_id_length <= kMaxPlaybackOutputIdLength) {
                            output_id.assign(output_id_item->valuestring, output_id_length);
                        } else {
                            ESP_LOGW(TAG, "Ignoring oversized playback output_id");
                        }
                    }
                }
                if (!output_id.empty()) {
                    {
                        std::lock_guard<std::mutex> lock(playback_protocol_mutex_);
                        active_remote_output_id_ = output_id;
                    }
                    // Register the output before the first audio packet arrives so an
                    // early duck/cancel cannot be rejected as stale.
                    audio_service_.BeginRemotePlayback(output_id);
                }
#endif
                Schedule([this]() {
                    auto call_state = phone_call_controller_.state();
                    if (call_state == PhoneCallState::kHangingUp ||
                        call_state == PhoneCallState::kFailed) {
                        return;
                    }
                    aborted_ = false;
                    SetDeviceState(kDeviceStateSpeaking);
                });
            } else if (strcmp(state->valuestring, "stop") == 0) {
#if CONFIG_DUPLEX_PLAYBACK_CONTROL
                std::string output_id;
                std::string reason = "completed";
                auto payload = cJSON_GetObjectItem(root, "payload");
                if (cJSON_IsObject(payload)) {
                    auto output_id_item = cJSON_GetObjectItem(payload, "output_id");
                    auto reason_item = cJSON_GetObjectItem(payload, "reason");
                    if (cJSON_IsString(output_id_item)) {
                        const size_t output_id_length = strnlen(
                            output_id_item->valuestring, kMaxPlaybackOutputIdLength + 1);
                        if (output_id_length <= kMaxPlaybackOutputIdLength) {
                            output_id.assign(output_id_item->valuestring, output_id_length);
                        }
                    }
                    if (cJSON_IsString(reason_item)) {
                        const size_t reason_length = strnlen(
                            reason_item->valuestring, kMaxPlaybackReasonLength + 1);
                        if (reason_length <= kMaxPlaybackReasonLength) {
                            reason.assign(reason_item->valuestring, reason_length);
                        }
                    }
                }
                {
                    std::lock_guard<std::mutex> lock(playback_protocol_mutex_);
                    if (output_id.empty()) {
                        output_id = active_remote_output_id_;
                    }
                    if (output_id == active_remote_output_id_) {
                        active_remote_output_id_.clear();
                    }
                }
                if (!output_id.empty()) {
                    audio_service_.EndRemotePlayback(output_id, reason);
                }
#endif
                Schedule([this]() {
#if CONFIG_VOICE_LATENCY_METRICS
                    turn_completion_waiting_for_playback_drain_ = true;
                    if (audio_service_.IsPlaybackIdle()) {
                        turn_completion_waiting_for_playback_drain_ = false;
                        LogVoiceMetric("TURN_COMPLETED", voice_metric_turn_id_);
                    }
#endif
                    if (phone_call_controller_.state() == PhoneCallState::kHangingUp) {
                        return;
                    }
                    if (GetDeviceState() == kDeviceStateSpeaking) {
                        if (listening_mode_ == kListeningModeManualStop) {
                            SetDeviceState(kDeviceStateIdle);
                        } else {
                            SetDeviceState(kDeviceStateListening);
                        }
                    }
                });
            } else if (strcmp(state->valuestring, "sentence_start") == 0) {
                auto text = cJSON_GetObjectItem(root, "text");
                if (cJSON_IsString(text)) {
                    std::vector<TextGlyph> glyphs;
                    uint8_t bpp = 0;
                    if (!TextGlyphPayload::Parse(root, glyphs, bpp)) {
                        glyphs.clear();
                    }
                    ESP_LOGI(TAG, "<< %s", text->valuestring);
                    Schedule([display, message = std::string(text->valuestring),
                              glyphs = std::move(glyphs), bpp]() {
                        display->AddTextGlyphs(glyphs, bpp);
                        display->SetChatMessage("assistant", message.c_str());
                    });
                }
            }
        }
#if CONFIG_DUPLEX_PLAYBACK_CONTROL
        else if (strcmp(type->valuestring, "playback_control") == 0) {
            auto payload = cJSON_GetObjectItem(root, "payload");
            auto action_item = cJSON_IsObject(payload)
                                   ? cJSON_GetObjectItem(payload, "action")
                                   : nullptr;
            auto output_id_item = cJSON_IsObject(payload)
                                      ? cJSON_GetObjectItem(payload, "output_id")
                                      : nullptr;
            if (!cJSON_IsString(action_item) || !cJSON_IsString(output_id_item)) {
                ESP_LOGW(TAG, "Invalid playback_control payload");
                return;
            }
            float target_gain_db = 0.0f;
            auto gain_item = cJSON_GetObjectItem(payload, "target_gain_db");
            if (cJSON_IsNumber(gain_item)) {
                target_gain_db = static_cast<float>(gain_item->valuedouble);
            }
            const size_t action_length = strnlen(
                action_item->valuestring, kMaxPlaybackActionLength + 1);
            const size_t output_id_length = strnlen(
                output_id_item->valuestring, kMaxPlaybackOutputIdLength + 1);
            if (action_length == 0 || action_length > kMaxPlaybackActionLength ||
                output_id_length == 0 || output_id_length > kMaxPlaybackOutputIdLength) {
                ESP_LOGW(TAG, "Rejected oversized or empty playback_control fields");
                return;
            }
            std::string action(action_item->valuestring, action_length);
            std::string output_id(output_id_item->valuestring, output_id_length);
            Schedule([this, action, output_id, target_gain_db]() {
                RemotePlaybackAction control;
                if (action == "duck") {
                    control = RemotePlaybackAction::kDuck;
                } else if (action == "pause") {
                    control = RemotePlaybackAction::kPause;
                } else if (action == "resume") {
                    control = RemotePlaybackAction::kResume;
                } else if (action == "cancel") {
                    control = RemotePlaybackAction::kCancel;
                } else {
                    ESP_LOGW(TAG, "Unsupported playback control: %s", action.c_str());
                    return;
                }
                const bool controlled =
                    audio_service_.ControlRemotePlayback(output_id, control, target_gain_db);
                if (!controlled) {
                    ESP_LOGW(TAG, "Ignored stale playback control: action=%s output_id=%s",
                             action.c_str(), output_id.c_str());
                }
                if (controlled && control == RemotePlaybackAction::kCancel) {
                    aborted_ = true;
                    std::lock_guard<std::mutex> lock(playback_protocol_mutex_);
                    if (active_remote_output_id_ == output_id) {
                        active_remote_output_id_.clear();
                    }
                }
            });
        }
#endif
#if CONFIG_LISTENER_FEEDBACK
        else if (strcmp(type->valuestring, "listener_feedback") == 0) {
            auto state = cJSON_GetObjectItem(root, "state");
            auto payload = cJSON_GetObjectItem(root, "payload");
            auto clip_id_item = cJSON_IsObject(payload)
                                    ? cJSON_GetObjectItem(payload, "clip_id")
                                    : nullptr;
            if (!cJSON_IsString(state) || strcmp(state->valuestring, "play") != 0 ||
                !cJSON_IsString(clip_id_item)) {
                ESP_LOGW(TAG, "Invalid listener_feedback payload");
                return;
            }
            const size_t clip_id_length = strnlen(
                clip_id_item->valuestring, kMaxFeedbackClipIdLength + 1);
            if (clip_id_length == 0 || clip_id_length > kMaxFeedbackClipIdLength) {
                ESP_LOGW(TAG, "Rejected oversized or empty listener feedback clip_id");
                return;
            }
            std::string clip_id(clip_id_item->valuestring, clip_id_length);
            auto sound = ListenerFeedbackSound(clip_id.c_str());
            if (sound.empty()) {
                ESP_LOGW(TAG, "Unknown listener feedback clip: %s", clip_id.c_str());
                return;
            }
            Schedule([this, clip_id, sound]() {
                if (GetDeviceState() != kDeviceStateListening ||
                    audio_service_.IsVoiceDetected()) {
                    ESP_LOGI(TAG, "Suppress listener feedback because user floor is not open");
                    return;
                }
                audio_service_.PlayListenerFeedback(clip_id, sound);
            });
        }
#endif
        else if (strcmp(type->valuestring, "phone_hangup") == 0) {
            auto state = cJSON_GetObjectItem(root, "state");
            if (cJSON_IsString(state) && strcmp(state->valuestring, "ready") == 0) {
                Schedule([this]() {
                    auto call_state = phone_call_controller_.state();
                    if (call_state == PhoneCallState::kConnected) {
                        if (!phone_call_controller_.BeginHangUp()) {
                            return;
                        }
                        phone_hangup_requested_us_ = esp_timer_get_time();
                    } else if (call_state != PhoneCallState::kHangingUp) {
                        return;
                    }
                    ESP_LOGI(TAG, "Phone farewell ready; waiting for playback drain");
                    phone_hangup_sound_playing_.store(false);
                    phone_hangup_ready_.store(true);
                    if (audio_service_.IsPlaybackIdle()) {
                        CompletePhoneHangup("farewell already drained");
                    }
                });
            }
        } else if (strcmp(type->valuestring, "stt") == 0) {
            auto text = cJSON_GetObjectItem(root, "text");
            if (cJSON_IsString(text)) {
                std::vector<TextGlyph> glyphs;
                uint8_t bpp = 0;
                if (!TextGlyphPayload::Parse(root, glyphs, bpp)) {
                    glyphs.clear();
                }
                ESP_LOGI(TAG, ">> %s", text->valuestring);
                Schedule([display, message = std::string(text->valuestring),
                          glyphs = std::move(glyphs), bpp]() {
                    display->AddTextGlyphs(glyphs, bpp);
                    display->SetChatMessage("user", message.c_str());
                });
            }
        } else if (strcmp(type->valuestring, "llm") == 0) {
            auto emotion = cJSON_GetObjectItem(root, "emotion");
            if (cJSON_IsString(emotion)) {
                Schedule([display, emotion_str = std::string(emotion->valuestring)]() {
                    display->SetEmotion(emotion_str.c_str());
                });
            }
        } else if (strcmp(type->valuestring, "mcp") == 0) {
            auto payload = cJSON_GetObjectItem(root, "payload");
            if (cJSON_IsObject(payload)) {
                McpServer::GetInstance().ParseMessage(payload);
            }
        } else if (strcmp(type->valuestring, "system") == 0) {
            auto command = cJSON_GetObjectItem(root, "command");
            if (cJSON_IsString(command)) {
                ESP_LOGI(TAG, "System command: %s", command->valuestring);
                if (strcmp(command->valuestring, "reboot") == 0) {
                    // Do a reboot if user requests a OTA update
                    Schedule([this]() { Reboot(); });
                } else {
                    ESP_LOGW(TAG, "Unknown system command: %s", command->valuestring);
                }
            }
        } else if (strcmp(type->valuestring, "alert") == 0) {
            auto status = cJSON_GetObjectItem(root, "status");
            auto message = cJSON_GetObjectItem(root, "message");
            auto emotion = cJSON_GetObjectItem(root, "emotion");
            if (cJSON_IsString(status) && cJSON_IsString(message) && cJSON_IsString(emotion)) {
                Alert(status->valuestring, message->valuestring, emotion->valuestring,
                      Lang::Sounds::OGG_VIBRATION);
            } else {
                ESP_LOGW(TAG, "Alert command requires status, message and emotion");
            }
#if CONFIG_RECEIVE_CUSTOM_MESSAGE
        } else if (strcmp(type->valuestring, "custom") == 0) {
            auto payload = cJSON_GetObjectItem(root, "payload");
            ESP_LOGI(TAG, "Received custom message: %s", cJSON_PrintUnformatted(root));
            if (cJSON_IsObject(payload)) {
                Schedule(
                    [this, display, payload_str = std::string(cJSON_PrintUnformatted(payload))]() {
                        display->SetChatMessage("system", payload_str.c_str());
                    });
            } else {
                ESP_LOGW(TAG, "Invalid custom message format: missing payload");
            }
#endif
        } else {
            ESP_LOGW(TAG, "Unknown message type: %s", type->valuestring);
        }
    });

    protocol_->Start();
}

void Application::ShowActivationCode(const std::string& code, const std::string& message) {
    struct digit_sound {
        char digit;
        const std::string_view& sound;
    };
    static const std::array<digit_sound, 10> digit_sounds{
        {digit_sound{'0', Lang::Sounds::OGG_0}, digit_sound{'1', Lang::Sounds::OGG_1},
         digit_sound{'2', Lang::Sounds::OGG_2}, digit_sound{'3', Lang::Sounds::OGG_3},
         digit_sound{'4', Lang::Sounds::OGG_4}, digit_sound{'5', Lang::Sounds::OGG_5},
         digit_sound{'6', Lang::Sounds::OGG_6}, digit_sound{'7', Lang::Sounds::OGG_7},
         digit_sound{'8', Lang::Sounds::OGG_8}, digit_sound{'9', Lang::Sounds::OGG_9}}};

    // This sentence uses 9KB of SRAM, so we need to wait for it to finish
    Alert(Lang::Strings::ACTIVATION, message.c_str(), "link", Lang::Sounds::OGG_ACTIVATION);

    for (const auto& digit : code) {
        auto it = std::find_if(digit_sounds.begin(), digit_sounds.end(),
                               [digit](const digit_sound& ds) { return ds.digit == digit; });
        if (it != digit_sounds.end()) {
            audio_service_.PlaySound(it->sound);
        }
    }
}

void Application::Alert(const char* status, const char* message, const char* emotion,
                        const std::string_view& sound) {
    ESP_LOGW(TAG, "Alert [%s] %s: %s", emotion, status, message);
    auto display = Board::GetInstance().GetDisplay();
    display->SetStatus(status);
    display->SetEmotion(emotion);
    display->SetChatMessage("system", message);
    if (!sound.empty()) {
        audio_service_.PlaySound(sound);
    }
}

void Application::DismissAlert() {
    if (GetDeviceState() == kDeviceStateIdle) {
        auto display = Board::GetInstance().GetDisplay();
        display->SetStatus(Lang::Strings::STANDBY);
        display->SetEmotion("neutral");
        display->SetChatMessage("system", "");
    }
}

void Application::ToggleChatState() { xEventGroupSetBits(event_group_, MAIN_EVENT_TOGGLE_CHAT); }

void Application::TogglePhoneChatState() {
    xEventGroupSetBits(event_group_, MAIN_EVENT_TOGGLE_PHONE_CALL);
}

void Application::SetPhoneHookState(bool off_hook) {
    phone_hook_off_hook_.store(off_hook);
    xEventGroupSetBits(event_group_, MAIN_EVENT_PHONE_HOOK_CHANGED);
}

void Application::StartListening() { xEventGroupSetBits(event_group_, MAIN_EVENT_START_LISTENING); }

void Application::StopListening() { xEventGroupSetBits(event_group_, MAIN_EVENT_STOP_LISTENING); }

void Application::HandleTogglePhoneCallEvent() {
    switch (phone_call_controller_.state()) {
        case PhoneCallState::kIdle:
            BeginPhoneCall();
            break;
        case PhoneCallState::kConnecting:
        case PhoneCallState::kConnected:
            HangUpPhoneCall();
            break;
        case PhoneCallState::kHangingUp:
        case PhoneCallState::kFailed:
            ESP_LOGI(TAG, "Phone call cleanup is still in progress");
            break;
    }
}

void Application::HandlePhoneHookChangedEvent() {
    const bool off_hook = phone_hook_off_hook_.load();
    switch (phone_call_controller_.ActionForHookState(off_hook)) {
        case PhoneHookAction::kBegin:
            ESP_LOGI(TAG, "Handset lifted; starting phone call");
            BeginPhoneCall();
            break;
        case PhoneHookAction::kHangUp:
            ESP_LOGI(TAG, "Handset returned to cradle; hanging up phone call");
            HangUpPhoneCall();
            break;
        case PhoneHookAction::kNone:
            ESP_LOGI(TAG, "Handset state already applied: %s", off_hook ? "off-hook" : "on-hook");
            break;
    }
}

void Application::BeginPhoneCall() {
    if (!protocol_) {
        ESP_LOGE(TAG, "Protocol not initialized");
        audio_service_.ResetDecoder();
        audio_service_.PlaySound(Lang::Sounds::OGG_PHONE_FAILED);
        return;
    }
    if (phone_connect_task_running_.load()) {
        ESP_LOGW(TAG, "Previous phone connection task is still running");
        return;
    }

    const uint32_t generation = phone_call_controller_.Begin();
    if (generation == 0) {
        return;
    }

    phone_call_started_us_ = esp_timer_get_time();
    phone_last_ringback_us_ = phone_call_started_us_;
    phone_ringback_active_.store(true);
    phone_hangup_ready_.store(false);
    phone_hangup_sound_playing_.store(false);
    audio_service_.ResetDecoder();

    const ListeningMode mode = GetDefaultListeningMode();
    SetDeviceState(kDeviceStateConnecting);
    // Start transport setup before enqueueing the four-second local intro.
    // PlaySound can block for roughly 1.4 seconds while its Opus packets wait
    // for decode-queue capacity. Running the lower-priority connection task in
    // parallel hides that setup behind the first ring without starving audio.
    StartPhoneConnectionTask(mode, generation);
    if (!phone_call_controller_.IsConnecting(generation)) {
        return;
    }

    // Keep the initial cue and first ring in one Ogg stream. Enqueuing two Ogg
    // streams back-to-back creates an audible seam on small speakers.
    audio_service_.PlaySound(kPhoneCallingIntroSound);
}

void Application::StartPhoneConnectionTask(ListeningMode mode, uint32_t generation) {
    phone_connect_mode_ = mode;
    phone_connect_generation_ = generation;
    phone_connect_task_running_.store(true);

    BaseType_t result = xTaskCreate(
        [](void* arg) {
            auto* app = static_cast<Application*>(arg);
            app->RunPhoneConnectionTask();
            vTaskDelete(nullptr);
        },
        "phone_connect", kPhoneConnectTaskStackSize, this,
        kPhoneConnectTaskPriority, nullptr);
    if (result != pdPASS) {
        phone_connect_task_running_.store(false);
        FailPhoneCall(generation, "failed to create connection task");
    }
}

void Application::RunPhoneConnectionTask() {
    const uint32_t generation = phone_connect_generation_;
    const ListeningMode mode = phone_connect_mode_;
    bool opened = false;

    auto& board = Board::GetInstance();
    board.SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);
    if (protocol_) {
        opened = protocol_->IsAudioChannelOpened() || protocol_->OpenAudioChannel();
    }

    // OpenAudioChannel has returned, so the main task may safely close a stale
    // transport from this point onward.
    phone_connect_task_running_.store(false);
    Schedule([this, mode, generation, opened]() {
        CompletePhoneConnection(mode, generation, opened);
    });
}

void Application::CompletePhoneConnection(ListeningMode mode, uint32_t generation,
                                          bool opened) {
    if (!phone_call_controller_.IsConnecting(generation)) {
        if (protocol_) {
            protocol_->CloseAudioChannel();
        }
        phone_call_controller_.Finish();
        SetDeviceState(kDeviceStateIdle);
        return;
    }

    if (!opened) {
        FailPhoneCall(generation, "audio channel open failed");
        return;
    }

    if (!phone_call_controller_.MarkConnected(generation)) {
        protocol_->CloseAudioChannel();
        phone_call_controller_.Finish();
        SetDeviceState(kDeviceStateIdle);
        return;
    }

    ESP_LOGI(TAG, "Phone audio channel connected; waiting for first remote audio frame");
    SetListeningMode(mode);
}

void Application::HangUpPhoneCall() {
    const auto previous_state = phone_call_controller_.state();
    if (!phone_call_controller_.BeginHangUp()) {
        return;
    }

    ESP_LOGI(TAG, "Phone hang-up requested");
    phone_ringback_active_.store(false);
    phone_hangup_ready_.store(false);
    phone_hangup_sound_playing_.store(false);
    phone_hangup_requested_us_ = esp_timer_get_time();
    audio_service_.ResetDecoder();

    if (previous_state == PhoneCallState::kConnecting || !protocol_ ||
        !protocol_->IsAudioChannelOpened()) {
        CompletePhoneHangup("call was not connected");
        return;
    }

    protocol_->SendAbortSpeaking(kAbortReasonNone);
    protocol_->SendStopListening();
    SetDeviceState(kDeviceStateSpeaking);
    if (!protocol_->SendPhoneHangupRequest()) {
        ESP_LOGW(TAG, "Failed to request coordinated phone hang-up; using local fallback");
        CompletePhoneHangup("hangup request send failed");
    }
}

void Application::CompletePhoneHangup(const char* reason) {
    if (phone_call_controller_.state() != PhoneCallState::kHangingUp) {
        return;
    }

    if (phone_hangup_sound_playing_.load()) {
        return;
    }

    ESP_LOGI(TAG, "Playing local phone hang-up sound: %s",
             reason != nullptr ? reason : "unknown");
    phone_ringback_active_.store(false);
    phone_hangup_ready_.store(false);
    phone_hangup_sound_playing_.store(true);
    phone_hangup_requested_us_ = esp_timer_get_time();
    audio_service_.ResetDecoder();
    audio_service_.PlaySound(Lang::Sounds::OGG_PHONE_HANGUP);

    if (audio_service_.IsPlaybackIdle()) {
        FinalizePhoneHangup("local hang-up sound unavailable");
    }
}

void Application::FinalizePhoneHangup(const char* reason) {
    if (phone_call_controller_.state() != PhoneCallState::kHangingUp) {
        return;
    }

    ESP_LOGI(TAG, "Finalizing phone hang-up: %s", reason != nullptr ? reason : "unknown");
    phone_ringback_active_.store(false);
    phone_hangup_ready_.store(false);
    phone_hangup_sound_playing_.store(false);
    phone_hangup_requested_us_ = 0;

    // Mark idle before closing so the transport callback cannot enqueue a
    // duplicate local hang-up sound after the local sound has drained.
    phone_call_controller_.Finish();
    if (!phone_connect_task_running_.load() && protocol_) {
        protocol_->CloseAudioChannel();
    }
    SetDeviceState(kDeviceStateIdle);
}

void Application::FailPhoneCall(uint32_t generation, const char* reason) {
    if (!phone_call_controller_.Fail(generation)) {
        return;
    }

    ESP_LOGW(TAG, "Phone call failed: %s", reason != nullptr ? reason : "unknown");
    phone_ringback_active_.store(false);
    phone_hangup_ready_.store(false);
    phone_hangup_sound_playing_.store(false);
    audio_service_.ResetDecoder();
    if (!phone_connect_task_running_.load() && protocol_) {
        protocol_->CloseAudioChannel();
    }
    SetDeviceState(kDeviceStateIdle);
    audio_service_.PlaySound(Lang::Sounds::OGG_PHONE_FAILED);
    if (!phone_connect_task_running_.load()) {
        phone_call_controller_.Finish();
    }
}

void Application::HandlePhoneCallClockTick() {
    auto state = phone_call_controller_.state();
    if (state == PhoneCallState::kHangingUp) {
        if (phone_hangup_sound_playing_.load()) {
            if (audio_service_.IsPlaybackIdle()) {
                FinalizePhoneHangup("local hang-up sound drained on clock tick");
            } else if (phone_hangup_requested_us_ > 0 &&
                       esp_timer_get_time() - phone_hangup_requested_us_ >=
                           kPhoneHangupTimeoutUs) {
                ESP_LOGW(TAG, "Local phone hang-up sound timeout; closing channel");
                audio_service_.ResetDecoder();
                FinalizePhoneHangup("local hang-up sound timeout");
            }
        } else if (phone_hangup_ready_.load() && audio_service_.IsPlaybackIdle()) {
            CompletePhoneHangup("farewell drained on clock tick");
        } else if (phone_hangup_requested_us_ > 0 &&
                   esp_timer_get_time() - phone_hangup_requested_us_ >=
                       kPhoneHangupTimeoutUs) {
            ESP_LOGW(TAG, "Phone farewell timeout; playing local hang-up sound");
            CompletePhoneHangup("farewell timeout");
        }
        return;
    }
    if (!phone_ringback_active_.load()) {
        return;
    }
    if (state != PhoneCallState::kConnecting && state != PhoneCallState::kConnected) {
        return;
    }

    const int64_t now = esp_timer_get_time();
    if (now - phone_call_started_us_ >= kPhoneCallTimeoutUs) {
        FailPhoneCall(phone_call_controller_.generation(), "connection greeting timeout");
        return;
    }
    if (now - phone_last_ringback_us_ >= kPhoneRingbackCadenceUs &&
        audio_service_.IsPlaybackIdle()) {
        phone_last_ringback_us_ = now;
        audio_service_.PlaySound(Lang::Sounds::OGG_PHONE_RINGBACK);
    }
}

bool Application::StopPhoneRingbackForRemoteAudio() {
    if (!phone_ringback_active_.exchange(false)) {
        return false;
    }
    ESP_LOGI(TAG, "First remote phone audio received; stopping local ringback");
    audio_service_.ResetDecoder();
    return true;
}

void Application::HandleToggleChatEvent() {
    auto state = GetDeviceState();

    if (state == kDeviceStateActivating) {
        SetDeviceState(kDeviceStateIdle);
        return;
    } else if (state == kDeviceStateWifiConfiguring) {
        audio_service_.EnableAudioTesting(true);
        SetDeviceState(kDeviceStateAudioTesting);
        return;
    } else if (state == kDeviceStateAudioTesting) {
        audio_service_.EnableAudioTesting(false);
        SetDeviceState(kDeviceStateWifiConfiguring);
        return;
    }

    if (!protocol_) {
        ESP_LOGE(TAG, "Protocol not initialized");
        return;
    }

    if (state == kDeviceStateIdle) {
        ListeningMode mode = GetDefaultListeningMode();
        if (!protocol_->IsAudioChannelOpened()) {
            SetDeviceState(kDeviceStateConnecting);
            // Schedule to let the state change be processed first (UI update)
            Schedule([this, mode]() { ContinueOpenAudioChannel(mode); });
            return;
        }
        SetListeningMode(mode);
    } else if (state == kDeviceStateSpeaking) {
        AbortSpeaking(kAbortReasonNone);
    } else if (state == kDeviceStateListening) {
        protocol_->CloseAudioChannel();
    }
}

void Application::ContinueOpenAudioChannel(ListeningMode mode) {
    // Check state again in case it was changed during scheduling
    if (GetDeviceState() != kDeviceStateConnecting) {
        return;
    }

    // Switch to performance mode before connecting to reduce latency
    auto& board = Board::GetInstance();
    board.SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);

    if (!protocol_->IsAudioChannelOpened()) {
        if (!protocol_->OpenAudioChannel()) {
            // Return to idle so the device is not stuck in the connecting
            // state (not every failure path reports a network error)
            SetDeviceState(kDeviceStateIdle);
            return;
        }
    }

    SetListeningMode(mode);
}

void Application::HandleStartListeningEvent() {
    auto state = GetDeviceState();

    if (state == kDeviceStateActivating) {
        SetDeviceState(kDeviceStateIdle);
        return;
    } else if (state == kDeviceStateWifiConfiguring) {
        audio_service_.EnableAudioTesting(true);
        SetDeviceState(kDeviceStateAudioTesting);
        return;
    }

    if (!protocol_) {
        ESP_LOGE(TAG, "Protocol not initialized");
        return;
    }

    if (state == kDeviceStateIdle) {
        if (!protocol_->IsAudioChannelOpened()) {
            SetDeviceState(kDeviceStateConnecting);
            // Schedule to let the state change be processed first (UI update)
            Schedule([this]() { ContinueOpenAudioChannel(kListeningModeManualStop); });
            return;
        }
        SetListeningMode(kListeningModeManualStop);
    } else if (state == kDeviceStateSpeaking) {
        AbortSpeaking(kAbortReasonNone);
        SetListeningMode(kListeningModeManualStop);
    }
}

void Application::HandleStopListeningEvent() {
    auto state = GetDeviceState();

    if (state == kDeviceStateAudioTesting) {
        audio_service_.EnableAudioTesting(false);
        SetDeviceState(kDeviceStateWifiConfiguring);
        return;
    } else if (state == kDeviceStateListening) {
        if (protocol_) {
            protocol_->SendStopListening();
        }
        SetDeviceState(kDeviceStateIdle);
    }
}

void Application::HandleWakeWordDetectedEvent() {
    if (!protocol_) {
        return;
    }

    auto state = GetDeviceState();
    auto wake_word = audio_service_.GetLastWakeWord();
    ESP_LOGI(TAG, "Wake word detected: %s (state: %d)", wake_word.c_str(), (int)state);

    if (state == kDeviceStateIdle) {
        BeginWakeWordInvoke(wake_word);
    } else if (state == kDeviceStateSpeaking || state == kDeviceStateListening) {
        AbortSpeaking(kAbortReasonWakeWordDetected);
        // Clear send queue to avoid sending residues to server
        while (audio_service_.PopPacketFromSendQueue())
            ;

        if (state == kDeviceStateListening) {
            protocol_->SendStartListening(GetDefaultListeningMode());
            audio_service_.ResetDecoder();
            audio_service_.PlaySound(Lang::Sounds::OGG_POPUP);
            // Re-enable wake word detection as it was stopped by the detection itself
            audio_service_.EnableWakeWordDetection(true);
        } else {
            // Play popup sound and start listening again
            play_popup_on_listening_ = true;
            SetListeningMode(GetDefaultListeningMode());
        }
    } else if (state == kDeviceStateActivating) {
        // Restart the activation check if the wake word is detected during activation
        SetDeviceState(kDeviceStateIdle);
    }
}

void Application::BeginWakeWordInvoke(const std::string& wake_word) {
    // Must run in the main task with the device in idle state
    audio_service_.EncodeWakeWord();

    // Always pass through the connecting state, even if the audio channel is
    // already opened. ContinueWakeWordInvoke() rejects any other state, so
    // skipping this transition would silently drop the wake word invocation.
    if (!SetDeviceState(kDeviceStateConnecting)) {
        // Wake word detection was stopped by the detection itself; restore it
        // so the device does not become unresponsive to wake words.
        audio_service_.EnableWakeWordDetection(true);
        return;
    }

    if (!protocol_->IsAudioChannelOpened()) {
        // Schedule to let the state change be processed first (UI update),
        // then continue with OpenAudioChannel which may block for ~1 second
        Schedule([this, wake_word]() { ContinueWakeWordInvoke(wake_word); });
        return;
    }
    // Channel already opened, continue directly
    ContinueWakeWordInvoke(wake_word);
}

void Application::ContinueWakeWordInvoke(const std::string& wake_word) {
    // Check state again in case it was changed during scheduling
    if (GetDeviceState() != kDeviceStateConnecting) {
        return;
    }

    // Switch to performance mode before connecting to reduce latency
    auto& board = Board::GetInstance();
    board.SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);

    if (!protocol_->IsAudioChannelOpened()) {
        if (!protocol_->OpenAudioChannel()) {
            // Return to idle so the device is not stuck in the connecting
            // state (not every failure path reports a network error), and
            // wake word detection is re-enabled by the idle state handler.
            SetDeviceState(kDeviceStateIdle);
            return;
        }
    }

    ESP_LOGI(TAG, "Wake word detected: %s", wake_word.c_str());
#if CONFIG_SEND_WAKE_WORD_DATA
    // Encode and send the wake word data to the server
    while (auto packet = audio_service_.PopWakeWordPacket()) {
        protocol_->SendAudio(std::move(packet));
    }
    // Set the chat state to wake word detected
    protocol_->SendWakeWordDetected(wake_word);
    SetListeningMode(GetDefaultListeningMode());
#else
    // Set flag to play popup sound after state changes to listening
    // (PlaySound here would be cleared by ResetDecoder in EnableVoiceProcessing)
    play_popup_on_listening_ = true;
    SetListeningMode(GetDefaultListeningMode());
#endif
}

void Application::HandleStateChangedEvent() {
    DeviceState new_state = state_machine_.GetState();
    clock_ticks_ = 0;
    // Any state change invalidates a pending deferred listening start;
    // the Listening case below re-arms it when needed.
    pending_listening_start_ = false;

    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    auto led = board.GetLed();
    led->OnStateChanged();

    switch (new_state) {
        case kDeviceStateUnknown:
        case kDeviceStateIdle:
#if CONFIG_LOCAL_VAD_BARGE_IN
            local_vad_barge_in_armed_ = false;
            barge_in_waiting_for_playback_drain_ = false;
#endif
#if CONFIG_ML307_LOCAL_VAD_BARGE_IN
            audio_service_.SetUplinkAudioEnabled(true, false);
            ml307_local_barge_in_pending_ = false;
#endif
            display->SetStatus(Lang::Strings::STANDBY);
            display->ClearChatMessages();    // Clear messages first
            display->SetEmotion("neutral");  // Then set emotion (wechat mode checks child count)
            audio_service_.EnableVoiceProcessing(false);
            audio_service_.EnableWakeWordDetection(true);
            break;
        case kDeviceStateConnecting:
            display->SetStatus(Lang::Strings::CONNECTING);
            display->SetEmotion("neutral");
            display->SetChatMessage("system", "");
            break;
        case kDeviceStateListening:
#if CONFIG_LOCAL_VAD_BARGE_IN
            local_vad_barge_in_armed_ = false;
#endif
#if CONFIG_ML307_LOCAL_VAD_BARGE_IN
            audio_service_.SetUplinkAudioEnabled(true, ml307_local_barge_in_pending_);
            ESP_LOGI(TAG, "ML307 uplink resumed, flush_preroll=%d",
                     ml307_local_barge_in_pending_);
            ml307_local_barge_in_pending_ = false;
#endif
            display->SetStatus(Lang::Strings::LISTENING);
            display->SetEmotion("neutral");

            // Make sure the audio processor is running
            if (play_popup_on_listening_ || !audio_service_.IsAudioProcessorRunning()) {
                // For auto mode, wait for the playback queue to drain before enabling
                // voice processing. This prevents audio truncation when STOP arrives
                // late due to network jitter. Instead of blocking the main loop here,
                // defer the start until MAIN_EVENT_PLAYBACK_DRAINED arrives.
                if (listening_mode_ == kListeningModeAutoStop && !audio_service_.IsPlaybackIdle()) {
                    pending_listening_start_ = true;
                } else {
                    StartListeningAudio();
                }
            } else {
                ConfigureWakeWordForListening();
            }
            break;
        case kDeviceStateSpeaking:
            display->SetStatus(Lang::Strings::SPEAKING);

#if CONFIG_LOCAL_VAD_BARGE_IN
            barge_in_guard_.ResetForAssistantTurn();
            if (listening_mode_ == kListeningModeRealtime) {
#if CONFIG_DUPLEX_PLAYBACK_CONTROL
                const bool preplay_user_continuation = audio_service_.IsVoiceDetected();
                local_vad_barge_in_armed_ = !preplay_user_continuation;
                ESP_LOGI(TAG, "Local VAD barge-in armed=%d",
                         local_vad_barge_in_armed_);
                if (preplay_user_continuation && !aborted_) {
                    ESP_LOGI(TAG,
                             "Preplay user continuation detected; aborting assistant before "
                             "first audio");
#if CONFIG_VOICE_LATENCY_METRICS
                    LogVoiceMetric("PREPLAY_CONTINUATION_CANCEL", voice_metric_turn_id_);
#endif
                    AbortSpeaking(kAbortReasonNone);
                }
#else
                local_vad_barge_in_armed_ = !audio_service_.IsVoiceDetected();
                ESP_LOGI(TAG, "Local VAD barge-in armed=%d",
                         local_vad_barge_in_armed_);
#endif
            }
#endif
#if CONFIG_ML307_LOCAL_VAD_BARGE_IN
            if (listening_mode_ == kListeningModeRealtime) {
                audio_service_.SetUplinkAudioEnabled(false);
                ml307_local_barge_in_pending_ = false;
            }
#endif
#if CONFIG_VOICE_LATENCY_METRICS
            device_first_audio_logged_ = false;
            turn_completion_waiting_for_playback_drain_ = false;
            audio_service_.ArmPlaybackMetric(voice_metric_turn_id_);
#endif
            if (listening_mode_ != kListeningModeRealtime) {
                audio_service_.EnableVoiceProcessing(false);
                // Only AFE wake word can be detected in speaking mode
                audio_service_.EnableWakeWordDetection(audio_service_.IsAfeWakeWord());
            }
            // A phone ringback remains audible until the first remote audio
            // frame, not merely until the earlier tts:start control packet.
            if (!phone_ringback_active_.load()) {
                audio_service_.ResetDecoder();
            }
            break;
        case kDeviceStateWifiConfiguring:
            audio_service_.EnableVoiceProcessing(false);
            audio_service_.EnableWakeWordDetection(false);
            break;
        default:
            // Do nothing
            break;
    }
}

void Application::StartListeningAudio() {
    // Runs in the main loop, either directly from HandleStateChangedEvent or
    // deferred via MAIN_EVENT_PLAYBACK_DRAINED once the playback queue drains.
    if (GetDeviceState() != kDeviceStateListening) {
        return;
    }

    // Send the start listening command
    protocol_->SendStartListening(listening_mode_);
    const bool preserve_phone_ringback = phone_call_controller_.IsConnected() &&
                                         phone_ringback_active_.load();
    audio_service_.EnableVoiceProcessing(true, preserve_phone_ringback);

    ConfigureWakeWordForListening();

    // Play popup sound after ResetDecoder (in EnableVoiceProcessing) has been called
    if (play_popup_on_listening_) {
        play_popup_on_listening_ = false;
        audio_service_.PlaySound(Lang::Sounds::OGG_POPUP);
    }
}

void Application::ConfigureWakeWordForListening() {
#ifdef CONFIG_WAKE_WORD_DETECTION_IN_LISTENING
    // Enable wake word detection in listening mode (configured via Kconfig)
    audio_service_.EnableWakeWordDetection(audio_service_.IsAfeWakeWord());
#else
    // Disable wake word detection in listening mode
    audio_service_.EnableWakeWordDetection(false);
#endif
}

void Application::Schedule(std::function<void()>&& callback) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        main_tasks_.push_back(std::move(callback));
    }
    xEventGroupSetBits(event_group_, MAIN_EVENT_SCHEDULE);
}

void Application::AbortSpeaking(AbortReason reason) {
    ESP_LOGI(TAG, "Abort speaking");
    aborted_ = true;
    if (protocol_) {
        protocol_->SendAbortSpeaking(reason);
    }
}

void Application::SetListeningMode(ListeningMode mode) {
    listening_mode_ = mode;
    SetDeviceState(kDeviceStateListening);
}

ListeningMode Application::GetDefaultListeningMode() const {
    return aec_mode_ == kAecOff ? kListeningModeAutoStop : kListeningModeRealtime;
}

void Application::Reboot() {
    ESP_LOGI(TAG, "Rebooting...");
    // Disconnect the audio channel
    if (protocol_ && protocol_->IsAudioChannelOpened()) {
        protocol_->CloseAudioChannel();
    }
    protocol_.reset();
    audio_service_.Stop();

    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

bool Application::UpgradeFirmware(const std::string& url, const std::string& version) {
    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();

    std::string upgrade_url = url;
    std::string version_info = version.empty() ? "(Manual upgrade)" : version;

    // Close audio channel if it's open
    if (protocol_ && protocol_->IsAudioChannelOpened()) {
        ESP_LOGI(TAG, "Closing audio channel before firmware upgrade");
        protocol_->CloseAudioChannel();
    }
    ESP_LOGI(TAG, "Starting firmware upgrade from URL: %s", upgrade_url.c_str());

    Alert(Lang::Strings::OTA_UPGRADE, Lang::Strings::UPGRADING, "download",
          Lang::Sounds::OGG_UPGRADE);
    vTaskDelay(pdMS_TO_TICKS(3000));

    SetDeviceState(kDeviceStateUpgrading);

    std::string message = std::string(Lang::Strings::NEW_VERSION) + version_info;
    display->SetChatMessage("system", message.c_str());

    board.SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);
    audio_service_.Stop();
    vTaskDelay(pdMS_TO_TICKS(1000));

    bool upgrade_success = Ota::Upgrade(upgrade_url, [this, display](int progress, size_t speed) {
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "%d%% %uKB/s", progress, speed / 1024);
        Schedule([display, message = std::string(buffer)]() {
            display->SetChatMessage("system", message.c_str());
        });
    });

    if (!upgrade_success) {
        // Upgrade failed, restart audio service and continue running
        ESP_LOGE(TAG,
                 "Firmware upgrade failed, restarting audio service and continuing operation...");
        audio_service_.Start();                              // Restart audio service
        board.SetPowerSaveLevel(PowerSaveLevel::LOW_POWER);  // Restore power save level
        Alert(Lang::Strings::ERROR, Lang::Strings::UPGRADE_FAILED, "cancel",
              Lang::Sounds::OGG_EXCLAMATION);
        vTaskDelay(pdMS_TO_TICKS(3000));
        return false;
    } else {
        // Upgrade success, reboot immediately
        ESP_LOGI(TAG, "Firmware upgrade successful, rebooting...");
        display->SetChatMessage("system", "Upgrade successful, rebooting...");
        vTaskDelay(pdMS_TO_TICKS(1000));  // Brief pause to show message
        Reboot();
        return true;
    }
}

void Application::WakeWordInvoke(const std::string& wake_word) {
    if (!protocol_) {
        return;
    }

    auto state = GetDeviceState();

    if (state == kDeviceStateIdle) {
        // May be called from outside the main task (e.g. board button
        // callbacks), so schedule the invocation instead of running it here
        Schedule([this, wake_word]() {
            if (GetDeviceState() == kDeviceStateIdle) {
                BeginWakeWordInvoke(wake_word);
            }
        });
    } else if (state == kDeviceStateSpeaking) {
        Schedule([this]() { AbortSpeaking(kAbortReasonNone); });
    } else if (state == kDeviceStateListening) {
        Schedule([this]() {
            if (protocol_) {
                protocol_->CloseAudioChannel();
            }
        });
    }
}

bool Application::CanEnterSleepMode() {
    if (GetDeviceState() != kDeviceStateIdle) {
        return false;
    }

    if (protocol_ && protocol_->IsAudioChannelOpened()) {
        return false;
    }

    if (!audio_service_.IsIdle()) {
        return false;
    }

    // Now it is safe to enter sleep mode
    return true;
}

void Application::RegisterMcpBroadcastCallback(std::function<void(const std::string&)> callback) {
    mcp_broadcast_callback_ = std::move(callback);
}

void Application::SendMcpMessage(const std::string& payload) {
    // Always schedule to run in main task for thread safety
    Schedule([this, payload]() {
        if (protocol_) {
            protocol_->SendMcpMessage(payload);
        }
        if (mcp_broadcast_callback_) {
            mcp_broadcast_callback_(payload);
        }
    });
}

void Application::SetAecMode(AecMode mode) {
    aec_mode_ = mode;
    Schedule([this]() {
        auto& board = Board::GetInstance();
        auto display = board.GetDisplay();
        switch (aec_mode_) {
            case kAecOff:
                audio_service_.EnableDeviceAec(false);
                display->ShowNotification(Lang::Strings::RTC_MODE_OFF);
                break;
            case kAecOnServerSide:
                audio_service_.EnableDeviceAec(false);
                display->ShowNotification(Lang::Strings::RTC_MODE_ON);
                break;
            case kAecOnDeviceSide:
                audio_service_.EnableDeviceAec(true);
                display->ShowNotification(Lang::Strings::RTC_MODE_ON);
                break;
        }

        // If the AEC mode is changed, close the audio channel
        if (protocol_ && protocol_->IsAudioChannelOpened()) {
            protocol_->CloseAudioChannel();
        }
    });
}

void Application::PlaySound(const std::string_view& sound) { audio_service_.PlaySound(sound); }

void Application::ResetProtocol() {
    Schedule([this]() {
#if CONFIG_DUPLEX_PLAYBACK_CONTROL
        {
            std::lock_guard<std::mutex> lock(playback_protocol_mutex_);
            active_remote_output_id_.clear();
        }
        audio_service_.ResetDecoder();
#endif
        // Close audio channel if opened
        if (protocol_ && protocol_->IsAudioChannelOpened()) {
            protocol_->CloseAudioChannel();
        }
        // Reset protocol
        protocol_.reset();
    });
}
