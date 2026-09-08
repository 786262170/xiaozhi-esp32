#include "audio_service.h"
#include <esp_log.h>
#include <algorithm>
#include <cmath>
#include <cstring>

#define RATE_CVT_CFG(_src_rate, _dest_rate, _channel)                                        \
    (esp_ae_rate_cvt_cfg_t) {                                                                \
        .src_rate = (uint32_t)(_src_rate), .dest_rate = (uint32_t)(_dest_rate),              \
        .channel = (uint8_t)(_channel), .bits_per_sample = ESP_AUDIO_BIT16, .complexity = 2, \
        .perf_type = ESP_AE_RATE_CVT_PERF_TYPE_SPEED,                                        \
    }

#define OPUS_DEC_CFG(_sample_rate, _frame_duration_ms)                                     \
    (esp_opus_dec_cfg_t) {                                                                 \
        .sample_rate = (uint32_t)(_sample_rate), .channel = ESP_AUDIO_MONO,                \
        .frame_duration =                                                                  \
            (esp_opus_dec_frame_duration_t)AS_OPUS_GET_FRAME_DRU_ENUM(_frame_duration_ms), \
        .self_delimited = false,                                                           \
    }

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4 || CONFIG_IDF_TARGET_ESP32S31
#include "engines/afe_audio_engine.h"
#else
#include "engines/lite_audio_engine.h"
#endif

#define TAG "AudioService"

AudioService::AudioService() { event_group_ = xEventGroupCreate(); }

AudioService::~AudioService() {
    if (event_group_ != nullptr) {
        vEventGroupDelete(event_group_);
    }
    if (opus_encoder_ != nullptr) {
        esp_opus_enc_close(opus_encoder_);
    }
    if (opus_decoder_ != nullptr) {
        esp_opus_dec_close(opus_decoder_);
    }
    if (input_resampler_ != nullptr) {
        esp_ae_rate_cvt_close(input_resampler_);
    }
    if (output_resampler_ != nullptr) {
        esp_ae_rate_cvt_close(output_resampler_);
    }
}

void AudioService::Initialize(AudioCodec* codec) {
    codec_ = codec;
    codec_->Start();

    esp_opus_dec_cfg_t opus_dec_cfg =
        OPUS_DEC_CFG(codec->output_sample_rate(), OPUS_FRAME_DURATION_MS);
    auto ret = esp_opus_dec_open(&opus_dec_cfg, sizeof(esp_opus_dec_cfg_t), &opus_decoder_);
    if (opus_decoder_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create audio decoder, error code: %d", ret);
    } else {
        decoder_sample_rate_ = codec->output_sample_rate();
        decoder_duration_ms_ = OPUS_FRAME_DURATION_MS;
        decoder_frame_size_ = decoder_sample_rate_ / 1000 * OPUS_FRAME_DURATION_MS;
    }
    esp_opus_enc_config_t opus_enc_cfg = AS_OPUS_ENC_CONFIG();
    ret = esp_opus_enc_open(&opus_enc_cfg, sizeof(esp_opus_enc_config_t), &opus_encoder_);
    if (opus_encoder_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create audio encoder, error code: %d", ret);
    } else {
        encoder_sample_rate_ = 16000;
        encoder_duration_ms_ = OPUS_FRAME_DURATION_MS;
        esp_opus_enc_get_frame_size(opus_encoder_, &encoder_frame_size_, &encoder_outbuf_size_);
        encoder_frame_size_ = encoder_frame_size_ / sizeof(int16_t);
    }

    if (codec->input_sample_rate() != 16000) {
        esp_ae_rate_cvt_cfg_t input_resampler_cfg = RATE_CVT_CFG(
            codec->input_sample_rate(), ESP_AUDIO_SAMPLE_RATE_16K, codec->input_channels());
        auto resampler_ret = esp_ae_rate_cvt_open(&input_resampler_cfg, &input_resampler_);
        if (input_resampler_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create input resampler, error code: %d", resampler_ret);
        }
    }

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4 || CONFIG_IDF_TARGET_ESP32S31
    audio_engine_ = std::make_unique<AfeAudioEngine>();
#else
    audio_engine_ = std::make_unique<LiteAudioEngine>();
#endif
    audio_engine_->OnOutput([this](std::vector<int16_t>&& data) {
        {
            std::lock_guard<std::mutex> lock(external_capture_mutex_);
            if (external_capture_enabled_) {
                size_t overflow =
                    external_capture_queue_.size() + data.size() > MAX_EXTERNAL_CAPTURE_SAMPLES
                        ? external_capture_queue_.size() + data.size() -
                              MAX_EXTERNAL_CAPTURE_SAMPLES
                        : 0;
                while (overflow-- > 0 && !external_capture_queue_.empty()) {
                    external_capture_queue_.pop_front();
                }
                external_capture_queue_.insert(external_capture_queue_.end(), data.begin(),
                                               data.end());
                external_capture_cv_.notify_one();
                return;
            }
        }
        PushTaskToEncodeQueue(kAudioTaskTypeEncodeToSendQueue, std::move(data));
    });
    audio_engine_->OnVadStateChange([this](bool speaking) {
        voice_detected_ = speaking;
        if (callbacks_.on_vad_change) {
            callbacks_.on_vad_change(speaking);
        }
    });
    audio_engine_->OnWakeWordDetected([this](const std::string& wake_word) {
        xEventGroupClearBits(event_group_, AS_EVENT_WAKE_WORD_RUNNING);
        if (callbacks_.on_wake_word_detected) {
            callbacks_.on_wake_word_detected(wake_word);
        }
    });

    esp_timer_create_args_t audio_power_timer_args = {
        .callback =
            [](void* arg) {
                AudioService* audio_service = (AudioService*)arg;
                audio_service->CheckAndUpdateAudioPowerState();
            },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "audio_power_timer",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&audio_power_timer_args, &audio_power_timer_);
}

void AudioService::Start() {
    service_stopped_.store(false);
    xEventGroupClearBits(event_group_, AS_EVENT_AUDIO_TESTING_RUNNING | AS_EVENT_WAKE_WORD_RUNNING |
                                           AS_EVENT_AUDIO_PROCESSOR_RUNNING |
                                           AS_EVENT_AUDIO_INPUT_STOP_REQUEST);

    esp_timer_start_periodic(audio_power_timer_, 1000000);

#if CONFIG_USE_AUDIO_PROCESSOR
    /* Start the audio input task */
    xTaskCreatePinnedToCore(
        [](void* arg) {
            AudioService* audio_service = (AudioService*)arg;
            audio_service->AudioInputTask();
            vTaskDelete(NULL);
        },
        "audio_input", 2048 * 3, this, 8, &audio_input_task_handle_, 0);

    /* Start the audio output task */
    xTaskCreate(
        [](void* arg) {
            AudioService* audio_service = (AudioService*)arg;
            audio_service->AudioOutputTask();
            vTaskDelete(NULL);
        },
        "audio_output", 2048 * 2, this, 4, &audio_output_task_handle_);
#else
    /* Start the audio input task */
    xTaskCreate(
        [](void* arg) {
            AudioService* audio_service = (AudioService*)arg;
            audio_service->AudioInputTask();
            vTaskDelete(NULL);
        },
        "audio_input", 2048 * 2, this, 8, &audio_input_task_handle_);

    /* Start the audio output task */
    xTaskCreate(
        [](void* arg) {
            AudioService* audio_service = (AudioService*)arg;
            audio_service->AudioOutputTask();
            vTaskDelete(NULL);
        },
        "audio_output", 2048, this, 4, &audio_output_task_handle_);
#endif

    /* Start the opus codec task */
    xTaskCreate(
        [](void* arg) {
            AudioService* audio_service = (AudioService*)arg;
            audio_service->OpusCodecTask();
            vTaskDelete(NULL);
        },
        "opus_codec", 2048 * 12, this, 2, &opus_codec_task_handle_);
}

void AudioService::Stop() {
    esp_timer_stop(audio_power_timer_);
    service_stopped_.store(true);
    xEventGroupSetBits(event_group_, AS_EVENT_AUDIO_TESTING_RUNNING | AS_EVENT_WAKE_WORD_RUNNING |
                                         AS_EVENT_AUDIO_PROCESSOR_RUNNING);

    bool notify_drained = false;
    {
        std::lock_guard<std::mutex> lock(audio_queue_mutex_);
        ++playback_generation_;
        audio_encode_queue_.clear();
        audio_decode_queue_.clear();
        audio_playback_queue_.clear();
        audio_testing_queue_.clear();
        remote_output_id_.clear();
        remote_played_ms_ = 0;
        remote_last_reported_ms_ = 0;
        remote_stop_pending_ = false;
        remote_stop_reason_.clear();
        playback_gain_linear_ = 1.0f;
        remote_playback_paused_ = false;
        listener_feedback_playing_ = false;
        listener_feedback_id_.clear();
        notify_drained = MarkPlaybackDrainedLocked();
        audio_queue_cv_.notify_all();
    }
    if (notify_drained && callbacks_.on_playback_drained) {
        callbacks_.on_playback_drained();
    }
}

bool AudioService::ReadAudioData(std::vector<int16_t>& data, int sample_rate, int samples) {
    if (!codec_->input_enabled()) {
        esp_timer_stop(audio_power_timer_);
        esp_timer_start_periodic(audio_power_timer_, AUDIO_POWER_CHECK_INTERVAL_MS * 1000);
        codec_->EnableInput(true);
    }

    if (codec_->input_sample_rate() != sample_rate) {
        data.resize(samples * codec_->input_sample_rate() / sample_rate * codec_->input_channels());
        if (!codec_->InputData(data)) {
            return false;
        }
        if (input_resampler_ != nullptr) {
            std::lock_guard<std::mutex> lock(input_resampler_mutex_);
            uint32_t in_sample_num = data.size() / codec_->input_channels();
            uint32_t output_samples = 0;
            esp_ae_rate_cvt_get_max_out_sample_num(input_resampler_, in_sample_num,
                                                   &output_samples);
            auto resampled = std::vector<int16_t>(output_samples * codec_->input_channels());
            uint32_t actual_output = output_samples;
            esp_ae_rate_cvt_process(input_resampler_, (esp_ae_sample_t)data.data(), in_sample_num,
                                    (esp_ae_sample_t)resampled.data(), &actual_output);
            resampled.resize(actual_output * codec_->input_channels());
            data = std::move(resampled);
        }
    } else {
        data.resize(samples * codec_->input_channels());
        if (!codec_->InputData(data)) {
            return false;
        }
    }

    /* Update the last input time */
    last_input_time_ = std::chrono::steady_clock::now();
    debug_statistics_.input_count++;

#if CONFIG_ESP32S3_KORVO2_V3_ADC_BUTTON_DIAGNOSTICS
    // The AEC diagnostic build uses this to distinguish a silent/incorrect
    // ES7210 input path from failures later in AFE/VAD or the cloud pipeline.
    // The production Wi-Fi and cellular builds leave this code compiled out.
    constexpr int kDiagnosticChannelLimit = 4;
    static uint64_t square_sum[kDiagnosticChannelLimit] = {};
    static uint32_t sample_count[kDiagnosticChannelLimit] = {};
    static uint32_t nonzero_count[kDiagnosticChannelLimit] = {};
    static int32_t peak[kDiagnosticChannelLimit] = {};
    static int64_t window_started_us = 0;

    const int channel_count = codec_->input_channels();
    const int measured_channels =
        channel_count < kDiagnosticChannelLimit ? channel_count : kDiagnosticChannelLimit;
    for (size_t frame = 0; frame < data.size() / channel_count; ++frame) {
        for (int channel = 0; channel < measured_channels; ++channel) {
            const int32_t value = data[frame * channel_count + channel];
            const int32_t magnitude = value < 0 ? -value : value;
            if (magnitude > peak[channel]) {
                peak[channel] = magnitude;
            }
            square_sum[channel] += static_cast<uint64_t>(value * value);
            sample_count[channel]++;
            if (value != 0) {
                nonzero_count[channel]++;
            }
        }
    }

    const int64_t now_us = esp_timer_get_time();
    if (window_started_us == 0) {
        window_started_us = now_us;
    } else if (now_us - window_started_us >= 1000000) {
        int rms[kDiagnosticChannelLimit] = {};
        for (int channel = 0; channel < measured_channels; ++channel) {
            if (sample_count[channel] > 0) {
                rms[channel] = static_cast<int>(
                    std::sqrt(static_cast<double>(square_sum[channel]) / sample_count[channel]));
            }
        }
        ESP_LOGI(TAG,
                 "MIC_INPUT_METRIC channels=%d "
                 "ch0_peak=%ld ch0_rms=%d ch0_nonzero=%lu "
                 "ch1_peak=%ld ch1_rms=%d ch1_nonzero=%lu "
                 "ch2_peak=%ld ch2_rms=%d ch2_nonzero=%lu "
                 "ch3_peak=%ld ch3_rms=%d ch3_nonzero=%lu",
                 channel_count, static_cast<long>(peak[0]), rms[0],
                 static_cast<unsigned long>(nonzero_count[0]),
                 static_cast<long>(peak[1]), rms[1],
                 static_cast<unsigned long>(nonzero_count[1]),
                 static_cast<long>(peak[2]), rms[2],
                 static_cast<unsigned long>(nonzero_count[2]),
                 static_cast<long>(peak[3]), rms[3],
                 static_cast<unsigned long>(nonzero_count[3]));
        for (int channel = 0; channel < kDiagnosticChannelLimit; ++channel) {
            square_sum[channel] = 0;
            sample_count[channel] = 0;
            nonzero_count[channel] = 0;
            peak[channel] = 0;
        }
        window_started_us = now_us;
    }
#endif

#if CONFIG_USE_AUDIO_DEBUGGER
    // 音频调试：发送原始音频数据
    if (audio_debugger_ == nullptr) {
        audio_debugger_ = std::make_unique<AudioDebugger>();
    }
    audio_debugger_->Feed(data);
#endif

    return true;
}

void AudioService::AudioInputTask() {
    constexpr EventBits_t kAudioInputActiveBits = AS_EVENT_AUDIO_TESTING_RUNNING |
                                                  AS_EVENT_WAKE_WORD_RUNNING |
                                                  AS_EVENT_AUDIO_PROCESSOR_RUNNING;

    while (true) {
        EventBits_t bits = xEventGroupWaitBits(
            event_group_, kAudioInputActiveBits | AS_EVENT_AUDIO_INPUT_STOP_REQUEST, pdFALSE,
            pdFALSE, portMAX_DELAY);

        if (service_stopped_.load()) {
            // ADC continuous mode keeps its hardware mutex from start until stop,
            // so the input task that started it must also stop it before exiting.
            if (codec_->input_enabled()) {
                codec_->EnableInput(false);
            }
            break;
        }

        if (bits & AS_EVENT_AUDIO_INPUT_STOP_REQUEST) {
            xEventGroupClearBits(event_group_, AS_EVENT_AUDIO_INPUT_STOP_REQUEST);

            // Recheck the active state in this task. Audio capture may have been
            // enabled after the timer posted the stop request.
            bits = xEventGroupGetBits(event_group_);
            if ((bits & kAudioInputActiveBits) == 0) {
                if (codec_->input_enabled()) {
                    codec_->EnableInput(false);
                }
                // Do not process the stale active bits returned by waitBits().
                continue;
            }
        }

        if (audio_input_need_warmup_.exchange(false)) {
            vTaskDelay(pdMS_TO_TICKS(120));
            continue;
        }

        /* Used for audio testing in NetworkConfiguring mode by clicking the BOOT button */
        if (bits & AS_EVENT_AUDIO_TESTING_RUNNING) {
            if (audio_testing_queue_.size() >=
                AUDIO_TESTING_MAX_DURATION_MS / OPUS_FRAME_DURATION_MS) {
                ESP_LOGW(TAG, "Audio testing queue is full, stopping audio testing");
                EnableAudioTesting(false);
                continue;
            }
            std::vector<int16_t> data;
            int samples = OPUS_FRAME_DURATION_MS * 16000 / 1000;
            if (ReadAudioData(data, 16000, samples)) {
                // If input channels is 2, fetch the microphone in the first channel.
                if (codec_->input_channels() == 2) {
                    auto mono_data = std::vector<int16_t>(data.size() / 2);
                    for (size_t i = 0, j = 0; i < mono_data.size(); ++i, j += 2) {
                        mono_data[i] = data[j];
                    }
                    data = std::move(mono_data);
                }
                PushTaskToEncodeQueue(kAudioTaskTypeEncodeToTestingQueue, std::move(data));
                continue;
            }
        }

        /* Feed the selected audio engine */
        if (bits & (AS_EVENT_WAKE_WORD_RUNNING | AS_EVENT_AUDIO_PROCESSOR_RUNNING)) {
            int samples = 160;  // 10ms
            std::vector<int16_t> data;
            if (ReadAudioData(data, 16000, samples)) {
                audio_engine_->Feed(std::move(data));
                continue;
            }
        }

        // Read timeout/error should not terminate the input task.
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGW(TAG, "Audio input task stopped");
}

void AudioService::AudioOutputTask() {
    while (true) {
        std::unique_lock<std::mutex> lock(audio_queue_mutex_);
        audio_queue_cv_.wait(lock, [this]() {
            if (service_stopped_.load()) {
                return true;
            }
            if (audio_playback_queue_.empty()) {
                return false;
            }
            const auto& next = audio_playback_queue_.front();
            return !remote_playback_paused_ || next->output_id != remote_output_id_;
        });
        if (service_stopped_.load()) {
            break;
        }

        auto task = std::move(audio_playback_queue_.front());
        audio_playback_queue_.pop_front();
        output_in_flight_ = true;
        float gain_linear = task->gain_linear;
        if (!task->output_id.empty() && task->output_id == remote_output_id_) {
            gain_linear *= playback_gain_linear_;
        }
        audio_queue_cv_.notify_all();
        lock.unlock();

        if (!codec_->output_enabled()) {
            esp_timer_stop(audio_power_timer_);
            esp_timer_start_periodic(audio_power_timer_, AUDIO_POWER_CHECK_INTERVAL_MS * 1000);
            codec_->EnableOutput(true);
        }

#if CONFIG_VOICE_LATENCY_METRICS
        if (playback_metric_armed_.exchange(false)) {
            ESP_LOGI(TAG, "VOICE_METRIC event=DEVICE_PCM_PLAYBACK_START turn_id=%lu ts_mono_ms=%lld",
                     static_cast<unsigned long>(playback_metric_turn_id_.load()),
                     static_cast<long long>(esp_timer_get_time() / 1000));
        }
#endif
        ApplyPcmGain(task->pcm, gain_linear);
        codec_->OutputData(task->pcm);

        /* Update the last output time */
        last_output_time_ = std::chrono::steady_clock::now();
        debug_statistics_.playback_count++;

        bool notify_drained = false;
        bool report_progress = false;
        std::string progress_output_id;
        int64_t progress_played_ms = 0;
        int64_t progress_buffered_ms = 0;
        int64_t progress_device_ts = 0;
        lock.lock();
#if CONFIG_USE_SERVER_AEC
        /* Record the timestamp for server AEC */
        if (task->timestamp > 0) {
            timestamp_queue_.push_back(task->timestamp);
        }
#endif
        if (!task->listener_feedback && !task->output_id.empty() &&
            task->output_id == remote_output_id_) {
            remote_played_ms_ += task->frame_duration_ms;
            if (remote_played_ms_ - remote_last_reported_ms_ >= 180) {
                remote_last_reported_ms_ = remote_played_ms_;
                report_progress = true;
                progress_output_id = remote_output_id_;
                progress_played_ms = remote_played_ms_;
                progress_buffered_ms = BufferedRemotePlaybackMsLocked(remote_output_id_);
                progress_device_ts = esp_timer_get_time() / 1000;
            }
        }
        output_in_flight_ = false;
        notify_drained = MarkPlaybackDrainedLocked();
        if (task->listener_feedback) {
            listener_feedback_playing_ = HasListenerFeedbackLocked();
            if (!listener_feedback_playing_) {
                listener_feedback_id_.clear();
            }
        }
        audio_queue_cv_.notify_all();
        lock.unlock();

        if (report_progress && callbacks_.on_playback_progress) {
            callbacks_.on_playback_progress(progress_output_id, progress_played_ms,
                                            progress_buffered_ms, progress_device_ts);
        }
        if (notify_drained && callbacks_.on_playback_drained) {
            callbacks_.on_playback_drained();
        }
    }

    ESP_LOGW(TAG, "Audio output task stopped");
}

void AudioService::OpusCodecTask() {
    while (true) {
        std::unique_lock<std::mutex> lock(audio_queue_mutex_);
        audio_queue_cv_.wait(lock, [this]() {
            return service_stopped_.load() || !audio_encode_queue_.empty() ||
                   (!audio_decode_queue_.empty() &&
                    audio_playback_queue_.size() < MAX_PLAYBACK_TASKS_IN_QUEUE);
        });
        if (service_stopped_.load()) {
            break;
        }

        /* Decode the audio from decode queue */
        if (!audio_decode_queue_.empty() &&
            audio_playback_queue_.size() < MAX_PLAYBACK_TASKS_IN_QUEUE) {
            auto packet = std::move(audio_decode_queue_.front());
            audio_decode_queue_.pop_front();
            decode_in_flight_ = true;
            decode_in_flight_listener_feedback_ = packet->listener_feedback;
            const uint32_t generation = playback_generation_;
            audio_queue_cv_.notify_all();
            lock.unlock();

            auto task = std::make_unique<AudioTask>();
            task->type = kAudioTaskTypeDecodeToPlaybackQueue;
            task->timestamp = packet->timestamp;
            task->frame_duration_ms = packet->frame_duration;
            task->output_id = packet->output_id;
            task->listener_feedback = packet->listener_feedback;
            task->gain_linear = packet->gain_linear;

            SetDecodeSampleRate(packet->sample_rate, packet->frame_duration);
            bool decoded = false;
            if (opus_decoder_ != nullptr) {
                task->pcm.resize(decoder_frame_size_);
                esp_audio_dec_in_raw_t raw = {
                    .buffer = (uint8_t*)(packet->payload.data()),
                    .len = (uint32_t)(packet->payload.size()),
                    .consumed = 0,
                    .frame_recover = ESP_AUDIO_DEC_RECOVERY_NONE,
                };
                esp_audio_dec_out_frame_t out_frame = {
                    .buffer = (uint8_t*)(task->pcm.data()),
                    .len = (uint32_t)(task->pcm.size() * sizeof(int16_t)),
                    .decoded_size = 0,
                };
                esp_audio_dec_info_t dec_info = {};
                std::unique_lock<std::mutex> decoder_lock(decoder_mutex_);
                auto ret = esp_opus_dec_decode(opus_decoder_, &raw, &out_frame, &dec_info);
                decoder_lock.unlock();
                if (ret == ESP_AUDIO_ERR_OK) {
                    task->pcm.resize(out_frame.decoded_size / sizeof(int16_t));
                    if (decoder_sample_rate_ != codec_->output_sample_rate() &&
                        output_resampler_ != nullptr) {
                        uint32_t target_size = 0;
                        esp_ae_rate_cvt_get_max_out_sample_num(output_resampler_, task->pcm.size(),
                                                               &target_size);
                        std::vector<int16_t> resampled(target_size);
                        uint32_t actual_output = target_size;
                        esp_ae_rate_cvt_process(output_resampler_,
                                                (esp_ae_sample_t)task->pcm.data(), task->pcm.size(),
                                                (esp_ae_sample_t)resampled.data(), &actual_output);
                        resampled.resize(actual_output);
                        task->pcm = std::move(resampled);
                    }
                    decoded = true;
                } else {
                    ESP_LOGE(TAG, "Failed to decode audio after resize, error code: %d", ret);
                }
            } else {
                ESP_LOGE(TAG, "Audio decoder is not configured");
            }

            lock.lock();
            if (decoded && generation == playback_generation_ && !service_stopped_.load()) {
                audio_playback_queue_.push_back(std::move(task));
            }
            decode_in_flight_ = false;
            decode_in_flight_listener_feedback_ = false;
            debug_statistics_.decode_count++;
            const bool notify_drained = MarkPlaybackDrainedLocked();
            audio_queue_cv_.notify_all();
            lock.unlock();
            if (notify_drained && callbacks_.on_playback_drained) {
                callbacks_.on_playback_drained();
            }
            lock.lock();
        }
        /* Encode the audio to send queue */
        if (!audio_encode_queue_.empty()) {
            auto task = std::move(audio_encode_queue_.front());
            audio_encode_queue_.pop_front();
            audio_queue_cv_.notify_all();
            lock.unlock();

            auto packet = std::make_unique<AudioStreamPacket>();
            packet->frame_duration = OPUS_FRAME_DURATION_MS;
            packet->sample_rate = 16000;
            packet->timestamp = task->timestamp;

            if (opus_encoder_ != nullptr && task->pcm.size() == encoder_frame_size_) {
                std::vector<uint8_t> buf(encoder_outbuf_size_);
                esp_audio_enc_in_frame_t in = {
                    .buffer = (uint8_t*)(task->pcm.data()),
                    .len = (uint32_t)(encoder_frame_size_ * sizeof(int16_t)),
                };
                esp_audio_enc_out_frame_t out = {
                    .buffer = buf.data(),
                    .len = (uint32_t)encoder_outbuf_size_,
                    .encoded_bytes = 0,
                };
                auto ret = esp_opus_enc_process(opus_encoder_, &in, &out);
                if (ret == ESP_AUDIO_ERR_OK) {
                    packet->payload.assign(buf.data(), buf.data() + out.encoded_bytes);

                    if (task->type == kAudioTaskTypeEncodeToSendQueue) {
                        bool notify_send_queue = false;
                        {
                            std::lock_guard<std::mutex> lock2(audio_queue_mutex_);
                            if (uplink_audio_enabled_.load()) {
                                /* Never let a full send queue stall encoding: stale realtime
                                 * audio is useless to the server, so drop the oldest packet. */
                                if (audio_send_queue_.size() >= MAX_SEND_PACKETS_IN_QUEUE) {
                                    audio_send_queue_.pop_front();
                                }
                                audio_send_queue_.push_back(std::move(packet));
                                notify_send_queue = true;
                            } else {
                                /* Keep only the speech onset needed for local barge-in.
                                 * AEC/VAD and Opus stay warm while the modem uplink is gated. */
                                if (gated_uplink_preroll_.size() >=
                                    GATED_UPLINK_PREROLL_PACKETS) {
                                    gated_uplink_preroll_.pop_front();
                                }
                                gated_uplink_preroll_.push_back(std::move(packet));
                            }
                        }
                        if (notify_send_queue && callbacks_.on_send_queue_available) {
                            callbacks_.on_send_queue_available();
                        }
                    } else if (task->type == kAudioTaskTypeEncodeToTestingQueue) {
                        std::lock_guard<std::mutex> lock2(audio_queue_mutex_);
                        audio_testing_queue_.push_back(std::move(packet));
                    }
                    debug_statistics_.encode_count++;
                } else {
                    ESP_LOGE(TAG, "Failed to encode audio, error code: %d", ret);
                }
            } else {
                ESP_LOGE(TAG,
                         "Failed to encode audio: encoder not configured or invalid frame size "
                         "(got %u, expected %u)",
                         task->pcm.size(), encoder_frame_size_);
            }
            lock.lock();
        }
    }

    ESP_LOGW(TAG, "Opus codec task stopped");
}

void AudioService::SetDecodeSampleRate(int sample_rate, int frame_duration) {
    if (decoder_sample_rate_ == sample_rate && decoder_duration_ms_ == frame_duration) {
        return;
    }
    std::unique_lock<std::mutex> decoder_lock(decoder_mutex_);
    if (opus_decoder_ != nullptr) {
        esp_opus_dec_close(opus_decoder_);
        opus_decoder_ = nullptr;
    }
    decoder_lock.unlock();
    esp_opus_dec_cfg_t opus_dec_cfg = OPUS_DEC_CFG(sample_rate, frame_duration);
    auto ret = esp_opus_dec_open(&opus_dec_cfg, sizeof(esp_opus_dec_cfg_t), &opus_decoder_);
    if (opus_decoder_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create audio decoder, error code: %d", ret);
        return;
    }
    decoder_sample_rate_ = sample_rate;
    decoder_duration_ms_ = frame_duration;
    decoder_frame_size_ = decoder_sample_rate_ / 1000 * frame_duration;

    auto codec = Board::GetInstance().GetAudioCodec();
    if (decoder_sample_rate_ != codec->output_sample_rate()) {
        ESP_LOGI(TAG, "Resampling audio from %d to %d", decoder_sample_rate_,
                 codec->output_sample_rate());
        if (output_resampler_ != nullptr) {
            esp_ae_rate_cvt_close(output_resampler_);
            output_resampler_ = nullptr;
        }
        esp_ae_rate_cvt_cfg_t output_resampler_cfg =
            RATE_CVT_CFG(decoder_sample_rate_, codec->output_sample_rate(), ESP_AUDIO_MONO);
        auto resampler_ret = esp_ae_rate_cvt_open(&output_resampler_cfg, &output_resampler_);
        if (output_resampler_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create output resampler, error code: %d", resampler_ret);
        }
    }
}

void AudioService::PushTaskToEncodeQueue(AudioTaskType type, std::vector<int16_t>&& pcm) {
    auto task = std::make_unique<AudioTask>();
    task->type = type;
    task->pcm = std::move(pcm);

    uint32_t dropped_total = 0;
    {
        /* Push the task to the encode queue */
        std::unique_lock<std::mutex> lock(audio_queue_mutex_);

        /* If the task is to send queue, we need to set the timestamp */
        if (type == kAudioTaskTypeEncodeToSendQueue && !timestamp_queue_.empty()) {
            if (timestamp_queue_.size() <= MAX_TIMESTAMPS_IN_QUEUE) {
                task->timestamp = timestamp_queue_.front();
            } else {
                ESP_LOGW(TAG, "Timestamp queue (%u) is full, dropping timestamp",
                         timestamp_queue_.size());
            }
            timestamp_queue_.pop_front();
        }

        /* Microphone audio is realtime, so drop the oldest frame instead of blocking.
         * Blocking here would stall the audio engine task (AFE fetch) and deadlock the
         * whole input pipeline when the send queue stops being drained (e.g. network
         * congestion or a failed UDP send). */
        if (audio_encode_queue_.size() >= MAX_ENCODE_TASKS_IN_QUEUE) {
            audio_encode_queue_.pop_front();
            dropped_total = ++debug_statistics_.encode_drop_count;
        }
        audio_encode_queue_.push_back(std::move(task));
        audio_queue_cv_.notify_all();
    }

    /* Log outside the lock (UART writes are slow and would starve the codec task),
     * at most once per second. */
    if (dropped_total > 0) {
        int64_t now = esp_timer_get_time();
        if (now - last_encode_drop_log_time_ >= 1000000) {
            last_encode_drop_log_time_ = now;
            ESP_LOGW(TAG, "Encode queue is full, dropping oldest frame (dropped %lu so far)",
                     (unsigned long)dropped_total);
        }
    }
}

bool AudioService::PushPacketToDecodeQueue(std::unique_ptr<AudioStreamPacket> packet, bool wait) {
    std::unique_lock<std::mutex> lock(audio_queue_mutex_);
    if (audio_decode_queue_.size() >= MAX_DECODE_PACKETS_IN_QUEUE) {
        if (wait) {
            audio_queue_cv_.wait(lock, [this]() {
                return service_stopped_.load() ||
                       audio_decode_queue_.size() < MAX_DECODE_PACKETS_IN_QUEUE;
            });
        } else {
            return false;
        }
    }
    if (service_stopped_.load()) {
        return false;
    }
    playback_drained_notified_ = false;
    audio_decode_queue_.push_back(std::move(packet));
    audio_queue_cv_.notify_all();
    return true;
}

bool AudioService::PushPcmToPlaybackQueue(std::vector<int16_t>&& pcm) {
    if (pcm.empty()) {
        return true;
    }

    auto task = std::make_unique<AudioTask>();
    task->type = kAudioTaskTypeDecodeToPlaybackQueue;
    task->pcm = std::move(pcm);

    std::lock_guard<std::mutex> lock(audio_queue_mutex_);
    if (service_stopped_.load()) {
        return false;
    }
    if (audio_playback_queue_.size() >= MAX_PLAYBACK_TASKS_IN_QUEUE) {
        audio_playback_queue_.pop_front();
    }
    playback_drained_notified_ = false;
    audio_playback_queue_.push_back(std::move(task));
    audio_queue_cv_.notify_all();
    return true;
}

void AudioService::EnableExternalPcmCapture(bool enable) {
    std::lock_guard<std::mutex> lock(external_capture_mutex_);
    external_capture_enabled_ = enable;
    external_capture_queue_.clear();
    external_capture_cv_.notify_all();
}

bool AudioService::ReadExternalPcm(int16_t* samples, size_t sample_count, int timeout_ms) {
    if (samples == nullptr || sample_count == 0) {
        return false;
    }

    std::unique_lock<std::mutex> lock(external_capture_mutex_);
    external_capture_cv_.wait_for(
        lock, std::chrono::milliseconds(timeout_ms), [this, sample_count]() {
            return !external_capture_enabled_ || external_capture_queue_.size() >= sample_count;
        });
    if (!external_capture_enabled_) {
        return false;
    }

    size_t copied = 0;
    while (copied < sample_count && !external_capture_queue_.empty()) {
        samples[copied++] = external_capture_queue_.front();
        external_capture_queue_.pop_front();
    }
    // Keep the RTP clock moving during local VAD pauses or brief AFE underruns.
    std::fill(samples + copied, samples + sample_count, 0);
    return true;
}

bool AudioService::PushRemotePacketToDecodeQueue(std::unique_ptr<AudioStreamPacket> packet,
                                                 const std::string& output_id) {
    if (packet == nullptr || output_id.empty()) {
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(audio_queue_mutex_);
        if (remote_output_id_ != output_id) {
            remote_output_id_ = output_id;
            remote_played_ms_ = 0;
            remote_last_reported_ms_ = 0;
            remote_stop_pending_ = false;
            remote_stop_reason_.clear();
            playback_gain_linear_ = 1.0f;
            remote_playback_paused_ = false;
        }
        packet->output_id = output_id;
    }
    return PushPacketToDecodeQueue(std::move(packet));
}

void AudioService::BeginRemotePlayback(const std::string& output_id) {
    if (output_id.empty()) {
        return;
    }
    std::lock_guard<std::mutex> lock(audio_queue_mutex_);
    if (remote_output_id_ == output_id) {
        return;
    }
    remote_output_id_ = output_id;
    remote_played_ms_ = 0;
    remote_last_reported_ms_ = 0;
    remote_stop_pending_ = false;
    remote_stop_reason_.clear();
    playback_gain_linear_ = 1.0f;
    remote_playback_paused_ = false;
    audio_queue_cv_.notify_all();
}

void AudioService::EndRemotePlayback(const std::string& output_id, const std::string& reason) {
    if (output_id.empty()) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(audio_queue_mutex_);
        if (remote_output_id_ != output_id) {
            return;
        }
        remote_stop_pending_ = true;
        remote_stop_reason_ = reason.empty() ? "completed" : reason;
    }
    CompleteRemotePlaybackIfDrained();
}

bool AudioService::ControlRemotePlayback(const std::string& output_id,
                                         RemotePlaybackAction action,
                                         float target_gain_db) {
    if (output_id.empty()) {
        return false;
    }

    bool notify_drained = false;
    bool report_stopped = false;
    int64_t stopped_played_ms = 0;
    std::string stopped_reason;
    {
        std::lock_guard<std::mutex> lock(audio_queue_mutex_);
        if (remote_output_id_ != output_id) {
            return false;
        }
        switch (action) {
            case RemotePlaybackAction::kDuck:
                playback_gain_linear_ = GainDbToLinear(target_gain_db);
                break;
            case RemotePlaybackAction::kPause:
                remote_playback_paused_ = true;
                break;
            case RemotePlaybackAction::kResume:
                playback_gain_linear_ = 1.0f;
                remote_playback_paused_ = false;
                break;
            case RemotePlaybackAction::kCancel: {
                ++playback_generation_;
                const auto matches_output = [&output_id](const auto& item) {
                    return item != nullptr && item->output_id == output_id;
                };
                audio_decode_queue_.erase(
                    std::remove_if(audio_decode_queue_.begin(), audio_decode_queue_.end(),
                                   matches_output),
                    audio_decode_queue_.end());
                audio_playback_queue_.erase(
                    std::remove_if(audio_playback_queue_.begin(), audio_playback_queue_.end(),
                                   matches_output),
                    audio_playback_queue_.end());
                std::unique_lock<std::mutex> decoder_lock(decoder_mutex_);
                if (opus_decoder_ != nullptr) {
                    esp_opus_dec_reset(opus_decoder_);
                }
                decoder_lock.unlock();
                report_stopped = true;
                stopped_played_ms = remote_played_ms_;
                stopped_reason = "cancelled";
                remote_output_id_.clear();
                remote_stop_pending_ = false;
                remote_stop_reason_.clear();
                playback_gain_linear_ = 1.0f;
                remote_playback_paused_ = false;
                notify_drained = MarkPlaybackDrainedLocked();
                break;
            }
        }
        audio_queue_cv_.notify_all();
    }

    if (report_stopped && callbacks_.on_playback_stopped) {
        callbacks_.on_playback_stopped(output_id, stopped_played_ms, stopped_reason,
                                       esp_timer_get_time() / 1000);
    }
    if (notify_drained && callbacks_.on_playback_drained) {
        callbacks_.on_playback_drained();
    }
    return true;
}

void AudioService::CompleteRemotePlaybackIfDrained() {
    std::string output_id;
    std::string reason;
    int64_t played_ms = 0;
    {
        std::lock_guard<std::mutex> lock(audio_queue_mutex_);
        if (!remote_stop_pending_ || remote_output_id_.empty() || !IsPlaybackDrainedLocked()) {
            return;
        }
        output_id = remote_output_id_;
        played_ms = remote_played_ms_;
        reason = remote_stop_reason_.empty() ? "completed" : remote_stop_reason_;
        remote_output_id_.clear();
        remote_stop_pending_ = false;
        remote_stop_reason_.clear();
        playback_gain_linear_ = 1.0f;
        remote_playback_paused_ = false;
    }
    if (callbacks_.on_playback_stopped) {
        callbacks_.on_playback_stopped(output_id, played_ms, reason,
                                       esp_timer_get_time() / 1000);
    }
}

std::unique_ptr<AudioStreamPacket> AudioService::PopPacketFromSendQueue() {
    std::lock_guard<std::mutex> lock(audio_queue_mutex_);
    if (audio_send_queue_.empty()) {
        return nullptr;
    }
    auto packet = std::move(audio_send_queue_.front());
    audio_send_queue_.pop_front();
    audio_queue_cv_.notify_all();
    return packet;
}

void AudioService::SetUplinkAudioEnabled(bool enabled, bool flush_preroll) {
    bool notify_send_queue = false;
    {
        std::lock_guard<std::mutex> lock(audio_queue_mutex_);
        if (uplink_audio_enabled_.load() == enabled) {
            return;
        }

        uplink_audio_enabled_.store(enabled);
        if (!enabled) {
            audio_send_queue_.clear();
            gated_uplink_preroll_.clear();
        } else if (flush_preroll) {
            while (!gated_uplink_preroll_.empty()) {
                if (audio_send_queue_.size() >= MAX_SEND_PACKETS_IN_QUEUE) {
                    audio_send_queue_.pop_front();
                }
                audio_send_queue_.push_back(std::move(gated_uplink_preroll_.front()));
                gated_uplink_preroll_.pop_front();
            }
            notify_send_queue = !audio_send_queue_.empty();
        } else {
            gated_uplink_preroll_.clear();
        }
        audio_queue_cv_.notify_all();
    }

    if (notify_send_queue && callbacks_.on_send_queue_available) {
        callbacks_.on_send_queue_available();
    }
}

void AudioService::EncodeWakeWord() {
    if (audio_engine_) {
        audio_engine_->EncodeWakeWordData();
    }
}

const std::string& AudioService::GetLastWakeWord() const {
    static const std::string empty;
    return audio_engine_ ? audio_engine_->GetLastDetectedWakeWord() : empty;
}

std::unique_ptr<AudioStreamPacket> AudioService::PopWakeWordPacket() {
    auto packet = std::make_unique<AudioStreamPacket>();
    if (audio_engine_ && audio_engine_->GetWakeWordOpus(packet->payload)) {
        return packet;
    }
    return nullptr;
}

void AudioService::EnableWakeWordDetection(bool enable) {
    ESP_LOGD(TAG, "%s wake word detection", enable ? "Enabling" : "Disabling");
    if (enable) {
        if (!InitializeAudioEngine() || !audio_engine_->HasWakeWord()) {
            xEventGroupClearBits(event_group_, AS_EVENT_WAKE_WORD_RUNNING);
            return;
        }
        {
            std::lock_guard<std::mutex> lock(input_resampler_mutex_);
            if (input_resampler_ != nullptr) {
                esp_ae_rate_cvt_reset(input_resampler_);
            }
        }
        audio_engine_->EnableWakeWordDetection(true);
        xEventGroupSetBits(event_group_, AS_EVENT_WAKE_WORD_RUNNING);
    } else {
        if (audio_engine_initialized_) {
            audio_engine_->EnableWakeWordDetection(false);
        }
        xEventGroupClearBits(event_group_, AS_EVENT_WAKE_WORD_RUNNING);
    }
}

void AudioService::EnableVoiceProcessing(bool enable, bool preserve_playback) {
    ESP_LOGD(TAG, "%s voice processing", enable ? "Enabling" : "Disabling");

    if (enable) {
        if (!InitializeAudioEngine()) {
            return;
        }
        if (!preserve_playback) {
            ResetDecoder();
        }
        audio_input_need_warmup_ = true;
        {
            std::lock_guard<std::mutex> lock(input_resampler_mutex_);
            if (input_resampler_ != nullptr) {
                esp_ae_rate_cvt_reset(input_resampler_);
            }
        }
        audio_engine_->EnableVoiceProcessing(true);
        xEventGroupSetBits(event_group_, AS_EVENT_AUDIO_PROCESSOR_RUNNING);
    } else {
        if (audio_engine_initialized_) {
            audio_engine_->EnableVoiceProcessing(false);
        }
        xEventGroupClearBits(event_group_, AS_EVENT_AUDIO_PROCESSOR_RUNNING);
    }
}

void AudioService::EnableAudioTesting(bool enable) {
    ESP_LOGI(TAG, "%s audio testing", enable ? "Enabling" : "Disabling");
    if (enable) {
        xEventGroupSetBits(event_group_, AS_EVENT_AUDIO_TESTING_RUNNING);
    } else {
        xEventGroupClearBits(event_group_, AS_EVENT_AUDIO_TESTING_RUNNING);
        /* Copy audio_testing_queue_ to audio_decode_queue_ */
        std::lock_guard<std::mutex> lock(audio_queue_mutex_);
        audio_decode_queue_ = std::move(audio_testing_queue_);
        if (!audio_decode_queue_.empty()) {
            playback_drained_notified_ = false;
        }
        audio_queue_cv_.notify_all();
    }
}

void AudioService::EnableDeviceAec(bool enable) {
    ESP_LOGI(TAG, "%s device AEC", enable ? "Enabling" : "Disabling");
    device_aec_enabled_ = enable;

    if (audio_engine_initialized_) {
        audio_engine_->EnableDeviceAec(enable);
    } else {
        ESP_LOGI(TAG, "Deferring AEC change until the audio engine is initialized");
    }
}

void AudioService::SetCallbacks(AudioServiceCallbacks& callbacks) { callbacks_ = callbacks; }

void AudioService::PlaySound(const std::string_view& ogg) {
    if (!codec_->output_enabled()) {
        esp_timer_stop(audio_power_timer_);
        esp_timer_start_periodic(audio_power_timer_, AUDIO_POWER_CHECK_INTERVAL_MS * 1000);
        codec_->EnableOutput(true);
    }

    const auto* buf = reinterpret_cast<const uint8_t*>(ogg.data());
    size_t size = ogg.size();

    auto demuxer = std::make_unique<OggDemuxer>();
    demuxer->OnDemuxerFinished([this](const uint8_t* data, int sample_rate, size_t size) {
        auto packet = std::make_unique<AudioStreamPacket>();
        packet->sample_rate = sample_rate;
        packet->frame_duration = 60;
        packet->payload.resize(size);
        std::memcpy(packet->payload.data(), data, size);
        PushPacketToDecodeQueue(std::move(packet), true);
    });
    demuxer->Reset();
    demuxer->Process(buf, size);
}

void AudioService::PlayListenerFeedback(const std::string& feedback_id,
                                        const std::string_view& ogg, float gain_db) {
    if (feedback_id.empty() || ogg.empty()) {
        return;
    }
    CancelListenerFeedback();
    {
        std::lock_guard<std::mutex> lock(audio_queue_mutex_);
        listener_feedback_playing_ = true;
        listener_feedback_id_ = feedback_id;
    }

    if (!codec_->output_enabled()) {
        esp_timer_stop(audio_power_timer_);
        esp_timer_start_periodic(audio_power_timer_, AUDIO_POWER_CHECK_INTERVAL_MS * 1000);
        codec_->EnableOutput(true);
    }

    const auto* buf = reinterpret_cast<const uint8_t*>(ogg.data());
    size_t size = ogg.size();
    const float gain_linear = GainDbToLinear(gain_db);
    auto demuxer = std::make_unique<OggDemuxer>();
    demuxer->OnDemuxerFinished(
        [this, feedback_id, gain_linear](const uint8_t* data, int sample_rate, size_t size) {
            auto packet = std::make_unique<AudioStreamPacket>();
            packet->sample_rate = sample_rate;
            packet->frame_duration = OPUS_FRAME_DURATION_MS;
            packet->output_id = feedback_id;
            packet->listener_feedback = true;
            packet->gain_linear = gain_linear;
            packet->payload.resize(size);
            std::memcpy(packet->payload.data(), data, size);
            PushPacketToDecodeQueue(std::move(packet), true);
        });
    demuxer->Reset();
    demuxer->Process(buf, size);
}

void AudioService::CancelListenerFeedback() {
    bool notify_drained = false;
    {
        std::lock_guard<std::mutex> lock(audio_queue_mutex_);
        if (!listener_feedback_playing_) {
            return;
        }
        ++playback_generation_;
        const auto is_feedback = [](const auto& item) {
            return item != nullptr && item->listener_feedback;
        };
        audio_decode_queue_.erase(
            std::remove_if(audio_decode_queue_.begin(), audio_decode_queue_.end(), is_feedback),
            audio_decode_queue_.end());
        audio_playback_queue_.erase(
            std::remove_if(audio_playback_queue_.begin(), audio_playback_queue_.end(), is_feedback),
            audio_playback_queue_.end());
        std::unique_lock<std::mutex> decoder_lock(decoder_mutex_);
        if (opus_decoder_ != nullptr) {
            esp_opus_dec_reset(opus_decoder_);
        }
        decoder_lock.unlock();
        listener_feedback_playing_ = false;
        listener_feedback_id_.clear();
        notify_drained = MarkPlaybackDrainedLocked();
        audio_queue_cv_.notify_all();
    }
    if (notify_drained && callbacks_.on_playback_drained) {
        callbacks_.on_playback_drained();
    }
}

bool AudioService::IsListenerFeedbackPlaying() {
    std::lock_guard<std::mutex> lock(audio_queue_mutex_);
    return listener_feedback_playing_;
}

bool AudioService::IsIdle() {
    std::lock_guard<std::mutex> lock(audio_queue_mutex_);
    return audio_encode_queue_.empty() && IsPlaybackDrainedLocked() && audio_testing_queue_.empty();
}

bool AudioService::IsPlaybackIdle() {
    std::lock_guard<std::mutex> lock(audio_queue_mutex_);
    return IsPlaybackDrainedLocked();
}

void AudioService::ResetDecoder() {
    bool notify_drained = false;
    {
        std::lock_guard<std::mutex> lock(audio_queue_mutex_);
        ++playback_generation_;
        std::unique_lock<std::mutex> decoder_lock(decoder_mutex_);
        if (opus_decoder_ != nullptr) {
            esp_opus_dec_reset(opus_decoder_);
        }
        decoder_lock.unlock();
        timestamp_queue_.clear();
        audio_decode_queue_.clear();
        audio_playback_queue_.clear();
        audio_testing_queue_.clear();
        listener_feedback_playing_ = false;
        listener_feedback_id_.clear();
        decode_in_flight_listener_feedback_ = false;
        remote_playback_paused_ = false;
        playback_gain_linear_ = 1.0f;
        notify_drained = MarkPlaybackDrainedLocked();
        audio_queue_cv_.notify_all();
    }
    if (notify_drained && callbacks_.on_playback_drained) {
        callbacks_.on_playback_drained();
    }
}

void AudioService::ArmPlaybackMetric(uint32_t turn_id) {
#if CONFIG_VOICE_LATENCY_METRICS
    playback_metric_turn_id_.store(turn_id);
    playback_metric_armed_.store(true);
#else
    (void)turn_id;
#endif
}

bool AudioService::IsPlaybackDrainedLocked() const {
    return audio_decode_queue_.empty() && audio_playback_queue_.empty() && !decode_in_flight_ &&
           !output_in_flight_;
}

bool AudioService::MarkPlaybackDrainedLocked() {
    if (!IsPlaybackDrainedLocked() || playback_drained_notified_) {
        return false;
    }
    playback_drained_notified_ = true;
    return true;
}

int64_t AudioService::BufferedRemotePlaybackMsLocked(const std::string& output_id) const {
    int64_t buffered_ms = 0;
    for (const auto& packet : audio_decode_queue_) {
        if (packet != nullptr && packet->output_id == output_id) {
            buffered_ms += packet->frame_duration;
        }
    }
    for (const auto& task : audio_playback_queue_) {
        if (task != nullptr && task->output_id == output_id) {
            buffered_ms += task->frame_duration_ms;
        }
    }
    return buffered_ms;
}

bool AudioService::HasListenerFeedbackLocked() const {
    if (decode_in_flight_listener_feedback_) {
        return true;
    }
    for (const auto& packet : audio_decode_queue_) {
        if (packet != nullptr && packet->listener_feedback) {
            return true;
        }
    }
    for (const auto& task : audio_playback_queue_) {
        if (task != nullptr && task->listener_feedback) {
            return true;
        }
    }
    return false;
}

float AudioService::GainDbToLinear(float gain_db) {
    gain_db = std::clamp(gain_db, -60.0f, 0.0f);
    return std::pow(10.0f, gain_db / 20.0f);
}

void AudioService::ApplyPcmGain(std::vector<int16_t>& pcm, float gain_linear) {
    if (gain_linear >= 0.999f) {
        return;
    }
    gain_linear = std::clamp(gain_linear, 0.0f, 1.0f);
    for (auto& sample : pcm) {
        sample = static_cast<int16_t>(std::lround(static_cast<float>(sample) * gain_linear));
    }
}

void AudioService::CheckAndUpdateAudioPowerState() {
    auto now = std::chrono::steady_clock::now();
    auto input_elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_input_time_).count();
    auto output_elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_output_time_).count();
    if (input_elapsed > AUDIO_POWER_TIMEOUT_MS && codec_->input_enabled()) {
        // ADC continuous start/stop must run in the same task. Wake the audio
        // input task instead of closing the codec from the esp_timer task.
        xEventGroupSetBits(event_group_, AS_EVENT_AUDIO_INPUT_STOP_REQUEST);
    }
    if (output_elapsed > AUDIO_POWER_TIMEOUT_MS && codec_->output_enabled()) {
        // Keep TX clock when duplex RX is active; otherwise RX may stall on some boards.
        if (!(codec_->duplex() && codec_->input_enabled())) {
            codec_->EnableOutput(false);
        }
    }
    if (!codec_->input_enabled() && !codec_->output_enabled()) {
        esp_timer_stop(audio_power_timer_);
    }
}

void AudioService::SetModelsList(srmodel_list_t* models_list) {
    if (audio_engine_initialized_ && models_list_ != models_list) {
        ESP_LOGW(TAG, "Ignoring speech model replacement after audio engine initialization");
        return;
    }
    models_list_ = models_list;
}

bool AudioService::IsAfeWakeWord() {
    return audio_engine_initialized_ && audio_engine_->IsAfeWakeWord();
}

bool AudioService::InitializeAudioEngine() {
    if (!audio_engine_) {
        return false;
    }
    if (audio_engine_initialized_) {
        return true;
    }
    if (!audio_engine_->Initialize(codec_, OPUS_FRAME_DURATION_MS, models_list_)) {
        ESP_LOGE(TAG, "Failed to initialize audio engine");
        return false;
    }
    audio_engine_initialized_ = true;
    audio_engine_->EnableDeviceAec(device_aec_enabled_);
    return true;
}
