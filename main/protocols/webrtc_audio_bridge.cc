#include "webrtc_audio_bridge.h"

#include <cstring>
#include <new>
#include <vector>

#include <esp_log.h>
#include <esp_timer.h>

namespace {
constexpr char kTag[] = "WebRTCAudio";
constexpr uint32_t kCaptureSampleRate = 16000;
constexpr uint8_t kCaptureChannels = 1;
constexpr uint8_t kBitsPerSample = 16;
constexpr int kCaptureReadTimeoutMs = 30;
}  // namespace

WebRTCAudioBridge::WebRTCAudioBridge(AudioService& audio_service) : audio_service_(audio_service) {
    capture_source_.owner = this;
    capture_source_.base.open = CaptureOpen;
    capture_source_.base.get_support_codecs = CaptureGetCodecs;
    capture_source_.base.set_fixed_caps = CaptureSetFixedCaps;
    capture_source_.base.negotiate_caps = CaptureNegotiateCaps;
    capture_source_.base.start = CaptureStart;
    capture_source_.base.read_frame = CaptureRead;
    capture_source_.base.stop = CaptureStop;
    capture_source_.base.close = CaptureClose;
}

WebRTCAudioBridge::~WebRTCAudioBridge() { Shutdown(); }

bool WebRTCAudioBridge::Initialize() {
    if (initialized_.load()) {
        return true;
    }

    esp_capture_cfg_t capture_config = {
        .sync_mode = ESP_CAPTURE_SYNC_MODE_AUDIO,
        .audio_src = &capture_source_.base,
    };
    if (esp_capture_open(&capture_config, &capturer_) != ESP_CAPTURE_ERR_OK ||
        capturer_ == nullptr) {
        ESP_LOGE(kTag, "Failed to open external PCM capture");
        Shutdown();
        return false;
    }

    RendererConfig renderer_context = {.audio_service = &audio_service_};
    audio_render_cfg_t audio_config = {
        .ops =
            {
                .init = RendererInit,
                .open = RendererOpen,
                .write = RendererWrite,
                .get_latency = RendererGetLatency,
                .get_frame_info = RendererGetFrameInfo,
                .set_speed = RendererSetSpeed,
                .close = RendererClose,
                .deinit = RendererDeinit,
            },
        .cfg = &renderer_context,
        .cfg_size = sizeof(renderer_context),
    };
    audio_renderer_ = audio_render_alloc_handle(&audio_config);
    if (audio_renderer_ == nullptr) {
        ESP_LOGE(kTag, "Failed to allocate external PCM renderer");
        Shutdown();
        return false;
    }

    av_render_cfg_t render_config = {
        .audio_render = audio_renderer_,
        .sync_mode = AV_RENDER_SYNC_NONE,
        .audio_raw_fifo_size = 8 * 4096,
        .audio_render_fifo_size = 32 * 1024,
        .quit_when_eos = false,
        .allow_drop_data = true,
    };
    renderer_ = av_render_open(&render_config);
    if (renderer_ == nullptr) {
        ESP_LOGE(kTag, "Failed to open AV renderer");
        Shutdown();
        return false;
    }

    av_render_audio_frame_info_t frame_info = {
        .channel = 1,
        .bits_per_sample = kBitsPerSample,
        .sample_rate = static_cast<uint32_t>(audio_service_.output_sample_rate()),
    };
    if (av_render_set_fixed_frame_info(renderer_, &frame_info) != 0) {
        ESP_LOGE(kTag, "Failed to set renderer output format");
        Shutdown();
        return false;
    }

    initialized_.store(true);
    ESP_LOGI(kTag, "Audio bridge ready: capture=%lu Hz, playback=%d Hz",
             static_cast<unsigned long>(kCaptureSampleRate), audio_service_.output_sample_rate());
    return true;
}

void WebRTCAudioBridge::Shutdown() {
    audio_service_.EnableExternalPcmCapture(false);
    initialized_.store(false);

    if (renderer_ != nullptr) {
        av_render_close(renderer_);
        renderer_ = nullptr;
    }
    if (audio_renderer_ != nullptr) {
        audio_render_free_handle(audio_renderer_);
        audio_renderer_ = nullptr;
    }
    if (capturer_ != nullptr) {
        esp_capture_close(capturer_);
        capturer_ = nullptr;
    }
}

WebRTCAudioBridge::CaptureSource* WebRTCAudioBridge::CaptureFrom(
    esp_capture_audio_src_if_t* source) {
    return reinterpret_cast<CaptureSource*>(source);
}

esp_capture_err_t WebRTCAudioBridge::CaptureOpen(esp_capture_audio_src_if_t* source) {
    if (source == nullptr) {
        return ESP_CAPTURE_ERR_INVALID_ARG;
    }
    auto* capture = CaptureFrom(source);
    capture->opened = true;
    return ESP_CAPTURE_ERR_OK;
}

esp_capture_err_t WebRTCAudioBridge::CaptureGetCodecs(esp_capture_audio_src_if_t* source,
                                                      const esp_capture_format_id_t** codecs,
                                                      uint8_t* count) {
    if (source == nullptr || codecs == nullptr || count == nullptr) {
        return ESP_CAPTURE_ERR_INVALID_ARG;
    }
    static const esp_capture_format_id_t kCodecs[] = {ESP_CAPTURE_FMT_ID_PCM};
    *codecs = kCodecs;
    *count = 1;
    return ESP_CAPTURE_ERR_OK;
}

esp_capture_err_t WebRTCAudioBridge::CaptureSetFixedCaps(esp_capture_audio_src_if_t* source,
                                                         const esp_capture_audio_info_t* caps) {
    if (source == nullptr || caps == nullptr || caps->format_id != ESP_CAPTURE_FMT_ID_PCM ||
        caps->sample_rate != kCaptureSampleRate || caps->channel != kCaptureChannels ||
        caps->bits_per_sample != kBitsPerSample) {
        return ESP_CAPTURE_ERR_INVALID_ARG;
    }
    auto* capture = CaptureFrom(source);
    if (capture->started) {
        return ESP_CAPTURE_ERR_INVALID_STATE;
    }
    capture->info = *caps;
    return ESP_CAPTURE_ERR_OK;
}

esp_capture_err_t WebRTCAudioBridge::CaptureNegotiateCaps(esp_capture_audio_src_if_t* source,
                                                          esp_capture_audio_info_t* requested,
                                                          esp_capture_audio_info_t* negotiated) {
    if (source == nullptr || requested == nullptr || negotiated == nullptr ||
        requested->format_id != ESP_CAPTURE_FMT_ID_PCM) {
        return ESP_CAPTURE_ERR_NOT_SUPPORTED;
    }
    auto* capture = CaptureFrom(source);
    capture->info = {
        .format_id = ESP_CAPTURE_FMT_ID_PCM,
        .sample_rate = kCaptureSampleRate,
        .channel = kCaptureChannels,
        .bits_per_sample = kBitsPerSample,
    };
    *negotiated = capture->info;
    return ESP_CAPTURE_ERR_OK;
}

esp_capture_err_t WebRTCAudioBridge::CaptureStart(esp_capture_audio_src_if_t* source) {
    if (source == nullptr) {
        return ESP_CAPTURE_ERR_INVALID_ARG;
    }
    auto* capture = CaptureFrom(source);
    if (!capture->opened) {
        return ESP_CAPTURE_ERR_INVALID_STATE;
    }
    capture->started = true;
    capture->owner->audio_service_.EnableExternalPcmCapture(true);
    return ESP_CAPTURE_ERR_OK;
}

esp_capture_err_t WebRTCAudioBridge::CaptureRead(esp_capture_audio_src_if_t* source,
                                                 esp_capture_stream_frame_t* frame) {
    if (source == nullptr || frame == nullptr || frame->data == nullptr || frame->size <= 0 ||
        frame->size % sizeof(int16_t) != 0) {
        return ESP_CAPTURE_ERR_INVALID_ARG;
    }
    auto* capture = CaptureFrom(source);
    if (!capture->started) {
        return ESP_CAPTURE_ERR_INVALID_STATE;
    }
    const size_t samples = static_cast<size_t>(frame->size) / sizeof(int16_t);
    if (!capture->owner->audio_service_.ReadExternalPcm(reinterpret_cast<int16_t*>(frame->data),
                                                        samples, kCaptureReadTimeoutMs)) {
        return ESP_CAPTURE_ERR_INVALID_STATE;
    }
    frame->pts = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    return ESP_CAPTURE_ERR_OK;
}

esp_capture_err_t WebRTCAudioBridge::CaptureStop(esp_capture_audio_src_if_t* source) {
    if (source == nullptr) {
        return ESP_CAPTURE_ERR_INVALID_ARG;
    }
    auto* capture = CaptureFrom(source);
    capture->owner->audio_service_.EnableExternalPcmCapture(false);
    capture->started = false;
    return ESP_CAPTURE_ERR_OK;
}

esp_capture_err_t WebRTCAudioBridge::CaptureClose(esp_capture_audio_src_if_t* source) {
    if (source == nullptr) {
        return ESP_CAPTURE_ERR_INVALID_ARG;
    }
    auto* capture = CaptureFrom(source);
    capture->owner->audio_service_.EnableExternalPcmCapture(false);
    capture->started = false;
    capture->opened = false;
    return ESP_CAPTURE_ERR_OK;
}

audio_render_handle_t WebRTCAudioBridge::RendererInit(void* config, int config_size) {
    if (config == nullptr || config_size != sizeof(RendererConfig)) {
        return nullptr;
    }
    auto* renderer_config = static_cast<RendererConfig*>(config);
    return new (std::nothrow) RendererContext{.audio_service = renderer_config->audio_service};
}

int WebRTCAudioBridge::RendererOpen(audio_render_handle_t renderer,
                                    av_render_audio_frame_info_t* info) {
    if (renderer == nullptr || info == nullptr || info->bits_per_sample != kBitsPerSample ||
        info->channel != 1) {
        return -1;
    }
    static_cast<RendererContext*>(renderer)->info = *info;
    return 0;
}

int WebRTCAudioBridge::RendererWrite(audio_render_handle_t renderer,
                                     av_render_audio_frame_t* frame) {
    if (renderer == nullptr || frame == nullptr) {
        return -1;
    }
    if (frame->eos || frame->size == 0) {
        return 0;
    }
    if (frame->data == nullptr || frame->size < 0 || frame->size % sizeof(int16_t) != 0) {
        return -1;
    }
    auto* context = static_cast<RendererContext*>(renderer);
    const auto* begin = reinterpret_cast<const int16_t*>(frame->data);
    std::vector<int16_t> pcm(begin, begin + frame->size / sizeof(int16_t));
    return context->audio_service->PushPcmToPlaybackQueue(std::move(pcm)) ? 0 : -1;
}

int WebRTCAudioBridge::RendererGetLatency(audio_render_handle_t renderer, uint32_t* latency_ms) {
    if (renderer == nullptr || latency_ms == nullptr) {
        return -1;
    }
    *latency_ms = 0;
    return 0;
}

int WebRTCAudioBridge::RendererGetFrameInfo(audio_render_handle_t renderer,
                                            av_render_audio_frame_info_t* info) {
    if (renderer == nullptr || info == nullptr) {
        return -1;
    }
    *info = static_cast<RendererContext*>(renderer)->info;
    return 0;
}

int WebRTCAudioBridge::RendererSetSpeed(audio_render_handle_t renderer, float speed) {
    return renderer != nullptr && speed >= 0.0f ? 0 : -1;
}

int WebRTCAudioBridge::RendererClose(audio_render_handle_t renderer) {
    return renderer == nullptr ? -1 : 0;
}

void WebRTCAudioBridge::RendererDeinit(audio_render_handle_t renderer) {
    delete static_cast<RendererContext*>(renderer);
}
