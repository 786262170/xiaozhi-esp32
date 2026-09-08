#ifndef WEBRTC_AUDIO_BRIDGE_H
#define WEBRTC_AUDIO_BRIDGE_H

#include "audio_service.h"

#include <atomic>

#include <esp_capture.h>
#include <audio_render.h>
#include <av_render.h>

class WebRTCAudioBridge {
public:
    explicit WebRTCAudioBridge(AudioService& audio_service);
    ~WebRTCAudioBridge();

    WebRTCAudioBridge(const WebRTCAudioBridge&) = delete;
    WebRTCAudioBridge& operator=(const WebRTCAudioBridge&) = delete;

    bool Initialize();
    void Shutdown();

    esp_capture_handle_t capturer() const { return capturer_; }
    av_render_handle_t renderer() const { return renderer_; }

private:
    struct CaptureSource {
        esp_capture_audio_src_if_t base = {};
        WebRTCAudioBridge* owner = nullptr;
        esp_capture_audio_info_t info = {};
        bool opened = false;
        bool started = false;
    };

    struct RendererConfig {
        AudioService* audio_service;
    };

    struct RendererContext {
        AudioService* audio_service;
        av_render_audio_frame_info_t info = {};
    };

    AudioService& audio_service_;
    CaptureSource capture_source_;
    esp_capture_handle_t capturer_ = nullptr;
    audio_render_handle_t audio_renderer_ = nullptr;
    av_render_handle_t renderer_ = nullptr;
    std::atomic<bool> initialized_{false};

    static CaptureSource* CaptureFrom(esp_capture_audio_src_if_t* source);
    static esp_capture_err_t CaptureOpen(esp_capture_audio_src_if_t* source);
    static esp_capture_err_t CaptureGetCodecs(esp_capture_audio_src_if_t* source,
                                              const esp_capture_format_id_t** codecs,
                                              uint8_t* count);
    static esp_capture_err_t CaptureSetFixedCaps(esp_capture_audio_src_if_t* source,
                                                 const esp_capture_audio_info_t* caps);
    static esp_capture_err_t CaptureNegotiateCaps(esp_capture_audio_src_if_t* source,
                                                  esp_capture_audio_info_t* requested,
                                                  esp_capture_audio_info_t* negotiated);
    static esp_capture_err_t CaptureStart(esp_capture_audio_src_if_t* source);
    static esp_capture_err_t CaptureRead(esp_capture_audio_src_if_t* source,
                                         esp_capture_stream_frame_t* frame);
    static esp_capture_err_t CaptureStop(esp_capture_audio_src_if_t* source);
    static esp_capture_err_t CaptureClose(esp_capture_audio_src_if_t* source);

    static audio_render_handle_t RendererInit(void* config, int config_size);
    static int RendererOpen(audio_render_handle_t renderer, av_render_audio_frame_info_t* info);
    static int RendererWrite(audio_render_handle_t renderer, av_render_audio_frame_t* frame);
    static int RendererGetLatency(audio_render_handle_t renderer, uint32_t* latency_ms);
    static int RendererGetFrameInfo(audio_render_handle_t renderer,
                                    av_render_audio_frame_info_t* info);
    static int RendererSetSpeed(audio_render_handle_t renderer, float speed);
    static int RendererClose(audio_render_handle_t renderer);
    static void RendererDeinit(audio_render_handle_t renderer);
};

#endif  // WEBRTC_AUDIO_BRIDGE_H
