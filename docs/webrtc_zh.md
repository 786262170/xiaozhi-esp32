# WebRTC（LiveKit）传输

固件把 WebRTC 作为与 WebSocket、MQTT+UDP 并列的会话传输。语音命令、MCP、
打断和 TTS 状态仍使用原有小智 JSON 协议；控制消息走可靠 DataChannel，音频走
LiveKit Opus 音轨。

## 启用条件

- 当前实现支持带 PSRAM 和硬件 HMAC 的 ESP32-S3/ESP32-P4；首个启用并验证编译的
  板型是 `esp32-s3-korvo-2-v3.0`。
- 板型配置需要启用 `CONFIG_USE_WEBRTC=y`。Korvo-2 配置已包含所需 WebSocket、
  DTLS-SRTP 和 FreeRTOS timer 配置。
- 设备必须完成安全烧录：eFuse `HMAC_KEY0` 应与 Manager 中设备的
  `pre_secret_key` 对应。没有密钥或硬件 HMAC 失败时，WebRTC 会关闭并尝试 OTA
  指定的后备传输，不会在固件或 OTA 中下发长期密钥。

## OTA 协议

OTA 服务启用 WebRTC 时返回：

```json
{
  "webrtc": {
    "enabled": true,
    "provider": "livekit",
    "session_url": "https://api.example.com/xiaozhi/rtc/session",
    "sdk_version": "0.3.10",
    "topics": {
      "command": "xiaozhi.cmd.v1",
      "event": "xiaozhi.evt.v1"
    }
  },
  "transport_policy": {
    "preferred": "webrtc",
    "fallback": ["websocket", "mqtt_udp"],
    "connect_timeout_ms": 8000
  }
}
```

`preferred` 和 `fallback` 支持 `webrtc`、`websocket`、`mqtt_udp`；`mqtt`、`udp`
会归一化为 `mqtt_udp`。同一会话只选择一个传输。WebRTC bootstrap、入房、服务端
参与者或 hello 超时后，会完整释放 RTC 音频和房间资源，再尝试下一项。

## 会话和音频

1. 固件用 UTC Unix 时间、随机 nonce 和 eFuse `HMAC_KEY0` 签名
   `POST /xiaozhi/rtc/session`。
2. 服务端验证设备、时间窗和 nonce，返回短期 LiveKit token、房间、服务端身份及
   DataChannel topic。
3. 固件只发布 16 kHz、单声道的后 AFE PCM；LiveKit SDK 编码为 Opus。服务端 Opus
   音轨解码后直接进入现有扬声器 PCM 队列。
4. 音频交接队列有界；实时生产线程不阻塞，消费者落后时丢弃最旧帧。
5. 只有房间已连接、预期服务端参与者已激活后才发送小智 hello，避免首条未缓冲
   DataChannel 消息丢失。

## 构建

LiveKit SDK 固定到远端 commit，构建不依赖本机绝对路径：

```bash
source /path/to/esp-idf-v6.0.2/export.sh
python3 scripts/build.py espressif/esp32-s3-korvo-2-v3.0 \
  --name esp32s3-korvo2-v3
```

编译成功只证明依赖、接口和镜像可用。正式验收还需烧录真实设备，检查 bootstrap、
ICE/TURN、上下行音频、连续多轮、打断/AEC、断网重连、WSS/MQTT+UDP 回退，以及
持续运行时的最小堆和任务栈余量。
