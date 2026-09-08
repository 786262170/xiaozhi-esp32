from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
BOARD_DIR = ROOT / "main/boards/espressif/esp32-s3-korvo-2-v3.0"


class StoryPhoneFeatureTests(unittest.TestCase):
    def test_korvo2_gpio4_hook_is_enabled_and_debounced(self):
        config = (BOARD_DIR / "config.json").read_text(encoding="utf-8")
        config_h = (BOARD_DIR / "config.h").read_text(encoding="utf-8")
        board = (BOARD_DIR / "esp32s3_korvo2_v3_board.cc").read_text(
            encoding="utf-8"
        )
        kconfig = (ROOT / "main/Kconfig.projbuild").read_text(encoding="utf-8")

        self.assertIn("CONFIG_ESP32S3_KORVO2_V3_HANDSET_HOOK_GPIO4=y", config)
        self.assertIn("HANDSET_HOOK_GPIO GPIO_NUM_4", config_h)
        self.assertIn("config ESP32S3_KORVO2_V3_HANDSET_HOOK_GPIO4", kconfig)
        self.assertIn("kHandsetHookDebounceUs = 80 * 1000", board)
        self.assertIn("handset_hook_armed_", board)
        self.assertIn("SetPhoneHookState(off_hook)", board)

    def test_phone_call_flow_and_livekit_capability_are_present(self):
        application = (ROOT / "main/application.cc").read_text(encoding="utf-8")
        application_h = (ROOT / "main/application.h").read_text(encoding="utf-8")
        protocol_h = (ROOT / "main/protocols/protocol.h").read_text(encoding="utf-8")
        webrtc = (ROOT / "main/protocols/webrtc_protocol.cc").read_text(
            encoding="utf-8"
        )

        for marker in (
            "TogglePhoneChatState",
            "BeginPhoneCall",
            "HangUpPhoneCall",
            "OGG_PHONE_CONNECT",
            "OGG_PHONE_RINGBACK",
            "OGG_PHONE_HANGUP",
            "OGG_PHONE_FAILED",
        ):
            self.assertIn(marker, application)
        self.assertIn("PhoneCallController", application_h)
        self.assertIn("SendPhoneHangupRequest", protocol_h)
        self.assertIn(
            'cJSON_AddBoolToObject(features, "phone_hangup", true)', webrtc
        )

    def test_phone_ogg_assets_are_embedded_sources(self):
        for filename in (
            "phone_dial.ogg",
            "phone_ringback.ogg",
            "phone_connect.ogg",
            "phone_hangup.ogg",
            "phone_failed.ogg",
        ):
            payload = (ROOT / "main/assets/common" / filename).read_bytes()
            self.assertGreater(len(payload), 64)
            self.assertEqual(payload[:4], b"OggS")


if __name__ == "__main__":
    unittest.main()
