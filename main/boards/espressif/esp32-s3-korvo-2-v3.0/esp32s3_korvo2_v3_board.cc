#ifdef KORVO2_ML307_BOARD
#include "ml307_board.h"
using Korvo2NetworkBoard = Ml307Board;
#elif defined(KORVO2_CLM920_RNDIS_BOARD)
#include "rndis_board.h"
using Korvo2NetworkBoard = RndisBoard;
#else
#include "wifi_board.h"
using Korvo2NetworkBoard = WifiBoard;
#endif
#include "application.h"
#include "assets/lang_config.h"
#include "button.h"
#include "codecs/box_audio_codec.h"
#include "config.h"
#include "display/lcd_display.h"
#include "i2c_device.h"
#include "settings.h"

#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_io_expander_tca9554.h>
#include <esp_lcd_ili9341.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <esp_timer.h>
#ifdef KORVO2_ML307_BOARD
#include <driver/uart.h>
#include <hal/usb_serial_jtag_ll.h>
#include <hal/usb_wrap_ll.h>
#endif
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp32_camera.h"
#include "power_manager.h"
#include "power_save_timer.h"

#include <atomic>

#define TAG "esp32s3_korvo2_v3"

namespace {
constexpr int kStoryPhoneDefaultVolume = 80;
constexpr const char* kStoryPhoneVolumeMigrationKey = "phone_vol80_v1";
#if CONFIG_ESP32S3_KORVO2_V3_HANDSET_HOOK_GPIO4
constexpr int64_t kHandsetHookDebounceUs = 80 * 1000;
#endif
}  // namespace
/* ADC Buttons */
typedef enum {
    BSP_ADC_BUTTON_REC,
    BSP_ADC_BUTTON_VOL_MUTE,
    BSP_ADC_BUTTON_PLAY,
    BSP_ADC_BUTTON_SET,
    BSP_ADC_BUTTON_VOL_DOWN,
    BSP_ADC_BUTTON_VOL_UP,
    BSP_ADC_BUTTON_NUM
} bsp_adc_button_t;

typedef struct {
    int min_mv;
    int max_mv;
    const char* name;
} bsp_adc_button_range_t;

static constexpr bsp_adc_button_range_t kAdcButtonRanges[BSP_ADC_BUTTON_NUM] = {
    {2310, 2510, "REC"}, {1880, 2080, "MUTE"}, {1550, 1750, "PLAY"},
    {1015, 1215, "SET"}, {720, 920, "VOL-"},   {280, 480, "VOL+"},
};

// Init ili9341 by custom cmd
static const ili9341_lcd_init_cmd_t vendor_specific_init[] = {
    {0xC8, (uint8_t[]){0xFF, 0x93, 0x42}, 3, 0},
    {0xC0, (uint8_t[]){0x0E, 0x0E}, 2, 0},
    {0xC5, (uint8_t[]){0xD0}, 1, 0},
    {0xC1, (uint8_t[]){0x02}, 1, 0},
    {0xB4, (uint8_t[]){0x02}, 1, 0},
    {0xE0,
     (uint8_t[]){0x00, 0x03, 0x08, 0x06, 0x13, 0x09, 0x39, 0x39, 0x48, 0x02, 0x0a, 0x08, 0x17, 0x17,
                 0x0F},
     15, 0},
    {0xE1,
     (uint8_t[]){0x00, 0x28, 0x29, 0x01, 0x0d, 0x03, 0x3f, 0x33, 0x52, 0x04, 0x0f, 0x0e, 0x37, 0x38,
                 0x0F},
     15, 0},

    {0xB1, (uint8_t[]){00, 0x1B}, 2, 0},
    {0x36, (uint8_t[]){0x08}, 1, 0},
    {0x3A, (uint8_t[]){0x55}, 1, 0},
    {0xB7, (uint8_t[]){0x06}, 1, 0},

    {0x11, (uint8_t[]){0}, 0x80, 0},
    {0x29, (uint8_t[]){0}, 0x80, 0},

    {0, (uint8_t[]){0}, 0xff, 0},
};

class Esp32S3Korvo2V3Board : public Korvo2NetworkBoard {
private:
    Button boot_button_;
#if CONFIG_ESP32S3_KORVO2_V3_HANDSET_HOOK_GPIO4
    Button handset_hook_button_{HANDSET_HOOK_GPIO};
    std::atomic<bool> handset_hook_armed_{false};
    esp_timer_handle_t handset_hook_debounce_timer_ = nullptr;
#endif
    Button* adc_button_[BSP_ADC_BUTTON_NUM];
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    adc_oneshot_unit_handle_t bsp_adc_handle = NULL;
#endif
    i2c_master_bus_handle_t i2c_bus_;
    LcdDisplay* display_;
    esp_io_expander_handle_t io_expander_ = NULL;
    Esp32Camera* camera_;
    PowerSaveTimer* power_save_timer_;
    PowerManager* power_manager_;

    void InitializeStoryPhoneDefaultVolume() {
        Settings settings("audio", true);
        if (settings.GetBool(kStoryPhoneVolumeMigrationKey, false)) {
            return;
        }

        settings.SetInt("output_volume", kStoryPhoneDefaultVolume);
        settings.SetBool(kStoryPhoneVolumeMigrationKey, true);
        ESP_LOGI(TAG, "Story phone output volume initialized to %d (one-time migration)",
                 kStoryPhoneDefaultVolume);
    }

#ifdef KORVO2_ML307_BOARD
    struct Ml307ProbePins {
        gpio_num_t tx;
        gpio_num_t rx;
    };

    void ReleaseUsbPadsForMl307() {
        // GPIO19/20 are also the ESP32-S3 native USB D-/D+ pads.  The ML307
        // variant uses them as UART pins, so release both USB controllers
        // explicitly before installing the UART driver.
        usb_serial_jtag_ll_phy_enable_pad(false);
        usb_wrap_ll_phy_disable_pull_override(&USB_WRAP);
        usb_wrap_ll_phy_enable_pad(&USB_WRAP, false);
        gpio_reset_pin(ML307_TX_PIN);
        gpio_reset_pin(ML307_RX_PIN);
    }

    bool ProbeMl307AtDirection(const Ml307ProbePins& pins, int& detected_baud,
                               int& received_bytes) {
        static constexpr int kProbeBaudRates[] = {
            115200, 921600, 460800, 230400, 57600, 38400, 19200, 9600,
        };
        static constexpr char kAtCommand[] = "AT\r\n";

        ESP_ERROR_CHECK_WITHOUT_ABORT(
            uart_set_pin(UART_NUM_1, pins.tx, pins.rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        gpio_set_pull_mode(pins.rx, GPIO_PULLUP_ONLY);

        for (int baud : kProbeBaudRates) {
            uart_set_baudrate(UART_NUM_1, baud);
            uart_flush_input(UART_NUM_1);
            uart_write_bytes(UART_NUM_1, kAtCommand, sizeof(kAtCommand) - 1);
            uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(100));

            std::string response;
            TickType_t start = xTaskGetTickCount();
            while (xTaskGetTickCount() - start < pdMS_TO_TICKS(300)) {
                uint8_t buffer[64];
                int length = uart_read_bytes(UART_NUM_1, buffer, sizeof(buffer), pdMS_TO_TICKS(50));
                if (length > 0) {
                    response.append(reinterpret_cast<const char*>(buffer), length);
                    if (response.find("OK") != std::string::npos) {
                        break;
                    }
                }
            }

            received_bytes = static_cast<int>(response.size());
            char status[64];
            snprintf(status, sizeof(status), "ML307 AT %d>%d %d RX=%d", static_cast<int>(pins.tx),
                     static_cast<int>(pins.rx), baud, received_bytes);
            display_->SetStatus(status);
            ESP_LOGI(TAG, "%s", status);

            if (response.find("OK") != std::string::npos) {
                detected_baud = baud;
                return true;
            }
        }
        return false;
    }

    bool ProbeMl307At(int& detected_baud, int& received_bytes) {
        const uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
            .source_clk = UART_SCLK_DEFAULT,
            .flags = {},
        };

        esp_err_t result = uart_param_config(UART_NUM_1, &uart_config);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "ML307 probe UART config failed: %s", esp_err_to_name(result));
            return false;
        }
        result = uart_driver_install(UART_NUM_1, 1024, 0, 0, nullptr, 0);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "ML307 probe UART install failed: %s", esp_err_to_name(result));
            return false;
        }

        static constexpr Ml307ProbePins kProbeDirections[] = {
            {ML307_TX_PIN, ML307_RX_PIN},
            {ML307_RX_PIN, ML307_TX_PIN},
        };

        bool found = false;
        for (const auto& pins : kProbeDirections) {
            if (ProbeMl307AtDirection(pins, detected_baud, received_bytes)) {
                tx_pin_ = pins.tx;
                rx_pin_ = pins.rx;
                found = true;
                break;
            }
        }

        if (found && detected_baud != 921600) {
            static constexpr char kSetBaudCommand[] = "AT+IPR=921600\r\n";
            uart_write_bytes(UART_NUM_1, kSetBaudCommand, sizeof(kSetBaudCommand) - 1);
            uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(100));
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        uart_driver_delete(UART_NUM_1);
        return found;
    }

    static void Ml307ProbeTask(void* arg) {
        auto* board = static_cast<Esp32S3Korvo2V3Board*>(arg);
        int attempt = 0;
        while (true) {
            ++attempt;
            int detected_baud = 0;
            int received_bytes = 0;
            board->ReleaseUsbPadsForMl307();
            if (board->ProbeMl307At(detected_baud, received_bytes)) {
                char status[64];
                snprintf(status, sizeof(status), "ML307 AT OK %d>%d %d",
                         static_cast<int>(board->tx_pin_), static_cast<int>(board->rx_pin_),
                         detected_baud);
                board->display_->SetStatus(status);
                ESP_LOGI(TAG, "%s", status);
                vTaskDelay(pdMS_TO_TICKS(200));
                board->Korvo2NetworkBoard::StartNetwork();
                vTaskDelete(nullptr);
                return;
            }

            char status[64];
            snprintf(status, sizeof(status), "ML307 NO AT #%d RX=%d", attempt, received_bytes);
            board->display_->SetStatus(status);
            ESP_LOGW(TAG, "%s", status);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
#endif

#if CONFIG_ESP32S3_KORVO2_V3_ADC_BUTTON_DIAGNOSTICS
    static int MatchAdcButton(int voltage_mv) {
        for (int i = 0; i < BSP_ADC_BUTTON_NUM; ++i) {
            if (voltage_mv >= kAdcButtonRanges[i].min_mv &&
                voltage_mv <= kAdcButtonRanges[i].max_mv) {
                return i;
            }
        }
        return -1;
    }

    static void AdcButtonDiagnosticTask(void* arg) {
        auto* board = static_cast<Esp32S3Korvo2V3Board*>(arg);
        adc_cali_handle_t calibration_handle = nullptr;
        const adc_cali_curve_fitting_config_t calibration_config = {
            .unit_id = ADC_UNIT_1,
            .chan = ADC_CHANNEL_4,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };
        esp_err_t calibration_result =
            adc_cali_create_scheme_curve_fitting(&calibration_config, &calibration_handle);
        if (calibration_result != ESP_OK) {
            ESP_LOGW(TAG, "ADC button diagnostics has no calibration: %s",
                     esp_err_to_name(calibration_result));
        }

        ESP_LOGI(TAG,
                 "ADC button diagnostics enabled on GPIO5/ADC1_CH4; "
                 "REC expected at 2310-2510mV");

        int last_raw = -1;
        int last_button = -2;
        TickType_t last_log_tick = 0;
        while (true) {
            int raw = 0;
            esp_err_t read_result = adc_oneshot_read(board->bsp_adc_handle, ADC_CHANNEL_4, &raw);
            if (read_result != ESP_OK) {
                if (read_result != ESP_ERR_TIMEOUT) {
                    ESP_LOGW(TAG, "ADC button diagnostic read failed: %s",
                             esp_err_to_name(read_result));
                }
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            int voltage_mv = -1;
            int matched_button = -1;
            if (calibration_handle != nullptr &&
                adc_cali_raw_to_voltage(calibration_handle, raw, &voltage_mv) == ESP_OK) {
                matched_button = MatchAdcButton(voltage_mv);
            }

            TickType_t now = xTaskGetTickCount();
            int raw_delta = last_raw < 0 ? 0 : (raw > last_raw ? raw - last_raw : last_raw - raw);
            bool state_changed = matched_button != last_button;
            bool raw_changed = last_raw >= 0 && raw_delta >= 160;
            bool heartbeat_due = last_log_tick == 0 || now - last_log_tick >= pdMS_TO_TICKS(15000);
            if (state_changed || raw_changed || heartbeat_due) {
                const char* button_name =
                    matched_button >= 0 ? kAdcButtonRanges[matched_button].name : "NONE";
                ESP_LOGI(TAG, "ADC button sample: raw=%d, voltage=%dmV, matched=%s", raw,
                         voltage_mv, button_name);
                last_raw = raw;
                last_button = matched_button;
                last_log_tick = now;
            }

            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    void InitializeAdcButtonDiagnostics() {
        BaseType_t result =
            xTaskCreate(AdcButtonDiagnosticTask, "adc_button_diag", 3072, this, 2, nullptr);
        if (result != pdPASS) {
            ESP_LOGE(TAG, "Failed to start ADC button diagnostic task");
        }
    }
#endif

    void InitializePowerManager() {
        // PowerManager需要复用按钮的ADC句柄，所以在InitializeButtons之后调用
        // 传入按钮的ADC句柄指针，让PowerManager复用
        power_manager_ = new PowerManager(GPIO_NUM_NC, &bsp_adc_handle);
    }

    void InitializePowerSaveTimer() {
        power_save_timer_ = new PowerSaveTimer(-1, 60);
        power_save_timer_->OnEnterSleepMode([this]() { GetDisplay()->SetPowerSaveMode(true); });
        power_save_timer_->OnExitSleepMode([this]() { GetDisplay()->SetPowerSaveMode(false); });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)1,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags =
                {
                    .enable_internal_pullup = 1,
                },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void I2cDetect() {
        uint8_t address;
        printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
        for (int i = 0; i < 128; i += 16) {
            printf("%02x: ", i);
            for (int j = 0; j < 16; j++) {
                fflush(stdout);
                address = i + j;
                esp_err_t ret = i2c_master_probe(i2c_bus_, address, pdMS_TO_TICKS(200));
                if (ret == ESP_OK) {
                    printf("%02x ", address);
                } else if (ret == ESP_ERR_TIMEOUT) {
                    printf("UU ");
                } else {
                    printf("-- ");
                }
            }
            printf("\r\n");
        }
    }

    void InitializeTca9554() {
        esp_err_t ret = esp_io_expander_new_i2c_tca9554(
            i2c_bus_, ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000, &io_expander_);
        if (ret != ESP_OK) {
            ret = esp_io_expander_new_i2c_tca9554(
                i2c_bus_, ESP_IO_EXPANDER_I2C_TCA9554A_ADDRESS_000, &io_expander_);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "TCA9554 create returned error");
                return;
            }
        }
        // 配置IO0-IO3为输出模式
        ESP_ERROR_CHECK(esp_io_expander_set_dir(io_expander_,
                                                IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1 |
                                                    IO_EXPANDER_PIN_NUM_2 | IO_EXPANDER_PIN_NUM_3,
                                                IO_EXPANDER_OUTPUT));

        // 复位LCD和TouchPad
        ESP_ERROR_CHECK(esp_io_expander_set_level(
            io_expander_, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1 | IO_EXPANDER_PIN_NUM_2,
            1));
        vTaskDelay(pdMS_TO_TICKS(300));
        ESP_ERROR_CHECK(esp_io_expander_set_level(
            io_expander_, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1 | IO_EXPANDER_PIN_NUM_2,
            0));
        vTaskDelay(pdMS_TO_TICKS(300));
        ESP_ERROR_CHECK(esp_io_expander_set_level(
            io_expander_, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1 | IO_EXPANDER_PIN_NUM_2,
            1));
    }

    void EnableLcdCs() {
        if (io_expander_ != NULL) {
            esp_io_expander_set_level(io_expander_, IO_EXPANDER_PIN_NUM_3, 0);  // 置低 LCD CS
        }
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = GPIO_NUM_0;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = GPIO_NUM_1;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void ChangeVol(int val) {
        auto codec = GetAudioCodec();
        auto volume = codec->output_volume() + val;
        if (volume > 100) {
            volume = 100;
        }
        if (volume < 0) {
            volume = 0;
        }
        codec->SetOutputVolume(volume);
        GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
    }

    void MuteVol() {
        auto codec = GetAudioCodec();
        auto volume = codec->output_volume();
        if (volume > 1) {
            volume = 0;
        } else {
            volume = 50;
        }
        codec->SetOutputVolume(volume);
        GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
    }

#if CONFIG_ESP32S3_KORVO2_V3_HANDSET_HOOK_GPIO4
    void ApplyDebouncedHandsetHookState() {
        const bool off_hook = gpio_get_level(HANDSET_HOOK_GPIO) == 0;
        if (!handset_hook_armed_.load()) {
            if (off_hook) {
                ESP_LOGW(TAG, "Ignoring boot-time off-hook; return handset to cradle to arm switch");
                return;
            }
            handset_hook_armed_.store(true);
            ESP_LOGI(TAG, "Handset hook armed after stable on-hook state");
        }

        ESP_LOGI(TAG, "Handset hook stable: %s", off_hook ? "off-hook" : "on-hook");
        Application::GetInstance().SetPhoneHookState(off_hook);
    }

    void ScheduleHandsetHookDebounce() {
        if (handset_hook_debounce_timer_ == nullptr) {
            return;
        }
        const esp_err_t stop_result = esp_timer_stop(handset_hook_debounce_timer_);
        if (stop_result != ESP_OK && stop_result != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "Failed to stop handset debounce timer: %s",
                     esp_err_to_name(stop_result));
        }
        const esp_err_t start_result =
            esp_timer_start_once(handset_hook_debounce_timer_, kHandsetHookDebounceUs);
        if (start_result != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start handset debounce timer: %s",
                     esp_err_to_name(start_result));
        }
    }

    void InitializeHandsetHookSwitch() {
        esp_timer_create_args_t timer_args = {};
        timer_args.callback = [](void* arg) {
            static_cast<Esp32S3Korvo2V3Board*>(arg)->ApplyDebouncedHandsetHookState();
        };
        timer_args.arg = this;
        timer_args.dispatch_method = ESP_TIMER_TASK;
        timer_args.name = "handset_hook";
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &handset_hook_debounce_timer_));

        handset_hook_button_.OnPressDown([this]() { ScheduleHandsetHookDebounce(); });
        handset_hook_button_.OnPressUp([this]() { ScheduleHandsetHookDebounce(); });

        const bool initially_off_hook = gpio_get_level(HANDSET_HOOK_GPIO) == 0;
        handset_hook_armed_.store(!initially_off_hook);
        ESP_LOGI(TAG, "GPIO4 handset hook initialized: %s%s",
                 initially_off_hook ? "off-hook" : "on-hook",
                 initially_off_hook ? "; waiting for on-hook before arming" : "");
    }
#endif

    void InitializeButtons() {
        button_adc_config_t adc_cfg = {};
        adc_cfg.adc_channel = ADC_CHANNEL_4;  // ADC1 channel 0 is GPIO5
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        const adc_oneshot_unit_init_cfg_t init_config1 = {
            .unit_id = ADC_UNIT_1,
        };
        adc_oneshot_new_unit(&init_config1, &bsp_adc_handle);
        adc_cfg.adc_handle = &bsp_adc_handle;
#endif
        for (int i = 0; i < BSP_ADC_BUTTON_NUM; ++i) {
            adc_cfg.button_index = i;
            adc_cfg.min = kAdcButtonRanges[i].min_mv;
            adc_cfg.max = kAdcButtonRanges[i].max_mv;
            adc_button_[i] = new AdcButton(adc_cfg);
        }

        auto volume_up_button = adc_button_[BSP_ADC_BUTTON_VOL_UP];
        volume_up_button->OnClick([this]() { ChangeVol(10); });
        volume_up_button->OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(100);
            GetDisplay()->ShowNotification(Lang::Strings::MAX_VOLUME);
        });

        auto volume_down_button = adc_button_[BSP_ADC_BUTTON_VOL_DOWN];
        volume_down_button->OnClick([this]() { ChangeVol(-10); });
        volume_down_button->OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(0);
            GetDisplay()->ShowNotification(Lang::Strings::MUTED);
        });

        auto volume_mute_button = adc_button_[BSP_ADC_BUTTON_VOL_MUTE];
        volume_mute_button->OnClick([this]() { MuteVol(); });

        auto play_button = adc_button_[BSP_ADC_BUTTON_PLAY];
        play_button->OnClick([this]() { ESP_LOGI(TAG, " TODO %s:%d\n", __func__, __LINE__); });

        auto set_button = adc_button_[BSP_ADC_BUTTON_SET];
        set_button->OnClick([this]() {
#if !defined(KORVO2_ML307_BOARD) && !defined(KORVO2_CLM920_RNDIS_BOARD)
            EnterWifiConfigMode();
#endif
        });

        auto rec_button = adc_button_[BSP_ADC_BUTTON_REC];
        rec_button->OnClick([this]() {
            ESP_LOGI(TAG, "REC button click detected; toggling phone chat state");
            Application::GetInstance().TogglePhoneChatState();
        });
#if CONFIG_ESP32S3_KORVO2_V3_HANDSET_HOOK_GPIO4
        InitializeHandsetHookSwitch();
#endif
        boot_button_.OnClick([this]() {});
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
#if !defined(KORVO2_ML307_BOARD) && !defined(KORVO2_CLM920_RNDIS_BOARD)
                EnterWifiConfigMode();
#endif
                return;
            }
            app.ToggleChatState();
        });

#if CONFIG_USE_DEVICE_AEC
        boot_button_.OnDoubleClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateIdle) {
                app.SetAecMode(app.GetAecMode() == kAecOff ? kAecOnDeviceSide : kAecOff);
            }
        });
#endif
    }

    void InitializeIli9341Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GPIO_NUM_NC;
        io_config.dc_gpio_num = GPIO_NUM_2;
        io_config.spi_mode = 0;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");
        const ili9341_vendor_config_t vendor_config = {
            .init_cmds = &vendor_specific_init[0],
            .init_cmds_size = sizeof(vendor_specific_init) / sizeof(ili9341_lcd_init_cmd_t),
        };

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        // panel_config.flags.reset_active_high = 0,
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        panel_config.vendor_config = (void*)&vendor_config;
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));

        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
        EnableLcdCs();
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, false));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
        display_ = new SpiLcdDisplay(panel_io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                     DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X,
                                     DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeSt7789Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GPIO_NUM_46;
        io_config.dc_gpio_num = GPIO_NUM_2;
        io_config.spi_mode = 0;
        io_config.pclk_hz = 60 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片ST7789
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
        EnableLcdCs();
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, true));

        display_ = new SpiLcdDisplay(panel_io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                     DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X,
                                     DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeCamera() {
        camera_config_t camera_config = {
            .pin_pwdn = CAMERA_PIN_PWDN,
            .pin_reset = CAMERA_PIN_RESET,
            .pin_xclk = CAMERA_PIN_XCLK,
            .pin_sccb_sda = -1,  // Use initialized I2C
            .pin_sccb_scl = -1,
            .pin_d7 = CAMERA_PIN_D7,
            .pin_d6 = CAMERA_PIN_D6,
            .pin_d5 = CAMERA_PIN_D5,
            .pin_d4 = CAMERA_PIN_D4,
            .pin_d3 = CAMERA_PIN_D3,
            .pin_d2 = CAMERA_PIN_D2,
            .pin_d1 = CAMERA_PIN_D1,
            .pin_d0 = CAMERA_PIN_D0,
            .pin_vsync = CAMERA_PIN_VSYNC,
            .pin_href = CAMERA_PIN_HREF,
            .pin_pclk = CAMERA_PIN_PCLK,

            .xclk_freq_hz = XCLK_FREQ_HZ,
            .ledc_timer = LEDC_TIMER_0,
            .ledc_channel = LEDC_CHANNEL_0,

            .pixel_format = PIXFORMAT_RGB565,
            .frame_size = FRAMESIZE_QVGA,
            .jpeg_quality = 12,
            .fb_count = 2,
            .fb_location = CAMERA_FB_IN_PSRAM,
            .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
            .sccb_i2c_port = (i2c_port_t)1,
        };

        camera_ = new Esp32Camera(camera_config);
        if (camera_ != nullptr) {
            camera_->SetVFlip(true);
        }
    }

public:
#ifdef KORVO2_ML307_BOARD
    Esp32S3Korvo2V3Board()
        : Korvo2NetworkBoard(ML307_TX_PIN, ML307_RX_PIN), boot_button_(BOOT_BUTTON_GPIO) {
#elif defined(KORVO2_CLM920_RNDIS_BOARD)
    Esp32S3Korvo2V3Board()
        : Korvo2NetworkBoard(CLM920_USB_VENDOR_ID, CLM920_USB_PRODUCT_ID),
          boot_button_(BOOT_BUTTON_GPIO){
#else
    Esp32S3Korvo2V3Board() : boot_button_(BOOT_BUTTON_GPIO) {
#endif
        ESP_LOGI(TAG, "Initializing esp32s3_korvo2_v3 Board");
        InitializeStoryPhoneDefaultVolume();
        InitializePowerSaveTimer();
        InitializeI2c();
        I2cDetect();
        InitializeTca9554();
        InitializeCamera();
        InitializeSpi();
        InitializeButtons();       // 先初始化按钮（创建ADC1句柄）
        InitializePowerManager();  // 后初始化PowerManager（复用ADC1句柄）
#if CONFIG_ESP32S3_KORVO2_V3_ADC_BUTTON_DIAGNOSTICS
        InitializeAdcButtonDiagnostics();
#endif
#ifdef LCD_TYPE_ILI9341_SERIAL
        InitializeIli9341Display();
#else
    InitializeSt7789Display();
#endif
    }

    virtual AudioCodec* GetAudioCodec() override {
        static BoxAudioCodec audio_codec(
            i2c_bus_, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE, AUDIO_I2S_GPIO_MCLK,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR, AUDIO_CODEC_ES7210_ADDR,
            AUDIO_INPUT_REFERENCE, 30.0f, AUDIO_INPUT_REFERENCE_GAIN_CHANNEL, 0.0f);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override { return display_; }
#ifdef KORVO2_ML307_BOARD
    virtual void StartNetwork() override {
        OnNetworkEvent(NetworkEvent::ModemDetecting);
        BaseType_t result = xTaskCreate(Ml307ProbeTask, "ml307_probe", 4096, this, 5, nullptr);
        if (result != pdPASS) {
            ESP_LOGE(TAG, "Failed to start ML307 probe task");
            OnNetworkEvent(NetworkEvent::ModemErrorInitFailed);
        }
    }
#endif
    virtual Camera* GetCamera() override { return camera_; }
    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        static bool last_discharging = false;
        charging = power_manager_->IsCharging();
        discharging = power_manager_->IsDischarging();
        if (discharging != last_discharging) {
            power_save_timer_->SetEnabled(discharging);
            last_discharging = discharging;
        }
        level = power_manager_->GetBatteryLevel();
        return true;
    }

    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (level != PowerSaveLevel::LOW_POWER) {
            power_save_timer_->WakeUp();
        }
        Korvo2NetworkBoard::SetPowerSaveLevel(level);
    }
};

DECLARE_BOARD(Esp32S3Korvo2V3Board);
