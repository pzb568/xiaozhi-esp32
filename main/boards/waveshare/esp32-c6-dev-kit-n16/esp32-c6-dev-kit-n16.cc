/*
 * Waveshare ESP32-C6-DEV-KIT-N16 板级支持
 *
 * 硬件清单：
 *   - 微雪 ESP32-C6-DEV-KIT-N16（ESP32-C6-WROOM-1，16MB Flash，无 PSRAM）
 *   - MAX98357 I2S 功放模块 + 4欧3W 喇叭（音频输出）
 *   - INMP441 I2S 麦克风模块（音频输入）
 *   - NEC 红外收发一体模块（UART，YS-IRTM 兼容：遥控器控制小智 + 发射 NEC 控制电视）
 *   - 分立红外发射管（GPIO10，空调专有协议：格力/美的/海尔）
 *
 * 无屏幕配置：状态通过板载 WS2812 RGB LED 指示。
 * 语音控制：通过 MCP 工具 self.ir_ac_control / self.ir_send_nec 由 AI 自动调用。
 */
#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "uart_nec_remote.h"
#include "ir_transmitter.h"
#include "led/single_led.h"
#include "mcp_server.h"

#include <esp_log.h>

#define TAG "esp32-c6-dev-kit-n16"

class Esp32C6DevKitN16Board : public WifiBoard {
private:
    Button boot_button_;
    UartNecRemote* ir_remote_ = nullptr;
    IrTransmitter* ir_transmitter_ = nullptr;
    Display* display_ = nullptr;

    void InitializeDisplay() {
        display_ = new NoDisplay();
    }

    void InitializeButtons() {
        // BOOT 键：单击开始/停止对话；开机状态（Starting）时单击进入配网
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
        // BOOT 键长按：最大音量
        boot_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(100);
            ESP_LOGI(TAG, "Volume: 100");
        });
    }

    void ChangeVolume(int delta) {
        auto codec = GetAudioCodec();
        int volume = codec->output_volume() + delta;
        if (volume > 100) {
            volume = 100;
        } else if (volume < 0) {
            volume = 0;
        }
        codec->SetOutputVolume(volume);
        ESP_LOGI(TAG, "Volume: %d", volume);
    }

    void InitializeIrRemote() {
        ir_remote_ = new UartNecRemote((uart_port_t)IR_REMOTE_UART_NUM, IR_REMOTE_TX_PIN,
                                       IR_REMOTE_RX_PIN, IR_REMOTE_BAUD_RATE,
                                       IR_NEC_ADDRESS_FILTER, IR_REMOTE_CMD_ADDR);
        ir_remote_->SetCommandHandler([this](uint16_t address, uint8_t command) {
            HandleIrCommand(address, command);
        });

        // 分立红外发射管（空调协议）
        ir_transmitter_ = new IrTransmitter(IR_AC_TX_PIN);
    }

    void HandleIrCommand(uint16_t address, uint8_t command) {
        switch (command) {
            case IR_NEC_CMD_CHAT_TOGGLE:
            case IR_NEC_CMD_CHAT_TOGGLE2: {
                auto& app = Application::GetInstance();
                if (app.GetDeviceState() == kDeviceStateStarting) {
                    EnterWifiConfigMode();
                    return;
                }
                app.ToggleChatState();
                break;
            }
            case IR_NEC_CMD_VOLUME_UP:
            case IR_NEC_CMD_VOLUME_UP2:
                ChangeVolume(10);
                break;
            case IR_NEC_CMD_VOLUME_DOWN:
            case IR_NEC_CMD_VOLUME_DOWN2:
                ChangeVolume(-10);
                break;
            case IR_NEC_CMD_MUTE: {
                auto codec = GetAudioCodec();
                int volume = codec->output_volume();
                if (volume > 0) {
                    codec->SetOutputVolume(0);
                    ESP_LOGI(TAG, "Muted");
                } else {
                    codec->SetOutputVolume(50);
                    ESP_LOGI(TAG, "Unmuted, volume: 50");
                }
                break;
            }
            case IR_NEC_CMD_WIFI_CONFIG:
                EnterWifiConfigMode();
                break;
            default:
                // 未知按键：打印键值，便于用户在 config.h 中补充映射
                ESP_LOGI(TAG, "Unmapped IR key: addr=0x%04X cmd=0x%02X", address, command);
                break;
        }
    }

    // 注册 MCP 工具：对小智说"打开空调/空调调到26度"等，AI 自动调用
    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();

        // 空调控制（格力/美的/海尔，经 GPIO10 分立发射管发送）
        mcp_server.AddTool(
            "self.ir_ac_control",
            "Control the air conditioner via infrared. "
            "power: true=turn on, false=turn off; "
            "mode: 0=auto, 1=cool, 2=dry, 3=heat, 4=fan; "
            "temperature: 16-30 (Celsius); "
            "fan: 0=auto, 1=low, 2=medium, 3=high; "
            "brand: 'gree', 'midea' or 'haier' (default: " IR_AC_DEFAULT_BRAND ")",
            PropertyList({
                Property("power", kPropertyTypeBoolean, true),
                Property("mode", kPropertyTypeInteger, 1, 0, 4),
                Property("temperature", kPropertyTypeInteger, 26, 16, 30),
                Property("fan", kPropertyTypeInteger, 0, 0, 3),
                Property("brand", kPropertyTypeString, IR_AC_DEFAULT_BRAND),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (ir_transmitter_ == nullptr || !ir_transmitter_->IsReady()) {
                    return std::string("IR transmitter not initialized");
                }
                bool power = properties["power"].value<bool>();
                int mode = properties["mode"].value<int>();
                int temperature = properties["temperature"].value<int>();
                int fan = properties["fan"].value<int>();
                std::string brand = properties["brand"].value<std::string>();

                bool ok = false;
                auto b = IrTransmitter::BrandFromString(brand.c_str(), &ok);
                if (!ok) {
                    return std::string("Unsupported brand: " + brand +
                                       " (supported: gree/midea/haier)");
                }
                bool sent = ir_transmitter_->SendAcCommand(
                    b, power, (AcMode)mode, temperature, fan);
                if (!sent) {
                    return std::string("Failed to send IR signal");
                }
                return true;
            });

        // NEC 发射（电视/机顶盒/风扇等，经 YS-IRTM 模块发送）
        mcp_server.AddTool(
            "self.ir_send_nec",
            "Send a NEC infrared code to control TV/set-top-box/fan etc. "
            "address: 16-bit NEC user code (low byte first), e.g. 0x00FF = 255; "
            "command: 8-bit command code (0-255). "
            "Lookup the device's NEC code table for the specific command.",
            PropertyList({
                Property("address", kPropertyTypeInteger, 0x00FF, 0, 65535),
                Property("command", kPropertyTypeInteger, 0, 0, 255),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (ir_remote_ == nullptr) {
                    return std::string("IR remote module not initialized");
                }
                int address = properties["address"].value<int>();
                int command = properties["command"].value<int>();
                // 低字节在前（NEC 发送顺序）
                uint8_t u1 = address & 0xFF;
                uint8_t u2 = (address >> 8) & 0xFF;
                if (ir_remote_->SendNec(u1, u2, (uint8_t)command)) {
                    return true;
                }
                return std::string("Failed to send NEC code");
            });
    }

public:
    Esp32C6DevKitN16Board() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeDisplay();
        InitializeButtons();
        InitializeIrRemote();
        InitializeTools();
        ESP_LOGI(TAG, "Waveshare ESP32-C6-DEV-KIT-N16 board initialized");
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_DUPLEX
        // 全双工：INMP441 与 MAX98357 共用 BCLK/WS
        static NoAudioCodecDuplex audio_codec(
            AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS,
            AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#else
        // 简单模式：功放与麦克风各自独立时钟
        static NoAudioCodecSimplex audio_codec(
            AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT,
            AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }
};

DECLARE_BOARD(Esp32C6DevKitN16Board);
