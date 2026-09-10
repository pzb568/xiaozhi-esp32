/*
 * 红外发射驱动实现（ESP32-C6 RMT 外设）
 *
 * 协议时序规格移植自 IRremoteESP8266 开源库：
 *   - Gree:  ir_Gree.cpp / ir_Kelvinator.cpp（块校验和）
 *   - Midea: ir_Midea.cpp
 *   - Haier: ir_Haier.cpp（HSU07-HEA03）
 *   - NEC:   标准 NEC 32 位
 */
#include "ir_transmitter.h"

#include <esp_log.h>
#include <esp_check.h>
#include <string.h>

#define TAG "ir_transmitter"

// ---------------- Gree（格力）协议常量 ----------------
static const uint32_t kGreeHdrMark  = 9000;
static const uint32_t kGreeHdrSpace = 4500;
static const uint32_t kGreeBitMark  = 620;
static const uint32_t kGreeOneSpace = 1600;
static const uint32_t kGreeZeroSpace = 540;
static const uint32_t kGreeMsgSpace = 19980;   // 段间/帧尾长间隔

// ---------------- Midea（美的）协议常量 ----------------
static const uint32_t kMideaHdrMark  = 4480;
static const uint32_t kMideaHdrSpace = 4480;
static const uint32_t kMideaBitMark  = 560;
static const uint32_t kMideaOneSpace = 1680;
static const uint32_t kMideaZeroSpace = 560;
static const uint32_t kMideaMinGap   = 5600;

// ---------------- Haier（海尔）协议常量 ----------------
static const uint32_t kHaierHdrMark  = 3000;
static const uint32_t kHaierHdrSpace = 3000;
static const uint32_t kHaierHdrMark2 = 3000;
static const uint32_t kHaierHdrSpace2 = 4300;
static const uint32_t kHaierBitMark  = 520;
static const uint32_t kHaierOneSpace = 1650;
static const uint32_t kHaierZeroSpace = 650;

// ---------------- NEC 协议常量 ----------------
static const uint32_t kNecHdrMark  = 9000;
static const uint32_t kNecHdrSpace = 4500;
static const uint32_t kNecBitMark  = 560;
static const uint32_t kNecOneSpace = 1690;
static const uint32_t kNecZeroSpace = 560;

IrTransmitter::IrTransmitter(gpio_num_t tx_pin) : tx_pin_(tx_pin) {
    rmt_tx_channel_config_t tx_cfg = {};
    tx_cfg.gpio_num = tx_pin_;
    tx_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_cfg.resolution_hz = 1000000;   // 1 tick = 1 us
    tx_cfg.mem_block_symbols = 48;    // ESP32-C6 每通道 48 symbols
    esp_err_t err = rmt_new_tx_channel(&tx_cfg, &tx_channel_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_tx_channel failed: %s", esp_err_to_name(err));
        return;
    }

    // 叠加 38kHz 红外载波（高电平期间自动调制）
    rmt_carrier_config_t carrier_cfg = {};
    carrier_cfg.frequency_hz = 38000;
    carrier_cfg.duty_cycle = 0.33;
    carrier_cfg.flags.polarity_active_low = false;
    err = rmt_apply_carrier(tx_channel_, &carrier_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_apply_carrier failed: %s", esp_err_to_name(err));
        return;
    }

    rmt_copy_encoder_config_t enc_cfg = {};
    err = rmt_new_copy_encoder(&enc_cfg, &copy_encoder_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_copy_encoder failed: %s", esp_err_to_name(err));
        return;
    }

    err = rmt_enable(tx_channel_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_enable failed: %s", esp_err_to_name(err));
        return;
    }

    ready_ = true;
    ESP_LOGI(TAG, "IR transmitter ready on GPIO%d (38kHz carrier)", tx_pin_);
}

IrTransmitter::~IrTransmitter() {
    if (tx_channel_ != nullptr) {
        rmt_del_channel(tx_channel_);
    }
    if (copy_encoder_ != nullptr) {
        rmt_del_encoder(copy_encoder_);
    }
}

// 发送一段 symbols（<=44 个，单个 RMT 内存块内），阻塞等待完成
static bool TransmitChunk(rmt_channel_handle_t chan, rmt_encoder_handle_t enc,
                          const rmt_symbol_word_t* symbols, size_t count) {
    rmt_transmit_config_t tx_cfg = {};
    tx_cfg.loop_count = 0;
    tx_cfg.flags.eot_level = 0;
    esp_err_t err = rmt_transmit(chan, enc, symbols,
                                 count * sizeof(rmt_symbol_word_t), &tx_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_transmit failed: %s", esp_err_to_name(err));
        return false;
    }
    err = rmt_tx_wait_all_done(chan, 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_tx_wait_all_done failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

bool IrTransmitter::SendPulseTrain(const uint32_t* pairs, size_t pair_count) {
    if (!ready_) {
        ESP_LOGE(TAG, "IR transmitter not ready");
        return false;
    }

    // ESP32-C6 单通道 RMT 内存 48 symbols，超过需分段。
    // 在长 space（>=4000us，协议固有的段/帧间隙）处切分，额外延迟被长静默掩盖，
    // 接收端无感知。每段保守上限 44 个 symbols。
    rmt_symbol_word_t symbols[44];
    size_t nsym = 0;
    for (size_t i = 0; i < pair_count; i++) {
        uint32_t on = pairs[2 * i];
        uint32_t off = pairs[2 * i + 1];
        if (on > 65535) {
            ESP_LOGE(TAG, "mark too long: %u", (unsigned)on);
            return false;
        }
        if (off > 65535) {
            off = 0;  // 超长静默（如海尔帧尾 150ms）无需实际发送
        }
        symbols[nsym].level0 = 1;
        symbols[nsym].duration0 = (uint16_t)on;
        symbols[nsym].level1 = 0;
        symbols[nsym].duration1 = (uint16_t)off;
        nsym++;
        bool last = (i == pair_count - 1);
        bool at_boundary = (off >= 4000);   // 长 space = 天然分界
        if (nsym >= 44 || last || at_boundary) {
            if (!TransmitChunk(tx_channel_, copy_encoder_, symbols, nsym)) {
                return false;
            }
            nsym = 0;
        }
    }
    return true;
}

// ---- 脉冲序列构造辅助（操作成员 pulse_buf_）----
namespace {

class PulseBuilder {
public:
    uint32_t* buf_;
    size_t count_ = 0;   // 脉冲对数量

    explicit PulseBuilder(uint32_t* buf) : buf_(buf) {}

    void P(uint32_t on, uint32_t off) {
        buf_[count_ * 2] = on;
        buf_[count_ * 2 + 1] = off;
        count_++;
    }
    // 发送一个字节（LSB first）
    void ByteLsb(uint8_t v, uint32_t bit_mark, uint32_t one_space, uint32_t zero_space) {
        for (int i = 0; i < 8; i++) {
            P(bit_mark, (v >> i) & 1 ? one_space : zero_space);
        }
    }
    // 发送一个字节（MSB first）
    void ByteMsb(uint8_t v, uint32_t bit_mark, uint32_t one_space, uint32_t zero_space) {
        for (int i = 7; i >= 0; i--) {
            P(bit_mark, (v >> i) & 1 ? one_space : zero_space);
        }
    }
};

uint8_t ReverseBits8(uint8_t v) {
    v = ((v & 0xAA) >> 1) | ((v & 0x55) << 1);
    v = ((v & 0xCC) >> 2) | ((v & 0x33) << 2);
    return (v >> 4) | (v << 4);
}

} // namespace

// ---------------- 格力（Gree YBOF/YAW1F）----------------
bool IrTransmitter::SendGree(bool power_on, AcMode mode, int temp, int fan) {
    // 模式映射：格力 Auto=0 Cool=1 Dry=2 Fan=3 Heat=4
    uint8_t gree_mode;
    switch (mode) {
        case AC_MODE_AUTO: gree_mode = 0; break;
        case AC_MODE_COOL: gree_mode = 1; break;
        case AC_MODE_DRY:  gree_mode = 2; break;
        case AC_MODE_HEAT: gree_mode = 4; break;
        case AC_MODE_FAN:  gree_mode = 3; break;
        default:           gree_mode = 0; break;
    }

    uint8_t s[8] = {0};
    // B0: Mode(3) | Power(1) | Fan(2) | SwingAuto(1) | Sleep(1)
    s[0] = (gree_mode & 0x07) | ((power_on ? 1 : 0) << 3) | ((fan & 0x03) << 4);
    // B1: Temp(4) = 温度 - 16，其余为定时器位（全 0）
    s[1] = (temp - 16) & 0x0F;
    // B2: Timer 相关全 0；Light(bit5)=1 显示温度；ModelA(bit6)=YAW1F 开机时置 1
    s[2] = (1 << 5) | ((power_on ? 1 : 0) << 6);
    // B3: 高 4 位为固定值 0b0101
    s[3] = 0x50;
    // B4: 摆风（默认保持原位）
    s[4] = 0x00;
    // B5: 固定值 0b100 在 bit3-5
    s[5] = 0x20;
    s[6] = 0x00;
    // B7: 高 4 位为校验和（Kelvinator 块校验，初值 10）
    uint8_t sum = 10;
    for (int i = 0; i < 4; i++) sum += s[i] & 0x0F;
    for (int i = 4; i < 7; i++) sum += s[i] >> 4;
    s[7] = sum & 0x0F;

    PulseBuilder pb(pulse_buf_);
    pb.P(kGreeHdrMark, kGreeHdrSpace);
    for (int i = 0; i < 4; i++) {
        pb.ByteLsb(s[i], kGreeBitMark, kGreeOneSpace, kGreeZeroSpace);
    }
    // 中段页脚 0b010（3 位，LSB first）+ 段尾
    pb.P(kGreeBitMark, kGreeZeroSpace);   // bit0 = 0
    pb.P(kGreeBitMark, kGreeOneSpace);    // bit1 = 1
    pb.P(kGreeBitMark, kGreeZeroSpace);   // bit2 = 0
    pb.P(kGreeBitMark, kGreeMsgSpace);    // 段尾 mark + 19980us 间隙
    for (int i = 4; i < 8; i++) {
        pb.ByteLsb(s[i], kGreeBitMark, kGreeOneSpace, kGreeZeroSpace);
    }
    pb.P(kGreeBitMark, kGreeMsgSpace);     // 帧尾

    return SendPulseTrain(pb.buf_, pb.count_);
}

// ---------------- 美的（Midea）----------------
bool IrTransmitter::SendMidea(bool power_on, AcMode mode, int temp, int fan) {
    // 模式映射：美的 Cool=0 Dry=1 Auto=2 Heat=3 Fan=4
    uint8_t midea_mode;
    switch (mode) {
        case AC_MODE_AUTO: midea_mode = 2; break;
        case AC_MODE_COOL: midea_mode = 0; break;
        case AC_MODE_DRY:  midea_mode = 1; break;
        case AC_MODE_HEAT: midea_mode = 3; break;
        case AC_MODE_FAN:  midea_mode = 4; break;
        default:           midea_mode = 2; break;
    }

    uint8_t s[6] = {0};
    s[1] = 0xFF;   // SensorTemp 未使用
    s[2] = 0xFF;   // OffTimer 未使用
    s[3] = (temp - 17) & 0x1F;   // 温度 17-30°C
    // B4: Mode(3) | Fan(2) | unknown(1) | Sleep(1) | Power(1)
    s[4] = (midea_mode & 0x07) | ((fan & 0x03) << 3) | ((power_on ? 1 : 0) << 7);
    // B5: Type(3)=0 普通 | Header(5)=0b10100
    s[5] = 0b10100 << 3;
    // B0: Sum = 256 - Σ(reverseBits(B1..B5))
    uint8_t sum = 0;
    for (int i = 1; i < 6; i++) sum += ReverseBits8(s[i]);
    s[0] = (uint8_t)(256 - sum);

    // 美的协议：整帧发两遍，第二遍为第一遍的反码
    PulseBuilder pb(pulse_buf_);
    for (int pass = 0; pass < 2; pass++) {
        pb.P(kMideaHdrMark, kMideaHdrSpace);
        for (int i = 0; i < 6; i++) {
            uint8_t v = pass == 0 ? s[i] : (uint8_t)~s[i];
            pb.ByteMsb(v, kMideaBitMark, kMideaOneSpace, kMideaZeroSpace);
        }
        pb.P(kMideaBitMark, kMideaMinGap);
    }

    return SendPulseTrain(pb.buf_, pb.count_);
}

// ---------------- 海尔（Haier HSU07-HEA03）----------------
bool IrTransmitter::SendHaier(bool power_on, AcMode mode, int temp, int fan) {
    // 海尔命令码：Off=0 On=1 Mode=2 Fan=3 TempUp=6 TempDown=7
    uint8_t cmd = power_on ? 0x1 : 0x0;
    // 模式映射：海尔 Auto=0 Cool=1 Dry=2 Heat=3 Fan=4（与统一枚举一致）
    uint8_t haier_mode = (uint8_t)mode;
    // 风速字段映射：0=自动 3=低速 2=中速 1=高速
    uint8_t haier_fan;
    switch (fan) {
        case AC_FAN_AUTO: haier_fan = 0; break;
        case AC_FAN_LOW:  haier_fan = 3; break;
        case AC_FAN_MED:  haier_fan = 2; break;
        case AC_FAN_HIGH: haier_fan = 1; break;
        default:          haier_fan = 0; break;
    }

    uint8_t s[9];
    s[0] = 0xA5;                                    // 固定前缀
    s[1] = (cmd & 0x0F) | ((temp - 16) << 4);       // Command + Temp(16-30)
    s[2] = 0x20;                                    // bit5 固定为 1（时钟未用）
    s[3] = 0x00;                                     // 时钟分钟（未用）
    s[4] = 0x00;                                     // 关机定时（未用）
    s[5] = haier_fan << 6;                           // 风速在高 2 位
    s[6] = haier_mode & 0x07;                        // 模式在高 3 位
    s[7] = 0x00;                                     // 开机定时（未用）
    // B8: 前 8 字节累加和
    uint8_t sum = 0;
    for (int i = 0; i < 8; i++) sum += s[i];
    s[8] = sum;

    PulseBuilder pb(pulse_buf_);
    pb.P(kHaierHdrMark, kHaierHdrSpace);      // 双段引导头
    pb.P(kHaierHdrMark2, kHaierHdrSpace2);
    for (int i = 0; i < 9; i++) {
        pb.ByteMsb(s[i], kHaierBitMark, kHaierOneSpace, kHaierZeroSpace);
    }
    pb.P(kHaierBitMark, 150000);              // 帧尾（超长 off 由驱动截断）

    return SendPulseTrain(pb.buf_, pb.count_);
}

// ---------------- 标准 NEC（32 位）----------------
bool IrTransmitter::SendNec(uint16_t address, uint8_t command) {
    PulseBuilder pb(pulse_buf_);
    pb.P(kNecHdrMark, kNecHdrSpace);
    pb.ByteLsb(address >> 8, kNecBitMark, kNecOneSpace, kNecZeroSpace);
    pb.ByteLsb(address & 0xFF, kNecBitMark, kNecOneSpace, kNecZeroSpace);
    pb.ByteLsb(command, kNecBitMark, kNecOneSpace, kNecZeroSpace);
    pb.ByteLsb((uint8_t)~command, kNecBitMark, kNecOneSpace, kNecZeroSpace);
    pb.P(kNecBitMark, 0);   // 帧结束位

    return SendPulseTrain(pb.buf_, pb.count_);
}

bool IrTransmitter::SendAcCommand(Brand brand, bool power_on, AcMode mode,
                                  int temperature, int fan) {
    // 参数范围保护
    if (temperature < 16) temperature = 16;
    if (temperature > 30) temperature = 30;
    if (fan < 0) fan = 0;
    if (fan > 3) fan = 3;

    switch (brand) {
        case BRAND_GREE:
            return SendGree(power_on, mode, temperature, fan);
        case BRAND_MIDEA:
            return SendMidea(power_on, mode, temperature, fan);
        case BRAND_HAIER:
            return SendHaier(power_on, mode, temperature, fan);
    }
    return false;
}

IrTransmitter::Brand IrTransmitter::BrandFromString(const char* name, bool* ok) {
    if (ok != nullptr) *ok = true;
    if (name == nullptr) {
        if (ok != nullptr) *ok = false;
        return BRAND_GREE;
    }
    if (strcasecmp(name, "gree") == 0 || strcasecmp(name, "格力") == 0) {
        return BRAND_GREE;
    }
    if (strcasecmp(name, "midea") == 0 || strcasecmp(name, "美的") == 0) {
        return BRAND_MIDEA;
    }
    if (strcasecmp(name, "haier") == 0 || strcasecmp(name, "海尔") == 0) {
        return BRAND_HAIER;
    }
    if (ok != nullptr) *ok = false;
    return BRAND_GREE;
}
