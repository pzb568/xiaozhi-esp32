#ifndef IR_TRANSMITTER_H_
#define IR_TRANSMITTER_H_

#include <stdint.h>
#include <stddef.h>
#include <driver/gpio.h>
#include <driver/rmt_tx.h>
#include <driver/rmt_encoder.h>

/*
 * 红外发射驱动（RMT 外设，38kHz 载波自动调制）
 *
 * 硬件：GPIO -> 100Ω 限流电阻 -> 红外发射管(940nm) -> GND
 *       （如需远距离，可加 S8050 三极管驱动）
 *
 * 支持协议：
 *   - Gree  格力空调（YBOF/YAW1F 遥控器，64 位双段帧）
 *   - Midea 美的空调（48 位，正码 + 反码双发）
 *   - Haier 海尔空调（HSU07-HEA03，72 位，9 字节状态帧）
 *   - NEC   通用红外（电视/机顶盒/风扇等，32 位）
 *
 * 时序规格来源：IRremoteESP8266 开源库（crankyoldgit/IRremoteESP8266）
 */

// 空调工作模式（统一抽象，内部映射为各品牌编码）
enum AcMode {
    AC_MODE_AUTO = 0,   // 自动
    AC_MODE_COOL = 1,   // 制冷
    AC_MODE_DRY  = 2,   // 除湿
    AC_MODE_HEAT = 3,   // 制热
    AC_MODE_FAN  = 4,   // 送风
};

// 风速（0=自动 1=低 2=中 3=高）
enum AcFanSpeed {
    AC_FAN_AUTO = 0,
    AC_FAN_LOW  = 1,
    AC_FAN_MED  = 2,
    AC_FAN_HIGH = 3,
};

class IrTransmitter {
public:
    // 空调品牌
    enum Brand {
        BRAND_GREE = 0,    // 格力
        BRAND_MIDEA = 1,   // 美的
        BRAND_HAIER = 2,   // 海尔
    };

    explicit IrTransmitter(gpio_num_t tx_pin);
    ~IrTransmitter();

    // 是否初始化成功（RMT 通道创建成功）
    bool IsReady() const { return ready_; }

    // ---- 高层接口：空调控制 ----
    // power_on: 开/关机；mode: AcMode；temperature: 摄氏度(16-30)；fan: AcFanSpeed
    // 返回 true 表示已成功发出红外信号
    bool SendAcCommand(Brand brand, bool power_on, AcMode mode,
                       int temperature, int fan);

    // ---- 通用 NEC 发射（电视/机顶盒等，直连发射管）----
    // address/address_high: 16 位用户码；command: 命令码
    bool SendNec(uint16_t address, uint8_t command);

    // ---- 底层接口：直接发送脉冲对序列 ----
    // pairs 格式: [on1_us, off1_us, on2_us, off2_us, ...]
    // 38kHz 载波自动叠加在 on 段上
    bool SendPulseTrain(const uint32_t* pairs, size_t pair_count);

    // 品牌名字符串转枚举（"gree"/"midea"/"haier"），失败返回 BRAND_GREE 并置 *ok=false
    static Brand BrandFromString(const char* name, bool* ok);

private:
    // 逐协议构造并发送
    bool SendGree(bool power_on, AcMode mode, int temp, int fan);
    bool SendMidea(bool power_on, AcMode mode, int temp, int fan);
    bool SendHaier(bool power_on, AcMode mode, int temp, int fan);

    gpio_num_t tx_pin_;
    rmt_channel_handle_t tx_channel_ = nullptr;
    rmt_encoder_handle_t copy_encoder_ = nullptr;
    bool ready_ = false;

    // 脉冲缓冲（最长需求：美的 48bit x 2 帧 + 头尾 ≈ 106 对）
    static constexpr size_t kMaxPairs = 160;
    uint32_t pulse_buf_[kMaxPairs * 2];
};

#endif /* IR_TRANSMITTER_H_ */
