#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

/*
 * 微雪 ESP32-C6-DEV-KIT-N16 (ESP32-C6-WROOM-1-N16, 16MB Flash)
 *
 * 外接配件：
 *   - MAX98357 I2S 功放模块 + 4欧3W 全频喇叭（音频输出）
 *   - INMP441 I2S 麦克风模块（音频输入）
 *   - NEC 红外解码模块（UART 输出，音频遥控）
 *
 * 默认使用 Duplex（全双工共用 BCLK/WS）接线方式，INMP441 与 MAX98357
 * 共用一组时钟线，只需 4 根信号线。如需 Simplex（各自独立时钟），
 * 注释掉下面的 AUDIO_I2S_METHOD_SIMPLEX_DUPLEX 宏即可（见 README）。
 */

#define AUDIO_INPUT_SAMPLE_RATE  24000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

// 选择接线模式：默认 Duplex（共用时钟，接线最简）
// 如需 Simplex（功放/麦克风各用一组 BCLK/WS），注释掉下面这行
#define AUDIO_I2S_METHOD_DUPLEX

#ifdef AUDIO_I2S_METHOD_DUPLEX

// -------- Duplex：共用 BCLK/WS（推荐，接线最少）--------
// GPIO4  -> MAX98357 BCLK + INMP441 SCK（两脚并接）
// GPIO5  -> MAX98357 LRC  + INMP441 WS （两脚并接）
// GPIO6  -> MAX98357 DIN
// GPIO7  -> INMP441 SD
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_4
#define AUDIO_I2S_GPIO_WS   GPIO_NUM_5
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_6   // ESP 输出 -> MAX98357 DIN
#define AUDIO_I2S_GPIO_DIN  GPIO_NUM_7   // INMP441 SD -> ESP 输入

#else

// -------- Simplex：功放与麦克风分开两组时钟（备用方案）--------
// 功放：GPIO4=BCLK, GPIO5=LRC, GPIO6=DIN
// 麦克风：GPIO2=SCK, GPIO3=WS, GPIO7=SD
#define AUDIO_I2S_MIC_GPIO_WS    GPIO_NUM_3
#define AUDIO_I2S_MIC_GPIO_SCK   GPIO_NUM_2
#define AUDIO_I2S_MIC_GPIO_DIN   GPIO_NUM_7
#define AUDIO_I2S_SPK_GPIO_DOUT  GPIO_NUM_6
#define AUDIO_I2S_SPK_GPIO_BCLK  GPIO_NUM_4
#define AUDIO_I2S_SPK_GPIO_LRCK  GPIO_NUM_5

#endif

// 板载 WS2812 RGB LED（兼容 ESP32-C6-DevKitC-1 引脚，如无 LED 可改为 GPIO_NUM_NC）
#define BUILTIN_LED_GPIO GPIO_NUM_8

// BOOT 按键（按住进下载模式，运行时单击=开始/停止对话）
#define BOOT_BUTTON_GPIO GPIO_NUM_9

// ---------------- NEC 红外收发一体模块（UART，YS-IRTM 兼容）----------------
// 模块 TXD -> GPIO21（ESP 接收解码数据）；模块 RXD -> GPIO20（ESP 发射指令）
// 注意：该类模块多为 5V TTL 电平，模块 TXD 建议经分压电阻（1k+2k）再接 GPIO21
#define IR_REMOTE_UART_NUM      1
#define IR_REMOTE_TX_PIN        GPIO_NUM_20   // ESP TX -> 模块 RXD（发射 NEC 用）
#define IR_REMOTE_RX_PIN        GPIO_NUM_21   // 模块 TXD -> ESP RX
#define IR_REMOTE_BAUD_RATE     9600
#define IR_REMOTE_CMD_ADDR      0xA1          // 模块通信地址（YS-IRTM 默认，可被改为 FA）

// ---------------- 空调红外发射管（分立元件，RMT 直驱）----------------
// GPIO10 -> 100Ω 电阻 -> 红外发射管(940nm) -> GND
// 空调为专有协议（非 NEC），必须用独立发射管发送，模块发不了空调码
#define IR_AC_TX_PIN            GPIO_NUM_10
// 默认空调品牌：gree / midea / haier（语音控制未指定品牌时使用）
#define IR_AC_DEFAULT_BRAND     "gree"

// 地址过滤：0 表示接受任意遥控器地址；填入 16 位 NEC 地址（如 0x00FF）
// 可只响应指定遥控器
#define IR_NEC_ADDRESS_FILTER   0x0000

// NEC 命令码 -> 功能映射（默认按常见 0x00FF 遥控器布局，可自行修改）
// 首次使用请看串口日志中的 "IR-NEC: addr=0x.... cmd=0x.." 输出，
// 按下遥控器按键后把对应 cmd 值填到下面即可。
#define IR_NEC_CMD_CHAT_TOGGLE   0x0C   // OK 键：唤醒/开始对话，再按结束
#define IR_NEC_CMD_CHAT_TOGGLE2  0x45   // 数字 1：同上（备用键）
#define IR_NEC_CMD_VOLUME_UP     0x18   // 上箭头：音量 +
#define IR_NEC_CMD_VOLUME_UP2    0x1B   // 右箭头：音量 +（备用键）
#define IR_NEC_CMD_VOLUME_DOWN   0x52   // 下箭头：音量 -
#define IR_NEC_CMD_VOLUME_DOWN2  0x5A   // 左箭头：音量 -（备用键）
#define IR_NEC_CMD_MUTE          0x19   // * 键：静音/取消静音
#define IR_NEC_CMD_WIFI_CONFIG   0x0D   // # 键：重新进入 WiFi 配网模式

#endif // _BOARD_CONFIG_H_
