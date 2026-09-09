#ifndef _UART_NEC_REMOTE_H_
#define _UART_NEC_REMOTE_H_

#include <driver/gpio.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <functional>

/**
 * @brief NEC 红外收发一体模块（UART 接口，YS-IRTM 兼容）驱动
 *
 * 接线：模块 TXD -> ESP rx_pin（解码输出）
 *       模块 RXD <- ESP tx_pin（发射指令入口，可不接）
 *
 * 接收（解码输出，两种格式自动识别）：
 *   A. 标准 4 字节：[地址码] [地址反码] [命令码] [命令反码]
 *      cmd ^ ~cmd == 0xFF 校验通过才生效
 *   B. YS-IRTM 3 字节：[用户码1] [用户码2] [命令码]
 *
 * 发射（SendNec）：
 *   通过 UART 发送 5 字节指令 [cmd_addr][0xF1][用户码高][用户码低][命令码]，
 *   模块自动组成完整 NEC 帧从红外头发射（可控制电视/机顶盒等 99% NEC 设备）。
 *   注意：模块只能发射 NEC 格式，空调等专有协议请使用 IrTransmitter。
 *
 * 发射后短时间内忽略接收数据（避免收到操作反馈/自身回波）。
 */
class UartNecRemote {
public:
    using CommandHandler = std::function<void(uint16_t address, uint8_t command)>;

    /**
     * @param uart_num UART 端口号（如 UART_NUM_1，注意避开 UART0 日志串口）
     * @param tx_pin   ESP 的 TX 引脚，接模块 RXD（发射指令；不接传 GPIO_NUM_NC）
     * @param rx_pin   ESP 的 RX 引脚，接模块 TXD
     * @param baud_rate 模块波特率，默认 9600
     * @param address_filter 仅响应该 16 位地址，0 表示不过滤
     * @param cmd_addr 模块通信地址（YS-IRTM 默认 0xA1，出厂可能不同）
     */
    UartNecRemote(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin,
                  int baud_rate = 9600, uint16_t address_filter = 0,
                  uint8_t cmd_addr = 0xA1);
    ~UartNecRemote();

    /** 注册命令回调：address 为 16 位地址，command 为 8 位命令码 */
    void SetCommandHandler(CommandHandler handler) {
        command_handler_ = std::move(handler);
    }

    /**
     * @brief 发射 NEC 红外码（控制电视等 NEC 设备）
     * @param user_code1 用户码高字节（NEC 帧第 1 字节）
     * @param user_code2 用户码低字节（NEC 帧第 2 字节）
     * @param command    命令码（模块自动补发反码）
     * @return true 已成功写入 UART（模块通常还会回 1 字节 0xF1 确认）
     */
    bool SendNec(uint8_t user_code1, uint8_t user_code2, uint8_t command);

private:
    static void TaskEntry(void* arg);
    void IrTask();
    void ProcessFrame(const uint8_t* data, size_t len);
    bool DispatchCommand(uint16_t address, uint8_t command);

    uart_port_t uart_num_;
    gpio_num_t tx_pin_;
    gpio_num_t rx_pin_;
    int baud_rate_;
    uint16_t address_filter_;
    uint8_t cmd_addr_;
    TaskHandle_t task_handle_ = nullptr;
    CommandHandler command_handler_ = nullptr;

    // 长按重复码节流
    uint8_t last_command_ = 0;
    int64_t last_command_time_us_ = 0;
    bool has_last_command_ = false;

    // 发射自抑制：发射后短时间内的 RX 数据视为回波忽略
    volatile int64_t last_tx_time_us_ = 0;

    static constexpr size_t kMaxFrameLen = 16;
    static constexpr int kFrameGapMs = 15;        // 字节间超时（分帧）
    static constexpr int kRepeatThrottleMs = 350; // 长按重复码节流
    static constexpr int kTxSuppressMs = 200;     // 发射后忽略接收的时长
};

#endif // _UART_NEC_REMOTE_H_
