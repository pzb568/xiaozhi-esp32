#include "uart_nec_remote.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <stdio.h>

#define TAG "UartNecRemote"

UartNecRemote::UartNecRemote(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin,
                             int baud_rate, uint16_t address_filter, uint8_t cmd_addr)
    : uart_num_(uart_num),
      tx_pin_(tx_pin),
      rx_pin_(rx_pin),
      baud_rate_(baud_rate),
      address_filter_(address_filter),
      cmd_addr_(cmd_addr) {
    uart_config_t uart_config = {};
    uart_config.baud_rate = baud_rate_;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_param_config(uart_num_, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num_, tx_pin_, rx_pin_,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // RX 缓冲区 2KB
    ESP_ERROR_CHECK(uart_driver_install(uart_num_, 2048, 0, 0, nullptr, 0));

    BaseType_t ret = xTaskCreate(TaskEntry, "uart_nec", 4096, this, 5, &task_handle_);
    ESP_ERROR_CHECK(ret == pdPASS ? ESP_OK : ESP_FAIL);
    ESP_LOGI(TAG, "NEC IR transceiver ready: uart=%d tx=%d rx=%d baud=%d",
             uart_num_, tx_pin_, rx_pin_, baud_rate_);
}

UartNecRemote::~UartNecRemote() {
    if (task_handle_ != nullptr) {
        vTaskDelete(task_handle_);
    }
    uart_driver_delete(uart_num_);
}

bool UartNecRemote::SendNec(uint8_t user_code1, uint8_t user_code2, uint8_t command) {
    // YS-IRTM 发射指令：[地址 0xA1][操作 0xF1][用户码高][用户码低][命令码]
    uint8_t packet[5] = {cmd_addr_, 0xF1, user_code1, user_code2, command};
    int written = uart_write_bytes(uart_num_, packet, sizeof(packet));
    if (written != sizeof(packet)) {
        ESP_LOGE(TAG, "SendNec uart write failed (%d)", written);
        return false;
    }
    uart_wait_tx_done(uart_num_, pdMS_TO_TICKS(100));
    last_tx_time_us_ = esp_timer_get_time();
    ESP_LOGI(TAG, "IR send NEC: u1=0x%02X u2=0x%02X cmd=0x%02X",
             user_code1, user_code2, command);
    return true;
}

void UartNecRemote::TaskEntry(void* arg) {
    static_cast<UartNecRemote*>(arg)->IrTask();
}

void UartNecRemote::IrTask() {
    uint8_t frame[kMaxFrameLen];
    size_t frame_len = 0;

    while (true) {
        uint8_t byte = 0;
        // 每次最多等 kFrameGapMs：字节流停止超过该时间即认为一帧结束
        int read = uart_read_bytes(uart_num_, &byte, 1, pdMS_TO_TICKS(kFrameGapMs));
        if (read <= 0) {
            continue;
        }
        frame_len = 0;
        if (frame_len < kMaxFrameLen) {
            frame[frame_len++] = byte;
        }
        while (frame_len < kMaxFrameLen &&
               (read = uart_read_bytes(uart_num_, &byte, 1,
                                       pdMS_TO_TICKS(kFrameGapMs))) > 0) {
            frame[frame_len++] = byte;
        }
        if (frame_len > 0) {
            ProcessFrame(frame, frame_len);
        }
    }
}

bool UartNecRemote::DispatchCommand(uint16_t address, uint8_t command) {
    // 地址过滤（兼容字节序颠倒的地址写法）
    if (address_filter_ != 0) {
        uint16_t swapped = static_cast<uint16_t>(((address & 0xFF) << 8) | (address >> 8));
        if (address != address_filter_ && swapped != address_filter_) {
            ESP_LOGD(TAG, "IR address 0x%04X filtered (expect 0x%04X)",
                     address, address_filter_);
            return false;
        }
    }

    // 长按重复码节流：相同命令在 kRepeatThrottleMs 内只触发一次
    int64_t now_us = esp_timer_get_time();
    if (has_last_command_ && last_command_ == command &&
        (now_us - last_command_time_us_) < kRepeatThrottleMs * 1000LL) {
        return false;
    }
    last_command_ = command;
    last_command_time_us_ = now_us;
    has_last_command_ = true;

    ESP_LOGI(TAG, "IR-NEC: addr=0x%04X cmd=0x%02X", address, command);

    if (command_handler_) {
        command_handler_(address, command);
    }
    return true;
}

void UartNecRemote::ProcessFrame(const uint8_t* data, size_t len) {
    // 发射自抑制：刚发射完的红外可能被模块自己收到（回波），忽略
    int64_t now_us = esp_timer_get_time();
    if (last_tx_time_us_ != 0 &&
        (now_us - last_tx_time_us_) < kTxSuppressMs * 1000LL) {
        return;
    }

    // 格式 A：标准 4 字节 NEC 序列 [addr_l][addr_h][cmd][~cmd]
    // 在帧内搜索（部分模块会加帧头）
    for (size_t i = 0; i + 4 <= len; ++i) {
        const uint8_t addr_l = data[i];
        const uint8_t addr_h = data[i + 1];
        const uint8_t cmd = data[i + 2];
        const uint8_t cmd_inv = data[i + 3];

        if (static_cast<uint8_t>(cmd ^ cmd_inv) != 0xFF) {
            continue;
        }

        uint16_t address = static_cast<uint16_t>(addr_l | (addr_h << 8));
        if (DispatchCommand(address, cmd)) {
            return;
        }
    }

    // 格式 B：YS-IRTM 3 字节输出 [用户码1][用户码2][命令码]
    // 帧长为 3 的倍数时取最后一组（连续按键会输出多组）
    if (len >= 3 && len % 3 == 0) {
        size_t i = len - 3;
        uint16_t address = static_cast<uint16_t>((data[i] << 8) | data[i + 1]);
        if (DispatchCommand(address, data[i + 2])) {
            return;
        }
    }

    // 单字节 0xF1 为模块发射操作反馈，静默忽略
    if (len == 1 && data[0] == 0xF1) {
        return;
    }

    // 无法识别，打印原始帧帮助调试
    char hex[kMaxFrameLen * 3 + 1] = {0};
    for (size_t i = 0; i < len; ++i) {
        snprintf(hex + i * 3, 4, "%02X ", data[i]);
    }
    ESP_LOGI(TAG, "Unparsed IR frame (%u bytes): %s", (unsigned)len, hex);
}
