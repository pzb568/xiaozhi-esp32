#include "no_audio_codec.h"

#include <esp_log.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "NoAudioCodec"

// Integer soft-knee gain: linear up to kKnee, then smoothly compressed towards +-32767.
// y = k + r*d/(d+r) for d = |x|-k (slope 1 at the knee, never overshoots). No floating point:
// ESP32-C6 has no FPU, so double tanh() at 24 kHz would eat a large share of the CPU.
static inline int16_t SoftGain(int16_t in, int32_t gain_q8) {
    constexpr int32_t kKnee = 20000;
    constexpr int32_t kMax = 32767;
    int32_t x = ((int32_t)in * gain_q8) >> 8;
    bool neg = x < 0;
    int32_t a = neg ? -x : x;
    if (a > kKnee) {
        int32_t d = a - kKnee;
        int32_t r = kMax - kKnee;
        a = kKnee + (int32_t)(((int64_t)r * d) / (d + r));
    }
    return (int16_t)(neg ? -a : a);
}

NoAudioCodec::~NoAudioCodec() {
    if (rx_handle_ != nullptr) {
        ESP_ERROR_CHECK(i2s_channel_disable(rx_handle_));
    }
    if (tx_handle_ != nullptr) {
        ESP_ERROR_CHECK(i2s_channel_disable(tx_handle_));
    }
}

NoAudioCodecDuplex::NoAudioCodecDuplex(int input_sample_rate, int output_sample_rate, gpio_num_t bclk, gpio_num_t ws, gpio_num_t dout, gpio_num_t din) {
    duplex_ = true;
    input_sample_rate_ = input_sample_rate;
    output_sample_rate_ = output_sample_rate;

    // ---- 引脚自检：I2S 初始化前，用内部上拉/下拉回读 4 根信号线 ----
    // 上拉读 1 + 下拉读 0 = 引脚悬空可驱动（正常）；
    // 两读皆 0 = 被外部死钳到 GND（对地短路/焊连）；两读皆 1 = 被钳到 VCC。
    {
        const gpio_num_t pins[4] = {bclk, ws, dout, din};
        const char* names[4] = {"BCLK", "WS  ", "DOUT", "DIN "};
        ESP_LOGW(TAG, "[PinTest] 引脚自检开始（I2S 初始化前）...");
        for (int i = 0; i < 4; i++) {
            gpio_config_t cfg = {};
            cfg.pin_bit_mask = 1ULL << pins[i];
            cfg.mode = GPIO_MODE_INPUT;
            cfg.pull_up_en = GPIO_PULLUP_ENABLE;
            cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
            gpio_config(&cfg);
            vTaskDelay(pdMS_TO_TICKS(2));
            int up = gpio_get_level(pins[i]);
            cfg.pull_up_en = GPIO_PULLUP_DISABLE;
            cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
            gpio_config(&cfg);
            vTaskDelay(pdMS_TO_TICKS(2));
            int down = gpio_get_level(pins[i]);
            const char* st = (up == 1 && down == 0) ? "OK（悬空可驱动）"
                           : (up == 0 && down == 0) ? "❌ 疑似对地短路！"
                           : (up == 1 && down == 1) ? "❌ 疑似对电源短路！"
                                                    : "异常（浮空漂移）";
            ESP_LOGW(TAG, "[PinTest] %s GPIO%d: 上拉读=%d 下拉读=%d -> %s",
                     names[i], (int)pins[i], up, down, st);
        }
        for (int i = 0; i < 4; i++) {
            gpio_reset_pin(pins[i]);  // drop the test pulls
        }
        ESP_LOGW(TAG, "[PinTest] 引脚自检完成。注：DIN 在麦克风无时钟时三态，悬空属正常");
    }
    // 使用标准 I2S 立体声帧结构（每帧 64 SCK）：
    // INMP441 内部 Σ-Δ 调制器设计工作点为过采样率 64（SCK = 采样率 × 64）。
    // 若用 mono 帧结构（每帧 32 SCK），SCK 减半 → 过采样率仅 32 →
    // 内部抽取滤波器失配，大量高频量化噪声泄漏（实测 zcr 0.34、dc 漂移），
    // 16kHz 时（过采样率 16）甚至完全失效输出静音。
    stereo_slots_ = true;

    i2s_chan_config_t chan_cfg = {
        .id = XIAOZHI_I2S_PORT(0),
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = AUDIO_CODEC_DMA_DESC_NUM,
        .dma_frame_num = AUDIO_CODEC_DMA_FRAME_NUM,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = false,
        .intr_priority = 0,
    };
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle_, &rx_handle_));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = (uint32_t)output_sample_rate_,
            .clk_src = I2S_CLK_SRC_DEFAULT,
#if SOC_I2S_HW_VERSION_2
            .ext_clk_freq_hz = 0,
#endif
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,

        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            // 立体声帧结构：WS = SCK/64，INMP441 内部过采样率恢复为设计值 64
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = I2S_STD_SLOT_BOTH,
            .ws_width = I2S_DATA_BIT_WIDTH_32BIT,
            .ws_pol = false,
            .bit_shift = true,
            #if SOC_I2S_HW_VERSION_2
                .left_align = true,
                .big_endian = false,
                .bit_order_lsb = false
            #endif

        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = bclk,
            .ws = ws,
            .dout = dout,
            .din = din,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_, &std_cfg));
    ESP_LOGI(TAG, "Duplex channels created (stereo frame, SCK=rate*64)");
}


NoAudioCodecSimplex::NoAudioCodecSimplex(int input_sample_rate, int output_sample_rate, gpio_num_t spk_bclk, gpio_num_t spk_ws, gpio_num_t spk_dout, gpio_num_t mic_sck, gpio_num_t mic_ws, gpio_num_t mic_din) {
    duplex_ = false;
    input_sample_rate_ = input_sample_rate;
    output_sample_rate_ = output_sample_rate;

    // Create a new channel for speaker
    i2s_chan_config_t chan_cfg = {
        .id = XIAOZHI_I2S_PORT(0),
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = AUDIO_CODEC_DMA_DESC_NUM,
        .dma_frame_num = AUDIO_CODEC_DMA_FRAME_NUM,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = false,
        .intr_priority = 0,
    };
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle_, nullptr));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = (uint32_t)output_sample_rate_,
            .clk_src = I2S_CLK_SRC_DEFAULT,
#if SOC_I2S_HW_VERSION_2
            .ext_clk_freq_hz = 0,
#endif
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,

        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = I2S_STD_SLOT_LEFT,
            .ws_width = I2S_DATA_BIT_WIDTH_32BIT,
            .ws_pol = false,
            .bit_shift = true,
            #if SOC_I2S_HW_VERSION_2
                .left_align = true,
                .big_endian = false,
                .bit_order_lsb = false
            #endif

        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = spk_bclk,
            .ws = spk_ws,
            .dout = spk_dout,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_, &std_cfg));

    // Create a new channel for MIC
    chan_cfg.id = XIAOZHI_I2S_PORT(1);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, nullptr, &rx_handle_));
    std_cfg.clk_cfg.sample_rate_hz = (uint32_t)input_sample_rate_;
    std_cfg.gpio_cfg.bclk = mic_sck;
    std_cfg.gpio_cfg.ws = mic_ws;
    std_cfg.gpio_cfg.dout = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.din = mic_din;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_, &std_cfg));
    ESP_LOGI(TAG, "Simplex channels created");
}

NoAudioCodecSimplex::NoAudioCodecSimplex(int input_sample_rate, int output_sample_rate, gpio_num_t spk_bclk, gpio_num_t spk_ws, gpio_num_t spk_dout, i2s_std_slot_mask_t spk_slot_mask, gpio_num_t mic_sck, gpio_num_t mic_ws, gpio_num_t mic_din, i2s_std_slot_mask_t mic_slot_mask){
    duplex_ = false;
    input_sample_rate_ = input_sample_rate;
    output_sample_rate_ = output_sample_rate;

    // Create a new channel for speaker
    i2s_chan_config_t chan_cfg = {
        .id = XIAOZHI_I2S_PORT(0),
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = AUDIO_CODEC_DMA_DESC_NUM,
        .dma_frame_num = AUDIO_CODEC_DMA_FRAME_NUM,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = false,
        .intr_priority = 0,
    };
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle_, nullptr));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = (uint32_t)output_sample_rate_,
            .clk_src = I2S_CLK_SRC_DEFAULT,
#if SOC_I2S_HW_VERSION_2
            .ext_clk_freq_hz = 0,
#endif
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,

        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = spk_slot_mask,
            .ws_width = I2S_DATA_BIT_WIDTH_32BIT,
            .ws_pol = false,
            .bit_shift = true,
            #if SOC_I2S_HW_VERSION_2
                .left_align = true,
                .big_endian = false,
                .bit_order_lsb = false
            #endif

        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = spk_bclk,
            .ws = spk_ws,
            .dout = spk_dout,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_, &std_cfg));

    // Create a new channel for MIC
    chan_cfg.id = XIAOZHI_I2S_PORT(1);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, nullptr, &rx_handle_));
    std_cfg.clk_cfg.sample_rate_hz = (uint32_t)input_sample_rate_;
    std_cfg.slot_cfg.slot_mask = mic_slot_mask;
    std_cfg.gpio_cfg.bclk = mic_sck;
    std_cfg.gpio_cfg.ws = mic_ws;
    std_cfg.gpio_cfg.dout = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.din = mic_din;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_, &std_cfg));
    ESP_LOGI(TAG, "Simplex channels created");
}

int NoAudioCodec::Write(const int16_t* data, int samples) {
    std::lock_guard<std::mutex> lock(data_if_mutex_);

    // output_volume_: 0-100
    // volume_factor_: 0-65536
    int32_t volume_factor = pow(double(output_volume_) / 100.0, 2) * 65536;

    size_t bytes_written;
    if (stereo_slots_) {
        // 立体声帧结构：音频数据放在 stereo_slot_index_ 对应的 slot
        // （与 RX 自动判定结果一致，保证 MAX98357 在其锁存 slot 收到数据），
        // 另一 slot 保持 0 静默。
        int idx = stereo_slot_index_;
        std::vector<int32_t> buffer(samples * 2, 0);
        for (int i = 0; i < samples; i++) {
            int64_t temp = int64_t(data[i]) * volume_factor; // 使用 int64_t 进行乘法运算
            int32_t value = (temp > INT32_MAX) ? INT32_MAX : (temp < INT32_MIN) ? INT32_MIN : static_cast<int32_t>(temp);
            buffer[i * 2 + idx] = value;
        }
        ESP_ERROR_CHECK(i2s_channel_write(tx_handle_, buffer.data(), samples * 2 * sizeof(int32_t), &bytes_written, portMAX_DELAY));
        return samples;
    }

    std::vector<int32_t> buffer(samples);
    for (int i = 0; i < samples; i++) {
        int64_t temp = int64_t(data[i]) * volume_factor; // 使用 int64_t 进行乘法运算
        if (temp > INT32_MAX) {
            buffer[i] = INT32_MAX;
        } else if (temp < INT32_MIN) {
            buffer[i] = INT32_MIN;
        } else {
            buffer[i] = static_cast<int32_t>(temp);
        }
    }

    ESP_ERROR_CHECK(i2s_channel_write(tx_handle_, buffer.data(), samples * sizeof(int32_t), &bytes_written, portMAX_DELAY));
    return bytes_written / sizeof(int32_t);
}

int NoAudioCodec::Read(int16_t* dest, int samples) {
    size_t bytes_read;
    constexpr uint32_t kReadTimeoutMs = 200;

    if (stereo_slots_) {
        // 立体声帧结构：每帧含两个 32-bit slot，DMA 按交错顺序存入缓冲。
        // INMP441（L/R 引脚接 GND）只在其锁存 slot 输出有效数据，
        // 另一 slot 为三态浮空残留（跟随上一 slot 末尾符号扩展位电平）。
        std::vector<int32_t> bit32_buffer(samples * 2);
        if (i2s_channel_read(rx_handle_, bit32_buffer.data(), samples * 2 * sizeof(int32_t), &bytes_read, kReadTimeoutMs) != ESP_OK) {
            return 0;
        }
        int frames = bytes_read / (2 * sizeof(int32_t));

        // ---- slot 选择 ----
        // 实测结论（2026-10）：INMP441 数据在**偶数索引 slot**（slot0）。
        // 依据：自动判定版在锁定前的窗口里成功唤醒"你好小智"，
        // 而那时的取值正是偶数索引；锁定后取到全零 slot1 即静音。
        // 此前 stereo 版"整流指纹"（pk+ 数倍于 pk-）经查是增益 ×8 软压缩
        // 放大残留所致，而非 slot 取反。
        //
        // 保留一个仅用于"诊断输出"的旁路统计：不影响主数据通路，
        // 仅打印两路对比，便于后续排查；只有 slot0 近乎无声时才切换到 slot1。
        if (!slot_locked_) {
            static int64_t s_sum[2] = {0, 0};
            static int64_t s_sum2[2] = {0, 0};
            static int32_t s_pkp[2] = {INT32_MIN, INT32_MIN};
            static int32_t s_pkn[2] = {0, 0};
            static uint32_t s_cnt[2] = {0, 0};
            static uint32_t s_sat[2] = {0, 0};   // 饱和样本数 |v|>=32000
            static uint32_t s_cross[2] = {0, 0}; // 过零次数（符号翻转）
            static int32_t s_prev[2] = {0, 0};
            for (int i = 0; i < frames; i++) {
                for (int k = 0; k < 2; k++) {
                    int32_t s = bit32_buffer[i * 2 + k] >> 12;
                    s_sum2[k] += (int64_t)s * s;
                    s_sum[k] += s;
                    if (s > s_pkp[k]) s_pkp[k] = s;
                    if (s < s_pkn[k]) s_pkn[k] = s;
                    if (s >= 32000 || s <= -32000) s_sat[k]++;
                    if ((s >= 0) != (s_prev[k] >= 0)) s_cross[k]++;
                    s_prev[k] = s;
                    s_cnt[k]++;
                }
            }
            constexpr uint32_t kAutoDetectFrames = 72000;  // 约 3 秒 @ 24kHz
            if (s_cnt[0] >= kAutoDetectFrames) {
                double rms_v[2], dc_v[2], asym_v[2], zcr_v[2];
                double sat_pct[2];
                for (int k = 0; k < 2; k++) {
                    rms_v[k] = std::sqrt((double)s_sum2[k] / s_cnt[k]);
                    dc_v[k] = (double)s_sum[k] / s_cnt[k];
                    double pkp_d = (double)s_pkp[k], pkn_d = (double)s_pkn[k];
                    asym_v[k] = std::fabs(pkp_d + pkn_d) / (std::fabs(pkp_d) + std::fabs(pkn_d) + 1.0);
                    zcr_v[k] = (double)s_cross[k] / s_cnt[k];
                    sat_pct[k] = 100.0 * s_sat[k] / s_cnt[k];
                }
                ESP_LOGW(TAG, "[SlotSelect] slot0(偶): rms=%.0f dc=%.1f pk+=%d pk-=%d asym=%.2f zcr=%.3f",
                         rms_v[0], dc_v[0], s_pkp[0], s_pkn[0], asym_v[0], zcr_v[0]);
                ESP_LOGW(TAG, "[SlotSelect] slot1(奇): rms=%.0f dc=%.1f pk+=%d pk-=%d asym=%.2f zcr=%.3f",
                         rms_v[1], dc_v[1], s_pkp[1], s_pkn[1], asym_v[1], zcr_v[1]);
                // 默认 slot0（已实证）；仅当 slot0 近乎无声且 slot1 有信号时才切换。
                if (rms_v[0] < 10 && rms_v[1] > 50) {
                    stereo_slot_index_ = 1;
                    ESP_LOGW(TAG, "[SlotSelect] slot0 无信号，回退到 slot1");
                } else {
                    stereo_slot_index_ = 0;
                    ESP_LOGW(TAG, "[SlotSelect] 锁定 slot0（偶数索引，已实证为 INMP441 数据）");
                }
                slot_locked_ = true;
                // ---- 麦克风数据级自检（启动 3 秒时给出一次性健康判定）----
                const char* verdict;
                if (rms_v[0] < 2) {
                    verdict = "❌ 无信号：SD 数据线断路 / 麦克风未供电（查 VDD 3.3V 与 SD 接线）";
                } else if (sat_pct[0] > 30.0) {
                    verdict = "❌ 信号饱和：数据线疑似短路或时钟异常（查接线）";
                } else if (rms_v[0] < 20) {
                    verdict = "⚠️ 信号偏弱：能收到数据但幅度极低（查供电稳定性/接地）";
                } else {
                    verdict = "✅ 麦克风工作正常：数据、幅度、动态范围均正常";
                }
                ESP_LOGW(TAG, "[MicSelfTest] 麦克风自检: %s", verdict);
                ESP_LOGW(TAG, "[MicSelfTest] slot0: rms=%.0f 饱和=%.2f%% zcr=%.3f （自检仅判定硬件通断，信噪比随供电波动需看 MicDebug）",
                         rms_v[0], sat_pct[0], zcr_v[0]);
            }
        }

        int idx = stereo_slot_index_;
        for (int i = 0; i < frames; i++) {
            int32_t value = bit32_buffer[i * 2 + idx] >> 12;
            dest[i] = (value > INT16_MAX) ? INT16_MAX : (value < -INT16_MAX) ? -INT16_MAX : (int16_t)value;
        }
        // Soft-knee gain (integer math).
        if (input_gain_ > 0) {
            const int32_t gain_q8 = (int32_t)(input_gain_ * 256.0f);
            for (int i = 0; i < frames; i++) {
                dest[i] = SoftGain(dest[i], gain_q8);
            }
        }
        return frames;
    }

    std::vector<int32_t> bit32_buffer(samples);
    if (i2s_channel_read(rx_handle_, bit32_buffer.data(), samples * sizeof(int32_t), &bytes_read, kReadTimeoutMs) != ESP_OK) {
        return 0;
    }

    samples = bytes_read / sizeof(int32_t);
    for (int i = 0; i < samples; i++) {
        int32_t value = bit32_buffer[i] >> 12;
        dest[i] = (value > INT16_MAX) ? INT16_MAX : (value < -INT16_MAX) ? -INT16_MAX : (int16_t)value;
    }
    // 应用输入增益：采用软压缩（soft-knee）而非硬截断，
    // 低电平线性放大、高电平平滑压缩，避免削波产生的谐波失真。
    // 整数 soft-knee（见 SoftGain）：小信号线性放大 gain 倍，大信号渐近到 ±32767 永不过冲。
    if (input_gain_ > 0) {
        const int32_t gain_q8 = (int32_t)(input_gain_ * 256.0f);
        for (int i = 0; i < samples; i++) {
            dest[i] = SoftGain(dest[i], gain_q8);
        }
    }
    return samples;
}

void NoAudioCodec::EnableInput(bool enable) {
    std::lock_guard<std::mutex> lock(data_if_mutex_);
    if (enable == input_enabled_) {
        return;
    }
    if (enable) {
        ESP_ERROR_CHECK(i2s_channel_enable(rx_handle_));
    } else {
        ESP_ERROR_CHECK(i2s_channel_disable(rx_handle_));
    }
    AudioCodec::EnableInput(enable);
}

void NoAudioCodec::EnableOutput(bool enable) {
    std::lock_guard<std::mutex> lock(data_if_mutex_);
    if (enable == output_enabled_) {
        return;
    }
    if (enable) {
        ESP_ERROR_CHECK(i2s_channel_enable(tx_handle_));
    } else {
        ESP_ERROR_CHECK(i2s_channel_disable(tx_handle_));
    }
    AudioCodec::EnableOutput(enable);
}

// Delegating constructor: calls the main constructor with default slot mask
NoAudioCodecSimplexPdm::NoAudioCodecSimplexPdm(int input_sample_rate, int output_sample_rate, gpio_num_t spk_bclk, gpio_num_t spk_ws, gpio_num_t spk_dout, gpio_num_t mic_sck, gpio_num_t mic_din) 
    : NoAudioCodecSimplexPdm(input_sample_rate, output_sample_rate, spk_bclk, spk_ws, spk_dout, I2S_STD_SLOT_LEFT, mic_sck, mic_din) {
    // All initialization is handled by the delegated constructor
}

NoAudioCodecSimplexPdm::NoAudioCodecSimplexPdm(int input_sample_rate, int output_sample_rate, gpio_num_t spk_bclk, gpio_num_t spk_ws, gpio_num_t spk_dout, i2s_std_slot_mask_t spk_slot_mask, gpio_num_t mic_sck, gpio_num_t mic_din) {
    duplex_ = false;
    input_sample_rate_ = input_sample_rate;
    output_sample_rate_ = output_sample_rate;

    // Create a new channel for speaker
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(XIAOZHI_I2S_PORT(1), I2S_ROLE_MASTER);
    tx_chan_cfg.dma_desc_num = AUDIO_CODEC_DMA_DESC_NUM;
    tx_chan_cfg.dma_frame_num = AUDIO_CODEC_DMA_FRAME_NUM;
    tx_chan_cfg.auto_clear_after_cb = true;
    tx_chan_cfg.auto_clear_before_cb = false;
    tx_chan_cfg.intr_priority = 0;
    ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_handle_, NULL));


    i2s_std_config_t tx_std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = (uint32_t)output_sample_rate_,
            .clk_src = I2S_CLK_SRC_DEFAULT,
#if SOC_I2S_HW_VERSION_2
            .ext_clk_freq_hz = 0,
#endif
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,

        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = spk_slot_mask,
            .ws_width = I2S_DATA_BIT_WIDTH_32BIT,
            .ws_pol = false,
            .bit_shift = true,
            #if SOC_I2S_HW_VERSION_2
                .left_align = true,
                .big_endian = false,
                .bit_order_lsb = false
            #endif

        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = spk_bclk,
            .ws = spk_ws,
            .dout = spk_dout,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_, &tx_std_cfg));
#if SOC_I2S_SUPPORTS_PDM_RX
    // Create a new channel for MIC in PDM mode
    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(XIAOZHI_I2S_PORT(0), I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &rx_handle_));
    i2s_pdm_rx_config_t pdm_rx_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG((uint32_t)input_sample_rate_),
        /* The data bit-width of PDM mode is fixed to 16 */
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = mic_sck,
            .din = mic_din,

            .invert_flags = {
                .clk_inv = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(rx_handle_, &pdm_rx_cfg));
#else
    ESP_LOGE(TAG, "PDM is not supported");
#endif
    ESP_LOGI(TAG, "Simplex channels created");
}

int NoAudioCodecSimplexPdm::Read(int16_t* dest, int samples) {
    size_t bytes_read;

    // PDM 解调后的数据位宽为 16 位，直接读取到目标缓冲区
    if (i2s_channel_read(rx_handle_, dest, samples * sizeof(int16_t), &bytes_read, portMAX_DELAY) != ESP_OK) {
        ESP_LOGE(TAG, "Read Failed!");
        return 0;
    }

    samples = bytes_read / sizeof(int16_t);
    if (input_gain_ > 0) {
        int gain_factor = (int)input_gain_;
        for (int i = 0; i < samples; i++) {
            int32_t amplified = dest[i] * gain_factor;
            dest[i] = (amplified > INT16_MAX) ? INT16_MAX : (amplified < -INT16_MAX) ? -INT16_MAX : (int16_t)amplified;
        }
    }
    return samples;
}
