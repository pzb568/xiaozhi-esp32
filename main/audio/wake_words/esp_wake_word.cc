#include "esp_wake_word.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <mbedtls/base64.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstdio>
#include <cmath>
#include <string>


#define TAG "EspWakeWord"

// ASCII waveform = 8 log lines/s on a 115200 UART console, long enough to overrun the I2S DMA
// (6 x 240 frames = 60 ms). Keep off unless specifically needed.
static constexpr bool kWaveDebug = false;

EspWakeWord::EspWakeWord() {
}

EspWakeWord::~EspWakeWord() {
    if (wakenet_data_ != nullptr) {
        wakenet_iface_->destroy(wakenet_data_);
    }
    if (owns_models_ && wakenet_model_ != nullptr) {
        esp_srmodel_deinit(wakenet_model_);
    }
}

bool EspWakeWord::Initialize(AudioCodec* codec, srmodel_list_t* models_list) {
    codec_ = codec;

    if (models_list == nullptr) {
        wakenet_model_ = esp_srmodel_init("model");
        owns_models_ = wakenet_model_ != nullptr;
    } else {
        wakenet_model_ = models_list;
    }

    if (wakenet_model_ == nullptr || wakenet_model_->num == -1) {
        ESP_LOGE(TAG, "Failed to initialize wakenet model");
        return false;
    }
    if (wakenet_model_->num == 0) {
        ESP_LOGE(TAG, "No model found");
        return false;
    }
    char *model_name = esp_srmodel_filter(wakenet_model_, ESP_WN_PREFIX, nullptr);
    if (model_name == nullptr) {
        ESP_LOGE(TAG, "No WakeNet model found");
        return false;
    }
    wakenet_iface_ = (esp_wn_iface_t*)esp_wn_handle_from_name(model_name);
    if (wakenet_iface_ == nullptr) {
        ESP_LOGE(TAG, "No WakeNet interface found for %s", model_name);
        return false;
    }
    // DET_MODE_90：放宽检测模式（较 DET_MODE_95 阈值更低、召回更高），
    // 适配远场/灵敏度偏低的麦克风场景，提高"你好小智"的唤醒成功率
    wakenet_data_ = wakenet_iface_->create(model_name, DET_MODE_90);
    if (wakenet_data_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create WakeNet model %s", model_name);
        return false;
    }

    // ---- 唤醒阈值探针 + 手动压到最低 0.4（接口允许范围 0.4~0.9999）----
    // 单唤醒词模型 word_index 从 1 开始；先打印当前阈值，再压到 0.4，
    // 最后回读确认。目标是"喊了必醒"，误触发后续再逐步上调。
    {
        int word_num = wakenet_iface_->get_word_num(wakenet_data_);
        for (int i = 1; i <= word_num; i++) {
            float th_before = wakenet_iface_->get_det_threshold(wakenet_data_, i);
            int ok = wakenet_iface_->set_det_threshold(wakenet_data_, 0.4f, i);
            float th_after = wakenet_iface_->get_det_threshold(wakenet_data_, i);
            ESP_LOGI(TAG, "[WakeDebug] word%d threshold %.3f -> set(0.40) ret=%d -> %.3f",
                     i, th_before, ok, th_after);
        }
    }
    // ---- 唤醒阈值探针 end ----

    int frequency = wakenet_iface_->get_samp_rate(wakenet_data_);
    int audio_chunksize = wakenet_iface_->get_samp_chunksize(wakenet_data_);
    ESP_LOGI(TAG, "Wake word(%s),freq: %d, chunksize: %d", model_name, frequency, audio_chunksize);

    return true;
}

void EspWakeWord::OnWakeWordDetected(std::function<void(const std::string& wake_word)> callback) {
    wake_word_detected_callback_ = callback;
}

void EspWakeWord::Start() {
    running_ = true;
}

void EspWakeWord::Stop() {
    running_ = false;

    std::lock_guard<std::mutex> lock(input_buffer_mutex_);
    input_buffer_.clear();
}

void EspWakeWord::Feed(const std::vector<int16_t>& data) {
    if (wakenet_data_ == nullptr) {
        return;
    }

    // ---- PcmDump (debug, remove later) ----
    // Once uptime >= 30 s: record 4 s of the exact 16 kHz stream WakeNet receives (decimated to 8 kHz),
    // then print it as base64 lines "PCM <n> <data>". Speak the wake word repeatedly during the 4 s.
    {
        constexpr int kDumpSamples = 8000 * 4;
        static int16_t* pcm = nullptr;
        static int pcm_len = 0;
        static int pcm_state = 0;  // 0 waiting, 1 recording, 2 done
        static int acc = 0, acc_n = 0;
        if (pcm_state == 0 && esp_timer_get_time() / 1000000 >= 30) {
            pcm = (int16_t*)heap_caps_malloc(kDumpSamples * sizeof(int16_t), MALLOC_CAP_8BIT);
            pcm_state = pcm ? 1 : 2;
            ESP_LOGW("PcmDump", "%s", pcm ? "START - speak the wake word now for 4 s" : "malloc failed");
        }
        if (pcm_state == 1) {
            int dch = codec_->input_channels() > 1 ? codec_->input_channels() : 1;
            for (size_t i = 0; i + dch - 1 < data.size() && pcm_len < kDumpSamples; i += dch) {
                acc += data[i];
                if (++acc_n == 2) {
                    pcm[pcm_len++] = (int16_t)(acc / 2);
                    acc = 0;
                    acc_n = 0;
                }
            }
            if (pcm_len >= kDumpSamples) {
                pcm_state = 2;
                ESP_LOGW("PcmDump", "DUMP BEGIN rate=8000 samples=%d", pcm_len);
                constexpr int kLineBytes = 192;
                unsigned char out[4 * kLineBytes / 3 + 8];
                const uint8_t* bytes = (const uint8_t*)pcm;
                int total = pcm_len * (int)sizeof(int16_t);
                for (int off = 0, seq = 0; off < total; off += kLineBytes, seq++) {
                    size_t n = total - off < kLineBytes ? total - off : kLineBytes;
                    size_t olen = 0;
                    mbedtls_base64_encode(out, sizeof(out), &olen, bytes + off, n);
                    out[olen] = 0;
                    printf("PCM %d %s\n", seq, (char*)out);
                    vTaskDelay(pdMS_TO_TICKS(30));
                }
                ESP_LOGW("PcmDump", "DUMP END");
                heap_caps_free(pcm);
                pcm = nullptr;
            }
        }
    }
    // ---- PcmDump end ----

    // ---- MicDebug 诊断：统计实际送入唤醒引擎的音量 + 信号质量，每秒打印一次 ----
    // RMS=0        : I2S 完全无数据（接线/驱动问题）
    // RMS < 20     : 采集到的几乎是静音（L/R 声道接反或麦克风失效）
    // RMS 30~100   : 正常环境底噪
    // 喊话时 > 500 : 麦克风完全正常，问题在唤醒识别环节
    // zcr          : 过零率，真人语音约 0.02~0.15；异常高(>0.3)=噪声/杂波，
    //                异常低(<0.005)=直流偏置/恒定信号
    // dc           : 直流偏置（应接近 0，偏大说明有 DC 分量）
    // [波形]       : 每秒把样本降采样成 40 点 ASCII 波形，肉眼判断波形形状
    {
        static int64_t sum_sq = 0;
        static int32_t peak_pos = 0, peak_neg = 0;
        static uint32_t n_samples = 0;
        static int64_t sum_val = 0;
        static uint32_t zc_count = 0;
        static int16_t prev_s = 0;
        static int64_t last_stats = 0;
        static int32_t wave[40];
        static uint32_t wave_idx = 0;
        static uint32_t wave_step = 0;
        static uint32_t big_count = 0;   // 尖峰计数（|s| > 8000 的样本数）
        int ch = codec_->input_channels() > 1 ? codec_->input_channels() : 1;
        for (size_t i = 0; i + ch - 1 < data.size(); i += ch) {
            int32_t s = data[i];  // 与下方送入 wakenet 的声道提取逻辑一致
            sum_sq += s * s;
            sum_val += s;
            if (s > peak_pos) peak_pos = s;
            if (-s > peak_neg) peak_neg = -s;
            if (s > 8000 || s < -8000) big_count++;   // 统计大幅尖峰
            if ((prev_s < 0 && s >= 0) || (prev_s >= 0 && s < 0)) zc_count++;
            prev_s = (int16_t)s;
            n_samples++;
            // 波形采样：连续取 40 个样本（每 40 个取 1 个），保留原始波形细节
            // 之前是每 400 个取 1 个，严重欠采样导致看不出形状
            if (wave_idx < 40 && wave_step++ % 40 == 0) {
                wave[wave_idx++] = s;
            }
        }
        int64_t now = esp_timer_get_time();
        if (last_stats == 0) {
            last_stats = now;
        } else if (now - last_stats >= 1000000) {
            if (n_samples > 0) {
                double rms = std::sqrt((double)sum_sq / n_samples);
                double dc = (double)sum_val / n_samples;
                double zcr = (double)zc_count / n_samples;
                ESP_LOGI(TAG, "[MicDebug] RMS=%.0f peak+=%d peak-=%d dc=%.1f zcr=%.3f n=%u spike=%.3f%%",
                         rms, peak_pos, peak_neg, dc, zcr, (unsigned)n_samples,
                         100.0 * big_count / n_samples);
                // ASCII 波形：自适应量程，7 行显示，能直接看出波形形状
                if (kWaveDebug && rms > 100) {
                    int32_t wmax = peak_pos > peak_neg ? peak_pos : peak_neg;
                    if (wmax < 100) wmax = 100;
                    const int kRows = 7;            // 7 行：+3~-3
                    auto row = [&](int level) {     // level: 3..-3
                        char line[45];
                        int p = 0;
                        line[p++] = '|';
                        for (int k = 0; k < 40; k++) {
                            // 该样本落在第几行（-3..3）
                            int band = (int)((int64_t)wave[k] * 3 / wmax);
                            if (band > 3) band = 3;
                            if (band < -3) band = -3;
                            line[p++] = (band == level) ? '#' : ' ';
                        }
                        line[p] = 0;
                        return std::string(line);
                    };
                    ESP_LOGI(TAG, "[波形] 量程 +-%d", wmax);
                    for (int lv = 3; lv >= -3; lv--) {
                        ESP_LOGI(TAG, "[波形]%s%s", lv == 0 ? "<" : " ", row(lv).c_str());
                    }
                }
            } else {
                ESP_LOGI(TAG, "[MicDebug] NO DATA! I2S input delivered no samples");
            }
            last_stats = now;
            sum_sq = 0;
            sum_val = 0;
            zc_count = 0;
            prev_s = 0;
            wave_idx = 0;
            wave_step = 0;
            big_count = 0;
            peak_pos = 0;
            peak_neg = 0;
            n_samples = 0;
        }
    }
    // ---- MicDebug end ----

    // ---- FeedMeter：每 10 秒打印累计送入 wakenet 的样本数 ----
    // 理论值 = 160000/10s。若显著偏多（如 170000），说明上游数据流"加速"
    // （重采样输出样本偏多），语音音调被拉高，模型必然无法识别。
    {
        static uint64_t total_fed = 0;
        static int64_t feed_t0 = 0;
        total_fed += data.size();
        int64_t now_f = esp_timer_get_time();
        if (feed_t0 == 0) {
            feed_t0 = now_f;
        } else if (now_f - feed_t0 >= 10000000) {
            ESP_LOGI(TAG, "[FeedMeter] 10s total=%llu (expect 160000, rate=%.1f%%)",
                     (unsigned long long)total_fed,
                     100.0 * total_fed / 160000.0);
            feed_t0 = now_f;
            total_fed = 0;
        }
    }
    // ---- FeedMeter end ----

    std::lock_guard<std::mutex> lock(input_buffer_mutex_);
    // Check running state inside lock to avoid TOCTOU race with Stop()
    if (!running_) {
        return;
    }

    if (codec_->input_channels() > 1) {
        for (size_t i = 0; i < data.size(); i += codec_->input_channels()) {
            input_buffer_.push_back(data[i]);
        }
    } else {
        input_buffer_.insert(input_buffer_.end(), data.begin(), data.end());
    }

    int chunksize = wakenet_iface_->get_samp_chunksize(wakenet_data_);
    while (input_buffer_.size() >= chunksize) {
        // ---- detect 探针：统计每秒 detect 调用次数（应约 31 次/秒 = 16000/512）----
        static uint32_t detect_count = 0;
        static int64_t detect_last = 0;
        detect_count++;
        {
            int64_t now_d = esp_timer_get_time();
            if (detect_last == 0) {
                detect_last = now_d;
            } else if (now_d - detect_last >= 1000000) {
                ESP_LOGI(TAG, "[MicDebug] detects=%u chunks/s", (unsigned)detect_count);
                detect_last = now_d;
                detect_count = 0;
            }
        }
        // ---- detect 探针 end ----
        int res = wakenet_iface_->detect(wakenet_data_, input_buffer_.data());
        if (res > 0) {
            last_detected_wake_word_ = wakenet_iface_->get_word_name(wakenet_data_, res);
            running_ = false;
            input_buffer_.clear();

            if (wake_word_detected_callback_) {
                wake_word_detected_callback_(last_detected_wake_word_);
            }
            break;
        }
        input_buffer_.erase(input_buffer_.begin(), input_buffer_.begin() + chunksize);
    }
}

size_t EspWakeWord::GetFeedSize() {
    if (wakenet_data_ == nullptr) {
        return 0;
    }
    return wakenet_iface_->get_samp_chunksize(wakenet_data_);
}

void EspWakeWord::EncodeWakeWordData() {
}

bool EspWakeWord::GetWakeWordOpus(std::vector<uint8_t>& opus) {
    return false;
}
