#include "esp_wake_word.h"
#include <esp_log.h>


#define TAG "EspWakeWord"

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
    wakenet_data_ = wakenet_iface_->create(model_name, DET_MODE_95);
    if (wakenet_data_ == nullptr) {
        ESP_LOGE(TAG, "Failed to create WakeNet model %s", model_name);
        return false;
    }

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

    std::lock_guard<std::mutex> lock(input_buffer_mutex_);
    // Check running state inside lock to avoid TOCTOU race with Stop()
    if (!running_) {
        static int not_running_cnt = 0;
        static TickType_t last_log = 0;
        not_running_cnt++;
        TickType_t now = xTaskGetTickCount();
        if ((now - last_log) > pdMS_TO_TICKS(5000)) {
            ESP_LOGW("WakeDbg", "NOT RUNNING! Feed called %d times in last 5s", not_running_cnt);
            not_running_cnt = 0;
            last_log = now;
        }
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
    static int detect_cnt = 0;
    static int buf_peak = 0;
    while (input_buffer_.size() >= chunksize) {
        int res = wakenet_iface_->detect(wakenet_data_, input_buffer_.data());
        detect_cnt++;
        if ((int)input_buffer_.size() > buf_peak) buf_peak = (int)input_buffer_.size();
        if (detect_cnt % 500 == 0) {
            ESP_LOGI("WakeDbg", "detect#%d chunk=%d buf=%d peak=%d res=%d",
                     detect_cnt, chunksize, (int)input_buffer_.size(), buf_peak, res);
            buf_peak = 0;
            ESP_LOGI("WakeDbg", "samples: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
                     input_buffer_[0], input_buffer_[1], input_buffer_[2], input_buffer_[3],
                     input_buffer_[4], input_buffer_[5], input_buffer_[6], input_buffer_[7],
                     input_buffer_[8], input_buffer_[9], input_buffer_[10], input_buffer_[11],
                     input_buffer_[12], input_buffer_[13], input_buffer_[14], input_buffer_[15]);
        }
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
