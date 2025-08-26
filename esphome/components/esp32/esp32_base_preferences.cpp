#ifdef USE_ESP32

#include "esp32_base_preferences.h"

#include <nvs_flash.h>

namespace esphome {
namespace esp32 {

static const char *const TAG = "esp32.preferences";

bool ESP32BasePreferences::open() {
  if (this->nvs_namespace_.empty()) {
    ESP_LOGW(TAG, "namespace isn't set");
    return false;
  }
  esp_err_t err = nvs_open(this->nvs_namespace_.c_str(), NVS_READWRITE, &this->nvs_handle_);
  if (err != 0) {
    ESP_LOGW(TAG, "Space '%s': nvs_open failed: %s", this->nvs_namespace_.c_str(), esp_err_to_name(err));
    return false;
  }
  return true;
}

bool ESP32BasePreferences::is_existing() {
  if (this->nvs_namespace_.empty()) {
    ESP_LOGW(TAG, "namespace isn't set");
    return false;
  }
  uint32_t nvs_handle_temp;
  esp_err_t err = nvs_open(this->nvs_namespace_.c_str(), NVS_READONLY, &nvs_handle_temp);
  if (err == 0) {
    nvs_close(nvs_handle_temp);
    return true;
  }
  return false;
}

bool ESP32BasePreferences::sync() {
  if (this->pending_save_.empty())
    return true;

  ESP_LOGV(TAG, "Saving %d items...", this->pending_save_.size());
  // goal try write all pending saves even if one fails
  int cached = 0, written = 0, failed = 0;
  esp_err_t last_err = ESP_OK;
  std::string last_key{};

  // go through vector from back to front (makes erase easier/more efficient)
  for (ssize_t i = this->pending_save_.size() - 1; i >= 0; i--) {
    const auto &save = this->pending_save_[i];
    ESP_LOGVV(TAG, "Checking if NVS data %s has changed", save.key.c_str());
    esp_err_t err = ESP_OK;
    bool changed = false;
    if (save.data.empty()) {
      err = nvs_erase_key(this->nvs_handle_, save.key.c_str());
      ESP_LOGV(TAG, "remove: key: %s", save.key.c_str());
      changed = true;
    } else if (is_changed(save)) {
      err = nvs_set_blob(this->nvs_handle_, save.key.c_str(), save.data.data(), save.data.size());
      ESP_LOGV(TAG, "sync: key: %s, len: %d", save.key.c_str(), save.data.size());
      changed = true;
    }

    if (changed) {
      if (err != 0) {
        ESP_LOGV(TAG, "nvs_set_blob('%s', len=%u) failed: %s", save.key.c_str(), save.data.size(),
                 esp_err_to_name(err));
        failed++;
        last_err = err;
        last_key = save.key;
        continue;
      }
      written++;
    } else {
      ESP_LOGV(TAG, "NVS data not changed skipping %s  len=%u", save.key.c_str(), save.data.size());
      cached++;
    }
    this->pending_save_.erase(this->pending_save_.begin() + i);
  }

  ESP_LOGD(TAG, "Space '%s': writing %d items: %d cached, %d written, %d failed", this->nvs_namespace_.c_str(),
           cached + written + failed, cached, written, failed);
  if (failed > 0) {
    ESP_LOGE(TAG, "Space '%s': Writing %d items failed. Last error=%s for key=%s", this->nvs_namespace_.c_str(), failed,
             esp_err_to_name(last_err), last_key.c_str());
  }

  // note: commit on esp-idf currently is a no-op, nvs_set_blob always writes
  esp_err_t err = nvs_commit(this->nvs_handle_);
  if (err != 0) {
    ESP_LOGV(TAG, "nvs_commit() failed: %s", esp_err_to_name(err));
    return false;
  }

  return failed == 0;
}

bool ESP32BasePreferences::is_changed(const NVSData &to_save) {
  NVSData stored_data{};
  size_t actual_len;
  esp_err_t err = nvs_get_blob(this->nvs_handle_, to_save.key.c_str(), nullptr, &actual_len);
  if (err != 0) {
    ESP_LOGV(TAG, "nvs_get_blob('%s'): %s - the key might not be set yet", to_save.key.c_str(), esp_err_to_name(err));
    return true;
  }
  stored_data.data.resize(actual_len);
  err = nvs_get_blob(this->nvs_handle_, to_save.key.c_str(), stored_data.data.data(), &actual_len);
  if (err != 0) {
    ESP_LOGV(TAG, "nvs_get_blob('%s') failed: %s", to_save.key.c_str(), esp_err_to_name(err));
    return true;
  }
  return to_save.data != stored_data.data;
}

void ESP32BasePreferences::reset() {
  ESP_LOGD(TAG, "Space '%s': erasing", this->nvs_namespace_.c_str());
  this->pending_save_.clear();
  nvs_erase_all(this->nvs_handle_);
  nvs_commit(this->nvs_handle_);
}

}  // namespace esp32
}  // namespace esphome

#endif  // USE_ESP32
