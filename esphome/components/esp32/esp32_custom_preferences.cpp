#ifdef USE_ESP32

#include "esp32_custom_preferences.h"

#include <nvs_flash.h>

namespace esphome {
namespace esp32 {

static const char *const TAG = "esp32.custom.preferences";

bool ESP32CustomPreferences::open() {
  if (this->nvs_namespace_.empty()) {
    ESP_LOGW(TAG, "namespace isn't set");
    return false;
  }
  esp_err_t err = nvs_open(this->nvs_namespace_.c_str(), NVS_READWRITE, &nvs_handle);
  if (err != 0) {
    ESP_LOGW(TAG, "Space '%s': nvs_open failed: %s", this->nvs_namespace_.c_str(), esp_err_to_name(err));
    return false;
  }
  return true;
}

bool ESP32CustomPreferences::sync() {
  if (pending_save.empty())
    return true;

  ESP_LOGV(TAG, "Saving %d items...", pending_save.size());
  // goal try write all pending saves even if one fails
  int cached = 0, written = 0, failed = 0;
  esp_err_t last_err = ESP_OK;
  std::string last_key{};

  // go through vector from back to front (makes erase easier/more efficient)
  for (ssize_t i = pending_save.size() - 1; i >= 0; i--) {
    const auto &save = pending_save[i];
    ESP_LOGVV(TAG, "Checking if NVS data %s has changed", save.key.c_str());
    if (is_changed(nvs_handle, save)) {
      esp_err_t err;
      if (save.data.size() == 0) {
        err = nvs_erase_key(nvs_handle, save.key.c_str());
      } else {
        err = nvs_set_blob(nvs_handle, save.key.c_str(), save.data.data(), save.data.size());
        ESP_LOGV(TAG, "sync: key: %s, len: %d", save.key.c_str(), save.data.size());
      }

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
    pending_save.erase(pending_save.begin() + i);
  }

  ESP_LOGD(TAG, "Space '%s': writing %d items: %d cached, %d written, %d failed", this->nvs_namespace_.c_str(),
           cached + written + failed, cached, written, failed);
  if (failed > 0) {
    ESP_LOGE(TAG, "Space '%s': Writing %d items failed. Last error=%s for key=%s", this->nvs_namespace_.c_str(), failed,
             esp_err_to_name(last_err), last_key.c_str());
  }

  // note: commit on esp-idf currently is a no-op, nvs_set_blob always writes
  esp_err_t err = nvs_commit(nvs_handle);
  if (err != 0) {
    ESP_LOGV(TAG, "nvs_commit() failed: %s", esp_err_to_name(err));
    return false;
  }

  return failed == 0;
}

bool ESP32CustomPreferences::is_changed(const uint32_t nvs_handle, const NVSData &to_save) {
  NVSData stored_data{};
  size_t actual_len;
  esp_err_t err = nvs_get_blob(nvs_handle, to_save.key.c_str(), nullptr, &actual_len);
  if (err != 0) {
    ESP_LOGV(TAG, "nvs_get_blob('%s'): %s - the key might not be set yet", to_save.key.c_str(), esp_err_to_name(err));
    return true;
  }
  stored_data.data.resize(actual_len);
  err = nvs_get_blob(nvs_handle, to_save.key.c_str(), stored_data.data.data(), &actual_len);
  if (err != 0) {
    ESP_LOGV(TAG, "nvs_get_blob('%s') failed: %s", to_save.key.c_str(), esp_err_to_name(err));
    return true;
  }
  return to_save.data != stored_data.data;
}

void ESP32CustomPreferences::reset() {
  ESP_LOGD(TAG, "Space '%s': erasing", this->nvs_namespace_.c_str());
  pending_save.clear();
  nvs_erase_all(nvs_handle);
  nvs_commit(nvs_handle);
}

}  // namespace esp32
}  // namespace esphome

#endif  // USE_ESP32
