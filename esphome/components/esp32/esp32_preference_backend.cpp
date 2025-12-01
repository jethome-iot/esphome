#ifdef USE_ESP32
#include "esp32_preference_backend.h"
#include <nvs_flash.h>

namespace esphome {
namespace esp32 {

static const char *const TAG = "esp32.preferences";

bool ESP32PreferenceBackend::save(const uint8_t *data, size_t len) {
  // try find in pending saves and update that
  for (auto &obj : pending_save) {
    if (obj.key == key) {
      obj.set_data(data, len);
      return true;
    }
  }
  NVSData save{};
  save.key = key;
  save.set_data(data, len);
  pending_save.emplace_back(std::move(save));
  ESP_LOGVV(TAG, "pending_save: key: %s, len: %zu", key.c_str(), len);
  return true;
}
bool ESP32PreferenceBackend::load(uint8_t *data, size_t len) {
  // try find in pending saves and load from that
  for (auto &obj : pending_save) {
    if (obj.key == key) {
      if (obj.len != len) {
        // size mismatch
        return false;
      }
      memcpy(data, obj.data.get(), len);
      return true;
    }
  }
  size_t actual_len;
  esp_err_t err = nvs_get_blob(nvs_handle, key.c_str(), nullptr, &actual_len);
  if (err != 0) {
    ESP_LOGV(TAG, "nvs_get_blob('%s'): %s - the key might not be set yet", key.c_str(), esp_err_to_name(err));
    return false;
  }
  if (actual_len != len) {
    ESP_LOGVV(TAG, "NVS length does not match (%zu!=%zu)", actual_len, len);
    return false;
  }
  err = nvs_get_blob(nvs_handle, key.c_str(), data, &len);
  if (err != 0) {
    ESP_LOGV(TAG, "nvs_get_blob('%s') failed: %s", key.c_str(), esp_err_to_name(err));
    return false;
  } else {
    ESP_LOGVV(TAG, "nvs_get_blob: key: %s, len: %zu", key.c_str(), len);
  }
  return true;
}

bool ESP32PreferenceBackend::remove() {
  for (auto it = pending_save.begin(); it != pending_save.end(); it++) {
    if (it->key == key) {
      it = pending_save.erase(it);
      break;
    }
  }
  NVSData save{};
  save.key = key;
  save.data = nullptr;
  save.len = 0;
  pending_save.emplace_back(std::move(save));
  return true;
}

}  // namespace esp32
}  // namespace esphome

#endif
