#ifdef USE_ESP32

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"
#include "esp32_preference_backend.h"
#include "esp32_base_preferences.h"
#include <nvs_flash.h>
#include <cstring>
#include <cinttypes>
#include <vector>
#include <string>

namespace esphome {
namespace esp32 {

static const char *const TAG = "esp32.preferences";
static const char *const NAMESPACE = "esphome";

class ESP32Preferences : public ESPPreferences, protected ESP32BasePreferences {
 public:
  void open() {
    nvs_flash_init();
    this->set_namespace(NAMESPACE);
    esp_err_t err = nvs_open(NAMESPACE, NVS_READWRITE, &this->nvs_handle_);
    if (err == 0)
      return;

    ESP_LOGW(TAG, "nvs_open failed: %s - erasing NVS", esp_err_to_name(err));
    nvs_flash_deinit();
    nvs_flash_erase();
    nvs_flash_init();

    err = nvs_open(NAMESPACE, NVS_READWRITE, &this->nvs_handle_);
    if (err != 0) {
      this->nvs_handle_ = 0;
    }
  }
  ESPPreferenceObject make_preference(size_t length, uint32_t type, bool in_flash) override {
    return make_preference(length, type);
  }
  ESPPreferenceObject make_preference(size_t length, uint32_t type) override {
    auto *pref = new ESP32PreferenceBackend(this->pending_save_);
    pref->nvs_handle = this->nvs_handle_;

    uint32_t keyval = type;
    pref->key = str_sprintf("%" PRIu32, keyval);

    return ESPPreferenceObject(pref);
  }

  bool sync() override { return ESP32BasePreferences::sync(); }

  bool reset() override {
    ESP_LOGD(TAG, "Erasing storage");
    this->pending_save_.clear();

    nvs_flash_deinit();
    nvs_flash_erase();
    // Make the handle invalid to prevent any saves until restart
    this->nvs_handle_ = 0;
    return true;
  }
};

void setup_preferences() {
  auto *prefs = new ESP32Preferences();  // NOLINT(cppcoreguidelines-owning-memory)
  prefs->open();
  global_preferences = prefs;
}

}  // namespace esp32

ESPPreferences *global_preferences;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

}  // namespace esphome

#endif  // USE_ESP32
