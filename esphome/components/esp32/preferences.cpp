#ifdef USE_ESP32

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"
#include "esp32_preference_backend.h"
#include "esp32_custom_preferences.h"
#include <nvs_flash.h>
#include <cstring>
#include <cinttypes>
#include <vector>
#include <string>

namespace esphome {
namespace esp32 {

static const char *const TAG = "esp32.preferences";

class ESP32Preferences : public ESPPreferences, protected ESP32CustomPreferences {
 public:
  void open() {
    nvs_flash_init();
    esp_err_t err = nvs_open("esphome", NVS_READWRITE, &nvs_handle);
    if (err == 0)
      return;

    ESP_LOGW(TAG, "nvs_open failed: %s - erasing NVS", esp_err_to_name(err));
    nvs_flash_deinit();
    nvs_flash_erase();
    nvs_flash_init();

    err = nvs_open("esphome", NVS_READWRITE, &nvs_handle);
    if (err != 0) {
      nvs_handle = 0;
    }
  }
  ESPPreferenceObject make_preference(size_t length, uint32_t type, bool in_flash) override {
    return make_preference(length, type);
  }
  ESPPreferenceObject make_preference(size_t length, uint32_t type) override {
    auto *pref = new ESP32PreferenceBackend(pending_save);  // NOLINT(cppcoreguidelines-owning-memory)
    pref->nvs_handle = nvs_handle;

    uint32_t keyval = type;
    pref->key = str_sprintf("%" PRIu32, keyval);

    return ESPPreferenceObject(pref);
  }

  bool sync() override { return ESP32CustomPreferences::sync(); }

  bool reset() override {
    ESP_LOGD(TAG, "Erasing storage");
    pending_save.clear();

    nvs_flash_deinit();
    nvs_flash_erase();
    // Make the handle invalid to prevent any saves until restart
    nvs_handle = 0;
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
