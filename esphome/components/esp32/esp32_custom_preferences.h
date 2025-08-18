#pragma once
#ifdef USE_ESP32

#include "esp32_preference_backend.h"

#include <string>

namespace esphome {
namespace esp32 {

class ESP32CustomPreferences {
 public:
  void set_namespace(const char *name) { this->nvs_namespace_ = std::string(name); }

  bool open();

  ESPPreferenceObject make_preference(std::string &&key) {
    auto *pref = new ESP32PreferenceBackend(pending_save);
    pref->nvs_handle = nvs_handle;
    pref->key = key;

    return ESPPreferenceObject(pref);
  }

  bool sync();
  bool is_changed(const NVSData &to_save);

  void reset();

 protected:
  uint32_t nvs_handle{0};
  std::vector<NVSData> pending_save;
  std::string nvs_namespace_;
};

}  // namespace esp32
}  // namespace esphome

#endif  // USE_ESP32
