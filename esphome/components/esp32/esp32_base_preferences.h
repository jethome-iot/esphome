#pragma once
#ifdef USE_ESP32

#include "esp32_preference_backend.h"

#include <string>

namespace esphome {
namespace esp32 {

class ESP32BasePreferences {
 public:
  void set_namespace(const char *name) { this->nvs_namespace_ = std::string(name); }
  const char *get_namespace() { return this->nvs_namespace_.c_str(); }
  bool open();
  bool is_existing();
  ESPPreferenceObject make_preference(std::string &&key) {
    auto *pref = new ESP32PreferenceBackend(this->pending_save_);
    pref->nvs_handle = this->nvs_handle_;
    pref->key = std::move(key);

    return ESPPreferenceObject(pref);
  }

  bool sync();
  bool is_changed(const NVSData &to_save);

  void reset();

 protected:
  uint32_t nvs_handle_{0};
  std::vector<NVSData> pending_save_;
  std::string nvs_namespace_;
};

}  // namespace esp32
}  // namespace esphome

#endif  // USE_ESP32
