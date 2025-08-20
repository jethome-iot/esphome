#pragma once
#ifdef USE_ESP32
#include "esp32_base_preferences.h"
#include "esphome/core/preferences_array.h"

namespace esphome {
namespace esp32 {

template<typename RecordType> class ESP32PreferencesArray : public ESPPreferencesArray<RecordType> {
 public:
  using base_class = ESPPreferencesArray<RecordType>;
  ESPPreferenceObject make_counter_pref() override { return this->preference_.make_preference("counter"); }

  ESPPreferenceObject make_index_pref(uint32_t index) override {
    return this->preference_.make_preference(str_snprintf("%d", 5, index));
  }

  void sync() override { this->preference_.sync(); }
  void set_namespace(const char *name) { this->preference_.set_namespace(name); }

  bool init(bool restore_data = true) {
    bool res = this->preference_.open();
    if (!res)
      return false;

    return base_class::init(restore_data);
  }

 protected:
  ESP32BasePreferences preference_;
};

template<typename RecordType> class ESP32PreferencesArrayKey : public ESPPreferencesArrayKey<RecordType> {};

}  // namespace esp32
}  // namespace esphome

#endif
