#pragma once
#ifdef USE_ESP32
#include "esp32_base_preferences.h"
#include "esphome/core/preferences_array.h"

namespace esphome {
namespace esp32 {

template<typename RecordType, typename BaseClassType> class ESP32PreferencesArrayBase : public BaseClassType {
 public:
  ESPPreferenceObject make_counter_pref() override { return this->preference_.make_preference("counter"); }

  ESPPreferenceObject make_index_pref(uint32_t index) override {
    return this->preference_.make_preference(std::to_string(index));
  }

  void sync() override { this->preference_.sync(); }
  void set_namespace(const char *name) { this->preference_.set_namespace(name); }
  const char *get_namespace() { return this->preference_.get_namespace(); }
  bool is_existing() { return this->preference_.is_existing(); }

  bool init(bool restore_data = true) {
    bool res = this->preference_.open();
    if (!res)
      return false;

    return BaseClassType::init(restore_data);
  }

 protected:
  ESP32BasePreferences preference_;
};

template<typename RecordType>
using ESP32PreferencesArray = ESP32PreferencesArrayBase<RecordType, esphome::ESPPreferencesArray<RecordType>>;

template<typename RecordType>
using ESP32PreferencesArrayKey = ESP32PreferencesArrayBase<RecordType, esphome::ESPPreferencesArrayKey<RecordType>>;

}  // namespace esp32
}  // namespace esphome

#endif
