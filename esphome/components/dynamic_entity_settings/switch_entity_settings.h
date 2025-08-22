#pragma once
#if defined(USE_ESP32) && defined(USE_SWITCH)
#include "dynamic_entity_settings.h"
#include "esphome/components/esp32/esp32_preferences_array.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/application.h"
#include "settings_names.h"

namespace esphome {
namespace dynamic_entity_settings {

#pragma pack(1)
struct SwitchSettings_ver1 {
  uint32_t entity_id = 0;
  uint32_t key() const { return this->entity_id; }
  switch_::SwitchRestoreMode restore_mode;
  uint8_t reserved[7];
  bool inverted;
};
#pragma pack()

class SwitchSettingsVer1 : public SettingsBase {
 public:
  SwitchSettingsVer1() { this->set_namespace(switch_settings_v1_name); }

  void set_namespace(std::string &&name) { this->preference_array_.set_namespace(name); }

  const char *get_namespace() override { return this->preference_array_.get_namespace(); };

  bool check_available() override { return this->preference_array_.get_preference().is_existing(); }

  bool load() override { this->preference_array_.restore_records_data(); }

  void apply() override {
    for (SwitchSettings_ver1 *set : this->preference_array_::records()) {
      if (set == nullptr)
        continue;
      switch_::Switch *switch_ptr =
          static_cast<switch_::Switch>(App.get_entity_by_key(EntityType::SWITCH, set->entity_id));
      if (switch_ptr == nullptr)
        continue;
      switch_ptr->set_restore_mode(set->restore_mode);
      switch_ptr->set_inverted(set->inverted);
    }
  }

  void make_record(SwitchSettings_ver1 &setting) { this->preference_array_.make_record(setting); }

  bool make_conversion_from_last_version(SettingsBase *last) override { return true; }

 protected:
  esp32::ESP32PreferencesArrayKey<SwitchSettings_ver1> preference_array_;
};

}

#endif
