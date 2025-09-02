#pragma once
#if defined(USE_ESP32) && defined(USE_SWITCH)
#include "dynamic_entity_settings.h"
#include "esphome/components/switch/switch.h"
#include "settings_names.h"

namespace esphome {
namespace dynamic_entity_settings {

#pragma pack(1)
struct SwitchSettingsVer1Data {
  uint32_t entity_id = 0;
  uint32_t key() const { return this->entity_id; }
  switch_::SwitchRestoreMode restore_mode = switch_::SWITCH_ALWAYS_OFF;
  uint8_t reserved[7] = {0};
  bool inverted = false;
};
#pragma pack()

class SwitchSettingsVer1 : public SettingsBaseInterface {
 public:
  SwitchSettingsVer1() { this->set_namespace(switch_settings_v1_name); }

  void set_namespace(const char *name) override { this->preference_array_.set_namespace(name); }

  const char *get_namespace() override { return this->preference_array_.get_namespace(); };

  bool check_available() override { return this->preference_array_.is_existing(); }

  bool init() override { return this->preference_array_.init(false); }

  bool load() override {
    this->preference_array_.restore_records_data();
    return true;
  }

  void apply() override {
    for (SwitchSettingsVer1Data *set : this->preference_array_.records()) {
      apply(set);
    }
  }

  void apply(SwitchSettingsVer1Data *set) {
    if (set == nullptr)
      return;
    switch_::Switch *switch_ptr =
        static_cast<switch_::Switch *>(App.get_entity_by_key(EntityType::SWITCH, set->entity_id));
    if (switch_ptr == nullptr)
      return;
    switch_ptr->set_restore_mode(set->restore_mode);
    switch_ptr->set_inverted(set->inverted);
    switch_ptr->update();
  }

  void make_record(SwitchSettingsVer1Data *setting) { this->preference_array_.make_record(*setting); }

  bool get_record(switch_::Switch *switch_obj, SwitchSettingsVer1Data *set) {
    set->entity_id = switch_obj->get_object_id_hash();
    switch_::Switch *switch_ptr =
        static_cast<switch_::Switch *>(App.get_entity_by_key(EntityType::SWITCH, set->entity_id));
    if (switch_ptr == nullptr)
      return false;
    bool res = this->preference_array_.find_record_by_key(*set);
    return res;
  }

  size_t size() override { return preference_array_.get_size(); }

  bool make_conversion_from_last_version(SettingsBaseInterface *last) override { return true; }

  void reset() override { this->preference_array_.clear_all(); }

 protected:
  PreferenceArrayType<SwitchSettingsVer1Data> preference_array_;
};

}  // namespace dynamic_entity_settings
}  // namespace esphome

#endif
