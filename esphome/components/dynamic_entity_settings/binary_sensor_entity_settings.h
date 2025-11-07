#pragma once
#if defined(USE_ESP32) && defined(USE_BINARY_SENSOR)
#include "dynamic_entity_settings.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/application.h"
#include "settings_names.h"

namespace esphome {
namespace dynamic_entity_settings {

#pragma pack(1)
struct BinarySensorSettingsVer1Data {
  uint32_t entity_id = 0;
  uint32_t key() const { return this->entity_id; }
  uint8_t reserved[11] = {0};
  bool inverted = false;
};
#pragma pack()

class BinarySensorSettingsVer1 : public SettingsBaseInterface {
 public:
  BinarySensorSettingsVer1() { this->set_namespace(binary_sensor_settings_v1_name); }

  void set_namespace(const char *name) override { this->preference_array_.set_namespace(name); }

  const char *get_namespace() override { return this->preference_array_.get_namespace(); };

  bool check_available() override { return this->preference_array_.is_existing(); }

  bool init() override { return this->preference_array_.init(false); }

  bool load() override {
    this->preference_array_.restore_records_data();
    return true;
  }

  void apply() override {
    for (BinarySensorSettingsVer1Data *set : this->preference_array_.records()) {
      apply(set);
    }
  }

  void apply(BinarySensorSettingsVer1Data *set) {
    if (set == nullptr)
      return;
    binary_sensor::BinarySensor *sensor_ptr =
        static_cast<binary_sensor::BinarySensor *>(App.get_entity_by_key(EntityType::BINARY_SENSOR, set->entity_id));
    if (sensor_ptr == nullptr)
      return;
    sensor_ptr->set_inverted(set->inverted);
  }

  void make_record(BinarySensorSettingsVer1Data *setting) { this->preference_array_.make_record(*setting); }

  bool get_record(binary_sensor::BinarySensor *sensor_obj, BinarySensorSettingsVer1Data *set) {
    set->entity_id = sensor_obj->get_object_id_hash();
    binary_sensor::BinarySensor *sensor_ptr =
        static_cast<binary_sensor::BinarySensor *>(App.get_entity_by_key(EntityType::BINARY_SENSOR, set->entity_id));
    if (sensor_ptr == nullptr)
      return false;
    bool res = this->preference_array_.find_record_by_key(*set);
    return res;
  }

  size_t size() override { return preference_array_.get_size(); }

  bool make_conversion_from_last_version(SettingsBaseInterface *last) override { return true; }

  void reset() override { this->preference_array_.clear_all(); }

  void create_apply_components(std::vector<Component *> &components) override {
    if (!this->preference_array_.records().empty()) {
      components.push_back(new SettingsApplyComponent<BinarySensorSettingsVer1>(this));
    }
  }

 protected:
  PreferenceArrayType<BinarySensorSettingsVer1Data> preference_array_;
};

}  // namespace dynamic_entity_settings
}  // namespace esphome

#endif
