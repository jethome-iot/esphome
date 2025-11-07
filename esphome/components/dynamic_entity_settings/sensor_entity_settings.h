#pragma once
#if defined(USE_ESP32) && defined(USE_SENSOR)
#include "dynamic_entity_settings.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "settings_names.h"

namespace esphome {
namespace dynamic_entity_settings {

#pragma pack(1)
struct SensorSettingsVer1Data {
  uint32_t entity_id = 0;
  uint32_t key() const { return this->entity_id; }
  uint32_t period_ms = 60000;
  uint8_t reserved[4] = {0};
};
#pragma pack()

class SensorSettingsVer1 : public SettingsBaseInterface {
 public:
  SensorSettingsVer1() { this->set_namespace(sensor_settings_v1_name); }

  void set_namespace(const char *name) override { this->preference_array_.set_namespace(name); }

  const char *get_namespace() override { return this->preference_array_.get_namespace(); };

  bool check_available() override { return this->preference_array_.is_existing(); }

  bool init() override { return this->preference_array_.init(false); }

  bool load() override {
    this->preference_array_.restore_records_data();
    return true;
  }

  void apply() override {
    for (SensorSettingsVer1Data *set : this->preference_array_.records()) {
      apply(set);
    }
  }

  void apply(SensorSettingsVer1Data *set) {
    if (set == nullptr)
      return;
    sensor::Sensor *sensor_ptr =
        static_cast<sensor::Sensor *>(App.get_entity_by_key(EntityType::SENSOR, set->entity_id));
    if (sensor_ptr == nullptr)
      return;

    // Cast to PollingComponent to set update interval
    // IMPORTANT: This only works for sensors that inherit from BOTH Sensor and PollingComponent
    // (such as DallasTemperatureSensor). Without RTTI, we can't verify this at runtime.
    // Using reinterpret_cast to handle multiple inheritance without RTTI.
    PollingComponent *polling_ptr = reinterpret_cast<PollingComponent *>(sensor_ptr);
    polling_ptr->set_update_interval(set->period_ms);
    polling_ptr->stop_poller();
    polling_ptr->start_poller();
  }

  void make_record(SensorSettingsVer1Data *setting) { this->preference_array_.make_record(*setting); }

  bool get_record(sensor::Sensor *sensor_obj, SensorSettingsVer1Data *set) {
    set->entity_id = sensor_obj->get_object_id_hash();
    sensor::Sensor *sensor_ptr =
        static_cast<sensor::Sensor *>(App.get_entity_by_key(EntityType::SENSOR, set->entity_id));
    if (sensor_ptr == nullptr)
      return false;
    bool res = this->preference_array_.find_record_by_key(*set);
    return res;
  }

  size_t size() override { return preference_array_.get_size(); }

  bool make_conversion_from_last_version(SettingsBaseInterface *last) override { return true; }

  void reset() override { this->preference_array_.clear_all(); }

 protected:
  PreferenceArrayType<SensorSettingsVer1Data> preference_array_;
};

}  // namespace dynamic_entity_settings
}  // namespace esphome

#endif
