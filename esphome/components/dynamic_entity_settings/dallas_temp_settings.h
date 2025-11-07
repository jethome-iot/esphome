#pragma once
#if defined(USE_ESP32)
#include "dynamic_entity_settings.h"
#include "esphome/components/dallas_temp/dallas_temp.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/application.h"
#include "esphome/core/component.h"
#include "settings_names.h"

namespace esphome {
namespace dynamic_entity_settings {

#pragma pack(1)
struct DallasTempSettingsVer1Data {
  uint32_t entity_id = 0;
  uint32_t key() const { return this->entity_id; }
  uint32_t period_ms = 60000;
  uint8_t reserved[4] = {0};
};
#pragma pack()

struct DallasTempApplyPriority {
  static float get() { return setup_priority::DATA + 10.0f; }
};

class DallasTempSettingsVer1 : public SettingsBaseInterface {
 public:
  DallasTempSettingsVer1() { this->set_namespace(dallas_temp_settings_v1_name); }

  void set_namespace(const char *name) override { this->preference_array_.set_namespace(name); }

  const char *get_namespace() override { return this->preference_array_.get_namespace(); };

  bool check_available() override { return this->preference_array_.is_existing(); }

  bool init() override { return this->preference_array_.init(false); }

  bool load() override {
    this->preference_array_.restore_records_data();
    return true;
  }

  void apply() override {
    for (DallasTempSettingsVer1Data *set : this->preference_array_.records()) {
      this->apply(set);
    }
    this->setup_applied_ = true;
  }

  void apply(DallasTempSettingsVer1Data *set) {
    if (set == nullptr)
      return;

    auto *sensor_ptr = static_cast<sensor::Sensor *>(App.get_sensor_by_key(set->entity_id));
    if (sensor_ptr == nullptr)
      return;

    auto *dallas_ptr = static_cast<dallas_temp::DallasTemperatureSensor *>(sensor_ptr);
    dallas_ptr->set_update_interval(set->period_ms);
    if (this->setup_applied_) {
      dallas_ptr->stop_poller();
      dallas_ptr->start_poller();
    }
  }

  void make_record(DallasTempSettingsVer1Data *setting) { this->preference_array_.make_record(*setting); }

  bool get_record(dallas_temp::DallasTemperatureSensor *sensor_obj, DallasTempSettingsVer1Data *set) {
    set->entity_id = sensor_obj->get_object_id_hash();
    auto *sensor_ptr = static_cast<sensor::Sensor *>(App.get_entity_by_key(EntityType::SENSOR, set->entity_id));
    if (sensor_ptr == nullptr)
      return false;
    return this->preference_array_.find_record_by_key(*set);
  }

  size_t size() override { return this->preference_array_.get_size(); }

  bool make_conversion_from_last_version(SettingsBaseInterface *last) override { return true; }

  void reset() override { this->preference_array_.clear_all(); }

  void create_apply_components(std::vector<Component *> &components) override {
    if (!this->preference_array_.records().empty()) {
      components.push_back(new SettingsApplyComponent<DallasTempSettingsVer1, DallasTempApplyPriority>(this));
    }
  }

 protected:
  PreferenceArrayType<DallasTempSettingsVer1Data> preference_array_;
  bool setup_applied_ = false;
};

}  // namespace dynamic_entity_settings
}  // namespace esphome

#endif
