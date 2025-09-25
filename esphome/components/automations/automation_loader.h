#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include <vector>

namespace esphome {
namespace automations {

// Class for dynamic creation DallasTemperatureSensor for sensors connected to onewire bus
class AutomationLoader : public Component {
 public:
  void setup() override{
      // TODO it should be loaded from flash storage
      // Dynamic creation for test

  };

  float get_setup_priority() const override { return setup_priority::HARDWARE + 1; }

  void dump_config() override{};

  uint16_t sensors_size() { return this->sensors_.size(); }

  // dallas_temp::DallasTemperatureSensor *sensor(uint16_t number) {
  //   if (number > this->sensors_.size() || number == 0)
  //     return nullptr;
  //   return this->sensors_[number - 1];
  // }

 protected:
  // std::vector<dallas_temp::DallasTemperatureSensor *> sensors_;
  // uint8_t saved_sensors_num_ = 0;

  // SearchMode search_mode_ = SearchMode::ALL;
  ESPPreferenceObject sensors_count_pref_;
  std::vector<ESPPreferenceObject> addresses_pref_;
};

}  // namespace automations
}  // namespace esphome
