#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/core/automation.h"
#include <vector>

namespace esphome {
namespace automations {

class BaseAutomation {
 public:
  // dallas_temp::DallasTemperatureSensor *sensor(uint16_t number) {
  //   if (number > this->sensors_.size() || number == 0)
  //     return nullptr;
  //   return this->sensors_[number - 1];
  // }

 protected:
  // std::vector<dallas_temp::DallasTemperatureSensor *> sensors_;
  // uint8_t saved_sensors_num_ = 0;
};

}  // namespace automations
}  // namespace esphome
