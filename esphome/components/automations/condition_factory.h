#pragma once

#include "esphome/core/automation.h"
#include "automation_config.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome {
namespace automations {

class ConditionFactory {
 public:
  // Create a Condition object from a ConditionConfig
  // Returns nullptr if the config is invalid
  static Condition<> *create_condition(const ConditionConfig &config);

 private:
  // Helper to create input (binary sensor) condition
  static Condition<> *create_input_condition(const ConditionConfig &config);

  // Helper to create composite conditions
  static Condition<> *create_and_condition(const ConditionConfig &config);
  static Condition<> *create_or_condition(const ConditionConfig &config);
};

}  // namespace automations
}  // namespace esphome