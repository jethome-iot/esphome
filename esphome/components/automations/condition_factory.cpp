#include "condition_factory.h"
#include "esphome/core/base_automation.h"
#include "esphome/components/binary_sensor/automation.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace automations {

static const char *const TAG = "condition_factory";

Condition<> *ConditionFactory::create_condition(const ConditionConfig &config) {
  if (!config.is_valid()) {
    return nullptr;
  }

  switch (config.type) {
    case ConditionType::Input:
      return create_input_condition(config);
    case ConditionType::And:
      return create_and_condition(config);
    case ConditionType::Or:
      return create_or_condition(config);
    default:
      ESP_LOGW(TAG, "Unknown condition type");
      return nullptr;
  }
}

Condition<> *ConditionFactory::create_input_condition(const ConditionConfig &config) {
  // Find the binary sensor by ID using the efficient lookup method
  auto *sensor = App.get_binary_sensor_by_key(config.sensor_id);

  if (!sensor) {
    ESP_LOGW(TAG, "Binary sensor with ID 0x%08X not found", config.sensor_id);
    return nullptr;
  }

  bool expected_state = (config.state == InputConditionState::True);
  return new binary_sensor::BinarySensorCondition<>(sensor, expected_state);
}

Condition<> *ConditionFactory::create_and_condition(const ConditionConfig &config) {
  std::vector<Condition<> *> conditions;

  for (const auto &sub_config : config.sub_conditions) {
    Condition<> *sub_condition = create_condition(sub_config);
    if (sub_condition) {
      conditions.push_back(sub_condition);
    }
  }

  if (conditions.empty()) {
    ESP_LOGW(TAG, "AND condition has no valid sub-conditions");
    return nullptr;
  }

  return new AndCondition<>(conditions);
}

Condition<> *ConditionFactory::create_or_condition(const ConditionConfig &config) {
  std::vector<Condition<> *> conditions;

  for (const auto &sub_config : config.sub_conditions) {
    Condition<> *sub_condition = create_condition(sub_config);
    if (sub_condition) {
      conditions.push_back(sub_condition);
    }
  }

  if (conditions.empty()) {
    ESP_LOGW(TAG, "OR condition has no valid sub-conditions");
    return nullptr;
  }

  return new OrCondition<>(conditions);
}

}  // namespace automations
}  // namespace esphome