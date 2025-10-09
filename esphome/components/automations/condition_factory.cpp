#include "condition_factory.h"
#include "esphome/core/base_automation.h"
#include "esphome/components/binary_sensor/automation.h"
#include "esphome/components/sensor/automation.h"
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
    case ConditionType::Temperature:
      return create_temperature_condition(config);
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

Condition<> *ConditionFactory::create_temperature_condition(const ConditionConfig &config) {
  // Find the temperature sensor by ID
  auto *sensor = App.get_sensor_by_key(config.sensor_id);

  if (!sensor) {
    ESP_LOGW(TAG, "Temperature sensor with ID 0x%08X not found", config.sensor_id);
    return nullptr;
  }

  // Create the appropriate range based on the temperature condition type
  float min_value, max_value;

  switch (config.temperature_type) {
    case TypesTemperatureCondition::Below:
      // Temperature < threshold
      min_value = NAN;  // No lower limit
      max_value = config.threshold;
      break;

    case TypesTemperatureCondition::Above:
      // Temperature > threshold
      min_value = config.threshold;
      max_value = NAN;  // No upper limit
      break;

    case TypesTemperatureCondition::Range:
      // min_threshold < Temperature < max_threshold
      min_value = config.min_threshold;
      max_value = config.max_threshold;
      break;

    default:
      ESP_LOGW(TAG, "Invalid temperature condition type");
      return nullptr;
  }
  auto *sensor_cond = new sensor::SensorInRangeCondition<>(sensor);
  sensor_cond->set_min(min_value);
  sensor_cond->set_max(max_value);
  return sensor_cond;
}

}  // namespace automations
}  // namespace esphome