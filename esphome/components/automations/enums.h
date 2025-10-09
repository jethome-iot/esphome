#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string>

namespace esphome {
namespace automations {

enum class SourceTrigger : uint8_t { None = 0, Input, Temperature, MAX_TRIGGER_TYPES };

enum class TypesInputTrigger : uint8_t {
  None,
  Press,
  Release,
  Click,
};

enum class TypesTemperatureTrigger : uint8_t {
  None,
  Below,
  Above,
  Range,
};

enum class SourceAction : uint8_t { None = 0, Delay, Switch, MAX_ACTION_TYPES };

enum class TypeSwitchAction : uint8_t {
  None,
  TurnOn,
  TurnOff,
  Toggle,
};

enum class ConditionType : uint8_t { None = 0, And, Or, Not, BinarySensor, MAX_CONDITION_TYPES };

enum class BinarySensorConditionState : uint8_t {
  False = 0,
  True = 1,
};

constexpr size_t MAX_TRIGGER_TYPES = static_cast<size_t>(SourceTrigger::MAX_TRIGGER_TYPES);
constexpr size_t MAX_ACTION_TYPES = static_cast<size_t>(SourceAction::MAX_ACTION_TYPES);
constexpr size_t MAX_CONDITION_TYPES = static_cast<size_t>(ConditionType::MAX_CONDITION_TYPES);

namespace EnumUtils {
// SourceTrigger
const char *source_trigger_to_string(SourceTrigger source);
SourceTrigger string_to_source_trigger(const std::string &str);

// TypesInputTrigger
const char *input_trigger_type_to_string(TypesInputTrigger type);
TypesInputTrigger string_to_input_trigger_type(const std::string &str);

// TypesTemperatureTrigger
const char *temperature_trigger_type_to_string(TypesTemperatureTrigger type);
TypesTemperatureTrigger string_to_temperature_trigger_type(const std::string &str);

// SourceAction
const char *source_action_to_string(SourceAction source);
SourceAction string_to_source_action(const std::string &str);

// TypeSwitchAction
const char *switch_action_type_to_string(TypeSwitchAction type);
TypeSwitchAction string_to_switch_action_type(const std::string &str);

// ConditionType
const char *condition_type_to_string(ConditionType type);
ConditionType string_to_condition_type(const std::string &str);

// BinarySensorConditionState
const char *binary_sensor_condition_state_to_string(BinarySensorConditionState state);
BinarySensorConditionState string_to_binary_sensor_condition_state(const std::string &str);

constexpr size_t trigger_to_index(SourceTrigger source) { return static_cast<size_t>(source); }

constexpr size_t action_to_index(SourceAction source) { return static_cast<size_t>(source); }

constexpr size_t condition_to_index(ConditionType type) { return static_cast<size_t>(type); }
}  // namespace EnumUtils

}  // namespace automations
}  // namespace esphome
