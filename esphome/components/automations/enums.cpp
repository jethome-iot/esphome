#include "enums.h"
#include <string>

namespace esphome {
namespace automations {
namespace EnumUtils {

const char *source_trigger_to_string(SourceTrigger source) {
  switch (source) {
    case SourceTrigger::Input:
      return "Input";
    case SourceTrigger::Temperature:
      return "Temperature";
    default:
      return "None";
  }
}

SourceTrigger string_to_source_trigger(const std::string &str) {
  if (str == "Input")
    return SourceTrigger::Input;
  if (str == "Temperature")
    return SourceTrigger::Temperature;
  return SourceTrigger::None;
}

const char *source_action_to_string(SourceAction source) {
  switch (source) {
    case SourceAction::Delay:
      return "Delay";
    case SourceAction::Switch:
      return "Switch";
    default:
      return "None";
  }
}

SourceAction string_to_source_action(const std::string &str) {
  if (str == "Delay")
    return SourceAction::Delay;
  if (str == "Switch")
    return SourceAction::Switch;
  return SourceAction::None;
}

// TypesInputTrigger
const char *input_trigger_type_to_string(TypesInputTrigger type) {
  switch (type) {
    case TypesInputTrigger::Press:
      return "press";
    case TypesInputTrigger::Release:
      return "release";
    case TypesInputTrigger::Click:
      return "click";
    case TypesInputTrigger::StateChange:
      return "state_change";
    default:
      return "none";
  }
}

TypesInputTrigger string_to_input_trigger_type(const std::string &str) {
  if (str == "press")
    return TypesInputTrigger::Press;
  if (str == "release")
    return TypesInputTrigger::Release;
  if (str == "click")
    return TypesInputTrigger::Click;
  if (str == "state_change")
    return TypesInputTrigger::StateChange;
  return TypesInputTrigger::None;
}

// TypesTemperatureTrigger
const char *temperature_trigger_type_to_string(TypesTemperatureTrigger type) {
  switch (type) {
    case TypesTemperatureTrigger::Below:
      return "below";
    case TypesTemperatureTrigger::Above:
      return "above";
    case TypesTemperatureTrigger::Range:
      return "range";
    default:
      return "none";
  }
}

TypesTemperatureTrigger string_to_temperature_trigger_type(const std::string &str) {
  if (str == "below")
    return TypesTemperatureTrigger::Below;
  if (str == "above")
    return TypesTemperatureTrigger::Above;
  if (str == "range")
    return TypesTemperatureTrigger::Range;
  return TypesTemperatureTrigger::None;
}

// TypeSwitchAction
const char *switch_action_type_to_string(TypeSwitchAction type) {
  switch (type) {
    case TypeSwitchAction::TurnOn:
      return "turn_on";
    case TypeSwitchAction::TurnOff:
      return "turn_off";
    case TypeSwitchAction::Toggle:
      return "toggle";
    default:
      return "none";
  }
}

TypeSwitchAction string_to_switch_action_type(const std::string &str) {
  if (str == "turn_on")
    return TypeSwitchAction::TurnOn;
  if (str == "turn_off")
    return TypeSwitchAction::TurnOff;
  if (str == "toggle")
    return TypeSwitchAction::Toggle;
  return TypeSwitchAction::None;
}

// ConditionType
const char *condition_type_to_string(ConditionType type) {
  switch (type) {
    case ConditionType::And:
      return "and";
    case ConditionType::Or:
      return "or";
    case ConditionType::Xor:
      return "xor";
    case ConditionType::Input:
      return "input";
    case ConditionType::Temperature:
      return "temperature";
    default:
      return "none";
  }
}

ConditionType string_to_condition_type(const std::string &str) {
  if (str == "and")
    return ConditionType::And;
  if (str == "or")
    return ConditionType::Or;
  if (str == "xor")
    return ConditionType::Xor;
  if (str == "input")
    return ConditionType::Input;
  if (str == "temperature")
    return ConditionType::Temperature;
  return ConditionType::None;
}

// InputConditionState
const char *input_condition_state_to_string(InputConditionState state) {
  switch (state) {
    case InputConditionState::True:
      return "true";
    case InputConditionState::False:
      return "false";
    default:
      return "false";
  }
}

InputConditionState string_to_input_condition_state(const std::string &str) {
  if (str == "true")
    return InputConditionState::True;
  return InputConditionState::False;
}

// TypesTemperatureCondition
const char *temperature_condition_type_to_string(TypesTemperatureCondition type) {
  switch (type) {
    case TypesTemperatureCondition::Below:
      return "below";
    case TypesTemperatureCondition::Above:
      return "above";
    case TypesTemperatureCondition::Range:
      return "range";
    default:
      return "none";
  }
}

TypesTemperatureCondition string_to_temperature_condition_type(const std::string &str) {
  if (str == "below")
    return TypesTemperatureCondition::Below;
  if (str == "above")
    return TypesTemperatureCondition::Above;
  if (str == "range")
    return TypesTemperatureCondition::Range;
  return TypesTemperatureCondition::None;
}

}  // namespace EnumUtils

}  // namespace automations
}  // namespace esphome
