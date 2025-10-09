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

}  // namespace EnumUtils

}  // namespace automations
}  // namespace esphome
