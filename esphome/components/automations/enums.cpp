#include "enums.h"
#include <string>

namespace esphome {
namespace automations {
namespace EnumUtils {

const char *sourceTriggerToString(SourceTrigger source) {
  switch (source) {
    case SourceTrigger::Input:
      return "Input";
    default:
      return "None";
  }
}

SourceTrigger stringToSourceTrigger(const std::string &str) {
  if (str == "Input")
    return SourceTrigger::Input;
  return SourceTrigger::None;
}

const char *sourceActionToString(SourceAction source) {
  switch (source) {
    case SourceAction::Delay:
      return "Delay";
    case SourceAction::Switch:
      return "Switch";
    default:
      return "None";
  }
}

SourceAction stringToSourceAction(const std::string &str) {
  if (str == "Delay")
    return SourceAction::Delay;
  if (str == "Switch")
    return SourceAction::Switch;
  return SourceAction::None;
}

// TypesInputTrigger
const char *inputTriggerTypeToString(TypesInputTrigger type) {
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

TypesInputTrigger stringToInputTriggerType(const std::string &str) {
  if (str == "press")
    return TypesInputTrigger::Press;
  if (str == "release")
    return TypesInputTrigger::Release;
  if (str == "click")
    return TypesInputTrigger::Click;
  return TypesInputTrigger::None;
}

// TypeSwitchAction
const char *switchActionTypeToString(TypeSwitchAction type) {
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

TypeSwitchAction stringToSwitchActionType(const std::string &str) {
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