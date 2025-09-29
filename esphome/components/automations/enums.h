#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string>

namespace esphome {
namespace automations {

enum class SourceTrigger : uint8_t { None = 0, Input, MAX_TRIGGER_TYPES };

enum class TypesInputTrigger : uint8_t {
  None,
  Press,
  Release,
  Click,
};

enum class SourceAction : uint8_t { None = 0, Delay, Switch, MAX_ACTION_TYPES };

enum class TypeSwitchAction : uint8_t {
  None,
  TurnOn,
  TurnOff,
  Toggle,
};

constexpr size_t MAX_TRIGGER_TYPES = static_cast<size_t>(SourceTrigger::MAX_TRIGGER_TYPES);
constexpr size_t MAX_ACTION_TYPES = static_cast<size_t>(SourceAction::MAX_ACTION_TYPES);

namespace EnumUtils {
// SourceTrigger
const char *sourceTriggerToString(SourceTrigger source);
SourceTrigger stringToSourceTrigger(const std::string &str);

// TypesInputTrigger
const char *inputTriggerTypeToString(TypesInputTrigger type);
TypesInputTrigger stringToInputTriggerType(const std::string &str);

// SourceAction
const char *sourceActionToString(SourceAction source);
SourceAction stringToSourceAction(const std::string &str);

// TypeSwitchAction
const char *switchActionTypeToString(TypeSwitchAction type);
TypeSwitchAction stringToSwitchActionType(const std::string &str);

constexpr size_t triggerToIndex(SourceTrigger source) { return static_cast<size_t>(source); }

constexpr size_t actionToIndex(SourceAction source) { return static_cast<size_t>(source); }
}  // namespace EnumUtils

}  // namespace automations
}  // namespace esphome