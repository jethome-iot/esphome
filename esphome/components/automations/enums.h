#pragma once
#include <cstdint>

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

// Константы для размеров массивов
constexpr size_t MAX_TRIGGER_TYPES = static_cast<size_t>(SourceTrigger::MAX_TRIGGER_TYPES);
constexpr size_t MAX_ACTION_TYPES = static_cast<size_t>(SourceAction::MAX_ACTION_TYPES);

}  // namespace automations
}  // namespace esphome