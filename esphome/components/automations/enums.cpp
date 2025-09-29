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
}  // namespace EnumUtils

}  // namespace automations
}  // namespace esphome