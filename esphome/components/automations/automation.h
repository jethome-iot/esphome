#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/core/automation.h"
#include <vector>

namespace esphome {
namespace automations {

enum class SourceTrigger : uint8_t {
  None,
  Input,
  // Temperature
};

enum class TypesInputTrigger : uint8_t {
  None,
  Press,
  Release,
  Click,
};

struct BaseTrigger {
  SourceTrigger source_trigger;
};

struct InputTrigger : public BaseTrigger {
  TypesInputTrigger type;
  // May be options
};

enum class SourceAction {
  None,
  Common,
  Switch,
};

enum class TypeSwitchAction {
  None,
  TurnOn,
  TurnOff,
  Toggle,
};

enum class TypeCommonAction { None, Delay };

struct BaseAction {
  SourceAction source_action;
};

struct SwitchAction : public BaseAction {
  TypeSwitchAction type;
};

struct CommonAction : public BaseAction {
  TypeCommonAction type;
};

class BaseAutomation {
 public:
  // dallas_temp::DallasTemperatureSensor *sensor(uint16_t number) {
  //   if (number > this->sensors_.size() || number == 0)
  //     return nullptr;
  //   return this->sensors_[number - 1];
  // }

 protected:
  // std::vector<dallas_temp::DallasTemperatureSensor *> sensors_;
  // uint8_t saved_sensors_num_ = 0;
};

}  // namespace automations
}  // namespace esphome
