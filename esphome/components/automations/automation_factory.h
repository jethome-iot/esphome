#pragma once
#include "automation_system.h"
#include "esphome/core/application.h"
#include "esphome/components/binary_sensor/automation.h"
#include "esphome/components/switch/automation.h"
#include "esphome/core/base_automation.h"
#include "esphome/core/automation.h"
#include "esphome/core/log.h"
#include "esphome/components/simple/simple_action.h"

namespace esphome {
namespace automations {

template<typename... Ts> class TriggerFactory {
 public:
  static Trigger<Ts...> *createTrigger(const TriggerConfig &config) {
    switch (config.source) {
      case SourceTrigger::Input:
        return createInputTrigger(config);
      default:
        return nullptr;
    }
  }

 private:
  static Trigger<Ts...> *createInputTrigger(const TriggerConfig &config) {
    auto *sensor = App.get_binary_sensor_by_key(config.params.input.input_id);
    if (sensor == nullptr) {
      ESP_LOGE("automation", "Cannot find trigger object_id %s",
               format_hex_pretty(config.params.input.input_id).c_str());
      return nullptr;
    }
    switch (config.params.input.type) {
      case TypesInputTrigger::Press:
        return new binary_sensor::PressTrigger(sensor);
      case TypesInputTrigger::Release:
        return new binary_sensor::ReleaseTrigger(sensor);
      case TypesInputTrigger::Click:
        return new binary_sensor::ClickTrigger(sensor, 200, 1000);
    };
    return nullptr;
  }
};

template<typename... Ts> class ActionFactory {
 public:
  static Action<Ts...> *createAction(const ActionConfig &config) {
    switch (config.source) {
      case SourceAction::Switch:
        return createSwitchAction(config);
      case SourceAction::Delay:
        return createDelayAction(config);
      default:
        return createEmptyAction();
    }
  }

 private:
  static Action<Ts...> *createSwitchAction(const ActionConfig &config) {
    auto *switch_obj = App.get_switch_by_key(config.params.switch_action.switch_id);
    if (switch_obj == nullptr) {
      ESP_LOGE("automation", "Cannot find switch with object_id %s",
               format_hex_pretty(config.params.switch_action.switch_id).c_str());
      return createEmptyAction();
    }

    switch (config.params.switch_action.type) {
      case TypeSwitchAction::TurnOff:
        return new switch_::TurnOffAction(switch_obj);
      case TypeSwitchAction::TurnOn:
        return new switch_::TurnOnAction(switch_obj);
      case TypeSwitchAction::Toggle:
        return new switch_::ToggleAction(switch_obj);
    };
    return createEmptyAction();
  }

  static Action<Ts...> *createDelayAction(const ActionConfig &config) {
    auto *delay = new DelayAction();
    delay->set_delay(config.params.delay.delay_s * 1000);
    return delay;
  }

  static Action<Ts...> *createEmptyAction() {
    auto *empty = new simple::EmptyAction<Ts...>();
    return empty;
  }
};

template<typename... Ts> class AutomationFactory {
 public:
  // Создание одной автоматизации
  static std::unique_ptr<Automation<Ts...>> createAutomation(const AutomationConfig &config) {
    if (!config.enabled) {
      return nullptr;
    }

    Trigger<Ts...> *trigger = TriggerFactory<Ts...>::createTrigger(config.trigger);
    if (!trigger) {
      ESP_LOGE("automation", "Error create Trigger");
      return nullptr;
    }

    auto automation = std::make_unique<Automation<Ts...>>(trigger);

    for (const auto &action_config : config.actions) {
      Action<Ts...> *action = ActionFactory<Ts...>::createAction(action_config);
      if (action) {
        automation->add_action(action);
      } else {
        ESP_LOGE("automation", "Error create Action");
        return nullptr;
      }
    }

    return automation;
  }

  static std::vector<std::unique_ptr<Automation<Ts...>>> createAllAutomations(const AutomationConfigStorage &storage) {
    std::vector<std::unique_ptr<Automation<Ts...>>> automations;
    automations.reserve(storage.size());

    for (const auto &config : storage.getAllConfigs()) {
      auto automation = createAutomation(config);
      if (automation) {
        automations.push_back(std::move(automation));
      }
    }

    return automations;
  }
};

}  // namespace automations
}  // namespace esphome
