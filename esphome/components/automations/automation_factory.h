#pragma once
#include "automation_config.h"
#include "esphome/core/application.h"
#include "esphome/components/binary_sensor/automation.h"
#include "esphome/components/switch/automation.h"
#include "esphome/core/base_automation.h"
#include "esphome/core/automation.h"
#include "esphome/components/simple/simple_action.h"
#include "temperature_triggers.h"
#include "esphome/components/sensor/sensor.h"
#include <string>

namespace esphome {
namespace automations {

// Logging helpers (implemented in automation_factory.cpp)
void log_trigger_not_found(const std::string &object_id);
void log_switch_not_found(const std::string &object_id);
void log_sensor_not_found(const std::string &object_id);
void log_trigger_creation_error();
void log_action_creation_error();

template<typename... Ts> class TriggerFactory {
 public:
  static Trigger<Ts...> *create_trigger(const TriggerConfig &config) {
    switch (config.source) {
      case SourceTrigger::Input:
        return create_input_trigger(config);
      case SourceTrigger::Temperature:
        return create_temperature_trigger(config);
      default:
        return nullptr;
    }
  }

 private:
  static Trigger<Ts...> *create_input_trigger(const TriggerConfig &config) {
    auto *sensor = App.get_binary_sensor_by_key(config.params.input.input_id);
    if (sensor == nullptr) {
      log_trigger_not_found(format_hex_pretty(config.params.input.input_id));
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

  static Trigger<Ts...> *create_temperature_trigger(const TriggerConfig &config) {
    auto *sensor = App.get_sensor_by_key(config.params.temperature.sensor_id);
    if (sensor == nullptr) {
      log_sensor_not_found(format_hex_pretty(config.params.temperature.sensor_id));
      return nullptr;
    }

    switch (config.params.temperature.type) {
      case TypesTemperatureTrigger::Below:
        return new TemperatureBelowTrigger(sensor, config.params.temperature.threshold);
      case TypesTemperatureTrigger::Above:
        return new TemperatureAboveTrigger(sensor, config.params.temperature.threshold);
      case TypesTemperatureTrigger::Range:
        return new TemperatureRangeTrigger(sensor, config.params.temperature.min_threshold,
                                           config.params.temperature.max_threshold);
      default:
        return nullptr;
    }
  }
};

template<typename... Ts> class ActionFactory {
 public:
  static Action<Ts...> *create_action(const ActionConfig &config) {
    switch (config.source) {
      case SourceAction::Switch:
        return create_switch_action(config);
      case SourceAction::Delay:
        return create_delay_action(config);
      default:
        return create_empty_action();
    }
  }

 private:
  static Action<Ts...> *create_switch_action(const ActionConfig &config) {
    auto *switch_obj = App.get_switch_by_key(config.params.switch_action.switch_id);
    if (switch_obj == nullptr) {
      log_switch_not_found(format_hex_pretty(config.params.switch_action.switch_id));
      return create_empty_action();
    }

    switch (config.params.switch_action.type) {
      case TypeSwitchAction::TurnOff:
        return new switch_::TurnOffAction(switch_obj);
      case TypeSwitchAction::TurnOn:
        return new switch_::TurnOnAction(switch_obj);
      case TypeSwitchAction::Toggle:
        return new switch_::ToggleAction(switch_obj);
    };
    return create_empty_action();
  }

  static Action<Ts...> *create_delay_action(const ActionConfig &config) {
    auto *delay = new DelayAction();
    delay->set_delay(config.params.delay.delay_s * 1000);
    return delay;
  }

  static Action<Ts...> *create_empty_action() {
    auto *empty = new simple::EmptyAction<Ts...>();
    return empty;
  }
};

template<typename... Ts> class AutomationFactory {
 public:
  // Create automation from config
  static std::unique_ptr<Automation<Ts...>> create_automation(const AutomationConfig &config) {
    Trigger<Ts...> *trigger = TriggerFactory<Ts...>::create_trigger(config.trigger);
    if (!trigger) {
      log_trigger_creation_error();
      return nullptr;
    }

    auto automation = std::make_unique<Automation<Ts...>>(trigger);

    for (const auto &action_config : config.actions) {
      Action<Ts...> *action = ActionFactory<Ts...>::create_action(action_config);
      if (action) {
        automation->add_action(action);
      } else {
        log_action_creation_error();
        return nullptr;
      }
    }

    automation->set_enabled(config.enabled);

    return automation;
  }

  static std::vector<std::unique_ptr<Automation<Ts...>>> create_all_automations(
      const AutomationConfigStorage &storage) {
    std::vector<std::unique_ptr<Automation<Ts...>>> automations;
    automations.reserve(storage.size());

    for (const auto &config : storage.get_all_configs()) {
      auto automation = create_automation(config);
      automations.push_back(std::move(automation));
    }

    return automations;
  }
};

}  // namespace automations
}  // namespace esphome
