#pragma once
#include "automation_system.h"

namespace esphome {
namespace automations {

// Фабрика с фиксированными массивами для embedded систем
template<typename... Ts> class AutomationFactory {
 private:
  using TriggerCreator = std::function<Trigger<Ts...> *(const TriggerConfig &)>;
  using ActionCreator = std::function<Action<Ts...> *(const ActionConfig &)>;

  // Фиксированные массивы для быстрого доступа O(1)
  TriggerCreator trigger_creators_[MAX_TRIGGER_TYPES] = {nullptr};
  ActionCreator action_creators_[MAX_ACTION_TYPES] = {nullptr};

 public:
  AutomationFactory() = default;

  // Регистрация создателей
  void registerTriggerCreator(SourceTrigger source, TriggerCreator creator) {
    size_t index = EnumUtils::triggerToIndex(source);
    if (index < MAX_TRIGGER_TYPES) {
      trigger_creators_[index] = creator;
    }
  }

  void registerActionCreator(SourceAction source, ActionCreator creator) {
    size_t index = EnumUtils::actionToIndex(source);
    if (index < MAX_ACTION_TYPES) {
      action_creators_[index] = creator;
    }
  }

  // Поиск создателей
  TriggerCreator getTriggerCreator(SourceTrigger source) const {
    size_t index = EnumUtils::triggerToIndex(source);
    return (index < MAX_TRIGGER_TYPES) ? trigger_creators_[index] : nullptr;
  }

  ActionCreator getActionCreator(SourceAction source) const {
    size_t index = EnumUtils::actionToIndex(source);
    return (index < MAX_ACTION_TYPES) ? action_creators_[index] : nullptr;
  }

  // Создание одной автоматизации
  std::unique_ptr<Automation<Ts...>> createAutomation(const AutomationConfig &config) {
    if (!config.enabled) {
      return nullptr;
    }

    auto trigger_creator = getTriggerCreator(config.trigger.source);
    if (!trigger_creator) {
      return nullptr;
    }

    Trigger<Ts...> *trigger = trigger_creator(config.trigger);
    if (!trigger) {
      return nullptr;
    }

    auto automation = std::make_unique<Automation<Ts...>>(trigger);

    for (const auto &action_config : config.actions) {
      auto action_creator = getActionCreator(action_config.source);
      if (action_creator) {
        Action<Ts...> *action = action_creator(action_config);
        if (action) {
          automation->add_action(action);
        }
      }
    }

    return automation;
  }

  // Создание всех автоматизаций из хранилища
  std::vector<std::unique_ptr<Automation<Ts...>>> createAllAutomations(const AutomationStorage &storage) {
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

  // Проверка зарегистрированных типов
  bool isTriggerTypeSupported(SourceTrigger source) const { return getTriggerCreator(source) != nullptr; }

  bool isActionTypeSupported(SourceAction source) const { return getActionCreator(source) != nullptr; }

  // Очистка регистраций
  void clearTriggerCreator(SourceTrigger source) {
    size_t index = EnumUtils::triggerToIndex(source);
    if (index < MAX_TRIGGER_TYPES) {
      trigger_creators_[index] = nullptr;
    }
  }

  void clearActionCreator(SourceAction source) {
    size_t index = EnumUtils::actionToIndex(source);
    if (index < MAX_ACTION_TYPES) {
      action_creators_[index] = nullptr;
    }
  }

  void clearAll() {
    for (size_t i = 0; i < MAX_TRIGGER_TYPES; ++i) {
      trigger_creators_[i] = nullptr;
    }
    for (size_t i = 0; i < MAX_ACTION_TYPES; ++i) {
      action_creators_[i] = nullptr;
    }
  }
};

}  // namespace automations
}  // namespace esphome
