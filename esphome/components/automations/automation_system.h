#pragma once
#include <ArduinoJson.h>
#include <memory>
#include <vector>
#include <string>
#include <functional>
#include "enums.h"
#include "esphome/core/automation.h"

namespace esphome {
namespace automations {

struct TriggerConfig {
  SourceTrigger source = SourceTrigger::None;

  union {
    struct {
      TypesInputTrigger type;
      std::string input_id;
    } input;

    // Other
  } params;

  TriggerConfig();
  ~TriggerConfig();
  TriggerConfig(const TriggerConfig &other);
  TriggerConfig &operator=(const TriggerConfig &other);

  void serialize(JsonObject &obj) const;
  bool deserialize(const JsonObject &obj);
};

struct ActionConfig {
  SourceAction source = SourceAction::None;

  union {
    struct {
      TypeSwitchAction type;
      std::string switch_id;
    } switch_action;

    struct {
      uint32_t delay_s;
    } common;
  } params;

  ActionConfig();
  ~ActionConfig();
  ActionConfig(const ActionConfig &other);
  ActionConfig &operator=(const ActionConfig &other);

  void serialize(JsonObject &obj) const;
  bool deserialize(const JsonObject &obj);
};

struct AutomationConfig {
  std::string name;
  bool enabled = true;
  TriggerConfig trigger;
  std::vector<ActionConfig> actions;

  void serialize(JsonObject &obj) const;
  bool deserialize(const JsonObject &obj);
};

class AutomationStorage {
 private:
  std::vector<AutomationConfig> configs_;

 public:
  bool loadFromJson(const std::string &json_str);
  bool loadFromJson(const JsonArray &array);
  std::string saveToJson() const;

  void addConfig(const AutomationConfig &config);
  bool removeConfig(const std::string &id);
  const AutomationConfig *getConfig(const std::string &id) const;
  const std::vector<AutomationConfig> &getAllConfigs() const { return configs_; }
  void clear() { configs_.clear(); }

  size_t size() const { return configs_.size(); }
  bool empty() const { return configs_.empty(); }
};

// Вспомогательные функции для работы с enum
namespace EnumUtils {
const char *sourceTriggerToString(SourceTrigger source);
SourceTrigger stringToSourceTrigger(const std::string &str);

const char *sourceActionToString(SourceAction source);
SourceAction stringToSourceAction(const std::string &str);

// Конвертация enum в индекс
constexpr size_t triggerToIndex(SourceTrigger source) { return static_cast<size_t>(source); }

constexpr size_t actionToIndex(SourceAction source) { return static_cast<size_t>(source); }
}  // namespace EnumUtils

}  // namespace automations
}  // namespace esphome