#pragma once
#include <ArduinoJson.h>
#include <vector>
#include "enums.h"

namespace esphome {
namespace automations {

struct TriggerConfig {
  SourceTrigger source = SourceTrigger::None;

  union {
    struct {
      TypesInputTrigger type;
      uint32_t input_id;
    } input;

    // Other
  } params;

  TriggerConfig();
  ~TriggerConfig() = default;
  TriggerConfig(const TriggerConfig &other) = default;
  TriggerConfig &operator=(const TriggerConfig &other) = default;

  void serialize(JsonObject &obj) const;
  bool deserialize(const JsonObject &obj);
};

struct ActionConfig {
  SourceAction source = SourceAction::None;

  union {
    struct {
      TypeSwitchAction type;
      uint32_t switch_id;
    } switch_action;

    struct {
      uint32_t delay_s;
    } delay;
  } params;

  ActionConfig();
  ~ActionConfig() = default;
  ActionConfig(const ActionConfig &other) = default;
  ActionConfig &operator=(const ActionConfig &other) = default;

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
  const AutomationConfig *getConfig(uint8_t index) const;
  const std::vector<AutomationConfig> &getAllConfigs() const { return configs_; }
  void clear() { configs_.clear(); }

  size_t size() const { return configs_.size(); }
  bool empty() const { return configs_.empty(); }
};

extern AutomationStorage global_automation_storage;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

}  // namespace automations
}  // namespace esphome
