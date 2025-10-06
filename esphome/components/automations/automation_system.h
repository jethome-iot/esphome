#pragma once
#include <ArduinoJson.h>
#include <vector>
#include "enums.h"
#include "esphome/core/preferences.h"

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

struct JsonData {
  char data[4096];
};

class AutomationStorage {
 private:
  std::vector<AutomationConfig> configs_;
  ESPPreferenceObject json_obj_;

 public:
  void init();
  bool loadFromPreference();
  bool loadFromJson(const char *json_str);
  bool loadFromJson(const JsonArray &array);
  std::string saveToJson();
  bool saveToPreference();

  void addConfig(const AutomationConfig &config);
  bool removeConfig(const std::string &id);
  const AutomationConfig *getConfig(uint8_t index) const;
  bool removeConfig(uint8_t index);
  const std::vector<AutomationConfig> &getAllConfigs() const { return configs_; }
  void clear() { configs_.clear(); }

  size_t size() const { return configs_.size(); }
  bool empty() const { return configs_.empty(); }
};

extern AutomationStorage global_automation_storage;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

}  // namespace automations
}  // namespace esphome
