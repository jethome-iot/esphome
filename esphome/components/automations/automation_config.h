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

    struct {
      TypesTemperatureTrigger type;
      uint32_t sensor_id;
      float threshold;      // For Below/Above triggers
      float min_threshold;  // For Range trigger
      float max_threshold;  // For Range trigger
    } temperature;

    // Other
  } params;

  TriggerConfig();
  ~TriggerConfig() = default;
  TriggerConfig(const TriggerConfig &other) = default;
  TriggerConfig &operator=(const TriggerConfig &other) = default;

  void serialize(JsonObject &obj) const;
  bool deserialize(const JsonObject &obj);
};

struct ConditionConfig {
  ConditionType type = ConditionType::None;

  // For composite conditions (And, Or)
  std::vector<ConditionConfig> sub_conditions;

  // For Input condition
  uint32_t sensor_id = 0;
  InputConditionState state = InputConditionState::True;

  // For Temperature condition
  TypesTemperatureCondition temperature_type = TypesTemperatureCondition::None;
  float threshold = 0.0f;      // For Below/Above conditions
  float min_threshold = 0.0f;  // For Range condition
  float max_threshold = 0.0f;  // For Range condition

  ConditionConfig();
  ~ConditionConfig() = default;
  ConditionConfig(const ConditionConfig &other) = default;
  ConditionConfig &operator=(const ConditionConfig &other) = default;

  void serialize(JsonObject &obj) const;
  bool deserialize(const JsonObject &obj);
  bool is_valid() const { return type != ConditionType::None; }
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
  ConditionConfig condition;  // Optional condition - check type != None
  std::vector<ActionConfig> actions;

  void serialize(JsonObject &obj) const;
  bool deserialize(const JsonObject &obj);
};

class AutomationConfigStorage {
 private:
  std::vector<AutomationConfig> configs_;
  ESPPreferenceObject json_obj_;

 public:
  bool load_from_json(const char *json_str, size_t max_buffer_size);
  bool load_from_json(const JsonArray &array);
  size_t save_to_json(char *json_str, size_t max_buffer_size);

  void add_config(const AutomationConfig &config);
  AutomationConfig *get_config(uint8_t index);
  void update_config(uint8_t index, AutomationConfig *);
  bool remove_config(uint8_t index);
  const std::vector<AutomationConfig> &get_all_configs() const { return configs_; }
  void clear() { configs_.clear(); }

  size_t size() const { return configs_.size(); }
  bool empty() const { return configs_.empty(); }
};

}  // namespace automations
}  // namespace esphome
