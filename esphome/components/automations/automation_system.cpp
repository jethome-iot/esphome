#include "automation_system.h"
#include <cstring>
#include <algorithm>

namespace esphome {
namespace automations {

TriggerConfig::TriggerConfig() : source(SourceTrigger::None) { memset(&params, 0, sizeof(params)); }

TriggerConfig::~TriggerConfig() {
  switch (source) {
    case SourceTrigger::Input:
      params.input.input_id.~basic_string();
      break;
    default:
      break;
  }
}

TriggerConfig::TriggerConfig(const TriggerConfig &other) : source(other.source) {
  switch (source) {
    case SourceTrigger::Input:
      new (&params.input.input_id) std::string(other.params.input.input_id);
      params.input.type = other.params.input.type;
      break;
    default:
      memset(&params, 0, sizeof(params));
      break;
  }
}

TriggerConfig &TriggerConfig::operator=(const TriggerConfig &other) {
  if (this != &other) {
    this->~TriggerConfig();
    new (this) TriggerConfig(other);
  }
  return *this;
}

void TriggerConfig::serialize(JsonObject &obj) const {
  obj["source"] = EnumUtils::sourceTriggerToString(source);

  switch (source) {
    case SourceTrigger::Input:
      obj["type"] = static_cast<uint8_t>(params.input.type);
      obj["input_id"] = params.input.input_id;
      break;
    default:
      break;
  }
}

bool TriggerConfig::deserialize(const JsonObject &obj) {
  if (!obj.containsKey("source"))
    return false;

  source = EnumUtils::stringToSourceTrigger(obj["source"].as<std::string>());

  switch (source) {
    case SourceTrigger::Input:
      params.input.type = static_cast<TypesInputTrigger>(obj["type"].as<uint8_t>());
      params.input.input_id = obj["input_id"].as<std::string>();
      break;
    default:
      break;
  }

  return true;
}

ActionConfig::ActionConfig() : source(SourceAction::None) { memset(&params, 0, sizeof(params)); }

ActionConfig::~ActionConfig() {
  switch (source) {
    case SourceAction::Switch:
      params.switch_action.switch_id.~basic_string();
      break;
    default:
      break;
  }
}

ActionConfig::ActionConfig(const ActionConfig &other) : source(other.source) {
  switch (source) {
    case SourceAction::Switch:
      new (&params.switch_action.switch_id) std::string(other.params.switch_action.switch_id);
      params.switch_action.type = other.params.switch_action.type;
      break;
    case SourceAction::Delay:
      params.common.delay = other.params.common.delay;
      break;
    default:
      memset(&params, 0, sizeof(params));
      break;
  }
}

ActionConfig &ActionConfig::operator=(const ActionConfig &other) {
  if (this != &other) {
    this->~ActionConfig();
    new (this) ActionConfig(other);
  }
  return *this;
}

void ActionConfig::serialize(JsonObject &obj) const {
  obj["source"] = EnumUtils::sourceActionToString(source);

  switch (source) {
    case SourceAction::Switch:
      obj["type"] = static_cast<uint8_t>(params.switch_action.type);
      obj["switch_id"] = params.switch_action.switch_id;
      break;
    case SourceAction::Delay:
      obj["delay_s"] = params.common.delay;
      break;
    default:
      break;
  }
}

bool ActionConfig::deserialize(const JsonObject &obj) {
  if (!obj.containsKey("source"))
    return false;

  source = EnumUtils::stringToSourceAction(obj["source"].as<std::string>());

  switch (source) {
    case SourceAction::Switch:
      params.switch_action.type = static_cast<TypeSwitchAction>(obj["type"].as<uint8_t>());
      params.switch_action.switch_id = obj["switch_id"].as<std::string>();
      break;
    case SourceAction::Common:
      params.common.delay = obj["delay_s"].as<uint32_t>();
      break;
    default:
      break;
  }

  return true;
}

void AutomationConfig::serialize(JsonObject &obj) const {
  obj["name"] = name;
  obj["id"] = id;
  obj["enabled"] = enabled;

  JsonObject trigger_obj = obj.createNestedObject("trigger");
  trigger.serialize(trigger_obj);

  JsonArray actions_array = obj.createNestedArray("actions");
  for (const auto &action : actions) {
    JsonObject action_obj = actions_array.createNestedObject();
    action.serialize(action_obj);
  }
}

bool AutomationConfig::deserialize(const JsonObject &obj) {
  if (!obj.containsKey("name") || !obj.containsKey("trigger")) {
    return false;
  }

  name = obj["name"].as<std::string>();
  enabled = obj["enabled"].as<bool>();

  if (!trigger.deserialize(obj["trigger"].as<JsonObject>())) {
    return false;
  }

  if (obj.containsKey("actions")) {
    JsonArray actions_array = obj["actions"].as<JsonArray>();
    for (const auto &action_obj : actions_array) {
      ActionConfig action;
      if (action.deserialize(action_obj.as<JsonObject>())) {
        actions.push_back(action);
      }
    }
  }

  return true;
}

// Реализация AutomationStorage
bool AutomationStorage::loadFromJson(const std::string &json_str) {
  DynamicJsonDocument doc(4096);
  DeserializationError error = deserializeJson(doc, json_str);

  if (error) {
    return false;
  }

  return loadFromJson(doc.as<JsonArray>());
}

bool AutomationStorage::loadFromJson(const JsonArray &array) {
  configs_.clear();

  for (const auto &item : array) {
    AutomationConfig config;
    if (config.deserialize(item.as<JsonObject>())) {
      configs_.push_back(config);
    }
  }

  return true;
}

std::string AutomationStorage::saveToJson() const {
  DynamicJsonDocument doc(4096);
  JsonArray array = doc.to<JsonArray>();

  for (const auto &config : configs_) {
    JsonObject obj = array.createNestedObject();
    config.serialize(obj);
  }

  std::string output;
  serializeJson(doc, output);
  return output;
}

void AutomationStorage::addConfig(const AutomationConfig &config) {
  for (auto &existing : configs_) {
    if (existing.id == config.id) {
      existing = config;
      return;
    }
  }
  configs_.push_back(config);
}

bool AutomationStorage::removeConfig(const std::string &id) {
  auto it = std::remove_if(configs_.begin(), configs_.end(),
                           [&id](const AutomationConfig &config) { return config.id == id; });

  if (it != configs_.end()) {
    configs_.erase(it, configs_.end());
    return true;
  }
  return false;
}

const AutomationConfig *AutomationStorage::getConfig(uint8_t index) const {
  if (index < configs_.size()) {
    return configs_[index];
  }
  return nullptr;
}

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