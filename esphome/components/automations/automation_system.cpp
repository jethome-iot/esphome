#include "automation_system.h"
#include <algorithm>
#include "esphome/core/helpers.h"
#include "esphome/components/json/json_util.h"
#include "esphome/core/log.h"

namespace esphome {
namespace automations {

TriggerConfig::TriggerConfig() : source(SourceTrigger::None) { memset(&params, 0, sizeof(params)); }

void TriggerConfig::serialize(JsonObject &obj) const {
  obj["source"] = EnumUtils::sourceTriggerToString(source);

  switch (source) {
    case SourceTrigger::Input:
      obj["type"] = EnumUtils::inputTriggerTypeToString(params.input.type);
      obj["input_id"] = format_hex(params.input.input_id);
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
    case SourceTrigger::Input: {
      params.input.type = EnumUtils::stringToInputTriggerType(obj["type"].as<std::string>());
      std::string id_str = obj["input_id"].as<std::string>();
      params.input.input_id = parse_hex<uint32_t>(id_str).value();
      break;
    }
    default:
      break;
  }

  return true;
}

ActionConfig::ActionConfig() : source(SourceAction::None) { memset(&params, 0, sizeof(params)); }

void ActionConfig::serialize(JsonObject &obj) const {
  obj["source"] = EnumUtils::sourceActionToString(source);

  switch (source) {
    case SourceAction::Switch:
      obj["type"] = EnumUtils::switchActionTypeToString(params.switch_action.type);
      obj["switch_id"] = format_hex(params.switch_action.switch_id);
      break;
    case SourceAction::Delay:
      obj["delay_s"] = params.delay.delay_s;
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
    case SourceAction::Switch: {
      params.switch_action.type = EnumUtils::stringToSwitchActionType(obj["type"].as<std::string>());
      auto id_str = obj["switch_id"].as<std::string>();
      params.switch_action.switch_id = parse_hex<uint32_t>(id_str).value();
      break;
    }
    case SourceAction::Delay:
      params.delay.delay_s = obj["delay_s"].as<uint32_t>();
      break;
    default:
      break;
  }

  return true;
}

void AutomationConfig::serialize(JsonObject &obj) const {
  obj["name"] = name;
  // obj["id"] = id;
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
    ESP_LOGI("Alex", "Failed load trigger");
    return false;
  }

  if (obj.containsKey("actions")) {
    JsonArray actions_array = obj["actions"].as<JsonArray>();
    for (const auto &action_obj : actions_array) {
      ActionConfig action;
      if (action.deserialize(action_obj.as<JsonObject>())) {
        actions.push_back(action);
      } else {
        ESP_LOGI("Alex", "Failed load action");
        return false;
      }
    }
  }

  return true;
}

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
    if (existing.name == config.name) {
      existing = config;
      return;
    }
  }
  configs_.push_back(config);
}

bool AutomationStorage::removeConfig(const std::string &name) {
  auto it = std::remove_if(configs_.begin(), configs_.end(),
                           [&name](const AutomationConfig &config) { return config.name == name; });

  if (it != configs_.end()) {
    configs_.erase(it, configs_.end());
    return true;
  }
  return false;
}

const AutomationConfig *AutomationStorage::getConfig(uint8_t index) const {
  if (index < configs_.size()) {
    return &(configs_[index]);
  }
  return nullptr;
}

}  // namespace automations
}  // namespace esphome