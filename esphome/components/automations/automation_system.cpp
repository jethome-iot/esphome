#include "automation_system.h"
#include <algorithm>
#include "esphome/core/helpers.h"
#include "esphome/components/json/json_util.h"
#include "esphome/core/log.h"

namespace esphome {
namespace automations {

static const char *const TAG = "automations";

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

bool AutomationConfigStorage::loadFromJson(const char *json_str, size_t max_buffer_size) {
  DynamicJsonDocument doc(max_buffer_size);
  DeserializationError error = deserializeJson(doc, json_str);

  if (error) {
    return false;
  }

  return loadFromJson(doc.as<JsonArray>());
}

bool AutomationConfigStorage::loadFromJson(const JsonArray &array) {
  configs_.clear();

  for (const auto &item : array) {
    AutomationConfig config;
    if (config.deserialize(item.as<JsonObject>())) {
      configs_.push_back(config);
    }
  }

  return true;
}

size_t AutomationConfigStorage::saveToJson(const char *json_str, size_t max_buffer_size) {
  DynamicJsonDocument doc(max_buffer_size);
  JsonArray array = doc.to<JsonArray>();

  for (const auto &config : configs_) {
    JsonObject obj = array.createNestedObject();
    config.serialize(obj);
  }

  size_t json_size = measureJson(doc);
  if (json_size > max_buffer_size - 1) {
    ESP_LOGE(TAG, "Buffer size is small. Json Doc size is %d", json_size);
    return 0;
  }

  serializeJson(doc, (void *) json_str, max_buffer_size);

  return json_size;
}

void AutomationConfigStorage::addConfig(const AutomationConfig &config) { configs_.push_back(config); }

void AutomationConfigStorage::updateConfig(uint8_t index, AutomationConfig *config) {
  if (index < configs_.size()) {
    configs_[index] = *config;
  }
}

bool AutomationConfigStorage::removeConfig(uint8_t index) {
  if (index < configs_.size()) {
    configs_.erase(configs_.begin() + index);
    return true;
  }
  return false;
}

AutomationConfig *AutomationConfigStorage::getConfig(uint8_t index) {
  if (index < configs_.size()) {
    return &(configs_[index]);
  }
  return nullptr;
}

}  // namespace automations
}  // namespace esphome