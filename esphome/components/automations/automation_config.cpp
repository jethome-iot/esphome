#include "automation_config.h"
#include <algorithm>
#include "esphome/core/helpers.h"
#include "esphome/components/json/json_util.h"
#include "esphome/core/log.h"

namespace esphome {
namespace automations {

static const char *const TAG = "automations";

// Helper function to serialize a vector of uint8_t to cron field string
static std::string serialize_cron_field(const std::vector<uint8_t> &values, uint8_t min_val, uint8_t max_val) {
  if (values.empty()) {
    return "*";
  }

  // Check if this is a full range (all values present)
  bool is_full_range = true;
  for (uint8_t i = min_val; i <= max_val; i++) {
    if (std::find(values.begin(), values.end(), i) == values.end()) {
      is_full_range = false;
      break;
    }
  }

  if (is_full_range) {
    return "*";
  }

  // Build comma-separated list
  std::string result;
  for (size_t i = 0; i < values.size(); i++) {
    if (i > 0) {
      result += ",";
    }
    result += std::to_string(values[i]);
  }
  return result;
}

// Helper function to deserialize a cron field string to vector
static std::vector<uint8_t> deserialize_cron_field(const std::string &field, uint8_t min_val, uint8_t max_val) {
  std::vector<uint8_t> result;

  if (field == "*") {
    // Add all values in range
    for (uint8_t i = min_val; i <= max_val; i++) {
      result.push_back(i);
    }
    return result;
  }

  // Parse comma-separated values
  size_t start = 0;
  size_t comma_pos;
  while ((comma_pos = field.find(',', start)) != std::string::npos) {
    std::string val_str = field.substr(start, comma_pos - start);
    uint8_t val = static_cast<uint8_t>(std::stoi(val_str));
    if (val >= min_val && val <= max_val) {
      result.push_back(val);
    }
    start = comma_pos + 1;
  }
  // Last value
  if (start < field.length()) {
    std::string val_str = field.substr(start);
    uint8_t val = static_cast<uint8_t>(std::stoi(val_str));
    if (val >= min_val && val <= max_val) {
      result.push_back(val);
    }
  }

  return result;
}

TriggerConfig::TriggerConfig() : source(SourceTrigger::None) { memset(&params, 0, sizeof(params)); }

void TriggerConfig::serialize(JsonObject &obj) const {
  obj["source"] = EnumUtils::source_trigger_to_string(source);

  switch (source) {
    case SourceTrigger::Input:
      obj["type"] = EnumUtils::input_trigger_type_to_string(params.input.type);
      obj["input_id"] = format_hex(params.input.input_id);
      break;
    case SourceTrigger::Temperature:
      obj["type"] = EnumUtils::temperature_trigger_type_to_string(params.temperature.type);
      obj["sensor_id"] = format_hex(params.temperature.sensor_id);
      if (params.temperature.type == TypesTemperatureTrigger::Below ||
          params.temperature.type == TypesTemperatureTrigger::Above) {
        obj["threshold"] = params.temperature.threshold;
      } else if (params.temperature.type == TypesTemperatureTrigger::Range) {
        obj["min_threshold"] = params.temperature.min_threshold;
        obj["max_threshold"] = params.temperature.max_threshold;
      }
      break;
    case SourceTrigger::Cron: {
      // Serialize as space-separated cron string: "seconds minutes hours days_of_month months days_of_week"
      std::string cron_str;
      cron_str += serialize_cron_field(cron_seconds, 0, 60);
      cron_str += " ";
      cron_str += serialize_cron_field(cron_minutes, 0, 59);
      cron_str += " ";
      cron_str += serialize_cron_field(cron_hours, 0, 23);
      cron_str += " ";
      cron_str += serialize_cron_field(cron_days_of_month, 1, 31);
      cron_str += " ";
      cron_str += serialize_cron_field(cron_months, 1, 12);
      cron_str += " ";
      cron_str += serialize_cron_field(cron_days_of_week, 1, 7);
      obj["cron"] = cron_str;
      break;
    }
    default:
      break;
  }
}

bool TriggerConfig::deserialize(const JsonObject &obj) {
  if (!obj.containsKey("source"))
    return false;

  source = EnumUtils::string_to_source_trigger(obj["source"].as<std::string>());

  switch (source) {
    case SourceTrigger::Input: {
      params.input.type = EnumUtils::string_to_input_trigger_type(obj["type"].as<std::string>());
      std::string id_str = obj["input_id"].as<std::string>();
      params.input.input_id = parse_hex<uint32_t>(id_str).value();
      break;
    }
    case SourceTrigger::Temperature: {
      params.temperature.type = EnumUtils::string_to_temperature_trigger_type(obj["type"].as<std::string>());
      std::string id_str = obj["sensor_id"].as<std::string>();
      params.temperature.sensor_id = parse_hex<uint32_t>(id_str).value();

      if (params.temperature.type == TypesTemperatureTrigger::Below ||
          params.temperature.type == TypesTemperatureTrigger::Above) {
        params.temperature.threshold = obj["threshold"].as<float>();
      } else if (params.temperature.type == TypesTemperatureTrigger::Range) {
        params.temperature.min_threshold = obj["min_threshold"].as<float>();
        params.temperature.max_threshold = obj["max_threshold"].as<float>();
      }
      break;
    }
    case SourceTrigger::Cron: {
      if (!obj.containsKey("cron"))
        return false;

      std::string cron_str = obj["cron"].as<std::string>();

      // Parse space-separated cron string: "seconds minutes hours days_of_month months days_of_week"
      std::vector<std::string> fields;
      size_t start = 0;
      size_t space_pos;
      while ((space_pos = cron_str.find(' ', start)) != std::string::npos) {
        fields.push_back(cron_str.substr(start, space_pos - start));
        start = space_pos + 1;
      }
      // Last field
      if (start < cron_str.length()) {
        fields.push_back(cron_str.substr(start));
      }

      if (fields.size() != 6) {
        ESP_LOGE(TAG, "Invalid cron format, expected 6 fields but got %d", fields.size());
        return false;
      }

      cron_seconds = deserialize_cron_field(fields[0], 0, 60);
      cron_minutes = deserialize_cron_field(fields[1], 0, 59);
      cron_hours = deserialize_cron_field(fields[2], 0, 23);
      cron_days_of_month = deserialize_cron_field(fields[3], 1, 31);
      cron_months = deserialize_cron_field(fields[4], 1, 12);
      cron_days_of_week = deserialize_cron_field(fields[5], 1, 7);
      break;
    }
    default:
      break;
  }

  return true;
}

ConditionConfig::ConditionConfig()
    : type(ConditionType::None),
      sensor_id(0),
      state(InputConditionState::True),
      temperature_type(TypesTemperatureCondition::None),
      threshold(0.0f),
      min_threshold(0.0f),
      max_threshold(0.0f) {}

void ConditionConfig::serialize(JsonObject &obj) const {
  obj["type"] = EnumUtils::condition_type_to_string(type);

  switch (type) {
    case ConditionType::And:
    case ConditionType::Or:
    case ConditionType::Xor: {
      JsonArray sub_array = obj.createNestedArray("conditions");
      for (const auto &sub : sub_conditions) {
        JsonObject sub_obj = sub_array.createNestedObject();
        sub.serialize(sub_obj);
      }
      break;
    }
    case ConditionType::Input:
      obj["sensor_id"] = format_hex(sensor_id);
      obj["state"] = EnumUtils::input_condition_state_to_string(state);
      break;
    case ConditionType::Temperature:
      obj["sensor_id"] = format_hex(sensor_id);
      obj["temperature_type"] = EnumUtils::temperature_condition_type_to_string(temperature_type);
      if (temperature_type == TypesTemperatureCondition::Below ||
          temperature_type == TypesTemperatureCondition::Above) {
        obj["threshold"] = threshold;
      } else if (temperature_type == TypesTemperatureCondition::Range) {
        obj["min_threshold"] = min_threshold;
        obj["max_threshold"] = max_threshold;
      }
      break;
    default:
      break;
  }
}

bool ConditionConfig::deserialize(const JsonObject &obj) {
  if (!obj.containsKey("type"))
    return false;

  type = EnumUtils::string_to_condition_type(obj["type"].as<std::string>());

  switch (type) {
    case ConditionType::And:
    case ConditionType::Or:
    case ConditionType::Xor: {
      if (obj.containsKey("conditions")) {
        JsonArray sub_array = obj["conditions"].as<JsonArray>();
        for (const auto &sub_obj : sub_array) {
          ConditionConfig sub_condition;
          if (sub_condition.deserialize(sub_obj.as<JsonObject>())) {
            sub_conditions.push_back(sub_condition);
          }
        }
      }
      break;
    }
    case ConditionType::Input: {
      std::string id_str = obj["sensor_id"].as<std::string>();
      sensor_id = parse_hex<uint32_t>(id_str).value();
      if (obj.containsKey("state")) {
        state = EnumUtils::string_to_input_condition_state(obj["state"].as<std::string>());
      }
      break;
    }
    case ConditionType::Temperature: {
      std::string id_str = obj["sensor_id"].as<std::string>();
      sensor_id = parse_hex<uint32_t>(id_str).value();
      temperature_type = EnumUtils::string_to_temperature_condition_type(obj["temperature_type"].as<std::string>());

      if (temperature_type == TypesTemperatureCondition::Below ||
          temperature_type == TypesTemperatureCondition::Above) {
        threshold = obj["threshold"].as<float>();
      } else if (temperature_type == TypesTemperatureCondition::Range) {
        min_threshold = obj["min_threshold"].as<float>();
        max_threshold = obj["max_threshold"].as<float>();
      }
      break;
    }
    default:
      break;
  }

  return true;
}

ActionConfig::ActionConfig() : source(SourceAction::None) { memset(&params, 0, sizeof(params)); }

void ActionConfig::serialize(JsonObject &obj) const {
  obj["source"] = EnumUtils::source_action_to_string(source);

  switch (source) {
    case SourceAction::Switch:
      obj["type"] = EnumUtils::switch_action_type_to_string(params.switch_action.type);
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

  source = EnumUtils::string_to_source_action(obj["source"].as<std::string>());

  switch (source) {
    case SourceAction::Switch: {
      params.switch_action.type = EnumUtils::string_to_switch_action_type(obj["type"].as<std::string>());
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

  // Serialize condition if it exists
  if (condition.is_valid()) {
    JsonObject condition_obj = obj.createNestedObject("condition");
    condition.serialize(condition_obj);
  }

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

  // Deserialize condition if it exists
  if (obj.containsKey("condition")) {
    if (!condition.deserialize(obj["condition"].as<JsonObject>())) {
      ESP_LOGI("Alex", "Failed to load condition");
      // Condition is optional, so we don't return false here
    }
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

bool AutomationConfigStorage::load_from_json(const char *json_str, size_t max_buffer_size) {
  DynamicJsonDocument doc(max_buffer_size);
  DeserializationError error = deserializeJson(doc, json_str);

  if (error) {
    return false;
  }

  return load_from_json(doc.as<JsonArray>());
}

bool AutomationConfigStorage::load_from_json(const JsonArray &array) {
  configs_.clear();

  for (const auto &item : array) {
    AutomationConfig config;
    if (config.deserialize(item.as<JsonObject>())) {
      configs_.push_back(config);
    }
  }

  return true;
}

size_t AutomationConfigStorage::save_to_json(char *json_str, size_t max_buffer_size) {
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

void AutomationConfigStorage::add_config(const AutomationConfig &config) { configs_.push_back(config); }

void AutomationConfigStorage::update_config(uint8_t index, AutomationConfig *config) {
  if (index < configs_.size()) {
    configs_[index] = *config;
  }
}

bool AutomationConfigStorage::remove_config(uint8_t index) {
  if (index < configs_.size()) {
    configs_.erase(configs_.begin() + index);
    return true;
  }
  return false;
}

AutomationConfig *AutomationConfigStorage::get_config(uint8_t index) {
  if (index < configs_.size()) {
    return &(configs_[index]);
  }
  return nullptr;
}

}  // namespace automations
}  // namespace esphome
