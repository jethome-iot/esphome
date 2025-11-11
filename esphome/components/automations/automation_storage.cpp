#include "automation_storage.h"
#include "automation_factory.h"
#include "esphome/components/time/real_time_clock.h"
#include <esp_log.h>

namespace esphome {

automations::AutomationStorage
    *global_automation_storage;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

namespace automations {

static const char *const TAG = "automation_storage";

// Helper template function to load JSON data
template<size_t N>
static bool load_json_data_impl(ESPPreferenceObject &pref, char *buffer, size_t buffer_capacity, size_t &actual_size) {
  JsonData<N> data;
  if (!pref.load(&data)) {
    return false;
  }
  if (data.size > buffer_capacity) {
    ESP_LOGE(TAG, "Loaded data size (%d) exceeds buffer capacity (%d)", data.size, buffer_capacity);
    return false;
  }
  memcpy(buffer, data.data, data.size);
  actual_size = data.size;
  return true;
}

// Helper template function to save JSON data
template<size_t N> static bool save_json_data_impl(ESPPreferenceObject &pref, const char *buffer, size_t size) {
  if (size > N) {
    ESP_LOGE(TAG, "Data size (%d) exceeds template buffer size (%d)", size, N);
    return false;
  }
  JsonData<N> data;
  memcpy(data.data, buffer, size);
  data.size = size;
  return pref.save(&data);
}

// Create preference object based on buffer size
ESPPreferenceObject make_json_preference(size_t buffer_size, uint32_t hash) {
  switch (buffer_size) {
    case 1024:
      return global_preferences->make_preference<JsonData<1024>>(hash, true);
    case 2048:
      return global_preferences->make_preference<JsonData<2048>>(hash, true);
    case 3072:
      return global_preferences->make_preference<JsonData<3072>>(hash, true);
    case 4096:
      return global_preferences->make_preference<JsonData<4096>>(hash, true);
    case 5120:
      return global_preferences->make_preference<JsonData<5120>>(hash, true);
    case 6144:
      return global_preferences->make_preference<JsonData<6144>>(hash, true);
    case 7168:
      return global_preferences->make_preference<JsonData<7168>>(hash, true);
    case 8192:
      return global_preferences->make_preference<JsonData<8192>>(hash, true);
    case 9216:
      return global_preferences->make_preference<JsonData<9216>>(hash, true);
    case 10240:
      return global_preferences->make_preference<JsonData<10240>>(hash, true);
    case 11264:
      return global_preferences->make_preference<JsonData<11264>>(hash, true);
    case 12288:
      return global_preferences->make_preference<JsonData<12288>>(hash, true);
    case 13312:
      return global_preferences->make_preference<JsonData<13312>>(hash, true);
    case 14336:
      return global_preferences->make_preference<JsonData<14336>>(hash, true);
    case 15360:
      return global_preferences->make_preference<JsonData<15360>>(hash, true);
    case 16384:
      return global_preferences->make_preference<JsonData<16384>>(hash, true);
    default:
      ESP_LOGW(TAG, "Invalid buffer size %d, using default 4096", buffer_size);
      return global_preferences->make_preference<JsonData<4096>>(hash, true);
  }
}

// Load JSON with runtime buffer size selection
bool load_json_with_size(size_t buffer_size, ESPPreferenceObject &pref, char *buffer, size_t capacity, size_t &size) {
  switch (buffer_size) {
    case 1024:
      return load_json_data_impl<1024>(pref, buffer, capacity, size);
    case 2048:
      return load_json_data_impl<2048>(pref, buffer, capacity, size);
    case 3072:
      return load_json_data_impl<3072>(pref, buffer, capacity, size);
    case 4096:
      return load_json_data_impl<4096>(pref, buffer, capacity, size);
    case 5120:
      return load_json_data_impl<5120>(pref, buffer, capacity, size);
    case 6144:
      return load_json_data_impl<6144>(pref, buffer, capacity, size);
    case 7168:
      return load_json_data_impl<7168>(pref, buffer, capacity, size);
    case 8192:
      return load_json_data_impl<8192>(pref, buffer, capacity, size);
    case 9216:
      return load_json_data_impl<9216>(pref, buffer, capacity, size);
    case 10240:
      return load_json_data_impl<10240>(pref, buffer, capacity, size);
    case 11264:
      return load_json_data_impl<11264>(pref, buffer, capacity, size);
    case 12288:
      return load_json_data_impl<12288>(pref, buffer, capacity, size);
    case 13312:
      return load_json_data_impl<13312>(pref, buffer, capacity, size);
    case 14336:
      return load_json_data_impl<14336>(pref, buffer, capacity, size);
    case 15360:
      return load_json_data_impl<15360>(pref, buffer, capacity, size);
    case 16384:
      return load_json_data_impl<16384>(pref, buffer, capacity, size);
    default:
      return load_json_data_impl<4096>(pref, buffer, capacity, size);
  }
}

// Save JSON with runtime buffer size selection
bool save_json_with_size(size_t buffer_size, ESPPreferenceObject &pref, const char *buffer, size_t size) {
  switch (buffer_size) {
    case 1024:
      return save_json_data_impl<1024>(pref, buffer, size);
    case 2048:
      return save_json_data_impl<2048>(pref, buffer, size);
    case 3072:
      return save_json_data_impl<3072>(pref, buffer, size);
    case 4096:
      return save_json_data_impl<4096>(pref, buffer, size);
    case 5120:
      return save_json_data_impl<5120>(pref, buffer, size);
    case 6144:
      return save_json_data_impl<6144>(pref, buffer, size);
    case 7168:
      return save_json_data_impl<7168>(pref, buffer, size);
    case 8192:
      return save_json_data_impl<8192>(pref, buffer, size);
    case 9216:
      return save_json_data_impl<9216>(pref, buffer, size);
    case 10240:
      return save_json_data_impl<10240>(pref, buffer, size);
    case 11264:
      return save_json_data_impl<11264>(pref, buffer, size);
    case 12288:
      return save_json_data_impl<12288>(pref, buffer, size);
    case 13312:
      return save_json_data_impl<13312>(pref, buffer, size);
    case 14336:
      return save_json_data_impl<14336>(pref, buffer, size);
    case 15360:
      return save_json_data_impl<15360>(pref, buffer, size);
    case 16384:
      return save_json_data_impl<16384>(pref, buffer, size);
    default:
      return save_json_data_impl<4096>(pref, buffer, size);
  }
}

AutomationStorage::AutomationStorage() { global_automation_storage = this; }

void AutomationStorage::setup() {
  // Create buffer size preference
  this->buffer_size_pref_ =
      global_preferences->make_preference<size_t>(fnv1_hash(std::string("_automation_buf_size_")), true);

  // Load buffer size preference (or use default)
  size_t buffer_size = DEFAULT_BUFFER_SIZE;
  bool has_buffer_pref = this->buffer_size_pref_.load(&buffer_size);
  if (!has_buffer_pref) {
    ESP_LOGD(TAG, "No buffer size preference found, using default: %d", DEFAULT_BUFFER_SIZE);
    buffer_size = DEFAULT_BUFFER_SIZE;
  } else {
    ESP_LOGD(TAG, "Loaded buffer size preference: %d", buffer_size);
  }

  // Clamp and round buffer size to 1KB boundary
  buffer_size = clamp_buffer_size(buffer_size);
  this->current_buffer_size_ = buffer_size;

  ESP_LOGD(TAG, "Using buffer size: %d bytes", buffer_size);

  // Create preference object with appropriate template size
  uint32_t hash = fnv1_hash(std::string("_automation_storage_"));
  this->json_obj_ = make_json_preference(buffer_size, hash);

  // Allocate temporary buffer for loading
  std::unique_ptr<char[]> temp_buffer(new char[buffer_size]);
  size_t actual_size = 0;

  // Try to load data from preferences
  bool res = load_json_with_size(buffer_size, this->json_obj_, temp_buffer.get(), buffer_size, actual_size);

  if (!res) {
    ESP_LOGD(TAG, "No saved automations in storage");
    return;
  }

  // Parse JSON
  res = this->config_storage_.load_from_json(temp_buffer.get(), actual_size);
  if (!res) {
    ESP_LOGD(TAG, "Error parsing JSON");
    return;
  }

  // Set RTC for cron triggers before creating automations
  TriggerFactory<>::set_rtc(this->rtc_);

  automations_ = std::move(AutomationFactory<>::create_all_automations(this->config_storage_));
  ESP_LOGD(TAG, "Loaded %d automations (actual size: %d bytes, buffer: %d bytes)", automations_.size(), actual_size,
           buffer_size);
};

void AutomationStorage::dump_config() {
  ESP_LOGCONFIG(TAG, "Automation Storage:");
  ESP_LOGCONFIG(TAG, "  Total Automations: %d", this->config_storage_.size());
  ESP_LOGCONFIG(TAG, "  Buffer Size: %d bytes", this->current_buffer_size_);

  const auto &configs = this->config_storage_.get_all_configs();

  for (size_t i = 0; i < configs.size(); i++) {
    const auto &config = configs[i];

    ESP_LOGCONFIG(TAG, "");
    ESP_LOGCONFIG(TAG, "Automation [%d]: %s", i, config.name.c_str());
    ESP_LOGCONFIG(TAG, "  Enabled: %s", config.enabled ? "YES" : "NO");

    // Print trigger information
    ESP_LOGCONFIG(TAG, "  Triggers: %d", config.triggers.size());
    for (size_t j = 0; j < config.triggers.size(); j++) {
      ESP_LOGCONFIG(TAG, "  [%d]:", j);
      print_trigger_info(config.triggers[j], 4);
    }

    // Print condition information if present
    if (config.condition.is_valid()) {
      print_condition_info(config.condition, 2);
    }

    // Print actions
    ESP_LOGCONFIG(TAG, "  Actions: %d", config.actions.size());
    for (size_t j = 0; j < config.actions.size(); j++) {
      ESP_LOGCONFIG(TAG, "  [%d]:", j);
      print_action_info(config.actions[j], 4);
    }
  }
}

void AutomationStorage::save_configs() {
  // First, measure the actual JSON size needed
  size_t measured_size = 0;
  {
    // Create temporary document to measure size
    JsonDocument doc;
    JsonArray array = doc.to<JsonArray>();

    for (const auto &config : this->config_storage_.get_all_configs()) {
      JsonObject obj = array.add<JsonObject>();
      config.serialize(obj);
    }

    measured_size = measureJson(doc);
    ESP_LOGD(TAG, "Measured JSON size: %d bytes", measured_size);
  }

  if (measured_size == 0) {
    ESP_LOGW(TAG, "No data to save (empty config)");
    return;
  }

  // Calculate required buffer size with margin
  size_t needed_size = static_cast<size_t>(measured_size * BUFFER_SIZE_MARGIN);

  // Clamp and round to 1KB boundary
  needed_size = clamp_buffer_size(needed_size);

  // Check if measured size fits
  if (measured_size > MAX_BUFFER_SIZE) {
    ESP_LOGE(TAG, "JSON data (%d bytes) exceeds maximum allowed size (%d bytes)", measured_size, MAX_BUFFER_SIZE);
    return;
  }

  ESP_LOGD(TAG, "Calculated buffer size: %d bytes (measured: %d, margin: %.1f%%)", needed_size, measured_size,
           (BUFFER_SIZE_MARGIN - 1.0f) * 100.0f);

  // If buffer size changed, recreate preference object
  if (needed_size != this->current_buffer_size_) {
    ESP_LOGD(TAG, "Buffer size changed from %d to %d bytes, recreating preference", this->current_buffer_size_,
             needed_size);
    uint32_t hash = fnv1_hash(std::string("_automation_storage_"));
    this->json_obj_ = make_json_preference(needed_size, hash);
    this->current_buffer_size_ = needed_size;
  }

  // Allocate temporary buffer for serialization
  std::unique_ptr<char[]> temp_buffer(new char[needed_size]);

  // Serialize the data
  size_t actual_size = this->config_storage_.save_to_json(temp_buffer.get(), needed_size);

  if (actual_size == 0) {
    ESP_LOGE(TAG, "Failed to serialize config");
    return;
  }

  // Save using template-based function
  bool res = save_json_with_size(needed_size, this->json_obj_, temp_buffer.get(), actual_size);
  if (!res) {
    ESP_LOGE(TAG, "Failed to save config to preferences");
    return;
  }

  // Update buffer size preference
  this->buffer_size_pref_.save(&needed_size);

  global_preferences->sync();

  ESP_LOGD(TAG, "Config saved. Actual size: %d bytes, Buffer: %d bytes", actual_size, needed_size);
  ESP_LOGV(TAG, "Saved data: %s", temp_buffer.get());

  this->changed_ = true;
}

void AutomationStorage::set_enable_automation(uint32_t index, bool enable) {
  if (index < this->automations_.size()) {
    if (!enable)
      this->automations_[index]->stop();
    this->automations_[index]->set_enabled(enable);
  }
}

void AutomationStorage::print_trigger_info(const TriggerConfig &trigger, int indent) {
  std::string indent_str(indent, ' ');

  switch (trigger.source) {
    case SourceTrigger::Input:
      ESP_LOGCONFIG(TAG, "%sTrigger: Input", indent_str.c_str());
      ESP_LOGCONFIG(TAG, "%s  Type: %s", indent_str.c_str(),
                    EnumUtils::input_trigger_type_to_string(trigger.params.input.type));
      ESP_LOGCONFIG(TAG, "%s  Sensor ID: 0x%08X", indent_str.c_str(), trigger.params.input.input_id);
      break;

    case SourceTrigger::Temperature:
      ESP_LOGCONFIG(TAG, "%sTrigger: Temperature", indent_str.c_str());
      ESP_LOGCONFIG(TAG, "%s  Type: %s", indent_str.c_str(),
                    EnumUtils::temperature_trigger_type_to_string(trigger.params.temperature.type));
      ESP_LOGCONFIG(TAG, "%s  Sensor ID: 0x%08X", indent_str.c_str(), trigger.params.temperature.sensor_id);

      if (trigger.params.temperature.type == TypesTemperatureTrigger::Below ||
          trigger.params.temperature.type == TypesTemperatureTrigger::Above) {
        ESP_LOGCONFIG(TAG, "%s  Threshold: %.2f", indent_str.c_str(), trigger.params.temperature.threshold);
      } else if (trigger.params.temperature.type == TypesTemperatureTrigger::Range) {
        ESP_LOGCONFIG(TAG, "%s  Min Threshold: %.2f", indent_str.c_str(), trigger.params.temperature.min_threshold);
        ESP_LOGCONFIG(TAG, "%s  Max Threshold: %.2f", indent_str.c_str(), trigger.params.temperature.max_threshold);
      }
      break;

    case SourceTrigger::Cron: {
      ESP_LOGCONFIG(TAG, "%sTrigger: Cron", indent_str.c_str());

      // Helper lambda to format cron field
      auto format_cron_field = [](const std::vector<uint8_t> &field) -> std::string {
        if (field.empty())
          return "*";
        std::string result;
        for (size_t i = 0; i < field.size(); i++) {
          if (i > 0)
            result += ",";
          result += std::to_string(field[i]);
        }
        return result;
      };

      ESP_LOGCONFIG(
          TAG, "%s  Schedule: %s %s %s %s %s %s", indent_str.c_str(), format_cron_field(trigger.cron_seconds).c_str(),
          format_cron_field(trigger.cron_minutes).c_str(), format_cron_field(trigger.cron_hours).c_str(),
          format_cron_field(trigger.cron_days_of_month).c_str(), format_cron_field(trigger.cron_months).c_str(),
          format_cron_field(trigger.cron_days_of_week).c_str());
      break;
    }
    case SourceTrigger::Startup:
      ESP_LOGCONFIG(TAG, "%sTrigger: Startup", indent_str.c_str());
      break;

    case SourceTrigger::None:
      ESP_LOGCONFIG(TAG, "%sTrigger: None", indent_str.c_str());
      break;

    default:
      ESP_LOGCONFIG(TAG, "%sTrigger: Unknown", indent_str.c_str());
      break;
  }
}

void AutomationStorage::print_condition_info(const ConditionConfig &condition, int indent) {
  std::string indent_str(indent, ' ');

  if (!condition.is_valid()) {
    return;
  }

  switch (condition.type) {
    case ConditionType::Input:
      ESP_LOGCONFIG(TAG, "%sCondition: Input", indent_str.c_str());
      ESP_LOGCONFIG(TAG, "%s  Sensor ID: 0x%08X", indent_str.c_str(), condition.sensor_id);
      ESP_LOGCONFIG(TAG, "%s  Expected State: %s", indent_str.c_str(),
                    EnumUtils::input_condition_state_to_string(condition.state));
      break;

    case ConditionType::Temperature:
      ESP_LOGCONFIG(TAG, "%sCondition: Temperature", indent_str.c_str());
      ESP_LOGCONFIG(TAG, "%s  Type: %s", indent_str.c_str(),
                    EnumUtils::temperature_condition_type_to_string(condition.temperature_type));
      ESP_LOGCONFIG(TAG, "%s  Sensor ID: 0x%08X", indent_str.c_str(), condition.sensor_id);

      if (condition.temperature_type == TypesTemperatureCondition::Below ||
          condition.temperature_type == TypesTemperatureCondition::Above) {
        ESP_LOGCONFIG(TAG, "%s  Threshold: %.2f", indent_str.c_str(), condition.threshold);
      } else if (condition.temperature_type == TypesTemperatureCondition::Range) {
        ESP_LOGCONFIG(TAG, "%s  Min Threshold: %.2f", indent_str.c_str(), condition.min_threshold);
        ESP_LOGCONFIG(TAG, "%s  Max Threshold: %.2f", indent_str.c_str(), condition.max_threshold);
      }
      break;

    case ConditionType::And:
    case ConditionType::Or:
    case ConditionType::Xor:
      ESP_LOGCONFIG(TAG, "%sCondition: %s", indent_str.c_str(), EnumUtils::condition_type_to_string(condition.type));
      ESP_LOGCONFIG(TAG, "%s  Sub-conditions: %d", indent_str.c_str(), condition.sub_conditions.size());

      for (size_t i = 0; i < condition.sub_conditions.size(); i++) {
        ESP_LOGCONFIG(TAG, "%s  [%d]:", indent_str.c_str(), i);
        print_condition_info(condition.sub_conditions[i], indent + 4);
      }
      break;

    default:
      ESP_LOGCONFIG(TAG, "%sCondition: Unknown", indent_str.c_str());
      break;
  }
}

void AutomationStorage::print_action_info(const ActionConfig &action, int indent) {
  std::string indent_str(indent, ' ');

  switch (action.source) {
    case SourceAction::Switch:
      ESP_LOGCONFIG(TAG, "%sAction: Switch", indent_str.c_str());
      ESP_LOGCONFIG(TAG, "%s  Type: %s", indent_str.c_str(),
                    EnumUtils::switch_action_type_to_string(action.params.switch_action.type));
      ESP_LOGCONFIG(TAG, "%s  Switch ID: 0x%08X", indent_str.c_str(), action.params.switch_action.switch_id);
      break;

    case SourceAction::Delay:
      ESP_LOGCONFIG(TAG, "%sAction: Delay", indent_str.c_str());
      ESP_LOGCONFIG(TAG, "%s  Duration: %d seconds", indent_str.c_str(), action.params.delay.delay_s);
      break;

    case SourceAction::None:
      ESP_LOGCONFIG(TAG, "%sAction: None", indent_str.c_str());
      break;

    default:
      ESP_LOGCONFIG(TAG, "%sAction: Unknown", indent_str.c_str());
      break;
  }
}

}  // namespace automations
}  // namespace esphome
