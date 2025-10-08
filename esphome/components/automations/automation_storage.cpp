#pragma once

#include "automation_storage.h"
#include "automation_factory.h"

namespace esphome {

automations::AutomationStorage
    *global_automation_storage;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

namespace automations {

static const char *const TAG = "automation_storage";

AutomationStorage::AutomationStorage() { global_automation_storage = this; }

void AutomationStorage::setup() {
  this->json_obj_ = global_preferences->make_preference<JsonData>(fnv1_hash(std::string("_automation_storage_")), true);

  JsonData data;
  bool res = this->json_obj_.load(&data);
  if (!res) {
    ESP_LOGD(TAG, "No saved automations in storage");
    return;
  }

  res = this->config_storage_.load_from_json(data.data, JsonData::MAX_DATA_SIZE);

  if (!res) {
    ESP_LOGD(TAG, "Error json parsing");
  }

  automations_ = std::move(AutomationFactory<>::create_all_automations(this->config_storage_));
};

void AutomationStorage::save_configs() {
  JsonData data;
  data.size = this->config_storage_.save_to_json(data.data, data.MAX_DATA_SIZE);

  if (data.size == 0) {
    ESP_LOGE(TAG, "Config isn't saved");
    return;
  }

  this->json_obj_.save<JsonData>(&data);
  global_preferences->sync();
  ESP_LOGD(TAG, "Config is saved. Size %d", data.size);
  ESP_LOGV(TAG, "Saved data %s", data.data);

  this->changed_ = true;
}

void AutomationStorage::set_enable_automation(uint32_t index, bool enable) {
  if (index < this->automations_.size()) {
    if (!enable)
      this->automations_[index]->stop();
    this->automations_[index]->set_enabled(enable);
  }
}

}  // namespace automations
}  // namespace esphome
