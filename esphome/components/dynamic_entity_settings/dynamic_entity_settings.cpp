#include "dynamic_entity_settings.h"
#include "esphome/core/helpers.h"
#include <nvs_flash.h>

namespace esphome {
dynamic_entity_settings::EntitySettingsKeeper
    *global_entity_settings_keeper =  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
    nullptr;
namespace dynamic_entity_settings {
static const char *const TAG = "dynamic.entity.params";

EntitySettingsKeeper::EntitySettingsKeeper() { global_entity_settings_keeper = this; }

void EntitySettingsKeeper::setup() {
  // TODO Logic for checking existing settings and conversion between them

  // Just apply first setting now
  for (auto &list : this->settings_list_) {
    if (list.empty())
      continue;
    SettingsBaseInterface *setting = list[0];

    if (!setting->check_available())
      continue;

    bool res = setting->init();
    if (!res)
      continue;
    res = setting->load();
    if (res)
      setting->apply();
  }
}

void EntitySettingsKeeper::dump_config() {
  ESP_LOGCONFIG(TAG, "Active settings store:");
  for (auto &list : this->settings_list_) {
    if (!list.empty()) {
      ESP_LOGCONFIG(TAG, " - %s, records: %d", list[0]->get_namespace(), list[0]->size());
    }
  }
}

SettingsBaseInterface *EntitySettingsKeeper::get_settings_preference(const char *name) {
  // Find only first element each array now
  for (auto &list : this->settings_list_) {
    if (list.empty())
      continue;
    if (strncmp(list[0]->get_namespace(), name, NVS_KEY_NAME_MAX_SIZE) == 0) {
      return list[0];
    }
  }
  return nullptr;
}

void EntitySettingsKeeper::reset_all() {
  for (auto &list : this->settings_list_) {
    for (auto *settings : list) {
      settings->reset();
    }
  }
}

}  // namespace dynamic_entity_settings
}  // namespace esphome
