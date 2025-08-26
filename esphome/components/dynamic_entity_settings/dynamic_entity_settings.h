#pragma once
#ifdef USE_ESP32
#include <vector>
#include "esphome/core/component.h"
#include "esphome/components/esp32/esp32_preferences_array.h"

// TODO Возможно стоит создать механизм подобно рендеру - имеем базовый класс интерфейса
// В основной класс они добавляются в формате указателей и затем один за одним выполняют определенную функцию
// Можно попробовать реализовать как лямбду при наличии новой версии настроечника она может без проблем считывать старые
// настройки и переносить их в новый неймспейс
//

namespace esphome {
namespace dynamic_entity_settings {

extern const char *TAG;
template<typename T> using PreferenceArrayType = esphome::esp32::ESP32PreferencesArray<T>;

class SettingsBase {
 public:
  virtual void set_namespace(const char *) = 0;
  virtual const char *get_namespace() = 0;

  // Check that namespace is available
  virtual bool check_available() = 0;
  virtual bool load() = 0;
  // Make conversion to this version
  virtual bool make_conversion_from_last_version(SettingsBase *last) = 0;
  virtual void apply() = 0;

  virtual void reset() = 0;
};

// Class for setting parameters for entities
class EntitySettingsKeeper : public Component {
 public:
  EntitySettingsKeeper();

  void setup() override {
    // TODO Logic for checking existing settings and conversion between them

    // Just apply current setting
    for (auto &list : this->settings_list_) {
      if (list.empty())
        continue;
      SettingsBase *setting = list[0];
      bool res = setting->load();
      if (res)
        setting->apply();
    }
  }

  // setup should be called before api connected
  float get_setup_priority() const override { return setup_priority::HARDWARE + 1; }

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "Active settings store:");
    for (auto &list : this->settings_list_) {
      if (!list.empty()) {
        ESP_LOGCONFIG(TAG, "  -%s", list[0]->get_namespace());
      }
    }
  }

  SettingsBase *get_settings_preferences(const char *name) {
    // Find only first element each array
    for (auto &list : this->settings_list_) {
      if (list.empty())
        continue;
      if (strncmp(list[0]->get_namespace(), name, 16) == 0) {
        return list[0];
      }
    }
    return nullptr;
  }

  void reset_all() {
    for (auto &list : this->settings_list_) {
      for (auto *settings : list) {
        settings->reset();
      }
    }
  }

  void add_settings_list(std::vector<SettingsBase *> list) { this->settings_list_.push_back(list); }

 protected:
  std::vector<std::vector<SettingsBase *>> settings_list_;
};

}  // namespace dynamic_entity_settings

extern dynamic_entity_settings::EntitySettingsKeeper
    *global_entity_settings_keeper;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

}  // namespace esphome
#endif
