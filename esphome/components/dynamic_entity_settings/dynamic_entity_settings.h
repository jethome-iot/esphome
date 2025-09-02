#pragma once
#ifdef USE_ESP32
#include <vector>
#include "esphome/core/component.h"
#include "esphome/components/esp32/esp32_preferences_array.h"

namespace esphome {
namespace dynamic_entity_settings {
template<typename T> using PreferenceArrayType = esphome::esp32::ESP32PreferencesArrayKey<T>;

// Interface for settings type
class SettingsBaseInterface {
 public:
  virtual void set_namespace(const char *) = 0;
  virtual const char *get_namespace() = 0;

  // Check that namespace is available
  virtual bool check_available() = 0;
  // Init storage
  virtual bool init() = 0;
  // Load to ram
  virtual bool load() = 0;
  // Make conversion to this version
  virtual bool make_conversion_from_last_version(SettingsBaseInterface *last) = 0;

  // Apply all loaded settings
  virtual void apply() = 0;

  // Records size
  virtual size_t size() = 0;
  // Clear all records
  virtual void reset() = 0;
};

// Class for setting parameters for entities
class EntitySettingsKeeper : public Component {
 public:
  EntitySettingsKeeper();

  void setup() override;

  // setup should be called before api connected
  float get_setup_priority() const override { return setup_priority::HARDWARE + 1; }

  void dump_config() override;

  SettingsBaseInterface *get_settings_preference(const char *name);

  void reset_all();

  void add_settings_list(std::vector<SettingsBaseInterface *> &&list) {
    this->settings_list_.push_back(std::move(list));
  }

 protected:
  std::vector<std::vector<SettingsBaseInterface *>> settings_list_;
};

}  // namespace dynamic_entity_settings

extern dynamic_entity_settings::EntitySettingsKeeper
    *global_entity_settings_keeper;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

}  // namespace esphome
#endif
