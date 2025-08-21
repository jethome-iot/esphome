#pragma once
#include "entity_parameters.h"

#ifdef USE_ESP32
#include "esphome/components/esp32/esp32_preferences_array.h"
template<typename T> using PreferenceArrayType = esphome::esp32::ESP32PreferencesArray<T>;
#else
#include "esphome/core/preferences_array.h"
template<typename T> using PreferenceArrayType = esphome::ESPPreferencesArray<T>;
#endif

// TODO Возможно стоит создать механизм подобно рендеру - имеем базовый класс интерфейса
// В основной класс они добавляются в формате указателей и затем один за одним выполняют определенную функцию
// Можно попробовать реализовать как лямбду при наличии новой версии настроечника она может без проблем считывать старые
// настройки и переносить их в новый неймспейс
//

namespace esphome {
namespace dynamic_entity_parameters {

const char *TAG = "dynamic.entity.params";

// Class for setting parameters for entities
class DynamicEntityParameters : public Component {
 public:
  DynamicEntityParameters();

  void setup() override;

  // setup should be called before api connected
  float get_setup_priority() const override { return setup_priority::HARDWARE + 1; }

  void dump_config() override;

  uint16_t record_size() { return this->preference_.get_size(); }

  UserNamesRecord *record(uint16_t index) {
    if (index >= this->preference_.get_size())
      return nullptr;

    return this->preference_.records()[index];
  }

  void make_record(EntityBase *entity, const char *name);

  void reset_all();

 protected:
#ifdef USE_SWITCH
  PreferenceArrayType<SwitchParameters_ver1> switch_preference_;
  void process_switch_parameters_() {}

#endif
};

}  // namespace dynamic_entity_parameters

// extern user_names::UserNamesComponent *global_user_names;  //
// NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

}  // namespace esphome
