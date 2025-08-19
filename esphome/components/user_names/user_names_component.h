#pragma once

#include "esphome/core/entity_types.h"
#include "esphome/core/entity_base.h"
#include "esphome/core/application.h"

#ifdef USE_ESP32
#include "esphome/components/esp32/esp32_preferences_array.h"
template<typename T> using PreferenceArrayType = esphome::esp32::ESP32PreferencesArray<T>;
#else
#include "esphome/core/preferences_array.h"
template<typename T> using PreferenceArrayType = esphome::ESPPreferencesArray<T>;
#endif

namespace esphome {
namespace user_names {

struct UserNamesRecord {
  static const size_t MAX_NAME_SIZE = 32;
  uint32_t key = 0;
  char name[MAX_NAME_SIZE + 1] = {0};
  EntityType type = EntityType::NONE;

  void fill(EntityBase *entity, const char *name) {
    if (entity == nullptr || name == nullptr)
      return;
    this->type = entity->type();
    this->key = entity->get_object_id_hash();
    strncpy(this->name, name, MAX_NAME_SIZE);
  }
};

// Class for setting user names for entities
class UserNamesComponent : public Component {
 public:
  UserNamesComponent();

  void setup() override;

  // setup should be called before api connected
  float get_setup_priority() const override { return setup_priority::DATA; }

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
  PreferenceArrayType<UserNamesRecord> preference_;
};

}  // namespace user_names

extern user_names::UserNamesComponent *global_user_names;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

}  // namespace esphome
