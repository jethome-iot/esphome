#pragma once

#include "esphome/core/entity_types.h"
#include "esphome/core/entity_base.h"
#include "esphome/core/application.h"

#include <vector>

namespace esphome {
namespace user_names {

struct UserNamesRecord {
  uint32_t key;
  char name[33];
  EntityType type;
};

// Class for setting user names for entities
class UserNamesComponent : public Component {
 public:
  void setup() override;

  // setup should be called before api connected
  float get_setup_priority() const override { return setup_priority::HARDWARE_LATE; }

  void dump_config() override;

  uint16_t record_size() { return this->records_.size(); }

  UserNamesRecord *record(uint16_t index) {
    if (index >= this->sensors_.size())
      return nullptr;
    return this->record_[index];
  }

  void add_record(const EntityBase *entity, const char *name);

 protected:
  void restore_records_count_();
  bool restore_address_data_(ESPPreferenceObject &obj);

  uint16_t saved_records_num_ = 0;
  std::vector<UserNamesRecord *> records_;

  ESPPreferenceObject records_count_pref_;
  std::vector<ESPPreferenceObject> records_pref_;
};

}  // namespace user_names
}  // namespace esphome
