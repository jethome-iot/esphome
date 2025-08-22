#include "user_names_component.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <cstring>

namespace esphome {
user_names::UserNamesComponent *global_user_names =  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
    nullptr;
namespace user_names {

static const char *const TAG = "user.names.component";

UserNamesComponent::UserNamesComponent() { global_user_names = this; }

void UserNamesComponent::setup() {
#ifdef USE_ESP32
  this->preference_.set_namespace("user_names");
#else
  this->preference_.set_base_hash(fnv1_hash(std::string("_user_name_component")));
#endif
  this->preference_.init();

  auto &records = this->preference_.records();
  for (size_t i = 0; i < records.size(); i++) {
    const UserNamesRecord *record = this->preference_.records()[i];
    if (record == nullptr)
      continue;
    EntityBase *entity = App.get_entity_by_key(record->type, record->entity_id, false);
    if (entity)
      entity->set_name(record->name);
  }
}

void UserNamesComponent::make_record(EntityBase *entity, const char *name) {
  if (entity == nullptr || name == nullptr)
    return;

  UserNamesRecord rec;
  rec.fill(entity, name);
  UserNamesRecord *int_rec = this->preference_.make_record(rec);

  if (int_rec != nullptr) {
    entity->set_name(int_rec->name);
  }
}

void UserNamesComponent::reset_all() { this->preference_.clear_all(); }

void UserNamesComponent::dump_config() { ESP_LOGCONFIG(TAG, "find %d records", this->preference_.get_size()); }

}  // namespace user_names
}  // namespace esphome
