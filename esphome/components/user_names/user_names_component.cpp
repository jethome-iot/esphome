#include "user_names_component.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <cstring>

namespace esphome {
user_names::UserNamesComponent *global_user_names =
    nullptr;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
namespace user_names {

static const char *const TAG = "user.names.component";

UserNamesComponent::UserNamesComponent() { global_user_names = this; }

void UserNamesComponent::setup() {
  // Restore records count
  this->base_hash_ = fnv1_hash(std::string("_user_name_component"));
  this->records_num_pref_ = global_preferences->make_preference<uint16_t>(this->base_hash_, true);
  this->restore_records_count_();

  // Create ESPPreferenceObjects
  for (uint16_t i = 0; i < this->records_num_; i++) {
    ESPPreferenceObject record_perf =
        global_preferences->make_preference<UserNamesRecord>(this->base_hash_ + i + 1, true);
    this->records_pref_.push_back(record_perf);
    restore_record_data_(record_perf);
  }

  for (uint16_t i = 0; i < this->records_num_; i++) {
    const UserNamesRecord *record = this->records_[i];
    if (record == nullptr)
      continue;
    EntityBase *entity = App.get_entity_by_key(record->type, record->key, false);
    if (entity)
      entity->set_name(record->name);
  }
}

void UserNamesComponent::make_record(EntityBase *entity, const char *name) {
  if (entity == nullptr || name == nullptr)
    return;

  // Try to find existing id
  for (uint16_t i = 0; i < this->records_num_; i++) {
    UserNamesRecord *record = this->records_[i];
    if (entity->get_object_id_hash() == record->key) {
      record->fill(entity, name);
      entity->set_name(record->name);
      records_pref_[i].save(record);
      global_preferences->sync();
      return;
    }
  }

  // Try to find nullptr
  for (uint16_t i = 0; i < this->records_num_; i++) {
    UserNamesRecord *record = this->records_[i];
    if (record == nullptr) {
      UserNamesRecord *record = new UserNamesRecord;
      record->fill(entity, name);
      entity->set_name(record->name);
      records_pref_[i].save(record);
      global_preferences->sync();
      return;
    }
  }

  // If no free place - extend
  ESPPreferenceObject record_perf =
      global_preferences->make_preference<UserNamesRecord>(this->base_hash_ + this->records_num_ + 1, true);
  this->records_pref_.push_back(record_perf);

  UserNamesRecord *record = new UserNamesRecord;
  record->fill(entity, name);
  entity->set_name(record->name);
  this->records_.push_back(record);

  this->records_pref_[this->records_num_++].save(record);
  this->records_num_pref_.save(&this->records_num_);

  global_preferences->sync();
}

void UserNamesComponent::reset_all() {
  this->records_num_ = 0;
  this->records_num_pref_.save(&this->records_num_);

  global_preferences->sync();
}

void UserNamesComponent::dump_config() { ESP_LOGCONFIG(TAG, "find %d records", this->records_num_); }

bool UserNamesComponent::restore_record_data_(ESPPreferenceObject &obj) {
  UserNamesRecord *record = new UserNamesRecord;
  if (obj.load(record)) {
    ESP_LOGD(TAG, "Loaded name %s from memory", record->name);
    records_.push_back(record);
    return true;
  }
  delete record;
  records_.push_back(nullptr);
  return false;
}

void UserNamesComponent::restore_records_count_() {
  if (this->records_num_pref_.load(&this->records_num_)) {
    ESP_LOGD(TAG, "Successfully restored records count from memory - %d", this->records_num_);
  } else {
    ESP_LOGW(TAG, "No stored records count found");
  }
}

}  // namespace user_names
}  // namespace esphome
