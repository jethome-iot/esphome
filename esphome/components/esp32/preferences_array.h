#pragma once
#ifdef USE_ESP32
#include "esp32_custom_preferences.h"

namespace esphome {
namespace esp32 {

namespace pref_array {
extern const char *TAG;
}

template<typename RecordType> class ESP32PreferencesArray {
 public:
  ~ESP32PreferencesArray() {
    records_num_pref_.remove_backend();
    clear_records_data_cache();
  }

  void set_namespace(const char *name) { this->preference_.set_namespace(name); }

  bool init(bool restore_data = true) {
    bool res = this->preference_.open();
    if (!res)
      return false;

    this->records_num_pref_ = this->preference_.make_preference("counter");
    this->restore_records_count_();
    if (restore_data)
      this->restore_records_data();

    return true;
  }

  void restore_records_data() {
    if (restored_state_)
      return;

    for (uint32_t i = 0; i < this->records_num_; i++) {
      ESPPreferenceObject record_perf = this->preference_.make_preference(str_snprintf("%d", 5, i));
      ESPPreferenceObjectManage perf_manage(record_perf);
      restore_record_data_(record_perf);
    }
    this->restored_state_ = true;
  }

  RecordType *make_record(RecordType &record) {
    if (!this->restored_state_)
      return nullptr;

    // Try to find nullptr
    for (uint32_t i = 0; i < this->records_num_; i++) {
      RecordType *internal_record = this->records_[i];
      if (internal_record == nullptr) {
        internal_record = new RecordType;
        *internal_record = record;
        this->records_[i] = internal_record;
        write_index_(i, internal_record);
        sync();
        return internal_record;
      }
    }

    // If no free place - extend
    RecordType *internal_record = new RecordType;
    *internal_record = record;
    this->records_.push_back(internal_record);

    write_index_(this->records_num_++, internal_record);
    this->records_num_pref_.save(&this->records_num_);

    sync();
    return internal_record;
  }

  uint32_t get_size() { return this->records_num_; }

  std::vector<RecordType *> &records() { return this->records_; }

  void sync() { this->preference_.sync(); }

  bool clear_index(uint32_t index) {
    if (index >= this->records_.size()) {
      return false;
    }

    if (this->records_[index]) {
      delete this->records_[index];
      this->records_[index] = nullptr;

      ESPPreferenceObject record_perf = this->preference_.make_preference(str_snprintf("%d", 5, index));
      ESPPreferenceObjectManage perf_manage(record_perf);
      record_perf.remove();
      this->sync();
    }
    return true;
  }

  void clear_all() {
    clear_records_data_cache();
    preference_.reset();
    this->records_num_ = 0;
    this->restored_state_ = true;
  }

 protected:
  void clear_records_data_cache() {
    for (auto *record : this->records_) {
      delete record;
    }
    this->records_.clear();
    this->restored_state_ = false;
  }

  bool write_index_(uint32_t index, RecordType *record) {
    if (index >= this->records_.size() || record == nullptr) {
      return false;
    }

    ESPPreferenceObject record_perf = this->preference_.make_preference(str_snprintf("%d", 5, index));
    ESPPreferenceObjectManage perf_manage(record_perf);
    record_perf.save(record);

    return true;
  }

  void restore_records_count_() {
    if (this->records_num_pref_.load(&this->records_num_)) {
      ESP_LOGD(pref_array::TAG, "Successfully restored records count from namespace %s - %d",
               this->preference_.get_namespace(), this->records_num_);
    } else {
      ESP_LOGW(pref_array::TAG, "No stored records count found");
    }
  }

  bool restore_record_data_(ESPPreferenceObject &obj) {
    RecordType *record = new RecordType;
    if (obj.load(record)) {
      records_.push_back(record);
      return true;
    }
    delete record;
    records_.push_back(nullptr);
    return false;
  }

  ESP32CustomPreferences preference_;

  uint32_t records_num_{0};
  ESPPreferenceObject records_num_pref_;

  std::vector<RecordType *> records_;

  bool restored_state_ = false;
};

template<typename RecordType> class ESP32PreferencesArrayKey : public ESP32PreferencesArray<RecordType> {
 public:
  RecordType *make_record(RecordType &record) {
    if (!this->restored_state_)
      return nullptr;

    // Try to find existing id
    for (uint32_t i = 0; i < this->records_num_; i++) {
      RecordType *internal_record = this->records_[i];
      if (record.key() == internal_record->key()) {
        *internal_record = record;
        write_index_(i, internal_record);
        sync();
        return internal_record;
      }
    }
    return ESP32PreferencesArray<RecordType>::make_record(record);
  }
};

}  // namespace esp32
}  // namespace esphome

#endif
