#pragma once
#include "preferences.h"

namespace esphome {

template<typename RecordType> class ESPPreferencesArray {
 public:
  ~ESPPreferencesArray() {
    records_num_pref_.remove_backend();
    clear_records_data_cache_();
  }

  void set_base_hash(uint32_t hash) { this->base_hash_ = hash; }

  virtual ESPPreferenceObject make_counter_pref() {
    return global_preferences->make_preference<uint32_t>(this->base_hash_, true);
  }

  virtual ESPPreferenceObject make_index_pref(uint32_t index) {
    return global_preferences->make_preference<RecordType>(this->base_hash_ + index + 1, true);
  }

  bool init(bool restore_data = true) {
    this->records_num_pref_ = make_counter_pref();
    this->restore_records_count_();
    if (restore_data)
      this->restore_records_data();

    return true;
  }

  void restore_records_data() {
    if (restored_state_)
      return;

    for (uint32_t i = 0; i < this->records_num_; i++) {
      ESPPreferenceObject record_perf = make_index_pref(i);
      ESPPreferenceObjectManage perf_manage(record_perf);
      this->restore_record_data_(record_perf);
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
        this->write_index_(i, internal_record);
        this->sync();
        return internal_record;
      }
    }

    // If no free place - extend
    RecordType *internal_record = new RecordType;
    *internal_record = record;
    this->records_.push_back(internal_record);

    this->write_index_(this->records_num_++, internal_record);
    this->records_num_pref_.save(&this->records_num_);

    this->sync();
    return internal_record;
  }

  uint32_t get_size() { return this->records_num_; }

  std::vector<RecordType *> &records() { return this->records_; }

  virtual void sync() { global_preferences->sync(); }

  bool clear_index(uint32_t index) {
    if (index >= this->records_.size()) {
      return false;
    }

    if (this->records_[index]) {
      delete this->records_[index];
      this->records_[index] = nullptr;

      ESPPreferenceObject record_perf = make_index_pref(index);
      ESPPreferenceObjectManage perf_manage(record_perf);
      record_perf.remove();
      this->sync();
    }
    return true;
  }

  void clear_all() {
    clear_records_data_cache_();
    this->records_num_ = 0;
    this->records_num_pref_.save(&this->records_num_);
    this->restored_state_ = true;
    this->sync();
  }

 protected:
  void clear_records_data_cache_() {
    for (auto *record : this->records_) {
      delete record;
    }
    this->records_.clear();
    this->restored_state_ = false;
  }

  void write_index_(uint32_t index, RecordType *record) {
    if (record == nullptr) {
      return;
    }

    ESPPreferenceObject record_perf = make_index_pref(index);
    ESPPreferenceObjectManage perf_manage(record_perf);
    record_perf.save(record);
  }

  void restore_records_count_() { this->records_num_pref_.load(&this->records_num_); }

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

  uint32_t records_num_{0};
  ESPPreferenceObject records_num_pref_;
  std::vector<RecordType *> records_;
  uint32_t base_hash_;
  bool restored_state_ = false;
};

template<typename RecordType> class ESPPreferencesArrayKey : public ESPPreferencesArray<RecordType> {
 public:
  RecordType *make_record(RecordType &record) {
    if (!this->restored_state_)
      return nullptr;

    // Try to find existing id
    for (uint32_t i = 0; i < this->records_num_; i++) {
      RecordType *internal_record = this->records_[i];
      if (internal_record != nullptr && record.key() == internal_record->key()) {
        *internal_record = record;
        this->write_index_(i, internal_record);
        this->sync();
        return internal_record;
      }
    }
    return ESPPreferencesArray<RecordType>::make_record(record);
  }

  bool find_record_by_key(RecordType &record) {
    for (uint32_t i = 0; i < this->records_num_; i++) {
      RecordType *internal_record = this->records_[i];
      if (internal_record != nullptr && record.key() == internal_record->key()) {
        record = *internal_record;
        return true;
      }
    }
    return false;
  }
};

}  // namespace esphome
