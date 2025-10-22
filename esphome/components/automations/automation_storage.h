#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/core/automation.h"
#include <vector>
#include "automation_config.h"

namespace esphome {
namespace automations {

// Template-based JsonData with fixed buffer size per instantiation
// Buffer sizes in 1KB steps: 1024, 2048, 3072, ..., 16384
template<size_t BufferSize> struct JsonData {
  static constexpr size_t BUFFER_SIZE = BufferSize;
  char data[BufferSize];
  size_t size;  // Actual used size
};

// Helper functions for buffer size management
namespace {
// Round up to nearest 1KB multiple
inline size_t round_to_1kb(size_t size) { return ((size + 1023) / 1024) * 1024; }

// Clamp between 1KB and 16KB
inline size_t clamp_buffer_size(size_t size) {
  if (size < 1024)
    return 1024;
  if (size > 16384)
    return 16384;
  return round_to_1kb(size);
}
}  // namespace

// Class for loading Automation config from preferences
class AutomationStorage : public Component {
 public:
  static constexpr size_t DEFAULT_BUFFER_SIZE = 4096;
  static constexpr size_t MIN_BUFFER_SIZE = 1024;
  static constexpr size_t MAX_BUFFER_SIZE = 16384;
  static constexpr float BUFFER_SIZE_MARGIN = 1.2f;  // 20% margin above measured size

  AutomationStorage();

  void setup() override;

  float get_setup_priority() const override { return setup_priority::DATA - 1; }

  void dump_config() override{};

  void save_configs();

  AutomationConfigStorage &configs() { return config_storage_; }

  bool is_changed() { return changed_; }

  void set_enable_automation(uint32_t index, bool enable);

 protected:
  std::vector<std::unique_ptr<Automation<>>> automations_;
  ESPPreferenceObject json_obj_;
  ESPPreferenceObject buffer_size_pref_;
  AutomationConfigStorage config_storage_;
  size_t current_buffer_size_{DEFAULT_BUFFER_SIZE};
  bool changed_{false};  // If automations aren't consistent with configs
};

// Helper functions for template-based preference operations
ESPPreferenceObject make_json_preference(size_t buffer_size, uint32_t hash);
bool load_json_with_size(size_t buffer_size, ESPPreferenceObject &pref, char *buffer, size_t capacity, size_t &size);
bool save_json_with_size(size_t buffer_size, ESPPreferenceObject &pref, const char *buffer, size_t size);

}  // namespace automations

extern automations::AutomationStorage
    *global_automation_storage;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

}  // namespace esphome
