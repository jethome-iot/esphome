#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/core/automation.h"
#include <vector>
#include "automation_config.h"

namespace esphome {
namespace automations {

// Datablock struct
struct JsonData {
  static constexpr size_t MAX_DATA_SIZE = 4096;
  char data[MAX_DATA_SIZE];
  size_t size;
};

// Class for loading Automation config from preferences
class AutomationStorage : public Component {
 public:
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
  AutomationConfigStorage config_storage_;
  bool changed_{false};  // If automations aren't consistent with configs
};

}  // namespace automations

extern automations::AutomationStorage
    *global_automation_storage;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

}  // namespace esphome
