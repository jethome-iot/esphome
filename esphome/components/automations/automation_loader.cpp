#pragma once

#include "automation_loader.h"
#include "automation_factory.h"
#include "automation_system.h"

namespace esphome {
namespace automations {

const char *TAG = "automation";

void AutomationLoader::setup() {
  // TODO it should be loaded from flash storage
  // Dynamic creation for test

  static const char *json_config = R"([
    {
      "id": "auto1",
      "name": "Inp1-react",
      "enabled": true,
      "trigger": {
        "source": "Input",
        "type": "press",
        "input_id": "68670887"
      },
      "actions": [
        {
          "source": "Switch",
          "type": "turn_on",
          "switch_id": "6ffef2f8"
        },
        {
          "source": "Delay",
          "delay_s": 5
        },
        {
          "source": "Switch",
          "type": "turn_off",
          "switch_id": "6ffef2f8"
        }
      ]
    }
  ])";

  if (!global_automation_storage.loadFromJson(json_config)) {
    ESP_LOGW(TAG, "Cannot load automations");
    return;
  }

  automations_ = std::move(AutomationFactory<>::createAllAutomations(global_automation_storage));
};

}  // namespace automations
}  // namespace esphome
