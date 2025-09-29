#pragma once

#include "automation_loader.h"
#include "automation_factory.h"

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
        "type": 3,
        "input_id": "68670887"
      },
      "actions": [
        {
          "source": "Switch",
          "type": 1,
          "switch_id": "6ffef2f8"
        },
        {
          "source": "Delay",
          "delay_s": 5
        },
        {
          "source": "Switch",
          "type": 2,
          "switch_id": "6ffef2f8"
        }
      ]
    }
  ])";

  AutomationStorage storage;
  if (!storage.loadFromJson(json_config)) {
    ESP_LOGW(TAG, "Cannot load automations");
    return;
  }

  automations_ = std::move(AutomationFactory<>::createAllAutomations(storage));
};

}  // namespace automations
}  // namespace esphome
