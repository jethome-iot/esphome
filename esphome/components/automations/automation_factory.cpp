#include "automation_factory.h"
#include "esphome/core/log.h"

namespace esphome {
namespace automations {

void log_trigger_not_found(const std::string &object_id) {
  ESP_LOGE("automation", "Cannot find trigger object_id %s", object_id.c_str());
}

void log_switch_not_found(const std::string &object_id) {
  ESP_LOGE("automation", "Cannot find switch with object_id %s", object_id.c_str());
}

void log_sensor_not_found(const std::string &object_id) {
  ESP_LOGE("automation", "Cannot find sensor with object_id %s", object_id.c_str());
}

void log_trigger_creation_error() { ESP_LOGE("automation", "Error create Trigger"); }

void log_action_creation_error() { ESP_LOGE("automation", "Error create Action"); }

}  // namespace automations
}  // namespace esphome
