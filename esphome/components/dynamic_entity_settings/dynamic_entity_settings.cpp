#include "dynamic_entity_settings.h"
#include "esphome/core/helpers.h"

namespace esphome {
dynamic_entity_settings::EntitySettingsKeeper
    *global_entity_settings_keeper =  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
    nullptr;
namespace dynamic_entity_settings {
const char *TAG = "dynamic.entity.params";

EntitySettingsKeeper::EntitySettingsKeeper() { global_entity_settings_keeper = this; }

}  // namespace dynamic_entity_settings
}  // namespace esphome
