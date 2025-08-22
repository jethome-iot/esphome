#include "user_names_component.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <cstring>

namespace esphome {
dynamic_entity_parameters::EntitySettingsKeeper
    *global_entity_settings_keeper =  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
    nullptr;
namespace dynamic_entity_settings {

EntitySettingsKeeper::EntitySettingsKeeper() { global_entity_settings_keeper = this; }

}  // namespace dynamic_entity_settings
}  // namespace esphome
