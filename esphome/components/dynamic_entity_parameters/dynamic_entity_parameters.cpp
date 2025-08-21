#include "user_names_component.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <cstring>

namespace esphome {
dynamic_entity_parameters::SettingsKeeper
    *global_settings_keeper =  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
    nullptr;
namespace user_names {

// static const char *const TAG = "user.names.component";

SettingsKeeper::SettingsKeeper() { global_settings_keeper = this; }

}  // namespace user_names
}  // namespace esphome
