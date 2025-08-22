#pragma once
#include "esphome/core/application.h"

#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif

namespace esphome {
namespace dynamic_entity_parameters {

#ifdef USE_SWITCH
struct SwitchParameters_ver1 {
  switch_::SwitchRestoreMode restore_mode;
  bool inverted;
};
#endif

}  // namespace dynamic_entity_parameters
}  // namespace esphome