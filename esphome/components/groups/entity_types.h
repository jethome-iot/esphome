#pragma once
#include <stdint.h>

namespace esphome {
namespace groups {

enum class EntityType : uint8_t {
  NONE,
  SWITCH,
  SENSOR,
};

}  // namespace groups
}  // namespace esphome
