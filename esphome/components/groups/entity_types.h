#pragma once
#include <stdint.h>

namespace esphome {
namespace groups {

enum class EntityType : uint8_t {
  NONE,
  SWITCH,
  SENSOR,
};

enum class EntitySubtype : uint8_t {
  NONE,
  DALLAS,
};

}  // namespace groups
}  // namespace esphome
