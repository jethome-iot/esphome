#pragma once
#include <stdint.h>

namespace esphome {

enum class EntityType : uint8_t {
  NONE,
  BINARY_SENSOR,
  SWITCH,
  BUTTON,
  SENSOR,
  TEXT_SENSOR,
  FAN,
  COVER,
  LIGHT,
  CLIMATE,
  NUMBER,
  DATETIME_DATE,
  DATETIME_TIME,
  DATETIME_DATETIME,
  TEXT,
  SELECT,
  LOCK,
  VALVE,
  MEDIA_PLAYER,
  ALARM_CONTROL_PANEL,
  EVENT,
  UPDATE,
};

}  // namespace esphome
