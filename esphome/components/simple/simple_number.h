#pragma once
#include "esphome/components/number/number.h"

namespace esphome {
namespace simple {
class SimpleNumber : public number::Number {
 protected:
  void control(float value) override { this->publish_state(value); }
};

}  // namespace simple
}  // namespace esphome
