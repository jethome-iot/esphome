#pragma once
#include "esphome/components/switch/switch.h"

namespace esphome {
namespace simple {
class SimpleSwitch : public switch_::Switch {
 protected:
  bool assumed_state() override { return this->assumed_state_; }

  void write_state(bool state) override { this->publish_state(state); }

  bool assumed_state_{false};
};

}  // namespace simple
}  // namespace esphome
