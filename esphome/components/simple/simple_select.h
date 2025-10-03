#pragma once
#include "esphome/components/select/select.h"

namespace esphome {
namespace simple {
class SimpleSelect : public select::Select {
 protected:
  void control(const std::string &value) override { this->publish_state(value); }
};

}  // namespace simple
}  // namespace esphome
