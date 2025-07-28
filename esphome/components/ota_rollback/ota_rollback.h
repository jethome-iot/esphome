#pragma once

#include "esphome/components/ota/ota_backend.h"

namespace esphome {
namespace ota {

class OTARollback : public Component {
 public:
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void setup() override;

  bool is_available();
  void rollback();

 protected:
  bool rollback_available_;
};

}  // namespace ota
}  // namespace esphome
