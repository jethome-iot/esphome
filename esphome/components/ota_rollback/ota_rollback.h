#pragma once

#include "esphome/components/ota/ota_backend.h"

namespace esphome {
namespace ota_rollback {

class OTARollback : public Component {
 public:
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void setup() override;
  void dump_config() override;

  bool is_available();
  void rollback();

 protected:
  bool rollback_available_;
};

}  // namespace ota_rollback
}  // namespace esphome
