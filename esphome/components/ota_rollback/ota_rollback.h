#pragma once

#include "esphome/components/ota/ota_backend.h"

namespace esphome {
namespace ota {

class OTARollback : public Component {
 public:
  float get_setup_priority() const { return setup_priority::AFTER_WIFI; }

  void setup() override {
    std::unique_ptr<ota::OTABackend> backend = make_ota_backend();
    backend->mark_app_valid();
  }

  bool is_available() {
    std::unique_ptr<ota::OTABackend> backend = make_ota_backend();
    return backend->is_rollback_available();
  }

  void rollback() {
    std::unique_ptr<ota::OTABackend> backend = make_ota_backend();
    backend->make_rollback();
  };
};

}  // namespace ota
}  // namespace esphome
