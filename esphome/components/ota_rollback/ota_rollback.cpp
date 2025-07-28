#include "ota_rollback.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ota_rollback {
static const char *const TAG = "ota_rollback";

void OTARollback::setup() {
  std::unique_ptr<ota::OTABackend> backend = ota::make_ota_backend();
  backend->mark_app_valid();
  this->rollback_available_ = backend->is_rollback_available();
}

void OTARollback::dump_config() { ESP_LOGCONFIG(TAG, this->rollback_available_ ? "available" : "unavailable"); }

bool OTARollback::is_available() { return this->rollback_available_; }

void OTARollback::rollback() {
  if (!this->rollback_available_)
    return;
  ESP_LOGI(TAG, "rollback process");
  std::unique_ptr<ota::OTABackend> backend = ota::make_ota_backend();
  backend->make_rollback();
}

}  // namespace ota_rollback
}  // namespace esphome
