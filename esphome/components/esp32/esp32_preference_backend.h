#pragma once
#ifdef USE_ESP32

#include <string>
#include <memory>
#include <cstring>
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"

namespace esphome {
namespace esp32 {

struct NVSData {
  std::string key;
  std::unique_ptr<uint8_t[]> data;
  size_t len;

  void set_data(const uint8_t *src, size_t size) {
    data = std::make_unique<uint8_t[]>(size);
    memcpy(data.get(), src, size);
    len = size;
  }
};

class ESP32PreferenceBackend : public ESPPreferenceBackend {
 public:
  ESP32PreferenceBackend(std::vector<NVSData> &vec) : pending_save(vec) {}
  std::string key;
  uint32_t nvs_handle;
  std::vector<NVSData> &pending_save;
  bool save(const uint8_t *data, size_t len) override;
  bool load(uint8_t *data, size_t len) override;
  bool remove() override;
};

}  // namespace esp32
}  // namespace esphome

#endif
