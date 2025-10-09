#pragma once
#include "esphome/core/automation.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace automations {

// Trigger that fires when temperature goes below a threshold
class TemperatureBelowTrigger : public Trigger<> {
 public:
  explicit TemperatureBelowTrigger(sensor::Sensor *sensor, float threshold)
      : sensor_(sensor), threshold_(threshold), was_above_(true) {
    sensor->add_on_state_callback([this](float state) { this->on_state_(state); });
  }

 protected:
  void on_state_(float state) {
    if (std::isnan(state))
      return;

    // Trigger when crossing from above to below threshold
    if (was_above_ && state < threshold_) {
      this->trigger();
      was_above_ = false;
    } else if (state >= threshold_) {
      was_above_ = true;
    }
  }

  sensor::Sensor *sensor_;
  float threshold_;
  bool was_above_;
};

// Trigger that fires when temperature goes above a threshold
class TemperatureAboveTrigger : public Trigger<> {
 public:
  explicit TemperatureAboveTrigger(sensor::Sensor *sensor, float threshold)
      : sensor_(sensor), threshold_(threshold), was_below_(true) {
    sensor->add_on_state_callback([this](float state) { this->on_state_(state); });
  }

 protected:
  void on_state_(float state) {
    if (std::isnan(state))
      return;

    // Trigger when crossing from below to above threshold
    if (was_below_ && state > threshold_) {
      this->trigger();
      was_below_ = false;
    } else if (state <= threshold_) {
      was_below_ = true;
    }
  }

  sensor::Sensor *sensor_;
  float threshold_;
  bool was_below_;
};

// Trigger that fires when temperature enters a range
class TemperatureRangeTrigger : public Trigger<> {
 public:
  explicit TemperatureRangeTrigger(sensor::Sensor *sensor, float min, float max)
      : sensor_(sensor), min_(min), max_(max), was_outside_(true) {
    sensor->add_on_state_callback([this](float state) { this->on_state_(state); });
  }

 protected:
  void on_state_(float state) {
    if (std::isnan(state))
      return;

    bool in_range = (state >= min_ && state <= max_);

    // Trigger when entering the range from outside
    if (was_outside_ && in_range) {
      this->trigger();
      was_outside_ = false;
    } else if (!in_range) {
      was_outside_ = true;
    }
  }

  sensor::Sensor *sensor_;
  float min_;
  float max_;
  bool was_outside_;
};

}  // namespace automations
}  // namespace esphome