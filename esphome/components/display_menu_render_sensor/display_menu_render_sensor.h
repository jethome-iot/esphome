#pragma once

#include "esphome/components/display_menu_render_base/display_menu_render_base.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace display_menu_render_sensor {

using namespace display_menu_base;
using namespace display_menu_render_base;

class SensorMenuRender : public MenuRenderInterface {
 public:
  SensorMenuRender() : MenuRenderInterface(EntityType::SENSOR) {}
  size_t render_entity(MenuItemMenu *menu, EntityBase *entity) override;
  static auto get_render_lambda(sensor::Sensor *sensor_obj, bool with_name) {
    auto lambda = [=](const display_menu_base::MenuItem *it) -> std::string {
      char buf[50];
      char format_buf[10];
      int symb_nums = 0;
      if (with_name)
        symb_nums += sprintf(buf, "%s: ", sensor_obj->get_name().c_str());
      sprintf(format_buf, "%%0.%df", static_cast<int>(sensor_obj->get_accuracy_decimals()));

      if (sensor_obj->has_state())
        sprintf(buf + symb_nums, format_buf, sensor_obj->state);
      else
        sprintf(buf + symb_nums, "Nan");
      return buf;
    };
    return lambda;
  }
};

}  // namespace display_menu_render_sensor
}  // namespace esphome
