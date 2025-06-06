#pragma once

#include "esphome/components/display_menu_base/display_menu_base.h"
#include "esphome/components/groups/entity_types.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/dallas_temp/dallas_temp.h"

namespace esphome {
namespace display_menu_renderers {

using namespace display_menu_base;

class SwitchMenuRender : public MenuRenderInterface {
 public:
  SwitchMenuRender() : MenuRenderInterface(groups::EntityType::SWITCH) {}
  size_t render_entity(MenuItemMenu *menu, const groups::EntityInfo &info) override;
};

class SensorMenuRender : public MenuRenderInterface {
 public:
  SensorMenuRender() : MenuRenderInterface(groups::EntityType::SENSOR) {}
  size_t render_entity(MenuItemMenu *menu, const groups::EntityInfo &info) override;
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

  static auto get_value_lambda(sensor::Sensor *sensor_obj) {
    auto lambda = [=](const display_menu_base::MenuItem *it) -> std::string {
      char buf[16];
      char format_buf[10];
      sprintf(format_buf, "%%0.%df", 2);
      if (sensor_obj->has_state())
        sprintf(buf, format_buf, sensor_obj->state);
      else
        sprintf(buf, "Nan");
      return buf;
    };
    return lambda;
  }
};

class DallasTempMenuRender : public MenuRenderInterface {
 public:
  DallasTempMenuRender() : MenuRenderInterface(groups::EntityType::SENSOR, groups::EntitySubtype::DALLAS) {}
  size_t render_entity(MenuItemMenu *menu, const groups::EntityInfo &info) override;

 protected:
  void proccess_submenu(MenuItemMenu *menu, dallas_temp::DallasTemperatureSensor *sensor_obj);
};

}  // namespace display_menu_renderers
}  // namespace esphome
