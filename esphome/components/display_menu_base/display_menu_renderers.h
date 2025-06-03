#pragma once

#include "esphome/components/display_menu_base/display_menu_base.h"
#include "esphome/components/groups/entity_types.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/dallas_temp/dallas_temp.h"

namespace esphome {
namespace display_menu_base {

class SwitchMenuRender : public MenuRenderInterface {
 public:
  SwitchMenuRender() : MenuRenderInterface(groups::EntityType::SWITCH) {}
  size_t render_entity(MenuItemMenu *menu, const groups::EntityInfo &info) override;
};

class SensorMenuRender : public MenuRenderInterface {
 public:
  SensorMenuRender() : MenuRenderInterface(groups::EntityType::SENSOR) {}
  size_t render_entity(MenuItemMenu *menu, const groups::EntityInfo &info) override;
  static auto get_render_lambda(sensor::Sensor *sensor_obj) {
    auto lambda = [=](const display_menu_base::MenuItem *it) -> std::string {
      char buf[50];
      char format_buf[10];
      sprintf(format_buf, "%%s %%0.%df", 2);
      if (sensor_obj->has_state())
        sprintf(buf, format_buf, sensor_obj->get_name().c_str(), sensor_obj->state);
      else
        sprintf(buf, "%s Nan", sensor_obj->get_name().c_str());
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

}  // namespace display_menu_base
}  // namespace esphome
