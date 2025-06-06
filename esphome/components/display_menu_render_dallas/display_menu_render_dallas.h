#pragma once

#include "esphome/components/display_menu_base/display_menu_base.h"
#include "esphome/components/groups/entity_types.h"
#include "esphome/components/dallas_temp/dallas_temp.h"

namespace esphome {
namespace display_menu_renderers {

using namespace display_menu_base;

class DallasTempMenuRender : public MenuRenderInterface {
 public:
  DallasTempMenuRender() : MenuRenderInterface(groups::EntityType::SENSOR, groups::EntitySubtype::DALLAS) {}
  size_t render_entity(MenuItemMenu *menu, const groups::EntityInfo &info) override;

 protected:
  void proccess_submenu(MenuItemMenu *menu, dallas_temp::DallasTemperatureSensor *sensor_obj);
};

}  // namespace display_menu_renderers
}  // namespace esphome
