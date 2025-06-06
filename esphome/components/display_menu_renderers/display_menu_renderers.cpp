#include "display_menu_renderers.h"

namespace esphome {
namespace display_menu_renderers {

size_t SwitchMenuRender::render_entity(MenuItemMenu *menu, const groups::EntityInfo &info) {
  switch_::Switch *switch_obj = static_cast<switch_::Switch *>(info.entity);
  MenuItemSwitch *n_switch = new MenuItemSwitch();
  n_switch->set_text(switch_obj->get_name());
  n_switch->set_immediate_edit(true);
  n_switch->set_switch_variable(switch_obj);
  n_switch->set_on_text("On");
  n_switch->set_off_text("Off");

  menu->add_generated_items(n_switch);
  return 1;
}

size_t SensorMenuRender::render_entity(MenuItemMenu *menu, const groups::EntityInfo &info) {
  sensor::Sensor *sensor_obj = static_cast<sensor::Sensor *>(info.entity);
  MenuItem *item = new MenuItem(MENU_ITEM_LABEL);
  auto lambda = get_render_lambda(sensor_obj, true);
  item->set_text(lambda);

  menu->add_generated_items(item);
  return 1;
}

size_t DallasTempMenuRender::render_entity(MenuItemMenu *menu, const groups::EntityInfo &info) {
  dallas_temp::DallasTemperatureSensor *sensor_obj = static_cast<dallas_temp::DallasTemperatureSensor *>(info.entity);

  MenuItemMenu *item = new MenuItemMenu();
  auto lambda = SensorMenuRender::get_render_lambda(sensor_obj, true);
  item->set_text(lambda);

  proccess_submenu(item, sensor_obj);

  menu->add_generated_items(item);
  return 1;
}

void DallasTempMenuRender::proccess_submenu(MenuItemMenu *menu, dallas_temp::DallasTemperatureSensor *sensor_obj) {
  MenuItem *name_info = new MenuItem(MENU_ITEM_LABEL);
  name_info->set_text(sensor_obj->get_name());
  auto internal_lambda = SensorMenuRender::get_render_lambda(sensor_obj, false);

  MenuItem *temp_info = new MenuItem(MENU_ITEM_LABEL);
  auto lambda = [=](const display_menu_base::MenuItem *it) -> std::string {
    return "Temp: " + internal_lambda(nullptr);
  };
  temp_info->set_text(lambda);

  MenuItem *address_info = new MenuItem(MENU_ITEM_LABEL);
  auto address_lambda = [=](const display_menu_base::MenuItem *it) -> std::string {
    return sensor_obj->get_address_name();
  };
  address_info->set_text(address_lambda);

  menu->add_item(name_info);
  menu->add_item(temp_info);
  menu->add_item(address_info);
}

}  // namespace display_menu_renderers
}  // namespace esphome