#include "display_menu_renderers.h"

namespace esphome {
namespace display_menu_base {

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

auto SensorMenuRender::getRenderLambda(sensor::Sensor *sensor_obj) {
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

size_t SensorMenuRender::render_entity(MenuItemMenu *menu, const groups::EntityInfo &info) {
  sensor::Sensor *sensor_obj = static_cast<sensor::Sensor *>(info.entity);
  MenuItem *item = new MenuItem(MENU_ITEM_LABEL);
  auto lambda = get_render_lambda(sensor_obj);
  item->set_text(lambda);

  menu->add_generated_items(item);
  return 1;
}

size_t DallasTempMenuRender::render_entity(MenuItemMenu *menu, const groups::EntityInfo &info) {
  dallas_temp::DallasTemperatureSensor *sensor_obj = static_cast<dallas_temp::DallasTemperatureSensor *>(info.entity);

  MenuItemMenu *item = new MenuItemMenu();
  auto lambda = SensorMenuRender::get_render_lambda(sensor_obj);
  item->set_text(lambda);

  proccess_submenu(item, sensor_obj);

  menu->add_generated_items(item);
  return 1;
}

void DallasTempMenuRender::proccess_submenu(MenuItemMenu *menu, dallas_temp::DallasTemperatureSensor *sensor_obj) {
  MenuItem *internal_info1 = new MenuItem(MENU_ITEM_LABEL);
  internal_info1->set_text("Address:");
  MenuItem *internal_info2 = new MenuItem(MENU_ITEM_LABEL);

  auto lambda2 = [=](const display_menu_base::MenuItem *it) -> std::string { return sensor_obj->get_address_name(); };

  internal_info2->set_text(lambda2);
  menu->add_item(internal_info1);
  menu->add_item(internal_info2);
}

}  // namespace display_menu_base
}  // namespace esphome