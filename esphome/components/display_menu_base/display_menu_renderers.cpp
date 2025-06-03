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

size_t SensorMenuRender::render_entity(MenuItemMenu *menu, const groups::EntityInfo &info) {
  sensor::Sensor *sensor_obj = static_cast<sensor::Sensor *>(info.entity);
  MenuItem *item = new MenuItem(MENU_ITEM_LABEL);
  auto lambda = [=](const display_menu_base::MenuItem *it) -> std::string {
    char buf[50];
    char format_buf[10];
    sprintf(format_buf, "%%s %%0.%df", 2);
    if (sensor_obj->has_state())
      sprintf(buf, format_buf, sensor_obj->get_name().c_str(), sensor_obj->state);
    else
      sprintf(buf, "Nan");
    return buf;
  };
  item->set_text(lambda);

  menu->add_generated_items(item);
  return 1;
}

}  // namespace display_menu_base
}  // namespace esphome