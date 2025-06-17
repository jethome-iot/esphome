#include "display_menu_renderers.h"

namespace esphome {
namespace display_menu_render_base {

#ifdef USE_SENSOR
size_t SensorMenuRender::render_entity(MenuItemMenu *menu, EntityBase *entity) {
  sensor::Sensor *sensor_obj = static_cast<sensor::Sensor *>(entity);
  MenuItem *item = new MenuItem(MENU_ITEM_LABEL);
  auto lambda = get_render_lambda(sensor_obj, true);
  item->set_text(lambda);

  menu->add_generated_items(item);
  return 1;
}
#endif

#ifdef USE_SWITCH
size_t SwitchMenuRender::render_entity(MenuItemMenu *menu, EntityBase *entity) {
  switch_::Switch *switch_obj = static_cast<switch_::Switch *>(entity);
  MenuItemSwitch *n_switch = new MenuItemSwitch();
  n_switch->set_text(switch_obj->get_name());
  n_switch->set_immediate_edit(true);
  n_switch->set_switch_variable(switch_obj);
  n_switch->set_on_text("On");
  n_switch->set_off_text("Off");

  menu->add_generated_items(n_switch);
  return 1;
}
#endif

}  // namespace display_menu_render_base
}  // namespace esphome
