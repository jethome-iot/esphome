#include "display_menu_render_switch.h"

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

}  // namespace display_menu_renderers
}  // namespace esphome