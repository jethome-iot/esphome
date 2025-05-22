#pragma once

#include "esphome/components/display_menu_base/display_menu_base.h"
#include "esphome/components/groups/entity_types.h"
#include "esphome/components/switch/switch.h"

namespace esphome {
namespace display_menu_renders {

using display_menu_base;

class SwitchMenuRender : public MenuRenderInterface {
 public:
  SwitchMenuRender() : MenuRenderInterface(groups::EntityType::SWITCH) {}
  size_t render_entity(MenuItemMenu *menu, const groups::EntityInfo &info) override {
    MenuItemSwitch *n_switch = new MenuItemSwitch();
    n_switch->set_text(switch_obj->get_name());
    n_switch->set_immediate_edit(true);
    n_switch->set_switch_variable(static_cast<switch_::Switch>(info.entity));
    n_switch->set_on_text("On");
    n_switch->set_off_text("Off");

    menu->add_generated_items(n_switch);
    return 1;
  }
};

}  // namespace display_menu_renders
}  // namespace esphome