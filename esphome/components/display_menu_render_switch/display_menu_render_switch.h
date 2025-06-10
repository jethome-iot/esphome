#pragma once

#include "esphome/components/display_menu_render_base/display_menu_render_base.h"
#include "esphome/components/switch/switch.h"

namespace esphome {
namespace display_menu_render_switch {

using namespace display_menu_base;
using namespace display_menu_render_base;

class SwitchMenuRender : public MenuRenderInterface {
 public:
  SwitchMenuRender() : MenuRenderInterface(groups::EntityType::SWITCH) {}
  size_t render_entity(MenuItemMenu *menu, const groups::EntityInfo &info) override;
};
}  // namespace display_menu_render_switch
}  // namespace esphome
