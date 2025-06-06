#pragma once

#include "esphome/components/display_menu_base/display_menu_base.h"
#include "esphome/components/groups/entity_types.h"
#include "esphome/components/switch/switch.h"

namespace esphome {
namespace display_menu_renderers {

using namespace display_menu_base;

class SwitchMenuRender : public MenuRenderInterface {
 public:
  SwitchMenuRender() : MenuRenderInterface(groups::EntityType::SWITCH) {}
  size_t render_entity(MenuItemMenu *menu, const groups::EntityInfo &info) override;
};
}  // namespace display_menu_renderers
}  // namespace esphome
