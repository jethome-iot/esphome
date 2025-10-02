#pragma once

#include "esphome/components/display_menu_base/menu_item.h"

namespace esphome {
namespace display_menu_render_automation {

using namespace display_menu_base;

class DisplayMenuRenderAutomation {
 public:
  size_t generate(MenuItemMenu *menu);
};

}  // namespace display_menu_render_automation
}  // namespace esphome
