#pragma once
#include "esphome/components/groups/entity_types.h"
#include "esphome/components/groups/groups.h"
#include "esphome/components/display_menu_base/menu_item.h"

namespace esphome {
namespace display_menu_render_base {

using namespace display_menu_base;

// Render interface for menu with groups
class MenuRenderInterface : public groups::GroupsStorage {
 public:
  MenuRenderInterface(groups::EntityType type, groups::EntitySubtype subtype = groups::EntitySubtype::NONE)
      : type_(type), subtype_(subtype){};

  groups::EntityType type() { return type_; }
  groups::EntitySubtype subtype() { return subtype_; }

  // Add needed items in menu with entity_info
  virtual size_t render_entity(MenuItemMenu *menu, const groups::EntityInfo &info) = 0;

  bool has_entity(EntityBase *entity) {
    for (auto *group : this->groups_) {
      if (group->has_entity(entity))
        return true;
    }
    return false;
  }

 protected:
  groups::EntityType type_;
  groups::EntitySubtype subtype_;
};

}  // namespace display_menu_render_base
}  // namespace esphome
