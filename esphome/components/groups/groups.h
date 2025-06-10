#pragma once
#include "esphome/core/entity_base.h"
#include <string>
#include <vector>
#include <algorithm>
#include "entity_types.h"

namespace esphome {
namespace groups {

struct EntityInfo {
  EntityBase *entity;
  EntityType type;
  EntitySubtype subtype;
};

// Class for single group
class Group {
 public:
  void set_group_name(std::string group_name) { this->group_name_ = std::move(group_name); }
  void add_entity(EntityBase *entity, EntityType type, EntitySubtype subtype = EntitySubtype::NONE) {
    entities_.push_back({entity, type, subtype});
  }
  const std::vector<EntityInfo> &items() { return entities_; }
  const std::string &get_name() { return group_name_; }

  bool has_entity(EntityBase *entity) {
    auto lambda = [entity](const EntityInfo &info) { return info.entity == entity; };
    return std::find_if(entities_.begin(), entities_.end(), lambda) != entities_.end();
  }

 protected:
  std::string group_name_;
  std::vector<EntityInfo> entities_;
};

class GroupsStorage {
 public:
  GroupsStorage() {}
  GroupsStorage(size_t group_size) { this->groups_.reserve(group_size); }
  Group *find_group(const char *group_name) {
    auto it = std::find_if(groups_.begin(), groups_.end(),
                           [group_name](Group *group) { return group->get_name() == group_name; });
    return it != groups_.end() ? *it : nullptr;
  }
  void add_group(Group *group) { groups_.push_back(group); }
  bool has_group(Group *group) { return std::find(groups_.begin(), groups_.end(), group) != groups_.end(); }

 protected:
  std::vector<Group *> groups_;
};

extern GroupsStorage global_groups_storage;

}  // namespace groups
}  // namespace esphome
