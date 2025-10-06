#include "esphome/components/display_menu_base/menu_item.h"
#include "esphome/components/automations/automation_system.h"
#include "esphome/components/display_menu_base/display_menu_base.h"
#include "esphome/components/automations/enums.h"
#include "esphome/core/application.h"
#include <memory>
#include <functional>
#include "esphome/components/simple/simple_switch.h"
#include "esphome/components/simple/simple_select.h"
#include "esphome/components/simple/simple_number.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/switch/switch.h"
#include <algorithm>
#include <vector>

namespace esphome {
namespace display_menu_render_automation {

using namespace display_menu_base;
using namespace automations;

class DisplayMenuRenderAutomation {
 public:
  DisplayMenuRenderAutomation() = default;

  size_t generate(MenuItemMenu *menu);

  void set_menu(DisplayMenuComponent *menu) { this->menu_component_ = menu; }

 protected:
  size_t generateAutomationEditor(MenuItemMenu *menu, size_t index);
  size_t generateTriggerEditor(MenuItemMenu *menu, TriggerConfig *trigger);
  size_t generateActionsEditor(MenuItemMenu *menu, std::vector<ActionConfig> *actions);
  size_t generateActionEditor(MenuItemMenu *menu, ActionConfig *action, size_t index,
                              std::vector<ActionConfig> *actions);

  // Helper methods for creating selection menus
  MenuItemSelect *createSourceTriggerSelect(TriggerConfig *trigger, MenuItemMenu *parent_menu);
  MenuItemSelect *createInputTriggerTypeSelect(TriggerConfig *trigger);
  MenuItemSelect *createBinarySensorSelect(TriggerConfig *trigger);

  MenuItemSelect *createSourceActionSelect(ActionConfig *action, MenuItemMenu *parent_menu);
  MenuItemSelect *createSwitchActionTypeSelect(ActionConfig *action);
  MenuItemSelect *createSwitchSelect(ActionConfig *action);

  // State management
  void createNewAutomation();
  void saveAutomation(AutomationConfig *config);
  void deleteAutomation(size_t index);

  // Utility methods
  std::string getTriggerSummary(const TriggerConfig &trigger);
  std::string getActionSummary(const ActionConfig &action);

  // Memory management
  void clear_helpers();

 protected:
  // Temporary state for editing
  std::unique_ptr<AutomationConfig> editing_automation_;
  bool is_new_automation_{false};

  // Track menu regeneration
  MenuItemMenu *root_menu_{nullptr};
  // Reference to the display menu component
  DisplayMenuComponent *menu_component_{nullptr};

  // Helper component storage for memory management
  std::vector<std::unique_ptr<simple::SimpleSelect>> select_helpers_;
  std::vector<std::unique_ptr<simple::SimpleNumber>> number_helpers_;
  std::vector<std::unique_ptr<simple::SimpleSwitch>> switch_helpers_;

  // Dynamic menu item tracking
  std::vector<MenuItem *> dynamic_trigger_items_;
  std::vector<MenuItem *> dynamic_action_items_;
};

// Helper class for managing dynamic selects
class DynamicSelect : public simple::SimpleSelect {
 public:
  DynamicSelect() { this->traits.set_options({}); }

  void set_options(const std::vector<std::string> &options) {
    this->traits.set_options(options);
    if (!options.empty() && this->state.empty()) {
      this->state = options[0];
      this->set_has_state(true);
    }
  }

  void set_index(size_t index) {
    auto options = this->traits.get_options();
    if (index < options.size()) {
      this->state = options[index];
      this->set_has_state(true);
    }
  }

  size_t get_index() const {
    auto options = this->traits.get_options();
    auto it = std::find(options.begin(), options.end(), this->state);
    if (it != options.end()) {
      return std::distance(options.begin(), it);
    }
    return 0;
  }
};

// Template class for entity selection with filtering
template<typename EntityType> class EntitySelect : public simple::SimpleSelect {
 public:
  EntitySelect() { this->traits.set_options({}); }

  struct EntityInfo {
    std::string display_name;
    EntityType *entity;
    uint32_t id_hash;
  };

  using FilterFunc = std::function<bool(EntityType *)>;

  void load_entities(const std::vector<EntityType *> &entities, FilterFunc filter = nullptr) {
    std::vector<std::string> display_names;
    this->entity_infos_.clear();

    for (auto *entity : entities) {
      if (!entity)
        continue;
      if (!entity->has_own_name())
        continue;
      if (entity->is_internal())
        continue;  // Skip internal entities

      if (filter && !filter(entity))
        continue;

      EntityInfo info;
      info.display_name = entity->get_name();
      info.entity = entity;
      info.id_hash = entity->get_object_id_hash();

      display_names.push_back(info.display_name);
      this->entity_infos_.push_back(info);
    }

    if (display_names.empty()) {
      display_names.push_back("<none available>");
    }

    this->traits.set_options(display_names);
    if (!display_names.empty() && this->state.empty()) {
      this->state = display_names[0];
      this->set_has_state(true);
    }
  }

  EntityType *get_selected_entity() const {
    size_t index = get_index();
    if (index < this->entity_infos_.size()) {
      return this->entity_infos_[index].entity;
    }
    return nullptr;
  }

  uint32_t get_selected_id_hash() const {
    size_t index = get_index();
    if (index < this->entity_infos_.size()) {
      return this->entity_infos_[index].id_hash;
    }
    return 0;
  }

  bool select_by_id_hash(uint32_t id_hash) {
    for (size_t i = 0; i < this->entity_infos_.size(); i++) {
      if (this->entity_infos_[i].id_hash == id_hash) {
        set_index(i);
        return true;
      }
    }
    return false;
  }

  void set_index(size_t index) {
    auto options = this->traits.get_options();
    if (index < options.size()) {
      this->state = options[index];
      this->set_has_state(true);
    }
  }

  size_t get_index() const {
    auto options = this->traits.get_options();
    auto it = std::find(options.begin(), options.end(), this->state);
    if (it != options.end()) {
      return std::distance(options.begin(), it);
    }
    return 0;
  }

 private:
  std::vector<EntityInfo> entity_infos_;
};

}  // namespace display_menu_render_automation
}  // namespace esphome
