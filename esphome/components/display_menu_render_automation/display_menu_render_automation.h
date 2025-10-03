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
  MenuItemSelect *createSourceTriggerSelect(TriggerConfig *trigger);
  MenuItemSelect *createInputTriggerTypeSelect(TriggerConfig *trigger);
  MenuItemSelect *createBinarySensorSelect(TriggerConfig *trigger);

  MenuItemSelect *createSourceActionSelect(ActionConfig *action);
  MenuItemSelect *createSwitchActionTypeSelect(ActionConfig *action);
  MenuItemSelect *createSwitchSelect(ActionConfig *action);
  MenuItemNumber *createDelayNumber(ActionConfig *action);

  // State management
  void createNewAutomation();
  void saveAutomation(AutomationConfig *config);
  void deleteAutomation(size_t index);

  // Utility methods
  std::string getTriggerSummary(const TriggerConfig &trigger);
  std::string getActionSummary(const ActionConfig &action);

 protected:
  // Temporary state for editing
  std::unique_ptr<AutomationConfig> editing_automation_;
  bool is_new_automation_{false};

  // Track menu regeneration
  MenuItemMenu *root_menu_{nullptr};
  // Reference to the display menu component
  DisplayMenuComponent *menu_component_{nullptr};
};

// Helper class for managing dynamic selects
class DynamicSelect : public simple::SimpleSelect {
 public:
  DynamicSelect() { this->traits.set_options({}); }

  void set_options(const std::vector<std::string> &options) {
    this->traits.set_options(options);
    if (!options.empty() && this->state.empty()) {
      this->state = options[0];
    }
  }

  void set_index(size_t index) {
    auto options = this->traits.get_options();
    if (index < options.size()) {
      this->state = options[index];
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

}  // namespace display_menu_render_automation
}  // namespace esphome
