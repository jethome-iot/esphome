#include "display_menu_render_automation.h"
#include "esphome/core/log.h"

namespace esphome {
namespace display_menu_render_automation {

static const char *const TAG = "menu_automation";

size_t DisplayMenuRenderAutomation::generate(MenuItemMenu *menu) {
  clear_helpers();  // Clean up old helpers before regenerating
  menu->clear_items();
  root_menu_ = menu;

  // Generate list of existing automations
  for (size_t i = 0; i < global_automation_storage.size(); i++) {
    const AutomationConfig *config = global_automation_storage.getConfig(i);
    if (!config)
      continue;

    MenuItemMenu *automation_item = new MenuItemMenu();
    std::string name = config->name.empty() ? "Automation " + std::to_string(i + 1) : config->name;
    automation_item->set_text(name);

    // Show summary as value
    // automation_item->set_value_lambda([this, i](const MenuItem *) {
    //   const AutomationConfig *cfg = global_automation_storage.getConfig(i);
    //   if (!cfg)
    //     return std::string("");
    //   return getTriggerSummary(cfg->trigger);
    // });

    // Generate submenu on enter
    automation_item->set_generate_on_enter(true);
    automation_item->set_generate_lambda(
        [this, i](MenuItemMenu *submenu) { return generateAutomationEditor(submenu, i); });

    menu->add_item(automation_item);
  }

  // Add "New Automation" button
  MenuItemCommand *add_new = new MenuItemCommand();
  add_new->set_text("+ Add New Automation");
  add_new->add_on_value_callback([this, menu]() {
    createNewAutomation();
    menu->generate();  // Regenerate to show new automation
  });
  menu->add_item(add_new);

  return menu->items_size();
}

size_t DisplayMenuRenderAutomation::generateAutomationEditor(MenuItemMenu *menu, size_t index) {
  menu->clear_items();

  const AutomationConfig *config = global_automation_storage.getConfig(index);
  if (!config)
    return 0;

  // Create editable copy
  if (!editing_automation_) {
    editing_automation_ = std::make_unique<AutomationConfig>(*config);
    is_new_automation_ = false;
  }

  // Name editor (using custom item for text input)
  MenuItemCustom *name_edit = new MenuItemCustom();
  name_edit->set_text("Name");
  name_edit->set_immediate_edit(true);
  name_edit->set_value_lambda(
      [this](const MenuItem *) { return editing_automation_->name.empty() ? "<unnamed>" : editing_automation_->name; });
  // Note: Text input would require additional implementation
  menu->add_item(name_edit);

  // Enabled toggle
  MenuItemSwitch *enabled_switch = new MenuItemSwitch();
  enabled_switch->set_text("Enabled");
  enabled_switch->set_on_text("Yes");
  enabled_switch->set_off_text("No");
  // Create a temporary switch for the enabled state
  auto temp_switch = std::make_unique<simple::SimpleSwitch>();
  temp_switch->state = editing_automation_->enabled;
  auto *switch_ptr = temp_switch.get();
  switch_helpers_.push_back(std::move(temp_switch));
  enabled_switch->set_switch_variable(switch_ptr);
  enabled_switch->add_on_value_callback([this]() { editing_automation_->enabled = !editing_automation_->enabled; });
  menu->add_item(enabled_switch);

  // Trigger section label
  MenuItem *trigger_label = new MenuItem(MENU_ITEM_LABEL);
  trigger_label->set_text("Trigger:");
  menu->add_item(trigger_label);

  // Trigger editor submenu
  MenuItemMenu *trigger_menu = new MenuItemMenu();
  trigger_menu->set_text(
      [this](const MenuItem *) { return std::string(" ") + getTriggerSummary(editing_automation_->trigger); });
  trigger_menu->set_generate_on_enter(true);
  trigger_menu->set_generate_lambda(
      [this](MenuItemMenu *submenu) { return generateTriggerEditor(submenu, &editing_automation_->trigger); });
  menu->add_item(trigger_menu);

  // Actions editor submenu
  MenuItemMenu *actions_menu = new MenuItemMenu();
  actions_menu->set_text("Actions");
  actions_menu->set_value_lambda(
      [this](const MenuItem *) { return std::to_string(editing_automation_->actions.size()) + " actions"; });
  actions_menu->set_generate_on_enter(true);
  actions_menu->set_generate_lambda(
      [this](MenuItemMenu *submenu) { return generateActionsEditor(submenu, &editing_automation_->actions); });
  menu->add_item(actions_menu);

  // Save command
  MenuItemCommand *save_cmd = new MenuItemCommand();
  save_cmd->set_text("Save");
  save_cmd->add_on_value_callback([this, index, menu]() {
    if (is_new_automation_) {
      global_automation_storage.addConfig(*editing_automation_);
    } else {
      // Update existing
      auto configs = global_automation_storage.getAllConfigs();
      if (index < configs.size()) {
        configs[index] = *editing_automation_;
        global_automation_storage.clear();
        for (const auto &cfg : configs) {
          global_automation_storage.addConfig(cfg);
        }
      }
    }
    editing_automation_.reset();
    // Navigate back and regenerate
    if (root_menu_) {
      root_menu_->generate();
    }
  });
  menu->add_item(save_cmd);

  // Delete command (only for existing automations)
  if (!is_new_automation_) {
    MenuItemCommand *delete_cmd = new MenuItemCommand();
    delete_cmd->set_text("Delete");
    delete_cmd->add_on_value_callback([this, index, menu]() {
      deleteAutomation(index);
      editing_automation_.reset();
      if (root_menu_) {
        root_menu_->generate();
      }
    });
    menu->add_item(delete_cmd);
  }

  // Back button
  MenuItem *back_item = new MenuItem(MENU_ITEM_BACK);
  back_item->set_text("Back");
  menu->add_item(back_item);

  return menu->items_size();
}

size_t DisplayMenuRenderAutomation::generateTriggerEditor(MenuItemMenu *menu, TriggerConfig *trigger) {
  menu->clear_items();

  // Source selector
  MenuItemSelect *source_select = createSourceTriggerSelect(trigger);
  menu->add_item(source_select);

  // Dynamic options based on source
  if (trigger->source == SourceTrigger::Input) {
    // Input sensor selector
    MenuItemSelect *sensor_select = createBinarySensorSelect(trigger);
    menu->add_item(sensor_select);

    // Input type selector
    MenuItemSelect *type_select = createInputTriggerTypeSelect(trigger);
    menu->add_item(type_select);
  }

  // Back button
  MenuItem *back_item = new MenuItem(MENU_ITEM_BACK);
  back_item->set_text("Back");
  menu->add_item(back_item);

  return menu->items_size();
}

size_t DisplayMenuRenderAutomation::generateActionsEditor(MenuItemMenu *menu, std::vector<ActionConfig> *actions) {
  menu->clear_items();

  // List existing actions
  for (size_t i = 0; i < actions->size(); i++) {
    MenuItemMenu *action_item = new MenuItemMenu();
    action_item->set_text("Action " + std::to_string(i + 1));
    action_item->set_value_lambda([this, actions, i](const MenuItem *) {
      if (i < actions->size()) {
        return getActionSummary((*actions)[i]);
      }
      return std::string("");
    });

    action_item->set_generate_on_enter(true);
    action_item->set_generate_lambda([this, actions, i](MenuItemMenu *submenu) {
      return generateActionEditor(submenu, &(*actions)[i], i, actions);
    });
    menu->add_item(action_item);
  }

  // Add new action button
  MenuItemCommand *add_action = new MenuItemCommand();
  add_action->set_text("+ Add Action");
  add_action->add_on_value_callback([actions, menu]() {
    actions->push_back(ActionConfig());
    menu->generate();  // Regenerate to show new action
  });
  menu->add_item(add_action);

  // Back button
  MenuItem *back_item = new MenuItem(MENU_ITEM_BACK);
  back_item->set_text("Back");
  menu->add_item(back_item);

  return menu->items_size();
}

size_t DisplayMenuRenderAutomation::generateActionEditor(MenuItemMenu *menu, ActionConfig *action, size_t index,
                                                         std::vector<ActionConfig> *actions) {
  menu->clear_items();

  // Source selector
  MenuItemSelect *source_select = createSourceActionSelect(action);
  menu->add_item(source_select);

  // Dynamic options based on source
  if (action->source == SourceAction::Switch) {
    // Switch selector
    MenuItemSelect *switch_select = createSwitchSelect(action);
    menu->add_item(switch_select);

    // Action type selector
    MenuItemSelect *type_select = createSwitchActionTypeSelect(action);
    menu->add_item(type_select);

  } else if (action->source == SourceAction::Delay) {
    // Delay value editor
    MenuItemNumber *delay_number = createDelayNumber(action);
    menu->add_item(delay_number);
  }

  // Delete action button
  MenuItemCommand *delete_cmd = new MenuItemCommand();
  delete_cmd->set_text("Delete Action");
  delete_cmd->add_on_value_callback([this, actions, index, menu]() {
    if (index < actions->size()) {
      actions->erase(actions->begin() + index);
      // Navigate back and regenerate parent
      this->menu_component_->back();
    }
  });
  menu->add_item(delete_cmd);

  // Back button
  MenuItem *back_item = new MenuItem(MENU_ITEM_BACK);
  back_item->set_text("Back");
  menu->add_item(back_item);

  return menu->items_size();
}

// Helper method implementations

MenuItemSelect *DisplayMenuRenderAutomation::createSourceTriggerSelect(TriggerConfig *trigger) {
  auto select_var = std::make_unique<DynamicSelect>();
  std::vector<std::string> options;

  for (int i = 0; i < static_cast<int>(SourceTrigger::MAX_TRIGGER_TYPES); i++) {
    options.push_back(EnumUtils::sourceTriggerToString(static_cast<SourceTrigger>(i)));
  }
  select_var->set_options(options);
  select_var->set_index(static_cast<size_t>(trigger->source));

  auto *select_ptr = select_var.get();
  select_helpers_.push_back(std::move(select_var));

  MenuItemSelect *item = new MenuItemSelect();
  item->set_text("Trigger Source");
  item->set_immediate_edit(true);
  item->set_select_variable(select_ptr);
  item->add_on_value_callback(
      [trigger, select_ptr, this]() { trigger->source = static_cast<SourceTrigger>(select_ptr->get_index()); });

  return item;
}

MenuItemSelect *DisplayMenuRenderAutomation::createInputTriggerTypeSelect(TriggerConfig *trigger) {
  auto select_var = std::make_unique<DynamicSelect>();
  std::vector<std::string> options = {"None", "Press", "Release", "Click"};
  select_var->set_options(options);
  select_var->set_index(static_cast<size_t>(trigger->params.input.type));

  auto *select_ptr = select_var.get();
  select_helpers_.push_back(std::move(select_var));

  MenuItemSelect *item = new MenuItemSelect();
  item->set_text("Trigger Type");
  item->set_immediate_edit(true);
  item->set_select_variable(select_ptr);
  item->add_on_value_callback([trigger, select_ptr]() {
    trigger->params.input.type = static_cast<TypesInputTrigger>(select_ptr->get_index());
  });

  return item;
}

MenuItemSelect *DisplayMenuRenderAutomation::createBinarySensorSelect(TriggerConfig *trigger) {
  using BinarySensorSelect = EntitySelect<binary_sensor::BinarySensor>;
  auto select_var = std::make_unique<BinarySensorSelect>();

  // Load all binary sensors (automatically filters internal ones)
  select_var->load_entities(App.get_binary_sensors());

  // Pre-select if editing existing config
  if (trigger->params.input.input_id != 0) {
    select_var->select_by_id_hash(trigger->params.input.input_id);
  }

  auto *select_ptr = select_var.get();
  select_helpers_.push_back(std::move(select_var));

  MenuItemSelect *item = new MenuItemSelect();
  item->set_text("Input Sensor");
  item->set_immediate_edit(true);
  item->set_select_variable(select_ptr);

  item->add_on_value_callback(
      [trigger, select_ptr]() { trigger->params.input.input_id = select_ptr->get_selected_id_hash(); });

  return item;
}

MenuItemSelect *DisplayMenuRenderAutomation::createSourceActionSelect(ActionConfig *action) {
  auto select_var = std::make_unique<DynamicSelect>();
  std::vector<std::string> options;

  for (int i = 0; i < static_cast<int>(SourceAction::MAX_ACTION_TYPES); i++) {
    options.push_back(EnumUtils::sourceActionToString(static_cast<SourceAction>(i)));
  }
  select_var->set_options(options);
  select_var->set_index(static_cast<size_t>(action->source));

  auto *select_ptr = select_var.get();
  select_helpers_.push_back(std::move(select_var));

  MenuItemSelect *item = new MenuItemSelect();
  item->set_text("Action Type");
  item->set_immediate_edit(true);
  item->set_select_variable(select_ptr);
  item->add_on_value_callback(
      [action, select_ptr]() { action->source = static_cast<SourceAction>(select_ptr->get_index()); });

  return item;
}

MenuItemSelect *DisplayMenuRenderAutomation::createSwitchActionTypeSelect(ActionConfig *action) {
  auto select_var = std::make_unique<DynamicSelect>();
  std::vector<std::string> options = {"None", "Turn On", "Turn Off", "Toggle"};
  select_var->set_options(options);
  select_var->set_index(static_cast<size_t>(action->params.switch_action.type));

  auto *select_ptr = select_var.get();
  select_helpers_.push_back(std::move(select_var));

  MenuItemSelect *item = new MenuItemSelect();
  item->set_text("Switch Action");
  item->set_immediate_edit(true);
  item->set_select_variable(select_ptr);
  item->add_on_value_callback([action, select_ptr]() {
    action->params.switch_action.type = static_cast<TypeSwitchAction>(select_ptr->get_index());
  });

  return item;
}

MenuItemSelect *DisplayMenuRenderAutomation::createSwitchSelect(ActionConfig *action) {
  using SwitchSelect = EntitySelect<switch_::Switch>;
  auto select_var = std::make_unique<SwitchSelect>();

  // Load all switches (automatically filters internal ones)
  select_var->load_entities(App.get_switches());

  // Pre-select if editing existing config
  if (action->params.switch_action.switch_id != 0) {
    select_var->select_by_id_hash(action->params.switch_action.switch_id);
  }

  auto *select_ptr = select_var.get();
  select_helpers_.push_back(std::move(select_var));

  MenuItemSelect *item = new MenuItemSelect();
  item->set_text("Switch");
  item->set_immediate_edit(true);
  item->set_select_variable(select_ptr);

  item->add_on_value_callback(
      [action, select_ptr]() { action->params.switch_action.switch_id = select_ptr->get_selected_id_hash(); });

  return item;
}

MenuItemNumber *DisplayMenuRenderAutomation::createDelayNumber(ActionConfig *action) {
  // Create a temporary number component for delay editing
  auto number_var = std::make_unique<simple::SimpleNumber>();
  number_var->traits.set_min_value(1);
  number_var->traits.set_max_value(3600);  // Max 1 hour
  number_var->traits.set_step(1);
  number_var->state = action->params.delay.delay_s;

  auto *number_ptr = number_var.get();
  number_helpers_.push_back(std::move(number_var));

  MenuItemNumber *item = new MenuItemNumber();
  item->set_text("Delay (seconds)");
  item->set_immediate_edit(true);
  item->set_number_variable(number_ptr);
  item->set_format("%.0f s");
  item->add_on_value_callback(
      [action, number_ptr]() { action->params.delay.delay_s = static_cast<uint32_t>(number_ptr->state); });

  return item;
}

void DisplayMenuRenderAutomation::createNewAutomation() {
  editing_automation_ = std::make_unique<AutomationConfig>();
  editing_automation_->name = "New Automation";
  editing_automation_->enabled = true;
  is_new_automation_ = true;

  global_automation_storage.addConfig(*editing_automation_);
}

void DisplayMenuRenderAutomation::saveAutomation(AutomationConfig *config) {
  // Save to storage
  std::string json = global_automation_storage.saveToJson();
  ESP_LOGI(TAG, "Saving automation: %s", json.c_str());
  // Here you would typically save to preferences or flash
}

void DisplayMenuRenderAutomation::clear_helpers() {
  select_helpers_.clear();
  number_helpers_.clear();
  switch_helpers_.clear();
}

void DisplayMenuRenderAutomation::deleteAutomation(size_t index) {
  auto configs = global_automation_storage.getAllConfigs();
  if (index < configs.size()) {
    configs.erase(configs.begin() + index);
    global_automation_storage.clear();
    for (const auto &cfg : configs) {
      global_automation_storage.addConfig(cfg);
    }
  }
}

std::string DisplayMenuRenderAutomation::getTriggerSummary(const TriggerConfig &trigger) {
  if (trigger.source == SourceTrigger::None) {
    return "Not configured";
  }

  std::string summary = EnumUtils::sourceTriggerToString(trigger.source);

  if (trigger.source == SourceTrigger::Input) {
    std::string sensor_name = "Unknown Sensor";
    auto *sensor = App.get_binary_sensor_by_key(trigger.params.input.input_id);
    if (sensor) {
      sensor_name = sensor->get_name();
    }
    summary = sensor_name + " - " + EnumUtils::inputTriggerTypeToString(trigger.params.input.type);
  }

  return summary;
}

std::string DisplayMenuRenderAutomation::getActionSummary(const ActionConfig &action) {
  if (action.source == SourceAction::None) {
    return "Not configured";
  }

  std::string summary = EnumUtils::sourceActionToString(action.source);

  if (action.source == SourceAction::Switch) {
    summary += " - ";
    summary += EnumUtils::switchActionTypeToString(action.params.switch_action.type);
  } else if (action.source == SourceAction::Delay) {
    summary += " " + std::to_string(action.params.delay.delay_s) + "s";
  }

  return summary;
}

}  // namespace display_menu_render_automation
}  // namespace esphome
