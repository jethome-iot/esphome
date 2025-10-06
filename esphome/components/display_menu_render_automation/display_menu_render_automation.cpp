#include "display_menu_render_automation.h"
#include "esphome/core/log.h"

namespace esphome {
namespace display_menu_render_automation {

static const char *const TAG = "menu_automation";

// Helper functions for time conversion
static void secondsToDHMS(uint32_t total_seconds, uint16_t &days, uint8_t &hours, uint8_t &minutes, uint8_t &seconds) {
  days = total_seconds / 86400;  // 86400 = 24 * 60 * 60
  hours = (total_seconds % 86400) / 3600;
  minutes = (total_seconds % 3600) / 60;
  seconds = total_seconds % 60;
}

static uint32_t dhmsToSeconds(uint16_t days, uint8_t hours, uint8_t minutes, uint8_t seconds) {
  return days * 86400 + hours * 3600 + minutes * 60 + seconds;
}

static std::string formatDelayDHMS(uint32_t total_seconds) {
  uint16_t days;
  uint8_t hours, minutes, seconds;
  secondsToDHMS(total_seconds, days, hours, minutes, seconds);

  std::string result;
  if (days > 0) {
    result += std::to_string(days) + "d ";
  }
  if (hours > 0) {
    result += std::to_string(hours) + "h ";
  }
  if (minutes > 0 || hours > 0 || days > 0) {
    result += std::to_string(minutes) + "m ";
  }
  result += std::to_string(seconds) + "s";

  return result;
}

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
        [this, i](MenuItemMenu *submenu) { return generate_automation_editor(submenu, i); });

    menu->add_item(automation_item);
  }

  // Add "New Automation" button
  MenuItemCommand *add_new = new MenuItemCommand();
  add_new->set_text("+ Add New Automation");
  add_new->add_on_value_callback([this, menu]() {
    create_new_automation();
    menu->generate();  // Regenerate to show new automation
  });
  menu->add_item(add_new);

  return menu->items_size();
}

size_t DisplayMenuRenderAutomation::generate_automation_editor(MenuItemMenu *menu, size_t index) {
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
      [this](const MenuItem *) { return std::string(" ") + get_trigger_summary(editing_automation_->trigger); });
  trigger_menu->set_generate_on_enter(true);
  trigger_menu->set_generate_lambda(
      [this](MenuItemMenu *submenu) { return generate_trigger_editor(submenu, &editing_automation_->trigger); });
  menu->add_item(trigger_menu);

  // Actions editor submenu
  MenuItemMenu *actions_menu = new MenuItemMenu();
  actions_menu->set_text("Actions");
  actions_menu->set_value_lambda(
      [this](const MenuItem *) { return std::to_string(editing_automation_->actions.size()) + " actions"; });
  actions_menu->set_generate_on_enter(true);
  actions_menu->set_generate_lambda(
      [this](MenuItemMenu *submenu) { return generate_actions_editor(submenu, &editing_automation_->actions); });
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
      delete_automation(index);
      editing_automation_.reset();
      if (root_menu_) {
        root_menu_->generate();
      }
    });
    menu->add_item(delete_cmd);
  }

  return menu->items_size();
}

size_t DisplayMenuRenderAutomation::generate_trigger_editor(MenuItemMenu *menu, TriggerConfig *trigger) {
  menu->clear_items();

  // Clear any previous dynamic items
  dynamic_trigger_items_.clear();

  // Source selector with dynamic menu management
  MenuItemSelect *source_select = create_source_trigger_select(trigger, menu);
  menu->add_to_index(0, source_select);
  source_select->on_value_();

  return menu->items_size();
}

size_t DisplayMenuRenderAutomation::generate_actions_editor(MenuItemMenu *menu, std::vector<ActionConfig> *actions) {
  menu->clear_items();

  // List existing actions with labels
  for (size_t i = 0; i < actions->size(); i++) {
    // Action label
    MenuItem *action_label = new MenuItem(MENU_ITEM_LABEL);
    action_label->set_text("Action " + std::to_string(i + 1) + ":");
    menu->add_item(action_label);

    // Action menu item
    MenuItemMenu *action_item = new MenuItemMenu();
    action_item->set_text([this, actions, i](const MenuItem *) {
      if (i < actions->size()) {
        return std::string(" ") + get_action_summary((*actions)[i]);
      }
      return std::string(" Not configured");
    });

    action_item->set_generate_on_enter(true);
    action_item->set_generate_lambda([this, actions, i](MenuItemMenu *submenu) {
      return generate_action_editor(submenu, &(*actions)[i], i, actions);
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

  return menu->items_size();
}

size_t DisplayMenuRenderAutomation::generate_action_editor(MenuItemMenu *menu, ActionConfig *action, size_t index,
                                                           std::vector<ActionConfig> *actions) {
  menu->clear_items();

  // Clear any previous dynamic items
  dynamic_action_items_.clear();

  // Source selector with dynamic menu management
  MenuItemSelect *source_select = create_source_action_select(action, menu);
  menu->add_to_index(0, source_select);

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

  source_select->on_value_();

  return menu->items_size();
}

// Helper method implementations

MenuItemSelect *DisplayMenuRenderAutomation::create_source_trigger_select(TriggerConfig *trigger,
                                                                          MenuItemMenu *parent_menu) {
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
  item->set_text("Source");
  item->set_immediate_edit(true);
  item->set_select_variable(select_ptr);

  // Dynamic menu management in callback
  item->add_on_value_callback([trigger, select_ptr, this, parent_menu]() {
    SourceTrigger new_source = static_cast<SourceTrigger>(select_ptr->get_index());

    // Remove existing dynamic items
    for (auto *dynamic_item : dynamic_trigger_items_) {
      parent_menu->remove_item(dynamic_item);
    }
    dynamic_trigger_items_.clear();

    // Update the trigger source
    trigger->source = new_source;

    // Add new items based on the selected source
    if (new_source == SourceTrigger::Input) {
      // Insert items at the end
      size_t insert_pos = parent_menu->items_size();

      // Add binary sensor selector
      MenuItemSelect *sensor_select = create_binary_sensor_select(trigger);
      parent_menu->add_item(sensor_select, insert_pos);
      dynamic_trigger_items_.push_back(sensor_select);

      // Add input type selector
      MenuItemSelect *type_select = create_input_trigger_type_select(trigger);
      parent_menu->add_item(type_select, insert_pos + 1);
      dynamic_trigger_items_.push_back(type_select);
    }
  });

  return item;
}

MenuItemSelect *DisplayMenuRenderAutomation::create_input_trigger_type_select(TriggerConfig *trigger) {
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

MenuItemSelect *DisplayMenuRenderAutomation::create_binary_sensor_select(TriggerConfig *trigger) {
  using BinarySensorSelect = EntitySelect<binary_sensor::BinarySensor>;
  auto select_var = std::make_unique<BinarySensorSelect>();

  // Load all binary sensors (automatically filters internal ones)
  select_var->load_entities(App.get_binary_sensors());

  // Pre-select if editing existing config
  if (trigger->params.input.input_id != 0) {
    select_var->select_by_id_hash(trigger->params.input.input_id);
  } else {
    select_var->set_index(0);
    trigger->params.input.input_id = select_var->get_selected_id_hash();
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

MenuItemSelect *DisplayMenuRenderAutomation::create_source_action_select(ActionConfig *action,
                                                                         MenuItemMenu *parent_menu) {
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

  // Dynamic menu management in callback
  item->add_on_value_callback([action, select_ptr, this, parent_menu]() {
    SourceAction new_source = static_cast<SourceAction>(select_ptr->get_index());

    // Remove existing dynamic items
    for (auto *dynamic_item : dynamic_action_items_) {
      parent_menu->remove_item(dynamic_item);
    }
    dynamic_action_items_.clear();

    // Update the action source
    action->source = new_source;

    // Add new items based on the selected source
    // Insert items before Delete Action button
    size_t insert_pos = parent_menu->items_size() - 1;

    if (new_source == SourceAction::Switch) {
      // Add switch selector
      MenuItemSelect *switch_select = create_switch_select(action);
      parent_menu->add_item(switch_select, insert_pos);
      dynamic_action_items_.push_back(switch_select);

      // Add switch action type selector
      MenuItemSelect *type_select = create_switch_action_type_select(action);
      parent_menu->add_item(type_select, insert_pos + 1);
      dynamic_action_items_.push_back(type_select);

    } else if (new_source == SourceAction::Delay) {
      // Add delay time fields directly to the action editor menu

      // Get current values
      uint16_t days;
      uint8_t hours, minutes, seconds;
      secondsToDHMS(action->params.delay.delay_s, days, hours, minutes, seconds);

      // Create Days input
      auto days_var = std::make_unique<simple::SimpleNumber>();
      days_var->traits.set_min_value(0);
      days_var->traits.set_max_value(365);  // Max 365 days
      days_var->traits.set_step(1);
      days_var->publish_state(days);
      auto *days_ptr = days_var.get();
      number_helpers_.push_back(std::move(days_var));

      // Create Hours input
      auto hours_var = std::make_unique<simple::SimpleNumber>();
      hours_var->traits.set_min_value(0);
      hours_var->traits.set_max_value(23);  // Max 23 hours
      hours_var->traits.set_step(1);
      hours_var->publish_state(hours);
      auto *hours_ptr = hours_var.get();
      number_helpers_.push_back(std::move(hours_var));

      // Create Minutes input
      auto minutes_var = std::make_unique<simple::SimpleNumber>();
      minutes_var->traits.set_min_value(0);
      minutes_var->traits.set_max_value(59);
      minutes_var->traits.set_step(1);
      minutes_var->publish_state(minutes);
      auto *minutes_ptr = minutes_var.get();
      number_helpers_.push_back(std::move(minutes_var));

      // Create Seconds input
      auto seconds_var = std::make_unique<simple::SimpleNumber>();
      seconds_var->traits.set_min_value(0);
      seconds_var->traits.set_max_value(59);
      seconds_var->traits.set_step(1);
      seconds_var->publish_state(seconds);
      auto *seconds_ptr = seconds_var.get();
      number_helpers_.push_back(std::move(seconds_var));

      // Add Days field
      MenuItemNumber *days_item = new MenuItemNumber();
      days_item->set_text("Days");
      days_item->set_immediate_edit(true);
      days_item->set_number_variable(days_ptr);
      days_item->set_format("%.0f d");
      days_item->add_on_value_callback([action, days_ptr, hours_ptr, minutes_ptr, seconds_ptr]() {
        action->params.delay.delay_s =
            dhmsToSeconds(static_cast<uint16_t>(days_ptr->state), static_cast<uint8_t>(hours_ptr->state),
                          static_cast<uint8_t>(minutes_ptr->state), static_cast<uint8_t>(seconds_ptr->state));
      });
      parent_menu->add_item(days_item, insert_pos);
      dynamic_action_items_.push_back(days_item);

      // Add Hours field
      MenuItemNumber *hours_item = new MenuItemNumber();
      hours_item->set_text("Hours");
      hours_item->set_immediate_edit(true);
      hours_item->set_number_variable(hours_ptr);
      hours_item->set_format("%.0f h");
      hours_item->add_on_value_callback([action, days_ptr, hours_ptr, minutes_ptr, seconds_ptr]() {
        action->params.delay.delay_s =
            dhmsToSeconds(static_cast<uint16_t>(days_ptr->state), static_cast<uint8_t>(hours_ptr->state),
                          static_cast<uint8_t>(minutes_ptr->state), static_cast<uint8_t>(seconds_ptr->state));
      });
      parent_menu->add_item(hours_item, insert_pos + 1);
      dynamic_action_items_.push_back(hours_item);

      // Add Minutes field
      MenuItemNumber *minutes_item = new MenuItemNumber();
      minutes_item->set_text("Minutes");
      minutes_item->set_immediate_edit(true);
      minutes_item->set_number_variable(minutes_ptr);
      minutes_item->set_format("%.0f m");
      minutes_item->add_on_value_callback([action, days_ptr, hours_ptr, minutes_ptr, seconds_ptr]() {
        action->params.delay.delay_s =
            dhmsToSeconds(static_cast<uint16_t>(days_ptr->state), static_cast<uint8_t>(hours_ptr->state),
                          static_cast<uint8_t>(minutes_ptr->state), static_cast<uint8_t>(seconds_ptr->state));
      });
      parent_menu->add_item(minutes_item, insert_pos + 2);
      dynamic_action_items_.push_back(minutes_item);

      // Add Seconds field
      MenuItemNumber *seconds_item = new MenuItemNumber();
      seconds_item->set_text("Seconds");
      seconds_item->set_immediate_edit(true);
      seconds_item->set_number_variable(seconds_ptr);
      seconds_item->set_format("%.0f s");
      seconds_item->add_on_value_callback([action, days_ptr, hours_ptr, minutes_ptr, seconds_ptr]() {
        action->params.delay.delay_s =
            dhmsToSeconds(static_cast<uint16_t>(days_ptr->state), static_cast<uint8_t>(hours_ptr->state),
                          static_cast<uint8_t>(minutes_ptr->state), static_cast<uint8_t>(seconds_ptr->state));
      });
      parent_menu->add_item(seconds_item, insert_pos + 3);
      dynamic_action_items_.push_back(seconds_item);
    }
  });

  return item;
}

MenuItemSelect *DisplayMenuRenderAutomation::create_switch_action_type_select(ActionConfig *action) {
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

MenuItemSelect *DisplayMenuRenderAutomation::create_switch_select(ActionConfig *action) {
  using SwitchSelect = EntitySelect<switch_::Switch>;
  auto select_var = std::make_unique<SwitchSelect>();

  // Load all switches (automatically filters internal ones)
  select_var->load_entities(App.get_switches());

  // Pre-select if editing existing config
  if (action->params.switch_action.switch_id != 0) {
    select_var->select_by_id_hash(action->params.switch_action.switch_id);
  } else {
    select_var->set_index(0);
    action->params.switch_action.switch_id = select_var->get_selected_id_hash();
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

void DisplayMenuRenderAutomation::create_new_automation() {
  editing_automation_ = std::make_unique<AutomationConfig>();
  editing_automation_->name = "New Automation";
  editing_automation_->enabled = true;
  is_new_automation_ = true;

  global_automation_storage.addConfig(*editing_automation_);
}

void DisplayMenuRenderAutomation::save_automation(AutomationConfig *config) {
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

void DisplayMenuRenderAutomation::delete_automation(size_t index) {
  auto configs = global_automation_storage.getAllConfigs();
  if (index < configs.size()) {
    configs.erase(configs.begin() + index);
    global_automation_storage.clear();
    for (const auto &cfg : configs) {
      global_automation_storage.addConfig(cfg);
    }
  }
}

std::string DisplayMenuRenderAutomation::get_trigger_summary(const TriggerConfig &trigger) {
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

std::string DisplayMenuRenderAutomation::get_action_summary(const ActionConfig &action) {
  if (action.source == SourceAction::None) {
    return "Not configured";
  }

  std::string summary = EnumUtils::sourceActionToString(action.source);

  if (action.source == SourceAction::Switch) {
    std::string switch_name = "Unknown Switch";
    auto *sw = App.get_switch_by_key(action.params.switch_action.switch_id);
    if (sw) {
      switch_name = sw->get_name();
    }
    summary = switch_name + " - " + EnumUtils::switchActionTypeToString(action.params.switch_action.type);
  } else if (action.source == SourceAction::Delay) {
    summary += " " + formatDelayDHMS(action.params.delay.delay_s);
  }

  return summary;
}

}  // namespace display_menu_render_automation
}  // namespace esphome
