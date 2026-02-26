import re

from esphome import automation, core
from esphome.automation import maybe_simple_id
import esphome.codegen as cg
from esphome.components import groups as gp
from esphome.components.binary_sensor import BinarySensor
from esphome.components.number import Number
from esphome.components.select import Select
from esphome.components.switch import Switch
import esphome.config_validation as cv
from esphome.const import (
    CONF_ACTIVE,
    CONF_AFTER,
    CONF_BINARY_SENSOR,
    CONF_COMMAND,
    CONF_CUSTOM,
    CONF_FORMAT,
    CONF_GROUPS,
    CONF_ID,
    CONF_ITEMS,
    CONF_MODE,
    CONF_NUMBER,
    CONF_ON_VALUE,
    CONF_POSITION,
    CONF_SENSOR,
    CONF_SWITCH,
    CONF_TEXT,
    CONF_TRIGGER_ID,
    CONF_TYPE,
    CONF_VALUE,
    CONF_WEIGHT,
)

CONF_GENERATE_LAMBDA = "generate_lambda"
CONF_GENERATE_ON_ENTER = "generate_on_enter"

CODEOWNERS = ["@numo68"]

AUTO_LOAD = [
    "display_menu_render_base",
    "groups",
    "binary_sensor",
    "number",
    "select",
    "switch",
]

display_menu_base_ns = cg.esphome_ns.namespace("display_menu_base")


CONF_ROTARY = "rotary"
CONF_JOYSTICK = "joystick"
CONF_RIGHT_FOR_MENU_ENTER = "right_for_menu_enter"
CONF_LABEL = "label"
CONF_MENU = "menu"
CONF_BACK = "back"
CONF_SELECT = "select"
CONF_ON_TEXT = "on_text"
CONF_OFF_TEXT = "off_text"
CONF_NO_DATA_TEXT = "no_data_text"
CONF_VALUE_LAMBDA = "value_lambda"
CONF_IMMEDIATE_EDIT = "immediate_edit"
CONF_ROOT_ITEM_ID = "root_item_id"
CONF_ON_ENTER = "on_enter"
CONF_ON_LEAVE = "on_leave"
CONF_ON_NEXT = "on_next"
CONF_ON_PREV = "on_prev"
CONF_BEFORE = "before"

DisplayMenuComponent = display_menu_base_ns.class_("DisplayMenuComponent", cg.Component)

MenuItem = display_menu_base_ns.class_("MenuItem")
MenuItemPtr = MenuItem.operator("ptr")
MenuItemConstPtr = MenuItem.operator("ptr").operator("const")
# Menu item classes inherit from MenuItem for proper ID type checking
MenuItemMenu = display_menu_base_ns.class_("MenuItemMenu", MenuItem)
MenuItemMenuPtr = MenuItemMenu.operator("ptr")
MenuItemSelect = display_menu_base_ns.class_("MenuItemSelect", MenuItem)
MenuItemNumber = display_menu_base_ns.class_("MenuItemNumber", MenuItem)
MenuItemSwitch = display_menu_base_ns.class_("MenuItemSwitch", MenuItem)
MenuItemCommand = display_menu_base_ns.class_("MenuItemCommand", MenuItem)
MenuItemCustom = display_menu_base_ns.class_("MenuItemCustom", MenuItem)
MenuItemBinarySensor = display_menu_base_ns.class_("MenuItemBinarySensor", MenuItem)
MenuItemValue = display_menu_base_ns.class_("MenuItemValue", MenuItem)

UpAction = display_menu_base_ns.class_("UpAction", automation.Action)
DownAction = display_menu_base_ns.class_("DownAction", automation.Action)
LeftAction = display_menu_base_ns.class_("LeftAction", automation.Action)
RightAction = display_menu_base_ns.class_("RightAction", automation.Action)
EnterAction = display_menu_base_ns.class_("EnterAction", automation.Action)
ShowAction = display_menu_base_ns.class_("ShowAction", automation.Action)
HideAction = display_menu_base_ns.class_("HideAction", automation.Action)
BackAction = display_menu_base_ns.class_("BackAction", automation.Action)
ShowMainAction = display_menu_base_ns.class_("ShowMainAction", automation.Action)

IsActiveCondition = display_menu_base_ns.class_(
    "IsActiveCondition", automation.Condition
)

MenuItemType = display_menu_base_ns.enum("MenuItemType")

MENU_ITEM_TYPES = {
    CONF_LABEL: MenuItemType.MENU_ITEM_LABEL,
    CONF_MENU: MenuItemType.MENU_ITEM_MENU,
    CONF_BACK: MenuItemType.MENU_ITEM_BACK,
    CONF_SELECT: MenuItemType.MENU_ITEM_SELECT,
    CONF_NUMBER: MenuItemType.MENU_ITEM_NUMBER,
    CONF_SWITCH: MenuItemType.MENU_ITEM_SWITCH,
    CONF_COMMAND: MenuItemType.MENU_ITEM_COMMAND,
    CONF_CUSTOM: MenuItemType.MENU_ITEM_CUSTOM,
    CONF_BINARY_SENSOR: MenuItemType.MENU_ITEM_BINARY_SENSOR,
    CONF_VALUE: MenuItemType.MENU_ITEM_VALUE,
}

MENU_ITEMS_WITH_SPECIALIZED_CLASSES = (
    CONF_MENU,
    CONF_SELECT,
    CONF_NUMBER,
    CONF_SWITCH,
    CONF_COMMAND,
    CONF_CUSTOM,
    CONF_BINARY_SENSOR,
    CONF_VALUE,
)

MenuMode = display_menu_base_ns.enum("MenuMode")

MENU_MODES = {
    CONF_ROTARY: MenuMode.MENU_MODE_ROTARY,
    CONF_JOYSTICK: MenuMode.MENU_MODE_JOYSTICK,
}

DisplayMenuOnEnterTrigger = display_menu_base_ns.class_(
    "DisplayMenuOnEnterTrigger", automation.Trigger
)

DisplayMenuOnLeaveTrigger = display_menu_base_ns.class_(
    "DisplayMenuOnLeaveTrigger", automation.Trigger
)

DisplayMenuOnValueTrigger = display_menu_base_ns.class_(
    "DisplayMenuOnValueTrigger", automation.Trigger
)

DisplayMenuOnNextTrigger = display_menu_base_ns.class_(
    "DisplayMenuOnNextTrigger", automation.Trigger
)

DisplayMenuOnPrevTrigger = display_menu_base_ns.class_(
    "DisplayMenuOnPrevTrigger", automation.Trigger
)


def validate_format(format):
    if re.search(r"^%([+-])*(\d+)*(\.\d+)*[fg]$", format) is None:
        raise cv.Invalid(
            f"{CONF_FORMAT}: has to specify a printf-like format string specifying exactly one f or g type conversion, '{format}' provided"
        )

    return format


def validate_items_groups_in_menu(config):
    if (
        CONF_ITEMS not in config
        and CONF_GROUPS not in config
        and CONF_GENERATE_LAMBDA not in config
    ):
        raise cv.Invalid(
            "Menu item should have at least one of the keys: groups, items, generate_lambda"
        )
    return config


# Use a simple indirection to circumvent the recursion limitation
def menu_item_schema(value):
    return MENU_ITEM_SCHEMA(value)


MENU_ITEM_POSITION_SCHEMA = cv.Schema(
    {
        cv.Exclusive(CONF_BEFORE, "position"): cv.use_id(MenuItem),
        cv.Exclusive(CONF_AFTER, "position"): cv.use_id(MenuItem),
    }
)

MENU_ITEM_COMMON_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_TEXT): cv.templatable(cv.string),
        cv.Optional(CONF_WEIGHT): cv.int_,
        cv.Optional(CONF_POSITION): MENU_ITEM_POSITION_SCHEMA,
    }
)

MENU_ITEM_ENTER_LEAVE_SCHEMA = MENU_ITEM_COMMON_SCHEMA.extend(
    {
        cv.Optional(CONF_ON_ENTER): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    DisplayMenuOnEnterTrigger
                ),
            }
        ),
        cv.Optional(CONF_ON_LEAVE): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    DisplayMenuOnLeaveTrigger
                ),
            }
        ),
    }
)

MENU_ITEM_VALUE_SCHEMA = MENU_ITEM_COMMON_SCHEMA.extend(
    {
        cv.Optional(CONF_ON_VALUE): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    DisplayMenuOnValueTrigger
                ),
            }
        ),
    }
)

MENU_ITEM_ENTER_LEAVE_VALUE_SCHEMA = MENU_ITEM_ENTER_LEAVE_SCHEMA.extend(
    {
        cv.Optional(CONF_ON_VALUE): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    DisplayMenuOnValueTrigger
                ),
            }
        ),
    }
)

MENU_ITEM_SCHEMA = cv.typed_schema(
    {
        CONF_LABEL: MENU_ITEM_COMMON_SCHEMA.extend(
            {
                cv.GenerateID(CONF_ID): cv.declare_id(MenuItem),
            }
        ),
        CONF_BACK: MENU_ITEM_COMMON_SCHEMA.extend(
            {
                cv.GenerateID(CONF_ID): cv.declare_id(MenuItem),
            }
        ),
        CONF_MENU: cv.All(
            MENU_ITEM_ENTER_LEAVE_SCHEMA.extend(
                {
                    cv.GenerateID(CONF_ID): cv.declare_id(MenuItemMenu),
                    cv.Optional(CONF_ITEMS): cv.All(
                        cv.ensure_list(menu_item_schema), cv.Length(min=1)
                    ),
                    cv.Optional(CONF_GENERATE_ON_ENTER): cv.boolean,
                    cv.Optional(CONF_GENERATE_LAMBDA): cv.lambda_,
                }
            ).extend(gp.LIST_OF_GROUPS_SCHEMA),
            validate_items_groups_in_menu,
        ),
        CONF_SELECT: MENU_ITEM_ENTER_LEAVE_VALUE_SCHEMA.extend(
            {
                cv.GenerateID(CONF_ID): cv.declare_id(MenuItemSelect),
                cv.Required(CONF_SELECT): cv.use_id(Select),
                cv.Optional(CONF_IMMEDIATE_EDIT, default=False): cv.boolean,
                cv.Optional(CONF_VALUE_LAMBDA): cv.returning_lambda,
            }
        ),
        CONF_NUMBER: MENU_ITEM_ENTER_LEAVE_VALUE_SCHEMA.extend(
            {
                cv.GenerateID(CONF_ID): cv.declare_id(MenuItemNumber),
                cv.Required(CONF_NUMBER): cv.use_id(Number),
                cv.Optional(CONF_IMMEDIATE_EDIT, default=False): cv.boolean,
                cv.Optional(CONF_FORMAT, default="%.1f"): cv.All(
                    cv.string_strict,
                    validate_format,
                ),
                cv.Optional(CONF_VALUE_LAMBDA): cv.returning_lambda,
            }
        ),
        CONF_SWITCH: MENU_ITEM_ENTER_LEAVE_VALUE_SCHEMA.extend(
            {
                cv.GenerateID(CONF_ID): cv.declare_id(MenuItemSwitch),
                cv.Required(CONF_SWITCH): cv.use_id(Switch),
                cv.Optional(CONF_IMMEDIATE_EDIT, default=False): cv.boolean,
                cv.Optional(CONF_ON_TEXT, default="On"): cv.string_strict,
                cv.Optional(CONF_OFF_TEXT, default="Off"): cv.string_strict,
                cv.Optional(CONF_VALUE_LAMBDA): cv.returning_lambda,
            }
        ),
        CONF_BINARY_SENSOR: MENU_ITEM_ENTER_LEAVE_VALUE_SCHEMA.extend(
            {
                cv.GenerateID(CONF_ID): cv.declare_id(MenuItemBinarySensor),
                cv.Required(CONF_SENSOR): cv.use_id(BinarySensor),
                cv.Optional(CONF_ON_TEXT, default="On"): cv.string_strict,
                cv.Optional(CONF_OFF_TEXT, default="Off"): cv.string_strict,
                cv.Optional(CONF_NO_DATA_TEXT, default="Nan"): cv.string_strict,
            }
        ),
        CONF_COMMAND: MENU_ITEM_VALUE_SCHEMA.extend(
            {
                cv.GenerateID(CONF_ID): cv.declare_id(MenuItemCommand),
            }
        ),
        CONF_CUSTOM: MENU_ITEM_ENTER_LEAVE_VALUE_SCHEMA.extend(
            {
                cv.GenerateID(CONF_ID): cv.declare_id(MenuItemCustom),
                cv.Optional(CONF_IMMEDIATE_EDIT, default=False): cv.boolean,
                cv.Optional(CONF_VALUE_LAMBDA): cv.returning_lambda,
                cv.Optional(CONF_ON_NEXT): automation.validate_automation(
                    {
                        cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                            DisplayMenuOnNextTrigger
                        ),
                    }
                ),
                cv.Optional(CONF_ON_PREV): automation.validate_automation(
                    {
                        cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                            DisplayMenuOnPrevTrigger
                        ),
                    }
                ),
            }
        ),
        CONF_VALUE: MENU_ITEM_ENTER_LEAVE_VALUE_SCHEMA.extend(
            {
                cv.GenerateID(CONF_ID): cv.declare_id(MenuItemValue),
                cv.Optional(CONF_VALUE_LAMBDA): cv.returning_lambda,
            }
        ),
    },
    default_type="label",
    lower=True,
)

DISPLAY_MENU_BASE_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_ACTIVE, default=True): cv.boolean,
        cv.GenerateID(CONF_ROOT_ITEM_ID): cv.declare_id(MenuItemMenu),
        cv.Optional(CONF_MODE, default=CONF_ROTARY): cv.enum(MENU_MODES),
        cv.Optional(CONF_RIGHT_FOR_MENU_ENTER, default=True): cv.boolean,
        cv.Optional(CONF_ON_ENTER): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    DisplayMenuOnEnterTrigger
                ),
            }
        ),
        cv.Optional(CONF_ON_LEAVE): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    DisplayMenuOnLeaveTrigger
                ),
            }
        ),
        cv.Required(CONF_ITEMS): cv.All(
            cv.ensure_list(MENU_ITEM_SCHEMA), cv.Length(min=1)
        ),
    }
).extend(cv.COMPONENT_SCHEMA)

MENU_ACTION_SCHEMA = maybe_simple_id(
    {
        cv.GenerateID(CONF_ID): cv.use_id(DisplayMenuComponent),
    }
)


@automation.register_action("display_menu.up", UpAction, MENU_ACTION_SCHEMA)
async def menu_up_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action("display_menu.down", DownAction, MENU_ACTION_SCHEMA)
async def menu_down_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action("display_menu.left", LeftAction, MENU_ACTION_SCHEMA)
async def menu_left_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action("display_menu.right", RightAction, MENU_ACTION_SCHEMA)
async def menu_right_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action("display_menu.enter", EnterAction, MENU_ACTION_SCHEMA)
async def menu_enter_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action("display_menu.show", ShowAction, MENU_ACTION_SCHEMA)
async def menu_show_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action("display_menu.hide", HideAction, MENU_ACTION_SCHEMA)
async def menu_hide_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action("display_menu.back", BackAction, MENU_ACTION_SCHEMA)
async def menu_back_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action(
    "display_menu.show_main", ShowMainAction, MENU_ACTION_SCHEMA
)
async def menu_show_main_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_condition(
    "display_menu.is_active",
    IsActiveCondition,
    automation.maybe_simple_id(
        {
            cv.GenerateID(CONF_ID): cv.use_id(DisplayMenuComponent),
        }
    ),
)
async def display_menu_is_active_to_code(config, condition_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(condition_id, template_arg, paren)


async def menu_item_to_code(menu, config):
    """Create and configure a menu item. Returns the item variable.

    Note: This function does NOT add the item to parent. Use add_sorted_items()
    to add items to parent in weight-sorted order.
    """
    if config[CONF_TYPE] in MENU_ITEMS_WITH_SPECIALIZED_CLASSES:
        item = cg.new_Pvariable(config[CONF_ID])
    else:
        item = cg.new_Pvariable(config[CONF_ID], MENU_ITEM_TYPES[config[CONF_TYPE]])

    if CONF_TEXT in config:
        if isinstance(config[CONF_TEXT], core.Lambda):
            template_ = await cg.templatable(
                config[CONF_TEXT], [(MenuItemConstPtr, "it")], cg.std_string
            )
            cg.add(item.set_text(template_))
        else:
            cg.add(item.set_text(config[CONF_TEXT]))
    if CONF_VALUE_LAMBDA in config:
        template_ = await cg.process_lambda(
            config[CONF_VALUE_LAMBDA],
            [(MenuItemConstPtr, "it")],
            return_type=cg.std_string,
        )
        cg.add(item.set_value_lambda(template_))

    # Process nested items with weight sorting
    if CONF_ITEMS in config:
        await add_sorted_items(menu, config[CONF_ITEMS], item)

    if CONF_IMMEDIATE_EDIT in config:
        cg.add(item.set_immediate_edit(config[CONF_IMMEDIATE_EDIT]))
    if config[CONF_TYPE] == CONF_SELECT:
        var = await cg.get_variable(config[CONF_SELECT])
        cg.add(item.set_select_variable(var))
    if config[CONF_TYPE] == CONF_NUMBER:
        var = await cg.get_variable(config[CONF_NUMBER])
        cg.add(item.set_number_variable(var))
        cg.add(item.set_format(config[CONF_FORMAT]))
    if config[CONF_TYPE] == CONF_SWITCH:
        var = await cg.get_variable(config[CONF_SWITCH])
        cg.add(item.set_switch_variable(var))
        cg.add(item.set_on_text(config[CONF_ON_TEXT]))
        cg.add(item.set_off_text(config[CONF_OFF_TEXT]))
    if config[CONF_TYPE] == CONF_BINARY_SENSOR:
        var = await cg.get_variable(config[CONF_SENSOR])
        cg.add(item.set_binary_sensor_variable(var))
        cg.add(item.set_on_text(config[CONF_ON_TEXT]))
        cg.add(item.set_off_text(config[CONF_OFF_TEXT]))
        cg.add(item.set_no_data_text(config[CONF_NO_DATA_TEXT]))
    if config[CONF_TYPE] == CONF_MENU:
        if (groups := config.get(CONF_GROUPS)) is not None:
            for group in groups:
                group_var = await cg.get_variable(group)
                cg.add(item.add_group(group_var))
        if config.get(CONF_GENERATE_ON_ENTER):
            cg.add(item.set_generate_on_enter(config[CONF_GENERATE_ON_ENTER]))
        if lambda_config := config.get(CONF_GENERATE_LAMBDA):
            lambda_ = await cg.process_lambda(
                lambda_config,
                [
                    (MenuItemMenuPtr, "menu"),
                ],
                return_type=cg.size_t,
            )
            cg.add(item.set_generate_lambda(lambda_))

    for conf in config.get(CONF_ON_ENTER, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], item)
        await automation.build_automation(trigger, [(MenuItemConstPtr, "it")], conf)
    for conf in config.get(CONF_ON_LEAVE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], item)
        await automation.build_automation(trigger, [(MenuItemConstPtr, "it")], conf)
    for conf in config.get(CONF_ON_VALUE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], item)
        await automation.build_automation(trigger, [(MenuItemConstPtr, "it")], conf)
    for conf in config.get(CONF_ON_NEXT, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], item)
        await automation.build_automation(trigger, [(MenuItemConstPtr, "it")], conf)
    for conf in config.get(CONF_ON_PREV, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], item)
        await automation.build_automation(trigger, [(MenuItemConstPtr, "it")], conf)

    return item


async def add_sorted_items(menu, items_config, parent):
    """Create menu items and add them to parent sorted by weight and position.

    Ordering rules:
    1. Items are first sorted by weight (lower weight = appears first)
    2. Items with relative position (before/after) are then repositioned
    3. Items without weight default to 0
    4. Items with equal weights preserve their declaration order (stable sort)
    """
    # First pass: Create all items and collect metadata
    items_data = []
    id_to_index = {}  # Map config ID to index in items_data

    for index, c in enumerate(items_config):
        item_var = await menu_item_to_code(menu, c)
        weight = c.get(CONF_WEIGHT, 0)
        position = c.get(CONF_POSITION)
        item_id = c.get(CONF_ID)

        items_data.append(
            {
                "var": item_var,
                "weight": weight,
                "index": index,
                "position": position,
                "id": item_id,
            }
        )

        if item_id is not None:
            id_to_index[item_id] = index

    # Sort by weight (stable sort - preserves order for equal weights)
    items_data.sort(key=lambda x: (x["weight"], x["index"]))

    # Process relative positions (before/after)
    # Items with relative position are moved to be adjacent to their reference
    final_order = []
    pending_before = {}  # ref_id -> list of items to insert before
    pending_after = {}  # ref_id -> list of items to insert after

    for item in items_data:
        position = item["position"]
        if position is not None:
            if CONF_BEFORE in position:
                ref_id = position[CONF_BEFORE]
                pending_before.setdefault(ref_id, []).append(item)
            elif CONF_AFTER in position:
                ref_id = position[CONF_AFTER]
                pending_after.setdefault(ref_id, []).append(item)
        else:
            final_order.append(item)

    # Build final list with relative positions resolved
    result = []
    for item in final_order:
        item_id = item["id"]
        # Insert items that should be before this one
        if item_id in pending_before:
            result.extend(pending_before[item_id])
        # Insert this item
        result.append(item)
        # Insert items that should be after this one
        if item_id in pending_after:
            result.extend(pending_after[item_id])

    # Add any items whose reference wasn't found (append at end)
    for ref_id, items in pending_before.items():
        if ref_id not in id_to_index or id_to_index[ref_id] not in [
            i["index"] for i in final_order
        ]:
            result.extend(items)
    for ref_id, items in pending_after.items():
        if ref_id not in id_to_index or id_to_index[ref_id] not in [
            i["index"] for i in final_order
        ]:
            result.extend(items)

    # Add items in final order
    for item in result:
        cg.add(parent.add_item(item["var"]))


async def display_menu_to_code(menu, config):
    cg.add(menu.set_active(config[CONF_ACTIVE]))
    root_item = cg.new_Pvariable(config[CONF_ROOT_ITEM_ID])
    cg.add(menu.set_root_item(root_item))
    cg.add(menu.set_mode(config[CONF_MODE]))
    cg.add(menu.set_right_for_menu_enter_opt(config[CONF_RIGHT_FOR_MENU_ENTER]))

    # Add items sorted by weight
    await add_sorted_items(menu, config[CONF_ITEMS], root_item)

    for conf in config.get(CONF_ON_ENTER, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], root_item)
        await automation.build_automation(trigger, [(MenuItemConstPtr, "it")], conf)
    for conf in config.get(CONF_ON_LEAVE, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], root_item)
        await automation.build_automation(trigger, [(MenuItemConstPtr, "it")], conf)
