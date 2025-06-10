import esphome.codegen as cg
from esphome.components import groups
from esphome.components.graphical_display_menu import GraphicalDisplayMenu
import esphome.config_validation as cv
from esphome.const import CONF_GROUPS, CONF_MENU_ID

DISPLAY_MENU_RENDER_BASE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_MENU_ID): cv.use_id(GraphicalDisplayMenu),
    }
).extend(groups.LIST_OF_GROUPS_SCHEMA)


async def render_to_code(var, config):
    menu_var = await cg.get_variable(config[CONF_MENU_ID])
    cg.add(menu_var.add_renderer(var))

    if group_config := config.get(CONF_GROUPS):
        await groups.add_groups_to_storage(var, group_config)
