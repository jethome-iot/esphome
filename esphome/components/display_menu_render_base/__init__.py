import esphome.codegen as cg
from esphome.components.graphical_display_menu import GraphicalDisplayMenu
import esphome.config_validation as cv
from esphome.const import CONF_MENU_ID

DISPLAY_MENU_RENDER_BASE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_MENU_ID): cv.use_id(GraphicalDisplayMenu),
    }
)


async def render_to_code(var, config):
    menu_var = await cg.get_variable(config[CONF_MENU_ID])
    cg.add(menu_var.add_renderer(var))
