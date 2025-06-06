import esphome.codegen as cg
from esphome.components.graphical_display_menu import GraphicalDisplayMenu
import esphome.config_validation as cv
from esphome.const import CONF_ID

display_menu_renderers_ns = cg.esphome_ns.namespace("display_menu_renderers")
DallasTempMenuRender = display_menu_renderers_ns.class_("DallasTempMenuRender")

CONF_MENU_ID = "menu_id"

AUTOLOAD = ["display_menu_render_sensor"]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(DallasTempMenuRender),
        cv.Required(CONF_MENU_ID): cv.use_id(GraphicalDisplayMenu),
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    menu_var = await cg.get_variable(config[CONF_MENU_ID])
    cg.add(menu_var.add_renderer(var))
