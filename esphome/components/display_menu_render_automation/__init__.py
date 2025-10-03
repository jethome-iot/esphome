import esphome.codegen as cg
from esphome.components import display_menu_base
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_MENU_ID

AUTO_LOAD = ["simple"]

display_menu_render_automation_ns = cg.esphome_ns.namespace(
    "display_menu_render_automation"
)
DisplayMenuRenderAutomation = display_menu_render_automation_ns.class_(
    "DisplayMenuRenderAutomation"
)


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(DisplayMenuRenderAutomation),
        cv.Required(CONF_MENU_ID): cv.use_id(display_menu_base.DisplayMenuComponent),
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    menu = await cg.get_variable(config[CONF_MENU_ID])
    cg.add(var.set_menu(menu))
