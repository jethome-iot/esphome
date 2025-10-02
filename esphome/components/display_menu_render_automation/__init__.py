import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

display_menu_render_automation_ns = cg.esphome_ns.namespace(
    "display_menu_render_automation"
)
DisplayMenuRenderAutomation = display_menu_render_automation_ns.class_(
    "DisplayMenuRenderAutomation"
)


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(DisplayMenuRenderAutomation),
    }
)


async def to_code(config):
    cg.new_Pvariable(config[CONF_ID])
