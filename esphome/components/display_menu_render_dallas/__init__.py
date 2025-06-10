import esphome.codegen as cg
import esphome.components.display_menu_render_base as render_base
import esphome.config_validation as cv
from esphome.const import CONF_ID

display_menu_renderers_ns = cg.esphome_ns.namespace("display_menu_render_dallas")
DallasTempMenuRender = display_menu_renderers_ns.class_("DallasTempMenuRender")


AUTO_LOAD = ["display_menu_render_sensor", "dallas_temp"]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(DallasTempMenuRender),
    }
).extend(render_base.DISPLAY_MENU_RENDER_BASE_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await render_base.render_to_code(var, config)
