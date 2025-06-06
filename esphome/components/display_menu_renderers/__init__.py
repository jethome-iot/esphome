import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ALL, CONF_SENSOR

CONF_RENDERERS = "renderers"
CONF_DALLAS_TEMP = "dallas_temp"
CONF_SWITCH = "switch"


display_menu_renderers_ns = cg.esphome_ns.namespace("display_menu_renderers")

SwitchMenuRender = display_menu_renderers_ns.class_("SwitchMenuRender")
SensorMenuRender = display_menu_renderers_ns.class_("SensorMenuRender")
DallasTempMenuRender = display_menu_renderers_ns.class_("DallasTempMenuRender")

MENU_ITEMS_RENDERERS = {
    CONF_SWITCH: SwitchMenuRender,
    CONF_SENSOR: SensorMenuRender,
    CONF_DALLAS_TEMP: DallasTempMenuRender,
}

DISPLAY_MENU_RENDERERS_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_RENDERERS): cv.All(
            cv.ensure_list(cv.one_of(*MENU_ITEMS_RENDERERS, CONF_ALL, lower=True)),
            cv.Length(min=1),
        ),
    }
)


def renderer_to_code(menu, key):
    type = MENU_ITEMS_RENDERERS[key]
    declare_id_func = cv.declare_id(type)

    render_id = declare_id_func(key + "_renderer_id")
    render_id.type = type
    item = cg.new_Pvariable(render_id)
    cg.add(menu.add_renderer(item))


def renderers_to_code(menu_var, config):
    if config[0] == CONF_ALL:
        for key in MENU_ITEMS_RENDERERS.keys():
            renderer_to_code(menu_var, key)
    else:
        for renderer in config:
            renderer_to_code(menu_var, renderer)
