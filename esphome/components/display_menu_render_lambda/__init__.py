import esphome.codegen as cg
from esphome.components import display_menu_base
import esphome.components.display_menu_render_base as render_base
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_LAMBDA
import esphome.cpp_types as core_types

display_menu_renderers_ns = cg.esphome_ns.namespace("display_menu_render_lambda")
LambdaMenuRender = display_menu_renderers_ns.class_("LambdaMenuRender")

LAMBDA_RENDER_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(LambdaMenuRender),
        cv.Required(CONF_LAMBDA): cv.lambda_,
    }
).extend(render_base.DISPLAY_MENU_RENDER_BASE_SCHEMA)

CONFIG_SCHEMA = cv.All(cv.ensure_list(LAMBDA_RENDER_SCHEMA), cv.Length(min=1))


async def to_code(config):
    for render in config:
        var = cg.new_Pvariable(render[CONF_ID])
        await render_base.render_to_code(var, render)

        if lambda_config := render.get(CONF_LAMBDA):
            lambda_ = await cg.process_lambda(
                lambda_config,
                [
                    (display_menu_base.MenuItemMenuPtr, "menu"),
                    (core_types.EntityBasePtr, "entity"),
                ],
                return_type=cg.size_t,
            )
            cg.add(var.set_lambda(lambda_))
