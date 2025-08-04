import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

user_names_ns = cg.esphome_ns.namespace("user_names")
UserNamesComponent = user_names_ns.class_("UserNamesComponent", cg.Component)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(UserNamesComponent),
        }
    )
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
