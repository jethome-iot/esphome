import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

dynamic_entity_parameters_ns = cg.esphome_ns.namespace("dynamic_entity_parameters")
UserNamesComponent = dynamic_entity_parameters_ns.class_(
    "UserNamesComponent", cg.Component
)


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
