import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

automations_ns = cg.esphome_ns.namespace("automations")
AutomationLoaderComponent = automations_ns.class_("AutomationLoader", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(AutomationLoaderComponent),
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
