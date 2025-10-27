import esphome.codegen as cg
from esphome.components import time as time_
import esphome.config_validation as cv
from esphome.const import CONF_ID

CONF_TIME_ID = "time_id"

automations_ns = cg.esphome_ns.namespace("automations")
AutomationStorageComponent = automations_ns.class_("AutomationStorage", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(AutomationStorageComponent),
        cv.Optional(CONF_TIME_ID): cv.use_id(time_.RealTimeClock),
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_TIME_ID in config:
        time_var = await cg.get_variable(config[CONF_TIME_ID])
        cg.add(var.set_time_source(time_var))
