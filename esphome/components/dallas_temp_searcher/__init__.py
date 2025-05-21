import esphome.codegen as cg
from esphome.components import one_wire
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_UPDATE_INTERVAL

AUTO_LOAD = ["dallas_temp"]

dallas_temp_searcher_ns = cg.esphome_ns.namespace("dallas_temp_searcher")
DallasTempSearcherComponent = dallas_temp_searcher_ns.class_(
    "DallasTemperatureSearcher", cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(DallasTempSearcherComponent),
        cv.Optional(CONF_UPDATE_INTERVAL, default="60s"): cv.update_interval,
    }
).extend(one_wire.one_wire_device_schema())


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await one_wire.register_one_wire_device(var, config)
