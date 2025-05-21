import esphome.codegen as cg
from esphome.components import one_wire
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_MODE, CONF_UPDATE_INTERVAL

AUTO_LOAD = ["dallas_temp"]

dallas_temp_searcher_ns = cg.esphome_ns.namespace("dallas_temp_searcher")
DallasTempSearcherComponent = dallas_temp_searcher_ns.class_(
    "DallasTemperatureSearcher", cg.Component
)

CONF_MAX_SENSORS_NUM = "maximum_sensors"

search_mode = dallas_temp_searcher_ns.enum("SearchMode", is_class=True)

SEARCH_MODE = {
    "all": search_mode.ALL,
    "address_map": search_mode.ADDRESS_MAP,
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(DallasTempSearcherComponent),
        cv.Optional(CONF_UPDATE_INTERVAL, default="60s"): cv.update_interval,
        cv.Optional(CONF_MAX_SENSORS_NUM, default=6): cv.int_range(0, 32),
        cv.Optional(CONF_MODE, default="all"): cv.enum(SEARCH_MODE, lower=True),
    }
).extend(one_wire.one_wire_device_schema())


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await one_wire.register_one_wire_device(var, config)

    cg.add(var.set_max_sensors_num(config[CONF_MAX_SENSORS_NUM]))
    cg.add(var.set_search_mode(config[CONF_MODE]))
