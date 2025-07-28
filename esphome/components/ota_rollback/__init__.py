import esphome.codegen as cg
from esphome.components import esp32
import esphome.config_validation as cv
from esphome.const import CONF_ID

AUTO_LOAD = ["ota"]

ota_ns = cg.esphome_ns.namespace("ota_rollback")
OTARollback = ota_ns.class_("OTARollback", cg.Component)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(OTARollback),
        }
    ),
    cv.only_with_esp_idf,
)


async def to_code(config):
    esp32.add_idf_sdkconfig_option(
        "CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE",
        True,
    )
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
