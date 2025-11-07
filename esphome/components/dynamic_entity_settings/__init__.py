import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.core import ID

dynamic_entity_settings_ns = cg.esphome_ns.namespace("dynamic_entity_settings")
EntitySettingsKeeper = dynamic_entity_settings_ns.class_(
    "EntitySettingsKeeper", cg.Component
)

SettingsBaseInterface = dynamic_entity_settings_ns.class_("SettingsBaseInterface")

SwitchSettingsVer1 = dynamic_entity_settings_ns.class_(
    "SwitchSettingsVer1", SettingsBaseInterface
)

BinarySensorSettingsVer1 = dynamic_entity_settings_ns.class_(
    "BinarySensorSettingsVer1", SettingsBaseInterface
)

SensorSettingsVer1 = dynamic_entity_settings_ns.class_(
    "SensorSettingsVer1", SettingsBaseInterface
)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(EntitySettingsKeeper),
        }
    )
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    switch_setting_ver1 = cg.new_Pvariable(
        ID(id="switch_setting_v1_ptr", type=SwitchSettingsVer1)
    )

    binary_sensor_setting_ver1 = cg.new_Pvariable(
        ID(id="binary_sensor_setting_v1_ptr", type=BinarySensorSettingsVer1)
    )

    sensor_setting_ver1 = cg.new_Pvariable(
        ID(id="sensor_setting_v1_ptr", type=SensorSettingsVer1)
    )

    cg.add(var.add_settings_list([switch_setting_ver1]))
    cg.add(var.add_settings_list([binary_sensor_setting_ver1]))
    cg.add(var.add_settings_list([sensor_setting_ver1]))
