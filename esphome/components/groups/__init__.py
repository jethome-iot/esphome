import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_GROUPS, CONF_ID, CONF_NAME

GROUPS_STORAGE_ID = "groups_storage_id"

groups_ns = cg.esphome_ns.namespace("groups")
GroupClass = groups_ns.class_("Group")
GroupStorage = groups_ns.class_("GroupsStorage")

GROUP_BASE_SCHEMA = {
    cv.Required(CONF_ID): cv.declare_id(GroupClass),
    cv.Optional(CONF_NAME): cv.string,
}

GROUP_ID_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.use_id(cg.int_),
    }
)

EntityType = groups_ns.enum("EntityType", is_class=True)

ENTITY_TYPE_ENUM = {
    "none": EntityType.NONE,
    "switch": EntityType.SWITCH,
    "sensor": EntityType.SENSOR,
}

EntitySubtype = groups_ns.enum("EntitySubtype", is_class=True)
ENTITY_SUBTYPE_ENUM = {"none": EntitySubtype.NONE, "dallas": EntitySubtype.DALLAS}


def group_id(value):
    return GROUP_ID_SCHEMA(value)


LIST_OF_GROUPS_SCHEMA = cv.Schema(
    {cv.Optional(CONF_GROUPS): cv.All(cv.ensure_list(group_id), cv.Length(min=1))}
)

CONFIG_SCHEMA = cv.All(cv.ensure_list(GROUP_BASE_SCHEMA), cv.Length(min=1))


async def add_entity_config(
    entity, config, entity_type=EntityType.NONE, entity_subtype=EntitySubtype.NONE
):
    for group in config:
        group_var = await cg.get_variable(group[CONF_ID])
        cg.add(group_var.add_entity(entity, entity_type, entity_subtype))


async def add_groups_to_storage(storage_var, config):
    for group_config in config:
        group_var = await cg.get_variable(group_config[CONF_ID])
        cg.add(storage_var.add_group(group_var))


async def to_code(config):
    var = cg.new_Pvariable(cv.declare_id(GroupStorage)(GROUPS_STORAGE_ID))

    for group_config in config:
        group_var = cg.new_Pvariable(group_config[CONF_ID])
        cg.add(group_var.set_group_name(group_config[CONF_NAME]))
        cg.add(var.add_group(group_var))
    cg.add_define("USE_GROUPS")
