import esphome.codegen as cg
from esphome.components import update
import esphome.config_validation as cv
from esphome.const import CONF_SOURCE

from .. import CONF_HTTP_REQUEST_ID, HttpRequestComponent, http_request_ns
from ..ota import OtaHttpRequestComponent

AUTO_LOAD = ["json"]
CODEOWNERS = ["@jesserockz"]
DEPENDENCIES = ["ota.http_request"]

HttpRequestUpdate = http_request_ns.class_(
    "HttpRequestUpdate", update.UpdateEntity, cg.PollingComponent
)

UpdateManifestParserInterface = http_request_ns.class_("UpdateManifestParserInterface")
DefaultUpdateManifestParser = http_request_ns.class_(
    "DefaultUpdateManifestParser", UpdateManifestParserInterface
)

CONF_OTA_ID = "ota_id"
CONF_PARSER_ID = "parser_id"

CONFIG_SCHEMA = (
    update.update_schema(HttpRequestUpdate)
    .extend(
        {
            cv.GenerateID(CONF_OTA_ID): cv.use_id(OtaHttpRequestComponent),
            cv.GenerateID(CONF_HTTP_REQUEST_ID): cv.use_id(HttpRequestComponent),
            cv.Required(CONF_SOURCE): cv.url,
            cv.Optional(CONF_PARSER_ID): cv.use_id(UpdateManifestParserInterface),
        }
    )
    .extend(cv.polling_component_schema("6h"))
)


async def to_code(config):
    var = await update.new_update(config)
    ota_parent = await cg.get_variable(config[CONF_OTA_ID])
    cg.add(var.set_ota_parent(ota_parent))
    request_parent = await cg.get_variable(config[CONF_HTTP_REQUEST_ID])
    cg.add(var.set_request_parent(request_parent))

    cg.add(var.set_source_url(config[CONF_SOURCE]))

    cg.add_define("USE_OTA_STATE_CALLBACK")

    await cg.register_component(var, config)

    parser = None
    if parser_id := config.get(CONF_PARSER_ID):
        parser = await cg.get_variable(parser_id)
    else:
        parser = cg.new_Pvariable(
            cv.declare_id(DefaultUpdateManifestParser)(
                cv.GenerateID("update_manifest_parser_id")
            )
        )

    cg.add(var.set_manifest_parser(parser))
