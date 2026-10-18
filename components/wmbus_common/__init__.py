from pathlib import Path
import logging

import esphome.config_validation as cv
from esphome import codegen as cg
from esphome.const import CONF_ID

LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@SzczepanLeon", "@kubasaw"]
CONF_DRIVERS = "drivers"

wmbus_common_ns = cg.esphome_ns.namespace("wmbus_common")
WMBusCommon = wmbus_common_ns.class_("WMBusCommon", cg.Component)


AVAILABLE_DRIVERS = {
    f.stem.removeprefix("driver_") for f in Path(__file__).parent.glob("driver_*.cpp")
}
_selected_driver_names = set()


def validate_driver(value):
    """Validate meter driver name used by wmbus_meter.

    Accepts "auto" (let wmbusmeters pick based on telegram contents) or a
    specific driver from AVAILABLE_DRIVERS.
    """
    if value == "auto":
        return value
    return cv.one_of(*AVAILABLE_DRIVERS, lower=True, space="_")(value)


def _validate_drivers(value):
    drivers = set(AVAILABLE_DRIVERS) if value == "all" else set(value)
    _selected_driver_names.update(drivers)
    return drivers


def FILTER_SOURCE_FILES():
    if not _selected_driver_names:
        LOGGER.info(
            "wmbus_common: no driver selection; including all %d driver sources",
            len(AVAILABLE_DRIVERS),
        )
        return set()
    excluded = {
        f"driver_{driver}.cpp"
        for driver in AVAILABLE_DRIVERS - _selected_driver_names
    }
    LOGGER.info(
        "wmbus_common: excluding %d of %d driver sources; selected=%s",
        len(excluded),
        len(AVAILABLE_DRIVERS),
        sorted(_selected_driver_names),
    )
    return excluded


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(WMBusCommon),
        cv.Optional(CONF_DRIVERS, default=[]): cv.All(
            cv.Any(
                "all",
                cv.ensure_list(validate_driver),
            ),
            _validate_drivers,
        ),
    }
)


async def to_code(config):
    selected_drivers = set(config.get(CONF_DRIVERS, set()))
    drivers_to_link = selected_drivers or AVAILABLE_DRIVERS
    anchors = sorted(
        f"wmbus_anchor_driver_{driver.replace('-', '_')}"
        for driver in drivers_to_link
    )
    for anchor in anchors:
        cg.add_global(cg.RawStatement(f"extern void {anchor}(void);"))
    if anchors:
        references = ", ".join(f"&{anchor}" for anchor in anchors)
        cg.add_global(
            cg.RawStatement(
                "static void (*const wmbus_kept_drivers[])(void) "
                f"__attribute__((used)) = {{{references}}};"
            )
        )

    if selected_drivers:
        LOGGER.info("wmbus_common: selected_drivers=%s", sorted(selected_drivers))

    var = cg.new_Pvariable(config[CONF_ID], sorted(selected_drivers))
    await cg.register_component(var, config)
