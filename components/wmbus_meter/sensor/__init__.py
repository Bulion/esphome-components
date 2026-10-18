from esphome import config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_UNIT_OF_MEASUREMENT

from .. import wmbus_meter_ns
from ..base_sensor import BASE_SCHEMA, register_meter, BaseSensor, CONF_FIELD
from ...wmbus_common.units import get_human_readable_unit, get_point_in_time_units


RegularSensor = wmbus_meter_ns.class_("Sensor", BaseSensor, sensor.Sensor)


def default_unit_of_measurement(config):
    config.setdefault(
        CONF_UNIT_OF_MEASUREMENT,
        get_human_readable_unit(config[CONF_FIELD].rsplit("_").pop()),
    )

    return config


def _reject_point_in_time_fields(config):
    """Raise an error when a PointInTime field (date/datetime/utc/time/ut) is
    configured as a numeric sensor.  These fields carry timestamps that HA
    cannot display correctly as numeric sensors; use text_sensor instead.

    The suffix is extracted by splitting on the last underscore, which mirrors
    what the C++ extractUnit() function does (std::string::rfind('_')).
    """
    suffix = config[CONF_FIELD].rsplit("_", 1)[-1].lower()
    if suffix in get_point_in_time_units():
        raise cv.Invalid(
            f"Field '{config[CONF_FIELD]}' is a date/time (PointInTime) field and "
            "cannot be used with the numeric 'sensor' platform — use 'text_sensor' "
            "instead so that Home Assistant receives a formatted date string."
        )
    return config


CONFIG_SCHEMA = cv.All(
    BASE_SCHEMA.extend(sensor.sensor_schema(RegularSensor)),
    _reject_point_in_time_fields,
    default_unit_of_measurement,
)


async def to_code(config):
    sensor_ = await sensor.new_sensor(config)
    await register_meter(sensor_, config)
