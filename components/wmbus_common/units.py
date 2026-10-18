from pathlib import Path
import string

_units_dict = {}
_point_in_time_units: set[str] = set()


def _build_dicts():
    """Builds dictionaries of units from the units.h file."""
    with Path(__file__).with_name("units.h").open("r", encoding="utf-8") as file:
        for line in file:
            if "LIST_OF_UNITS" in line:
                break

        for line in file:
            if line.strip() == "":
                break

            line = (
                line.strip(string.whitespace + "\\")
                .removeprefix("X(")
                .removesuffix(")")
            )
            if line:
                parts = line.split(",")
                if len(parts) == 5:
                    cname, lcname, hrname, quantity, explanation = parts
                    # Preserve original key/value format used by get_human_readable_unit.
                    # lcname has a leading space from the CSV format (e.g., " date"),
                    # which matches the original dict key format.
                    _units_dict[lcname] = hrname.strip('"')
                    if quantity.strip() == "PointInTime":
                        # Strip the leading space so the set contains clean strings
                        # (e.g., "date" not " date") for suffix comparisons.
                        _point_in_time_units.add(lcname.strip())


def _ensure_built():
    if not _units_dict:
        _build_dicts()


def get_human_readable_unit(unit: str):
    _ensure_built()
    return _units_dict.get(unit.lower(), unit or "?")


def get_point_in_time_units() -> set[str]:
    """Returns the set of unit lcnames that belong to Quantity::PointInTime."""
    _ensure_built()
    return _point_in_time_units
