"""Parametric involute spur gears built with CadQuery."""
from .gear import (GearSpec, build_gear, build_pair, center_distance,
                   contact_ratio, inv)

__all__ = ["GearSpec", "build_gear", "build_pair", "center_distance", "contact_ratio", "inv"]
