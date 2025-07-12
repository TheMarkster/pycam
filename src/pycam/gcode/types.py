from __future__ import annotations
from enum import Enum

class Unit(Enum):
    MM = 'mm'
    INCH = 'inch'

    def convert(self, value: float, other: Unit) -> float:
        if self == other:
            return value
        if self == Unit.MM and other == Unit.INCH:
            return value / 25.4
        if self == Unit.INCH and other == Unit.MM:
            return value * 25.4
        raise ValueError("Unknown unit conversion")