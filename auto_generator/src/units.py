"""
Python equivalent of the auto-generated Units.ts.

Provides unit types and measure creation functions that serialize
to the same JSON format the Java robot code expects.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class Measure:
    """A measure with a numeric value and a unit name string."""
    value: float
    unit: str

    def to_dict(self) -> dict:
        return {"value": self.value, "unit": self.unit}


@dataclass
class Unit:
    """A named unit that can create Measure instances."""
    name: str

    def of(self, value: float) -> Measure:
        return Measure(value=value, unit=self.name)


# ---------------------------------------------------------------------------
# Unit instances (matching Units.ts exports)
# ---------------------------------------------------------------------------

# Time
Second = Unit("Second")
Millisecond = Unit("Millisecond")
Microsecond = Unit("Microsecond")
Minute = Unit("Minute")

# Distance
Meter = Unit("Meter")
Centimeter = Unit("Centimeter")
Millimeter = Unit("Millimeter")
Inch = Unit("Inch")
Foot = Unit("Foot")

# Angle
Degree = Unit("Degree")
Radian = Unit("Radian")
Rotation = Unit("Rotation")
Revolution = Unit("Revolution")

# Voltage
Volt = Unit("Volt")
Millivolt = Unit("Millivolt")

# Current
Amp = Unit("Amp")
Milliamp = Unit("Milliamp")

# Angular velocity
Degree_per_Second = Unit("Degree per Second")
Radian_per_Second = Unit("Radian per Second")
Rotation_per_Second = Unit("Rotation per Second")
Rotation_per_Minute = Unit("Rotation per Minute")
Revolution_per_Second = Unit("Revolution per Second")

# Linear velocity
Meter_per_Second = Unit("Meter per Second")
Foot_per_Second = Unit("Foot per Second")
Inch_per_Second = Unit("Inch per Second")

# Linear acceleration
Meter_per_Second_per_Second = Unit("Meter per Second per Second")
Foot_per_Second_per_Second = Unit("Foot per Second per Second")
G = Unit("G")

# Angular acceleration
Degree_per_Second_per_Second = Unit("Degree per Second per Second")
Radian_per_Second_per_Second = Unit("Radian per Second per Second")
Rotation_per_Second_per_Second = Unit("Rotation per Second per Second")
Rotation_per_Minute_per_Second = Unit("Rotation per Minute per Second")

# Frequency
Hertz = Unit("Hertz")
Millihertz = Unit("Millihertz")

# Force
Newton = Unit("Newton")
Ounce_force = Unit("Ounce-force")
Pound_force = Unit("Pound-force")

# Torque
Meter_Newton = Unit("Meter-Newton")
Foot_Pound_force = Unit("Foot-Pound-force")
Inch_Pound_force = Unit("Inch-Pound-force")
Inch_Ounce_force = Unit("Inch-Ounce-force")

# Mass
Kilogram = Unit("Kilogram")
Gram = Unit("Gram")
Pound = Unit("Pound")
Ounce = Unit("Ounce")

# Energy
Joule = Unit("Joule")
Millijoule = Unit("Millijoule")
Kilojoule = Unit("Kilojoule")

# Power
Watt = Unit("Watt")
Milliwatt = Unit("Milliwatt")
Horsepower = Unit("Horsepower")

# Temperature
Celsius = Unit("Celsius")
Fahrenheit = Unit("Fahrenheit")
Kelvin = Unit("Kelvin")

# Resistance
Ohm = Unit("Ohm")
Milliohm = Unit("Milliohm")
Kiloohm = Unit("Kiloohm")

# Dimensionless
Percent = Unit("Percent")
Unitless = Unit("<?>")

# Voltage per acceleration
Volt_per_Meter_per_Second = Unit("Volt per Meter per Second")
Volt_per_Meter_per_Second_per_Second = Unit("Volt per Meter per Second per Second")
Volt_per_Radian_per_Second = Unit("Volt per Radian per Second")
Volt_per_Radian_per_Second_per_Second = Unit("Volt per Radian per Second per Second")

# Momentum
Kilogram_Meter_per_Second = Unit("Kilogram-Meter per Second")
Kilogram_Meter_per_Second_Meter = Unit("Kilogram-Meter per Second-Meter")
Kilogram_Meter_per_Second_Meter_per_Radian_per_Second = Unit(
    "Kilogram-Meter per Second-Meter per Radian per Second"
)

# Other
Amp_per_Second = Unit("Amp per Second")
Rotation_per_Second_per_Second_per_Second = Unit("Rotation per Second per Second per Second")
