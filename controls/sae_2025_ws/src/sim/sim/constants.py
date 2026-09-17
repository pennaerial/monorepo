#!/usr/bin/env python3
"""
Constants and enums for the sim package.
"""

from enum import IntEnum


class Competition(IntEnum):
    """Competition type enumeration."""

    IN_HOUSE = 0
    IARC = 1
    CUSTOM = 2
    SAE = 3
    IN_HOUSE_2026 = 4


# Competition name mapping (values are the directory under simulations/ and the world .sdf stem)
COMPETITION_NAMES = {
    Competition.IN_HOUSE: "in_house",
    Competition.IARC: "iarc",
    Competition.CUSTOM: "custom",
    Competition.SAE: "sae",
    Competition.IN_HOUSE_2026: "in_house_2026",
}

# Platform name mapping
PLATFORM_NAMES = {
    "win": "x86",
    "linux": "arm",
    "darwin": "arm",  # Mac
}

# Default values
DEFAULT_COMPETITION = Competition.IN_HOUSE
DEFAULT_USE_SCORING = False
