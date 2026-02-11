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

# Competition name mapping
COMPETITION_NAMES = {
    Competition.IN_HOUSE: "in_house",
    Competition.IARC: "iarc",
    Competition.CUSTOM: "custom",
    Competition.SAE: "sae",
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
