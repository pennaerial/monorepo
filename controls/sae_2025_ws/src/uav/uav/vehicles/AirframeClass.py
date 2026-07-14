from __future__ import annotations


from enum import IntEnum
from dataclasses import dataclass


class AirframeClass(IntEnum):
    """PX4 airframe classification used to choose a UAV implementation."""

    MULTICOPTER = 0
    PLANE = 1
    VTOL = 2
    OTHER = 3
    UNKNOWN = 4

    @classmethod
    def parse(cls, value):
        if isinstance(value, cls):
            return value
        if isinstance(value, str):
            try:
                return cls[value.upper()]
            except KeyError as exc:
                valid = ", ".join(member.name for member in cls)
                raise ValueError(
                    f"Invalid vehicle_class '{value}'. Expected one of: {valid}"
                ) from exc
        if isinstance(value, int):
            try:
                return cls(value)
            except ValueError as exc:
                valid = ", ".join(str(member.value) for member in cls)
                raise ValueError(
                    f"Invalid vehicle_class '{value}'. Expected one of: {valid}"
                ) from exc
        raise ValueError(
            f"Invalid vehicle_class type '{type(value)}'. Expected one of string, int, or {cls.__name__}."
        )


@dataclass(frozen=True, slots=True)
class PX4Airframe:
    id: int
    alias: str  # Optional. If empty there is no alias
    model: str
    airframe_class: AirframeClass

    @classmethod
    def lookup_airframe(cls, value: str) -> PX4Airframe:
        """Looks up airframe by either id, alias, or model
        if int --> id
        if str --> checks alias first then falls back to model.

        Throws KeyError if fails
        """
        if value.isdigit():
            return AIRFRAME_BY_ID[int(value)]

        if value in AIRFRAME_BY_ALIAS:
            return AIRFRAME_BY_ALIAS[value]
        return AIRFRAME_BY_MODEL[value]

    @classmethod
    def get_flying(cls) -> list[PX4Airframe]:
        return [
            a
            for a in AIRFRAMES
            if a.airframe_class
            in (AirframeClass.MULTICOPTER, AirframeClass.VTOL, AirframeClass.PLANE)
        ]

    def __str__(self):
        s = f"{self.alias}/" if self.alias else ""
        s += f"{self.id}/{self.model}"
        return s


# fmt: off
AIRFRAMES: list[PX4Airframe] = [
    # ==================================================================================
    #           ID      ALIAS            MODEL                AIRFRAME CLASS
    # ==================================================================================
    PX4Airframe(4001,  "quadcopter",    "x500",               AirframeClass.MULTICOPTER),
    PX4Airframe(4002,  "",              "x500_depth",         AirframeClass.MULTICOPTER),
    PX4Airframe(4003,  "",              "rc_cessna",          AirframeClass.PLANE),
    PX4Airframe(4004,  "",              "standard_vtol",      AirframeClass.VTOL),
    PX4Airframe(4005,  "",              "x500_vision",        AirframeClass.MULTICOPTER),
    PX4Airframe(4006,  "",              "px4vision",          AirframeClass.MULTICOPTER),
    PX4Airframe(4008,  "",              "advanced_plane",     AirframeClass.PLANE),
    PX4Airframe(4009,  "",              "r1_rover",           AirframeClass.OTHER),
    PX4Airframe(4010,  "mono_cam",      "x500_mono_cam",      AirframeClass.MULTICOPTER),
    PX4Airframe(4011,  "",              "lawnmower",          AirframeClass.OTHER),
    PX4Airframe(4013,  "",              "x500_lidar_2d",      AirframeClass.MULTICOPTER),
    PX4Airframe(4014,  "mono_cam_down", "x500_mono_cam_down", AirframeClass.MULTICOPTER),
    PX4Airframe(4016,  "",              "x500_lidar_down",    AirframeClass.MULTICOPTER),
    PX4Airframe(4017,  "",              "x500_lidar_front",   AirframeClass.MULTICOPTER),
    PX4Airframe(4018,  "",              "quadtailsitter",     AirframeClass.VTOL),
    PX4Airframe(4019,  "",              "x500_gimbal",        AirframeClass.MULTICOPTER),
    PX4Airframe(4020,  "",              "tiltrotor",          AirframeClass.VTOL),
    PX4Airframe(4021,  "",              "x500_flow",          AirframeClass.MULTICOPTER),
    PX4Airframe(50000, "",              "rover_differential", AirframeClass.OTHER),
    PX4Airframe(51000, "",              "rover_ackermann",    AirframeClass.OTHER),
    PX4Airframe(52000, "",              "rover_mecanum",      AirframeClass.OTHER),
    PX4Airframe(60002, "",              "uuv_bluerov2_heavy", AirframeClass.OTHER),
    PX4Airframe(70000, "",              "atmos",              AirframeClass.OTHER),
    PX4Airframe(8011,  "",              "omnicopter",         AirframeClass.MULTICOPTER),
]

AIRFRAME_BY_ID:    dict[int, PX4Airframe] = {airframe.id: airframe for airframe in AIRFRAMES}
AIRFRAME_BY_MODEL: dict[str, PX4Airframe] = {airframe.model: airframe for airframe in AIRFRAMES}
AIRFRAME_BY_ALIAS: dict[str, PX4Airframe] = {a.alias: a for a in AIRFRAMES if a.alias}
# fmt: on
