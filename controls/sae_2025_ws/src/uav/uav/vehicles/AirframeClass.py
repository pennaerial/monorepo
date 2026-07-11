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
    model: str
    airframe_class: AirframeClass

    @classmethod
    def lookup_airframe(cls, value: int | str) -> PX4Airframe:
        if isinstance(value, int):
            return AIRFRAME_BY_ID[value]
        return AIRFRAME_BY_MODEL[value]


AIRFRAMES: list[PX4Airframe] = [
    PX4Airframe(4001, "gz_x500", AirframeClass.MULTICOPTER),
    PX4Airframe(4002, "gz_x500_depth", AirframeClass.MULTICOPTER),
    PX4Airframe(4003, "gz_rc_cessna", AirframeClass.PLANE),
    PX4Airframe(4004, "gz_standard_vtol", AirframeClass.VTOL),
    PX4Airframe(4005, "gz_x500_vision", AirframeClass.MULTICOPTER),
    PX4Airframe(4006, "gz_px4vision", AirframeClass.MULTICOPTER),
    PX4Airframe(4008, "gz_advanced_plane", AirframeClass.PLANE),
    PX4Airframe(4009, "gz_r1_rover", AirframeClass.OTHER),
    PX4Airframe(4010, "gz_x500_mono_cam", AirframeClass.MULTICOPTER),
    PX4Airframe(4011, "gz_lawnmower", AirframeClass.OTHER),
    PX4Airframe(4013, "gz_x500_lidar_2d", AirframeClass.MULTICOPTER),
    PX4Airframe(4014, "gz_x500_mono_cam_down", AirframeClass.MULTICOPTER),
    PX4Airframe(4016, "gz_x500_lidar_down", AirframeClass.MULTICOPTER),
    PX4Airframe(4017, "gz_x500_lidar_front", AirframeClass.MULTICOPTER),
    PX4Airframe(4018, "gz_quadtailsitter", AirframeClass.VTOL),
    PX4Airframe(4019, "gz_x500_gimbal", AirframeClass.MULTICOPTER),
    PX4Airframe(4020, "gz_tiltrotor", AirframeClass.VTOL),
    PX4Airframe(4021, "gz_x500_flow", AirframeClass.MULTICOPTER),
    PX4Airframe(50000, "gz_rover_differential", AirframeClass.OTHER),
    PX4Airframe(51000, "gz_rover_ackermann", AirframeClass.OTHER),
    PX4Airframe(52000, "gz_rover_mecanum", AirframeClass.OTHER),
    PX4Airframe(60002, "gz_uuv_bluerov2_heavy", AirframeClass.OTHER),
    PX4Airframe(70000, "gz_atmos", AirframeClass.OTHER),
    PX4Airframe(8011, "gz_omnicopter", AirframeClass.MULTICOPTER),
]

AIRFRAME_BY_ID: dict[int, PX4Airframe] = {
    airframe.id: airframe for airframe in AIRFRAMES
}

AIRFRAME_BY_MODEL: dict[str, PX4Airframe] = {
    airframe.model: airframe for airframe in AIRFRAMES
}

