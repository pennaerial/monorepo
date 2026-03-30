from enum import IntEnum


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
