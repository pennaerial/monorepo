from __future__ import annotations

from typing import Literal, cast

DLZDirection = Literal["cw", "ccw"]
DLZStartPhase = Literal["wait_for_plane", "scan_tags", "line_follow"]

_DLZ_STATE_KEY = "payload.dlz_navigation"
_DIRECTION_KEY = "direction"


def _shared_state_for(node: object) -> dict[str, object]:
    shared_state_for = getattr(node, "shared_state_for", None)
    if not callable(shared_state_for):
        raise TypeError("DLZ navigation state requires node.shared_state_for().")
    state = shared_state_for(_DLZ_STATE_KEY)
    if isinstance(state, dict):
        return cast(dict[str, object], state)
    raise TypeError(
        f"DLZ navigation shared state must be a dict, got {type(state).__name__}."
    )


def parse_dlz_direction(value: object) -> DLZDirection | None:
    direction = str(value).lower().strip()
    if direction == "cw":
        return "cw"
    if direction == "ccw":
        return "ccw"
    return None


def parse_dlz_start_phase(value: object) -> DLZStartPhase | None:
    start_phase = str(value).lower().strip()
    if start_phase == "wait_for_plane":
        return "wait_for_plane"
    if start_phase == "scan_tags":
        return "scan_tags"
    if start_phase == "line_follow":
        return "line_follow"
    return None


def set_dlz_navigation_direction(node: object, direction: DLZDirection) -> None:
    _shared_state_for(node)[_DIRECTION_KEY] = direction


def get_dlz_navigation_direction(node: object) -> DLZDirection | None:
    return parse_dlz_direction(_shared_state_for(node).get(_DIRECTION_KEY))
