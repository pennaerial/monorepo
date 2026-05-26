from __future__ import annotations

from typing import Literal, cast

DLZDirection = Literal["cw", "ccw"]
DLZStartPhase = Literal["wait_for_plane", "scan_tags", "line_follow"]

_DLZ_STATE_KEY = "payload.dlz_navigation"
_DIRECTION_KEY = "direction"


def _shared_state_for(node: object) -> dict[str, object]:
    shared_state_for = getattr(node, "shared_state_for", None)
    if not callable(shared_state_for):
        return {}
    state = shared_state_for(_DLZ_STATE_KEY)
    if isinstance(state, dict):
        return cast(dict[str, object], state)
    return {}


def set_dlz_navigation_direction(node: object, direction: DLZDirection) -> None:
    _shared_state_for(node)[_DIRECTION_KEY] = direction


def get_dlz_navigation_direction(node: object) -> DLZDirection | None:
    value = _shared_state_for(node).get(_DIRECTION_KEY)
    if value in ("cw", "ccw"):
        return cast(DLZDirection, value)
    return None
