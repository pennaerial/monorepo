from __future__ import annotations

from typing import Literal, Protocol, TypeGuard, cast

DLZDirection = Literal["cw", "ccw"]
DLZStartPhase = Literal["wait_for_plane", "scan_tags", "line_follow"]

_DLZ_STATE_KEY = "payload.dlz_navigation"
_DIRECTION_KEY = "direction"

_VALID_DIRECTIONS: tuple[DLZDirection, ...] = ("cw", "ccw")
_VALID_START_PHASES: tuple[DLZStartPhase, ...] = (
    "wait_for_plane",
    "scan_tags",
    "line_follow",
)


class _SharedStateProvider(Protocol):
    def shared_state_for(self, mode_or_class: object) -> object: ...


def _supports_shared_state(node: object) -> TypeGuard[_SharedStateProvider]:
    return callable(getattr(node, "shared_state_for", None))


def _shared_state_for(node: object) -> dict[str, object]:
    if not _supports_shared_state(node):
        return {}
    state = node.shared_state_for(_DLZ_STATE_KEY)
    if isinstance(state, dict):
        return cast(dict[str, object], state)
    return {}


def parse_dlz_direction(value: object) -> DLZDirection | None:
    direction = str(value).lower().strip()
    if direction in _VALID_DIRECTIONS:
        return cast(DLZDirection, direction)
    return None


def parse_dlz_start_phase(value: object) -> DLZStartPhase | None:
    start_phase = str(value).lower().strip()
    if start_phase in _VALID_START_PHASES:
        return cast(DLZStartPhase, start_phase)
    return None


def set_dlz_navigation_direction(node: object, direction: DLZDirection) -> None:
    _shared_state_for(node)[_DIRECTION_KEY] = direction


def get_dlz_navigation_direction(node: object) -> DLZDirection | None:
    return parse_dlz_direction(_shared_state_for(node).get(_DIRECTION_KEY))
