from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

_SHARED_NAMESPACE_PREFIX = "/shared/"

ManagedCommKind = Literal["publisher", "subscription", "client", "service"]
ManagedCommScope = Literal["peer", "shared"]


@dataclass(frozen=True)
class ManagedTarget:
    scope: ManagedCommScope
    name: str
    peers: tuple[str, ...]


def resolve_managed_target(
    *,
    name: str,
    runtime_vehicle_name: str,
) -> ManagedTarget | None:
    normalized_name = str(name or "").strip()
    if not normalized_name:
        return None

    if normalized_name.startswith(_SHARED_NAMESPACE_PREFIX):
        return ManagedTarget(scope="shared", name=normalized_name, peers=())

    if not normalized_name.startswith("/"):
        return None

    prefix = normalized_name[1:].split("/", 1)[0]
    if not prefix or prefix == runtime_vehicle_name:
        return None
    return ManagedTarget(scope="peer", name=normalized_name, peers=(prefix,))
