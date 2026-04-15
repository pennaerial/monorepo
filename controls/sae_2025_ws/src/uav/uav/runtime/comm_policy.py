from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from .comm_naming import ManagedCommKind, ManagedCommScope, ManagedTarget

ManagedCommLifetime = Literal["persistent", "active"]


@dataclass(frozen=True)
class ManagedRegistration:
    scope: ManagedCommScope
    peers: tuple[str, ...]


def managed_creation_phase_error(kind: ManagedCommKind, name: str) -> ValueError:
    return ValueError(
        f"Managed {kind} '{name}' may only be created during mode __init__() or on_enter()."
    )


def validate_managed_registration(
    *,
    kind: ManagedCommKind,
    target: ManagedTarget,
    allowed_peer_names: tuple[str, ...],
    lifetime: ManagedCommLifetime,
    owner_label: str,
) -> ManagedRegistration:
    if target.scope == "shared":
        if kind not in {"publisher", "subscription"}:
            raise ValueError(f"{kind} cannot target shared namespace '{target.name}'.")
        if kind == "publisher" and lifetime != "persistent":
            raise ValueError(
                f"Shared publisher '{target.name}' must be created in mode __init__()."
            )
        return ManagedRegistration(scope="shared", peers=allowed_peer_names)

    peer_name = target.peers[0]
    if peer_name not in allowed_peer_names:
        raise ValueError(
            f"Mode '{owner_label}' tried to create peer {kind} for undeclared peer '{peer_name}'."
        )
    if kind not in {"subscription", "client"}:
        raise ValueError(f"{kind} cannot target peer namespace '{target.name}'.")
    return ManagedRegistration(scope="peer", peers=(peer_name,))
