from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .comm_naming import ManagedCommKind, ManagedTarget
from .comm_policy import ManagedCommLifetime, validate_managed_registration
from .managed_registry import ManagedCommDescriptor, ManagedCommRegistry


@dataclass(frozen=True)
class ModeCommBuilder:
    owner: object
    owner_label: str
    lifetime: ManagedCommLifetime
    allowed_peer_names: tuple[str, ...]
    registry: ManagedCommRegistry

    def create(
        self,
        *,
        kind: ManagedCommKind,
        interface_type: object,
        name: str,
        target: ManagedTarget,
        args: tuple[Any, ...],
        kwargs: dict[str, Any],
    ):
        registration = validate_managed_registration(
            kind=kind,
            target=target,
            allowed_peer_names=self.allowed_peer_names,
            lifetime=self.lifetime,
            owner_label=self.owner_label,
        )
        return self.registry.register(
            kind=kind,
            scope=registration.scope,
            lifetime=self.lifetime,
            owner=self.owner,
            owner_label=self.owner_label,
            interface_type=interface_type,
            name=name,
            args=args,
            kwargs=kwargs,
            peers=registration.peers,
        )


__all__ = (
    "ManagedCommDescriptor",
    "ManagedCommLifetime",
    "ManagedCommRegistry",
    "ModeCommBuilder",
)
