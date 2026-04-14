from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal

ManagedEntityKind = Literal["publisher", "subscription", "client"]
ManagedEntityScope = Literal["peer", "shared"]
ManagedEntityPhase = Literal["construct", "activate"]


@dataclass
class ManagedEntityDescriptor:
    kind: ManagedEntityKind
    scope: ManagedEntityScope
    phase: ManagedEntityPhase
    owner_label: str
    interface_name: str
    name: str
    peers: tuple[str, ...]


@dataclass
class ManagedEntitySpec:
    kind: ManagedEntityKind
    scope: ManagedEntityScope
    phase: ManagedEntityPhase
    owner: object
    owner_label: str
    interface_type: object
    name: str
    args: tuple[Any, ...]
    kwargs: dict[str, Any]
    peers: tuple[str, ...]

    def descriptor(self) -> ManagedEntityDescriptor:
        return ManagedEntityDescriptor(
            kind=self.kind,
            scope=self.scope,
            phase=self.phase,
            owner_label=self.owner_label,
            interface_name=getattr(
                self.interface_type, "__name__", str(self.interface_type)
            ),
            name=self.name,
            peers=self.peers,
        )


class ManagedEntity:
    def __init__(self, spec: ManagedEntitySpec):
        self.spec = spec
        self._entity = None
        self._destroyed = False

    @property
    def destroyed(self) -> bool:
        return self._destroyed

    def attach(self, entity: object) -> None:
        self._entity = entity

    def detach(self):
        entity = self._entity
        self._entity = None
        return entity

    def mark_destroyed(self) -> None:
        self._destroyed = True
        self._entity = None

    def get_underlying(self):
        return self._entity

    def __getattr__(self, name: str):
        entity = self._entity
        if entity is None:
            raise AttributeError(name)
        return getattr(entity, name)


class ManagedPublisher(ManagedEntity):
    def publish(self, message) -> None:
        entity = self.get_underlying()
        if entity is None or self.destroyed:
            return None
        return entity.publish(message)


class ManagedSubscription(ManagedEntity):
    pass


class ManagedClient(ManagedEntity):
    def wait_for_service(self, timeout_sec: float = 0.0) -> bool:
        entity = self.get_underlying()
        if entity is None or self.destroyed:
            return False
        wait_for_service = getattr(entity, "wait_for_service", None)
        if not callable(wait_for_service):
            return False
        return bool(wait_for_service(timeout_sec=timeout_sec))

    def call_async(self, request):
        entity = self.get_underlying()
        if entity is None or self.destroyed:
            return None
        if not self.wait_for_service(timeout_sec=0.0):
            return None
        return entity.call_async(request)
