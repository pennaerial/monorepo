from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Mapping

from .comm_naming import ManagedCommScope
from .comm_policy import ManagedCommLifetime
from .managed_entities import ManagedClient, ManagedEntity, ManagedPublisher

ManagedRegistryKind = "publisher", "subscription", "client"


@dataclass(frozen=True)
class ManagedCommDescriptor:
    kind: str
    scope: ManagedCommScope
    lifetime: ManagedCommLifetime
    owner_label: str
    interface_name: str
    name: str
    peers: tuple[str, ...]


@dataclass
class ManagedCommSpec:
    kind: str
    scope: ManagedCommScope
    lifetime: ManagedCommLifetime
    owner: object
    owner_label: str
    interface_type: object
    name: str
    args: tuple[Any, ...]
    kwargs: dict[str, Any]
    peers: tuple[str, ...]

    def descriptor(self) -> ManagedCommDescriptor:
        return ManagedCommDescriptor(
            kind=self.kind,
            scope=self.scope,
            lifetime=self.lifetime,
            owner_label=self.owner_label,
            interface_name=getattr(
                self.interface_type, "__name__", str(self.interface_type)
            ),
            name=self.name,
            peers=self.peers,
        )


class ManagedCommRegistry:
    def __init__(
        self,
        *,
        connection_status: Callable[[], Mapping[str, bool]],
        raw_create_publisher: Callable[..., Any],
        raw_create_subscription: Callable[..., Any],
        raw_create_client: Callable[..., Any],
        raw_destroy_publisher: Callable[..., bool | None],
        raw_destroy_subscription: Callable[..., bool | None],
        raw_destroy_client: Callable[..., bool | None],
    ) -> None:
        self._connection_status = connection_status
        self._raw_create_publisher = raw_create_publisher
        self._raw_create_subscription = raw_create_subscription
        self._raw_create_client = raw_create_client
        self._raw_destroy_publisher = raw_destroy_publisher
        self._raw_destroy_subscription = raw_destroy_subscription
        self._raw_destroy_client = raw_destroy_client
        self._entities: dict[int, ManagedEntity] = {}

    def debug_descriptors(self) -> tuple[ManagedCommDescriptor, ...]:
        return tuple(wrapper.spec.descriptor() for wrapper in self._entities.values())

    def register(
        self,
        *,
        kind: str,
        scope: ManagedCommScope,
        lifetime: ManagedCommLifetime,
        owner: object,
        owner_label: str,
        interface_type: object,
        name: str,
        args: tuple[Any, ...],
        kwargs: dict[str, Any],
        peers: tuple[str, ...],
    ):
        spec = ManagedCommSpec(
            kind=kind,
            scope=scope,
            lifetime=lifetime,
            owner=owner,
            owner_label=owner_label,
            interface_type=interface_type,
            name=str(name),
            args=args,
            kwargs=dict(kwargs),
            peers=tuple(peers),
        )
        wrapper = self._make_wrapper(spec)
        self._entities[id(wrapper)] = wrapper
        self._ensure_managed_underlying(wrapper)
        return wrapper

    def destroy_for_owner(
        self,
        owner: object,
        *,
        lifetime: ManagedCommLifetime | None = None,
    ) -> None:
        for wrapper in list(self._entities.values()):
            if wrapper.spec.owner is not owner:
                continue
            if lifetime is not None and wrapper.spec.lifetime != lifetime:
                continue
            self._destroy_managed_entity(wrapper)

    def destroy_publisher(self, publisher) -> bool:
        if isinstance(publisher, ManagedEntity) and publisher.spec.kind == "publisher":
            return self._destroy_managed_entity(publisher)
        return bool(self._raw_destroy_publisher(publisher))

    def destroy_subscription(self, subscription) -> bool:
        if (
            isinstance(subscription, ManagedEntity)
            and subscription.spec.kind == "subscription"
        ):
            return self._destroy_managed_entity(subscription)
        return bool(self._raw_destroy_subscription(subscription))

    def destroy_client(self, client) -> bool:
        if isinstance(client, ManagedEntity) and client.spec.kind == "client":
            return self._destroy_managed_entity(client)
        return bool(self._raw_destroy_client(client))

    def on_peer_connection_change(self, peer_name: str, *, connected: bool) -> None:
        for wrapper in list(self._entities.values()):
            if wrapper.destroyed or peer_name not in wrapper.spec.peers:
                continue
            if wrapper.spec.scope == "peer":
                if connected:
                    self._ensure_managed_underlying(wrapper)
                else:
                    self._destroy_managed_underlying(wrapper)
                continue
            if connected and wrapper.spec.scope == "shared":
                # Hardware testing showed shared endpoints need explicit rebind
                # when a remote unit rejoins the network.
                self._refresh_managed_underlying(wrapper)

    def close(self) -> None:
        for wrapper in list(self._entities.values()):
            self._destroy_managed_entity(wrapper)

    def _make_wrapper(self, spec: ManagedCommSpec) -> ManagedEntity:
        if spec.kind == "publisher":
            return ManagedPublisher(spec)
        if spec.kind == "client":
            return ManagedClient(spec)
        return ManagedEntity(spec)

    def _create_managed_underlying(self, wrapper: ManagedEntity):
        spec = wrapper.spec
        if spec.kind == "publisher":
            return self._raw_create_publisher(
                spec.interface_type,
                spec.name,
                *spec.args,
                **spec.kwargs,
            )
        if spec.kind == "subscription":
            return self._raw_create_subscription(
                spec.interface_type,
                spec.name,
                *spec.args,
                **spec.kwargs,
            )
        return self._raw_create_client(
            spec.interface_type,
            spec.name,
            *spec.args,
            **spec.kwargs,
        )

    def _destroy_managed_underlying(self, wrapper: ManagedEntity) -> None:
        underlying = wrapper.detach()
        if underlying is None:
            return
        if wrapper.spec.kind == "publisher":
            self._raw_destroy_publisher(underlying)
        elif wrapper.spec.kind == "subscription":
            self._raw_destroy_subscription(underlying)
        else:
            self._raw_destroy_client(underlying)

    def _should_have_managed_underlying(self, wrapper: ManagedEntity) -> bool:
        if wrapper.destroyed:
            return False
        if wrapper.spec.scope == "shared":
            return True
        connection_status = self._connection_status()
        return all(
            bool(connection_status.get(peer_name, False))
            for peer_name in wrapper.spec.peers
        )

    def _ensure_managed_underlying(self, wrapper: ManagedEntity) -> None:
        if not self._should_have_managed_underlying(wrapper):
            return
        if wrapper.get_underlying() is not None:
            return
        wrapper.attach(self._create_managed_underlying(wrapper))

    def _refresh_managed_underlying(self, wrapper: ManagedEntity) -> None:
        if wrapper.destroyed:
            return
        self._destroy_managed_underlying(wrapper)
        self._ensure_managed_underlying(wrapper)

    def _destroy_managed_entity(self, wrapper: ManagedEntity) -> bool:
        if id(wrapper) not in self._entities:
            return False
        self._destroy_managed_underlying(wrapper)
        wrapper.mark_destroyed()
        self._entities.pop(id(wrapper), None)
        return True
