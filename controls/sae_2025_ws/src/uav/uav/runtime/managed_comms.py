from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
from typing import Any, Callable, Iterator, Literal, Mapping

from .managed_entities import ManagedClient, ManagedEntity, ManagedPublisher

_SHARED_NAMESPACE_PREFIX = "/shared/"

ManagedCommKind = Literal["publisher", "subscription", "client"]
ManagedCommScope = Literal["peer", "shared"]
ManagedCommPhase = Literal["persistent", "active"]


@dataclass(frozen=True)
class ManagedCommContext:
    owner: object
    owner_label: str
    phase: ManagedCommPhase
    peer_vehicle_names: tuple[str, ...]


@dataclass
class ManagedCommDescriptor:
    kind: ManagedCommKind
    scope: ManagedCommScope
    phase: ManagedCommPhase
    owner_label: str
    interface_name: str
    name: str
    peers: tuple[str, ...]


@dataclass
class ManagedCommSpec:
    kind: ManagedCommKind
    scope: ManagedCommScope
    phase: ManagedCommPhase
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
            phase=self.phase,
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
        runtime_vehicle_name: str,
        connection_status: Callable[[], Mapping[str, bool]],
        raw_create_publisher: Callable[..., object],
        raw_create_subscription: Callable[..., object],
        raw_create_client: Callable[..., object],
        raw_create_service: Callable[..., object],
        raw_destroy_publisher: Callable[[object], bool | None],
        raw_destroy_subscription: Callable[[object], bool | None],
        raw_destroy_client: Callable[[object], bool | None],
    ) -> None:
        self._runtime_vehicle_name = runtime_vehicle_name
        self._connection_status = connection_status
        self._raw_create_publisher = raw_create_publisher
        self._raw_create_subscription = raw_create_subscription
        self._raw_create_client = raw_create_client
        self._raw_create_service = raw_create_service
        self._raw_destroy_publisher = raw_destroy_publisher
        self._raw_destroy_subscription = raw_destroy_subscription
        self._raw_destroy_client = raw_destroy_client

        self._context: ManagedCommContext | None = None
        self._entities: dict[int, ManagedEntity] = {}

    @property
    def descriptors(self) -> tuple[ManagedCommDescriptor, ...]:
        return tuple(wrapper.spec.descriptor() for wrapper in self._entities.values())

    @contextmanager
    def scope(
        self,
        *,
        owner: object,
        owner_label: str,
        phase: ManagedCommPhase,
        peer_vehicle_names: tuple[str, ...],
    ) -> Iterator[None]:
        previous_context = self._context
        self._context = ManagedCommContext(
            owner=owner,
            owner_label=owner_label,
            phase=phase,
            peer_vehicle_names=peer_vehicle_names,
        )
        try:
            yield
        finally:
            self._context = previous_context

    def bind_owner(self, old_owner: object, new_owner: object) -> None:
        for wrapper in self._entities.values():
            if wrapper.spec.owner is old_owner:
                wrapper.spec.owner = new_owner
                wrapper.spec.owner_label = type(new_owner).__name__

    def destroy_for_owner(
        self,
        owner: object,
        *,
        phase: ManagedCommPhase | None = None,
    ) -> None:
        for wrapper in list(self._entities.values()):
            if wrapper.spec.owner is not owner:
                continue
            if phase is not None and wrapper.spec.phase != phase:
                continue
            self._destroy_managed_entity(wrapper)

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

    def create_publisher(self, msg_type, topic: str, *args, **kwargs):
        scope, peers = self._resolve_scope("publisher", topic)
        if scope is None:
            return self._raw_create_publisher(msg_type, topic, *args, **kwargs)
        return self._register_managed_entity(
            "publisher",
            scope,
            msg_type,
            topic,
            args,
            kwargs,
            peers,
        )

    def create_subscription(self, msg_type, topic: str, *args, **kwargs):
        scope, peers = self._resolve_scope("subscription", topic)
        if scope is None:
            return self._raw_create_subscription(msg_type, topic, *args, **kwargs)
        return self._register_managed_entity(
            "subscription",
            scope,
            msg_type,
            topic,
            args,
            kwargs,
            peers,
        )

    def create_client(self, srv_type, srv_name: str, *args, **kwargs):
        scope, peers = self._resolve_scope("client", srv_name)
        if scope is None:
            return self._raw_create_client(srv_type, srv_name, *args, **kwargs)
        return self._register_managed_entity(
            "client",
            scope,
            srv_type,
            srv_name,
            args,
            kwargs,
            peers,
        )

    def create_service(self, srv_type, srv_name: str, *args, **kwargs):
        scope, _peers = self._resolve_scope("service", srv_name)
        if scope is None:
            return self._raw_create_service(srv_type, srv_name, *args, **kwargs)
        raise ValueError(
            f"create_service() does not support managed scope for '{srv_name}'."
        )

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

    def _resolve_scope(
        self,
        kind: Literal["publisher", "subscription", "client", "service"],
        name: str,
    ) -> tuple[ManagedCommScope | None, tuple[str, ...]]:
        normalized_name = str(name or "").strip()
        if not normalized_name:
            return None, ()

        context = self._context
        if normalized_name.startswith(_SHARED_NAMESPACE_PREFIX):
            if kind not in {"publisher", "subscription"}:
                raise ValueError(
                    f"{kind} cannot target shared namespace '{normalized_name}'."
                )
            if context is None:
                raise ValueError(
                    f"Shared {kind} '{normalized_name}' may only be created during "
                    "mode __init__() or on_enter()."
                )
            return "shared", context.peer_vehicle_names

        if not normalized_name.startswith("/"):
            return None, ()

        prefix = normalized_name[1:].split("/", 1)[0]
        if not prefix or prefix == self._runtime_vehicle_name:
            return None, ()
        if context is None:
            raise ValueError(
                f"Peer {kind} '{normalized_name}' may only be created during mode "
                "__init__() or on_enter()."
            )
        if prefix not in context.peer_vehicle_names:
            raise ValueError(
                f"Mode '{context.owner_label}' tried to create peer {kind} "
                f"for undeclared peer '{prefix}'."
            )
        if kind not in {"subscription", "client"}:
            raise ValueError(
                f"{kind} cannot target peer namespace '{normalized_name}'."
            )
        return "peer", (prefix,)

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

    def _register_managed_entity(
        self,
        kind: ManagedCommKind,
        scope: ManagedCommScope,
        interface_type,
        name: str,
        args: tuple[Any, ...],
        kwargs: dict[str, Any],
        peers: tuple[str, ...],
    ):
        context = self._context
        if context is None:
            raise ValueError(
                f"Managed {kind} '{name}' may only be created during mode "
                "__init__() or on_enter()."
            )
        if kind == "publisher" and scope == "shared" and context.phase != "persistent":
            raise ValueError(
                f"Shared publisher '{name}' must be created in mode __init__()."
            )
        spec = ManagedCommSpec(
            kind=kind,
            scope=scope,
            phase=context.phase,
            owner=context.owner,
            owner_label=context.owner_label,
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

    def _destroy_managed_entity(self, wrapper: ManagedEntity) -> bool:
        if id(wrapper) not in self._entities:
            return False
        self._destroy_managed_underlying(wrapper)
        wrapper.mark_destroyed()
        self._entities.pop(id(wrapper), None)
        return True
