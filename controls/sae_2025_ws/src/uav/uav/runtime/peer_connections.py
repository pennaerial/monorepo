from __future__ import annotations

from collections.abc import Iterable
from typing import Any, Callable, Mapping

from std_msgs.msg import Empty

HEARTBEAT_TOPIC_SUFFIX = "mode_manager/heartbeat"


def normalize_vehicle_name(vehicle_name: str | None) -> str:
    return str(vehicle_name or "").strip().strip("/")


def normalize_peer_vehicle_names(
    peer_vehicle_names: tuple[str, ...] | list[str] | set[str] | None,
) -> tuple[str, ...]:
    return tuple(
        sorted(
            {
                normalize_vehicle_name(peer_name)
                for peer_name in tuple(peer_vehicle_names or ())
                if normalize_vehicle_name(peer_name)
            }
        )
    )


def declared_remote_peer_names(
    peer_vehicle_names: object,
    runtime_vehicle_name: str,
) -> tuple[str, ...]:
    if hasattr(peer_vehicle_names, "peer_vehicle_names"):
        raw_peer_names = getattr(peer_vehicle_names, "peer_vehicle_names", ())
    else:
        raw_peer_names = (
            peer_vehicle_names
            if isinstance(peer_vehicle_names, (tuple, list, set))
            else ()
        )
    return tuple(
        peer_name
        for peer_name in normalize_peer_vehicle_names(raw_peer_names)
        if peer_name != normalize_vehicle_name(runtime_vehicle_name)
    )


def relevant_connection_status(
    connection_status: Mapping[str, bool],
    *,
    peer_vehicle_names: Iterable[str],
) -> dict[str, bool]:
    return {
        peer_name: bool(connection_status.get(peer_name, False))
        for peer_name in tuple(peer_vehicle_names)
    }


class PeerConnectionTracker:
    def __init__(
        self,
        *,
        runtime_vehicle_name: str = "",
        peer_heartbeat_hz: float = 10.0,
        peer_stale_timeout_s: float = 0.5,
        now_seconds: Callable[[], float],
        on_connection_change: Callable[[str], None] | Callable[..., None],
        diagnostic_logger: Callable[..., None] | None = None,
        raw_create_publisher: Callable[..., object],
        raw_create_subscription: Callable[..., object],
        raw_destroy_publisher: Callable[[object], bool | None],
        raw_destroy_subscription: Callable[[object], bool | None],
        raw_create_timer: Callable[..., object],
        raw_destroy_timer: Callable[[object], bool | None] | None = None,
    ) -> None:
        self._runtime_vehicle_name = normalize_vehicle_name(runtime_vehicle_name)
        self.peer_heartbeat_hz = float(peer_heartbeat_hz)
        self.peer_stale_timeout_s = float(peer_stale_timeout_s)
        self._now_seconds = now_seconds
        self._on_connection_change = on_connection_change
        self._diagnostic_logger = diagnostic_logger
        self._raw_create_publisher = raw_create_publisher
        self._raw_create_subscription = raw_create_subscription
        self._raw_destroy_publisher = raw_destroy_publisher
        self._raw_destroy_subscription = raw_destroy_subscription
        self._raw_create_timer = raw_create_timer
        self._raw_destroy_timer = raw_destroy_timer

        self._heartbeat_publisher = None
        self._heartbeat_timer = None
        self._heartbeat_subscriptions: dict[str, object] = {}
        self._connected: dict[str, bool] = {}
        self._last_seen: dict[str, float | None] = {}
        self._peer_names: tuple[str, ...] = ()

        if self._runtime_vehicle_name:
            self._heartbeat_publisher = self._raw_create_publisher(
                Empty,
                self._heartbeat_topic_for(self._runtime_vehicle_name),
                1,
            )
            self._heartbeat_timer = self._raw_create_timer(
                1.0 / self.peer_heartbeat_hz,
                self._peer_timer_callback,
            )
            self._log_diag(
                "HEARTBEAT",
                "initialized local heartbeat publisher",
                local_topic=self.local_heartbeat_topic(),
                peer_heartbeat_hz=self.peer_heartbeat_hz,
                peer_stale_timeout_s=self.peer_stale_timeout_s,
            )

    @property
    def peer_names(self) -> tuple[str, ...]:
        return self._peer_names

    def status(self) -> dict[str, bool]:
        return dict(self._connected)

    def local_heartbeat_topic(self) -> str | None:
        if not self._runtime_vehicle_name:
            return None
        return self._heartbeat_topic_for(self._runtime_vehicle_name)

    def _heartbeat_topic_for(self, vehicle_name: str) -> str:
        return f"/{vehicle_name}/{HEARTBEAT_TOPIC_SUFFIX}"

    def remote_heartbeat_topics(self) -> dict[str, str]:
        return {
            peer_name: self._heartbeat_topic_for(peer_name) for peer_name in self._peer_names
        }

    def debug_snapshot(self) -> dict[str, Any]:
        current_time = self._now_seconds()
        return {
            "local_heartbeat_topic": self.local_heartbeat_topic(),
            "remote_heartbeat_topics": self.remote_heartbeat_topics(),
            "peer_vehicle_names": self.peer_names,
            "connection_status": self.status(),
            "last_seen_age_s": {
                peer_name: (
                    None
                    if last_seen is None
                    else max(0.0, current_time - float(last_seen))
                )
                for peer_name, last_seen in sorted(self._last_seen.items())
            },
        }

    def _log_diag(self, category: str, message: str, **fields) -> None:
        if self._diagnostic_logger is None:
            return
        self._diagnostic_logger(category, message=message, **fields)

    def configure(
        self, peer_vehicle_names: tuple[str, ...] | list[str] | set[str]
    ) -> None:
        normalized_peer_names = [
            peer_name
            for peer_name in normalize_peer_vehicle_names(peer_vehicle_names)
            if peer_name != self._runtime_vehicle_name
        ]
        current_peer_names = set(self._peer_names)
        next_peer_names = set(normalized_peer_names)

        for peer_name in current_peer_names - next_peer_names:
            subscription = self._heartbeat_subscriptions.pop(peer_name, None)
            if subscription is not None:
                self._raw_destroy_subscription(subscription)
            self._connected.pop(peer_name, None)
            self._last_seen.pop(peer_name, None)

        for peer_name in normalized_peer_names:
            if peer_name in current_peer_names:
                continue
            self._connected[peer_name] = False
            self._last_seen[peer_name] = None
            self._heartbeat_subscriptions[peer_name] = self._raw_create_subscription(
                Empty,
                self._heartbeat_topic_for(peer_name),
                lambda _msg, peer_name=peer_name: self._peer_heartbeat_callback(
                    peer_name
                ),
                1,
            )

        self._peer_names = tuple(sorted(next_peer_names))
        self._log_diag(
            "HEARTBEAT",
            "configured peer heartbeat subscriptions",
            local_topic=self.local_heartbeat_topic(),
            peer_vehicle_names=self._peer_names,
            remote_heartbeat_topics=self.remote_heartbeat_topics(),
            removed_peer_vehicle_names=tuple(sorted(current_peer_names - next_peer_names)),
        )

    def _peer_heartbeat_callback(self, peer_name: str) -> None:
        self._last_seen[peer_name] = self._now_seconds()
        if self._connected.get(peer_name, False):
            return
        self._connected[peer_name] = True
        self._log_diag(
            "HEARTBEAT",
            "observed first heartbeat from peer",
            peer_name=peer_name,
            topic=self._heartbeat_topic_for(peer_name),
        )
        self._on_connection_change(peer_name, connected=True)

    def _peer_timer_callback(self) -> None:
        if self._heartbeat_publisher is not None:
            self._heartbeat_publisher.publish(Empty())

        current_time = self._now_seconds()
        for peer_name in self._peer_names:
            last_seen = self._last_seen.get(peer_name)
            if last_seen is None:
                continue
            if current_time - last_seen <= self.peer_stale_timeout_s:
                continue
            if not self._connected.get(peer_name, False):
                continue
            self._connected[peer_name] = False
            self._log_diag(
                "HEARTBEAT",
                "peer heartbeat went stale",
                peer_name=peer_name,
                topic=self._heartbeat_topic_for(peer_name),
                seconds_since_last_seen=max(0.0, current_time - float(last_seen)),
                peer_stale_timeout_s=self.peer_stale_timeout_s,
            )
            self._on_connection_change(peer_name, connected=False)

    def close(self) -> None:
        for peer_name, subscription in list(self._heartbeat_subscriptions.items()):
            self._heartbeat_subscriptions.pop(peer_name, None)
            self._raw_destroy_subscription(subscription)
        if self._heartbeat_timer is not None:
            cancel = getattr(self._heartbeat_timer, "cancel", None)
            if callable(cancel):
                cancel()
            if callable(self._raw_destroy_timer):
                self._raw_destroy_timer(self._heartbeat_timer)
            self._heartbeat_timer = None
        if self._heartbeat_publisher is not None:
            self._raw_destroy_publisher(self._heartbeat_publisher)
            self._heartbeat_publisher = None
