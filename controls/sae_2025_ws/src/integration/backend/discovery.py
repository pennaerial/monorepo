from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass, field

from .context import AppContext

LOG = logging.getLogger(__name__)

_DISCOVERY_SERVICE_TYPES = (
    "_ssh._tcp.local.",
    "_workstation._tcp.local.",
)


def _require_zeroconf():
    try:
        from zeroconf import IPVersion, ServiceBrowser, ServiceListener, Zeroconf
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Missing Python dependency `zeroconf`. "
            "If using conda/pip, run: pip install zeroconf. "
            "If using uv, rerun with `uv run app.py`."
        ) from exc
    return IPVersion, ServiceBrowser, ServiceListener, Zeroconf


def _normalize_host(hostname: str) -> str:
    value = (hostname or "").strip().rstrip(".").lower()
    if value.endswith(".local"):
        return value[:-6]
    return value


def _target_match(ctx: AppContext, hostname: str) -> tuple[str | None, str | None]:
    normalized = _normalize_host(hostname)
    for target in ctx.list_targets():
        target_host = _normalize_host(target.pi_host)
        if normalized == target_host:
            return target.target_id, target.label
    return None, None


@dataclass(slots=True)
class _DiscoveredService:
    hostname: str
    service_name: str
    service_type: str
    addresses: set[str] = field(default_factory=set)


def _browse_services(timeout_s: float) -> list[_DiscoveredService]:
    IPVersion, ServiceBrowser, ServiceListener, Zeroconf = _require_zeroconf()
    LOG.warning(
        "Integration discovery: browsing %s for %.2fs",
        ", ".join(_DISCOVERY_SERVICE_TYPES),
        timeout_s,
    )

    class Listener(ServiceListener):
        def __init__(self, zeroconf) -> None:
            self.zeroconf = zeroconf
            self.found: dict[str, _DiscoveredService] = {}

        def remove_service(self, zeroconf, service_type: str, name: str) -> None:
            return None

        def update_service(self, zeroconf, service_type: str, name: str) -> None:
            self.add_service(zeroconf, service_type, name)

        def add_service(self, zeroconf, service_type: str, name: str) -> None:
            LOG.warning(
                "Integration discovery: saw service announcement name=%s type=%s",
                name,
                service_type,
            )
            info = zeroconf.get_service_info(service_type, name, timeout=500)
            if info is None:
                LOG.warning(
                    "Integration discovery: service info lookup returned None name=%s type=%s timeout_ms=500",
                    name,
                    service_type,
                )
                return
            if not info.server:
                LOG.warning(
                    "Integration discovery: service info missing server name=%s type=%s",
                    name,
                    service_type,
                )
                return
            hostname = info.server.rstrip(".")
            key = _normalize_host(hostname)
            record = self.found.get(key)
            if record is None:
                record = _DiscoveredService(
                    hostname=hostname,
                    service_name=name,
                    service_type=service_type,
                )
                self.found[key] = record
            addresses = info.parsed_addresses(IPVersion.All) or []
            if not addresses:
                LOG.warning(
                    "Integration discovery: resolved service without addresses name=%s type=%s server=%s",
                    name,
                    service_type,
                    hostname,
                )
            for address in addresses:
                record.addresses.add(address)
            LOG.warning(
                "Integration discovery: resolved name=%s type=%s server=%s addresses=%s",
                name,
                service_type,
                hostname,
                sorted(record.addresses),
            )

    zeroconf = Zeroconf(ip_version=IPVersion.All)
    listener = Listener(zeroconf)
    browsers = [
        ServiceBrowser(zeroconf, service_type, listener)
        for service_type in _DISCOVERY_SERVICE_TYPES
    ]
    try:
        time.sleep(timeout_s)
        discovered = sorted(
            listener.found.values(), key=lambda item: _normalize_host(item.hostname)
        )
        LOG.warning(
            "Integration discovery: browse finished discovered=%s hosts=%s",
            len(discovered),
            [item.hostname for item in discovered],
        )
        return discovered
    finally:
        for browser in browsers:
            browser.cancel()
        zeroconf.close()


async def live_hardware_cards(
    ctx: AppContext, *, timeout_s: float = 1.25
) -> list[dict[str, object]]:
    LOG.warning("Integration discovery: /api/hardware/live requested")
    discovered = await asyncio.to_thread(_browse_services, timeout_s)
    cards: list[dict[str, object]] = []
    for device in discovered:
        matched_target_id, matched_label = _target_match(ctx, device.hostname)
        cards.append(
            {
                "hardware_id": _normalize_host(device.hostname) or device.hostname,
                "hostname": device.hostname,
                "addresses": sorted(device.addresses),
                "service_name": device.service_name,
                "service_type": device.service_type,
                "matched_target_id": matched_target_id,
                "matched_label": matched_label,
                "saved": matched_target_id is not None,
            }
        )
    LOG.warning(
        "Integration discovery: returning %s live hardware cards hostnames=%s",
        len(cards),
        [card["hostname"] for card in cards],
    )
    return cards
