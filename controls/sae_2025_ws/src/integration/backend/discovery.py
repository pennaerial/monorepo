from __future__ import annotations

import asyncio
import logging
import socket
import subprocess
import sys
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


def _decode_process_output(value: str | bytes | None) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode(errors="replace")
    return value


def _dns_sd_type_parts(service_type: str) -> tuple[str, str]:
    value = service_type.strip().rstrip(".")
    if "." not in value:
        return value, "local"
    browse_type, domain = value.rsplit(".", 1)
    return browse_type, domain


def _run_bounded_command(args: list[str], timeout_s: float) -> str:
    timeout_s = max(timeout_s, 0.25)
    try:
        result = subprocess.run(
            args,
            stdout=subprocess.PIPE,
            text=True,
            stderr=subprocess.STDOUT,
            timeout=timeout_s,
            check=False,
        )
        return result.stdout or ""
    except subprocess.TimeoutExpired as exc:
        return _decode_process_output(exc.stdout)
    except OSError as exc:
        LOG.warning("Integration discovery: command failed args=%s error=%s", args, exc)
        return ""


def _parse_dns_sd_browse_output(output: str, service_type: str) -> list[str]:
    browse_type, domain = _dns_sd_type_parts(service_type)
    discovered: list[str] = []
    seen: set[str] = set()
    for raw_line in output.splitlines():
        parts = raw_line.split()
        if len(parts) < 7 or parts[1] != "Add":
            continue
        if parts[4].rstrip(".") != domain or parts[5].rstrip(".") != browse_type:
            continue
        instance_name = " ".join(parts[6:]).strip()
        if not instance_name or instance_name in seen:
            continue
        seen.add(instance_name)
        discovered.append(instance_name)
    return discovered


def _parse_dns_sd_resolve_output(output: str) -> str | None:
    for raw_line in output.splitlines():
        marker = " can be reached at "
        if marker not in raw_line:
            continue
        tail = raw_line.split(marker, 1)[1].strip()
        host_part, _, _port = tail.partition(":")
        hostname = host_part.strip().rstrip(".")
        if hostname:
            return hostname
    return None


def _parse_dns_sd_addrinfo_output(output: str, hostname: str) -> set[str]:
    normalized_host = _normalize_host(hostname)
    addresses: set[str] = set()
    for raw_line in output.splitlines():
        parts = raw_line.split()
        if len(parts) < 7 or parts[1] != "Add":
            continue
        if _normalize_host(parts[4]) != normalized_host:
            continue
        addresses.add(parts[5])
    return addresses


def _resolve_socket_addresses(hostname: str) -> set[str]:
    addresses: set[str] = set()
    try:
        for info in socket.getaddrinfo(hostname, None, family=socket.AF_UNSPEC):
            sockaddr = info[4]
            if isinstance(sockaddr, tuple) and sockaddr:
                addresses.add(sockaddr[0])
    except OSError:
        return set()
    return addresses


def _browse_services_zeroconf(timeout_s: float) -> list[_DiscoveredService]:
    IPVersion, ServiceBrowser, ServiceListener, Zeroconf = _require_zeroconf()
    LOG.warning(
        "Integration discovery: zeroconf browsing %s for %.2fs",
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
            "Integration discovery: zeroconf browse finished discovered=%s hosts=%s",
            len(discovered),
            [item.hostname for item in discovered],
        )
        return discovered
    finally:
        for browser in browsers:
            browser.cancel()
        zeroconf.close()


def _browse_services_dns_sd(timeout_s: float) -> list[_DiscoveredService]:
    LOG.warning(
        "Integration discovery: dns-sd browsing %s for %.2fs",
        ", ".join(_DISCOVERY_SERVICE_TYPES),
        timeout_s,
    )
    discovered: dict[str, _DiscoveredService] = {}
    browse_timeout = max(timeout_s / max(len(_DISCOVERY_SERVICE_TYPES), 1), 0.75)

    for service_type in _DISCOVERY_SERVICE_TYPES:
        browse_type, domain = _dns_sd_type_parts(service_type)
        browse_output = _run_bounded_command(
            ["dns-sd", "-B", browse_type, domain],
            browse_timeout,
        )
        instance_names = _parse_dns_sd_browse_output(browse_output, service_type)
        LOG.warning(
            "Integration discovery: dns-sd browse type=%s instances=%s",
            service_type,
            instance_names,
        )
        for instance_name in instance_names:
            resolve_output = _run_bounded_command(
                ["dns-sd", "-L", instance_name, browse_type, domain],
                max(timeout_s, 1.0),
            )
            hostname = _parse_dns_sd_resolve_output(resolve_output)
            if not hostname:
                LOG.warning(
                    "Integration discovery: dns-sd resolve missing hostname name=%s type=%s",
                    instance_name,
                    service_type,
                )
                continue
            addresses = _parse_dns_sd_addrinfo_output(
                _run_bounded_command(
                    ["dns-sd", "-G", "v4v6", hostname],
                    max(timeout_s, 1.0),
                ),
                hostname,
            )
            if not addresses:
                addresses = _resolve_socket_addresses(hostname)
            key = _normalize_host(hostname)
            record = discovered.get(key)
            if record is None:
                record = _DiscoveredService(
                    hostname=hostname,
                    service_name=f"{instance_name}.{browse_type}.local.",
                    service_type=service_type,
                )
                discovered[key] = record
            record.addresses.update(addresses)
            LOG.warning(
                "Integration discovery: dns-sd resolved name=%s type=%s host=%s addresses=%s",
                instance_name,
                service_type,
                hostname,
                sorted(record.addresses),
            )

    resolved = sorted(discovered.values(), key=lambda item: _normalize_host(item.hostname))
    LOG.warning(
        "Integration discovery: dns-sd browse finished discovered=%s hosts=%s",
        len(resolved),
        [item.hostname for item in resolved],
    )
    return resolved


def _browse_services(timeout_s: float) -> list[_DiscoveredService]:
    if sys.platform == "darwin":
        try:
            discovered = _browse_services_zeroconf(timeout_s)
        except Exception as exc:
            LOG.warning("Integration discovery: zeroconf failed on macOS, falling back to dns-sd: %s", exc)
            discovered = []
        if discovered:
            return discovered
        LOG.warning("Integration discovery: zeroconf empty on macOS, falling back to dns-sd")
        return _browse_services_dns_sd(timeout_s)
    return _browse_services_zeroconf(timeout_s)


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
