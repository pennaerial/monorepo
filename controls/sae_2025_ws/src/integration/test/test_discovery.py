from __future__ import annotations

import asyncio
import sys
import types
from pathlib import Path
from types import SimpleNamespace
import subprocess


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

backend_pkg = types.ModuleType("backend")
backend_pkg.__path__ = [str(PACKAGE_ROOT / "backend")]
sys.modules.setdefault("backend", backend_pkg)

from backend import discovery  # noqa: E402
from backend.config import InventoryStore, OperatorConfig  # noqa: E402


def _reset_discovery_snapshot() -> None:
    discovery._DISCOVERY_SNAPSHOT.clear()
    discovery._DISCOVERY_LAST_REFRESH_MONOTONIC = 0.0


def _assert_device_card(
    device: dict[str, object], *, discovery_stale: bool = False
) -> None:
    assert isinstance(device.get("last_seen_at"), str)
    assert device["discovery_stale"] is discovery_stale


def setup_function(_function):
    _reset_discovery_snapshot()


def teardown_function(_function):
    _reset_discovery_snapshot()


def _make_operator(tmp_path: Path) -> OperatorConfig:
    return OperatorConfig(
        github_repo="pennaerial/monorepo",
        github_token="",
        hotspot_name="pennair-hotspot",
        inventory_path=tmp_path / ".integration_inventory.json",
        default_deploy_root="/home/penn/pennair-deploy",
        default_pi_user="penn",
        default_ssh_key="",
        default_ssh_pass="",
    )


def _make_context(tmp_path: Path) -> SimpleNamespace:
    operator = _make_operator(tmp_path)
    inventory = InventoryStore(
        operator.inventory_path,
        operator_config=operator,
        default_deploy_root=operator.default_deploy_root,
    )
    inventory.upsert_target(
        {
            "target_id": "pi-1",
            "label": "Primary Pi",
            "pi_user": "penn",
            "pi_host": "pi-1.local",
            "deploy_root": "/home/penn/pennair-deploy",
            "ssh_key": "",
            "ssh_pass": "",
            "vehicle_name": "uav_0",
            "overlay_yaml": "mission: hover\npx4_airframe_id: 4004\n",
            "service_unit": "pennair-autonomy.service",
        }
    )
    return SimpleNamespace(list_targets=inventory.list_targets)


def test_live_hardware_cards_return_hostname_scoped_devices(tmp_path, monkeypatch):
    ctx = _make_context(tmp_path)

    monkeypatch.setattr(
        discovery,
        "_browse_services",
        lambda _timeout: [
            discovery._DiscoveredService(
                hostname="pi-1.local",
                service_name="Primary._ssh._tcp.local.",
                service_type="_ssh._tcp.local.",
                addresses={"10.0.0.2"},
            ),
            discovery._DiscoveredService(
                hostname="payload.local",
                service_name="Payload._ssh._tcp.local.",
                service_type="_ssh._tcp.local.",
                addresses={"10.0.0.3"},
            ),
        ],
    )

    devices = asyncio.run(discovery.live_hardware_cards(ctx, timeout_s=0))

    assert len(devices) == 2
    devices_by_host = {device["hostname"]: device for device in devices}
    assert devices_by_host["pi-1.local"] == {
        "hardware_id": "pi-1",
        "hostname": "pi-1.local",
        "addresses": ["10.0.0.2"],
        "service_name": "Primary._ssh._tcp.local.",
        "service_type": "_ssh._tcp.local.",
        "last_seen_at": devices_by_host["pi-1.local"]["last_seen_at"],
        "discovery_stale": False,
    }
    assert devices_by_host["payload.local"] == {
        "hardware_id": "payload",
        "hostname": "payload.local",
        "addresses": ["10.0.0.3"],
        "service_name": "Payload._ssh._tcp.local.",
        "service_type": "_ssh._tcp.local.",
        "last_seen_at": devices_by_host["payload.local"]["last_seen_at"],
        "discovery_stale": False,
    }
    _assert_device_card(devices_by_host["pi-1.local"])
    _assert_device_card(devices_by_host["payload.local"])


def test_live_hardware_cards_ignore_inventory_matches(tmp_path, monkeypatch):
    operator = _make_operator(tmp_path)
    inventory = InventoryStore(
        operator.inventory_path,
        operator_config=operator,
        default_deploy_root=operator.default_deploy_root,
    )
    inventory.upsert_target(
        {
            "target_id": "payload",
            "label": "Payload Pi",
            "pi_user": "penn",
            "pi_host": "10.0.0.44",
            "deploy_root": "/home/penn/pennair-deploy",
            "ssh_key": "",
            "ssh_pass": "",
            "vehicle_name": "payload_0",
            "overlay_yaml": "mission: payload_retreat\npayload_controller: GPIOController\n",
            "service_unit": "pennair-autonomy.service",
        }
    )
    ctx = SimpleNamespace(list_targets=inventory.list_targets)

    monkeypatch.setattr(
        discovery,
        "_browse_services",
        lambda _timeout: [
            discovery._DiscoveredService(
                hostname="payload.local",
                service_name="Payload._ssh._tcp.local.",
                service_type="_ssh._tcp.local.",
                addresses={"10.0.0.44"},
            )
        ],
    )

    devices = asyncio.run(discovery.live_hardware_cards(ctx, timeout_s=0))

    assert devices[0]["hostname"] == "payload.local"
    assert devices[0]["hardware_id"] == "payload"
    _assert_device_card(devices[0])


def test_parse_dns_sd_browse_output_extracts_instances():
    output = """Browsing for _ssh._tcp.local
DATE: ---Sat 11 Apr 2026---
22:32:47.611  ...STARTING...
Timestamp     A/R    Flags  if Domain               Service Type         Instance Name
22:32:47.869  Add        2  14 local.               _ssh._tcp.           air-02
22:32:48.100  Add        2  14 local.               _ssh._tcp.           air-03
"""

    assert discovery._parse_dns_sd_browse_output(output, "_ssh._tcp.local.") == [
        "air-02",
        "air-03",
    ]


def test_parse_dns_sd_resolve_output_extracts_hostname():
    output = """Lookup air-02._ssh._tcp.local
DATE: ---Sat 11 Apr 2026---
22:32:49.000  air-02._ssh._tcp.local. can be reached at air-02.local.:22 (interface 14)
"""

    assert discovery._parse_dns_sd_resolve_output(output) == "air-02.local"


def test_parse_dns_sd_addrinfo_output_extracts_addresses():
    output = """DATE: ---Sat 11 Apr 2026---
Timestamp     A/R    Flags  if Hostname                               Address                                      TTL
22:32:50.000  Add        2  14 air-02.local.                         192.168.1.22                                 120
22:32:50.000  Add        2  14 air-02.local.                         fe80::1234%en0                               120
"""

    assert discovery._parse_dns_sd_addrinfo_output(output, "air-02.local") == {
        "192.168.1.22",
        "fe80::1234%en0",
    }


def test_browse_services_falls_back_to_dns_sd_on_darwin(monkeypatch):
    monkeypatch.setattr(discovery.sys, "platform", "darwin")
    monkeypatch.setattr(discovery, "_DARWIN_FORCE_DNS_SD", False)
    monkeypatch.setattr(discovery, "_browse_services_zeroconf", lambda _timeout: [])
    monkeypatch.setattr(
        discovery,
        "_browse_services_dns_sd",
        lambda _timeout: [
            discovery._DiscoveredService(
                hostname="air-02.local",
                service_name="air-02._ssh._tcp.local.",
                service_type="_ssh._tcp.local.",
                addresses={"192.168.1.22"},
            )
        ],
    )

    discovered = discovery._browse_services(1.25)

    assert discovered == [
        discovery._DiscoveredService(
            hostname="air-02.local",
            service_name="air-02._ssh._tcp.local.",
            service_type="_ssh._tcp.local.",
            addresses={"192.168.1.22"},
        )
    ]
    assert discovery._DARWIN_FORCE_DNS_SD is True


def test_browse_services_uses_cached_dns_sd_on_darwin(monkeypatch):
    monkeypatch.setattr(discovery.sys, "platform", "darwin")
    monkeypatch.setattr(discovery, "_DARWIN_FORCE_DNS_SD", True)

    def fail_zeroconf(_timeout):
        raise AssertionError("zeroconf should be skipped when dns-sd is cached")

    monkeypatch.setattr(discovery, "_browse_services_zeroconf", fail_zeroconf)
    monkeypatch.setattr(
        discovery,
        "_browse_services_dns_sd",
        lambda _timeout: [
            discovery._DiscoveredService(
                hostname="air-02.local",
                service_name="air-02._ssh._tcp.local.",
                service_type="_ssh._tcp.local.",
                addresses={"192.168.1.22"},
            )
        ],
    )

    discovered = discovery._browse_services(1.25)

    assert discovered[0].hostname == "air-02.local"


def test_run_bounded_command_merges_stderr_without_capture_output_conflict(monkeypatch):
    calls = []

    def fake_run(*args, **kwargs):
        calls.append(kwargs)
        return SimpleNamespace(stdout="ok\n")

    monkeypatch.setattr(discovery.subprocess, "run", fake_run)

    output = discovery._run_bounded_command(["dns-sd", "-B", "_ssh._tcp", "local"], 1.0)

    assert output == "ok\n"
    assert calls[0]["stdout"] is subprocess.PIPE
    assert calls[0]["stderr"] is subprocess.STDOUT
    assert "capture_output" not in calls[0]
