from __future__ import annotations

import asyncio
import sys
import types
from pathlib import Path
from types import SimpleNamespace


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

backend_pkg = types.ModuleType("backend")
backend_pkg.__path__ = [str(PACKAGE_ROOT / "backend")]
sys.modules.setdefault("backend", backend_pkg)

from backend import discovery  # noqa: E402
from backend.config import InventoryStore, OperatorConfig, TargetRecord  # noqa: E402


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
    seed_target = TargetRecord.from_dict(
        {
            "target_id": "pi-1",
            "label": "Primary Pi",
            "pi_user": "penn",
            "pi_host": "pi-1.local",
            "deploy_root": "/home/penn/pennair-deploy",
            "ssh_key": "",
            "ssh_pass": "",
            "fleet_file": "/tmp/fleet.yaml",
            "vehicle_name": "uav_0",
            "overlay_yaml": "mission: hover\npx4_airframe_id: 4004\n",
            "service_unit": "pennair-autonomy.service",
        },
        default_deploy_root=operator.default_deploy_root,
        default_fleet_file="/tmp/fleet.yaml",
    )
    inventory = InventoryStore(
        operator.inventory_path,
        operator_config=operator,
        default_deploy_root=operator.default_deploy_root,
        default_fleet_file="/tmp/fleet.yaml",
        seed_target=seed_target,
    )
    return SimpleNamespace(list_targets=inventory.list_targets)


def test_live_hardware_cards_match_inventory_hosts(tmp_path, monkeypatch):
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

    assert devices == [
        {
            "hardware_id": "pi-1",
            "hostname": "pi-1.local",
            "addresses": ["10.0.0.2"],
            "service_name": "Primary._ssh._tcp.local.",
            "service_type": "_ssh._tcp.local.",
            "matched_target_id": "pi-1",
            "matched_label": "Primary Pi",
            "saved": True,
        },
        {
            "hardware_id": "payload",
            "hostname": "payload.local",
            "addresses": ["10.0.0.3"],
            "service_name": "Payload._ssh._tcp.local.",
            "service_type": "_ssh._tcp.local.",
            "matched_target_id": None,
            "matched_label": None,
            "saved": False,
        },
    ]
