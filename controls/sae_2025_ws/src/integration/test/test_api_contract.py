from __future__ import annotations

import asyncio
import inspect
import sys
import types
from pathlib import Path
from types import SimpleNamespace

import httpx
import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))
UAV_ROOT = PACKAGE_ROOT.parent / "uav"
if str(UAV_ROOT) not in sys.path:
    sys.path.insert(0, str(UAV_ROOT))

backend_pkg = types.ModuleType("backend")
backend_pkg.__path__ = [str(PACKAGE_ROOT / "backend")]
sys.modules.setdefault("backend", backend_pkg)
sys.modules.pop("backend.context", None)
sys.modules.pop("backend.app_factory", None)

from backend.app_factory import create_app  # noqa: E402
from backend.config import (  # noqa: E402
    BuildSourceStore,
    InventoryStore,
    OperatorConfig,
    TargetRecord,
    build_source_store_paths,
)


def _assert_api_device_card(
    device: dict[str, object], *, allow_null_last_seen: bool = False
):
    if allow_null_last_seen:
        assert device["last_seen_at"] is None or isinstance(device["last_seen_at"], str)
    else:
        assert isinstance(device["last_seen_at"], str)
    assert isinstance(device["discovery_stale"], bool)


@pytest.fixture(autouse=True)
def _reset_discovery_snapshot():
    from backend import discovery

    discovery._DISCOVERY_SNAPSHOT.clear()
    discovery._DISCOVERY_LAST_REFRESH_MONOTONIC = 0.0
    yield
    discovery._DISCOVERY_SNAPSHOT.clear()
    discovery._DISCOVERY_LAST_REFRESH_MONOTONIC = 0.0


class _FakeSSH:
    def q(self, value: str) -> str:
        return value

    async def run(self, command: str, timeout: int = 15):
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    async def scp(self, local_path: str, remote_path: str, timeout: int = 300):
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    def friendly_error(self, message: str) -> str:
        return message

    def friendly_timeout(self) -> str:
        return "timeout"

    def format_remote_error(self, raw_error: str, prefix: str) -> str:
        return f"{prefix}: {raw_error}"


def _make_operator(tmp_path: Path) -> OperatorConfig:
    return OperatorConfig(
        github_repo="pennaerial/monorepo",
        github_token="token",
        hotspot_name="pennair-hotspot",
        inventory_path=tmp_path / ".integration_inventory.json",
        default_deploy_root="/home/penn/pennair-deploy",
        default_pi_user="penn",
        default_ssh_key="",
        default_ssh_pass="",
    )


def _make_target() -> TargetRecord:
    return TargetRecord.from_dict(
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
            "enabled": True,
        },
        default_deploy_root="/home/penn/pennair-deploy",
    )


def _make_context(tmp_path: Path) -> SimpleNamespace:
    base_dir = tmp_path / "src" / "integration"
    base_dir.mkdir(parents=True, exist_ok=True)
    operator = _make_operator(tmp_path)
    target = _make_target()
    inventory_kwargs = {
        "operator_config": operator,
        "default_deploy_root": operator.default_deploy_root,
    }
    if "seed_target" in inspect.signature(InventoryStore).parameters:
        inventory = InventoryStore(
            operator.inventory_path,
            seed_target=target,
            **inventory_kwargs,
        )
    else:
        inventory = InventoryStore(
            operator.inventory_path,
            **inventory_kwargs,
        )
        inventory.upsert_target(target.to_store_dict())
    build_source_path, cache_dir = build_source_store_paths(operator.inventory_path)
    build_source_store = BuildSourceStore(
        build_source_path,
        cache_dir=cache_dir,
    )

    def resolve_target(
        target_id: str | None = None,
        *,
        hostname: str | None = None,
        vehicle_name: str | None = None,
        label: str | None = None,
        pi_user: str | None = None,
        deploy_root: str | None = None,
        ssh_key: str | None = None,
        ssh_pass: str | None = None,
        service_unit: str | None = None,
        enabled: bool | None = None,
    ):
        resolved = None
        if target_id:
            resolved = inventory.get_target(target_id)
        elif hostname:
            normalized = hostname.strip().rstrip(".").lower()
            for candidate in inventory.list_targets():
                if candidate.pi_host.strip().rstrip(".").lower() == normalized:
                    resolved = candidate
                    break
            if resolved is None:
                resolved = TargetRecord.for_host(
                    hostname=hostname,
                    target_id=hostname,
                    label=label,
                    pi_user=pi_user or operator.default_pi_user,
                    deploy_root=deploy_root or operator.default_deploy_root,
                    ssh_key=ssh_key
                    if ssh_key is not None
                    else operator.default_ssh_key,
                    ssh_pass=ssh_pass
                    if ssh_pass is not None
                    else operator.default_ssh_pass,
                    vehicle_name=vehicle_name or "",
                    service_unit=service_unit or "pennair-autonomy.service",
                    enabled=True if enabled is None else enabled,
                    default_deploy_root=operator.default_deploy_root,
                )
        else:
            raise ValueError("target_id or hostname is required")

        if vehicle_name:
            resolved = TargetRecord.from_dict(
                {
                    **resolved.to_store_dict(),
                    "vehicle_name": vehicle_name,
                },
                default_deploy_root=operator.default_deploy_root,
            )
        return SimpleNamespace(
            target=resolved,
            ssh=_FakeSSH(),
            mission_state=SimpleNamespace(
                apply_launch_status=lambda **kwargs: None,
                snapshot=lambda: None,
                set=lambda **kwargs: None,
            ),
        )

    def resolve_live_target(
        *,
        hostname: str | None,
        vehicle_name: str | None = None,
        label: str | None = None,
        pi_user: str | None = None,
        deploy_root: str | None = None,
        ssh_key: str | None = None,
        ssh_pass: str | None = None,
        service_unit: str | None = None,
        enabled: bool | None = None,
    ):
        if not hostname:
            raise ValueError("hostname is required")
        resolved = TargetRecord.for_host(
            hostname=hostname,
            target_id=hostname,
            label=label,
            pi_user=pi_user or operator.default_pi_user,
            deploy_root=deploy_root or operator.default_deploy_root,
            ssh_key=ssh_key if ssh_key is not None else operator.default_ssh_key,
            ssh_pass=ssh_pass if ssh_pass is not None else operator.default_ssh_pass,
            vehicle_name=vehicle_name or "",
            service_unit=service_unit or "pennair-autonomy.service",
            enabled=True if enabled is None else enabled,
            default_deploy_root=operator.default_deploy_root,
        )
        return SimpleNamespace(
            target=resolved,
            ssh=_FakeSSH(),
            mission_state=SimpleNamespace(
                apply_launch_status=lambda **kwargs: None,
                snapshot=lambda: None,
                set=lambda **kwargs: None,
            ),
        )

    return SimpleNamespace(
        base_dir=base_dir,
        operator_config=operator,
        inventory=inventory,
        build_source_store=build_source_store,
        require_deploy_context=lambda require_build_source=True: None,
        resolve_target=resolve_target,
        resolve_live_target=resolve_live_target,
        list_targets=inventory.list_targets,
    )


def _write_fleet(tmp_path: Path, *, name: str = "fleet.yaml") -> Path:
    fleets_dir = tmp_path / "src" / "uav" / "uav" / "fleets"
    fleets_dir.mkdir(parents=True, exist_ok=True)
    fleet_path = fleets_dir / name
    fleet_path.write_text(
        "vehicles:\n  - name: uav_0\n    mission: hover\n",
        encoding="utf-8",
    )
    return fleet_path


@pytest.fixture
def client(tmp_path, monkeypatch):
    ctx = _make_context(tmp_path)
    monkeypatch.setattr("backend.app_factory.create_context", lambda base_dir: ctx)
    app = create_app(tmp_path / "src" / "integration")

    class ApiClient:
        def __init__(self, app):
            self.app = app

        def request(self, method: str, url: str, **kwargs):
            async def _send():
                transport = httpx.ASGITransport(app=self.app)
                async with httpx.AsyncClient(
                    transport=transport,
                    base_url="http://testserver",
                ) as async_client:
                    return await async_client.request(method, url, **kwargs)

            return asyncio.run(_send())

        def get(self, url: str, **kwargs):
            return self.request("GET", url, **kwargs)

        def post(self, url: str, **kwargs):
            return self.request("POST", url, **kwargs)

    return ApiClient(app)


def test_config_is_operator_global_only(client):
    response = client.get("/api/config")
    assert response.status_code == 200
    payload = response.json()

    assert payload["success"] is True
    assert "config" in payload
    assert "active_target_id" not in payload
    assert "workspace_paths" not in payload


def test_hardware_live_endpoint_returns_device_cards(client, monkeypatch):
    from backend import discovery

    async def fake_live_hardware_cards(ctx, timeout_s=5.0):
        return [
            {
                "hardware_id": "pi-1",
                "hostname": "pi-1.local",
                "addresses": ["10.0.0.2"],
                "service_name": "Primary._ssh._tcp.local.",
                "service_type": "_ssh._tcp.local.",
            },
            {
                "hardware_id": "payload",
                "hostname": "payload.local",
                "addresses": ["10.0.0.3"],
                "service_name": "Payload._ssh._tcp.local.",
                "service_type": "_ssh._tcp.local.",
            },
        ]

    monkeypatch.setattr(discovery, "live_hardware_cards", fake_live_hardware_cards)

    response = client.get("/api/hardware/live")
    assert response.status_code == 200
    payload = response.json()

    assert payload["success"] is True
    assert payload["error"] is None
    assert payload["devices"] == [
        {
            "hardware_id": "pi-1",
            "hostname": "pi-1.local",
            "addresses": ["10.0.0.2"],
            "service_name": "Primary._ssh._tcp.local.",
            "service_type": "_ssh._tcp.local.",
            "last_seen_at": None,
            "discovery_stale": False,
            "matched_target_id": None,
            "matched_label": None,
            "saved": False,
        },
        {
            "hardware_id": "payload",
            "hostname": "payload.local",
            "addresses": ["10.0.0.3"],
            "service_name": "Payload._ssh._tcp.local.",
            "service_type": "_ssh._tcp.local.",
            "last_seen_at": None,
            "discovery_stale": False,
            "matched_target_id": None,
            "matched_label": None,
            "saved": False,
        },
    ]
    for device in payload["devices"]:
        _assert_api_device_card(device, allow_null_last_seen=True)


def test_fleet_board_keeps_last_seen_device_when_discovery_returns_empty(
    client, monkeypatch
):
    from backend import discovery
    from backend.services import fleet as fleet_service

    calls = {"count": 0}

    async def fake_current_build(ctx, *, target_ctx=None, target_id=None):
        return {"success": True, "installed": True, "info": "build ok"}

    async def fake_launch_status(ctx, *, target_ctx=None, target_id=None):
        return {
            "success": True,
            "running": False,
            "state": "not_prepared",
            "pid": None,
            "error": None,
        }

    monkeypatch.setattr(discovery, "_DISCOVERY_REFRESH_INTERVAL_SECONDS", 0.0)
    discovery._DISCOVERY_SNAPSHOT.clear()
    discovery._DISCOVERY_LAST_REFRESH_MONOTONIC = 0.0

    def fake_browse_services(_timeout):
        calls["count"] += 1
        if calls["count"] == 1:
            return [
                discovery._DiscoveredService(
                    hostname="pi-1.local",
                    service_name="Primary._ssh._tcp.local.",
                    service_type="_ssh._tcp.local.",
                    addresses={"10.0.0.2"},
                )
            ]
        return []

    monkeypatch.setattr(discovery, "_browse_services", fake_browse_services)
    monkeypatch.setattr(
        fleet_service.deploy_service, "current_build", fake_current_build
    )
    monkeypatch.setattr(
        fleet_service.mission_service, "launch_status", fake_launch_status
    )

    first = client.get("/api/fleet/board")
    assert first.status_code == 200
    assert first.json()["success"] is True
    assert len(first.json()["devices"]) == 1

    second = client.get("/api/fleet/board")
    assert second.status_code == 200
    payload = second.json()

    assert payload["success"] is True
    assert payload["summary"]["total_devices"] == 1
    assert payload["summary"]["stale_devices"] == 1
    assert len(payload["devices"]) == 1
    assert payload["devices"][0]["hostname"] == "pi-1.local"
    assert payload["devices"][0]["discovery_stale"] is True
    _assert_api_device_card(payload["devices"][0])


@pytest.mark.parametrize(
    ("method", "url", "data"),
    [
        ("get", "/api/connection/status", None),
        ("get", "/api/connection/ssh-command", None),
        ("get", "/api/wifi/status", None),
        ("get", "/api/wifi/scan", None),
        ("post", "/api/wifi/connect", {"ssid": "wifi"}),
        ("post", "/api/wifi/hotspot", {}),
        ("get", "/api/mission/state", None),
        ("get", "/api/mission/launch/status", None),
        ("get", "/api/mission/launch/logs", None),
        ("post", "/api/mission/prepare", {}),
        ("post", "/api/mission/stop", {}),
        ("post", "/api/mission/start", {}),
        ("post", "/api/failsafe", {}),
        ("get", "/api/mission/launch-params", None),
        ("get", "/api/mission/mission-names", None),
        ("post", "/api/mission/launch-params", {"content": "mission: hover\n"}),
        ("get", "/api/mission/mission-file?name=hover", None),
        (
            "post",
            "/api/mission/mission-file",
            {
                "name": "hover",
                "content": "modes:\n  hover:\n    mode: uav.HoverMode\n",
            },
        ),
        ("get", "/api/builds/current", None),
        ("post", "/api/builds/deploy-selected", {}),
        (
            "post",
            "/api/builds/download",
            {"source": "release", "tag": "build-deadbeef"},
        ),
        ("post", "/api/builds/rollback", {}),
        ("post", "/api/builds/deploy-local", {}),
    ],
)
def test_target_scoped_http_routes_require_device_scope(client, method, url, data):
    request = getattr(client, method)
    kwargs = {}
    if data is not None:
        if method == "get":
            kwargs["params"] = data
        else:
            kwargs["data"] = data
    response = request(url, **kwargs)
    assert response.status_code == 400


def test_target_scoped_upload_routes_require_device_scope(client):
    response = client.post(
        "/api/builds/upload",
        files={"file": ("build.tar.gz", b"artifact", "application/gzip")},
    )
    assert response.status_code == 400

    response = client.post(
        "/api/builds/upload-source",
        files={"file": ("source.tar.gz", b"bundle", "application/gzip")},
    )
    assert response.status_code == 400


def test_terminal_websocket_accepts_hostname_scope(client):
    route = next(
        route
        for route in client.app.routes
        if getattr(route, "path", "") == "/ws/mission/terminal"
    )
    target_param = inspect.signature(route.endpoint).parameters["target_id"]
    hostname_param = inspect.signature(route.endpoint).parameters["hostname"]
    assert not target_param.default.is_required()
    assert not hostname_param.default.is_required()


def test_connection_status_accepts_hostname_scope(client):
    response = client.get("/api/connection/status", params={"hostname": "pi-1.local"})
    assert response.status_code == 200
    payload = response.json()
    assert payload["connected"] is True


def test_wifi_status_accepts_hostname_scope(client, monkeypatch):
    from backend.services import wifi as wifi_service

    async def fake_wifi_status(ctx, *, target_ctx=None, target_id=None):
        assert (
            target_ctx.target.target_id if target_ctx else target_id
        ) == "pi-1.local"
        return {"success": True, "is_hotspot": False, "current_wifi": "ops"}

    monkeypatch.setattr(wifi_service, "wifi_status", fake_wifi_status)

    response = client.get(
        "/api/wifi/status",
        params={"hostname": "pi-1.local", "vehicle_name": "uav_0"},
    )
    assert response.status_code == 200
    payload = response.json()
    assert payload["success"] is True
    assert payload["current_wifi"] == "ops"


def test_build_current_accepts_hostname_scope(client, monkeypatch):
    from backend.services import deploy as deploy_service

    async def fake_current_build(ctx, *, target_ctx=None, target_id=None):
        assert (
            target_ctx.target.target_id if target_ctx else target_id
        ) == "pi-1.local"
        return {"success": True, "installed": True, "info": "build ok"}

    monkeypatch.setattr(deploy_service, "current_build", fake_current_build)

    response = client.get(
        "/api/builds/current",
        params={"hostname": "pi-1.local", "vehicle_name": "uav_0"},
    )
    assert response.status_code == 200
    payload = response.json()
    assert payload["success"] is True
    assert payload["installed"] is True


def test_mission_state_accepts_hostname_scope(client, monkeypatch):
    from backend.services import mission as mission_service

    async def fake_mission_state(ctx, *, target_ctx=None, target_id=None):
        assert (
            target_ctx.target.target_id if target_ctx else target_id
        ) == "pi-1.local"
        return {
            "success": True,
            "state": {
                "phase": "idle",
                "launch_state": "not_prepared",
                "running": False,
                "pid": None,
                "message": None,
                "error": None,
                "updated_at": 0.0,
            },
        }

    monkeypatch.setattr(mission_service, "mission_state", fake_mission_state)

    response = client.get(
        "/api/mission/state",
        params={"hostname": "pi-1.local", "vehicle_name": "uav_0"},
    )
    assert response.status_code == 200
    payload = response.json()
    assert payload["success"] is True
    assert payload["state"]["phase"] == "idle"


def test_fleet_action_accepts_query_scoped_single_device(client, monkeypatch):
    from backend.services import fleet as fleet_service

    async def fake_batch_action(ctx, *, action, devices=None, session_assignments=None):
        assert action == "prepare"
        assert devices is not None
        assert len(devices) == 1
        assert session_assignments is None
        selection = devices[0]
        assert selection.hostname == "pi-1.local"
        assert selection.vehicle_name == "uav_0"
        return {
            "success": True,
            "action": action,
            "build_source": {
                "success": True,
                "source": {
                    "kind": "none",
                    "summary": "none",
                },
            },
            "fleet_catalog": {
                "success": True,
                "source_kind": "none",
                "fleet_vehicles": [],
            },
            "fleet_vehicles": [],
            "results": [
                {
                    "hardware_id": "pi-1.local",
                    "hostname": "pi-1.local",
                    "addresses": [],
                    "service_name": None,
                    "service_type": None,
                    "matched_target_id": None,
                    "matched_label": None,
                    "saved": False,
                    "target_id": "pi-1.local",
                    "vehicle_name": "uav_0",
                    "fleet_vehicle": None,
                    "connection": None,
                    "current_build": None,
                    "runtime": None,
                    "readiness": {
                        "connected": False,
                        "build_installed": False,
                        "runtime_ready": False,
                        "vehicle_assigned": True,
                        "ready": False,
                        "notes": [],
                    },
                    "action": action,
                    "success": True,
                    "output": "prepared",
                    "error": None,
                }
            ],
            "summary": {
                "requested_devices": 1,
                "successful_devices": 1,
                "failed_devices": 0,
            },
            "error": None,
        }

    monkeypatch.setattr(fleet_service, "batch_action", fake_batch_action)

    response = client.post(
        "/api/fleet/prepare",
        params={"hostname": "pi-1.local", "vehicle_name": "uav_0"},
    )
    assert response.status_code == 200
    payload = response.json()
    assert payload["success"] is True
    assert payload["results"][0]["vehicle_name"] == "uav_0"


def test_build_deploy_route_accepts_network_policy_fields(client, monkeypatch):
    from backend.services import deploy as deploy_service

    captured: dict[str, object] = {}

    async def fake_deploy_selected_source(
        ctx,
        *,
        target_ctx=None,
        target_id=None,
        network_policy_override=None,
        session_assignments=None,
    ):
        captured["target_ctx"] = target_ctx
        captured["target_id"] = target_id
        captured["network_policy_override"] = network_policy_override
        captured["session_assignments"] = session_assignments
        return {"success": True, "output": "deployed"}

    monkeypatch.setattr(
        deploy_service, "deploy_selected_source", fake_deploy_selected_source
    )

    response = client.post(
        "/api/builds/deploy-selected",
        data={
            "hostname": "pi-1.local",
            "vehicle_name": "uav_0",
            "network_role": "client",
            "allowed_ap_hosts": ["pi-ap-1.local", "pi-ap-2.local"],
            "ap_selection": "ordered_then_strongest",
        },
    )
    assert response.status_code == 200
    assert response.json()["success"] is True
    assert captured["target_ctx"].target.pi_host == "pi-1.local"
    assert captured["network_policy_override"].model_dump() == {
        "network_role": "client",
        "allowed_ap_hosts": ["pi-ap-1.local", "pi-ap-2.local"],
        "ap_selection": "ordered_then_strongest",
    }
    assert captured["session_assignments"] is None


def test_fleet_deploy_route_accepts_query_scoped_network_policy_fields(
    client, monkeypatch
):
    from backend.services import fleet as fleet_service

    async def fake_batch_action(ctx, *, action, devices=None, session_assignments=None):
        assert action == "deploy"
        assert devices is not None
        assert len(devices) == 1
        assert session_assignments is None
        selection = devices[0]
        assert selection.hostname == "pi-1.local"
        assert selection.vehicle_name == "uav_0"
        assert selection.network_role == "ap"
        assert selection.allowed_ap_vehicles == ["payload_0", "payload_1"]
        assert selection.ap_selection == "strongest"
        return {
            "success": True,
            "action": action,
            "build_source": {
                "success": True,
                "source": {
                    "kind": "none",
                    "summary": "none",
                },
            },
            "fleet_catalog": {
                "success": True,
                "source_kind": "none",
                "fleet_vehicles": [],
            },
            "fleet_vehicles": [],
            "results": [
                {
                    "hardware_id": "pi-1.local",
                    "hostname": "pi-1.local",
                    "addresses": [],
                    "service_name": None,
                    "service_type": None,
                    "matched_target_id": None,
                    "matched_label": None,
                    "saved": False,
                    "target_id": "pi-1.local",
                    "vehicle_name": "uav_0",
                    "fleet_vehicle": None,
                    "connection": None,
                    "current_build": None,
                    "runtime": None,
                    "readiness": {
                        "connected": False,
                        "build_installed": False,
                        "runtime_ready": False,
                        "vehicle_assigned": True,
                        "ready": False,
                        "notes": [],
                    },
                    "action": action,
                    "success": True,
                    "output": "deployed",
                    "error": None,
                }
            ],
            "summary": {
                "requested_devices": 1,
                "successful_devices": 1,
                "failed_devices": 0,
            },
            "error": None,
        }

    monkeypatch.setattr(fleet_service, "batch_action", fake_batch_action)

    response = client.post(
        "/api/fleet/deploy",
        params=[
            ("hostname", "pi-1.local"),
            ("vehicle_name", "uav_0"),
            ("network_role", "ap"),
            ("allowed_ap_vehicles", "payload_0"),
            ("allowed_ap_vehicles", "payload_1"),
            ("ap_selection", "strongest"),
        ],
    )
    assert response.status_code == 200
    assert response.json()["success"] is True


def test_fleet_deploy_route_accepts_body_session_assignments(client, monkeypatch):
    from backend.services import fleet as fleet_service

    async def fake_batch_action(ctx, *, action, devices=None, session_assignments=None):
        assert action == "deploy"
        assert devices is not None
        assert len(devices) == 1
        assert session_assignments == {
            "pi-1": "uav_0",
            "pi-ap-1": "payload_0",
        }
        selection = devices[0]
        assert selection.hostname == "pi-1.local"
        assert selection.vehicle_name == "uav_0"
        assert selection.network_role == "client"
        assert selection.allowed_ap_vehicles == ["payload_0"]
        assert selection.ap_selection == "ordered_then_strongest"
        return {
            "success": True,
            "action": action,
            "build_source": {
                "success": True,
                "source": {
                    "kind": "none",
                    "summary": "none",
                },
            },
            "fleet_catalog": {
                "success": True,
                "source_kind": "none",
                "fleet_vehicles": [],
            },
            "fleet_vehicles": [],
            "results": [
                {
                    "hardware_id": "pi-1.local",
                    "hostname": "pi-1.local",
                    "addresses": [],
                    "service_name": None,
                    "service_type": None,
                    "matched_target_id": None,
                    "matched_label": None,
                    "saved": False,
                    "target_id": "pi-1.local",
                    "vehicle_name": "uav_0",
                    "fleet_vehicle": None,
                    "connection": None,
                    "current_build": None,
                    "runtime": None,
                    "readiness": {
                        "connected": False,
                        "build_installed": False,
                        "runtime_ready": False,
                        "vehicle_assigned": True,
                        "ready": False,
                        "notes": [],
                    },
                    "action": action,
                    "success": True,
                    "output": "deployed",
                    "error": None,
                }
            ],
            "summary": {
                "requested_devices": 1,
                "successful_devices": 1,
                "failed_devices": 0,
            },
            "error": None,
        }

    monkeypatch.setattr(fleet_service, "batch_action", fake_batch_action)

    response = client.post(
        "/api/fleet/deploy",
        json={
            "devices": [
                {
                    "hostname": "pi-1.local",
                    "vehicle_name": "uav_0",
                    "network_role": "client",
                    "allowed_ap_vehicles": ["payload_0"],
                    "ap_selection": "ordered_then_strongest",
                }
            ],
            "session_assignments": {
                "pi-1.local": "uav_0",
                "pi-ap-1.local": "payload_0",
            },
        },
    )
    assert response.status_code == 200
    assert response.json()["success"] is True


def test_fleet_action_failure_response_marks_top_level_failure(client, monkeypatch):
    from backend.services import fleet as fleet_service

    async def fake_batch_action(ctx, *, action, devices=None, session_assignments=None):
        assert action == "deploy"
        assert session_assignments is None
        return {
            "success": False,
            "action": action,
            "build_source": {
                "success": True,
                "source": {
                    "kind": "none",
                    "summary": "none",
                },
            },
            "fleet_catalog": {
                "success": True,
                "source_kind": "none",
                "fleet_vehicles": [],
            },
            "fleet_vehicles": [],
            "results": [
                {
                    "hardware_id": "pi-1.local",
                    "hostname": "pi-1.local",
                    "addresses": [],
                    "service_name": None,
                    "service_type": None,
                    "matched_target_id": None,
                    "matched_label": None,
                    "saved": False,
                    "target_id": "pi-1.local",
                    "vehicle_name": "uav_0",
                    "fleet_vehicle": None,
                    "connection": None,
                    "current_build": {
                        "success": True,
                        "installed": True,
                        "info": "ok",
                        "release_id": "release-old",
                    },
                    "runtime": None,
                    "readiness": {
                        "connected": True,
                        "build_installed": True,
                        "runtime_ready": True,
                        "vehicle_assigned": True,
                        "ready": True,
                        "notes": [],
                    },
                    "action": action,
                    "success": False,
                    "attempted_release_id": "release-new",
                    "active_release_id": "release-old",
                    "rolled_back": True,
                    "output": None,
                    "error": "Restart runtime service failed: __ERR__:unstable",
                }
            ],
            "summary": {
                "requested_devices": 1,
                "successful_devices": 0,
                "failed_devices": 1,
            },
            "error": "One or more fleet actions failed.",
        }

    monkeypatch.setattr(fleet_service, "batch_action", fake_batch_action)

    response = client.post(
        "/api/fleet/deploy",
        params={"hostname": "pi-1.local", "vehicle_name": "uav_0"},
    )
    assert response.status_code == 200
    payload = response.json()
    assert payload["success"] is False
    assert payload["summary"]["failed_devices"] == 1
    assert payload["results"][0]["rolled_back"] is True
    assert payload["results"][0]["active_release_id"] == "release-old"


def test_build_source_routes_round_trip(client, tmp_path):
    fleet_path = _write_fleet(tmp_path)

    response = client.get("/api/build-source")
    assert response.status_code == 200
    assert response.json()["source"]["kind"] == "none"
    assert response.json()["source"]["fleet_file"] is None

    response = client.post(
        "/api/build-source/github",
        data={
            "source": "release",
            "tag": "build-deadbeef",
            "sha": "deadbee",
            "commit_subject": "Test release subject",
            "name": "build-deadbeef",
        },
    )
    assert response.status_code == 200
    payload = response.json()
    assert payload["success"] is True
    assert payload["source"]["kind"] == "github"
    assert payload["source"]["tag"] == "build-deadbeef"
    assert payload["source"]["commit_subject"] == "Test release subject"

    response = client.post("/api/build-source/local-codebase")
    assert response.status_code == 200
    payload = response.json()
    assert payload["success"] is True
    assert payload["source"]["kind"] == "local_codebase"
    assert payload["source"]["codebase_root"]
    assert payload["source"]["available_fleets"] == [fleet_path.name]
    assert payload["source"]["fleet_catalog_source"].endswith("src/uav/uav/fleets")

    response = client.post(
        "/api/build-source/fleet",
        data={"fleet_file": fleet_path.name},
    )
    assert response.status_code == 200
    payload = response.json()
    assert payload["success"] is True
    assert payload["source"]["kind"] == "local_codebase"
    assert payload["source"]["fleet_file"] == fleet_path.name
    assert payload["source"]["fleet_exists"] is True
    assert payload["source"]["fleet_error"] is None
    assert payload["source"]["fleet_vehicles"] == [
        {
            "name": "uav_0",
            "kind": None,
            "mission": "hover",
            "mission_path": None,
            "auto_launch": None,
            "debug": None,
            "vision_debug": None,
            "save_vision_milliseconds": None,
            "servo_only": None,
            "camera_mount_offsets": [],
            "camera_input_transport": None,
            "camera_rotate_degrees": None,
            "camera_preprocess_hook": None,
            "px4_airframe_id": None,
            "px4_namespace": None,
            "payload_controller": None,
            "network_role": None,
            "allowed_ap_vehicles": [],
            "ap_selection": None,
        }
    ]

    response = client.post(
        "/api/build-source/local-artifact",
        files={
            "file": ("hardware-build.tar.gz", b"artifact-bytes", "application/gzip")
        },
    )
    assert response.status_code == 200
    payload = response.json()
    assert payload["success"] is True
    assert payload["source"]["kind"] == "local_artifact"
    assert payload["source"]["artifact_name"] == "hardware-build.tar.gz"
    assert payload["source"]["local_artifact_exists"] is True

    response = client.post("/api/build-source/clear")
    assert response.status_code == 200
    payload = response.json()
    assert payload["success"] is True
    assert payload["source"]["kind"] == "none"


def test_build_source_catalog_endpoint_exposes_available_fleets(client, tmp_path):
    primary_fleet = _write_fleet(tmp_path, name="primary.yaml")
    _write_fleet(tmp_path, name="backup.yaml")

    response = client.post("/api/build-source/local-codebase")
    assert response.status_code == 200

    response = client.get("/api/build-source/fleet-catalog")
    assert response.status_code == 200
    payload = response.json()
    assert payload["success"] is True
    assert payload["source_kind"] == "local_codebase"
    assert sorted(payload["available_fleets"]) == [
        "backup.yaml",
        "primary.yaml",
    ]

    response = client.post(
        "/api/build-source/fleet",
        data={"fleet_file": primary_fleet.name},
    )
    assert response.status_code == 200
    payload = response.json()
    assert payload["success"] is True
    assert payload["source"]["fleet_file"] == primary_fleet.name
    assert payload["source"]["fleet_exists"] is True
    assert payload["source"]["fleet_error"] is None
    assert sorted(payload["source"]["available_fleets"]) == [
        "backup.yaml",
        "primary.yaml",
    ]
    assert payload["source"]["fleet_vehicles"][0]["name"] == "uav_0"

    response = client.get("/api/build-source/fleet-catalog")
    assert response.status_code == 200
    payload = response.json()
    assert payload["success"] is True
    assert payload["selected_fleet_file"] == primary_fleet.name
    assert payload["fleet_exists"] is True
    assert payload["fleet_error"] is None
    assert payload["fleet_vehicles"][0]["name"] == "uav_0"
