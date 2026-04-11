from __future__ import annotations

import sys
import types
from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

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
            "fleet_file": "/tmp/fleet.yaml",
            "vehicle_name": "uav_0",
            "overlay_yaml": "mission: hover\npx4_airframe_id: 4004\n",
            "service_unit": "pennair-autonomy.service",
            "enabled": True,
        },
        default_deploy_root="/home/penn/pennair-deploy",
        default_fleet_file="/tmp/fleet.yaml",
    )


def _make_context(tmp_path: Path) -> SimpleNamespace:
    base_dir = tmp_path / "src" / "integration"
    base_dir.mkdir(parents=True, exist_ok=True)
    operator = _make_operator(tmp_path)
    target = _make_target()
    inventory = InventoryStore(
        operator.inventory_path,
        operator_config=operator,
        default_deploy_root=operator.default_deploy_root,
        default_fleet_file=target.fleet_file,
        seed_target=target,
    )
    build_source_path, cache_dir = build_source_store_paths(operator.inventory_path)
    build_source_store = BuildSourceStore(build_source_path, cache_dir=cache_dir)

    def resolve_target(target_id: str | None = None):
        resolved = inventory.get_target(target_id)
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
        resolve_target=resolve_target,
        list_targets=inventory.list_targets,
    )


@pytest.fixture
def client(tmp_path, monkeypatch):
    ctx = _make_context(tmp_path)
    monkeypatch.setattr("backend.app_factory.create_context", lambda base_dir: ctx)
    app = create_app(tmp_path / "src" / "integration")
    return TestClient(app)


def test_config_is_operator_global_only(client):
    response = client.get("/api/config")
    assert response.status_code == 200
    payload = response.json()

    assert payload["success"] is True
    assert "config" in payload
    assert "active_target_id" not in payload
    assert "workspace_paths" not in payload


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
            {"name": "hover", "content": "modes: {}\n"},
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
def test_target_scoped_http_routes_require_target_id(client, method, url, data):
    request = getattr(client, method)
    kwargs = {}
    if data is not None:
        if method == "get":
            kwargs["params"] = data
        else:
            kwargs["data"] = data
    response = request(url, **kwargs)
    assert response.status_code == 422


def test_target_scoped_upload_routes_require_target_id(client):
    response = client.post(
        "/api/builds/upload",
        files={"file": ("build.tar.gz", b"artifact", "application/gzip")},
    )
    assert response.status_code == 422

    response = client.post(
        "/api/builds/upload-source",
        files={"file": ("source.tar.gz", b"bundle", "application/gzip")},
    )
    assert response.status_code == 422


def test_terminal_websocket_requires_target_id(client):
    with pytest.raises(Exception):
        with client.websocket_connect("/ws/mission/terminal"):
            pass


def test_build_source_routes_round_trip(client):
    response = client.get("/api/build-source")
    assert response.status_code == 200
    assert response.json()["source"]["kind"] == "none"

    response = client.post(
        "/api/build-source/github",
        data={
            "source": "release",
            "tag": "build-deadbeef",
            "sha": "deadbee",
            "name": "build-deadbeef",
        },
    )
    assert response.status_code == 200
    payload = response.json()
    assert payload["success"] is True
    assert payload["source"]["kind"] == "github"
    assert payload["source"]["tag"] == "build-deadbeef"

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

    response = client.post("/api/build-source/local-codebase")
    assert response.status_code == 200
    payload = response.json()
    assert payload["success"] is True
    assert payload["source"]["kind"] == "local_codebase"
    assert payload["source"]["codebase_root"]

    response = client.post("/api/build-source/clear")
    assert response.status_code == 200
    payload = response.json()
    assert payload["success"] is True
    assert payload["source"]["kind"] == "none"
