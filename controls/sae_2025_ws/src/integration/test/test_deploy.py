from __future__ import annotations

import asyncio
import sys
import tarfile
import types
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

backend_pkg = types.ModuleType("backend")
backend_pkg.__path__ = [str(PACKAGE_ROOT / "backend")]
sys.modules.setdefault("backend", backend_pkg)

context_pkg = types.ModuleType("backend.context")
context_pkg.AppContext = type("AppContext", (), {})
context_pkg.TargetContext = type("TargetContext", (), {})
sys.modules.setdefault("backend.context", context_pkg)

services_pkg = types.ModuleType("backend.services")
services_pkg.__path__ = [str(PACKAGE_ROOT / "backend" / "services")]
sys.modules.setdefault("backend.services", services_pkg)

from backend.config import (  # noqa: E402
    BuildSourceStore,
    InventoryStore,
    OperatorConfig,
    TargetRecord,
    build_source_store_paths,
)
from backend.services import deploy as deploy_service  # noqa: E402


class _FakeSSHResult:
    def __init__(self, *, returncode: int = 0, stdout: str = "", stderr: str = ""):
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


class _FakeSSH:
    def __init__(self, *, result: _FakeSSHResult):
        self.result = result
        self.run_calls: list[str] = []
        self.scp_calls: list[tuple[str, str]] = []

    def q(self, value: str) -> str:
        return value

    async def run(self, command: str, timeout: int = 15):
        self.run_calls.append(command)
        return self.result

    async def scp(self, local_path: str, remote_path: str, timeout: int = 300):
        self.scp_calls.append((local_path, remote_path))
        return _FakeSSHResult()

    def friendly_error(self, stderr: str) -> str:
        return (stderr or "").strip() or "ssh error"

    def friendly_timeout(self) -> str:
        return "ssh timeout"

    def format_remote_error(self, raw_error: str, prefix: str) -> str:
        return f"{prefix}: {raw_error}"


class _FakeResponse:
    def __init__(self, payload: object):
        self.payload = payload

    def raise_for_status(self) -> None:
        return None

    def json(self) -> object:
        return self.payload


class _FakeAsyncClient:
    def __init__(
        self,
        *,
        releases_payload: dict,
        artifacts_payload: dict,
        commit_payloads: dict[str, dict] | None = None,
    ):
        self.releases_payload = releases_payload
        self.artifacts_payload = artifacts_payload
        self.commit_payloads = commit_payloads or {}
        self.requests: list[str] = []

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc, tb):
        return False

    async def get(self, url: str, headers=None):
        self.requests.append(url)
        if "/releases?per_page=100&page=" in url:
            page = url.rsplit("=", 1)[-1]
            if page == "1":
                return _FakeResponse(self.releases_payload)
            return _FakeResponse([])
        if "/commits/" in url:
            commit_sha = url.rsplit("/", 1)[-1]
            return _FakeResponse(
                self.commit_payloads.get(
                    commit_sha,
                    {"commit": {"message": ""}},
                )
            )
        if url.endswith("/releases"):
            return _FakeResponse(self.releases_payload)
        if url.endswith("/actions/artifacts?per_page=30"):
            return _FakeResponse(self.artifacts_payload)
        raise AssertionError(f"Unexpected GET URL in test: {url}")


def _make_operator(tmp_path: Path) -> OperatorConfig:
    return OperatorConfig(
        github_repo="pennaerial/monorepo",
        github_token="secret-token",
        hotspot_name="pennair-hotspot",
        inventory_path=tmp_path / ".integration_inventory.json",
        default_deploy_root="/home/penn/pennair-deploy",
        default_pi_user="penn",
        default_ssh_key="",
        default_ssh_pass="",
    )


def _make_target(
    *,
    target_id: str,
    fleet_file: Path,
    vehicle_name: str,
    overlay_yaml: str,
    pi_host: str = "pi.local",
) -> TargetRecord:
    return TargetRecord.from_dict(
        {
            "target_id": target_id,
            "label": target_id,
            "pi_user": "penn",
            "pi_host": pi_host,
            "deploy_root": "/home/penn/pennair-deploy",
            "ssh_key": "",
            "ssh_pass": "",
            "fleet_file": str(fleet_file),
            "vehicle_name": vehicle_name,
            "overlay_yaml": overlay_yaml,
            "service_unit": "pennair-autonomy.service",
            "enabled": True,
        },
        default_deploy_root="/home/penn/pennair-deploy",
        default_fleet_file=str(fleet_file),
    )


def _make_context(tmp_path: Path, target: TargetRecord) -> SimpleNamespace:
    base_dir = tmp_path / "src" / "integration"
    base_dir.mkdir(parents=True, exist_ok=True)
    operator = _make_operator(tmp_path)
    inventory = InventoryStore(
        operator.inventory_path,
        operator_config=operator,
        default_deploy_root=operator.default_deploy_root,
        default_fleet_file=target.fleet_file,
        seed_target=target,
    )
    build_source_path, cache_dir = build_source_store_paths(operator.inventory_path)
    return SimpleNamespace(
        base_dir=base_dir,
        operator_config=operator,
        inventory=inventory,
        build_source_store=BuildSourceStore(
            build_source_path,
            cache_dir=cache_dir,
            default_fleet_file=target.fleet_file,
        ),
        resolve_target=lambda target_id=None: SimpleNamespace(
            target=inventory.get_target(target_id),
            ssh=_FakeSSH(result=_FakeSSHResult()),
        ),
    )


def _write_mission(root: Path, name: str, mission_class: str) -> Path:
    path = root / f"{name}.yaml"
    path.write_text(
        f"modes:\n  start:\n    class: {mission_class}\n",
        encoding="utf-8",
    )
    return path


def _write_fleet(tmp_path: Path, content: dict) -> Path:
    path = tmp_path / "fleet.yaml"
    path.write_text(yaml.safe_dump(content, sort_keys=False), encoding="utf-8")
    return path


def _write_source_bundle(tmp_path: Path, *, package_dir_name: str = "uav") -> Path:
    bundle_root = tmp_path / "bundle-root"
    package_dir = bundle_root / "src" / package_dir_name
    package_dir.mkdir(parents=True, exist_ok=True)
    (package_dir / "package.xml").write_text(
        (
            '<package format="3">\n'
            f"  <name>{package_dir_name}</name>\n"
            "  <version>0.0.0</version>\n"
            "  <description>test</description>\n"
            '  <maintainer email="test@example.com">Test</maintainer>\n'
            "  <license>MIT</license>\n"
            "</package>\n"
        ),
        encoding="utf-8",
    )
    (package_dir / "marker.txt").write_text("hello\n", encoding="utf-8")
    bundle_path = tmp_path / "source-bundle.tar.gz"
    with tarfile.open(bundle_path, "w:gz") as archive:
        archive.add(bundle_root / "src", arcname="src")
    return bundle_path


def test_render_runtime_fleet_for_uav(tmp_path, monkeypatch):
    missions_root = tmp_path / "src" / "uav" / "uav" / "missions"
    missions_root.mkdir(parents=True, exist_ok=True)
    _write_mission(missions_root, "hover", "uav.modes.uav.TakeoffMode")

    fleet_file = _write_fleet(
        tmp_path,
        {
            "defaults": {
                "auto_launch": False,
                "debug": False,
                "vision_debug": False,
                "save_vision_milliseconds": 0,
                "servo_only": False,
                "camera_mount_offsets": [0.0, 0.0, 0.0],
                "camera_input_transport": "raw",
                "camera_rotate_degrees": 0.0,
                "camera_preprocess_hook": "",
            },
            "vehicles": [
                {
                    "name": "uav_0",
                    "mission": "hover",
                    "sim_entity_name": "uav_backend_0",
                    "camera_mount_offsets": [0.1, 0.2, 0.3],
                }
            ],
        },
    )

    target = _make_target(
        target_id="pi-1",
        fleet_file=fleet_file,
        vehicle_name="uav_0",
        overlay_yaml=(
            "auto_launch: true\n"
            "debug: true\n"
            "vision_debug: true\n"
            "save_vision_milliseconds: 2500\n"
            "servo_only: true\n"
            "camera_mount_offsets: [1.0, 2.0, 3.0]\n"
            "camera_input_transport: compressed\n"
            "camera_rotate_degrees: 15\n"
            "camera_preprocess_hook: rotate_then_crop\n"
            "px4_airframe_id: 4004\n"
        ),
    )
    ctx = _make_context(tmp_path, target)

    monkeypatch.setattr(
        deploy_service,
        "_missions_root",
        lambda _ctx: missions_root,
    )
    monkeypatch.setattr(
        deploy_service.MissionSpec,
        "load",
        staticmethod(lambda _path: SimpleNamespace(target="uav")),
    )

    runtime_fleet_yaml, shared_fleet_yaml, local_mission_path = (
        deploy_service._render_runtime_fleet(ctx, ctx.resolve_target())
    )

    runtime = yaml.safe_load(runtime_fleet_yaml)
    shared = yaml.safe_load(shared_fleet_yaml)

    assert shared["vehicles"][0]["name"] == "uav_0"
    assert Path(local_mission_path).name == "hover.yaml"
    assert runtime["backend"] == {"kind": "hardware"}
    assert runtime["vehicles"] == [
        {
            "name": "uav_0",
            "kind": "uav",
            "mission": "hover",
            "mission_path": "/home/penn/pennair-deploy/config/missions/hover.yaml",
            "auto_launch": True,
            "debug": True,
            "vision_debug": True,
            "save_vision_milliseconds": 2500,
            "servo_only": True,
            "camera_mount_offsets": [1.0, 2.0, 3.0],
            "camera_input_transport": "compressed",
            "camera_rotate_degrees": 15.0,
            "camera_preprocess_hook": "rotate_then_crop",
            "px4_airframe_id": 4004,
            "px4_namespace": "uav_0",
        }
    ]


def test_render_runtime_fleet_for_payload_defaults(tmp_path, monkeypatch):
    missions_root = tmp_path / "src" / "uav" / "uav" / "missions"
    missions_root.mkdir(parents=True, exist_ok=True)
    _write_mission(
        missions_root, "payload_retreat", "uav.modes.payload.PayloadRetreatMode"
    )

    fleet_file = _write_fleet(
        tmp_path,
        {
            "defaults": {
                "auto_launch": False,
                "debug": False,
                "vision_debug": False,
                "save_vision_milliseconds": 0,
                "servo_only": False,
                "camera_mount_offsets": [0.0, 0.0, 0.0],
                "camera_input_transport": "compressed",
                "camera_preprocess_hook": "",
            },
            "vehicles": [
                {
                    "name": "payload_0",
                    "mission": "payload_retreat",
                    "sim_entity_name": "payload_backend_0",
                }
            ],
        },
    )
    target = _make_target(
        target_id="pi-2",
        fleet_file=fleet_file,
        vehicle_name="payload_0",
        overlay_yaml="auto_launch: false\npayload_controller: SimController\n",
        pi_host="pi-2.local",
    )
    ctx = _make_context(tmp_path, target)

    monkeypatch.setattr(
        deploy_service,
        "_missions_root",
        lambda _ctx: missions_root,
    )
    monkeypatch.setattr(
        deploy_service.MissionSpec,
        "load",
        staticmethod(lambda _path: SimpleNamespace(target="payload")),
    )

    runtime_fleet_yaml, _, _ = deploy_service._render_runtime_fleet(
        ctx, ctx.resolve_target()
    )
    runtime = yaml.safe_load(runtime_fleet_yaml)

    assert runtime["backend"] == {"kind": "hardware"}
    assert runtime["vehicles"] == [
        {
            "name": "payload_0",
            "kind": "payload",
            "mission": "payload_retreat",
            "mission_path": "/home/penn/pennair-deploy/config/missions/payload_retreat.yaml",
            "auto_launch": False,
            "debug": False,
            "vision_debug": False,
            "save_vision_milliseconds": 0,
            "servo_only": False,
            "camera_mount_offsets": [0.0, 0.0, 0.0],
            "camera_input_transport": "compressed",
            "camera_rotate_degrees": 180.0,
            "camera_preprocess_hook": "",
            "payload_controller": "SimController",
        }
    ]


def test_validate_overlay_preview_rejects_unsupported_fields(tmp_path, monkeypatch):
    missions_root = tmp_path / "src" / "uav" / "uav" / "missions"
    missions_root.mkdir(parents=True, exist_ok=True)
    _write_mission(missions_root, "hover", "uav.modes.uav.TakeoffMode")
    fleet_file = _write_fleet(
        tmp_path,
        {"vehicles": [{"name": "uav_0", "mission": "hover"}]},
    )
    target = _make_target(
        target_id="pi-validate",
        fleet_file=fleet_file,
        vehicle_name="uav_0",
        overlay_yaml="mission: hover\npx4_airframe_id: 4004\n",
    )
    ctx = _make_context(tmp_path, target)

    monkeypatch.setattr(deploy_service, "_missions_root", lambda _ctx: missions_root)
    monkeypatch.setattr(
        deploy_service.MissionSpec,
        "load",
        staticmethod(lambda _path: SimpleNamespace(target="uav")),
    )

    with pytest.raises(ValueError, match="unsupported fields"):
        deploy_service.validate_overlay_preview(
            ctx,
            ctx.resolve_target(),
            "mission: hover\npx4_airframe_id: 4004\nunknown: true\n",
        )


def test_release_metadata_payload_and_summary(tmp_path):
    target = _make_target(
        target_id="pi-meta",
        fleet_file=tmp_path / "fleet.yaml",
        vehicle_name="uav_0",
        overlay_yaml="mission: hover\npx4_airframe_id: 4004\n",
    )
    target_ctx = SimpleNamespace(target=target)

    metadata = deploy_service._release_metadata_payload(
        target_ctx,
        release_id="20260411-build-hover",
        source_type="source-build",
        source_label="local-uav.tar.gz",
        fleet_file=str(target.fleet_file),
        package_names=["uav", "uav_interfaces"],
    )

    assert metadata["release_id"] == "20260411-build-hover"
    assert metadata["source_type"] == "source-build"
    assert metadata["fleet_file"] == str(target.fleet_file)
    assert metadata["vehicle_name"] == "uav_0"
    assert metadata["packages"] == ["uav", "uav_interfaces"]

    summary = deploy_service._release_metadata_summary(metadata)
    assert "Release: 20260411-build-hover" in summary
    assert "Source: source-build" in summary
    assert "Packages: uav, uav_interfaces" in summary


def test_normalize_source_bundle_extracts_package_names(tmp_path):
    bundle_path = _write_source_bundle(tmp_path, package_dir_name="uav")

    normalized_path, package_names, sanitized_name = (
        deploy_service._normalize_source_bundle(str(bundle_path), "uav-bundle.tar.gz")
    )

    assert package_names == ["uav"]
    assert sanitized_name == "uav-bundle.tar.gz"

    extract_dir = tmp_path / "normalized"
    extract_dir.mkdir()
    with tarfile.open(normalized_path, "r:gz") as archive:
        archive.extractall(extract_dir)

    assert (extract_dir / "src" / "uav" / "package.xml").exists()
    assert (extract_dir / "src" / "uav" / "marker.txt").exists()


def test_build_local_source_bundle_scopes_packages_for_target(tmp_path, monkeypatch):
    repo_src = tmp_path / "src"
    repo_src.mkdir(parents=True, exist_ok=True)
    for package_name in (
        "actuator_msgs",
        "payload_interfaces",
        "px4_msgs",
        "uav_interfaces",
        "uav",
        "payload",
    ):
        package_dir = repo_src / package_name
        package_dir.mkdir(parents=True, exist_ok=True)
        (package_dir / "package.xml").write_text(
            (
                '<package format="3">\n'
                f"  <name>{package_name}</name>\n"
                "  <version>0.0.0</version>\n"
                "  <description>test</description>\n"
                '  <maintainer email="test@example.com">Test</maintainer>\n'
                "  <license>MIT</license>\n"
                "</package>\n"
            ),
            encoding="utf-8",
        )

    missions_root = tmp_path / "src" / "uav" / "uav" / "missions"
    missions_root.mkdir(parents=True, exist_ok=True)
    _write_mission(
        missions_root,
        "payload_retreat",
        "uav.modes.payload.PayloadRetreatMode",
    )

    fleet_file = _write_fleet(
        tmp_path,
        {
            "vehicles": [
                {
                    "name": "payload_0",
                    "mission": "payload_retreat",
                }
            ],
        },
    )
    target = _make_target(
        target_id="pi-payload",
        fleet_file=fleet_file,
        vehicle_name="payload_0",
        overlay_yaml="mission: payload_retreat\npayload_controller: GPIOController\n",
    )
    ctx = _make_context(tmp_path, target)

    monkeypatch.setattr(deploy_service, "_missions_root", lambda _ctx: missions_root)
    monkeypatch.setattr(
        deploy_service.MissionSpec,
        "load",
        staticmethod(lambda _path: SimpleNamespace(target="payload")),
    )

    bundle_path, package_names, bundle_name = deploy_service._build_local_source_bundle(
        ctx, ctx.resolve_target()
    )

    assert package_names == [
        "actuator_msgs",
        "payload_interfaces",
        "px4_msgs",
        "uav_interfaces",
        "uav",
        "payload",
    ]
    assert bundle_name.startswith("local-codebase-")

    extract_dir = tmp_path / "local-source"
    extract_dir.mkdir()
    with tarfile.open(bundle_path, "r:gz") as archive:
        archive.extractall(extract_dir)

    for package_name in package_names:
        assert (extract_dir / "src" / package_name / "package.xml").exists()


def test_list_builds_combines_releases_and_artifacts(tmp_path, monkeypatch):
    ctx = SimpleNamespace(
        base_dir=tmp_path / "src" / "integration",
        operator_config=_make_operator(tmp_path),
        inventory=InventoryStore(
            tmp_path / ".integration_inventory.json",
            operator_config=_make_operator(tmp_path),
            default_deploy_root="/home/penn/pennair-deploy",
            default_fleet_file="/tmp/fleet.yaml",
            seed_target=_make_target(
                target_id="pi-1",
                fleet_file=tmp_path / "fleet.yaml",
                vehicle_name="uav_0",
                overlay_yaml="mission: hover\npx4_airframe_id: 4004\n",
            ),
        ),
    )

    fake_httpx = SimpleNamespace(
        AsyncClient=lambda timeout=20: _FakeAsyncClient(
            releases_payload=[
                {
                    "tag_name": "build-deadbeef",
                    "name": "build-deadbeef",
                    "published_at": "2026-04-11T00:00:00Z",
                    "assets": [
                        {
                            "browser_download_url": "https://example.com/build.tar.gz",
                            "size": 1234567,
                        }
                    ],
                },
                {
                    "tag_name": "not-a-build",
                    "name": "skip-me",
                    "published_at": "2026-04-11T00:00:00Z",
                    "assets": [],
                },
            ],
            artifacts_payload={
                "artifacts": [
                    {
                        "id": 99,
                        "name": "arm-artifact",
                        "updated_at": "2026-04-11T01:00:00Z",
                        "size_in_bytes": 8_000_000,
                        "workflow_run": {
                            "id": 42,
                            "head_sha": "cafebabedeadbeef",
                            "head_branch": "feature/release-metadata",
                        },
                    }
                ]
            },
            commit_payloads={
                "deadbeef": {"commit": {"message": "Release commit subject\n\nbody"}},
                "cafebabedeadbeef": {
                    "commit": {"message": "Artifact commit subject\n\nbody"}
                },
            },
        )
    )
    monkeypatch.setattr(deploy_service, "_require_httpx", lambda: fake_httpx)

    result = asyncio.run(deploy_service.list_builds(ctx))

    assert result["success"] is True
    assert len(result["builds"]) == 2
    assert result["builds"][0] == {
        "source": "release",
        "tag": "build-deadbeef",
        "sha": "deadbee",
        "commit_sha": "deadbeef",
        "commit_subject": "Release commit subject",
        "name": "build-deadbeef",
        "date": "2026-04-11",
        "download_url": "https://example.com/build.tar.gz",
        "size_mb": 1.2,
    }
    assert result["builds"][1]["source"] == "actions"
    assert result["builds"][1]["commit_sha"] == "cafebabedeadbeef"
    assert result["builds"][1]["commit_subject"] == "Artifact commit subject"
    assert result["builds"][1]["artifact_id"] == "99"
    assert result["builds"][1]["run_id"] == "42"
    assert result["builds"][1]["branch"] == "feature/release-metadata"
    assert len(result["releases"]) == 1
    assert len(result["artifacts"]) == 1


def test_current_build_parses_release_marker(tmp_path):
    target = _make_target(
        target_id="pi-3",
        fleet_file=tmp_path / "fleet.yaml",
        vehicle_name="uav_0",
        overlay_yaml="mission: hover\npx4_airframe_id: 4004\n",
    )
    fake_target_ctx = SimpleNamespace(
        target=target,
        ssh=_FakeSSH(
            result=_FakeSSHResult(
                stdout="Build installed\n__RELEASE__:release-123\n",
                stderr="",
            )
        ),
    )
    ctx = _make_context(tmp_path, target)
    ctx.resolve_target = lambda target_id=None: fake_target_ctx  # type: ignore[method-assign]

    result = asyncio.run(deploy_service.current_build(ctx))

    assert result == {
        "success": True,
        "installed": True,
        "info": "Build installed",
        "release_id": "release-123",
    }


def test_current_build_includes_release_metadata(tmp_path):
    target = _make_target(
        target_id="pi-meta-build",
        fleet_file=tmp_path / "fleet.yaml",
        vehicle_name="uav_0",
        overlay_yaml="mission: hover\npx4_airframe_id: 4004\n",
    )
    metadata = yaml.safe_dump(
        {
            "release_id": "release-456",
            "source_type": "source-build",
            "source_label": "uav-bundle.tar.gz",
            "target_id": "pi-meta-build",
            "vehicle_name": "uav_0",
            "packages": ["uav"],
        },
        sort_keys=False,
    ).strip()
    fake_target_ctx = SimpleNamespace(
        target=target,
        ssh=_FakeSSH(
            result=_FakeSSHResult(
                stdout=(
                    "__META_BEGIN__\n"
                    f"{metadata}\n"
                    "__META_END__\n"
                    "__INFO_BEGIN__\n"
                    "Build Information\n"
                    "__INFO_END__\n"
                    "__RELEASE__:release-456\n"
                ),
                stderr="",
            )
        ),
    )
    ctx = _make_context(tmp_path, target)
    ctx.resolve_target = lambda target_id=None: fake_target_ctx  # type: ignore[method-assign]

    result = asyncio.run(deploy_service.current_build(ctx))

    assert result["success"] is True
    assert result["installed"] is True
    assert result["release_id"] == "release-456"
    assert "Source: source-build" in result["info"]
    assert "Packages: uav" in result["info"]


def test_sanitize_artifact_name_rejects_invalid_names():
    assert (
        deploy_service.sanitize_artifact_name("build-123.tar.gz") == "build-123.tar.gz"
    )
    with pytest.raises(ValueError, match="unsupported characters"):
        deploy_service.sanitize_artifact_name("bad name.tar.gz")


def test_build_source_round_trip_and_local_artifact_cache(tmp_path):
    target = _make_target(
        target_id="pi-source",
        fleet_file=tmp_path / "fleet.yaml",
        vehicle_name="uav_0",
        overlay_yaml="mission: hover\npx4_airframe_id: 4004\n",
    )
    ctx = _make_context(tmp_path, target)

    result = asyncio.run(
        deploy_service.set_github_build_source(
            ctx,
            source="release",
            tag="build-deadbeef",
            sha="deadbee",
            name="build-deadbeef",
        )
    )
    assert result["success"] is True
    assert result["source"]["kind"] == "github"
    assert result["source"]["tag"] == "build-deadbeef"

    result = asyncio.run(
        deploy_service.set_local_artifact_build_source(
            ctx,
            filename="hardware-build.tar.gz",
            file_bytes=b"deploy-bundle",
        )
    )
    assert result["success"] is True
    assert result["source"]["kind"] == "local_artifact"
    assert result["source"]["artifact_name"] == "hardware-build.tar.gz"
    assert result["source"]["local_artifact_exists"] is True
    assert Path(result["source"]["local_artifact_path"]).exists()


def test_deploy_selected_source_dispatches_to_persisted_selection(
    tmp_path, monkeypatch
):
    target = _make_target(
        target_id="pi-dispatch",
        fleet_file=tmp_path / "fleet.yaml",
        vehicle_name="uav_0",
        overlay_yaml="mission: hover\npx4_airframe_id: 4004\n",
    )
    ctx = _make_context(tmp_path, target)

    calls: list[tuple[str, dict[str, object]]] = []

    async def fake_download_build(inner_ctx, **kwargs):
        calls.append(("github", kwargs))
        return {"success": True, "output": "github deploy"}

    async def fake_deploy_artifact_path(inner_ctx, **kwargs):
        calls.append(("local_artifact", kwargs))
        return {"success": True, "output": "artifact deploy"}

    async def fake_deploy_local_codebase(inner_ctx, **kwargs):
        calls.append(("local_codebase", kwargs))
        return {"success": True, "output": "codebase deploy"}

    monkeypatch.setattr(deploy_service, "download_build", fake_download_build)
    monkeypatch.setattr(
        deploy_service, "_deploy_artifact_path", fake_deploy_artifact_path
    )
    monkeypatch.setattr(
        deploy_service, "deploy_local_codebase", fake_deploy_local_codebase
    )

    asyncio.run(
        deploy_service.set_github_build_source(
            ctx,
            source="actions",
            artifact_id="42",
            run_id="99",
            name="arm-artifact",
        )
    )
    result = asyncio.run(
        deploy_service.deploy_selected_source(ctx, target_id="pi-dispatch")
    )
    assert result["success"] is True
    assert calls[-1] == (
        "github",
        {
            "target_id": "pi-dispatch",
            "tag": "",
            "source": "actions",
            "artifact_id": "42",
        },
    )

    asyncio.run(
        deploy_service.set_local_artifact_build_source(
            ctx,
            filename="artifact.tar.gz",
            file_bytes=b"artifact",
        )
    )
    result = asyncio.run(
        deploy_service.deploy_selected_source(ctx, target_id="pi-dispatch")
    )
    assert result["success"] is True
    assert calls[-1][0] == "local_artifact"
    assert calls[-1][1]["target_id"] == "pi-dispatch"
    assert calls[-1][1]["artifact_name"] == "artifact.tar.gz"

    asyncio.run(deploy_service.set_local_codebase_build_source(ctx))
    result = asyncio.run(
        deploy_service.deploy_selected_source(ctx, target_id="pi-dispatch")
    )
    assert result["success"] is True
    assert calls[-1] == ("local_codebase", {"target_id": "pi-dispatch"})
