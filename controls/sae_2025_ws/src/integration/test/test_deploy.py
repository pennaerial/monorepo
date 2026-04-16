from __future__ import annotations

import asyncio
import io
import inspect
import sys
import tarfile
import types
from pathlib import Path
from types import SimpleNamespace
import zipfile

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))
UAV_ROOT = PACKAGE_ROOT.parent / "uav"
if str(UAV_ROOT) not in sys.path:
    sys.path.insert(0, str(UAV_ROOT))

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
    def __init__(
        self,
        payload: object,
        *,
        status_code: int = 200,
        headers: dict[str, str] | None = None,
    ):
        self.payload = payload
        self.status_code = status_code
        self.headers = headers or {}
        self.text = str(payload)

    def raise_for_status(self) -> None:
        if self.status_code >= 400:
            raise _FakeHTTPStatusError(self)
        return None

    def json(self) -> object:
        return self.payload


class _FakeHTTPStatusError(Exception):
    def __init__(self, response: _FakeResponse):
        self.response = response
        super().__init__(response.text)


class _FakeAsyncClient:
    def __init__(
        self,
        *,
        releases_payload: dict,
        artifacts_payload: dict,
        commit_payloads: dict[str, dict] | None = None,
        response_overrides: dict[str, _FakeResponse] | None = None,
    ):
        self.releases_payload = releases_payload
        self.artifacts_payload = artifacts_payload
        self.commit_payloads = commit_payloads or {}
        self.response_overrides = response_overrides or {}
        self.requests: list[str] = []

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc, tb):
        return False

    async def get(self, url: str, headers=None):
        self.requests.append(url)
        override = self.response_overrides.get(url)
        if override is not None:
            return override
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
        if "/actions/artifacts?per_page=" in url:
            return _FakeResponse(self.artifacts_payload)
        raise AssertionError(f"Unexpected GET URL in test: {url}")


class _FakeStreamResponse:
    def __init__(self, body: bytes):
        self.body = body

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        return False

    def raise_for_status(self) -> None:
        return None

    def iter_bytes(self, chunk_size: int = 65536):
        for start in range(0, len(self.body), chunk_size):
            yield self.body[start : start + chunk_size]


class _FakeHTTPXClient:
    def __init__(self, *, body: bytes):
        self.body = body
        self.requests: list[tuple[str, str]] = []

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        return False

    def stream(self, method: str, url: str, headers=None):
        self.requests.append((method, url))
        return _FakeStreamResponse(self.body)


@pytest.fixture(autouse=True)
def _clear_deploy_service_caches():
    deploy_service._BUILD_LIST_CACHE.clear()
    deploy_service._FLEET_CATALOG_CACHE.clear()
    deploy_service._COMMIT_SUBJECT_CACHE.clear()
    deploy_service._WORKFLOW_RUN_CACHE.clear()
    yield
    deploy_service._BUILD_LIST_CACHE.clear()
    deploy_service._FLEET_CATALOG_CACHE.clear()
    deploy_service._COMMIT_SUBJECT_CACHE.clear()
    deploy_service._WORKFLOW_RUN_CACHE.clear()


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
            "vehicle_name": vehicle_name,
            "overlay_yaml": overlay_yaml,
            "service_unit": "pennair-autonomy.service",
            "enabled": True,
        },
        default_deploy_root="/home/penn/pennair-deploy",
    )


def _make_context(
    tmp_path: Path,
    target: TargetRecord,
    *,
    fleet_file: Path | None = None,
) -> SimpleNamespace:
    base_dir = tmp_path / "src" / "integration"
    base_dir.mkdir(parents=True, exist_ok=True)
    operator = _make_operator(tmp_path)
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
    if fleet_file is not None:
        build_source_store.set_fleet_file(fleet_file=fleet_file.name)
        build_source_store.set_local_codebase(codebase_root=str(tmp_path))

    def require_deploy_context(require_build_source: bool = True) -> None:
        current = build_source_store.current()
        if require_build_source and current.kind == "none":
            raise ValueError(
                "Select a build source before opening per-device deploy controls."
            )
        if not current.fleet_file:
            raise ValueError(
                "Select a fleet file before opening per-device deploy controls."
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
            ssh=_FakeSSH(result=_FakeSSHResult()),
        )

    return SimpleNamespace(
        base_dir=base_dir,
        operator_config=operator,
        inventory=inventory,
        build_source_store=build_source_store,
        require_deploy_context=require_deploy_context,
        resolve_target=lambda target_id=None: SimpleNamespace(
            target=inventory.get_target(target_id),
            ssh=_FakeSSH(result=_FakeSSHResult()),
        ),
        resolve_live_target=resolve_live_target,
    )


def _write_mission(root: Path, name: str, mission_mode: str) -> Path:
    path = root / f"{name}.yaml"
    path.write_text(
        f"modes:\n  start:\n    mode: {mission_mode}\n",
        encoding="utf-8",
    )
    return path


def _write_fleet(tmp_path: Path, content: dict, *, name: str = "fleet.yaml") -> Path:
    path = tmp_path / "src" / "uav" / "uav" / "fleets" / name
    path.parent.mkdir(parents=True, exist_ok=True)
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


def _write_fleet_bundle(tmp_path: Path, *, fleet_names: tuple[str, ...]) -> Path:
    bundle_root = tmp_path / "fleet-bundle"
    fleets_root = bundle_root / "install" / "uav" / "share" / "uav" / "fleets"
    fleets_root.mkdir(parents=True, exist_ok=True)
    for fleet_name in fleet_names:
        (fleets_root / fleet_name).write_text(
            yaml.safe_dump(
                {
                    "vehicles": [
                        {
                            "name": "uav_0",
                            "mission": "hover",
                        }
                    ]
                },
                sort_keys=False,
            ),
            encoding="utf-8",
        )

    bundle_path = tmp_path / "fleet-bundle.tar.gz"
    with tarfile.open(bundle_path, "w:gz") as archive:
        archive.add(bundle_root / "install", arcname="install")
    return bundle_path


def _write_mirrored_fleet_bundle(
    tmp_path: Path,
    *,
    fleet_name: str = "shared.yaml",
) -> Path:
    bundle_root = tmp_path / "mirrored-fleet-bundle"
    install_fleets_root = bundle_root / "install" / "uav" / "share" / "uav" / "fleets"
    source_fleets_root = bundle_root / "src" / "uav" / "uav" / "fleets"
    install_fleets_root.mkdir(parents=True, exist_ok=True)
    source_fleets_root.mkdir(parents=True, exist_ok=True)

    (install_fleets_root / fleet_name).write_text(
        yaml.safe_dump(
            {
                "vehicles": [
                    {
                        "name": "uav_0",
                        "mission": "hover",
                    }
                ]
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    (source_fleets_root / fleet_name).write_text(
        "vehicles: invalid\n",
        encoding="utf-8",
    )

    bundle_path = tmp_path / "mirrored-fleet-bundle.tar.gz"
    with tarfile.open(bundle_path, "w:gz") as archive:
        archive.add(bundle_root / "install", arcname="install")
        archive.add(bundle_root / "src", arcname="src")
    return bundle_path


def _write_actions_artifact_zip(
    tmp_path: Path,
    *,
    fleet_names: tuple[str, ...],
    tarball_name: str = "ros2-build-cafebabe.tar.gz",
) -> Path:
    bundle_root = tmp_path / "actions-artifact"
    fleets_root = bundle_root / "install" / "uav" / "share" / "uav" / "fleets"
    fleets_root.mkdir(parents=True, exist_ok=True)
    for fleet_name in fleet_names:
        (fleets_root / fleet_name).write_text(
            yaml.safe_dump(
                {
                    "vehicles": [
                        {
                            "name": "uav_0",
                            "mission": "hover",
                        }
                    ]
                },
                sort_keys=False,
            ),
            encoding="utf-8",
        )

    inner_tarball = io.BytesIO()
    with tarfile.open(fileobj=inner_tarball, mode="w:gz") as archive:
        archive.add(bundle_root / "install", arcname="install")
    inner_tarball.seek(0)

    zip_path = tmp_path / "actions-artifact.zip"
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr(tarball_name, inner_tarball.getvalue())
    return zip_path


def _write_deploy_lib(tmp_path: Path) -> Path:
    path = tmp_path / "scripts" / "hardware" / "deploy-lib.sh"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        (
            "#!/usr/bin/env bash\n"
            "deploy_ensure_uav_python_runtime() { :; }\n"
            "deploy_ensure_source_build_prereqs() { :; }\n"
        ),
        encoding="utf-8",
    )
    return path


def test_render_runtime_fleet_for_uav(tmp_path, monkeypatch):
    missions_root = tmp_path / "src" / "uav" / "uav" / "missions"
    missions_root.mkdir(parents=True, exist_ok=True)
    _write_mission(missions_root, "hover", "uav.vtol.TakeoffMode")

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
                "camera_calibration_file": "",
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
            "camera_calibration_file: warehouse_cam.yaml\n"
            "camera_preprocess_hook: rotate_then_crop\n"
            "px4_airframe_id: 4004\n"
        ),
    )
    ctx = _make_context(tmp_path, target, fleet_file=fleet_file)

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
            "camera_calibration_file": "warehouse_cam.yaml",
            "camera_preprocess_hook": "rotate_then_crop",
            "px4_airframe_id": 4004,
            "px4_namespace": "uav_0",
        }
    ]


def test_render_runtime_fleet_for_payload_defaults(tmp_path, monkeypatch):
    missions_root = tmp_path / "src" / "uav" / "uav" / "missions"
    missions_root.mkdir(parents=True, exist_ok=True)
    _write_mission(missions_root, "payload_retreat", "payload.PayloadRetreatMode")

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
                "camera_calibration_file": "",
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
        vehicle_name="payload_0",
        overlay_yaml="auto_launch: false\npayload_controller: SimController\n",
        pi_host="pi-2.local",
    )
    ctx = _make_context(tmp_path, target, fleet_file=fleet_file)

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
    assert "camera_calibration_file" not in runtime["vehicles"][0]


def test_validate_overlay_preview_rejects_unsupported_fields(tmp_path, monkeypatch):
    missions_root = tmp_path / "src" / "uav" / "uav" / "missions"
    missions_root.mkdir(parents=True, exist_ok=True)
    _write_mission(missions_root, "hover", "uav.vtol.TakeoffMode")
    fleet_file = _write_fleet(
        tmp_path,
        {"vehicles": [{"name": "uav_0", "mission": "hover"}]},
    )
    target = _make_target(
        target_id="pi-validate",
        vehicle_name="uav_0",
        overlay_yaml="mission: hover\npx4_airframe_id: 4004\n",
    )
    ctx = _make_context(tmp_path, target, fleet_file=fleet_file)

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
    fleet_file = "fleet.yaml"
    target = _make_target(
        target_id="pi-meta",
        vehicle_name="uav_0",
        overlay_yaml="mission: hover\npx4_airframe_id: 4004\n",
    )
    target_ctx = SimpleNamespace(target=target)

    metadata = deploy_service._release_metadata_payload(
        target_ctx,
        release_id="20260411-build-hover",
        source_type="source-build",
        source_label="local-uav.tar.gz",
        fleet_file=fleet_file,
        package_names=["uav", "uav_interfaces"],
    )

    assert metadata["release_id"] == "20260411-build-hover"
    assert metadata["source_type"] == "source-build"
    assert metadata["fleet_file"] == fleet_file
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
        "payload.PayloadRetreatMode",
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
        vehicle_name="payload_0",
        overlay_yaml="mission: payload_retreat\npayload_controller: GPIOController\n",
    )
    ctx = _make_context(tmp_path, target, fleet_file=fleet_file)

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
    )

    fake_httpx = SimpleNamespace(
        AsyncClient=lambda timeout=20: _FakeAsyncClient(
            releases_payload=[
                {
                    "tag_name": "build-deadbeef",
                    "name": "ROS 2 Build deadbeef",
                    "published_at": "2026-04-11T00:05:00Z",
                    "body": (
                        "Commit: deadbeef\nBuilt: 2026-04-11T00:00:00Z\nBranch: main\n"
                    ),
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
                "total_count": 1,
                "artifacts": [
                    {
                        "id": 99,
                        "name": "ros2-build-cafebabe.tar.gz",
                        "updated_at": "2026-04-11T01:00:00Z",
                        "size_in_bytes": 8_000_000,
                        "workflow_run": {
                            "id": 42,
                            "head_sha": "cafebabedeadbeef",
                            "head_branch": "feature/release-metadata",
                            "event": "push",
                            "name": "ARM Artifact",
                            "conclusion": "success",
                        },
                    },
                    {
                        "id": 100,
                        "name": "ros2-build-fallback.tar.gz",
                        "updated_at": "2026-04-11T02:00:00Z",
                        "size_in_bytes": 8_000_000,
                        "workflow_run": {
                            "id": 43,
                            "head_sha": "feedfacefeedface",
                            "head_branch": "feature/fallback",
                            "event": "workflow_dispatch",
                            "name": "ARM Fallback",
                            "conclusion": "success",
                        },
                    },
                    {
                        "id": 101,
                        "name": "dockerbuild-123.zip",
                        "updated_at": "2026-04-11T03:00:00Z",
                        "size_in_bytes": 1_000_000,
                        "workflow_run": {
                            "id": 44,
                            "head_sha": "badc0debadc0de",
                            "head_branch": "feature/ignored",
                            "event": "push",
                            "name": "Docker Build",
                            "conclusion": "success",
                        },
                    },
                ],
            },
            commit_payloads={
                "deadbeef": {"commit": {"message": "Release commit subject\n\nbody"}},
                "cafebabedeadbeef": {
                    "commit": {"message": "Artifact commit subject\n\nbody"}
                },
                "feedfacefeedface": {
                    "commit": {"message": "Fallback artifact subject\n\nbody"}
                },
            },
        )
    )
    monkeypatch.setattr(deploy_service, "_require_httpx", lambda: fake_httpx)

    result = asyncio.run(deploy_service.list_builds(ctx))

    assert result["success"] is True
    assert len(result["builds"]) == 2
    assert result["builds"][0]["source"] == "actions"
    assert result["builds"][0]["commit_sha"] == "cafebabedeadbeef"
    assert result["builds"][0]["commit_subject"] == "Artifact commit subject"
    assert result["builds"][0]["artifact_id"] == "99"
    assert result["builds"][0]["run_id"] == "42"
    assert result["builds"][0]["branch"] == "feature/release-metadata"
    assert result["builds"][0]["date"] == "2026-04-11T01:00:00Z"
    assert result["builds"][0]["workflow_event"] == "push"
    assert result["builds"][0]["fallback"] is False
    assert result["builds"][1] == {
        "source": "release",
        "tag": "build-deadbeef",
        "sha": "deadbee",
        "commit_sha": "deadbeef",
        "commit_subject": "Release commit subject",
        "name": "Release commit subject",
        "date": "2026-04-11T00:00:00Z",
        "download_url": "https://example.com/build.tar.gz",
        "size_mb": 1.2,
        "branch": "main",
        "workflow_name": "ARM Artifact Release",
        "workflow_event": "release",
        "workflow_conclusion": None,
        "deployable": True,
        "fallback": False,
    }
    assert len(result["releases"]) == 1
    assert len(result["artifacts"]) == 1
    assert result["artifact_page"] == 1
    assert result["artifact_page_size"] == 20
    assert result["artifact_has_more"] is False

    filtered = asyncio.run(
        deploy_service.list_builds(
            ctx,
            include_fallback=True,
            q="fallback",
        )
    )
    assert len(filtered["builds"]) == 1
    assert filtered["builds"][0]["fallback"] is True
    assert filtered["builds"][0]["workflow_event"] == "workflow_dispatch"


def test_list_builds_enriches_actions_artifacts_from_workflow_run_lookup(
    tmp_path, monkeypatch
):
    ctx = SimpleNamespace(
        base_dir=tmp_path / "src" / "integration",
        operator_config=_make_operator(tmp_path),
    )

    run_id = "42"
    artifact_id = "99"
    commit_sha = "cafebabedeadbeef"
    run_url = (
        f"https://api.github.com/repos/{ctx.operator_config.github_repo}"
        f"/actions/runs/{run_id}"
    )

    fake_httpx = SimpleNamespace(
        AsyncClient=lambda timeout=20: _FakeAsyncClient(
            releases_payload=[],
            artifacts_payload={
                "total_count": 1,
                "artifacts": [
                    {
                        "id": int(artifact_id),
                        "name": "ros2-build-cafebabe.tar.gz",
                        "updated_at": "2026-04-11T01:00:00Z",
                        "size_in_bytes": 8_000_000,
                        "workflow_run": {
                            "id": int(run_id),
                            "head_sha": commit_sha,
                            "head_branch": "feature/release-metadata",
                        },
                    }
                ],
            },
            commit_payloads={
                commit_sha: {"commit": {"message": "Artifact commit subject\n\nbody"}}
            },
            response_overrides={
                run_url: _FakeResponse(
                    {
                        "id": int(run_id),
                        "name": "ARM Artifact",
                        "event": "push",
                        "conclusion": "success",
                        "head_sha": commit_sha,
                        "head_branch": "feature/release-metadata",
                    }
                )
            },
        )
    )
    monkeypatch.setattr(deploy_service, "_require_httpx", lambda: fake_httpx)

    result = asyncio.run(deploy_service.list_builds(ctx))

    assert result["success"] is True
    assert result["builds"] == [
        {
            "source": "actions",
            "tag": f"run-{run_id}",
            "sha": commit_sha[:7],
            "commit_sha": commit_sha,
            "commit_subject": "Artifact commit subject",
            "name": "Artifact commit subject",
            "date": "2026-04-11T01:00:00Z",
            "download_url": (
                f"https://api.github.com/repos/{ctx.operator_config.github_repo}"
                f"/actions/artifacts/{artifact_id}/zip"
            ),
            "size_mb": 8.0,
            "run_id": run_id,
            "artifact_id": artifact_id,
            "artifact_name": "ros2-build-cafebabe.tar.gz",
            "branch": "feature/release-metadata",
            "workflow_name": "ARM Artifact",
            "workflow_event": "push",
            "workflow_conclusion": "success",
            "deployable": True,
            "fallback": False,
        }
    ]


def test_list_builds_uses_cached_response_for_repeat_requests(tmp_path, monkeypatch):
    ctx = SimpleNamespace(
        base_dir=tmp_path / "src" / "integration",
        operator_config=_make_operator(tmp_path),
    )

    client_creations = 0

    def _client_factory(timeout=20):
        nonlocal client_creations
        client_creations += 1
        return _FakeAsyncClient(
            releases_payload=[
                {
                    "tag_name": "build-deadbeef",
                    "name": "ROS 2 Build deadbeef",
                    "published_at": "2026-04-11T00:05:00Z",
                    "body": "Commit: deadbeef\nBuilt: 2026-04-11T00:00:00Z\nBranch: main\n",
                    "assets": [
                        {
                            "browser_download_url": "https://example.com/build.tar.gz",
                            "size": 1234567,
                        }
                    ],
                }
            ],
            artifacts_payload={"total_count": 0, "artifacts": []},
            commit_payloads={
                "deadbeef": {"commit": {"message": "Release commit subject\n\nbody"}}
            },
        )

    fake_httpx = SimpleNamespace(AsyncClient=_client_factory)
    monkeypatch.setattr(deploy_service, "_require_httpx", lambda: fake_httpx)

    first = asyncio.run(deploy_service.list_builds(ctx))
    second = asyncio.run(deploy_service.list_builds(ctx))

    assert first["success"] is True
    assert second == first
    assert client_creations == 1


def test_list_builds_returns_stale_cache_when_github_rate_limited(
    tmp_path, monkeypatch
):
    ctx = SimpleNamespace(
        base_dir=tmp_path / "src" / "integration",
        operator_config=_make_operator(tmp_path),
    )
    ctx.operator_config.github_token = ""

    base_httpx = SimpleNamespace(
        AsyncClient=lambda timeout=20: _FakeAsyncClient(
            releases_payload=[
                {
                    "tag_name": "build-deadbeef",
                    "name": "ROS 2 Build deadbeef",
                    "published_at": "2026-04-11T00:05:00Z",
                    "body": "Commit: deadbeef\nBuilt: 2026-04-11T00:00:00Z\nBranch: main\n",
                    "assets": [
                        {
                            "browser_download_url": "https://example.com/build.tar.gz",
                            "size": 1234567,
                        }
                    ],
                }
            ],
            artifacts_payload={"total_count": 0, "artifacts": []},
            commit_payloads={
                "deadbeef": {"commit": {"message": "Release commit subject\n\nbody"}}
            },
        )
    )
    monkeypatch.setattr(deploy_service, "_require_httpx", lambda: base_httpx)

    first = asyncio.run(deploy_service.list_builds(ctx))
    assert first["success"] is True

    cache_key = deploy_service._build_list_cache_key(
        ctx,
        artifact_page=1,
        artifact_page_size=20,
        q="",
        sha="",
        commit_subject="",
        branch="",
        include_fallback=False,
    )
    cached_at, payload = deploy_service._BUILD_LIST_CACHE[cache_key]
    deploy_service._BUILD_LIST_CACHE[cache_key] = (
        cached_at - deploy_service._BUILD_LIST_CACHE_TTL_SECONDS - 1,
        payload,
    )

    rate_limit_url = (
        f"https://api.github.com/repos/{ctx.operator_config.github_repo}"
        "/releases?per_page=100&page=1"
    )
    rate_limited_httpx = SimpleNamespace(
        AsyncClient=lambda timeout=20: _FakeAsyncClient(
            releases_payload=[],
            artifacts_payload={"total_count": 0, "artifacts": []},
            response_overrides={
                rate_limit_url: _FakeResponse(
                    {"message": "API rate limit exceeded"},
                    status_code=403,
                    headers={"x-ratelimit-reset": "1775976000"},
                )
            },
        )
    )
    monkeypatch.setattr(deploy_service, "_require_httpx", lambda: rate_limited_httpx)

    second = asyncio.run(deploy_service.list_builds(ctx))

    assert second["success"] is True
    assert second["builds"] == first["builds"]
    assert "cached build list" in (second["error"] or "").lower()
    assert "gITHUB_TOKEN".lower() in (second["error"] or "").lower()


def test_source_catalog_key_is_stable_for_exact_build_sources(tmp_path):
    target = _make_target(
        target_id="pi-source-key",
        vehicle_name="uav_0",
        overlay_yaml="mission: hover\npx4_airframe_id: 4004\n",
    )
    ctx = _make_context(tmp_path, target)

    asyncio.run(
        deploy_service.set_github_build_source(
            ctx,
            source="actions",
            artifact_id="6389816289",
            run_id="24297396361",
            sha="591aeed",
            branch="user/ethayu/manual-merge",
            name="ros2-build-2657150",
            download_url="https://example.com/build.zip",
        )
    )
    current = ctx.build_source_store.current()
    baseline = deploy_service._source_catalog_key(current)

    ctx.build_source_store.set_fleet_file(fleet_file="/tmp/runtime-fleet-a.yaml")
    updated = deploy_service._source_catalog_key(ctx.build_source_store.current())
    assert updated == baseline

    ctx.build_source_store.current().updated_at = "2099-01-01T00:00:00Z"
    ctx.build_source_store.save()
    mutated = deploy_service._source_catalog_key(ctx.build_source_store.current())
    assert mutated == baseline


def test_get_fleet_catalog_reuses_stale_exact_build_cache_after_rate_limit(
    tmp_path, monkeypatch
):
    target = _make_target(
        target_id="pi-stale-cache",
        vehicle_name="uav_0",
        overlay_yaml="mission: hover\npx4_airframe_id: 4004\n",
    )
    ctx = _make_context(tmp_path, target)

    zip_path = _write_actions_artifact_zip(
        tmp_path,
        fleet_names=("alpha.yaml", "beta.yaml"),
        tarball_name="ros2-build-cafebabe.tar.gz",
    )
    archive_bytes = zip_path.read_bytes()
    archive_url = (
        f"https://api.github.com/repos/{ctx.operator_config.github_repo}"
        "/actions/artifacts/6389816289/zip"
    )
    fake_httpx = SimpleNamespace(
        Client=lambda timeout=300, follow_redirects=True: _FakeHTTPXClient(
            body=archive_bytes
        )
    )
    monkeypatch.setattr(deploy_service, "_require_httpx", lambda: fake_httpx)

    asyncio.run(
        deploy_service.set_github_build_source(
            ctx,
            source="actions",
            artifact_id="6389816289",
            run_id="24297396361",
            sha="591aeed",
            branch="user/ethayu/manual-merge",
            name="ros2-build-2657150",
        )
    )

    first = asyncio.run(deploy_service.get_fleet_catalog(ctx))
    assert first["success"] is True
    assert sorted(first["available_fleets"]) == ["alpha.yaml", "beta.yaml"]
    first_catalog_dir = deploy_service._fleet_catalog_dir(ctx)
    assert first_catalog_dir.exists()

    # Simulate the operator re-selecting the same exact build later, after the
    # selected-fleet stamp has changed and GitHub is rate-limiting.
    ctx.build_source_store.current().updated_at = "2099-01-01T00:00:00Z"
    ctx.build_source_store.current().fleet_file = "alpha.yaml"
    ctx.build_source_store.save()

    class _RateLimitedStream:
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            return False

        def raise_for_status(self) -> None:
            raise _FakeHTTPStatusError(
                _FakeResponse(
                    {"message": "API rate limit exceeded"},
                    status_code=403,
                    headers={"x-ratelimit-reset": "1775976000"},
                )
            )

        def iter_bytes(self, chunk_size: int = 65536):
            yield b""

    class _RateLimitedHTTPXClient:
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            return False

        def stream(self, method: str, url: str, headers=None):
            assert method == "GET"
            assert url == archive_url
            return _RateLimitedStream()

    monkeypatch.setattr(
        deploy_service,
        "_require_httpx",
        lambda: SimpleNamespace(
            Client=lambda timeout=300, follow_redirects=True: _RateLimitedHTTPXClient()
        ),
    )

    second = asyncio.run(deploy_service.get_fleet_catalog(ctx))

    assert second["success"] is True
    assert sorted(second["available_fleets"]) == ["alpha.yaml", "beta.yaml"]
    assert second["fleet_catalog_error"] is None


def test_current_build_parses_release_marker(tmp_path):
    target = _make_target(
        target_id="pi-3",
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
    ctx = _make_context(tmp_path, target, fleet_file=tmp_path / "fleet.yaml")
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
    ctx = _make_context(tmp_path, target, fleet_file=tmp_path / "fleet.yaml")
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
        vehicle_name="uav_0",
        overlay_yaml="mission: hover\npx4_airframe_id: 4004\n",
    )
    ctx = _make_context(tmp_path, target, fleet_file=tmp_path / "fleet.yaml")

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
        vehicle_name="uav_0",
        overlay_yaml="mission: hover\npx4_airframe_id: 4004\n",
    )
    ctx = _make_context(tmp_path, target, fleet_file=tmp_path / "fleet.yaml")

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
    kind, kwargs = calls[-1]
    assert kind == "github"
    assert kwargs["target_ctx"].target.target_id == "pi-dispatch"
    assert kwargs["tag"] == ""
    assert kwargs["source"] == "actions"
    assert kwargs["artifact_id"] == "42"

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
    assert calls[-1][1]["target_ctx"].target.target_id == "pi-dispatch"
    assert calls[-1][1]["artifact_name"] == "artifact.tar.gz"

    asyncio.run(deploy_service.set_local_codebase_build_source(ctx))
    result = asyncio.run(
        deploy_service.deploy_selected_source(ctx, target_id="pi-dispatch")
    )
    assert result["success"] is True
    assert calls[-1][0] == "local_codebase"
    assert calls[-1][1]["target_ctx"].target.target_id == "pi-dispatch"


def test_deploy_selected_source_requires_global_fleet_selection(tmp_path):
    target = _make_target(
        target_id="pi-no-fleet",
        vehicle_name="uav_0",
        overlay_yaml="mission: hover\npx4_airframe_id: 4004\n",
    )
    ctx = _make_context(tmp_path, target)

    asyncio.run(deploy_service.set_local_codebase_build_source(ctx))
    result = asyncio.run(
        deploy_service.deploy_selected_source(ctx, target_id="pi-no-fleet")
    )

    assert result["success"] is False
    assert "fleet file" in result["error"].lower()


def test_fleet_catalog_exposes_local_artifact_fleets_and_validates_selection(
    tmp_path,
):
    target = _make_target(
        target_id="pi-catalog",
        vehicle_name="uav_0",
        overlay_yaml="mission: hover\npx4_airframe_id: 4004\n",
    )
    ctx = _make_context(tmp_path, target)

    bundle_path = _write_fleet_bundle(
        tmp_path,
        fleet_names=("primary.yaml", "backup.yaml"),
    )

    result = asyncio.run(
        deploy_service.set_local_artifact_build_source(
            ctx,
            filename="fleet-bundle.tar.gz",
            file_bytes=bundle_path.read_bytes(),
        )
    )
    assert result["success"] is True

    catalog = asyncio.run(deploy_service.get_fleet_catalog(ctx))
    assert catalog["success"] is True
    assert len(catalog["available_fleets"]) == 2

    selected_fleet = catalog["available_fleets"][0]
    result = asyncio.run(
        deploy_service.set_global_fleet_file(ctx, fleet_file=selected_fleet)
    )
    assert result["success"] is True
    assert selected_fleet in result["source"]["available_fleets"]

    rogue_fleet = tmp_path / "rogue.yaml"
    rogue_fleet.write_text(
        "vehicles:\n  - name: uav_9\n    mission: hover\n",
        encoding="utf-8",
    )
    result = asyncio.run(
        deploy_service.set_global_fleet_file(ctx, fleet_file=str(rogue_fleet))
    )
    assert result["success"] is False
    assert "selected build source" in result["error"].lower()


def test_local_artifact_catalog_dedupes_mirrored_fleet_filenames(tmp_path):
    target = _make_target(
        target_id="pi-mirrored-fleet",
        vehicle_name="uav_0",
        overlay_yaml="mission: hover\npx4_airframe_id: 4004\n",
    )
    ctx = _make_context(tmp_path, target)

    bundle_path = _write_mirrored_fleet_bundle(tmp_path, fleet_name="shared.yaml")

    result = asyncio.run(
        deploy_service.set_local_artifact_build_source(
            ctx,
            filename="mirrored-fleet-bundle.tar.gz",
            file_bytes=bundle_path.read_bytes(),
        )
    )
    assert result["success"] is True
    assert result["source"]["fleet_catalog_error"] is None
    assert result["source"]["available_fleets"] == ["shared.yaml"]

    result = asyncio.run(
        deploy_service.set_global_fleet_file(ctx, fleet_file="shared.yaml")
    )
    assert result["success"] is True
    assert result["source"]["fleet_file"] == "shared.yaml"


def test_ensure_runtime_prereqs_uses_shared_deploy_lib(tmp_path):
    target = _make_target(
        target_id="pi-runtime-prereqs",
        vehicle_name="uav_0",
        overlay_yaml="mission: hover\npx4_airframe_id: 4004\n",
    )
    ctx = _make_context(tmp_path, target)
    helper_path = _write_deploy_lib(tmp_path)

    target_ctx = ctx.resolve_target("pi-runtime-prereqs")
    asyncio.run(deploy_service._ensure_runtime_prereqs(ctx, target_ctx))

    paths = target_ctx.target.deploy_paths()
    assert target_ctx.ssh.scp_calls == [
        (str(helper_path), f"{paths['incoming_dir']}/deploy-lib.sh")
    ]
    assert any(
        "deploy_ensure_uav_python_runtime run_root" in cmd
        for cmd in target_ctx.ssh.run_calls
    )
    assert any(target_ctx.target.pi_user in cmd for cmd in target_ctx.ssh.run_calls)


def test_ensure_source_build_prereqs_uses_shared_deploy_lib(tmp_path):
    target = _make_target(
        target_id="pi-source-prereqs",
        vehicle_name="uav_0",
        overlay_yaml="mission: hover\npx4_airframe_id: 4004\n",
    )
    ctx = _make_context(tmp_path, target)
    helper_path = _write_deploy_lib(tmp_path)

    target_ctx = ctx.resolve_target("pi-source-prereqs")
    asyncio.run(deploy_service._ensure_source_build_prereqs(ctx, target_ctx))

    paths = target_ctx.target.deploy_paths()
    assert target_ctx.ssh.scp_calls == [
        (str(helper_path), f"{paths['incoming_dir']}/deploy-lib.sh")
    ]
    assert any(
        "deploy_ensure_source_build_prereqs run_root" in cmd
        for cmd in target_ctx.ssh.run_calls
    )
    assert any(target_ctx.target.pi_user in cmd for cmd in target_ctx.ssh.run_calls)


def test_runner_script_disables_nounset_only_around_ros_setup(tmp_path):
    target = _make_target(
        target_id="pi-runner-script",
        vehicle_name="uav_0",
        overlay_yaml="mission: hover\npx4_airframe_id: 4004\n",
    )
    target_ctx = SimpleNamespace(
        target=target,
        ssh=_FakeSSH(result=_FakeSSHResult()),
    )

    script = deploy_service._runner_script(target_ctx)

    assert "set -euo pipefail" in script
    assert "set +u" in script
    assert "source /opt/ros/humble/setup.bash" in script
    assert 'source "$DEPLOY_ROOT/current/install/setup.bash"' in script
    assert "set -u\nexec ros2 launch" in script
    assert script.index("set +u") < script.index("source /opt/ros/humble/setup.bash")
    assert script.index(
        'source "$DEPLOY_ROOT/current/install/setup.bash"'
    ) < script.index("set -u\nexec ros2 launch")


def test_activate_release_polls_stability_and_rolls_back_on_instability(tmp_path):
    target = _make_target(
        target_id="pi-activate-stability",
        vehicle_name="uav_0",
        overlay_yaml="mission: hover\npx4_airframe_id: 4004\n",
    )

    class _ActivationSSH:
        def __init__(self):
            self.run_calls: list[str] = []

        def q(self, value: str) -> str:
            return value

        async def run(self, command: str, timeout: int = 15):
            self.run_calls.append(command)
            if "deadline=$((SECONDS + 8))" in command:
                return _FakeSSHResult(
                    returncode=1,
                    stdout="",
                    stderr="__ERR__:unstable:active:running:1234:1",
                )
            if "ln -sfn" in command and "previous_link" in command:
                return _FakeSSHResult(returncode=0, stdout="", stderr="")
            return _FakeSSHResult(returncode=0, stdout="ACTIVE", stderr="")

        def friendly_error(self, message: str) -> str:
            return message

        def format_remote_error(self, raw_error: str, prefix: str) -> str:
            return f"{prefix}: {raw_error}"

    target_ctx = SimpleNamespace(
        target=target,
        ssh=_ActivationSSH(),
    )

    with pytest.raises(RuntimeError, match="Reverted to the previous release"):
        asyncio.run(
            deploy_service._activate_release(
                target_ctx,
                release_dir="/home/penn/pennair-deploy/releases/release-new",
                previous_target="/home/penn/pennair-deploy/releases/release-old",
            )
        )

    restart_cmd = target_ctx.ssh.run_calls[0]
    assert "systemctl show" in restart_cmd
    assert "ActiveState" in restart_cmd
    assert "SubState" in restart_cmd
    assert "MainPID" in restart_cmd
    assert "NRestarts" in restart_cmd
    assert "sleep 1" in restart_cmd
    assert "deadline=$((SECONDS + 8))" in restart_cmd
    assert any("ln -sfn" in cmd for cmd in target_ctx.ssh.run_calls[1:])


def test_perform_action_refreshes_failed_deploy_rows_with_rollback_metadata(
    tmp_path, monkeypatch
):
    from backend.models import FleetDeviceSelection
    from backend.services import fleet as fleet_service

    target = _make_target(
        target_id="pi-row-refresh",
        vehicle_name="uav_0",
        overlay_yaml="mission: hover\npx4_airframe_id: 4004\n",
        pi_host="pi-row-refresh.local",
    )
    ctx = _make_context(tmp_path, target)
    selection = FleetDeviceSelection.model_validate(
        {"hostname": "pi-row-refresh.local", "vehicle_name": "uav_0"}
    )

    async def fake_deploy_selected_source(inner_ctx, target_ctx=None, target_id=None):
        return {
            "success": False,
            "error": "Restart runtime service failed: __ERR__:unstable",
            "attempted_release_id": "release-new",
            "rolled_back": True,
        }

    async def fake_summarize_device(inner_ctx, device, fleet_vehicle_lookup):
        return {
            "hardware_id": "pi-row-refresh.local",
            "hostname": "pi-row-refresh.local",
            "addresses": [],
            "service_name": None,
            "service_type": None,
            "last_seen_at": "2026-04-13T00:00:00Z",
            "discovery_stale": False,
            "target_id": "pi-row-refresh.local",
            "vehicle_name": "uav_0",
            "fleet_vehicle": None,
            "connection": {
                "success": True,
                "connected": True,
                "target": "pi-row-refresh.local",
                "info": "ok",
                "error": None,
            },
            "current_build": {
                "success": True,
                "installed": True,
                "info": "ok",
                "release_id": "release-old",
            },
            "runtime": {
                "success": True,
                "running": False,
                "state": "stopped",
                "pid": None,
                "error": None,
            },
            "readiness": {
                "connected": True,
                "build_installed": True,
                "runtime_ready": True,
                "vehicle_assigned": True,
                "ready": True,
                "notes": [],
            },
        }

    monkeypatch.setattr(
        fleet_service.deploy_service,
        "deploy_selected_source",
        fake_deploy_selected_source,
    )
    monkeypatch.setattr(fleet_service, "_summarize_device", fake_summarize_device)

    result = asyncio.run(
        fleet_service._perform_action(
            ctx,
            action="deploy",
            selection=selection,
            snapshot={"vehicle_lookup": {}},
        )
    )

    assert result["success"] is False
    assert result["attempted_release_id"] == "release-new"
    assert result["active_release_id"] == "release-old"
    assert result["rolled_back"] is True
    assert result["error"] == "Restart runtime service failed: __ERR__:unstable"
    assert result["current_build"]["release_id"] == "release-old"


def test_batch_action_marks_top_level_failure_when_any_row_fails(tmp_path, monkeypatch):
    from backend.models import FleetDeviceSelection
    from backend.services import fleet as fleet_service

    async def fake_build_snapshot(inner_ctx):
        return {
            "build_source": {"kind": "none"},
            "fleet_catalog": {"success": True, "fleet_vehicles": []},
            "fleet_vehicles": [],
            "vehicle_lookup": {},
        }

    async def fake_resolve_action_devices(inner_ctx, devices):
        return [
            FleetDeviceSelection.model_validate(
                {"hostname": "pi-1.local", "vehicle_name": "uav_0"}
            ),
            FleetDeviceSelection.model_validate(
                {"hostname": "pi-2.local", "vehicle_name": "payload_0"}
            ),
        ]

    async def fake_perform_action(inner_ctx, *, action, selection, snapshot):
        if selection.hostname == "pi-1.local":
            return {
                "hardware_id": "pi-1.local",
                "hostname": "pi-1.local",
                "addresses": [],
                "service_name": None,
                "service_type": None,
                "last_seen_at": "2026-04-13T00:00:00Z",
                "discovery_stale": False,
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
                "success": True,
                "attempted_release_id": "release-old",
                "active_release_id": "release-old",
                "rolled_back": False,
                "output": "done",
                "error": None,
            }
        return {
            "hardware_id": "pi-2.local",
            "hostname": "pi-2.local",
            "addresses": [],
            "service_name": None,
            "service_type": None,
            "last_seen_at": "2026-04-13T00:00:00Z",
            "discovery_stale": False,
            "target_id": "pi-2.local",
            "vehicle_name": "payload_0",
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

    monkeypatch.setattr(fleet_service, "_build_snapshot", fake_build_snapshot)
    monkeypatch.setattr(
        fleet_service, "_resolve_action_devices", fake_resolve_action_devices
    )
    monkeypatch.setattr(fleet_service, "_perform_action", fake_perform_action)

    result = asyncio.run(
        fleet_service.batch_action(
            SimpleNamespace(),
            action="deploy",
        )
    )

    assert result["success"] is False
    assert result["summary"].requested_devices == 2
    assert result["summary"].successful_devices == 1
    assert result["summary"].failed_devices == 1
    assert result["results"][1].rolled_back is True


def test_build_source_change_preserves_matching_fleet_filename(tmp_path):
    target = _make_target(
        target_id="pi-preserve-fleet",
        vehicle_name="uav_0",
        overlay_yaml="mission: hover\npx4_airframe_id: 4004\n",
    )
    ctx = _make_context(tmp_path, target)
    shared_fleet = _write_fleet(
        tmp_path,
        {"vehicles": [{"name": "uav_0", "mission": "hover"}]},
        name="shared.yaml",
    )

    result = asyncio.run(deploy_service.set_local_codebase_build_source(ctx))
    assert result["success"] is True

    result = asyncio.run(
        deploy_service.set_global_fleet_file(ctx, fleet_file=str(shared_fleet))
    )
    assert result["success"] is True
    assert result["source"]["fleet_file"] == "shared.yaml"

    bundle_path = _write_fleet_bundle(
        tmp_path,
        fleet_names=("shared.yaml", "backup.yaml"),
    )
    result = asyncio.run(
        deploy_service.set_local_artifact_build_source(
            ctx,
            filename="fleet-bundle.tar.gz",
            file_bytes=bundle_path.read_bytes(),
        )
    )

    assert result["success"] is True
    assert result["source"]["kind"] == "local_artifact"
    assert result["source"]["fleet_file"] == "shared.yaml"
    assert result["source"]["available_fleets"] == ["backup.yaml", "shared.yaml"]


def test_build_source_change_clears_missing_fleet_filename(tmp_path):
    target = _make_target(
        target_id="pi-clear-fleet",
        vehicle_name="uav_0",
        overlay_yaml="mission: hover\npx4_airframe_id: 4004\n",
    )
    ctx = _make_context(tmp_path, target)
    primary_fleet = _write_fleet(
        tmp_path,
        {"vehicles": [{"name": "uav_0", "mission": "hover"}]},
        name="primary.yaml",
    )

    result = asyncio.run(deploy_service.set_local_codebase_build_source(ctx))
    assert result["success"] is True

    result = asyncio.run(
        deploy_service.set_global_fleet_file(ctx, fleet_file=str(primary_fleet))
    )
    assert result["success"] is True
    assert result["source"]["fleet_file"] == "primary.yaml"

    bundle_path = _write_fleet_bundle(
        tmp_path,
        fleet_names=("backup.yaml",),
    )
    result = asyncio.run(
        deploy_service.set_local_artifact_build_source(
            ctx,
            filename="fleet-bundle.tar.gz",
            file_bytes=bundle_path.read_bytes(),
        )
    )

    assert result["success"] is True
    assert result["source"]["kind"] == "local_artifact"
    assert result["source"]["fleet_file"] is None
    assert result["source"]["available_fleets"] == ["backup.yaml"]


def test_fleet_catalog_discovers_nested_actions_artifact_tarball(tmp_path, monkeypatch):
    target = _make_target(
        target_id="pi-actions-catalog",
        vehicle_name="uav_0",
        overlay_yaml="mission: hover\npx4_airframe_id: 4004\n",
    )
    ctx = _make_context(tmp_path, target)

    zip_path = _write_actions_artifact_zip(
        tmp_path,
        fleet_names=("alpha.yaml", "beta.yaml"),
        tarball_name="ros2-build-cafebabe.tar.gz",
    )
    archive_bytes = zip_path.read_bytes()
    fake_httpx = SimpleNamespace(
        Client=lambda timeout=300, follow_redirects=True: _FakeHTTPXClient(
            body=archive_bytes
        )
    )
    monkeypatch.setattr(deploy_service, "_require_httpx", lambda: fake_httpx)

    asyncio.run(
        deploy_service.set_github_build_source(
            ctx,
            source="actions",
            artifact_id="6389816289",
            run_id="24297396361",
            sha="591aeed",
            branch="user/ethayu/manual-merge",
            name="ros2-build-2657150",
        )
    )

    catalog = asyncio.run(deploy_service.get_fleet_catalog(ctx))

    assert catalog["success"] is True
    assert catalog["fleet_catalog_error"] is None
    assert catalog["fleet_catalog_source"].endswith("/actions/artifacts/6389816289/zip")
    assert sorted(catalog["available_fleets"]) == [
        "alpha.yaml",
        "beta.yaml",
    ]
