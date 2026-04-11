from __future__ import annotations

import json
import sys
import types
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

backend_pkg = types.ModuleType("backend")
backend_pkg.__path__ = [str(PACKAGE_ROOT / "backend")]
sys.modules.setdefault("backend", backend_pkg)

from backend.config import (  # noqa: E402
    BuildSourceStore,
    InventoryStore,
    OperatorConfig,
    TargetRecord,
    build_source_store_paths,
)


def _make_operator(tmp_path: Path) -> OperatorConfig:
    return OperatorConfig(
        github_repo="pennaerial/monorepo",
        github_token="secret-token",
        hotspot_name="pennair-hotspot",
        inventory_path=tmp_path / ".integration_inventory.json",
        default_deploy_root="/home/penn/pennair-deploy",
        default_pi_user="penn",
        default_ssh_key="~/.ssh/pennair_pi_ed25519",
        default_ssh_pass="default-secret",
    )


def _make_target(
    *,
    target_id: str = "pi-1",
    vehicle_name: str = "uav_0",
    overlay_yaml: str = "mission: hover\npx4_airframe_id: 4004\n",
) -> dict[str, object]:
    return {
        "target_id": target_id,
        "label": "Primary Pi",
        "pi_user": "penn",
        "pi_host": "pi-1.local",
        "deploy_root": "/home/penn/pennair-deploy",
        "ssh_key": "",
        "ssh_pass": "",
        "vehicle_name": vehicle_name,
        "overlay_yaml": overlay_yaml,
        "service_unit": "pennair-autonomy.service",
        "enabled": True,
    }


def test_operator_config_masks_tokens_and_updates_from_form(tmp_path):
    operator = _make_operator(tmp_path)

    safe = operator.to_safe_dict()
    assert safe["github_token"] == "••••"
    assert safe["inventory_path"] == str(tmp_path / ".integration_inventory.json")
    assert safe["default_ssh_pass"] == "••••"

    changed = operator.update_from_form(
        {
            "github_repo": "other/repo",
            "github_token": "new-secret",
            "hotspot_name": "pi-hotspot",
            "default_deploy_root": "/tmp/deploy-root",
            "default_pi_user": "ubuntu",
            "default_ssh_key": "/tmp/id_ed25519",
            "default_ssh_pass": "new-default-secret",
        }
    )

    assert changed == {
        "github_repo": "other/repo",
        "github_token": "(set)",
        "hotspot_name": "pi-hotspot",
        "default_deploy_root": "/tmp/deploy-root",
        "default_pi_user": "ubuntu",
        "default_ssh_key": "/tmp/id_ed25519",
        "default_ssh_pass": "(set)",
    }
    assert operator.github_repo == "other/repo"
    assert operator.github_token == "new-secret"
    assert operator.hotspot_name == "pi-hotspot"
    assert operator.default_deploy_root == "/tmp/deploy-root"
    assert operator.default_pi_user == "ubuntu"
    assert operator.default_ssh_key == "/tmp/id_ed25519"
    assert operator.default_ssh_pass == "new-default-secret"


def test_target_record_validates_identity_and_overlay():
    record = TargetRecord.from_dict(
        _make_target(),
        default_deploy_root="/home/penn/pennair-deploy",
    )

    assert record.ssh_target() == "penn@pi-1.local"
    assert (
        record.deploy_paths()["runtime_fleet"]
        == "/home/penn/pennair-deploy/config/runtime_fleet.yaml"
    )
    assert (
        record.workspace_paths()["overlay_file"]
        == "/home/penn/pennair-deploy/config/overlay.yaml"
    )
    assert (
        record.mission_paths()["runner_script"]
        == "/home/penn/pennair-deploy/config/start-current.sh"
    )
    assert record.overlay_data() == {"mission": "hover", "px4_airframe_id": 4004}

    with pytest.raises(ValueError, match="Invalid target_id"):
        TargetRecord.from_dict(
            {
                **_make_target(target_id="bad target"),
                "vehicle_name": "uav_0",
            },
            default_deploy_root="/home/penn/pennair-deploy",
        )

    unassigned = TargetRecord.from_dict(
        {
            **_make_target(),
            "vehicle_name": "",
        },
        default_deploy_root="/home/penn/pennair-deploy",
    )
    assert unassigned.vehicle_name == ""

    bad_overlay = TargetRecord.from_dict(
        {
            **_make_target(),
            "overlay_yaml": "- not-a-mapping",
        },
        default_deploy_root="/home/penn/pennair-deploy",
    )
    with pytest.raises(ValueError, match="overlay must be a YAML mapping"):
        bad_overlay.overlay_data()

    with pytest.raises(ValueError, match="invalid pi_host"):
        TargetRecord.from_dict(
            {**_make_target(), "pi_host": "bad host"},
            default_deploy_root="/home/penn/pennair-deploy",
        )

    with pytest.raises(ValueError, match="invalid service_unit"):
        TargetRecord.from_dict(
            {**_make_target(), "service_unit": "bad service"},
            default_deploy_root="/home/penn/pennair-deploy",
        )


def test_inventory_store_round_trips_through_disk(tmp_path):
    operator = _make_operator(tmp_path)
    seed_target = TargetRecord.from_dict(
        _make_target(),
        default_deploy_root=operator.default_deploy_root,
    )

    store = InventoryStore(
        operator.inventory_path,
        operator_config=operator,
        default_deploy_root=operator.default_deploy_root,
        seed_target=seed_target,
    )

    created, was_created = store.upsert_target(
        {
            "target_id": "backup",
            "label": "Backup Pi",
            "pi_user": "ubuntu",
            "pi_host": "backup.local",
            "deploy_root": "/opt/pennair",
            "ssh_key": "/tmp/id_ed25519",
            "ssh_pass": "secret",
            "vehicle_name": "payload_0",
            "overlay_yaml": "mission: payload_retreat\npayload_controller: SimController\n",
            "service_unit": "backup.service",
            "enabled": False,
        }
    )
    assert was_created is True
    assert created.target_id == "backup"
    store.set_active_target("backup")
    store.save()

    raw = json.loads(operator.inventory_path.read_text(encoding="utf-8"))
    assert raw["active_target_id"] == "backup"
    assert len(raw["targets"]) == 2
    assert raw["operator"]["github_repo"] == "pennaerial/monorepo"

    restored_operator = OperatorConfig(
        github_repo="",
        github_token="",
        hotspot_name="",
        inventory_path=operator.inventory_path,
        default_deploy_root="/tmp/unused",
        default_pi_user="",
        default_ssh_key="",
        default_ssh_pass="",
    )
    restored = InventoryStore(
        operator.inventory_path,
        operator_config=restored_operator,
        default_deploy_root="/tmp/unused",
        seed_target=seed_target,
    )

    assert restored.active_target_id() == "backup"
    assert restored_operator.github_repo == "pennaerial/monorepo"
    assert restored_operator.github_token == "secret-token"
    assert restored_operator.hotspot_name == "pennair-hotspot"
    assert restored_operator.default_deploy_root == "/home/penn/pennair-deploy"
    assert restored_operator.default_pi_user == "penn"
    assert restored_operator.default_ssh_key == "~/.ssh/pennair_pi_ed25519"
    assert restored_operator.default_ssh_pass == "default-secret"
    assert [target.target_id for target in restored.list_targets()] == [
        "backup",
        "pi-1",
    ]

    with pytest.raises(ValueError, match="Cannot delete the last target"):
        lone_store = InventoryStore(
            tmp_path / "single.json",
            operator_config=_make_operator(tmp_path),
            default_deploy_root="/home/penn/pennair-deploy",
            seed_target=seed_target,
        )
        lone_store.delete_target("pi-1")


def test_inventory_store_revalidates_existing_target_updates(tmp_path):
    operator = _make_operator(tmp_path)
    seed_target = TargetRecord.from_dict(
        _make_target(),
        default_deploy_root=operator.default_deploy_root,
    )
    store = InventoryStore(
        operator.inventory_path,
        operator_config=operator,
        default_deploy_root=operator.default_deploy_root,
        seed_target=seed_target,
    )

    with pytest.raises(ValueError, match="invalid pi_host"):
        store.upsert_target(
            {
                "target_id": "pi-1",
                "pi_host": "bad host",
            }
        )


def test_inventory_store_exports_and_imports(tmp_path):
    operator = _make_operator(tmp_path)
    seed_target = TargetRecord.from_dict(
        _make_target(),
        default_deploy_root=operator.default_deploy_root,
    )

    store = InventoryStore(
        operator.inventory_path,
        operator_config=operator,
        default_deploy_root=operator.default_deploy_root,
        seed_target=seed_target,
    )

    created, was_created = store.upsert_target(
        {
            "target_id": "defaults-only",
            "label": "Defaults Only",
            "pi_host": "defaults.local",
            "vehicle_name": "uav_1",
        }
    )
    assert was_created is True
    assert created.pi_user == "penn"
    assert created.ssh_key == "~/.ssh/pennair_pi_ed25519"
    assert created.ssh_pass == "default-secret"

    payload = store.export_payload()
    assert payload["active_target_id"] == "pi-1"
    assert len(payload["targets"]) == 2

    summary = store.import_payload(
        {
            "operator": {
                "github_repo": "other/repo",
                "hotspot_name": "alt-hotspot",
            },
            "active_target_id": "pi-2",
            "targets": [
                _make_target(
                    target_id="pi-2",
                    vehicle_name="payload_0",
                    overlay_yaml="mission: payload_retreat\npayload_controller: GPIOController\n",
                )
            ],
        }
    )

    assert summary == {
        "targets_imported": 1,
        "active_target_id": "pi-2",
        "target_ids": ["pi-2"],
        "replace_existing": False,
    }
    assert operator.github_repo == "other/repo"
    assert operator.hotspot_name == "alt-hotspot"
    assert [target.target_id for target in store.list_targets()] == [
        "defaults-only",
        "pi-1",
        "pi-2",
    ]

    replace_summary = store.import_payload(
        {
            "active_target_id": "only",
            "targets": [
                _make_target(
                    target_id="only",
                    vehicle_name="uav_9",
                    overlay_yaml="mission: hover\npx4_airframe_id: 4010\n",
                )
            ],
        },
        replace_existing=True,
    )

    assert replace_summary["active_target_id"] == "only"
    assert [target.target_id for target in store.list_targets()] == ["only"]


def test_build_source_store_persists_separately_from_inventory(tmp_path):
    operator = _make_operator(tmp_path)
    seed_target = TargetRecord.from_dict(
        _make_target(),
        default_deploy_root=operator.default_deploy_root,
    )
    inventory = InventoryStore(
        operator.inventory_path,
        operator_config=operator,
        default_deploy_root=operator.default_deploy_root,
        seed_target=seed_target,
    )
    source_path, cache_dir = build_source_store_paths(operator.inventory_path)
    store = BuildSourceStore(source_path, cache_dir=cache_dir)

    record = store.set_github(
        github_source="release",
        tag="build-deadbeef",
        sha="deadbee",
        name="build-deadbeef",
    )
    assert record.kind == "github"
    assert source_path.exists()

    inventory_payload = inventory.export_payload()
    encoded_inventory = json.dumps(inventory_payload)
    assert "artifact_name" not in encoded_inventory
    assert "local_artifact_path" not in encoded_inventory
    assert "codebase_root" not in encoded_inventory
    assert "github_source" not in encoded_inventory

    restored = BuildSourceStore(source_path, cache_dir=cache_dir)
    assert restored.current().kind == "github"
    assert restored.current().tag == "build-deadbeef"


def test_build_source_store_local_artifact_clears_cached_file(tmp_path):
    operator = _make_operator(tmp_path)
    source_path, cache_dir = build_source_store_paths(operator.inventory_path)
    store = BuildSourceStore(source_path, cache_dir=cache_dir)

    record = store.set_local_artifact(
        artifact_name="bundle.tar.gz",
        file_bytes=b"artifact-bytes",
    )
    artifact_path = Path(record.local_artifact_path)
    assert artifact_path.exists()

    store.clear()

    assert store.current().kind == "none"
    assert not artifact_path.exists()


def test_build_source_store_persists_fleet_selection_and_local_sources(tmp_path):
    operator = _make_operator(tmp_path)
    source_path, cache_dir = build_source_store_paths(operator.inventory_path)
    store = BuildSourceStore(source_path, cache_dir=cache_dir)

    store.set_fleet_file(fleet_file="/tmp/runtime-fleet.yaml")
    artifact_record = store.set_local_artifact(
        artifact_name="bundle.tar.gz",
        file_bytes=b"artifact-bytes",
    )
    artifact_path = Path(artifact_record.local_artifact_path)

    restored = BuildSourceStore(source_path, cache_dir=cache_dir)
    assert restored.current().kind == "local_artifact"
    assert restored.current().fleet_file == "/tmp/runtime-fleet.yaml"
    assert restored.current().artifact_name == "bundle.tar.gz"
    assert artifact_path.exists()

    restored.set_local_codebase(codebase_root="/workspace/monorepo")
    reloaded = BuildSourceStore(source_path, cache_dir=cache_dir)
    assert reloaded.current().kind == "local_codebase"
    assert reloaded.current().fleet_file == "/tmp/runtime-fleet.yaml"
    assert reloaded.current().codebase_root == "/workspace/monorepo"
