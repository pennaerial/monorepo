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
    InventoryStore,
    OperatorConfig,
    TargetRecord,
)


def _make_operator(tmp_path: Path) -> OperatorConfig:
    return OperatorConfig(
        github_repo="pennaerial/monorepo",
        github_token="secret-token",
        hotspot_name="pennair-hotspot",
        inventory_path=tmp_path / ".integration_inventory.json",
        default_deploy_root="/home/penn/pennair-deploy",
    )


def _make_target(
    *,
    target_id: str = "pi-1",
    vehicle_name: str = "uav_0",
    fleet_file: str = "/tmp/fleet.yaml",
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
        "fleet_file": fleet_file,
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

    changed = operator.update_from_form(
        {
            "github_repo": "other/repo",
            "github_token": "new-secret",
            "hotspot_name": "pi-hotspot",
            "default_deploy_root": "/tmp/deploy-root",
        }
    )

    assert changed == {
        "github_repo": "other/repo",
        "github_token": "(set)",
        "hotspot_name": "pi-hotspot",
        "default_deploy_root": "/tmp/deploy-root",
    }
    assert operator.github_repo == "other/repo"
    assert operator.github_token == "new-secret"
    assert operator.hotspot_name == "pi-hotspot"
    assert operator.default_deploy_root == "/tmp/deploy-root"


def test_target_record_validates_identity_and_overlay():
    record = TargetRecord.from_dict(
        _make_target(),
        default_deploy_root="/home/penn/pennair-deploy",
        default_fleet_file="/tmp/fleet.yaml",
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
            default_fleet_file="/tmp/fleet.yaml",
        )

    with pytest.raises(ValueError, match="requires a non-empty vehicle_name"):
        TargetRecord.from_dict(
            {
                **_make_target(),
                "vehicle_name": "",
            },
            default_deploy_root="/home/penn/pennair-deploy",
            default_fleet_file="/tmp/fleet.yaml",
        )

    bad_overlay = TargetRecord.from_dict(
        {
            **_make_target(),
            "overlay_yaml": "- not-a-mapping",
        },
        default_deploy_root="/home/penn/pennair-deploy",
        default_fleet_file="/tmp/fleet.yaml",
    )
    with pytest.raises(ValueError, match="overlay must be a YAML mapping"):
        bad_overlay.overlay_data()

    with pytest.raises(ValueError, match="invalid pi_host"):
        TargetRecord.from_dict(
            {**_make_target(), "pi_host": "bad host"},
            default_deploy_root="/home/penn/pennair-deploy",
            default_fleet_file="/tmp/fleet.yaml",
        )

    with pytest.raises(ValueError, match="invalid service_unit"):
        TargetRecord.from_dict(
            {**_make_target(), "service_unit": "bad service"},
            default_deploy_root="/home/penn/pennair-deploy",
            default_fleet_file="/tmp/fleet.yaml",
        )


def test_inventory_store_round_trips_through_disk(tmp_path):
    operator = _make_operator(tmp_path)
    seed_target = TargetRecord.from_dict(
        _make_target(),
        default_deploy_root=operator.default_deploy_root,
        default_fleet_file="/tmp/fleet.yaml",
    )

    store = InventoryStore(
        operator.inventory_path,
        operator_config=operator,
        default_deploy_root=operator.default_deploy_root,
        default_fleet_file="/tmp/fleet.yaml",
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
            "fleet_file": "/tmp/backup_fleet.yaml",
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
    )
    restored = InventoryStore(
        operator.inventory_path,
        operator_config=restored_operator,
        default_deploy_root="/tmp/unused",
        default_fleet_file="/tmp/fleet.yaml",
        seed_target=seed_target,
    )

    assert restored.active_target_id() == "backup"
    assert restored_operator.github_repo == "pennaerial/monorepo"
    assert restored_operator.github_token == "secret-token"
    assert restored_operator.hotspot_name == "pennair-hotspot"
    assert restored_operator.default_deploy_root == "/home/penn/pennair-deploy"
    assert [target.target_id for target in restored.list_targets()] == [
        "backup",
        "pi-1",
    ]

    with pytest.raises(ValueError, match="Cannot delete the last target"):
        lone_store = InventoryStore(
            tmp_path / "single.json",
            operator_config=_make_operator(tmp_path),
            default_deploy_root="/home/penn/pennair-deploy",
            default_fleet_file="/tmp/fleet.yaml",
            seed_target=seed_target,
        )
        lone_store.delete_target("pi-1")


def test_inventory_store_exports_and_imports(tmp_path):
    operator = _make_operator(tmp_path)
    seed_target = TargetRecord.from_dict(
        _make_target(),
        default_deploy_root=operator.default_deploy_root,
        default_fleet_file="/tmp/fleet.yaml",
    )

    store = InventoryStore(
        operator.inventory_path,
        operator_config=operator,
        default_deploy_root=operator.default_deploy_root,
        default_fleet_file="/tmp/fleet.yaml",
        seed_target=seed_target,
    )

    payload = store.export_payload()
    assert payload["active_target_id"] == "pi-1"
    assert len(payload["targets"]) == 1

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
    assert [target.target_id for target in store.list_targets()] == ["pi-1", "pi-2"]

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
