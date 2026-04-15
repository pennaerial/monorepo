from __future__ import annotations

import os
import subprocess
import textwrap
from pathlib import Path
import sys

import pytest

from uav.runtime.fleet_spec import FleetDocumentModel, load_fleet_document
from uav.runtime.mission_spec import load_mission_spec
from uav.runtime.schema import (
    fleet_document_schema,
    mission_document_schema,
    mode_entry_for_class_path,
    mode_registry_entries,
    schema_registry_document,
)
from uav.runtime.schema_registry import ModeRegistryEntry


def _write_yaml(tmp_path: Path, name: str, contents: str) -> Path:
    path = tmp_path / name
    path.write_text(textwrap.dedent(contents), encoding="utf-8")
    return path


def test_mode_registry_entry_exposes_transition_labels_and_schema():
    entry = mode_entry_for_class_path("uav.vtol.TakeoffMode")

    assert entry.mode_id == "uav.vtol.TakeoffMode"
    assert entry.class_path == "uav.modes.uav.vtol.TakeoffMode"
    assert entry.mission_target == "uav"
    assert entry.requires_camera is False
    assert entry.transition_labels == ("complete",)
    assert "takeoff_type" in entry.params_schema["properties"]


def test_mode_registry_entry_marks_camera_only_modes():
    entry = mode_entry_for_class_path("payload.PayloadWaitForDriveOutMode")

    assert entry.mission_target == "payload"
    assert entry.required_vision_nodes == ()
    assert entry.requires_camera is True


def test_mode_registry_entry_preserves_canonical_vision_node_paths():
    entry = ModeRegistryEntry(
        mode_id="payload.PayloadScanForTagMode",
        class_path="uav.modes.payload.PayloadScanForTagMode",
        module_path="uav.modes.payload.PayloadScanForTagMode",
        class_name="PayloadScanForTagMode",
        display_name="PayloadScanForTagMode",
        description="Spin in place looking for a specific AprilTag.",
        mission_target="payload",
        required_vision_nodes=("uav.vision_nodes.PayloadAprilTagNode",),
        requires_camera=False,
        transition_labels=("found", "not_found"),
        params_schema={"type": "object"},
    )

    assert entry.required_vision_nodes == ("uav.vision_nodes.PayloadAprilTagNode",)


def test_mode_registry_entries_include_payload_modes():
    entries = mode_registry_entries(mission_target="payload")
    mode_ids = {entry.mode_id for entry in entries}

    assert "payload.PayloadDLZNavigateMode" in mode_ids
    assert "payload.PayloadAprilTagApproachMode" in mode_ids


def test_mode_registry_entry_exposes_peer_vehicle_names():
    entry = mode_entry_for_class_path("payload.PayloadPeerFleetTestMode")

    assert entry.mission_target == "payload"
    assert entry.peer_vehicle_names == ("payload_0", "payload_1")
    assert entry.requires_camera is False


def test_schema_registry_document_exposes_mode_entries_and_document_schemas():
    document = schema_registry_document()

    assert document.modes
    assert "properties" in mission_document_schema()
    assert "properties" in fleet_document_schema()


def test_mode_schema_registry_loads_without_mode_imports():
    package_root = Path(__file__).resolve().parents[1]
    command = textwrap.dedent(
        f"""
        import json
        import sys
        sys.path.insert(0, {str(package_root)!r})
        from uav.runtime.schema import mode_entry_for_class_path, mode_registry_entries

        takeoff = mode_entry_for_class_path("uav.vtol.TakeoffMode")
        payload_paths = [entry.mode_id for entry in mode_registry_entries(mission_target="payload")]
        print(json.dumps({{
            "takeoff": takeoff.mode_id,
            "payload_count": len(payload_paths),
        }}, sort_keys=True))
        """
    )
    env = {**os.environ, "PYTHONPATH": str(package_root)}
    result = subprocess.run(
        [sys.executable, "-c", command],
        capture_output=True,
        text=True,
        env=env,
        check=True,
    )

    assert '"takeoff": "uav.vtol.TakeoffMode"' in result.stdout
    assert '"payload_count":' in result.stdout


def test_mission_spec_rejects_unsupported_transition_labels(tmp_path):
    mission_path = _write_yaml(
        tmp_path,
        "invalid_transition.yaml",
        """
        modes:
          start:
            mode: uav.vtol.TakeoffMode
            transitions:
              invalid: end
          end:
            mode: uav.LandingMode
        """,
    )

    with pytest.raises(ValueError, match="unsupported transition label"):
        load_mission_spec(mission_path)


def test_mission_spec_rejects_invalid_mode_params(tmp_path):
    mission_path = _write_yaml(
        tmp_path,
        "invalid_params.yaml",
        """
        modes:
          start:
            mode: uav.vtol.TakeoffMode
            params:
              unexpected: true
        """,
    )

    with pytest.raises(ValueError, match="has invalid params"):
        load_mission_spec(mission_path)


def test_fleet_document_model_validates_checked_in_examples():
    fleets_dir = Path(__file__).resolve().parent.parent / "uav" / "fleets"
    assert fleets_dir.is_dir()

    for fleet_path in sorted(fleets_dir.glob("*.yaml")):
        document = load_fleet_document(fleet_path)
        assert isinstance(document, FleetDocumentModel)
        assert document.vehicles


def test_fleet_document_rejects_unknown_fields(tmp_path):
    fleet_path = _write_yaml(
        tmp_path,
        "invalid_fleet.yaml",
        """
        backend:
          kind: hardware
        vehicles:
          - name: uav_0
            mission: hover
            unsupported_knob: true
        """,
    )

    with pytest.raises(ValueError, match="unsupported_knob"):
        load_fleet_document(fleet_path)


def test_fleet_document_accepts_real_backend_alias(tmp_path):
    fleet_path = _write_yaml(
        tmp_path,
        "real_fleet.yaml",
        """
        backend:
          kind: real
        vehicles:
          - name: uav_0
            mission: hover
        """,
    )

    document = load_fleet_document(fleet_path)

    assert document.backend.kind == "hardware"
