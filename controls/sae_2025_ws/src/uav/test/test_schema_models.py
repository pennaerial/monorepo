from __future__ import annotations

import textwrap
from pathlib import Path

import pytest

from uav.runtime.fleet_spec import FleetDocumentModel, load_fleet_document
from uav.runtime.mission_spec import load_mission_spec
from uav.runtime.schema import mode_entry_for_class_path, mode_registry_entries


def _write_yaml(tmp_path: Path, name: str, contents: str) -> Path:
    path = tmp_path / name
    path.write_text(textwrap.dedent(contents), encoding="utf-8")
    return path


def test_mode_registry_entry_exposes_transition_labels_and_schema():
    entry = mode_entry_for_class_path("uav.modes.uav.TakeoffMode")

    assert entry.class_path == "uav.modes.uav.TakeoffMode"
    assert entry.mission_target == "uav"
    assert entry.transition_labels == ("complete",)
    assert "takeoff_type" in entry.params_schema["properties"]


def test_mode_registry_entries_include_payload_modes():
    entries = mode_registry_entries(mission_target="payload")
    class_paths = {entry.class_path for entry in entries}

    assert "uav.modes.payload.PayloadDLZNavigateMode" in class_paths
    assert "uav.modes.payload.PayloadAprilTagApproachMode" in class_paths


def test_mission_spec_rejects_unsupported_transition_labels(tmp_path):
    mission_path = _write_yaml(
        tmp_path,
        "invalid_transition.yaml",
        """
        modes:
          start:
            class: uav.modes.uav.TakeoffMode
            transitions:
              invalid: end
          end:
            class: uav.modes.uav.LandingMode
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
            class: uav.modes.uav.TakeoffMode
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
