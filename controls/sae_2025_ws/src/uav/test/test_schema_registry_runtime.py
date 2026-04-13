from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys
import textwrap

import uav.runtime.mission_spec as mission_spec_module
from uav.runtime.mission_spec import load_mission_spec
from uav.runtime.schema_registry import load_mode_registry_document


def _write_mission(tmp_path: Path, contents: str) -> Path:
    mission_path = tmp_path / "mission.yaml"
    mission_path.write_text(textwrap.dedent(contents), encoding="utf-8")
    return mission_path


def test_load_mission_spec_does_not_import_mode_classes(monkeypatch, tmp_path):
    mission_path = _write_mission(
        tmp_path,
        """
        modes:
          start:
            class: uav.modes.uav.TakeoffMode
            params:
              altitude: 3.0
            transitions:
              complete: land
          land:
            class: uav.modes.uav.LandingMode
        """,
    )

    def _unexpected_import(_class_path: str):
        raise AssertionError(
            "load_mode_class should not be used for mission validation"
        )

    monkeypatch.setattr(mission_spec_module, "load_mode_class", _unexpected_import)

    mission_spec = load_mission_spec(mission_path)

    assert mission_spec.target == "uav"
    assert mission_spec.modes["start"].params == {"altitude": 3.0}


def test_load_mission_spec_works_without_ros_setup(tmp_path):
    mission_path = _write_mission(
        tmp_path,
        """
        modes:
          start:
            class: uav.modes.payload.PayloadAprilTagApproachMode
            params:
              tag_id: 3
              stop_distance_m: 0.05
        """,
    )

    package_root = Path(__file__).resolve().parents[1]
    command = textwrap.dedent(
        f"""
        import json
        import sys
        sys.path.insert(0, {str(package_root)!r})
        from uav.runtime.mission_spec import load_mission_spec

        mission = load_mission_spec({str(mission_path)!r})
        print(json.dumps({{
            "target": mission.target,
            "requires_camera": mission.requires_camera,
            "vision_nodes": mission.vision_nodes,
            "params": mission.modes["start"].params,
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

    assert '"target": "payload"' in result.stdout
    assert '"requires_camera": true' in result.stdout
    assert '"vision_nodes": []' in result.stdout


def test_load_fleet_document_works_without_ros_setup():
    package_root = Path(__file__).resolve().parents[1]
    fleet_path = package_root / "uav" / "fleets" / "example_fleet.yaml"
    command = textwrap.dedent(
        f"""
        import json
        import sys
        sys.path.insert(0, {str(package_root)!r})
        from uav.runtime.fleet_spec import load_fleet_document

        fleet = load_fleet_document({str(fleet_path)!r})
        print(json.dumps({{
            "backend_kind": fleet.backend.kind,
            "vehicle_count": len(fleet.vehicles),
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

    assert '"backend_kind": "sim"' in result.stdout
    assert '"vehicle_count":' in result.stdout


def test_committed_mode_registry_contains_required_modes():
    modes = load_mode_registry_document().modes

    assert "uav.modes.uav.TakeoffMode" in modes
    assert "uav.modes.payload.PayloadRetreatMode" in modes
