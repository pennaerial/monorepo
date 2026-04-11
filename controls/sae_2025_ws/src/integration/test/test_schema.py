from __future__ import annotations

import os
import subprocess
import sys
import types
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]

for path in (PACKAGE_ROOT,):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

backend_pkg = types.ModuleType("backend")
backend_pkg.__path__ = [str(PACKAGE_ROOT / "backend")]
sys.modules.setdefault("backend", backend_pkg)
services_pkg = types.ModuleType("backend.services")
services_pkg.__path__ = [str(PACKAGE_ROOT / "backend" / "services")]
sys.modules.setdefault("backend.services", services_pkg)

from backend.schema import (  # noqa: E402
    fleet_schema,
    mission_catalog,
    mission_schema_for_name,
    mode_registry,
    schema_index,
)

_BLOCKED_IMPORT_PREFIXES = (
    "rclpy",
    "px4_msgs",
    "sensor_msgs",
    "cv_bridge",
    "payload_interfaces",
    "uav_interfaces",
    "uav.modes",
    "uav.vehicles",
    "uav.vision_nodes",
)


def _run_clean_import(script: str) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    uav_root = PACKAGE_ROOT.parent / "uav"
    env["PYTHONPATH"] = os.pathsep.join(
        [str(PACKAGE_ROOT), str(uav_root), env.get("PYTHONPATH", "")]
    ).rstrip(os.pathsep)
    return subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        text=True,
        env=env,
        check=False,
    )


def test_backend_schema_imports_without_ros_runtime_dependencies():
    script = f"""
import builtins
import types
import sys
from pathlib import Path

package_root = Path({str(PACKAGE_ROOT)!r})
backend_pkg = types.ModuleType("backend")
backend_pkg.__path__ = [str(package_root / "backend")]
sys.modules.setdefault("backend", backend_pkg)

blocked_prefixes = {tuple(_BLOCKED_IMPORT_PREFIXES)!r}
real_import = builtins.__import__

def guarded_import(name, globals=None, locals=None, fromlist=(), level=0):
    if name.startswith(blocked_prefixes):
        raise RuntimeError(f"blocked import: {{name}}")
    return real_import(name, globals, locals, fromlist, level)

builtins.__import__ = guarded_import
from backend.schema import schema_index
schema_index()
"""
    result = _run_clean_import(script)
    assert result.returncode == 0, result.stderr or result.stdout


def test_backend_services_import_without_ros_runtime_dependencies():
    script = f"""
import builtins
import importlib
import types
import sys
from pathlib import Path

package_root = Path({str(PACKAGE_ROOT)!r})
backend_pkg = types.ModuleType("backend")
backend_pkg.__path__ = [str(package_root / "backend")]
sys.modules.setdefault("backend", backend_pkg)
services_pkg = types.ModuleType("backend.services")
services_pkg.__path__ = [str(package_root / "backend" / "services")]
sys.modules.setdefault("backend.services", services_pkg)

blocked_prefixes = {tuple(_BLOCKED_IMPORT_PREFIXES)!r}
real_import = builtins.__import__

def guarded_import(name, globals=None, locals=None, fromlist=(), level=0):
    if name.startswith(blocked_prefixes):
        raise RuntimeError(f"blocked import: {{name}}")
    return real_import(name, globals, locals, fromlist, level)

builtins.__import__ = guarded_import
importlib.import_module("backend.services.deploy")
importlib.import_module("backend.services.mission")
"""
    result = _run_clean_import(script)
    assert result.returncode == 0, result.stderr or result.stdout


def test_schema_index_exposes_missions_fleet_and_modes():
    payload = schema_index().model_dump()

    assert payload["success"] is True
    assert "hover" in payload["missions"]["available_missions"]
    assert "payload_retreat" in payload["missions"]["available_missions"]
    assert "example_fleet" in payload["fleet"]["available_fleets"]
    assert "sae_payload_pickup" in payload["fleet"]["available_fleets"]
    assert set(payload["modes"]["targets"]) == {"uav", "payload"}

    backend_section = next(
        section
        for section in payload["fleet"]["sections"]
        if section["name"] == "backend"
    )
    assert backend_section["applies_to"] == ["sim", "hardware"]

    vehicle_sim = next(
        section
        for section in payload["fleet"]["sections"]
        if section["name"] == "vehicle.sim"
    )
    vehicle_hardware = next(
        section
        for section in payload["fleet"]["sections"]
        if section["name"] == "vehicle.hardware"
    )
    assert {field["name"] for field in vehicle_sim["fields"]} >= {
        "controllable",
        "px4_instance",
    }
    assert {field["name"] for field in vehicle_hardware["fields"]} >= {
        "px4_airframe_id",
        "px4_namespace",
        "payload_controller",
    }


def test_schema_mission_exposes_constructor_metadata():
    payload = mission_schema_for_name("hover").model_dump()

    assert payload["success"] is True
    assert payload["name"] == "hover"
    assert payload["target"] == "uav"
    assert payload["start_mode"] == "start"
    start_mode = next(mode for mode in payload["modes"] if mode["name"] == "start")
    takeoff_type = next(
        field
        for field in start_mode["metadata"]["params"]
        if field["name"] == "takeoff_type"
    )
    assert takeoff_type["schema_type"] == "enum"
    assert takeoff_type["default"] == "vertical"
    assert set(takeoff_type["choices"]) == {"vertical", "horizontal"}
    assert start_mode["metadata"]["transition_labels"] == ["complete"]
    assert payload["document_schema"]["type"] == "object"

    gps_mode = next(mode for mode in payload["modes"] if mode["name"] == "GPS")
    coordinates = next(
        field
        for field in gps_mode["metadata"]["params"]
        if field["name"] == "coordinates"
    )
    assert coordinates["schema_type"] == "list"
    assert coordinates["required"] is True


def test_schema_mode_registry_can_filter_by_target():
    payload = mode_registry(target="payload").model_dump()

    assert payload["success"] is True
    assert set(payload["targets"]) == {"payload"}
    assert any(
        item["name"] == "PayloadRetreatMode" for item in payload["targets"]["payload"]
    )


def test_schema_mission_catalog_can_filter_by_target():
    payload = mission_catalog(target="payload").model_dump()

    assert payload["success"] is True
    assert all(mission["target"] == "payload" for mission in payload["missions"])
    assert "payload_retreat" in payload["available_missions"]


def test_fleet_schema_exposes_section_shapes():
    payload = fleet_schema().model_dump()

    assert payload["success"] is True
    assert payload["backend_kinds"] == ["sim", "hardware"]
    assert "airframe" in payload["excluded_keys"]
    assert any(section["name"] == "defaults" for section in payload["sections"])
    assert payload["document_schema"]["type"] == "object"
