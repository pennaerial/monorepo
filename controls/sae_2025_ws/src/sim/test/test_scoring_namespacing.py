from __future__ import annotations

import sys
from pathlib import Path

import pytest

# The scoring package imports generated message types (sim_interfaces), which are
# only built in a full sim workspace. Skip cleanly when they are unavailable
# (e.g. the hardware-focused CI build) instead of failing collection.
pytest.importorskip(
    "sim_interfaces",
    reason="sim scoring requires the built sim_interfaces messages",
)

package_root = Path(__file__).resolve().parents[1]
if str(package_root) not in sys.path:
    sys.path.insert(0, str(package_root))

from sim.scoring.namespacing import (  # noqa: E402
    resolve_scoring_vehicle_name,
    vehicle_px4_local_position_topic,
)


def test_vehicle_px4_local_position_topic_uses_vehicle_name():
    assert vehicle_px4_local_position_topic("uav_0") == "/uav_0/fmu/out/vehicle_local_position"


def test_resolve_scoring_vehicle_name_prefers_explicit_vehicle_name():
    world_params = {
        "controllables": {
            "uav_0": {"kind": "uav", "px4_airframe_id": 4010},
            "uav_1": {"kind": "uav", "px4_airframe_id": 4004},
        }
    }
    scoring_params = {"vehicle_name": "uav_1"}

    assert resolve_scoring_vehicle_name(scoring_params, world_params) == "uav_1"


def test_resolve_scoring_vehicle_name_infers_single_uav():
    world_params = {
        "controllables": {
            "payload_0": {"kind": "payload"},
            "uav_0": {"kind": "uav", "px4_airframe_id": 4010},
        }
    }

    assert resolve_scoring_vehicle_name({}, world_params) == "uav_0"


def test_resolve_scoring_vehicle_name_rejects_ambiguous_multi_uav():
    world_params = {
        "controllables": {
            "uav_0": {"kind": "uav", "px4_airframe_id": 4010},
            "uav_1": {"kind": "uav", "px4_airframe_id": 4004},
        }
    }

    try:
        resolve_scoring_vehicle_name({}, world_params)
    except ValueError as exc:
        message = str(exc)
        assert "vehicle_name" in message
        assert "uav_0" in message
        assert "uav_1" in message
    else:
        raise AssertionError(
            "Expected resolve_scoring_vehicle_name to reject ambiguous multi-UAV scoring."
        )
