from __future__ import annotations

import pytest

from vehicle_common.runtime.mode_paths import (
    mission_target_from_mode_id,
    mode_id_from_class_path,
    normalize_public_mode_id,
)


@pytest.mark.parametrize(
    ("class_path", "expected"),
    [
        ("uav.modes.LandingMode", "uav.LandingMode"),
        ("uav.modes.vtol.TakeoffMode", "uav.vtol.TakeoffMode"),
        (
            "uav.modes.vtol.TakeoffMode.TakeoffMode",
            "uav.vtol.TakeoffMode",
        ),
        (
            "payload.modes.PayloadAprilTagApproachMode",
            "payload.PayloadAprilTagApproachMode",
        ),
        ("uav.modes.uav.LandingMode", "uav.LandingMode"),
        (
            "uav.modes.payload.PayloadAprilTagApproachMode",
            "payload.PayloadAprilTagApproachMode",
        ),
    ],
)
def test_mode_id_from_class_path_supports_split_packages(class_path, expected):
    assert mode_id_from_class_path(class_path) == expected


@pytest.mark.parametrize(
    "mode_id",
    [
        "uav.modes.LandingMode",
        "payload.modes.PayloadAprilTagApproachMode",
    ],
)
def test_normalize_public_mode_id_rejects_implementation_paths(mode_id):
    with pytest.raises(ValueError, match="uses the internal Python path"):
        normalize_public_mode_id(mode_id)


def test_mission_target_comes_from_public_mode_id():
    assert mission_target_from_mode_id("uav.vtol.TakeoffMode") == "uav"
    assert mission_target_from_mode_id("payload.PayloadRetreatMode") == "payload"
