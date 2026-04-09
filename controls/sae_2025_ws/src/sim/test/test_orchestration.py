from __future__ import annotations

import sys
from pathlib import Path

package_root = Path(__file__).resolve().parents[1]
if str(package_root) not in sys.path:
    sys.path.insert(0, str(package_root))

import pytest  # noqa: E402

from sim.orchestration import (  # noqa: E402
    deep_merge_dicts,
    normalize_named_records,
    normalized_stage_name,
    normalize_stage_world_params,
    parse_json_config,
    resolve_stage_world,
)


def test_parse_json_config_returns_empty_dict_for_blank_input():
    assert parse_json_config("", field_name="backend_json") == {}


def test_parse_json_config_requires_object():
    try:
        parse_json_config('["not", "an", "object"]', field_name="backend_json")
    except ValueError as exc:
        assert "must decode to a JSON object" in str(exc)
    else:
        raise AssertionError("Expected parse_json_config to reject non-object JSON.")


def test_deep_merge_dicts_merges_nested_structures():
    base = {
        "world": {
            "name": "SAEWorldNode",
            "params": {
                "physics": {"friction": 0.6},
                "entities": {"dlz": {"position": [0, 0, 0]}},
            },
        }
    }
    override = {
        "world": {
            "params": {
                "entities": {"payload_0": {"position": [1, 2, 3]}},
                "controllables": {"uav_0": {"position": [4, 5, 6]}},
            }
        }
    }

    merged = deep_merge_dicts(base, override)

    assert merged["world"]["name"] == "SAEWorldNode"
    assert merged["world"]["params"]["physics"] == {"friction": 0.6}
    assert merged["world"]["params"]["entities"]["dlz"]["position"] == [0, 0, 0]
    assert merged["world"]["params"]["entities"]["payload_0"]["position"] == [1, 2, 3]
    assert merged["world"]["params"]["controllables"]["uav_0"]["position"] == [4, 5, 6]


def test_normalize_named_records_accepts_mapping_and_list():
    mapping = normalize_named_records(
        {
            "payload_0": {"path_to_sdf": "payload.sdf"},
            "uav_0": {"path_to_sdf": "uav.sdf"},
        },
        record_type="controllable",
    )
    listed = normalize_named_records(
        [
            {"name": "payload_0", "path_to_sdf": "payload.sdf"},
            {"name": "uav_0", "path_to_sdf": "uav.sdf"},
        ],
        record_type="controllable",
    )

    assert mapping == listed
    assert set(mapping) == {"payload_0", "uav_0"}


def test_normalize_named_records_rejects_missing_names():
    try:
        normalize_named_records([{"path_to_sdf": "payload.sdf"}], record_type="entity")
    except ValueError as exc:
        assert "non-empty 'name'" in str(exc)
    else:
        raise AssertionError(
            "Expected normalize_named_records to reject missing names."
        )


def test_normalized_stage_name_defaults_to_base():
    assert normalized_stage_name("") == "base"
    assert normalized_stage_name(None) == "base"
    assert normalized_stage_name("payload_retreat") == "payload_retreat"


def test_normalize_stage_world_params_keeps_standard_spawnables_separated():
    params = normalize_stage_world_params(
        {
            "template_world": "worlds/template.sdf",
            "entities": {
                "dlz": {"path_to_sdf": "dlz.sdf"},
                "marker": {"path_to_sdf": "marker.sdf"},
            },
            "controllables": {
                "uav_0": {
                    "kind": "uav",
                    "model": "gz_x500_mono_cam",
                    "px4_airframe_id": 4010,
                },
                "payload_0": {
                    "kind": "payload",
                    "path_to_sdf": "payload.sdf",
                },
            },
        }
    )

    assert params["template_world"] == "worlds/template.sdf"
    assert params["entities"]["dlz"]["path_to_sdf"] == "dlz.sdf"
    assert params["entities"]["marker"]["path_to_sdf"] == "marker.sdf"
    assert params["controllables"]["uav_0"]["px4_airframe_id"] == 4010
    assert params["controllables"]["payload_0"]["path_to_sdf"] == "payload.sdf"


@pytest.mark.parametrize("legacy_key", ["dlz", "payload_0", "payload_1"])
def test_normalize_stage_world_params_rejects_legacy_top_level_spawnables(legacy_key):
    try:
        normalize_stage_world_params(
            {
                "template_world": "worlds/template.sdf",
                legacy_key: {"path_to_sdf": f"{legacy_key}.sdf"},
            }
        )
    except ValueError as exc:
        assert legacy_key in str(exc)
        assert "entities" in str(exc) or "controllables" in str(exc)
    else:
        raise AssertionError(
            "Expected normalize_stage_world_params to reject legacy top-level spawnables."
        )


def test_resolve_stage_world_exposes_standard_stage_spawnables(monkeypatch):
    def fake_load_sim_parameters(
        world_name,
        *,
        logger=None,
        competition_name=None,
        mission_stage=None,
    ):
        assert world_name == "sae"
        assert competition_name == "sae"
        assert mission_stage == "payload_retreat"
        return (
            {
                "world": {
                    "name": "SAEWorldNode",
                    "params": {
                        "template_world": "worlds/template.sdf",
                        "entities": {
                            "dlz": {
                                "path_to_sdf": "~/.simulation-gazebo/models/dlz_white/model.sdf",
                                "position": [0.0, 0.0, 0.0],
                                "rpy": [0.0, 0.0, 0.0],
                            }
                        },
                        "controllables": {
                            "uav_0": {
                                "kind": "uav",
                                "model": "gz_standard_vtol",
                                "px4_airframe_id": 4004,
                                "path_to_sdf": "~/.simulation-gazebo/models/standard_vtol/model.sdf",
                                "position": [5.0, 0.0, 0.1],
                                "rpy": [0.0, 0.0, 0.0],
                            },
                            "payload_0": {
                                "kind": "payload",
                                "path_to_sdf": "~/.simulation-gazebo/models/payload/model.sdf",
                                "position": [0.0, 0.0, 0.1],
                                "rpy": [0.0, 0.0, 0.0],
                            },
                        },
                    },
                }
            },
            Path("/tmp/sae/payload_retreat.yaml"),
        )

    monkeypatch.setattr(
        "sim.orchestration.load_sim_parameters", fake_load_sim_parameters
    )
    resolved = resolve_stage_world(world_name="sae", mission_stage="payload_retreat")

    assert resolved["world_name"] == "sae"
    assert resolved["mission_stage"] == "payload_retreat"
    assert resolved["config_path"].name == "payload_retreat.yaml"
    assert resolved["world"]["name"] == "SAEWorldNode"
    assert resolved["world"]["params"]["entities"]["dlz"]["path_to_sdf"].endswith(
        "dlz_white/model.sdf"
    )
    assert (
        resolved["world"]["params"]["controllables"]["uav_0"]["px4_airframe_id"] == 4004
    )
    assert (
        resolved["world"]["params"]["controllables"]["payload_0"]["kind"] == "payload"
    )


def test_resolve_stage_world_defaults_to_base_stage(monkeypatch):
    def fake_load_sim_parameters(
        world_name,
        *,
        logger=None,
        competition_name=None,
        mission_stage=None,
    ):
        assert world_name == "sae"
        assert competition_name == "sae"
        assert mission_stage == "base"
        return (
            {
                "world": {
                    "name": "SAEWorldNode",
                    "params": {
                        "template_world": "worlds/template.sdf",
                        "entities": {
                            "dlz": {
                                "path_to_sdf": "~/.simulation-gazebo/models/dlz/model.sdf"
                            }
                        },
                        "controllables": {
                            "payload_0": {
                                "kind": "payload",
                                "path_to_sdf": "~/.simulation-gazebo/models/payload/model.sdf",
                            },
                            "payload_1": {
                                "kind": "payload",
                                "path_to_sdf": "~/.simulation-gazebo/models/payload/model.sdf",
                            },
                        },
                    },
                }
            },
            Path("/tmp/sae/base.yaml"),
        )

    monkeypatch.setattr(
        "sim.orchestration.load_sim_parameters", fake_load_sim_parameters
    )
    resolved = resolve_stage_world(world_name="sae")

    assert resolved["world_name"] == "sae"
    assert resolved["mission_stage"] == "base"
    assert resolved["config_path"].name == "base.yaml"
    assert resolved["world"]["params"]["entities"]["dlz"]["path_to_sdf"].endswith(
        "dlz/model.sdf"
    )
    assert "payload_0" in resolved["world"]["params"]["controllables"]
    assert "payload_1" in resolved["world"]["params"]["controllables"]


def test_resolve_stage_world_loads_specific_stage_and_merges_world_overrides(
    monkeypatch,
):
    def fake_load_sim_parameters(
        world_name,
        *,
        logger=None,
        competition_name=None,
        mission_stage=None,
    ):
        assert world_name == "sae"
        assert competition_name == "sae"
        assert mission_stage == "payload_retreat"
        return (
            {
                "world": {
                    "name": "SAEWorldNode",
                    "params": {
                        "template_world": "worlds/template.sdf",
                        "entities": {
                            "dlz": {
                                "path_to_sdf": "~/.simulation-gazebo/models/dlz_white/model.sdf",
                                "position": [0.0, 0.0, 0.0],
                                "rpy": [0.0, 0.0, 0.0],
                            }
                        },
                        "controllables": {
                            "payload_0": {
                                "kind": "payload",
                                "path_to_sdf": "~/.simulation-gazebo/models/payload/model.sdf",
                                "position": [0.0, 0.0, 0.1],
                                "rpy": [0.0, 0.0, 0.0],
                            }
                        },
                    },
                }
            },
            Path("/tmp/sae/payload_retreat.yaml"),
        )

    monkeypatch.setattr(
        "sim.orchestration.load_sim_parameters", fake_load_sim_parameters
    )
    resolved = resolve_stage_world(
        world_name="sae",
        mission_stage="payload_retreat",
        world_overrides={
            "params": {
                "entities": {
                    "marker": {
                        "path_to_sdf": "~/.simulation-gazebo/models/marker/model.sdf",
                        "position": [1.0, 2.0, 3.0],
                        "rpy": [0.0, 0.0, 0.0],
                    }
                },
                "controllables": {
                    "uav_0": {
                        "kind": "uav",
                        "path_to_sdf": "~/.simulation-gazebo/models/x500/model.sdf",
                        "position": [4.0, 5.0, 6.0],
                        "rpy": [0.0, 0.0, 0.0],
                    }
                },
            }
        },
    )

    assert resolved["mission_stage"] == "payload_retreat"
    assert resolved["config_path"].name == "payload_retreat.yaml"
    assert resolved["world"]["params"]["entities"]["dlz"]["path_to_sdf"].endswith(
        "dlz_white/model.sdf"
    )
    assert resolved["world"]["params"]["entities"]["marker"]["path_to_sdf"].endswith(
        "marker/model.sdf"
    )
    assert (
        resolved["world"]["params"]["controllables"]["payload_0"]["kind"] == "payload"
    )
    assert resolved["world"]["params"]["controllables"]["uav_0"]["kind"] == "uav"
    assert resolved["world"]["params"]["controllables"]["uav_0"]["position"] == [
        4.0,
        5.0,
        6.0,
    ]


def test_resolve_stage_world_rejects_non_mapping_overrides():
    try:
        resolve_stage_world(world_name="sae", world_overrides=["bad"])
    except ValueError as exc:
        assert "world_overrides must be a mapping" in str(exc)
    else:
        raise AssertionError(
            "Expected resolve_stage_world to reject non-mapping overrides."
        )


def test_resolve_stage_world_rejects_unknown_world():
    try:
        resolve_stage_world(world_name="missing_world")
    except FileNotFoundError as exc:
        assert "base.yaml" in str(exc)
    else:
        raise AssertionError("Expected resolve_stage_world to reject unknown worlds.")


def test_resolve_stage_world_rejects_unknown_stage():
    try:
        resolve_stage_world(world_name="sae", mission_stage="missing_stage")
    except FileNotFoundError as exc:
        assert "missing_stage.yaml" in str(exc)
    else:
        raise AssertionError("Expected resolve_stage_world to reject unknown stages.")
