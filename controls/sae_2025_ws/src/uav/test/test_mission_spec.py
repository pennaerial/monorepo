from __future__ import annotations

from pathlib import Path
import textwrap
from types import SimpleNamespace
import sys
import types

import pytest

from uav.modes.Mode import Mode
import uav.runtime.mission_spec as mission_spec_module
import uav.runtime.schema as schema_module
from uav.runtime.mission_spec import (
    MissionSpec,
    load_mode_class,
    load_mission_spec,
    mission_path_for_name,
    mission_root,
)


def _write_mission(tmp_path: Path, contents: str) -> Path:
    mission_path = tmp_path / "mission.yaml"
    mission_path.write_text(textwrap.dedent(contents), encoding="utf-8")
    return mission_path


def _write_mode_module(
    tmp_path: Path,
    *,
    package_name: str,
    module_name: str,
    body: str,
) -> str:
    package_dir = tmp_path / package_name
    package_dir.mkdir(parents=True, exist_ok=True)
    (package_dir / "__init__.py").write_text("", encoding="utf-8")
    (package_dir / f"{module_name}.py").write_text(
        textwrap.dedent(body), encoding="utf-8"
    )
    return f"{package_name}.{module_name}"


def test_load_uav_mission_spec(tmp_path):
    mission_path = _write_mission(
        tmp_path,
        """
        modes:
          start:
            class: uav.modes.uav.TakeoffMode
            params:
              altitude: 3.0
            transitions:
              complete: cruise
          cruise:
            class: uav.modes.uav.NavGPSMode
            params:
              coordinates:
                - [[1.0, 2.0, 3.0], 0.0, LOCAL]
              margin: 0.2
            transitions:
              complete: land
          land:
            class: uav.modes.uav.LandingMode
        """,
    )

    mission_spec = load_mission_spec(mission_path)

    assert mission_spec.target == "uav"
    assert mission_spec.is_uav is True
    assert mission_spec.is_payload is False
    assert mission_spec.vision_nodes == ()
    assert mission_spec.requires_camera is False
    assert mission_spec.path == mission_path
    assert mission_spec.modes["start"].class_path == "uav.modes.uav.TakeoffMode"
    assert mission_spec.modes["start"].params == {"altitude": 3.0}
    assert mission_spec.modes["start"].transitions == {"complete": "cruise"}
    assert mission_spec.modes["cruise"].params == {
        "coordinates": [((1.0, 2.0, 3.0), 0.0, "LOCAL")],
        "margin": 0.2,
    }


def test_load_payload_mission_spec(tmp_path, monkeypatch):
    mission_path = _write_mission(
        tmp_path,
        """
        modes:
          start:
            class: uav.modes.payload.PayloadScanForTagMode
            params:
              tag_id: 1
            transitions:
              found: approach
              not_found: approach
          approach:
            class: uav.modes.payload.PayloadAprilTagApproachMode
            params:
              tag_id: 1
              stop_distance_m: 0.07
        """,
    )

    fake_entries = {
        "uav.modes.payload.PayloadScanForTagMode": SimpleNamespace(
            mission_target="payload",
            required_vision_nodes=("uav.vision_nodes.PayloadAprilTagNode",),
            requires_camera=False,
            transition_labels=("found", "not_found"),
        ),
        "uav.modes.payload.PayloadAprilTagApproachMode": SimpleNamespace(
            mission_target="payload",
            required_vision_nodes=(),
            requires_camera=True,
            transition_labels=("done",),
        ),
    }

    monkeypatch.setattr(
        schema_module,
        "mode_entry_for_class_path",
        lambda class_path: fake_entries[class_path],
    )
    monkeypatch.setattr(
        schema_module,
        "validate_mode_params",
        lambda _class_path, params: params,
    )

    mission_spec = load_mission_spec(mission_path)

    assert mission_spec.target == "payload"
    assert mission_spec.is_payload is True
    assert mission_spec.is_uav is False
    assert mission_spec.vision_nodes == ("uav.vision_nodes.PayloadAprilTagNode",)
    assert mission_spec.requires_camera is True
    assert mission_spec.modes["start"].class_path == (
        "uav.modes.payload.PayloadScanForTagMode"
    )
    assert mission_spec.modes["start"].params["tag_id"] == 1
    assert mission_spec.modes["approach"].class_path == (
        "uav.modes.payload.PayloadAprilTagApproachMode"
    )


def test_mission_spec_from_dict():
    mission_spec = MissionSpec.from_dict(
        {
            "modes": {
                "start": {
                    "class": "uav.modes.payload.PayloadAprilTagApproachMode",
                    "params": {"tag_id": 0, "stop_distance_m": 0.05},
                }
            }
        }
    )

    assert mission_spec.target == "payload"
    assert mission_spec.path is None
    assert mission_spec.vision_nodes == ()
    assert mission_spec.requires_camera is True
    assert mission_spec.modes["start"].params["tag_id"] == 0


def test_mission_spec_marks_camera_only_mode_as_camera_required():
    mission_spec = MissionSpec.from_dict(
        {
            "modes": {
                "start": {
                    "class": "uav.modes.payload.PayloadWaitForDriveOutMode.PayloadWaitForDriveOutMode",
                }
            }
        }
    )

    assert mission_spec.target == "payload"
    assert mission_spec.vision_nodes == ()
    assert mission_spec.requires_camera is True


@pytest.mark.parametrize(
    ("contents", "pattern"),
    [
        ("[1]", "must define a mapping at the top level"),
        ("modes: {}", "must define a non-empty modes mapping"),
        ("modes: []", "must define a non-empty modes mapping"),
        (
            """
            modes:
              cruise:
                class: uav.modes.uav.NavGPSMode
            """,
            "must define a start mode",
        ),
        (
            """
            target: uav
            modes:
              start:
                class: uav.modes.uav.TakeoffMode
            """,
            "unsupported top-level keys",
        ),
        (
            """
            modes:
              start: []
            """,
            "must be a mapping",
        ),
        (
            """
            modes:
              start:
                class: uav.modes.uav.TakeoffMode
              "":
                class: uav.modes.uav.TakeoffMode
            """,
            "contains an invalid mode name",
        ),
        (
            """
            modes:
              start:
                class: uav.modes.uav.TakeoffMode
                extra: true
            """,
            "contains unsupported keys",
        ),
        (
            """
            modes:
              start:
                class: 3
            """,
            "must define a non-empty class path",
        ),
        (
            """
            modes:
              start:
                class: uav.modes.uav.TakeoffMode
                params: []
            """,
            "must define params as a mapping",
        ),
        (
            """
            modes:
              start:
                class: uav.modes.uav.TakeoffMode
                transitions: []
            """,
            "must define transitions as a string-to-string mapping",
        ),
        (
            """
            modes:
              start:
                class: uav.modes.uav.TakeoffMode
                transitions:
                  complete: 1
            """,
            "must define transitions as a string-to-string mapping",
        ),
    ],
)
def test_invalid_mission_schema_is_rejected(tmp_path, contents, pattern):
    mission_path = _write_mission(tmp_path, contents)

    with pytest.raises(ValueError, match=pattern):
        load_mission_spec(mission_path)


def test_mode_derived_metadata_rejects_mixed_targets(tmp_path):
    mission_path = _write_mission(
        tmp_path,
        """
        modes:
          start:
            class: uav.modes.uav.TakeoffMode
            transitions:
              complete: payload
          payload:
            class: uav.modes.payload.PayloadRetreatMode
        """,
    )

    with pytest.raises(ValueError, match="mixes incompatible mode targets"):
        load_mission_spec(mission_path)


def test_undefined_transition_target_is_rejected(tmp_path):
    mission_path = _write_mission(
        tmp_path,
        """
        modes:
          start:
            class: uav.modes.uav.TakeoffMode
            transitions:
              complete: missing
        """,
    )

    with pytest.raises(ValueError, match="transitions to undefined mode"):
        load_mission_spec(mission_path)


@pytest.mark.parametrize("mission_target", [None, "boat"])
def test_invalid_mode_target_is_rejected(monkeypatch, mission_target):
    monkeypatch.setattr(
        schema_module,
        "mode_entry_for_class_path",
        lambda _path: type(
            "FakeEntry",
            (),
            {
                "mission_target": mission_target,
                "required_vision_nodes": (),
                "transition_labels": (),
            },
        )(),
    )
    monkeypatch.setattr(
        schema_module,
        "validate_mode_params",
        lambda _path, params: params,
    )

    with pytest.raises(ValueError, match="must declare mission_target"):
        load_mission_spec({"modes": {"start": {"class": "fake.module.FakeMode"}}})


def test_load_mode_class_accepts_module_path(monkeypatch):
    module_name = "uav.modes.payload.PayloadAprilTagApproachMode"
    fake_module = types.ModuleType(module_name)

    class PayloadAprilTagApproachMode(Mode):
        mission_target = "payload"

        def __init__(self, node, vehicle) -> None:
            super().__init__(node, vehicle)

        def on_update(self, time_delta: float) -> None:
            pass

        def check_status(self) -> str:
            return "continue"

    PayloadAprilTagApproachMode.__module__ = module_name
    fake_module.PayloadAprilTagApproachMode = PayloadAprilTagApproachMode
    monkeypatch.setitem(sys.modules, module_name, fake_module)

    mode_class = load_mode_class("uav.modes.payload.PayloadAprilTagApproachMode")

    assert mode_class.__name__ == "PayloadAprilTagApproachMode"
    assert mode_class.mission_target == "payload"


def test_load_mode_class_falls_back_from_class_path(tmp_path, monkeypatch):
    module_path = _write_mode_module(
        tmp_path,
        package_name="test_modes",
        module_name="fake_mode",
        body="""
        from uav.modes.Mode import Mode

        class FakeMode(Mode):
            mission_target = "uav"

            def __init__(self, node, vehicle):
                super().__init__(node, vehicle)

            def on_update(self, time_delta: float) -> None:
                pass

            def check_status(self) -> str:
                return "continue"
        """,
    )
    monkeypatch.syspath_prepend(str(tmp_path))

    mode_class = load_mode_class(f"{module_path}.FakeMode")

    assert mode_class.__name__ == "FakeMode"
    assert issubclass(mode_class, Mode)


def test_load_mode_class_rejects_non_mode_subclass(tmp_path, monkeypatch):
    module_path = _write_mode_module(
        tmp_path,
        package_name="test_modes_invalid",
        module_name="not_a_mode",
        body="""
        class NotAMode:
            pass
        """,
    )
    monkeypatch.syspath_prepend(str(tmp_path))

    with pytest.raises(TypeError, match="did not resolve to a Mode subclass"):
        load_mode_class(f"{module_path}.NotAMode")


def test_all_repo_missions_load_with_explicit_schema():
    missions_dir = Path(__file__).resolve().parent.parent / "uav" / "missions"
    assert missions_dir.is_dir()

    for mission_path in sorted(missions_dir.glob("*.yaml")):
        mission_spec = load_mission_spec(mission_path)
        assert mission_spec.target in {"uav", "payload"}
        assert "start" in mission_spec.modes
        assert mission_spec.path == mission_path


def test_mission_root_uses_package_share(monkeypatch, tmp_path):
    package_share = tmp_path / "install_share"
    monkeypatch.setattr(
        mission_spec_module,
        "get_package_share_directory",
        lambda package_name: str(package_share),
    )

    assert mission_root() == package_share / "missions"
    assert (
        Path(mission_path_for_name("basic"))
        == package_share / "missions" / "basic.yaml"
    )


def test_mission_root_falls_back_to_source_tree(monkeypatch):
    def _raise_package_not_found(_package_name: str) -> str:
        raise mission_spec_module.PackageNotFoundError

    monkeypatch.setattr(
        mission_spec_module,
        "get_package_share_directory",
        _raise_package_not_found,
    )

    expected = Path(mission_spec_module.__file__).resolve().parent.parent / "missions"
    assert mission_root() == expected
    assert Path(mission_path_for_name("basic")) == expected / "basic.yaml"
