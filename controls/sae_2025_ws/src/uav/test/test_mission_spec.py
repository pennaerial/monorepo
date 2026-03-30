import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from uav.runtime.mission_spec import load_mission_spec, mission_path_for_name


def _write_mission(tmpdir: str, content: str) -> Path:
    mission_path = Path(tmpdir) / "mission.yaml"
    mission_path.write_text(content, encoding="utf-8")
    return mission_path


def _fake_mode(target: str, vision_nodes: tuple[str, ...] = ()):
    class _FakeMode:
        mission_target = target

        @classmethod
        def required_vision_node_names(cls) -> tuple[str, ...]:
            return vision_nodes

    return _FakeMode


MODE_METADATA = {
    "uav.modes.uav.TakeoffMode": _fake_mode("uav"),
    "uav.modes.uav.NavGPSMode": _fake_mode("uav"),
    "uav.modes.uav.LandingMode": _fake_mode("uav"),
    "uav.modes.uav.TransitionMode": _fake_mode("uav"),
    "uav.modes.uav.PayloadPickupMode": _fake_mode("uav", ("PayloadTrackingNode",)),
    "uav.modes.uav.PayloadDropoffMode": _fake_mode("uav", ("PayloadTrackingNode",)),
    "uav.modes.uav.ServoDropoffMode": _fake_mode("uav"),
    "uav.modes.uav.WaypointMission": _fake_mode("uav"),
    "uav.modes.payload.PayloadDriveToAprilTagMode": _fake_mode(
        "payload", ("PayloadAprilTagNode",)
    ),
    "uav.modes.payload.PayloadColorOrbitToRearMode": _fake_mode(
        "payload", ("PayloadColorOrbitNode",)
    ),
}


def _resolve_fake_mode(class_path: str):
    try:
        return MODE_METADATA[class_path]
    except KeyError as exc:
        raise AssertionError(f"Missing fake metadata for {class_path}") from exc


class MissionSpecTests(unittest.TestCase):
    @patch("uav.runtime.mission_spec.load_mode_class", side_effect=_resolve_fake_mode)
    def test_load_uav_mission_spec(self, _resolver) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            mission_path = _write_mission(
                tmpdir,
                """
modes:
  start:
    class: uav.modes.uav.TakeoffMode
    params:
      takeoff_type: vertical
    transitions:
      complete: track
  track:
    class: uav.modes.uav.PayloadPickupMode
    params:
      color: yellow
""".strip(),
            )

            mission_spec = load_mission_spec(mission_path)

        self.assertTrue(mission_spec.is_uav)
        self.assertEqual(mission_spec.vision_nodes, ("PayloadTrackingNode",))
        self.assertEqual(mission_spec.modes["start"].params["takeoff_type"], "vertical")
        self.assertEqual(mission_spec.modes["track"].params["color"], "yellow")

    @patch("uav.runtime.mission_spec.load_mode_class", side_effect=_resolve_fake_mode)
    def test_load_payload_mission_spec(self, _resolver) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            mission_path = _write_mission(
                tmpdir,
                """
modes:
  start:
    class: uav.modes.payload.PayloadDriveToAprilTagMode
    params:
      tag_size_m: 0.1
      stop_distance_m: 0.05
""".strip(),
            )

            mission_spec = load_mission_spec(mission_path)

        self.assertTrue(mission_spec.is_payload)
        self.assertEqual(mission_spec.vision_nodes, ("PayloadAprilTagNode",))
        self.assertEqual(mission_spec.modes["start"].params["stop_distance_m"], 0.05)

    @patch("uav.runtime.mission_spec.load_mode_class", side_effect=_resolve_fake_mode)
    def test_mode_derived_metadata_rejects_mixed_targets(self, _resolver) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            mission_path = _write_mission(
                tmpdir,
                """
modes:
  start:
    class: uav.modes.uav.TakeoffMode
    transitions:
      complete: dock
  dock:
    class: uav.modes.payload.PayloadDriveToAprilTagMode
""".strip(),
            )

            with self.assertRaisesRegex(ValueError, "mixes incompatible mode targets"):
                load_mission_spec(mission_path)

    @patch("uav.runtime.mission_spec.load_mode_class", side_effect=_resolve_fake_mode)
    def test_target_key_is_rejected(self, _resolver) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            mission_path = _write_mission(
                tmpdir,
                """
target: payload
modes:
  start:
    class: uav.modes.payload.PayloadDriveToAprilTagMode
""".strip(),
            )

            with self.assertRaisesRegex(ValueError, "unsupported top-level keys"):
                load_mission_spec(mission_path)

    @patch("uav.runtime.mission_spec.load_mode_class", side_effect=_resolve_fake_mode)
    def test_undefined_transition_target_is_rejected(self, _resolver) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            mission_path = _write_mission(
                tmpdir,
                """
modes:
  start:
    class: uav.modes.uav.TakeoffMode
    transitions:
      complete: missing
""".strip(),
            )

            with self.assertRaisesRegex(ValueError, "undefined mode"):
                load_mission_spec(mission_path)

    @patch("uav.runtime.mission_spec.load_mode_class", side_effect=_resolve_fake_mode)
    def test_all_repo_missions_use_explicit_schema(self, _resolver) -> None:
        mission_dir = Path(__file__).resolve().parents[1] / "uav" / "missions"
        mission_paths = sorted(mission_dir.glob("*.yaml"))

        self.assertTrue(mission_paths)

        for mission_path in mission_paths:
            mission_spec = load_mission_spec(mission_path)
            self.assertEqual(mission_spec.path, mission_path)
            self.assertIn(mission_spec.target, {"uav", "payload"})
            self.assertIn("start", mission_spec.modes)

    def test_named_mission_path_resolves_inside_package(self) -> None:
        mission_path = Path(mission_path_for_name("basic"))
        self.assertEqual(mission_path.name, "basic.yaml")
        self.assertTrue(mission_path.exists())


if __name__ == "__main__":
    unittest.main()
