import inspect
import tempfile
import unittest
from pathlib import Path

from uav.runtime.mission_spec import (
    load_mission_spec,
    load_mode_class,
    mission_path_for_name,
)


def _write_mission(contents: str) -> str:
    with tempfile.NamedTemporaryFile(
        "w", suffix=".yaml", delete=False, encoding="utf-8"
    ) as mission_file:
        mission_file.write(contents)
        return mission_file.name


class _FakeLogger:
    def debug(self, *args, **kwargs):
        pass

    def info(self, *args, **kwargs):
        pass

    def warn(self, *args, **kwargs):
        pass

    def warning(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass


class _FakeTimer:
    def cancel(self):
        pass


class _FakeClockTime:
    nanoseconds = 0


class _FakeClock:
    def now(self):
        return _FakeClockTime()


class _FakeNode:
    def __init__(self):
        self._logger = _FakeLogger()
        self._clock = _FakeClock()

    def get_logger(self):
        return self._logger

    def create_timer(self, *_args, **_kwargs):
        return _FakeTimer()

    def get_clock(self):
        return self._clock


class _FakeUAVVehicle:
    camera_offsets = (0.0, 0.0, 0.0)


class _FakePayloadVehicle:
    pass


def _vehicle_for_class_path(class_path: str):
    if ".modes.payload." in class_path:
        return _FakePayloadVehicle()
    return _FakeUAVVehicle()


def _instantiate_mode(class_path: str, params: dict):
    mode_class = load_mode_class(class_path)
    signature = inspect.signature(mode_class.__init__)
    kwargs = {}
    accepted = set()

    for name, parameter in signature.parameters.items():
        if name == "self":
            continue
        accepted.add(name)
        if name == "node":
            kwargs[name] = _FakeNode()
            continue
        if name == "vehicle":
            kwargs[name] = _vehicle_for_class_path(class_path)
            continue
        if name in params:
            kwargs[name] = params[name]
            continue
        if parameter.default is not inspect.Parameter.empty:
            continue
        raise AssertionError(
            f"Mode '{class_path}' requires unsupported constructor argument '{name}'."
        )

    unexpected = sorted(set(params) - (accepted - {"node", "vehicle"}))
    if unexpected:
        raise AssertionError(
            f"Mode '{class_path}' received unexpected params: {unexpected}"
        )

    return mode_class(**kwargs)


def _instantiate_all_modes(mission_spec) -> None:
    for mode_spec in mission_spec.modes.values():
        _instantiate_mode(mode_spec.class_path, mode_spec.params)


class MissionSpecLoadingTests(unittest.TestCase):
    def test_load_uav_mission_spec(self):
        mission_path = _write_mission(
            """
modes:
  start:
    class: uav.modes.uav.TakeoffMode
    params:
      target_altitude: 3.0
    transitions:
      complete: cruise
  cruise:
    class: uav.modes.uav.NavGPSMode
    params:
      target_waypoint: [1.0, 2.0, 3.0]
      acceptance_radius: 0.2
    transitions:
      complete: land
  land:
    class: uav.modes.uav.LandingMode
"""
        )

        mission_spec = load_mission_spec(mission_path)
        _instantiate_all_modes(mission_spec)

        self.assertEqual(mission_spec.target, "uav")
        self.assertTrue(mission_spec.is_uav)
        self.assertFalse(mission_spec.is_payload)
        self.assertEqual(mission_spec.vision_nodes, ())
        self.assertEqual(mission_spec.path, Path(mission_path))

        start_mode = mission_spec.modes["start"]
        self.assertEqual(start_mode.class_path, "uav.modes.uav.TakeoffMode")
        self.assertEqual(start_mode.params["target_altitude"], 3.0)
        self.assertEqual(start_mode.transitions["complete"], "cruise")

    def test_load_payload_mission_spec(self):
        mission_path = _write_mission(
            """
modes:
  start:
    class: uav.modes.payload.PayloadDriveToAprilTagMode
    params:
      target_tag_id: 1
      approach_speed: 0.2
"""
        )

        mission_spec = load_mission_spec(mission_path)
        _instantiate_all_modes(mission_spec)

        self.assertEqual(mission_spec.target, "payload")
        self.assertTrue(mission_spec.is_payload)
        self.assertFalse(mission_spec.is_uav)
        self.assertEqual(mission_spec.vision_nodes, ("PayloadAprilTagNode",))

        start_mode = mission_spec.modes["start"]
        self.assertEqual(
            start_mode.class_path, "uav.modes.payload.PayloadDriveToAprilTagMode"
        )
        self.assertEqual(start_mode.params["target_tag_id"], 1)
        self.assertEqual(start_mode.params["approach_speed"], 0.2)

    def test_load_payload_apriltag_approach_mission_spec(self):
        mission_path = _write_mission(
            """
modes:
  start:
    class: uav.modes.payload.PayloadAprilTagApproachMode
    params:
      tag_id: 0
      tag_size_m: 0.0508
      max_forward_speed: 0.2
      forward_gain: 0.5
      angular_gain: 0.003
      yaw_gain: 0.65
      stop_distance_m: 0.07
"""
        )

        mission_spec = load_mission_spec(mission_path)
        _instantiate_all_modes(mission_spec)

        self.assertEqual(mission_spec.target, "payload")
        self.assertEqual(mission_spec.vision_nodes, ("PayloadAprilTagNode",))

        start_mode = mission_spec.modes["start"]
        self.assertEqual(
            start_mode.class_path, "uav.modes.payload.PayloadAprilTagApproachMode"
        )
        self.assertEqual(start_mode.params["tag_id"], 0)
        self.assertEqual(start_mode.params["stop_distance_m"], 0.07)

    def test_mode_derived_metadata_rejects_mixed_targets(self):
        mission_path = _write_mission(
            """
modes:
  start:
    class: uav.modes.uav.TakeoffMode
    transitions:
      complete: payload
  payload:
    class: uav.modes.payload.PayloadDriveToAprilTagMode
"""
        )

        with self.assertRaisesRegex(ValueError, "mixes incompatible mode targets"):
            load_mission_spec(mission_path)

    def test_target_key_is_rejected(self):
        mission_path = _write_mission(
            """
target: uav
modes:
  start:
    class: uav.modes.uav.TakeoffMode
"""
        )

        with self.assertRaisesRegex(ValueError, "unsupported top-level keys"):
            load_mission_spec(mission_path)

    def test_undefined_transition_target_is_rejected(self):
        mission_path = _write_mission(
            """
modes:
  start:
    class: uav.modes.uav.TakeoffMode
    transitions:
      complete: missing
"""
        )

        with self.assertRaisesRegex(ValueError, "transitions to undefined mode"):
            load_mission_spec(mission_path)

    def test_all_repo_missions_use_explicit_schema(self):
        missions_dir = Path(__file__).resolve().parent.parent / "uav" / "missions"
        self.assertTrue(missions_dir.is_dir())

        for mission_path in sorted(missions_dir.glob("*.yaml")):
            with self.subTest(mission=mission_path.name):
                mission_spec = load_mission_spec(mission_path)
                _instantiate_all_modes(mission_spec)

                self.assertIn(mission_spec.target, {"uav", "payload"})
                self.assertIn("start", mission_spec.modes)
                self.assertEqual(mission_spec.path, mission_path)

    def test_named_mission_path_resolves_inside_package(self):
        expected_path = (
            Path(__file__).resolve().parent.parent / "uav" / "missions" / "basic.yaml"
        )
        self.assertEqual(Path(mission_path_for_name("basic")), expected_path)


if __name__ == "__main__":
    unittest.main()
