from unittest.mock import MagicMock

import yaml
from sim_interfaces.msg import SearchLocation
from uav.modes.NextPatchMode import NextPatchMode, NextPatchParams
from uav.vehicles.UAV import UAV
from vehicle_common.mode_loader import ModeRegistry
from vehicle_common.runtime.mission_loader import RuntimeMission


class DummyLocalPosition:
    def __init__(self, x=0.0, y=0.0, z=-5.0):
        self.x = x
        self.y = y
        self.z = z


class DummyUAV:
    def __init__(self):
        self.local_position = DummyLocalPosition()
        self.gps_origin = (47.39797, 8.54616, 100.0)
        self.published_setpoints = []

    def gps_to_local(self, target_gps):
        lat, lon, alt = target_gps
        # Simple mock conversion
        dx = (lat - self.gps_origin[0]) * 100000.0
        dy = (lon - self.gps_origin[1]) * 100000.0
        dz = -(alt - self.gps_origin[2])
        return (dx, dy, dz)

    def publish_position_setpoint(self, coordinate, relative=False, lock_yaw=False):
        self.published_setpoints.append((coordinate, lock_yaw))

    def distance_to_waypoint(self, coordinate_system, waypoint):
        if coordinate_system == "LOCAL":
            dx = self.local_position.x - waypoint[0]
            dy = self.local_position.y - waypoint[1]
            dz = self.local_position.z - waypoint[2]
            return (dx**2 + dy**2 + dz**2) ** 0.5
        return 0.0


def test_next_patch_mode_registration():
    registry = ModeRegistry.get()
    reg = registry.get_registered_mode("uav.NextPatchMode")
    assert reg.id == "uav.NextPatchMode"
    assert reg.mode_cls == NextPatchMode
    assert reg.params_cls == NextPatchParams
    assert UAV in reg.targets
    assert "complete" in reg.transition_labels
    assert "no_patches" in reg.transition_labels


def test_next_patch_params():
    params = NextPatchParams()
    assert params.altitude == 5.0
    assert params.margin == 1.0
    assert params.service_name == "/get_search_patches"

    custom = NextPatchParams(altitude=10.0, margin=2.5, service_name="custom_patches")
    assert custom.altitude == 10.0
    assert custom.margin == 2.5
    assert custom.service_name == "custom_patches"


def test_next_patch_mode_sequential_popping_and_transitions():
    mock_node = MagicMock()
    mock_vehicle = DummyUAV()
    params = NextPatchParams(altitude=6.0, margin=1.0)

    mode = NextPatchMode()
    mode.initialize(mock_node, mock_vehicle, params)

    patch1 = SearchLocation(latitude_deg=47.3980, longitude_deg=8.5462, radius_m=4.0)
    patch2 = SearchLocation(latitude_deg=47.3985, longitude_deg=8.5465, radius_m=4.0)

    # Simulate patches initialized after first focus
    mode.patches = [patch1, patch2]

    # Re-entering focus pops the first patch
    mode.on_enter()
    assert mode.current_patch == patch1
    assert len(mode.patches) == 1
    assert mode.target is not None
    assert mode.target[2] == -6.0

    # Before arriving at patch 1
    mock_vehicle.local_position = DummyLocalPosition(0.0, 0.0, -6.0)
    mode.on_update(0.1)
    assert len(mock_vehicle.published_setpoints) > 0
    assert mode.check_status() == "continue"

    # UAV arrives at patch 1
    mock_vehicle.local_position = DummyLocalPosition(mode.target[0], mode.target[1], mode.target[2])
    assert mode.check_status() == "complete"

    # Exit mode and re-enter (e.g. from state transition loop)
    mode.on_exit()
    mode.on_enter()

    # Now popped patch 2
    assert mode.current_patch == patch2
    assert len(mode.patches) == 0
    assert mode.target is not None

    # Arrive at patch 2
    mock_vehicle.local_position = DummyLocalPosition(mode.target[0], mode.target[1], mode.target[2])
    assert mode.check_status() == "complete"

    # Exit and re-enter when no patches remain
    mode.on_exit()
    mode.on_enter()
    assert mode.current_patch is None
    assert mode.check_status() == "no_patches"


def test_next_patch_mode_async_fetch():
    mock_node = MagicMock()
    mock_client = MagicMock()
    mock_node.create_client.return_value = mock_client
    mock_client.service_is_ready.return_value = True

    mock_future = MagicMock()
    mock_future.done.return_value = True
    mock_response = MagicMock()
    mock_response.ready = True
    mock_response.target_tag_id = 42
    p1 = SearchLocation(latitude_deg=47.3980, longitude_deg=8.5462, radius_m=4.0)
    mock_response.search_locations = [p1]
    mock_future.result.return_value = mock_response
    mock_client.call_async.return_value = mock_future

    mock_vehicle = DummyUAV()
    params = NextPatchParams(altitude=5.0)

    mode = NextPatchMode()
    mode.initialize(mock_node, mock_vehicle, params)

    assert mode.patches is None
    mode.on_enter()
    assert mock_client.call_async.called

    mode.on_update(0.1)
    assert mode.patches == []  # 1 patch was loaded and immediately popped
    assert mode.current_patch == p1
    assert mode.target_tag_id == 42
    assert mode.target is not None


def test_runtime_mission_parses_next_patch_mode():
    mission_yaml = """
modes:
  start:
    mode: uav.NextPatchMode
    params:
      altitude: 7.5
      margin: 1.5
    transitions:
      complete: start
      no_patches: land
  land:
    mode: uav.LandingMode
"""
    mission = RuntimeMission.model_validate(yaml.safe_load(mission_yaml))
    assert "start" in mission.modes
    start_mode = mission.modes["start"]
    assert start_mode.mode == "uav.NextPatchMode"
    assert start_mode.transitions == {"complete": "start", "no_patches": "land"}
    assert start_mode._validated_params.altitude == 7.5
    assert start_mode._validated_params.margin == 1.5
