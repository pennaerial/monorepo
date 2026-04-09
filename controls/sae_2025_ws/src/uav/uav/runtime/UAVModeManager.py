from time import time

from px4_msgs.msg import VehicleStatus
from std_srvs.srv import Trigger

from uav.vehicles.AirframeClass import AirframeClass
from uav.vehicles.Multicopter import Multicopter
from uav.vehicles.VTOL import VTOL
from uav.modes.uav.LandingMode import LandingMode
from .ModeManager import ModeManager
from .mission_spec import MissionSpec


class UAVModeManager(ModeManager):
    """Mission manager for MAVLink/PX4 UAV vehicles."""

    def __init__(
        self,
        *,
        mission_spec: MissionSpec,
        debug: bool = False,
        servo_only: bool = False,
        vehicle_class: AirframeClass = AirframeClass.MULTICOPTER,
        camera_offsets=None,
        node_name: str = "mission",
    ) -> None:
        super().__init__(node_name)

        self.timer = None
        if not mission_spec.is_uav:
            raise ValueError(
                f"UAVModeManager requires a UAV mission spec, received target '{mission_spec.target}'."
            )

        camera_offsets = list(camera_offsets or [0.0, 0.0, 0.0])
        if len(camera_offsets) != 3:
            raise ValueError(
                f"'uav_camera_offsets' must have exactly 3 values. Received: {camera_offsets}"
            )

        self.servo_only = bool(servo_only)
        vehicle_class = AirframeClass.parse(vehicle_class)

        self.start_mission_trigger = self.create_service(
            Trigger, "/mode_manager/start_mission", self.start_mission
        )
        self.failsafe_trigger_service = self.create_service(
            Trigger, "/mode_manager/failsafe", self.trigger_failsafe
        )

        if vehicle_class == AirframeClass.VTOL:
            self.vehicle = VTOL(self, DEBUG=debug, camera_offsets=camera_offsets)
        else:
            self.vehicle = Multicopter(self, DEBUG=debug, camera_offsets=camera_offsets)

        self.get_logger().info("Mission Node has started.")
        self.setup_vision(list(mission_spec.vision_nodes))
        self.setup_modes(mission_spec)

    def start_mission(self, request, response):
        self.get_logger().info("MODE MANAGER | Starting Mission!")
        if self.timer is None:
            self.timer = self.create_timer(0.1, self.spin_once)
            response.success = True
            response.message = "Starting Mission!"
        else:
            response.success = False
            response.message = "Mission Already in Progress"
        return response

    def trigger_failsafe(self, request, response):
        self.get_logger().info("Failsafe triggered via service call")
        if self.vehicle is not None and hasattr(self.vehicle, "failsafe_trigger"):
            self.vehicle.failsafe_trigger = True
            self.vehicle.failsafe = (
                self.vehicle.failsafe_px4 or self.vehicle.failsafe_trigger
            )
        response.success = True
        response.message = "Failsafe triggered."
        return response

    def spin_once(self) -> None:
        current_time = time()
        if self.vehicle is None:
            return

        if self.active_mode is None:
            self.switch_mode("start")

        if self.vehicle.failsafe:
            if not self.vehicle.emergency_landing:
                self.vehicle.hover()
                self.get_logger().warn("Failsafe: Switching to AUTO_LOITER mode.")
                self.vehicle.emergency_landing = True
            if (
                self.vehicle.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
                or self.vehicle.arm_state != VehicleStatus.ARMING_STATE_ARMED
            ):
                self.vehicle.land()
                self.get_logger().warn("Failsafe: Initiating landing.")
            return

        if self.servo_only:
            self._run_active_mode(current_time)
            return

        if not self.vehicle.origin_set:
            self.vehicle.set_origin()

        if self.vehicle.arm_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().info(
                f"UAV is not armed. Current arm state: {self.vehicle.arm_state}"
            )
            if (
                self.active_mode is not None
                and isinstance(self.get_active_mode(), LandingMode)
                and self.vehicle.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND
            ):
                self.get_logger().info("Successfully Landed UAV")
                self.get_logger().info("Finishing Mission")
                self.destroy_node()
                return

            if self.vehicle.attempted_takeoff and self.active_mode is not None:
                self.get_logger().error(
                    "UAV disarmed unexpectedly after takeoff attempt. Terminating to prevent infinite cycle."
                )
                self.get_logger().error(
                    "This usually indicates preflight check failures or PX4 safety triggers."
                )
                self.destroy_node()
                return

            self.vehicle.arm()
            self.get_logger().info("Arming UAV")
            self.start_time = current_time
            return

        if self.vehicle.local_position is None or self.vehicle.global_position is None:
            return

        self.vehicle.publish_offboard_control_heartbeat_signal()
        self._run_active_mode(current_time)

        if self.vehicle.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            self.get_logger().info("Landing")

    def handle_mode_state(self, state: str) -> None:
        if state == "error":
            self.get_logger().error(
                f"Error in mode {self.active_mode}. Switching to failsafe."
            )
            if self.vehicle is not None:
                self.vehicle.failsafe = True
            return
        if state == "terminate":
            self.get_logger().info("Mission has completed.")
            self.destroy_node()
            return
        super().handle_mode_state(state)
