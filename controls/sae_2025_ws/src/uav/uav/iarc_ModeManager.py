#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from time import time
from uav import UAV
from uav.autonomous_modes import Mode, LandingMode
import yaml
import importlib
import inspect
import ast
from px4_msgs.msg import VehicleStatus
from std_msgs.msg import String

VISION_NODE_PATH = 'uav.vision_nodes'

class ModeManager(Node):    
    """
    A ROS 2 node for managing UAV modes and mission logic.
    """
    def __init__(self, mode_map: str, vision_nodes: str, camera_offsets, DEBUG=False, servo_only=False, safe_test=False, vehicle_id: int = 0) -> None:
        self.vehicle_id = int(vehicle_id)
        super().__init__(f'mission_node_{self.vehicle_id}')
        self.timer = self.create_timer(0.1, self.spin_once)
        self.modes = {}
        self.transitions = {}
        self.active_mode = None
        self.last_update_time = time()
        self.start_time = self.last_update_time
        self.uav = UAV(self, DEBUG=DEBUG, camera_offsets=camera_offsets)
        self.get_logger().info("Mission Node has started!")
        self.setup_vision(vision_nodes)
        self.setup_modes(mode_map)
        self.servo_only = servo_only
        self.safe_test = safe_test

        # Publish current autonomous mode for P2P sharing
        self._mode_pub = self.create_publisher(String, f'/vehicle_{self.vehicle_id}/current_mode', 10)

        # safe_test command/status IO (used by uav.autonomous_modes.iarc_SafetestModes)
        if self.safe_test:
            if not hasattr(self, '_safe_test_cmd'):
                self._safe_test_cmd = None
                self._safe_test_cmd_time = 0.0

            cmd_topic = f'/safe_test/vehicle_{self.vehicle_id}/command'
            status_topic = f'/safe_test/vehicle_{self.vehicle_id}/status'

            if not hasattr(self, '_safe_test_status_pub'):
                self._safe_test_status_pub = self.create_publisher(String, status_topic, 10)

            if not hasattr(self, '_safe_test_cmd_sub'):
                def _cb(msg: String) -> None:
                    cmd = (msg.data or '').strip().lower()
                    if not cmd:
                        return
                    self._safe_test_cmd = cmd
                    self._safe_test_cmd_time = time()

                self._safe_test_cmd_sub = self.create_subscription(String, cmd_topic, _cb, 10)
    
    def get_active_mode(self) -> Mode:
        """
        Get the active mode.

        Returns:
            Mode: The active mode.
        """
        return self.modes[self.active_mode]
        
    def setup_vision(self, vision_nodes: str) -> None:
        """
        Setup the vision node for this mode.

        Args:
            mode (Mode): The mode to setup vision for.
            vision (str): The comma-separated string of vision nodes to setup for this mode.
        """
        if vision_nodes.strip() == '':
            return
        module = importlib.import_module(VISION_NODE_PATH)
        for vision_node in vision_nodes.split(','):
            vision_class = getattr(module, vision_node)
            if vision_class.service_name() not in self.uav.vision_clients:
                client = self.create_client(vision_class.srv, vision_class.service_name())
                while not client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info(f"Service {vision_class.service_name()} not available, waiting again...")
                self.uav.vision_clients[vision_class.service_name()] = client
    
    def initialize_mode(self, mode_path: str, params: dict) -> Mode:
        module_name, class_name = mode_path.rsplit('.', 1)
        module = importlib.import_module(module_name)
        mode_class = getattr(module, class_name)

        signature = inspect.signature(mode_class.__init__)
        args = {}

        for name, param in signature.parameters.items():
            if name == 'self':
                continue
            if name in params:
                param_value = params[name]
                if param.annotation in (str, inspect.Parameter.empty) or name in ('node', 'uav'):
                    args[name] = param_value
                else:
                    try:
                        args[name] = ast.literal_eval(param_value)
                    except (ValueError, SyntaxError):
                        raise ValueError(f"Parameter '{name}' must be a valid literal for mode '{mode_path}'. Received: {param_value}")
            elif param.default != inspect.Parameter.empty:
                args[name] = param.default
            else:
                raise ValueError(f"Missing required parameter '{name}' for mode '{mode_path}'")

        return mode_class(**args)
    
    def setup_modes(self, mode_map: str) -> None:
        """
        Setup the modes for the mission node.

        Args:
            mode_yaml (dict): A dictionary mapping mode names to instances of the mode.
        """
        mode_yaml = self.load_yaml_to_dict(mode_map)

        assert 'start' in mode_yaml, "No start mode defined in mode map."

        for mode_name in mode_yaml.keys():
            mode_info = mode_yaml[mode_name]

            mode_path = mode_info['class']

            params = mode_info.get('params', {}) | {'node': self, 'uav': self.uav}
            mode = self.initialize_mode(mode_path, params)
            self.add_mode(mode_name, mode)
            self.transitions[mode_name] = mode_info.get('transitions', {})
        
    def add_mode(self, mode_name: str, mode_instance: Mode) -> None:
        """
        Register a mode to the mission node.

        Args:
            mode_name (str): Name of the mode.
            mode_instance (Mode): An instance of the mode.
        """
        self.modes[mode_name] = mode_instance
        self.get_logger().info(f"Mode {mode_name} registered.")

    def transition(self, state: str) -> str:
        """
        Transition to the next mode based on the current state.

        Args:
            state (str): The current state of the mode.

        Returns:
            str: The name of the next mode to transition to.
        """
        self.get_logger().info(f"Transitioning from {self.active_mode} based on state {state}.")
        return self.transitions[self.active_mode][state]

    def switch_mode(self, mode_name: str) -> None:
        """
        Switch to a new mode.

        Args:
            mode_name (str): Name of the mode to activate.
        """
        if self.active_mode:
            self.get_active_mode().deactivate()

        if mode_name in self.modes:
            self.active_mode = mode_name
            self.get_active_mode().activate()
            # Publish mode change for P2P network
            mode_msg = String()
            mode_msg.data = mode_name
            self._mode_pub.publish(mode_msg)
        else:
            self.get_logger().error(f"Mode {mode_name} not found.")

    def spin_once(self) -> None:
        """
        Execute one spin cycle of the node, updating the active mode.
        """
        current_time = time()
        if self.uav.failsafe:
            if not self.uav.emergency_landing:
                self.uav.hover()
                self.get_logger().info("Failsafe: Switching to AUTO_LOITER mode.")
                self.uav.emergency_landing = True
            if self.uav.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER or self.uav.arm_state != VehicleStatus.ARMING_STATE_ARMED:
                self.uav.land()  # Initiate the landing procedure.
                self.get_logger().info("Failsafe: Initiating landing.")
            return
        if self.servo_only:
            if self.active_mode is None:
                self.switch_mode('start')
            if self.active_mode:
                time_delta = current_time - self.last_update_time
                self.last_update_time = current_time
                try:
                    self.get_active_mode().update(time_delta)
                except Exception as e:
                    self.get_logger().error(f"Error in mode {self.active_mode}: {e}")
                    self.uav.failsafe = True
                    return
                state = self.get_active_mode().check_status()
                if state == 'error':
                    self.get_logger().error(f"Error in mode {self.active_mode}. Switching to failsafe.")
                    self.uav.failsafe = True
                elif state == 'terminate':
                    self.get_logger().info(f"Mission has completed.")
                    self.destroy_node()
                elif state != 'continue':
                    self.switch_mode(self.transition(state))
        elif self.safe_test:
            # safe_test runs the Mode graph without the normal flight-state gating.
            # The modes themselves are expected to react to /safe_test/vehicle_<id>/command.
            if not self.uav.origin_set:
                self.uav.set_origin()

            if self.active_mode is None:
                self.switch_mode('start')

            time_delta = current_time - self.last_update_time
            self.last_update_time = current_time

            try:
                self.get_active_mode().update(time_delta)
            except Exception as e:
                self.get_logger().error(f"Error in mode {self.active_mode}: {e}")
                self.uav.failsafe = True
                return

            state = self.get_active_mode().check_status()
            if state == 'error':
                self.get_logger().error(f"Error in mode {self.active_mode}. Switching to failsafe.")
                self.uav.failsafe = True
            elif state == 'terminate':
                self.get_logger().info("Mission has completed.")
                self.destroy_node()
            elif state != 'continue':
                self.switch_mode(self.transition(state))
        else:
            if not self.uav.origin_set:
                self.uav.set_origin()
            if self.uav.arm_state != VehicleStatus.ARMING_STATE_ARMED:
                if self.active_mode is not None and self.get_active_mode() == LandingMode and self.uav.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                    self.get_logger().info(f"Succesfully Landed UAV")
                    self.get_logger().info(f"Finishing Mission")
                    self.destroy_node()
                self.uav.arm()
                self.get_logger().info(f"Arming UAV")
                self.start_time = current_time
            if self.uav.local_position is None or self.uav.global_position is None: # Need to wait for the uav to be ready
                return
            if not self.uav.attempted_takeoff:
                self.uav.takeoff()
                self.get_logger().info("Attempting takeoff")
                self.start_time = current_time # Reset the start time because we will starting publishing heartbeat
                return
            self.uav.publish_offboard_control_heartbeat_signal()
            if self.uav.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                self.get_logger().info("Taking off")
            elif current_time - self.start_time < 1:
                return
            elif self.uav.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                self.get_logger().info("Takeoff Complete. Engaging Offboard Mode")
                self.uav.engage_offboard_mode()
            elif self.uav.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                # Start the mission
                if self.active_mode is None:
                    self.switch_mode('start')
                if self.active_mode and self.uav.flight_check:
                    time_delta = current_time - self.last_update_time
                    self.last_update_time = current_time
                    try:
                        self.get_active_mode().update(time_delta)
                    except Exception as e:
                        self.get_logger().error(f"Error in mode {self.active_mode}: {e}")
                        self.uav.failsafe = True
                        return
                    state = self.get_active_mode().check_status()
                    if state == 'error':
                        self.get_logger().error(f"Error in mode {self.active_mode}. Switching to failsafe.")
                        self.uav.failsafe = True
                    elif state == 'terminate':
                        self.get_logger().info(f"Mission has completed.")
                        self.destroy_node()
                    elif state != 'continue':
                        self.switch_mode(self.transition(state))
            elif self.uav.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND: # nav_state will/should change when LandingMode is spun
                self.get_logger().info("Landing")
            else:
                self.get_logger().info(f"Self.nav_state: {self.uav.nav_state}")
    def spin(self):
        """
        Run the mission node loop.
        """
        self.switch_mode('start')
        try:
            while rclpy.ok():
                self.spin_once()
        except KeyboardInterrupt:
            self.get_logger().info("Mission Node shutting down.")
        finally:
            rclpy.shutdown()

    def load_yaml_to_dict(self, filename: str):
        """
        Load a yaml file into a dictionary.

        Args:
            filename (str): The path to the yaml file.

        Returns:
            dict: The yaml file as a dictionary.
        """
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        return data
    

if __name__ == '__main__':
    mission_node = ModeManager('test_mode_manager')
    print(mission_node.modes)
    print(mission_node.transitions)
    print(mission_node.active_mode)
    # mission_node.spin()
    
    mission_node.transition('continue')
    print('transition continue')
    print(mission_node.modes)
    print(mission_node.transitions)
    print(mission_node.active_mode)
    
    mission_node.transition('continue')
    print('transition continu 2')
    print(mission_node.modes)
    print(mission_node.transitions)
    print(mission_node.active_mode)
     
