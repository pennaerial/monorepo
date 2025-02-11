import rclpy
from ModeManager import ModeManager
from autonomous_modes import NavigateToGPSMode
from autonomous_modes import LowerPayloadMode

def main(args=None):
    rclpy.init(args=args)
    # Create the offboard control node
    manager = ModeManager()
    manager.add_mode("nav_to_gps", NavigateToGPSMode(manager, manager.uav, (0, 0, 0)))
    manager.add_mode("lower_payload", LowerPayloadMode(manager, manager.uav))
    manager.switch_mode("nav_to_gps")
    manager.spin()
    manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()