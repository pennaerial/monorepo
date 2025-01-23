from uav import Mode, UAV
from rclpy.node import Node
from uav.src import PayloadTracking

class LowerPayloadMode(Mode):
    """
    A mode for lowering the payload.
    """

    def __init__(self, node: Node):
        """
        Initialize the LowerPayload.

        Args:
            node (Node): ROS 2 node managing the UAV.
        """
        super().__init__(node, {
            '/payload_tracking': PayloadTracking
        })
        self.obstacle_detected = False
        self.current_pose = None
        self.payload_pose = None

    def service_response_callback(self, future: Future):
        response = future.result()
        if response:
            self.get_logger().info(f'Response: {response}')
            
            self.payload_location = (response.x, response.y, response.direction)

        else:
            self.get_logger().error('Service call failed')

    def on_enter(self):
        """
        Initialize the navigation mode by preparing for payload lowering.
        """
        self.log("Entering Payload Lowering Mode.")

        self.payload_location = None
        if not self.uav.arm():
            self.log("Failed to arm the UAV.")
        else:
            self.log("UAV armed successfully. Starting navigation.")

    def on_exit(self):
        """
        Stop navigation and clean up.
        """
        self.log("Exiting Payload Lowering Mode.")
        self.uav.land()

    def on_update(self):
        """
        Periodic logic for lowering payload and handling obstacles.
        """

        
        if self.current_pose is None or self.payload_location is None:
            self.log("Current pose not available yet.")
        else:
            self.uav.go_to(self.payload_location)
