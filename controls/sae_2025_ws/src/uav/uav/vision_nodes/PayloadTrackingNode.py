from cv.track import track, conversion, confidence
from cv.recalibrate import recalibrate
from cv.threshold import threshold
from cv.tracking import find_payload
from uav import VisionNode
from numpy import ndarray as np
from uav.srv import PayloadTracking


class PayloadTrackingNode(VisionNode):
    """
    A vision node that performs object tracking and recalibration.
    """
    
    def __init__(self, image_topic: str = '/camera', payload_color: np.ndarray = np.array([0, 0, 0])):
        """
        Initialize the PayloadTrackingNode.
        image_topic (str): The name of the image topic to subscribe to.
        payload_color (np.ndarray): The color of the payload to track, in HSV format.
        """
        super().__init__('payload_tracking_node', image_topic)
        
        self.initialize_service(PayloadTracking, '/payload_tracking')
        self.get_logger().info(f"PayloadTracking service has started, subscribing to {image_topic}.")
        pass
    
    def service_callback(self, request: PayloadTracking.Request, response: PayloadTracking.Response):
        self.processed_fram = find_payload(self.curr_frame)
        response.x = self.processed_frame[0]
        response.y = self.processed_frame[1]
        response.direction = self.processed_frame[2]

        return response