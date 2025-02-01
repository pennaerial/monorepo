from cv.tracking import find_payload
from uav import VisionNode
from numpy import ndarray as np
from uav.srv import PayloadTracking

class PayloadTrackingNode(VisionNode):
    """
    A vision node that performs object tracking and recalibration.
    """
    
    def __init__(self):
        """
        Initialize the PayloadTrackingNode.
        """
        super().__init__('payload_tracking', PayloadTracking)
            
    def service_callback(self, request: PayloadTracking.Request, response: PayloadTracking.Response):
        self.camera_info
        altitude = request.gps["altitude"] if request.gps else None
        self.processed_frame = find_payload(self.curr_frame, self.camera_info, altitude)
        response.x, response.y, response.direction = self.processed_frame

        return response