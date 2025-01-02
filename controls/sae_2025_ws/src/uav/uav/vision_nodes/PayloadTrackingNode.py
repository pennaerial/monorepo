from cv.track import track
from cv.recalibrate import recalibrate
from cv.threshold import threshold
from cv.track import conversion, confidence
from uav import VisionNode
from numpy import ndarray as np


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
        # TODO: Initialize any additional attributes
        pass

    def process_frame(self, frame: np.ndarray) -> None:
        """
        Process a single frame for tracking and recalibration.

        Args:
            frame (np.ndarray): The image frame to process.
        """
        pass