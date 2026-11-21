# All detections from one camera frame.
# header is copied from the camera image, so stamp and frame_id identify the exact source frame.
# An empty detections list means the frame was processed and nothing was detected.
std_msgs/Header header
uint32 image_width     # source image size in pixels; bounding boxes are in this coordinate space
uint32 image_height
Detection[] detections
