# Single AprilTag detection
int32 tag_id            # AprilTag ID
float64 x               # X pixel coordinate of tag center
float64 y               # Y pixel coordinate of tag center
float64 distance        # Estimated distance to tag (meters)
float64 yaw             # Yaw angle relative to camera (radians)
float64 pitch           # Pitch angle relative to camera (radians)
float64 roll            # Roll angle relative to camera (radians)
float64[3] position     # 3D position [x, y, z] in camera frame (meters)
