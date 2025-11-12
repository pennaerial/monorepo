# Drone state estimate from Kalman Filter
std_msgs/Header header
float64[3] position          # NED position (m)
float64[3] velocity          # NED velocity (m/s)
float64[3] acceleration      # NED acceleration (m/s²)
float64[9] position_covariance     # 3x3 covariance matrix (flattened)
float64[9] velocity_covariance     # 3x3 covariance matrix (flattened)
float64 yaw                  # Current yaw angle (rad)
bool valid                   # Whether state estimate is valid