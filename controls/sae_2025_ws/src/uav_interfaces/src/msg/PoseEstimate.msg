# PoseEstimate.msg
# Noisy pose estimate from vision system

geometry_msgs/Point position
geometry_msgs/Vector3 rotation
float64[36] covariance  # 6x6 covariance matrix (position + rotation)
builtin_interfaces/Time timestamp

