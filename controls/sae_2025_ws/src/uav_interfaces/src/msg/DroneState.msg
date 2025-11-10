# DroneState.msg
# Estimated drone state with position, velocity, and covariance

geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64[36] covariance  # 6x6 covariance matrix (position + velocity)
builtin_interfaces/Time timestamp

