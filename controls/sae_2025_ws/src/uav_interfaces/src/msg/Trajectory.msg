# Polynomial trajectory for drone to follow
std_msgs/Header header
float64 duration             # Total trajectory duration (s)
float64[] waypoint_times     # Timestamps for each waypoint (s from start)
geometry_msgs/Point[] waypoints      # Position waypoints (NED)
geometry_msgs/Vector3[] velocities   # Velocity at each waypoint (m/s)
geometry_msgs/Vector3[] accelerations # Acceleration at each waypoint (m/s²)
float64[] yaws               # Yaw angles at each waypoint (rad)
uint32 polynomial_order      # Order of polynomial (e.g., 5, 7, 9)
bool valid                   # Whether trajectory is valid