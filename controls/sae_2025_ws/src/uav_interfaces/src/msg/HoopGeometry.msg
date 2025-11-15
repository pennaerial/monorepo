# Hoop cylinder geometry for collision avoidance
std_msgs/Header header
float64[3] center_position   # Hoop center in NED (m)
float64[3] normal_vector     # Normal vector of hoop plane (unit vector)
float64 radius               # Hoop radius (m)
float64 thickness            # Hoop thickness/depth (m) - for collision cylinder
uint32 hoop_id               # Unique identifier for this hoop
bool passed                  # Whether drone has passed through this hoop