# Broadcast by each payload for multi-payload mission coordination.
# Sent continuously over the UDP bridge so peers stay aware of each other's
# state even across isolated ROS graphs.

builtin_interfaces/Time stamp       # time this heartbeat was generated
string vehicle_name                 # e.g. "payload_0"
uint32 seq                          # monotonically increasing sequence number

# --- Mission state ---
string active_mode                  # current mode name; empty if mission not started
bool mission_started                # true once the mode manager timer is running
bool is_error                       # true if the mode manager hit an error state

# --- Peer connectivity (this vehicle's view of the mesh) ---
string[] connected_peers            # peer vehicle names currently alive
string[] disconnected_peers         # peer vehicle names currently timed out / stale

# --- Perception ---
bool tag_visible                    # currently tracking an AprilTag
float32 tag_distance_m              # estimated distance to tag (0.0 if not visible)
float32 tag_bearing_deg             # bearing to tag rel. to forward (0.0 if not visible)

# --- Drive state ---
float32 drive_linear_mps            # current commanded linear velocity (m/s)
float32 drive_angular_radps         # current commanded angular velocity (rad/s)
