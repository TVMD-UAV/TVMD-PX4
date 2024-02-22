uint64 timestamp		# time since system start (microseconds)

# For quaternion-based attitude control
float32[4] q_d			# Desired quaternion for quaternion control

# For clarification: For multicopters thrust_body[0] and thrust[1] are usually 0 and thrust[2] is the negative throttle demand.
# For fixed wings thrust_x is the throttle demand and thrust_y, thrust_z will usually be zero.
float32[3] sat_thrust_body	# Normalized thrust command in body NED frame [-1,1]
float32[3] coplan_vector	# Coplanar vector (k) in body NWU frame

float32 theta			# Deflection attitude angle in radians
uint8 is_feasible		# 1 if the reference attitude is feasible, 0 otherwise
uint8 bisection_count		# Number of bisection iterations

# TOPICS attitude_planner_meta_data
