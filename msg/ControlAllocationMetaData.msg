uint64 timestamp                        # time since system start (microseconds)

float32[6] control_sp
float32[6] allocated_control

float32[16] f_x 	# 4 iterations, 4 agents
float32[16] f_y
float32[16] f_z

float32[16] t_x 	# 4 iterations, 4 agents
float32[16] t_y
float32[16] t_z
float32[16] t_min

int8[4] saturated_idx
float32[4] increment
