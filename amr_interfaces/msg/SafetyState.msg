uint8 CLEAR=0
uint8 WARNING=1
uint8 PROTECTIVE_STOP=2
uint8 EMERGENCY_STOP=3

std_msgs/Header header

# Current safety state, one of the constants above
uint8 state

# Factor 0.0 .. 1.0 applied to the commanded linear velocity
float32 speed_scale

# Distance from the footprint edge to the closest scan point in metres, inf if none
float32 closest_obstacle_distance

# Active field lengths ahead of the footprint in metres
float32 protective_field_length
float32 warning_field_length

# Human-readable cause of the current state
string reason
