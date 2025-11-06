# OMPL oc::PathControl recording message

std_msgs/Header header
string frame_id

# flattened states [x,y,vx,vy] for each waypoint
float64[] states
uint32 state_dim

# flattened controls [ax,ay] per segment
float64[] controls
uint32 control_dim

# durations per segment (seconds)
float64[] durations

# sizes
uint32 num_states
uint32 num_segments
