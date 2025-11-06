string frame_id
string traj_id
bool   is_local
bool   is_global

# Dimensions and polynomial info
int32 dims
int32 n_coeffs
int32 n_segments

# Segment durations (seconds)
float64[] segment_durations

# Flattened polynomial coefficients row-major ((n_coeffs*n_segments) * dims)
float64[] poly_coeffs

# Optional metadata
float64 plan_start_time_sec
float64 total_duration