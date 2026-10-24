# Header frame is the parent coordinate frame; stamp is calibration time.
std_msgs/Header header
string anchor_id
string frame_id
geometry_msgs/Point position
# Metres above the calibrated floor; position.z is relative to header.frame_id.
float64 height_m
