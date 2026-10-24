# Stamp is the measurement epoch in the selected ROS clock; frame is the anchor frame.
std_msgs/Header header
string anchor_id
string tag_id
float64 range_m
# Negative means unavailable; never replace unknown uncertainty with zero.
float64 range_std_m
# Normalized diagnostic score [0,1]; -1 means unavailable, not a probability.
float32 quality
bool valid
uint8 REASON_OK=0
uint8 REASON_DROPOUT=1
uint8 REASON_NONFINITE=2
uint8 REASON_OUT_OF_RANGE=3
uint8 REASON_STALE=4
uint8 REASON_CLOCK_UNSYNCED=5
uint8 REASON_NLOS=6
uint8 REASON_UNCALIBRATED=7
uint8 reason
bool nlos
# False when the source cannot classify NLOS.
bool nlos_known
