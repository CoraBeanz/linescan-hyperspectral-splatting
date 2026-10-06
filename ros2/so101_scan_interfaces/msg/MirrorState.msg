# Scan-mirror status, published on /scan_mirror/state a few times a second and on every change.
std_msgs/Header header

bool homed        # the hall sensor has been found since power-up
bool busy         # a sweep (or homing) is running
uint32 sweep_id   # the running sweep, or the last one
int32 step        # microsteps from the home position
float64 angle     # rad, same convention as ScanLine.angle
