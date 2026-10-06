# One scan line: the scan mirror has settled at the angle this line is taken at.
# scan_mirror_bridge publishes one on /scan_mirror/line for every line the ESP32 reports.
#
# header.stamp     ROS time at which the mirror settled on this line. The ESP32 stamps the
#                  line with its own microsecond clock; the bridge maps that clock onto ROS
#                  time from ping round trips, so the arm pose can be looked up at the same
#                  instant.
# header.frame_id  "scan_mirror_link"
std_msgs/Header header

uint32 sweep_id   # from the StartSweep response
uint32 index      # 0 .. n_lines - 1 within the sweep
int32 step        # microsteps from the home (hall sensor) position
float64 angle     # rad; 0 is the mirror's 45 deg rest, right-handed about the head's +X (shaft) axis
bool last         # true on the sweep's final line
