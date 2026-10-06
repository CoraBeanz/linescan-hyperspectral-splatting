# One scan line: the scan mirror holds still at the angle this line is taken at.
# scan_mirror_bridge publishes one on /scan_mirror/line for every line the ESP32 reports.
#
# header.stamp     ROS time at which the mirror settled on this line (when the ESP32 pulses its
#                  TRIG pin). The ESP32 stamps each line on its own microsecond clock; the bridge
#                  maps that clock onto ROS time from ping round trips, so the arm pose can be
#                  looked up at the same instant.
# header.frame_id  "scan_mirror_link"
std_msgs/Header header

builtin_interfaces/Time hold_until  # ROS time at which the mirror moves on to the next line
                                    # (rounded early by the ESP32's 50 us timer step). A camera
                                    # frame belongs to this line if its whole exposure falls
                                    # between header.stamp and hold_until.
uint32 sweep_id   # from the StartSweep response
uint32 index      # 0 .. n_lines - 1 within the sweep
int32 step        # the ESP32's position in microsteps; 0 is the 45 deg rest once homed
float64 angle     # rad; 0 is the mirror's 45 deg rest, right-handed about the head's +X (shaft) axis
bool settled      # false if the mirror was still moving when the next line was due (a longer
                  # line_period fixes it); header.stamp is then when the line was due
bool last         # true on the sweep's final line
