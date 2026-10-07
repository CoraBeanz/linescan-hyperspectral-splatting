# When one frame of the spectrograph camera was exposed. line_camera publishes one on
# /line_camera/frame for every frame the sensor delivers, whether or not it is kept, and
# scan_mirror_bridge uses them to put the mirror's line ticks between exposures.
#
# The IMX219 has a rolling shutter: row r stops exposing when it is read out, at
# header.stamp + r x the line time, and started an exposure time before that. Only the rows
# that see the slit matter, so the window below covers those rows.
#
# header.stamp     ROS time at which the frame started (row 0 read out)
# header.frame_id  "spectrograph_optical_frame"
std_msgs/Header header

uint64 sequence                          # the driver's frame counter; a gap means frames were lost
builtin_interfaces/Time exposure_start   # the first row that matters starts exposing
builtin_interfaces/Time exposure_end     # the last row that matters is read out
float64 exposure                         # s
float64 gain                             # analogue gain, 1 = none
