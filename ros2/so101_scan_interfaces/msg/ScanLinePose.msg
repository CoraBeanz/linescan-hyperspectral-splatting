# Where the instrument was for one scan line. The scan_sweep node publishes one on
# /scan/line_pose for every ScanLine it logs, and writes the same fields to lines.csv.
#
# header.stamp     the ScanLine's stamp
# header.frame_id  the frame both poses are in (base_link)
std_msgs/Header header

uint32 viewpoint                    # index of the viewpoint in the scan plan
uint32 sweep_id
uint32 index                        # line index within the sweep
float64 mirror_angle                # rad, as in ScanLine.angle

geometry_msgs/Pose head_pose        # scan_head_link at the line's time
geometry_msgs/Pose line_camera_pose # line_camera_optical_frame: the objective seen through the mirror

string[] joint_names
float64[] joint_positions           # arm joints interpolated to the line's time, rad
