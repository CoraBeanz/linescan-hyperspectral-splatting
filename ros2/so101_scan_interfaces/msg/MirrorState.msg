# Scan-mirror status, published on /scan_mirror/state ten times a second and on every change.
std_msgs/Header header

string state      # the ESP32's: disabled, idle, moving, stopping, homing, scanning or fault;
                  # "disconnected" while the bridge can't reach it
bool homed        # HOME has succeeded since the ESP32 last started or was disabled
bool busy         # moving, homing or running a sweep
uint32 sweep_id   # the running sweep, or the last one
int32 step        # the ESP32's position, microsteps
float64 angle     # rad, same convention as ScanLine.angle; NaN until homed
