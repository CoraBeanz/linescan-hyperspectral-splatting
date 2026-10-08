# The scan arm's safety state, from the servo driver (so101_scan_hardware). Published on
# /arm_safety/state ten times a second and on every change; the last one is latched, so a node
# that starts later gets it at once.
#
# The driver stops the arm when a servo passes a limit (load, temperature, supply voltage, its
# own alarms), when a joint stalls, or on an e-stop: /arm_safety/estop or /arm_safety/torque_off
# (std_srvs/Trigger), or true on /estop (std_msgs/Bool). /arm_safety/reset lets it move again.

uint8 OK = 0          # following the controllers' commands
uint8 RESUMING = 1    # reset: holding until a controller asks for the pose the arm is in
uint8 HOLDING = 2     # stopped: the motors hold the arm where it stopped; commands are ignored
uint8 TORQUE_OFF = 3  # stopped: the motors are off and the arm is limp
uint8 READ_ONLY = 4   # torque:=false: nothing is driven; limits are only reported
uint8 INACTIVE = 5    # the driver isn't active

std_msgs/Header header
uint8 state
string reason              # why it stopped; empty unless HOLDING or TORQUE_OFF
string[] warnings          # warning levels passed, and soft limits holding the arm back

string[] joint_names
float64[] load             # fraction of each servo's maximum torque, signed
float64[] temperature      # deg C
float64[] voltage          # V
float64[] tracking_error   # rad: the goal sent minus where the joint is
float64[] gravity_load     # the model's gravity torque at the goal sent, fraction of stall

float64 workspace_margin   # m: how far inside the workspace the goal sent keeps the arm
