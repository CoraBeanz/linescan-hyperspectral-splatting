"""Following the arm's safety state from another node.

The servo driver publishes so101_scan_interfaces/ArmSafety on /arm_safety/state (latched, so
the last state arrives as soon as a node subscribes). SafetyWatch keeps the latest one and
calls back when the arm stops, which is how scan_sweep, move_arm and the mirror bridge stop
their own work on an e-stop or a fault.
"""

import threading
import time

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from so101_scan_interfaces.msg import ArmSafety

TOPIC = "/arm_safety/state"
QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
STATE_NAMES = {ArmSafety.OK: "ok", ArmSafety.RESUMING: "resuming", ArmSafety.HOLDING: "holding",
               ArmSafety.TORQUE_OFF: "torque off", ArmSafety.READ_ONLY: "read only",
               ArmSafety.INACTIVE: "inactive"}
STOPPED = (ArmSafety.HOLDING, ArmSafety.TORQUE_OFF)
RESET_HINT = "`ros2 run so101_scan_safety arm_estop --reset` lets it move again"


def is_stopped(msg):
    return msg is not None and msg.state in STOPPED


def describe(msg):
    if msg is None:
        return "no safety state (mock hardware has none)"
    text = STATE_NAMES.get(msg.state, str(msg.state))
    return text + (": " + msg.reason if msg.reason else "")


class SafetyWatch:
    def __init__(self, node, on_stop=None, callback_group=None):
        """on_stop(msg) is called each time the arm goes from moving to stopped while this
        watches; not for an arm that was stopped already, which has nothing to interrupt."""
        self.state = None
        self.on_stop = on_stop
        self.received = threading.Event()
        self._node = node
        node.create_subscription(ArmSafety, TOPIC, self._on_state, QOS, callback_group=callback_group)

    def _on_state(self, msg):
        before = self.state
        self.state = msg
        self.received.set()
        if before is not None and not is_stopped(before) and is_stopped(msg) and self.on_stop is not None:
            self.on_stop(msg)

    @property
    def stopped(self):
        return is_stopped(self.state)

    def wait(self, timeout=5.0, alone=1.0):
        """Wait for the driver's latched state; False if none came. After `alone` s with nobody
        publishing it (mock hardware) it gives up early; that only says something once the
        driver's process has been discovered, e.g. after its /joint_states arrived. alone=None
        waits the whole timeout."""
        if alone is None:
            return self.received.wait(timeout)
        deadline = time.monotonic() + timeout
        if self.received.wait(min(timeout, alone)):
            return True
        while time.monotonic() < deadline and self._node.count_publishers(TOPIC) > 0:
            if self.received.wait(0.05):
                return True
        return self.received.is_set()

    def not_ready(self, wait=5.0):
        """Why the arm can't take a move now, or None. No state at all means no driver to ask,
        as with mock hardware, which is fine. Call it once /joint_states is coming in."""
        self.wait(wait)
        msg = self.state
        if msg is None or msg.state == ArmSafety.OK:
            return None
        if msg.state == ArmSafety.RESUMING:
            return "the arm was reset but arm_controller hasn't caught up; " + RESET_HINT
        if msg.state in STOPPED:
            return "the arm is stopped (%s); %s" % (describe(msg), RESET_HINT)
        return "the arm can't move: " + describe(msg)
