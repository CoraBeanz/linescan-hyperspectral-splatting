"""arm_estop: a keyboard e-stop for the scan arm, and the way to reset it.

    ros2 run so101_scan_safety arm_estop            # keep it open in a terminal while the arm runs
    ros2 run so101_scan_safety arm_estop --stop     # or one action and exit
    ros2 run so101_scan_safety arm_estop --torque-off
    ros2 run so101_scan_safety arm_estop --reset
    ros2 run so101_scan_safety arm_estop --status

In the terminal: Enter or space stops the arm where it is (the motors hold it), l goes limp
(motors off: hold the arm first), r resets, q quits. It prints every change of the safety
state, and why the arm stopped.

A reset stops arm_controller, clears the stop in the servo driver, and starts arm_controller
again, so the controller starts from where the arm is instead of from the goal it had before
the stop. The driver refuses while the cause is still there (a servo still too hot, say).

This is a software stop: it works through the servo driver and the bus. The servos' power
switch is the e-stop that always works.
"""

import argparse
import os
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from controller_manager_msgs.srv import SwitchController
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from so101_scan_interfaces.msg import ArmSafety
from so101_scan_safety.watch import SafetyWatch, describe

CONTROLLER = "arm_controller"


class Estop(Node):
    def __init__(self):
        super().__init__("arm_estop")
        self.safety_clients = {name: self.create_client(Trigger, "/arm_safety/" + name)
                        for name in ("estop", "torque_off", "reset")}
        self.switch = self.create_client(SwitchController, "/controller_manager/switch_controller")
        self.watch = SafetyWatch(self)

    def call(self, client, request, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        return future.result() if done.wait(timeout) else None

    def trigger(self, name):
        res = self.call(self.safety_clients[name], Trigger.Request())
        if res is None:
            return False, "no answer from /arm_safety/%s: is scan_arm.launch.py running with the real arm?" % name
        return res.success, res.message

    def switch_controller(self, activate):
        req = SwitchController.Request()
        if activate:
            req.activate_controllers = [CONTROLLER]
        else:
            req.deactivate_controllers = [CONTROLLER]
        req.strictness = SwitchController.Request.BEST_EFFORT
        res = self.call(self.switch, req)
        return res is not None and res.ok

    def reset(self):
        """Stop arm_controller, reset the driver, start arm_controller from where the arm is."""
        if not self.switch_controller(False):
            self.get_logger().warning("couldn't stop %s; resetting anyway" % CONTROLLER)
        ok, message = self.trigger("reset")
        if not self.switch_controller(True):
            return False, message + "; and %s didn't start again (is it loaded?)" % CONTROLLER
        return ok, message


def interactive(node):
    print("arm_estop: Enter or space stops the arm, l goes limp, r resets, q quits")
    print("now: " + describe(node.watch.state if node.watch.wait() else None), flush=True)
    last = node.watch.state
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd) if os.isatty(fd) else None
    if old is not None:
        tty.setcbreak(fd)
    try:
        while True:
            state = node.watch.state
            if state is not None and (last is None or (state.state, state.reason) != (last.state, last.reason)):
                print("now: " + describe(state), flush=True)
                last = state
            if not select.select([sys.stdin], [], [], 0.1)[0]:
                continue
            key = sys.stdin.read(1)
            if key == "":
                return 0
            key = key.lower()
            if key in ("\n", "\r", " "):
                print("stop: %s" % node.trigger("estop")[1], flush=True)
            elif key == "l":
                print("limp: %s" % node.trigger("torque_off")[1], flush=True)
            elif key == "r":
                print("reset: %s" % node.reset()[1], flush=True)
            elif key == "q":
                return 0
    finally:
        if old is not None:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    group = ap.add_mutually_exclusive_group()
    group.add_argument("--stop", action="store_true", help="stop the arm where it is")
    group.add_argument("--torque-off", action="store_true", help="switch the motors off (hold the arm first)")
    group.add_argument("--reset", action="store_true", help="let the arm move again")
    group.add_argument("--status", action="store_true", help="print the safety state")
    args, _ = ap.parse_known_args(argv)
    rclpy.init()
    node = Estop()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    code = 0
    try:
        if args.stop or args.torque_off or args.reset:
            ok, message = (node.reset() if args.reset else node.trigger("estop" if args.stop else "torque_off"))
            print(message)
            code = 0 if ok else 1
            if ok and args.reset and node.watch.wait():  # say when the controller has caught up
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline and node.watch.state.state != ArmSafety.OK:
                    time.sleep(0.05)
                print("now: " + describe(node.watch.state))
        elif args.status:
            print(describe(node.watch.state if node.watch.wait() else None))
            state = node.watch.state
            if state is not None:
                for w in state.warnings:
                    print("warning: " + w)
        else:
            code = interactive(node)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        spinner.join(timeout=5.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
