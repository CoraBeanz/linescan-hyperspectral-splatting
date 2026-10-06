"""move_arm: move the arm to joint angles, or to a viewpoint of a scan plan, and stop there.

    ros2 run so101_scan_sweep move_arm --joints-deg 0 0 50 -50 -87
    ros2 run so101_scan_sweep move_arm --plan ~/so101_scan/plans/ring.yaml --viewpoint down

Joint order: shoulder_pan shoulder_lift elbow_flex wrist_flex wrist_roll. Needs
scan_arm.launch.py running with the motors on. --speed-deg sets how fast the joint that moves
furthest goes (default 20 deg/s, slow on purpose for first tries). Ctrl-C stops the arm where
it is.
"""

import argparse
import math
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from so101_scan_sweep import plan as plan_mod
from so101_scan_sweep.plan import ARM_JOINTS
from so101_scan_sweep.sweep import ScanError, ScanSweep


class _Speed:
    def __init__(self, deg_per_s, min_time):
        self.max_joint_speed = math.radians(deg_per_s)
        self.min_move_time = min_time


def main(argv=None):
    ap = argparse.ArgumentParser(description="Move the SO-101 to joint angles or a plan viewpoint")
    ap.add_argument("--joints-deg", type=float, nargs=5, metavar="DEG")
    ap.add_argument("--plan")
    ap.add_argument("--viewpoint", help="name or index in --plan (default: the first)")
    ap.add_argument("--speed-deg", type=float, default=20.0)
    args, _ = ap.parse_known_args(argv)  # leaves --ros-args to rclpy
    if args.joints_deg:
        goal = {j: math.radians(v) for j, v in zip(ARM_JOINTS, args.joints_deg)}
    elif args.plan:
        plan = plan_mod.load(args.plan)
        vps = plan.viewpoints
        pick = args.viewpoint or "0"
        match = [v for v in vps if v.name == pick]
        if not match and pick.isdigit() and int(pick) < len(vps):
            match = [vps[int(pick)]]
        if not match or not match[0].joints:
            ap.error("no viewpoint %r with joints in %s" % (pick, args.plan))
        goal = match[0].joints
    else:
        ap.error("give --joints-deg or --plan")

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)  # Ctrl-C -> KeyboardInterrupt below
    node = ScanSweep()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    code = 0
    try:
        node.wait_for(lambda: node.robot is not None, 10.0, "/robot_description")
        problems = [p for p in plan_mod.check_limits(plan_mod.Plan(
            "move", "", False, 1, 1, 0, [plan_mod.Viewpoint("goal", goal, None)]), node.robot.limits())]
        if problems:
            raise ScanError("; ".join(problems))
        node.wait_for(lambda: node.joints.latest_ns is not None, 5.0, "/joint_states")
        ok, message, duration = node.move_arm(goal, _Speed(args.speed_deg, 1.0))
        now, _ = node.joints.at(node.joints.latest_ns)
        print("%s in %.1f s; now at %s deg" % ("arrived" if ok else "move failed: " + message, duration,
                                                " ".join("%.1f" % math.degrees(now[j]) for j in ARM_JOINTS)))
        code = 0 if ok else 1
    except ScanError as e:
        node.get_logger().error(str(e))
        code = 1
    except KeyboardInterrupt:
        node.stop_arm()
        print("stopped")
        code = 1
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
