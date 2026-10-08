"""scan_sweep: run a scan plan and log where the instrument was at every scan line.

    ros2 run so101_scan_sweep scan_sweep --plan ~/so101_scan/plans/one_view.yaml

Needs scan_arm.launch.py running. For each viewpoint in the plan (plan.py) it moves the arm
there through arm_controller, waits for it to settle, starts a mirror sweep, and for every
scan line the mirror reports works out the head and line-camera poses at the line's timestamp
(line_log.py). A move that fails (the arm blocked, or the servo bus lost) ends the run. Each
run writes a folder:

    <output_dir>/<name>_<YYYYmmdd-HHMMSS>/
      lines.csv    one row per line: viewpoint, sweep_id, index, stamp_ns (the mirror settled),
                   hold_until_ns (it moved on), settled, mirror_angle, the head and line-camera
                   poses in base_link (m, quaternion x y z w), the arm joints (rad)
      scan.json    the run: the plan, per-viewpoint counts and timing, frames and units
      robot.urdf   the URDF the poses came from

With line_camera running (scan_arm.launch.py camera:=v4l2), it also has the camera record into
the same folder (frames/ and binned/, see so101_scan_camera/session.py), and the mirror bridge
locks the lines to the camera's frames; the plan's camera.record says whether that is needed
(plan.py). scan_to_dataset (so101_scan_camera) turns the folder into the splat trainer's dataset.

It also publishes every line's pose on /scan/line_pose and the scan lines as RViz markers on
/scan/markers. Ctrl-C stops the mirror and still writes what was logged.

--dry-run plays the plan on the running stack without moving the arm or the mirror, checks
it, and writes nothing (dry_run.py).
"""

import argparse
import json
import os
import threading
import time
from collections import deque
from datetime import datetime

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from so101_scan_description.kinematics import Robot
from so101_scan_interfaces.msg import MirrorState, ScanLine, ScanLinePose
from so101_scan_interfaces.srv import StartRecording, StartSweep
from so101_scan_sweep import plan as plan_mod
from so101_scan_sweep.line_log import JointBuffer, LinePoser, LinesCsv, quaternion
from so101_scan_sweep.plan import ARM_JOINTS

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
COLORS = [(0.2, 0.9, 1.0), (1.0, 0.6, 0.1), (0.5, 1.0, 0.3), (1.0, 0.3, 0.6), (0.7, 0.5, 1.0), (1.0, 1.0, 0.3)]


class ScanError(RuntimeError):
    pass


class SweepLog:
    """Everything logged for one sweep."""

    def __init__(self, viewpoint, name, n_lines):
        self.viewpoint, self.name, self.n_lines = viewpoint, name, n_lines
        self.stamps, self.joints, self.gaps_ns, self.ends = [], [], [], []
        self.unsettled = 0
        self.done = threading.Event()   # its last line is logged, or the mirror stopped early


def to_pose(t):
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in t[:3, 3])
    q = quaternion(t[:3, :3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(v) for v in q)
    return p


class ScanSweep(Node):
    def __init__(self):
        super().__init__("scan_sweep")
        self.joints = JointBuffer()
        self.robot = None
        self.poser = None
        self.urdf = None
        self.mirror_state = None
        self.pending = deque()
        self.sweeps = {}      # sweep_id -> SweepLog
        self.csv = None
        self.arm_goal = None  # the move in progress, to cancel on Ctrl-C
        self.lock = threading.Lock()
        group = ReentrantCallbackGroup()

        self.create_subscription(String, "/robot_description", self.on_description, LATCHED)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 100)
        self.create_subscription(ScanLine, "/scan_mirror/line", self.on_line, 500)
        self.create_subscription(MirrorState, "/scan_mirror/state", self.on_mirror_state, 10)
        self.pose_pub = self.create_publisher(ScanLinePose, "/scan/line_pose", 100)
        self.marker_pub = self.create_publisher(MarkerArray, "/scan/markers", LATCHED)
        self.arm = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory",
                                callback_group=group)
        self.home_client = self.create_client(Trigger, "/scan_mirror/home", callback_group=group)
        self.stop_client = self.create_client(Trigger, "/scan_mirror/stop", callback_group=group)
        self.sweep_client = self.create_client(StartSweep, "/scan_mirror/start_sweep", callback_group=group)
        self.record_client = self.create_client(StartRecording, "/line_camera/start_recording", callback_group=group)
        self.stop_record_client = self.create_client(Trigger, "/line_camera/stop_recording", callback_group=group)
        self.create_timer(0.02, self.process_pending)

    # --- subscriptions --------------------------------------------------------------------

    def on_description(self, msg):
        self.urdf = msg.data
        self.robot = Robot(msg.data)
        self.poser = LinePoser(self.robot)

    def on_joint_states(self, msg):
        self.joints.add(rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds, msg.name, msg.position)

    def on_mirror_state(self, msg):
        self.mirror_state = msg
        log = self.sweeps.get(msg.sweep_id)
        if log is not None and not msg.busy:
            log.done.set()  # ended without a last line: stopped, a driver fault, or the ESP32 restarted

    def on_line(self, msg):
        with self.lock:
            self.pending.append((msg, time.monotonic()))

    # --- line poses -----------------------------------------------------------------------

    def process_pending(self):
        """Pose every line whose instant the joint states have caught up with."""
        while True:
            with self.lock:
                if not self.pending:
                    return
                msg, arrived = self.pending[0]
                log = self.sweeps.get(msg.sweep_id)
                waited = time.monotonic() - arrived
                stamp = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds
                latest = self.joints.latest_ns
                if log is None:
                    if waited < 2.0:
                        return  # its StartSweep reply may still be on the way
                    self.pending.popleft()
                    self.get_logger().debug("line of unknown sweep %d dropped" % msg.sweep_id)
                    continue
                if self.poser is None or ((latest is None or latest < stamp) and waited < 0.5):
                    return
                self.pending.popleft()
            self.log_line(log, msg, stamp)

    def log_line(self, log, msg, stamp):
        joints, gap = self.joints.at(stamp)
        if joints is None:
            self.get_logger().error("no /joint_states with the arm joints; line %d not logged" % msg.index)
            return
        head, camera = self.poser.poses(joints, msg.angle)
        log.stamps.append(stamp)
        log.joints.append([joints[j] for j in ARM_JOINTS])
        log.gaps_ns.append(gap)
        log.ends.append(self.poser.line_ends(camera))
        log.unsettled += not msg.settled
        hold_until = rclpy.time.Time.from_msg(msg.hold_until).nanoseconds
        with self.lock:
            if self.csv:
                self.csv.write(log.viewpoint, msg.sweep_id, msg.index, stamp, hold_until, msg.settled, msg.angle,
                               head, camera, joints)
        out = ScanLinePose()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.poser.base
        out.viewpoint = log.viewpoint
        out.sweep_id = msg.sweep_id
        out.index = msg.index
        out.mirror_angle = msg.angle
        out.head_pose = to_pose(head)
        out.line_camera_pose = to_pose(camera)
        out.joint_names = list(ARM_JOINTS)
        out.joint_positions = [float(joints[j]) for j in ARM_JOINTS]
        self.pose_pub.publish(out)
        if msg.last:
            log.done.set()
        if msg.last or len(log.stamps) % 20 == 0:
            self.publish_markers()

    def publish_markers(self):
        arr = MarkerArray()
        for sweep_id, log in list(self.sweeps.items()):
            m = Marker()
            m.header.frame_id = "base_link"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = "scan_lines", int(log.viewpoint), Marker.LINE_LIST, Marker.ADD
            m.scale.x = 0.0008
            r, g, b = COLORS[log.viewpoint % len(COLORS)]
            m.color = ColorRGBA(r=r, g=g, b=b, a=0.9)
            m.pose.orientation.w = 1.0
            for a, b_ in list(log.ends):
                m.points.append(Point(x=float(a[0]), y=float(a[1]), z=float(a[2])))
                m.points.append(Point(x=float(b_[0]), y=float(b_[1]), z=float(b_[2])))
            arr.markers.append(m)
        self.marker_pub.publish(arr)

    # --- waiting on ROS from the main thread -----------------------------------------------

    @staticmethod
    def wait(future, timeout, what):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            raise ScanError("no answer from %s within %.0f s" % (what, timeout))
        return future.result()

    def call(self, client, request, timeout, what):
        if not client.wait_for_service(timeout_sec=5.0):
            raise ScanError("%s is not available; is scan_arm.launch.py running?" % what)
        return self.wait(client.call_async(request), timeout, what)

    def wait_for(self, predicate, timeout, what):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise ScanError("timed out waiting for %s" % what)
            time.sleep(0.05)

    # --- the plan ---------------------------------------------------------------------------

    def move_arm(self, goal_joints, plan):
        # right after the launch starts, the joint state broadcaster may not be publishing yet
        self.wait_for(lambda: self.joints.latest_ns is not None, 5.0,
                      "the arm joints on /joint_states (is scan_arm.launch.py running?)")
        now = self.joints.at(self.joints.latest_ns)[0]
        if not self.arm.wait_for_server(timeout_sec=5.0):
            raise ScanError("arm_controller is not running (with torque:=false it isn't started)")
        furthest = max(abs(goal_joints[j] - now[j]) for j in ARM_JOINTS)
        duration = max(plan.min_move_time, furthest / plan.max_joint_speed)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(goal_joints[j]) for j in ARM_JOINTS]
        point.time_from_start = DurationMsg(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        goal.trajectory.points = [point]
        handle = self.wait(self.arm.send_goal_async(goal), 5.0, "arm_controller")
        if not handle.accepted:
            raise ScanError("arm_controller rejected the move")
        self.arm_goal = handle
        result = self.wait(handle.get_result_async(), duration + 10.0, "the arm move").result
        self.arm_goal = None
        ok = result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        return ok, "" if ok else (result.error_string or "error code %d" % result.error_code), duration

    def settle(self, settle_time, still=0.003, window=0.25, timeout=3.0):
        """Wait settle_time, then until no joint has moved more than `still` rad in `window` s."""
        time.sleep(settle_time)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            latest = self.joints.latest_ns
            a, _ = self.joints.at(latest)
            b, _ = self.joints.at(latest - int(window * 1e9))
            if max(abs(a[j] - b[j]) for j in ARM_JOINTS) < still:
                return True
            time.sleep(0.05)
        return False

    def stop_arm(self):
        """Cancel the move in progress: the controller holds the arm where it is."""
        handle, self.arm_goal = self.arm_goal, None
        if handle is not None:
            try:
                self.wait(handle.cancel_goal_async(), 2.0, "arm_controller")
            except Exception as e:
                self.get_logger().error("could not cancel the arm move: %s" % e)

    def ensure_homed(self, plan):
        self.wait_for(lambda: self.mirror_state is not None, 5.0,
                      "/scan_mirror/state (is the mirror bridge running?)")
        if self.mirror_state.homed:
            return
        if not plan.home_mirror:
            raise ScanError("the mirror is not homed and the plan has home_mirror: false")
        self.get_logger().info("homing the scan mirror")
        res = self.call(self.home_client, Trigger.Request(), 90.0, "/scan_mirror/home")
        if not res.success:
            raise ScanError("homing failed: %s" % res.message)

    def start_recording(self, plan, out_dir, info):
        """Have line_camera write its lines into the scan folder. Returns whether it does."""
        mode = plan.camera_record
        info["camera"] = {"record": mode, "recording": False}
        if mode == "off":
            return False
        if not self.record_client.wait_for_service(timeout_sec=5.0 if mode == "required" else 2.0):
            if mode == "required":
                raise ScanError("line_camera is not running and the plan has camera.record: required; start it "
                                "with scan_arm.launch.py camera:=v4l2")
            self.get_logger().warning("line_camera is not running: scanning without the camera")
            return False
        deadline = time.monotonic() + 5.0     # it may still be starting the camera
        while True:
            res = self.wait(self.record_client.call_async(StartRecording.Request(directory=out_dir)), 10.0,
                            "/line_camera/start_recording")
            if res.success or time.monotonic() > deadline:
                break
            time.sleep(0.5)
        info["camera"]["message"] = res.message
        if not res.success:
            if mode == "required":
                raise ScanError("line_camera can't record: %s" % res.message)
            self.get_logger().warning("line_camera can't record (%s): scanning without the camera" % res.message)
            return False
        info["camera"].update(recording=True, calibrated=res.calibrated, raw=res.raw)
        references = plan.camera_references or res.references
        if references:
            info["camera"]["references"] = references   # where scan_to_dataset also looks for darks and whites
        self.get_logger().info(res.message)
        return True

    def stop_recording(self, info):
        try:
            res = self.call(self.stop_record_client, Trigger.Request(), 30.0, "/line_camera/stop_recording")
            info["camera"]["stopped"] = res.message
            self.get_logger().info(res.message)
        except Exception as e:  # still write scan.json
            self.get_logger().error("could not stop line_camera's recording: %s" % e)

    def sweep(self, index, viewpoint):
        s = viewpoint.sweep
        req = StartSweep.Request(start_angle=s.start_angle, steps_per_line=s.steps_per_line,
                                 n_lines=s.n_lines, line_period=s.line_period)
        res = self.call(self.sweep_client, req, 5.0, "/scan_mirror/start_sweep")
        if not res.accepted:
            raise ScanError("sweep refused: %s" % res.message)
        log = SweepLog(index, viewpoint.name, s.n_lines)
        with self.lock:
            self.sweeps[res.sweep_id] = log
        if res.frame_locked:
            self.get_logger().info(res.message)
        # locked to the camera, the period is a whole number of frames, so maybe longer than asked
        log.done.wait(s.n_lines * max(res.line_period, s.line_period) + 5.0)
        time.sleep(0.6)  # let the last lines' joint states arrive
        return res, log

    def run(self, plan, out_dir):
        info = {"format": "so101_scan lines v1", "name": plan.name, "started": datetime.now().isoformat(),
                "plan": plan.source, "viewpoints": [], "interrupted": False}
        self.wait_for(lambda: self.poser is not None, 10.0, "/robot_description")
        problems = plan_mod.check_limits(plan, self.robot.limits())
        if problems:
            raise ScanError("the plan is outside the joint limits:\n  " + "\n  ".join(problems))
        os.makedirs(out_dir)
        with open(os.path.join(out_dir, "robot.urdf"), "w") as f:
            f.write(self.urdf)
        with open(os.path.join(out_dir, "plan.yaml"), "w") as f:
            yaml.safe_dump(plan.source, f, sort_keys=False)
        self.csv = LinesCsv(os.path.join(out_dir, "lines.csv"))
        info["frames"] = {
            "base": self.poser.base, "head": self.poser.head, "line_camera": self.poser.camera,
            "line_camera_axes": "z: the view through the mirror; x: along the slit, in pixel order; y = z cross x",
            "scene_distance_m": self.poser.scene_distance, "scan_line_half_length_m": self.poser.half_line}
        info["units"] = {"position": "m", "angle": "rad", "quaternion": "x y z w",
                         "stamp_ns": "ROS time in ns at which the mirror settled on the line",
                         "hold_until_ns": "ROS time in ns at which the mirror moved on to the next line"}
        recording = False
        try:
            recording = self.start_recording(plan, out_dir, info)
            self.ensure_homed(plan)
            for i, vp in enumerate(plan.viewpoints):
                entry = {"name": vp.name, "joints_goal": vp.joints}
                info["viewpoints"].append(entry)
                if vp.joints:
                    self.get_logger().info("viewpoint %d/%d %s: moving" % (i + 1, len(plan.viewpoints), vp.name))
                    ok, message, duration = self.move_arm(vp.joints, plan)
                    entry["move"] = {"ok": ok, "message": message, "duration_s": duration}
                    if not ok:
                        # the arm is stuck or the bus is gone: stop rather than push on to the next pose
                        raise ScanError("the move to %s failed (%s); stopping the scan. The arm holds "
                                        "where it is." % (vp.name, message))
                    entry["settled"] = self.settle(plan.settle_time)
                self.get_logger().info("viewpoint %d/%d %s: sweeping %d lines"
                                       % (i + 1, len(plan.viewpoints), vp.name, vp.sweep.n_lines))
                res, log = self.sweep(i, vp)
                entry.update(summarize(res, log))
                self.get_logger().info("viewpoint %s: %d of %d lines logged"
                                       % (vp.name, len(log.stamps), vp.sweep.n_lines))
        except KeyboardInterrupt:
            info["interrupted"] = True
            self.get_logger().warning("interrupted; stopping the arm and the mirror")
            self.stop_arm()
            try:
                self.call(self.stop_client, Trigger.Request(), 2.0, "/scan_mirror/stop")
            except Exception as e:  # still write what was logged
                self.get_logger().error("could not stop the mirror: %s" % e)
        finally:
            # log what is still queued, then close the file
            deadline = time.monotonic() + 1.0
            while self.pending and time.monotonic() < deadline:
                time.sleep(0.05)
            with self.lock:
                csv_file, self.csv = self.csv, None
            if recording:
                self.stop_recording(info)
            info["finished"] = datetime.now().isoformat()
            if csv_file:
                csv_file.close()
                info["lines"] = csv_file.rows
            with open(os.path.join(out_dir, "scan.json"), "w") as f:
                json.dump(info, f, indent=2)
        return info


def summarize(res, log):
    out = {"sweep_id": res.sweep_id, "start_angle": res.start_angle, "rad_per_step": res.rad_per_step,
           "line_period_s": res.line_period, "frame_locked": res.frame_locked,
           "frames_per_line": res.frames_per_line,
           "lines_expected": log.n_lines, "lines_logged": len(log.stamps), "lines_unsettled": log.unsettled}
    if len(log.stamps) > 1:
        periods = np.diff(np.array(log.stamps, dtype=np.int64)) * 1e-9
        out["line_period_mean_s"] = float(periods.mean())
        out["line_period_max_deviation_s"] = float(np.abs(periods - periods.mean()).max())
    if log.joints:
        j = np.array(log.joints)
        out["joints_mean"] = dict(zip(ARM_JOINTS, j.mean(axis=0).tolist()))
        # how still the arm held during the sweep
        out["arm_motion_max_rad"] = float((j.max(axis=0) - j.min(axis=0)).max())
        out["joint_sample_gap_max_ms"] = max(log.gaps_ns) * 1e-6
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="Run a scan plan and log the pose of every scan line")
    ap.add_argument("--plan", required=True, help="plan YAML (see plans/)")
    ap.add_argument("--output", help="folder to write (default: <plan output_dir>/<name>_<date>-<time>)")
    ap.add_argument("--dry-run", action="store_true",
                    help="play the plan on the running stack without moving anything, and check it")
    ap.add_argument("--speed", type=float, default=1.0, help="with --dry-run: play this many times faster")
    ap.add_argument("--no-play", action="store_true", help="with --dry-run: just check, and draw the end result")
    ap.add_argument("--clearance", type=float, default=0.01,
                    help="with --dry-run: m the arm and head should keep from the table (and the object)")
    args, _ = ap.parse_known_args(argv)  # leaves --ros-args to rclpy
    plan = plan_mod.load(args.plan)
    if args.dry_run:
        from so101_scan_sweep import dry_run
        return dry_run.run(plan, args.speed, not args.no_play, args.clearance)
    out_dir = args.output or os.path.join(plan.output_dir, "%s_%s" % (
        plan.name, datetime.now().strftime("%Y%m%d-%H%M%S")))

    # reads --ros-args from sys.argv; Ctrl-C raises KeyboardInterrupt here instead of shutting
    # ROS down, so the mirror can still be told to stop
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ScanSweep()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    code = 0
    try:
        info = node.run(plan, out_dir)
        print("wrote %d lines to %s" % (info.get("lines", 0), out_dir))
    except ScanError as e:
        node.get_logger().error(str(e))
        code = 1
    finally:
        executor.shutdown()
        # let spin() return before Python exits: a daemon thread still inside rclpy's C++
        # wait when the interpreter shuts down aborts the process ("terminate called
        # without an active exception"), even though the scan itself went fine
        spinner.join(timeout=5.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
