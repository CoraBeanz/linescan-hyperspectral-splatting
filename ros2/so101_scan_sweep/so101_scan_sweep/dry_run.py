"""scan_sweep --dry-run: play a scan plan on the running stack without moving anything.

    ros2 run so101_scan_sweep scan_sweep --plan ~/so101_scan/plans/ring.yaml --dry-run
    ros2 run so101_scan_sweep scan_sweep --plan ... --dry-run --speed 4      # play it four times faster

It checks the plan against what is running and plays it in RViz or Foxglove on /scan/markers,
but sends nothing to arm_controller and nothing to the mirror:

  * the arm: a see-through copy moves from where the real arm is (/joint_states) through every
    viewpoint at the plan's speed, along the path the trajectory controller would take (a
    straight line in joint space). Each move is checked against the joint limits and for how
    close the arm and head come to the table (and to the object, with a coverage plan) on
    the way; a move only counts as too close if it goes lower than it started.
  * the mirror: each viewpoint's scan lines appear one by one at the line period, where they
    would land. The sweep's angles are checked against the bridge's min_angle and max_angle,
    and while line_camera runs the period is rounded up to whole camera frames, as the bridge
    will.
  * the stack: whether arm_controller, the mirror bridge (and whether it is homed) and
    line_camera are there for what the plan asks. Without the stack it checks the plan
    against the package's URDF and says so.
  * a coverage plan (make_plan --object): the object, and points on its surface that turn from
    red to yellow to green as the views that see them play.

Then it prints a report, per viewpoint the move's time and lowest point, the mirror's range and
where the lines land, and every problem found; it exits 1 if there are any. Ctrl-C stops the
playing and still prints the report.
"""

import math
import threading
import time
from dataclasses import dataclass, field

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from so101_scan_description.kinematics import Robot, rpy_matrix, transform
from so101_scan_interfaces.msg import FrameStamp, MirrorState
from so101_scan_interfaces.srv import StartRecording
from so101_scan_sweep import coverage as cov
from so101_scan_sweep import make_plan as planner
from so101_scan_sweep.line_log import JointBuffer, LinePoser, quaternion
from so101_scan_sweep.plan import ARM_JOINTS
from so101_scan_sweep.sweep import COLORS, LATCHED

NAMESPACES = ("dry_run_arm", "dry_run_skeleton", "dry_run_lines", "dry_run_object", "dry_run_coverage",
              "dry_run_label")
SKELETON = ("base_link", "shoulder_link", "upper_arm_link", "lower_arm_link", "wrist_link", "wrist_roll_link",
            "scan_head_link")
FPS = 20.0             # marker updates a second while playing


@dataclass
class ViewCheck:
    index: int
    name: str
    start: dict                    # joints the move starts from, rad
    goal: dict                     # joints of the viewpoint
    move_s: float = 0.0
    settle_s: float = 0.0
    period_s: float = 0.0          # the line period the sweep will get
    frames_per_line: int = 0       # when locked to the camera
    lowest_m: float = None         # lowest point of the arm and head on the way and at the viewpoint
    angles: np.ndarray = None      # mirror angle of every line, rad
    line_ends: list = None         # every line's two ends in base_link
    landing: np.ndarray = None     # the middle line's middle
    aim_error_m: float = None      # how far the mirror's rest line lands from the plan's aim, when it has one
    seen: np.ndarray = None        # surface points of the object it sees (coverage plans)
    problems: list = field(default_factory=list)

    @property
    def sweep_s(self):
        return len(self.angles) * self.period_s


def interpolate(a, b, t):
    return {j: a[j] + (b[j] - a[j]) * t for j in ARM_JOINTS}


def move_path(start, goal, step=math.radians(2.0)):
    """The joint-space straight line the trajectory controller follows, sampled."""
    furthest = max(abs(goal[j] - start[j]) for j in ARM_JOINTS)
    n = max(2, int(math.ceil(furthest / step)) + 1)
    return [interpolate(start, goal, t) for t in np.linspace(0.0, 1.0, n)]


def check_plan(robot, plan, start=None, mirror_limits=(-0.8, 0.8), rad_per_step=cov.RAD_PER_STEP, frame_period=None,
               clearance=0.01, table_z=planner.TABLE_Z, box=None, surface=None):
    """Everything a dry run checks, without ROS. start: the arm's joints now (None: the first
    viewpoint's). Returns (ViewChecks, problems with the plan as a whole)."""
    poser = LinePoser(robot)
    limits = robot.limits()
    problems = []
    first = next((v.joints for v in plan.viewpoints if v.joints), None)
    if start is None and first is None:
        problems.append("no viewpoint has joints and the arm's position isn't known")
        return [], problems
    here = dict(start or first)
    checks = []
    for i, vp in enumerate(plan.viewpoints):
        goal = dict(vp.joints or here)
        c = ViewCheck(i, vp.name, here, goal)
        for j in ARM_JOINTS:
            lo, hi = limits[j]
            if not lo - 1e-9 <= goal[j] <= hi + 1e-9:
                c.problems.append("%s = %.1f deg is outside its limits, %.1f..%.1f" % (
                    j, math.degrees(goal[j]), math.degrees(lo), math.degrees(hi)))
        if vp.joints:
            furthest = max(abs(goal[j] - here[j]) for j in ARM_JOINTS)
            c.move_s = max(plan.min_move_time, furthest / plan.max_joint_speed) if furthest > 1e-6 else 0.0
            c.settle_s = plan.settle_time
        path = move_path(here, goal)
        lows = [planner.lowest_point(robot, q) for q in path]
        c.lowest_m = min(lows)
        floor = table_z + clearance
        if lows[-1] < floor:
            c.problems.append("the arm or head is %.0f mm above the table here, under the %.0f mm clearance"
                              % ((lows[-1] - table_z) * 1e3, clearance * 1e3))
        elif min(lows) < min(floor, lows[0] - 0.001):
            dip = (min(lows) - table_z) * 1e3
            c.problems.append("on the way here the arm or head dips %s" % (
                "to %.0f mm above the table" % dip if dip >= 0 else "%.0f mm into the table" % -dip))
        if box is not None:
            inside = [box.contains(planner.body_points(robot, q, dense=True), clearance).any() for q in path]
            if inside[-1]:
                c.problems.append("the arm or head is within %.0f mm of the object here" % (clearance * 1e3))
            elif any(inside[1:]) and not inside[0]:
                c.problems.append("on the way here the arm or head passes within %.0f mm of the object"
                                  % (clearance * 1e3))
        # the sweep: as the bridge rounds it, to whole microsteps, and to whole camera frames
        s = vp.sweep
        start_step = round(s.start_angle / rad_per_step)
        c.angles = (start_step + np.arange(s.n_lines) * s.steps_per_line) * rad_per_step
        lo, hi = mirror_limits
        if c.angles.min() < lo - 1e-9 or c.angles.max() > hi + 1e-9:
            c.problems.append("the mirror would sweep %.2f..%.2f deg, outside the bridge's %.2f..%.2f deg"
                              % (math.degrees(c.angles.min()), math.degrees(c.angles.max()), math.degrees(lo),
                                 math.degrees(hi)))
        c.period_s = s.line_period
        if frame_period:
            c.frames_per_line = max(1, int(math.ceil(s.line_period / frame_period - 1e-3)))
            c.period_s = c.frames_per_line * frame_period
        c.line_ends = [poser.line_ends(poser.poses(goal, float(a))[1]) for a in c.angles]
        mid = c.line_ends[len(c.line_ends) // 2]
        c.landing = (mid[0] + mid[1]) / 2.0
        source = plan.source.get("viewpoints") or []
        aim = (source[i] if i < len(source) else {}).get("aim_m") or plan.source.get("target_m")
        if aim is not None:      # make_plan aims the line at the mirror's rest, angle 0
            rest = poser.line_ends(poser.poses(goal, 0.0)[1])
            c.aim_error_m = float(np.linalg.norm((rest[0] + rest[1]) / 2.0 - np.asarray(aim, float)))
        if surface is not None:
            c.seen = cov.sweep_sees(poser, goal, s, surface[0], surface[1], rad_per_step)
        checks.append(c)
        here = goal
    return checks, problems


def report(plan, checks, problems, notes, surface=None, views_per_point=2):
    out = ["dry run of %s: %d viewpoints" % (plan.name, len(checks))]
    total = 0.0
    for c in checks:
        total += c.move_s + c.settle_s + c.sweep_s
        line = "  %d %-16s move %4.1f s, lowest %3.0f mm above the table; mirror %+.1f..%+.1f deg, " \
               "%d lines in %.1f s" % (c.index + 1, c.name, c.move_s, (c.lowest_m - planner.TABLE_Z) * 1e3,
                                       math.degrees(c.angles.min()), math.degrees(c.angles.max()), len(c.angles),
                                       c.sweep_s)
        if c.frames_per_line:
            line += " (%d camera frame%s a line)" % (c.frames_per_line, "" if c.frames_per_line == 1 else "s")
        line += "; lines land around (%.3f, %.3f, %.3f) m" % tuple(c.landing)
        if c.aim_error_m is not None:
            line += ", %.1f mm from the aim" % (c.aim_error_m * 1e3)
        out.append(line)
        out += ["      ! " + p for p in c.problems]
    out.append("  about %.0f s in all" % total)
    if surface is not None and checks:
        count = np.sum([c.seen for c in checks], axis=0)
        out.append("  the object: %.0f%% of its surface seen, %.0f%% by %d or more views"
                   % (100 * (count > 0).mean(), 100 * (count >= views_per_point).mean(), views_per_point))
    out += ["  note: " + n for n in notes]
    every = problems + ["%s: %s" % (c.name, p) for c in checks for p in c.problems]
    out += ["  ! " + p for p in problems]
    out.append("%d problem%s: %s" % (len(every), "" if len(every) == 1 else "s",
                                      "scan_sweep would stop or misbehave on this plan" if every
                                      else "nothing found; the plan is good to run"))
    return "\n".join(out), every


# --- the ghost arm, lines and object as markers -------------------------------------------------

def visuals(robot):
    """The URDF's visuals: (link, origin 4x4, marker type, mesh, scale) for every mesh and box."""
    out = []
    for name, link in robot.links.items():
        for vis in link.findall("visual"):
            o = vis.find("origin")
            xyz = [float(v) for v in (o.get("xyz") if o is not None and o.get("xyz") else "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy") if o is not None and o.get("rpy") else "0 0 0").split()]
            origin = transform(rpy_matrix(*rpy), xyz)
            mesh = vis.find("geometry/mesh")
            box = vis.find("geometry/box")
            if mesh is not None:
                scale = [float(v) for v in (mesh.get("scale") or "1 1 1").split()]
                out.append((name, origin, Marker.MESH_RESOURCE, mesh.get("filename"), scale))
            elif box is not None:
                out.append((name, origin, Marker.CUBE, None, [float(v) for v in box.get("size").split()]))
    return out


def set_pose(marker, t):
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (float(v) for v in t[:3, 3])
    q = quaternion(t[:3, :3])
    marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = (
        float(v) for v in q)


def point(p):
    return Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))


class Scene:
    """What the dry run draws, built into one MarkerArray per update."""

    def __init__(self, robot, stamp, box=None, surface=None, views_per_point=2):
        self.robot, self.stamp = robot, stamp
        self.visuals = visuals(robot)
        self.box, self.surface, self.views_per_point = box, surface, views_per_point
        self.lines = {}            # viewpoint -> line ends drawn so far
        self.count = None if surface is None else np.zeros(len(surface[0]), int)
        self.arm = None
        self.colour = COLORS[0]
        self.label = ""
        self.label_at = np.zeros(3)

    def marker(self, ns, mid, kind):
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = self.stamp()
        m.ns, m.id, m.type, m.action = ns, mid, kind, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def clear(self):
        arr = MarkerArray()
        for ns in NAMESPACES:
            for mid in range(64):
                m = self.marker(ns, mid, Marker.CUBE)
                m.action = Marker.DELETE
                arr.markers.append(m)
        return arr

    def markers(self):
        arr = MarkerArray()
        r, g, b = self.colour
        if self.arm is not None:
            for k, (link, origin, kind, mesh, scale) in enumerate(self.visuals[:64]):
                m = self.marker("dry_run_arm", k, kind)
                set_pose(m, self.robot.fk(link, self.arm) @ origin)
                m.scale.x, m.scale.y, m.scale.z = scale
                if mesh:
                    m.mesh_resource = mesh
                m.color = ColorRGBA(r=r, g=g, b=b, a=0.35)
                arr.markers.append(m)
            m = self.marker("dry_run_skeleton", 0, Marker.LINE_STRIP)
            m.scale.x = 0.006
            m.color = ColorRGBA(r=r, g=g, b=b, a=0.9)
            m.points = [point(self.robot.fk(link, self.arm)[:3, 3]) for link in SKELETON]
            arr.markers.append(m)
        for vp, ends in self.lines.items():
            m = self.marker("dry_run_lines", vp % 64, Marker.LINE_LIST)
            m.scale.x = 0.0008
            cr, cg, cb = COLORS[vp % len(COLORS)]
            m.color = ColorRGBA(r=cr, g=cg, b=cb, a=0.9)
            for a, b_ in ends:
                m.points += [point(a), point(b_)]
            arr.markers.append(m)
        if self.box is not None:
            m = self.marker("dry_run_object", 0, Marker.CUBE)
            m.pose.position = point(self.box.centre)
            m.scale.x, m.scale.y, m.scale.z = (float(v) for v in self.box.size)
            m.color = ColorRGBA(r=0.8, g=0.8, b=0.8, a=0.25)
            arr.markers.append(m)
        if self.count is not None:
            m = self.marker("dry_run_coverage", 0, Marker.POINTS)
            m.scale.x = m.scale.y = 0.002
            m.points = [point(p) for p in self.surface[0]]
            m.colors = [ColorRGBA(r=0.9, g=0.15, b=0.1, a=1.0) if n == 0 else
                        ColorRGBA(r=1.0, g=0.85, b=0.1, a=1.0) if n < self.views_per_point else
                        ColorRGBA(r=0.2, g=0.9, b=0.3, a=1.0) for n in self.count]
            arr.markers.append(m)
        if self.label:
            m = self.marker("dry_run_label", 0, Marker.TEXT_VIEW_FACING)
            m.pose.position = point(self.label_at + [0.0, 0.0, 0.06])
            m.scale.z = 0.012
            m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            m.text = self.label
            arr.markers.append(m)
        return arr


# --- the node -------------------------------------------------------------------------------------

class DryRun(Node):
    def __init__(self):
        super().__init__("scan_dry_run")
        self.joints = JointBuffer()
        self.urdf = None
        self.mirror_state = None
        self.frame_stamps = []
        self.lock = threading.Lock()
        self.create_subscription(String, "/robot_description", self.on_description, LATCHED)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 100)
        self.create_subscription(MirrorState, "/scan_mirror/state", self.on_mirror_state, 10)
        self.create_subscription(FrameStamp, "/line_camera/frame", self.on_frame, 50)
        self.marker_pub = self.create_publisher(MarkerArray, "/scan/markers", LATCHED)
        # only to see whether they are there: nothing is ever sent to them
        self.arm = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.mirror_params = self.create_client(GetParameters, "/scan_mirror/get_parameters")
        self.record_client = self.create_client(StartRecording, "/line_camera/start_recording")

    def on_description(self, msg):
        self.urdf = msg.data

    def on_joint_states(self, msg):
        self.joints.add(rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds, msg.name, msg.position)

    def on_mirror_state(self, msg):
        self.mirror_state = msg

    def on_frame(self, msg):
        with self.lock:
            self.frame_stamps = (self.frame_stamps + [rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds])[-31:]

    @staticmethod
    def wait_for(predicate, timeout):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            time.sleep(0.05)
        return predicate()

    def frame_period(self):
        with self.lock:
            stamps = list(self.frame_stamps)
        return float(np.median(np.diff(stamps))) * 1e-9 if len(stamps) > 5 else None

    def mirror_settings(self):
        """(min_angle, max_angle, frame_lock) from the bridge's parameters, or None."""
        if not self.mirror_params.wait_for_service(timeout_sec=2.0):
            return None
        future = self.mirror_params.call_async(GetParameters.Request(names=["min_angle", "max_angle", "frame_lock"]))
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(3.0) or future.result() is None or len(future.result().values) != 3:
            return None
        lo, hi, lock = future.result().values
        return lo.double_value, hi.double_value, lock.bool_value

    def look(self, plan):
        """What is running, and what scan_sweep would make of it: (robot, start joints, mirror
        limits, camera frame period, problems, notes)."""
        problems, notes = [], []
        live = self.wait_for(lambda: self.urdf is not None, 3.0)
        if live:
            robot = Robot(self.urdf)
        else:
            robot = planner.load_robot()
            notes.append("no /robot_description, so the scan stack isn't running: checked the plan against the "
                         "package's URDF only")
        start = None
        if self.wait_for(lambda: self.joints.latest_ns is not None, 2.0 if live else 0.5):
            start = self.joints.at(self.joints.latest_ns)[0]
        elif live:
            notes.append("no arm joints on /joint_states: the moves start from the first viewpoint")
        limits, frame_period = (-0.8, 0.8), None
        if not live:
            return robot, start, limits, frame_period, problems, notes
        if not self.arm.wait_for_server(timeout_sec=2.0):
            problems.append("arm_controller isn't running (with torque:=false it isn't started), so the arm "
                            "couldn't move")
        if not self.wait_for(lambda: self.mirror_state is not None, 2.0):
            problems.append("the mirror bridge isn't running (no /scan_mirror/state)")
        elif self.mirror_state.state == "disconnected":
            problems.append("the mirror bridge can't reach the mirror ESP32")
        elif not self.mirror_state.homed:
            if plan.home_mirror:
                notes.append("the mirror isn't homed: scan_sweep would home it first")
            else:
                problems.append("the mirror isn't homed and the plan has home_mirror: false")
        elif self.mirror_state.busy:
            notes.append("the mirror is busy right now")
        settings = self.mirror_settings()
        lock = True
        if settings is None:
            if self.mirror_state is not None:
                notes.append("couldn't read the bridge's min_angle and max_angle: checked against +-0.8 rad")
        else:
            limits, lock = settings[:2], settings[2]
        camera = self.record_client.wait_for_service(timeout_sec=1.0 if plan.camera_record != "off" else 0.1)
        if plan.camera_record == "required" and not camera:
            problems.append("the plan has camera.record: required and line_camera isn't running")
        elif plan.camera_record != "off":
            notes.append("line_camera %s" % ("would record the lines" if camera else
                                             "isn't running: the scan would go without it"))
        if camera and lock:
            self.wait_for(lambda: self.frame_period() is not None, 1.0)
            frame_period = self.frame_period()
        return robot, start, limits, frame_period, problems, notes


def play(node, scene, checks, speed, plan):
    """Animate the plan: the arm's moves, then each sweep's lines at the line rate."""
    publish = lambda: node.marker_pub.publish(scene.markers())    # noqa: E731
    dt = 1.0 / FPS
    for c in checks:
        scene.colour = COLORS[c.index % len(COLORS)]
        scene.label_at = c.landing
        n_frames = max(1, int(c.move_s / speed * FPS))
        for k in range(n_frames + 1):
            scene.arm = interpolate(c.start, c.goal, k / n_frames)
            scene.label = "%d/%d %s: moving %.1f s" % (c.index + 1, len(checks), c.name, c.move_s)
            publish()
            time.sleep(dt)
        time.sleep(c.settle_s / speed)
        scene.lines[c.index] = []
        t0 = time.monotonic()
        shown = 0
        while shown < len(c.line_ends):
            due = min(len(c.line_ends), int((time.monotonic() - t0) * speed / c.period_s) + 1)
            scene.lines[c.index] = c.line_ends[:due]
            shown = due
            scene.label = "%d/%d %s: line %d of %d" % (c.index + 1, len(checks), c.name, shown, len(c.line_ends))
            publish()
            if shown < len(c.line_ends):
                time.sleep(dt)
        if scene.count is not None and c.seen is not None:
            scene.count += c.seen
    scene.label = "dry run of %s: %d viewpoints" % (plan.name, len(checks))
    publish()


def run(plan, speed=1.0, animate=True, clearance=0.01):
    """The dry run, from scan_sweep's main. Returns the exit code."""
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DryRun()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    code = 130
    try:
        robot, start, limits, frame_period, problems, notes = node.look(plan)
        box = surface = None
        obj = plan.source.get("object")
        if obj:
            box = cov.Box(np.asarray(obj["centre_m"], float), np.asarray(obj["size_m"], float))
            surface = box.surface()
        vpp = int((plan.source.get("coverage") or {}).get("views_per_point", 2))
        checks, plan_problems = check_plan(robot, plan, start, limits, frame_period=frame_period,
                                           clearance=clearance, box=box, surface=surface)
        text, every = report(plan, checks, problems + plan_problems, notes, surface, vpp)
        scene = Scene(robot, lambda: node.get_clock().now().to_msg(), box, surface, vpp)
        node.marker_pub.publish(scene.clear())
        try:
            if animate and checks:
                play(node, scene, checks, max(speed, 1e-3), plan)
            else:
                for c in checks:
                    scene.lines[c.index] = c.line_ends
                    if scene.count is not None:
                        scene.count += c.seen
                if checks:
                    scene.arm, scene.label_at = checks[-1].goal, checks[-1].landing
                    scene.label = "dry run of %s: %d viewpoints" % (plan.name, len(checks))
                node.marker_pub.publish(scene.markers())
        except KeyboardInterrupt:
            print("stopped playing")
        print(text)
        time.sleep(0.3)        # let the last markers go out
        code = 1 if every else 0
    except KeyboardInterrupt:
        print("stopped")
    finally:
        executor.shutdown()
        spinner.join(timeout=5.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return code
