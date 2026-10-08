"""The arm's safety layer on the whole stack without hardware.

scan_arm.launch.py runs the real STS3215 driver on the fake servo bus, as in test_scan_stack,
and the fake bus's control socket plays the faults. Then:
  * an e-stop during a scan stops the arm and the mirror, scan_sweep says why and keeps the
    log, move_arm refuses until arm_estop --reset, and after it the arm moves again;
  * /estop and arm_estop --torque-off stop it too, and a reset brings the motors back;
  * a joint blocked part way stalls, and the driver holds the arm;
  * move_arm refuses a goal in the table, a trajectory sent straight to the controller stops
    with the head at the clearance the Python model gives, and the arm can move back out.
"""

import json
import math
import os
import signal
import subprocess
import threading
import time

import pytest
import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from so101_scan_description.kinematics import Robot
from so101_scan_hardware.fake_bus import control
from so101_scan_interfaces.msg import ArmSafety, MirrorState
from so101_scan_safety.model import ArmModel
from so101_scan_safety.watch import SafetyWatch

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
DOWN = [0.0, -8.3, 60.5, -52.2, -87.2]
INTO_THE_TABLE = [0.0, 60.0, 30.0, 60.0, 0.0]
TICKS_PER_RAD = 4096 / (2 * math.pi)
DOMAIN = 10 + os.getpid() % 20


@pytest.fixture(scope="module")
def env():
    e = dict(os.environ)
    e["ROS_DOMAIN_ID"] = str(DOMAIN)
    return e


@pytest.fixture(scope="module")
def stack(tmp_path_factory, env):
    tmp = tmp_path_factory.mktemp("safety")
    link = tmp / "so101"
    bus = subprocess.Popen(["ros2", "run", "so101_scan_hardware", "sts_fake_bus", "--link", str(link)],
                           env=env, stdout=subprocess.DEVNULL, start_new_session=True)
    deadline = time.monotonic() + 20
    while not link.exists():
        assert time.monotonic() < deadline and bus.poll() is None, "the fake servo bus didn't start"
        time.sleep(0.1)
    cal = tmp / "calibration.yaml"
    cal.write_text("joints:\n" + "".join(
        "  %s: {id: %d, zero_ticks: 2048, sign: 1, min_ticks: 200, max_ticks: 3900}\n" % (n, i + 1)
        for i, n in enumerate(ARM)))
    log = open(tmp / "launch.log", "w")
    launch = subprocess.Popen(
        ["ros2", "launch", "so101_scan_bringup", "scan_arm.launch.py", "port:=%s" % link,
         "calibration_file:=%s" % cal, "mirror:=fake"],
        env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    yield tmp, str(link) + ".ctl"
    os.killpg(launch.pid, signal.SIGINT)
    try:
        launch.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(launch.pid, signal.SIGKILL)
    log.close()
    os.killpg(bus.pid, signal.SIGINT)
    bus.wait(timeout=10)


class Watch:
    """The latest joints, mirror state and safety state, and the services, from this process."""

    def __init__(self, context):
        self.node = rclpy.create_node("safety_test", context=context)
        self.joints, self.mirror, self.urdf = {}, None, None
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.joints.update(dict(zip(m.name, m.position))), 50)
        self.node.create_subscription(MirrorState, "/scan_mirror/state", self.on_mirror, 10)
        self.safety = SafetyWatch(self.node)
        self.estop = self.node.create_client(Trigger, "/arm_safety/estop")
        self.trajectory = ActionClient(self.node, FollowJointTrajectory,
                                       "/arm_controller/follow_joint_trajectory")

    def on_mirror(self, msg):
        self.mirror = msg

    @property
    def state(self):
        return self.safety.state

    def arm(self):
        return [self.joints[j] for j in ARM]

    def call(self, future, timeout=10.0):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        assert done.wait(timeout), "no answer"
        return future.result()


def wait_for(predicate, timeout=10.0, what=""):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, "timed out waiting for " + what
        time.sleep(0.05)


@pytest.fixture(scope="module")
def ros(stack):
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=DOMAIN)
    watch = Watch(context)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2, context=context)
    executor.add_node(watch.node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    wait_for(lambda: watch.state is not None and watch.state.state == ArmSafety.OK and len(watch.joints) >= 5,
             60.0, "the driver (launch log: %s)" % (stack[0] / "launch.log"))
    yield watch
    executor.shutdown()
    spinner.join(timeout=5.0)
    watch.trajectory.destroy()
    watch.node.destroy_node()
    rclpy.shutdown(context=context)


@pytest.fixture(scope="module")
def model(stack, ros):
    import xacro
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory("so101_scan_description")
    return ArmModel(Robot(xacro.process_file(os.path.join(share, "urdf", "so101_scan.urdf.xacro"),
                                             mappings={"use_mock_hardware": "true"}).toxml()))


def ros2_run(env, *args, timeout=60):
    run = subprocess.run(["ros2", "run"] + list(args), env=env, capture_output=True, text=True, timeout=timeout)
    return run.returncode, run.stdout + run.stderr


def move_arm(env, goal_deg, speed=60):
    return ros2_run(env, "so101_scan_sweep", "move_arm", "--speed-deg", str(speed),
                    "--joints-deg", *["%.3f" % g for g in goal_deg])


def reset(env, ros):
    code, out = ros2_run(env, "so101_scan_safety", "arm_estop", "--reset")
    assert code == 0, out
    wait_for(lambda: ros.state.state == ArmSafety.OK, 5.0, "the driver to follow the controller again")
    return out


@pytest.fixture(autouse=True)
def recover(env, ros):
    """Each test starts with the arm following the controller, whatever the last one left."""
    yield
    if ros.state.state != ArmSafety.OK:
        ros2_run(env, "so101_scan_safety", "arm_estop", "--reset")


def assert_still(ros, seconds=1.0):
    before = ros.arm()
    time.sleep(seconds)
    assert max(abs(a - b) for a, b in zip(ros.arm(), before)) < 0.003


def test_estop_stops_a_scan_and_reset_moves_again(stack, env, ros):
    tmp, _ = stack
    plan = tmp / "long.yaml"
    plan.write_text(yaml.safe_dump({
        "name": "estop_test",
        "move": {"max_joint_speed_deg": 90, "min_move_s": 0.5, "settle_s": 0.2},
        "sweep": {"start_angle_deg": -2.0, "steps_per_line": 2, "n_lines": 400, "line_period_s": 0.02},
        "viewpoints": [{"name": "down", "joints_deg": dict(zip(ARM, DOWN))}],
    }))
    out = tmp / "scan"
    proc = subprocess.Popen(["ros2", "run", "so101_scan_sweep", "scan_sweep", "--plan", str(plan),
                             "--output", str(out)], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, start_new_session=True)
    try:
        wait_for(lambda: (ros.mirror is not None and ros.mirror.state == "scanning") or proc.poll() is not None,
                 60.0, "a sweep")
        assert proc.poll() is None, proc.stdout.read()
        time.sleep(1.0)
        res = ros.call(ros.estop.call_async(Trigger.Request()))
        assert res.success, res.message
        output = proc.communicate(timeout=30)[0]
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGKILL)
    assert proc.returncode != 0, output
    assert "the arm stopped: holding: e-stop (/arm_safety/estop)" in output, output
    assert "arm_estop --reset" in output
    wait_for(lambda: not ros.mirror.busy, 5.0, "the mirror to stop")
    info = json.loads((out / "scan.json").read_text())
    assert info["arm_stopped"] == "holding: e-stop (/arm_safety/estop)"
    assert 0 < info["lines"] < 400
    assert ros.state.state == ArmSafety.HOLDING
    assert_still(ros)

    goal = [math.degrees(q) for q in ros.arm()]
    goal[0] += 10.0
    code, output = move_arm(env, goal)
    assert code == 1 and "the arm is stopped (holding: e-stop (/arm_safety/estop))" in output, output

    output = reset(env, ros)
    assert "now: ok" in output
    code, output = move_arm(env, goal)
    assert code == 0 and "arrived" in output, output
    assert math.degrees(ros.joints["shoulder_pan"]) == pytest.approx(goal[0], abs=0.5)


def test_estop_topic_and_torque_off(stack, env, ros):
    pub = subprocess.run(["ros2", "topic", "pub", "--once", "/estop", "std_msgs/msg/Bool", "{data: true}"],
                         env=env, capture_output=True, text=True, timeout=30)
    assert pub.returncode == 0, pub.stdout + pub.stderr
    wait_for(lambda: ros.state.state == ArmSafety.HOLDING, 5.0, "the /estop stop")
    assert ros.state.reason == "e-stop (/estop)"
    code, output = ros2_run(env, "so101_scan_safety", "arm_estop", "--torque-off")
    assert code == 0, output
    wait_for(lambda: ros.state.state == ArmSafety.TORQUE_OFF, 5.0, "the motors to go off")
    assert ros.state.reason.startswith("e-stop (/estop); then e-stop with the motors off")
    code, output = ros2_run(env, "so101_scan_safety", "arm_estop", "--status")
    assert "torque off: e-stop (/estop)" in output, output
    reset(env, ros)
    goal = [math.degrees(q) for q in ros.arm()]
    goal[0] -= 10.0
    code, output = move_arm(env, goal)
    assert code == 0, output


def test_a_blocked_joint_stalls_and_the_arm_holds(stack, env, ros):
    _, ctl = stack
    pan = ros.joints["shoulder_pan"]
    ticks = int(round(2048 + pan * TICKS_PER_RAD))
    control(ctl, 1, obstacle=[0, ticks + 60])   # shoulder_pan gets about 5 deg further, then hits something
    try:
        goal = [math.degrees(q) for q in ros.arm()]
        goal[0] += 30.0
        code, output = move_arm(env, goal, speed=20)
        assert code == 1, output
        assert "the arm stopped: holding: shoulder_pan stalled" in output, output
        assert ros.state.state == ArmSafety.HOLDING
        # it holds where it stopped, not at the goal it was pushing towards
        assert math.degrees(ros.joints["shoulder_pan"] - pan) == pytest.approx(60 / TICKS_PER_RAD * 180 / math.pi,
                                                                               abs=1.0)
        assert_still(ros)
        # the cause is gone once the arm holds: a reset works with the obstacle still there
    finally:
        control(ctl, 1, obstacle=None)
    reset(env, ros)
    assert ros.state.state == ArmSafety.OK


def test_soft_limits_keep_the_head_off_the_table(stack, env, ros, model):
    code, output = move_arm(env, DOWN)
    assert code == 0, output
    start = ros.arm()

    code, output = move_arm(env, INTO_THE_TABLE)
    assert code == 1 and "the arm's soft limits would stop it short" in output, output
    assert "viewpoint goal: the head goes 97.5 mm below the table top" in output
    assert "moving from the start to goal: the head comes within" in output
    assert_still(ros, 0.3)
    assert max(abs(a - b) for a, b in zip(ros.arm(), start)) < 0.003

    # straight to the controller, past the check: the driver stops the head at the clearance
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ARM
    goal.trajectory.points = [JointTrajectoryPoint(
        positions=[math.radians(v) for v in INTO_THE_TABLE], velocities=[0.0] * 5,
        time_from_start=rclpy.duration.Duration(seconds=3.0).to_msg())]
    assert ros.trajectory.wait_for_server(timeout_sec=5.0)
    handle = ros.call(ros.trajectory.send_goal_async(goal))
    assert handle.accepted
    future = handle.get_result_async()
    warnings = set()
    deadline = time.monotonic() + 20.0
    while not future.done():   # what the driver said while it held the arm back
        assert time.monotonic() < deadline, "no result from arm_controller"
        warnings.update(ros.state.warnings)
        time.sleep(0.02)
    assert future.result().result.error_code != FollowJointTrajectory.Result.SUCCESSFUL   # it never got there
    assert any(w.startswith("soft limit: the head would come within") for w in warnings), warnings
    time.sleep(0.5)
    e = model.evaluate(dict(zip(ARM, ros.arm())))
    assert -0.002 < e.workspace_margin < 0.01, e
    assert e.workspace_limit == "table"
    assert ros.state.state == ArmSafety.OK      # a soft limit holds back; it doesn't stop the arm

    # and moving back out is always allowed
    code, output = move_arm(env, DOWN)
    assert code == 0, output
    assert [math.degrees(q) for q in ros.arm()] == pytest.approx(DOWN, abs=0.5)
