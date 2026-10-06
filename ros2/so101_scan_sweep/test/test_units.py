"""The parts that need no ROS graph: protocol, clock mapping, plans, line poses, the planner."""

import math
import os
import random
import tempfile
from pathlib import Path

import numpy as np
import pytest
import xacro
import yaml

from so101_scan_description.kinematics import Robot
from so101_scan_sweep import make_plan as planner
from so101_scan_sweep import mirror_protocol as mp
from so101_scan_sweep import plan as plan_mod
from so101_scan_sweep.clock_sync import ClockSync
from so101_scan_sweep.line_log import CSV_COLUMNS, JointBuffer, LinePoser, LinesCsv, quaternion
from so101_scan_sweep.plan import ARM_JOINTS

PKG = Path(__file__).resolve().parent.parent
DESCRIPTION = PKG.parent / "so101_scan_description"


@pytest.fixture(scope="module")
def robot():
    """The URDF from the source tree, whether or not the description package is installed."""
    prefix = Path(tempfile.mkdtemp())
    (prefix / "share" / "ament_index" / "resource_index" / "packages").mkdir(parents=True)
    (prefix / "share" / "ament_index" / "resource_index" / "packages" / "so101_scan_description").touch()
    (prefix / "share" / "so101_scan_description").symlink_to(DESCRIPTION)
    old = os.environ.get("AMENT_PREFIX_PATH", "")
    os.environ["AMENT_PREFIX_PATH"] = str(prefix) + os.pathsep + old
    try:
        return Robot(xacro.process_file(str(DESCRIPTION / "urdf" / "so101_scan.urdf.xacro"),
                                        mappings={"use_mock_hardware": "true"}).toxml())
    finally:
        os.environ["AMENT_PREFIX_PATH"] = old


# --- protocol -----------------------------------------------------------------------------

def test_commands():
    assert mp.command("SCAN", mode="stare", start=-107, step=2, lines=107, period=1e6 / 30, id="7") == \
        "SCAN mode=stare start=-107 step=2 lines=107 period=33333.333333 id=7\n"
    assert mp.command("move", pos=-12, v=None) == "MOVE pos=-12\n"
    assert mp.command("SCAN", period=0.0333 * 1e6) == "SCAN period=33300\n"
    with pytest.raises(mp.ProtocolError):
        mp.command("CFG", note="two words")


def test_replies_and_events():
    m = mp.parse("EV LINE n=3 t=5343950 pos=-14 ready=5347600\r\n")
    assert (m.kind, m.name, m.int("n"), m.int("pos"), m.int("ready"), m.id) == ("EV", "LINE", 3, -14, 5347600, None)
    m = mp.parse("OK SCAN mode=stare t0=5243950 period=33333.333 start=-20 step=2 lines=5 id=3")
    assert m.kind == "OK" and m.name == "SCAN" and m.id == "3" and m.float("period") == pytest.approx(33333.333)
    m = mp.parse("ERR MOVE code=range id=9 msg=target 4000 is outside min..max -3200..3200")
    assert (m.kind, m.get("code"), m.id, m.get("msg")) == ("ERR", "range", "9", "target 4000 is outside min..max -3200..3200")
    assert mp.parse("EV FAULT code=drv_ot t=12 msg=TMC2209 overheated").get("msg") == "TMC2209 overheated"
    with pytest.raises(mp.ProtocolError):
        mp.parse("EV LINE n=x").int("n")
    for bad in ["", "ets Jun  8 2016 00:22:57", "OK", "PONG 1 2", "ok PING t=1"]:
        with pytest.raises(mp.ProtocolError):
            mp.parse(bad)


def test_finds_the_message_after_boot_garbage():
    boot = "EV BOOT fw=0.1.0 proto=1 reset=poweron drv=ok t=301234"
    # the boot ROM's bytes at the wrong baud rate, glued to EV BOOT, after a line the reset cut off
    for line in [boot, "\x00\ufffd\ufffd" + boot, "EV LINE n=5 t=12\x00\ufffd" + boot, "x\ufffd" + boot]:
        assert mp.find_message(line) == boot
    msg = "ERR SCAN code=range id=4 msg=scan covers -20..4000, outside min..max -3200..3200"
    assert mp.find_message(msg) == msg
    msg = "ERR MOVE code=busy id=7 msg=wait for EV MOVED or send STOP"   # protocol words in the text
    assert mp.find_message(msg + "\r") == msg
    assert mp.find_message("OK PING t=12 id=OK") == "OK PING t=12 id=OK"   # not inside a value
    assert mp.find_message("\x00\ufffd\x00") is None and mp.find_message("BREVITY OK") is None


# --- clock mapping ------------------------------------------------------------------------

def test_clock_sync_follows_offset_and_drift():
    rng = random.Random(3)
    offset_us, drift = 123_456_789, 40e-6        # ESP32 clock starts elsewhere and runs 40 ppm fast
    sync = ClockSync()

    def esp(ros_ns):
        return int(offset_us + ros_ns / 1000 * (1 + drift))

    ros = 1_700_000_000_000_000_000
    for _ in range(60):                          # a minute of pings, 0.3-1.5 ms each way, some late
        out = rng.uniform(0.3e6, 1.5e6) + (rng.random() < 0.2) * rng.uniform(2e6, 8e6)
        back = rng.uniform(0.3e6, 1.5e6)
        t_send = ros
        t_esp = esp(t_send + out)
        sync.add(t_send, int(t_send + out + back), t_esp)
        ros += 1_000_000_000
    for later_s in (0.0, 5.0):                   # now and 5 s after the last ping
        truth = ros + int(later_s * 1e9)
        assert abs(sync.to_ros_ns(esp(truth)) - truth) < 0.6e6  # well under a millisecond


def test_clock_sync_resets_when_esp32_reboots():
    sync = ClockSync(min_samples=1)
    sync.add(0, 1_000_000, 500_000_000)
    sync.add(1_000_000_000, 1_001_000_000, 1_000)   # its clock went back to zero
    assert len(sync.samples) == 1
    assert abs(sync.to_ros_ns(1_000) - 1_000_500_000) < 1000


# --- plans --------------------------------------------------------------------------------

def test_example_plans_load(robot):
    files = sorted((PKG / "plans").glob("*.yaml"))
    assert files
    for f in files:
        plan = plan_mod.load(f)
        assert plan.viewpoints and not plan_mod.check_limits(plan, robot.limits()), f.name


def test_plan_defaults_and_overrides():
    plan = plan_mod.from_dict({"viewpoints": [
        {"name": "a", "joints_deg": {j: 0 for j in ARM_JOINTS}},
        {"name": "b", "sweep": {"n_lines": 5, "start_angle_deg": 3}}]})
    a, b = plan.viewpoints
    assert a.sweep.n_lines == 107 and b.sweep.n_lines == 5
    assert b.sweep.start_angle == pytest.approx(math.radians(3)) and b.joints == {}
    assert plan.max_joint_speed == pytest.approx(math.radians(30))


@pytest.mark.parametrize("bad", [
    {},
    {"viewpoints": [{"joints_deg": {"shoulder_pan": 0}}]},
    {"viewpoints": [{"sweep": {"n_lines": 0}}]},
    {"viewpoints": [{"sweep": {"lines": 4}}]},
])
def test_bad_plans(bad):
    with pytest.raises(plan_mod.PlanError):
        plan_mod.from_dict(bad)


# --- line poses ---------------------------------------------------------------------------

def test_joint_buffer_interpolates():
    buf = JointBuffer()
    assert buf.at(5) == (None, None)
    assert not buf.add(0, ["shoulder_pan"], [0.0])  # partial message, e.g. the mirror's
    buf.add(1000, ARM_JOINTS, [0.0] * 5)
    buf.add(2000, ARM_JOINTS + ["scan_mirror_joint"], [1.0, 2.0, 3.0, 4.0, 5.0, 9.0])
    q, gap = buf.at(1250)
    assert q["elbow_flex"] == pytest.approx(0.75) and gap == 250
    q, gap = buf.at(5000)
    assert q["wrist_roll"] == 5.0 and gap == 3000


def test_quaternion_round_trip():
    from so101_scan_description.kinematics import axis_angle
    rng = np.random.default_rng(2)
    for _ in range(50):
        axis, angle = rng.normal(size=3), rng.uniform(-math.pi, math.pi)
        x, y, z, w = quaternion(axis_angle(axis, angle))
        r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        np.testing.assert_allclose(r, axis_angle(axis, angle), atol=1e-9)


def test_line_poser_and_csv(robot, tmp_path):
    poser = LinePoser(robot)
    assert poser.scene_distance == pytest.approx(0.15) and poser.half_line == pytest.approx(0.020937)
    q = dict(zip(ARM_JOINTS, np.radians([0, 0, 50.3, -50.3, -87.2])))
    head, cam = poser.poses(q, 0.0)
    np.testing.assert_allclose(cam[:3, 2], [0, 0, -1], atol=0.01)       # looking down
    ends = poser.line_ends(cam)
    assert np.linalg.norm(ends[1] - ends[0]) == pytest.approx(2 * 0.020937)
    # the mirror turns the view twice as far
    _, cam2 = poser.poses(q, math.radians(5))
    assert math.degrees(math.acos(np.dot(cam[:3, 2], cam2[:3, 2]))) == pytest.approx(10.0, abs=1e-6)
    out = LinesCsv(tmp_path / "lines.csv")
    out.write(0, 7, 3, 1234, 5678, True, 0.0, head, cam, q)
    out.close()
    header, row = (tmp_path / "lines.csv").read_text().splitlines()
    assert header.split(",") == CSV_COLUMNS and row.split(",")[:6] == ["0", "7", "3", "1234", "5678", "1"]


# --- viewpoint planner --------------------------------------------------------------------

def test_make_plan_views_look_at_the_target(robot):
    target = np.array([0.26, 0.0, planner.TABLE_Z])
    notes = []
    plan = planner.make_plan(robot, target, [0, 25], [0, 90, 180, 270], "ring", log=notes.append)
    assert len(plan["viewpoints"]) >= 3, notes
    loaded = plan_mod.from_dict(yaml.safe_load(yaml.safe_dump(plan)))
    assert not plan_mod.check_limits(loaded, robot.limits())
    for v in loaded.viewpoints:
        line = robot.fk("scan_line_frame", v.joints)
        np.testing.assert_allclose(line[:3, 3], target, atol=2e-4)  # joints are rounded to 0.01 deg
        assert planner.lowest_point(robot, v.joints) > planner.TABLE_Z + 0.009
    down = loaded.viewpoints[0]
    np.testing.assert_allclose(robot.fk("scan_line_frame", down.joints)[:3, 2], [0, 0, -1], atol=1e-3)
