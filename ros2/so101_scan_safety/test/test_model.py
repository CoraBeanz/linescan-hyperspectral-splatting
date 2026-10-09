"""The soft-limit model against the CAD report's gravity numbers, and the plan checks."""

import json
import math
import os
import re
import tempfile
from pathlib import Path

import pytest
import xacro

from so101_scan_description.kinematics import Robot
from so101_scan_safety import check
from so101_scan_safety.model import ARM_JOINTS, ArmModel, Limits

PKG = Path(__file__).resolve().parent.parent
ROS2 = PKG.parent
DESCRIPTION = ROS2 / "so101_scan_description"
PLANS = ROS2 / "so101_scan_sweep" / "plans"
CAD_REPORT = ROS2.parent / "cad" / "build_report.json"


@pytest.fixture(scope="module")
def prefix():
    """An ament prefix with the description package from the source tree in it."""
    prefix = Path(tempfile.mkdtemp())
    (prefix / "share" / "ament_index" / "resource_index" / "packages").mkdir(parents=True)
    (prefix / "share" / "ament_index" / "resource_index" / "packages" / "so101_scan_description").touch()
    (prefix / "share" / "so101_scan_description").symlink_to(DESCRIPTION)
    return prefix


@pytest.fixture(scope="module")
def robot(prefix):
    """The URDF from the source tree, whether or not the description package is installed."""
    old = os.environ.get("AMENT_PREFIX_PATH", "")
    os.environ["AMENT_PREFIX_PATH"] = str(prefix) + os.pathsep + old
    try:
        return Robot(xacro.process_file(str(DESCRIPTION / "urdf" / "so101_scan.urdf.xacro"),
                                        mappings={"use_mock_hardware": "true"}).toxml())
    finally:
        os.environ["AMENT_PREFIX_PATH"] = old


@pytest.fixture(scope="module")
def model(robot):
    return ArmModel(robot)


def deg(*values):
    return {j: math.radians(v) for j, v in zip(ARM_JOINTS, values)}


def test_limits_come_from_the_urdf(model):
    lim = model.limits
    assert lim.table_clearance == pytest.approx(0.01)
    assert lim.base_keepout_radius == pytest.approx(0.08)
    assert lim.stall_torque == pytest.approx(1.62)
    assert (lim.warn_gravity_load, lim.max_gravity_load) == pytest.approx((0.5, 0.6))
    assert lim.workspace_links == ["upper_arm_link", "lower_arm_link", "wrist_link", "wrist_roll_link"]
    assert lim.workspace_box_link == "scan_head_link"


def test_limits_fall_back_to_the_drivers_defaults():
    bare = Robot('<robot name="r"><link name="base_link"/></robot>')
    lim = Limits.from_urdf(bare)
    assert lim.table_z == pytest.approx(-0.0024) and lim.max_gravity_load == pytest.approx(0.6)


@pytest.mark.skipif(not CAD_REPORT.exists(), reason="no cad/build_report.json beside ros2/")
def test_gravity_matches_the_cad_report(model):
    """The URDF's masses are the CAD model's, so the torques match its report: at its reference
    pose, and stretched out where shoulder_lift holds the most."""
    report = json.loads(CAD_REPORT.read_text())["torque_Nm"]
    assert model.limits.stall_torque == pytest.approx(report["stall_Nm"])
    e = model.evaluate(deg(*(report["pose"][j] for j in ARM_JOINTS)))
    for j in ARM_JOINTS:
        assert abs(e.torque[j]) == pytest.approx(abs(report["with_head"][j]), abs=0.002), j
    # worst_head: [N m, [shoulder_lift, elbow_flex, wrist_flex] deg]
    torque, (lift, elbow, flex) = report["worst_head"]["shoulder_lift"]
    e = model.evaluate(deg(0, lift, elbow, flex, 0))
    assert abs(e.torque["shoulder_lift"]) == pytest.approx(torque, abs=0.002)


def test_the_stretched_out_arm_is_over_the_gravity_budget(model):
    e = model.evaluate(deg(0, 75, -75, 0, 0))
    # about 75%: the head's mass and centre of mass come from the CAD's mass budget, which moves a little
    load = e.gravity_load["shoulder_lift"]
    assert e.worst_gravity()[0] == "shoulder_lift" and load == pytest.approx(0.75, abs=0.02)
    assert model.problems(e) == ["shoulder_lift holds %.0f%% of its stall torque against gravity (limit 60%%)"
                                 % (100 * load)]
    # at zero (straight out along the table) it holds half of stall: just under the warning level
    zero = model.evaluate(deg(0, 0, 0, 0, 0))
    assert model.problems(zero) == [] and zero.workspace_margin > 0.1
    assert zero.gravity_load["shoulder_lift"] == pytest.approx(0.5, abs=0.01)


def test_workspace_table_and_base(model):
    into_table = model.evaluate(deg(0, 60, 30, 60, 0))
    assert into_table.workspace_margin < -0.05
    assert (into_table.workspace_limit, into_table.workspace_part) == ("table", "head")
    assert model.problems(into_table) == ["the head goes %.1f mm below the table top"
                                          % (-(into_table.workspace_margin + 0.01) * 1e3)]
    folded = model.evaluate(deg(0, 0, 90, 90, 0))   # the head tucked back over the base
    assert folded.workspace_margin < 0 and folded.workspace_limit == "base"
    assert "space kept clear around the base" in model.problems(folded)[0]
    # straight up and folded over stays clear of both
    assert model.evaluate(deg(0, -90, 90, 0, 0)).workspace_margin > 0.02


@pytest.mark.parametrize("plan", sorted(p.name for p in PLANS.glob("*.yaml")))
def test_the_shipped_plans_pass(model, plan):
    report = check.check_plan(model, check.viewpoints_from_yaml(str(PLANS / plan)), start=deg(0, 0, 0, 0, 0))
    assert report.ok, report.problems
    assert report.rows and all(e.workspace_margin > 0.02 for _, e in report.rows)
    assert "ok: the driver will let this plan through" in check.format_report(model, report)


def test_a_move_stops_where_the_driver_would(model):
    """Into the table: the check finds the step where the driver holds the arm back, just
    above the clearance, and says so as well as that the viewpoint itself is out."""
    a, b = deg(0, -8.3, 60.5, -52.2, -87.2), deg(0, 60, 30, 60, 0)
    stop = check.check_move(model, a, b)
    assert stop is not None
    q, e = stop
    assert e.workspace_margin < 0 and e.workspace_limit == "table"
    before = check.path(a, b)[check.path(a, b).index(q) - 1]
    assert 0 <= model.evaluate(before).workspace_margin < 0.005
    report = check.check_plan(model, [("down", a), ("low", b)])
    # about 97.5 mm: the head's collision box is centred on its centre of mass, which moves a little
    # whenever the CAD's mass budget changes
    low = re.match(r"viewpoint low: the head goes ([\d.]+) mm below the table top", report.problems[0])
    assert low and float(low.group(1)) == pytest.approx(97.5, abs=1.0), report.problems
    assert report.problems[1].startswith("moving from down to low: the head comes within")


def test_path_steps_and_ends_at_the_goal():
    q0, q1 = deg(0, 0, 0, 0, 0), deg(10, -5, 0, 0, 2.5)
    steps = check.path(q0, q1)
    assert len(steps) == 10 and steps[-1] == pytest.approx(q1)
    assert max(abs(steps[1][j] - steps[0][j]) for j in ARM_JOINTS) <= math.radians(1.0) + 1e-12


def test_blocks_lets_the_arm_move_back_in(model):
    """The driver refuses a step that goes further out, never one that comes back."""
    out_far, out_less = model.evaluate(deg(0, 75, -75, 0, 0)), model.evaluate(deg(0, 70, -70, 0, 0))
    assert model.blocks(out_far, out_less)
    assert not model.blocks(out_less, out_far)


def test_a_bad_plan_fails_from_the_command_line(tmp_path, monkeypatch, robot, capsys):
    plan = tmp_path / "bad.yaml"
    plan.write_text("viewpoints:\n"
                    "  - name: reach\n"
                    "    joints_deg: {shoulder_pan: 0, shoulder_lift: 75, elbow_flex: -75, wrist_flex: 0, wrist_roll: 0}\n")
    monkeypatch.setattr(check, "load_robot", lambda head: (robot, ""))
    assert check.main(["--plan", str(plan)]) == 1
    out = capsys.readouterr().out
    held = re.search(r"PROBLEM: viewpoint reach: shoulder_lift holds (\d+)%", out)
    assert held and int(held.group(1)) == pytest.approx(75, abs=2), out
    assert check.main(["--plan", str(PLANS / "ring.yaml")]) == 0


def test_the_head_is_where_the_launch_puts_it(prefix, robot, tmp_path, monkeypatch):
    """check_plan builds the URDF as scan_arm.launch.py does: the CAD head unless a head
    calibration exists, and a named file that doesn't is an error."""
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(prefix) + os.pathsep + os.environ.get("AMENT_PREFIX_PATH", ""))
    monkeypatch.setattr(check, "HEAD_CALIBRATION", str(tmp_path / "head_calibration.yaml"))
    cad, head = check.load_robot(check.HEAD_CALIBRATION)   # the default file, absent: the CAD numbers
    assert head == "" and cad.fk("scan_head_link", deg(0, 0, 0, 0, 0)) == pytest.approx(
        robot.fk("scan_head_link", deg(0, 0, 0, 0, 0)))
    assert check.load_robot("none")[1] == ""
    with pytest.raises(SystemExit, match="no head calibration at"):
        check.load_robot(str(tmp_path / "missing.yaml"))
