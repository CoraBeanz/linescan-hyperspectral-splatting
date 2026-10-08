"""Coverage planning around an object, and the checks a dry run makes, without a ROS graph."""

import math
import os
import tempfile
from pathlib import Path

import numpy as np
import pytest
import xacro
import yaml

from so101_scan_description.kinematics import Robot
from so101_scan_sweep import coverage as cov
from so101_scan_sweep import dry_run
from so101_scan_sweep import make_plan as planner
from so101_scan_sweep import plan as plan_mod
from so101_scan_sweep.line_log import LinePoser
from so101_scan_sweep.plan import ARM_JOINTS

DESCRIPTION = Path(__file__).resolve().parent.parent.parent / "so101_scan_description"
BOX = cov.Box.standing(0.26, 0.0, [0.04, 0.04, 0.02], table_z=planner.TABLE_Z)


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


@pytest.fixture(scope="module")
def coverage_plan(robot):
    notes = []
    plan = planner.make_coverage_plan(robot, BOX, [0, 50], [90, 180, 270], "box", log=notes.append)
    return plan, notes


def default_sweep():
    return plan_mod.from_dict({"viewpoints": [{"name": "x"}]}).viewpoints[0].sweep


def test_box_surface_and_rays():
    pts, normals, faces = BOX.surface(0.005)
    assert (faces == "top").sum() == 64 and (faces == "+x").sum() == 32 and len(pts) == 64 + 4 * 32
    assert np.allclose(pts[faces == "top"][:, 2], BOX.hi[2]) and (normals[faces == "-y"] == [0, -1, 0]).all()
    assert BOX.contains(pts, 1e-9).all() and not BOX.contains(pts + normals * 0.001).any()
    assert BOX.lo[2] == pytest.approx(planner.TABLE_Z)
    np.testing.assert_allclose(BOX.ray_hit(np.array([0.26, 0.0, 1.0]), np.array([0, 0, -1.0])),
                               [0.26, 0.0, BOX.hi[2]])
    np.testing.assert_allclose(BOX.ray_hit(np.array([0.0, 0.01, 0.0]), np.array([1.0, 0, 0])), [0.24, 0.01, 0.0])
    assert BOX.ray_hit(np.array([0.0, 0.1, 0.0]), np.array([1.0, 0, 0])) is None
    assert BOX.ray_hit(np.array([0.26, 0.0, 1.0]), np.array([0, 0, 1.0])) is None    # pointing away


def test_a_view_sees_what_faces_it_in_focus(robot):
    poser = LinePoser(robot)
    pts, normals, faces = BOX.surface(0.005)
    down, why = planner.solve_view(robot, np.array([0.26, 0.0, BOX.hi[2]]), cov.view_direction(0, 0))
    assert down is not None, why
    seen = cov.sweep_sees(poser, down, default_sweep(), pts, normals)
    assert seen[faces == "top"].all() and not seen[faces != "top"].any()     # the sides are edge on
    # out of focus: the same view 60 mm higher sees nothing
    high, _ = planner.solve_view(robot, np.array([0.26, 0.0, BOX.hi[2] - 0.06]), cov.view_direction(0, 0))
    assert not cov.sweep_sees(poser, high, default_sweep(), pts, normals).any()
    # leaning 50 deg toward -y (azimuth 270) shows the +y side; at 35 deg it sees that side at
    # over 60 deg, too obliquely
    side, why = planner.solve_view(robot, np.array([0.26, 0.0, BOX.hi[2]]), cov.view_direction(math.radians(50),
                                                                                                 math.radians(270)))
    assert side is not None, why
    seen = cov.sweep_sees(poser, side, default_sweep(), pts, normals)
    assert seen[faces == "+y"].mean() > 0.5 and not seen[faces == "-y"].any()
    steep, _ = planner.solve_view(robot, np.array([0.26, 0.0, BOX.hi[2]]), cov.view_direction(math.radians(35),
                                                                                                math.radians(270)))
    assert not cov.sweep_sees(poser, steep, default_sweep(), pts, normals)[faces == "+y"].any()
    assert cov.sweep_sees(poser, steep, default_sweep(), pts, normals, max_incidence=math.radians(70))[
        faces == "+y"].any()
    # a short sweep sees a narrower strip than a long one
    short = plan_mod.from_dict({"sweep": {"n_lines": 21, "start_angle_deg": -1.2},
                                "viewpoints": [{"name": "x"}]}).viewpoints[0].sweep
    assert cov.sweep_sees(poser, down, short, pts, normals).sum() < seen.size


def test_coverage_plan_sees_the_box_from_every_reachable_side(robot, coverage_plan):
    plan, notes = coverage_plan
    c = plan["coverage"]
    assert c["faces"]["top"]["seen"] == 1.0 and c["faces"]["top"]["seen_enough"] == 1.0
    for face in ("+x", "+y", "-y"):
        assert c["faces"][face]["seen"] == pytest.approx(c["faces"][face]["seeable"]), (face, c, notes)
    assert c["faces"]["+y"]["seen"] > 0.5 and c["faces"]["-y"]["seen"] > 0.5
    loaded = plan_mod.from_dict(yaml.safe_load(yaml.safe_dump(plan)))
    assert not plan_mod.check_limits(loaded, robot.limits())
    assert len(loaded.viewpoints) <= 12 and len(loaded.viewpoints) == len(plan["viewpoints"]) >= 3
    for v, src in zip(loaded.viewpoints, plan["viewpoints"]):
        np.testing.assert_allclose(robot.fk("scan_line_frame", v.joints)[:3, 3], src["aim_m"], atol=5e-4)
        assert planner.lowest_point(robot, v.joints) > planner.TABLE_Z + 0.009
        assert not BOX.contains(planner.body_points(robot, v.joints, dense=True), 0.009).any()
    # it starts at the most nearly straight-down view, and the stats are what the views see
    tilts = [math.acos(-robot.fk("scan_line_frame", v.joints)[2, 2]) for v in loaded.viewpoints]
    assert tilts[0] == pytest.approx(min(tilts), abs=1e-3)
    pts, normals, _ = BOX.surface(0.005)
    count = np.sum([cov.sweep_sees(LinePoser(robot), v.joints, v.sweep, pts, normals) for v in loaded.viewpoints],
                   axis=0)
    assert (count > 0).mean() == pytest.approx(c["seen"], abs=0.01)


def test_dry_run_checks(robot, coverage_plan):
    plan = plan_mod.from_dict(coverage_plan[0])
    surface = BOX.surface()
    checks, problems = dry_run.check_plan(robot, plan, box=BOX, surface=surface, frame_period=1 / 30.0)
    assert not problems and not [p for c in checks for p in c.problems]
    for c in checks:
        assert c.aim_error_m < 5e-4 and c.frames_per_line == 1 and c.period_s == pytest.approx(1 / 30.0)
        assert len(c.line_ends) == plan.viewpoints[c.index].sweep.n_lines and c.seen.any()
    assert checks[0].move_s == 0.0 and all(c.move_s >= plan.min_move_time for c in checks[1:])
    text, every = dry_run.report(plan, checks, [], ["a note"], surface)
    assert not every and "0 problems" in text and "note: a note" in text and "the object: " in text

    # from the arm's zero pose; a line period of two camera frames; the mirror past its limits;
    # a clearance the arm can't keep; a joint past its limit
    zero = {j: 0.0 for j in ARM_JOINTS}
    bad = plan_mod.from_dict(dict(coverage_plan[0], sweep={"start_angle_deg": -50.0, "line_period_s": 0.05}))
    bad.viewpoints[0].joints["wrist_flex"] = math.radians(150)
    checks, _ = dry_run.check_plan(robot, bad, start=zero, frame_period=1 / 30.0, clearance=0.2)
    assert checks[0].move_s > 1.0 and checks[0].frames_per_line == 2
    said = " ".join(checks[0].problems)
    assert "wrist_flex = 150.0 deg is outside" in said and "outside the bridge's -45.84..45.84 deg" in said
    assert "under the 200 mm clearance" in said
    text, every = dry_run.report(bad, checks, ["the stack said no"], [])
    assert len(every) >= 4 and "! the stack said no" in text and "scan_sweep would stop" in text


def test_a_move_that_dips_is_caught(robot):
    """Two poses well clear of the table, with a straight line in joint space between them that
    swings the head through it (found by a random search)."""
    a = {"shoulder_pan": -13, "shoulder_lift": 18, "elbow_flex": 85, "wrist_flex": 79, "wrist_roll": 143}
    b = dict(a, elbow_flex=-11, wrist_flex=-46)
    plan = plan_mod.from_dict({"viewpoints": [{"name": "a", "joints_deg": a}, {"name": "b", "joints_deg": b}]})
    checks, _ = dry_run.check_plan(robot, plan)
    assert not checks[0].problems and checks[0].lowest_m > planner.TABLE_Z + 0.03
    assert planner.lowest_point(robot, plan.viewpoints[1].joints) > planner.TABLE_Z + 0.1
    assert len(checks[1].problems) == 1 and "mm into the table" in checks[1].problems[0], checks[1].problems
    path = dry_run.move_path(plan.viewpoints[0].joints, plan.viewpoints[1].joints)
    assert len(path) == 64 and path[0] == plan.viewpoints[0].joints
