"""The hand-eye calibration, end to end on a simulated rig.

    python -m hsisim handeye                 # about five minutes
    python -m hsisim handeye --keep he_out   # keep the scan, the stills and the results

Scans headcal's calibration plan (calibration/headcal/plans/headcal.yaml) of the tag board with
a head off its CAD numbers as a hand-built one is (--head-errors 1) and the pose camera on, as
the rig would: the arm with its servo offsets, encoder ticks and flex, the mirror on the
camera's clock, the spectrograph through hsical's synthetic instrument. Then it calibrates the
instrument with hsical, bins the lines as line_camera would, runs `headcal solve` on the scan
and the stills, and compares the head it finds with the true one, in headcal's own terms
(headcal.synth.compare): where the line camera's rays meet the board, the pose camera on the
wrist, its lens, the scan line's length.

headcal's own self-test renders an idealised rig (only 0.03 deg of joint noise); here the
arm's errors are the simulator's, which a calibration can't tell from the head's, so expect
the line camera within a couple of millimetres rather than headcal's few tenths. --errors 0
gives a perfect arm.
"""

from __future__ import annotations

import json
import shutil
import tempfile
import time
from pathlib import Path

import numpy as np
import yaml

from . import repo

# every other viewpoint of the plan, and its sweep in quarter as many lines four steps apart
VIEWS_EVERY = 2
LINES, STEPS = 54, 4


def plan_views(every=VIEWS_EVERY):
    with open(repo.HEADCAL_PLANS / "headcal.yaml") as f:
        return [v["name"] for v in yaml.safe_load(f)["viewpoints"]][::every]


def truth(session):
    """headcal.synth.Truth for a session simulated with the pose camera or --head-errors."""
    from headcal import synth
    from headcal.board import Board
    from headcal.model import HeadGeometry, PinholeCamera
    d = json.loads((Path(session) / "truth" / "head.json").read_text())
    return synth.Truth(HeadGeometry.from_json(d["head"]), PinholeCamera.from_json(d["pose_camera_lens"]),
                       Board(**{k: v for k, v in d["board"].items() if k != "format"}), np.array(d["board_in_base"]),
                       slit_reversed=d["slit_reversed"], seed=d["seed"])


def compare(session, result):
    """headcal.synth.compare of a calibration (headcal.solve's result) with the session's truth,
    and of the CAD head, for scale."""
    from headcal import synth
    from headcal.model import HeadGeometry
    t = truth(session)
    plan = yaml.safe_load((Path(session) / "plan.yaml").read_text())
    robot = repo.robot()
    got = synth.compare(t, result, plan=plan, robot=robot)
    cad = HeadGeometry.from_json(json.loads((Path(session) / "truth" / "head.json").read_text())["cad"])
    as_cad = type("CAD", (), dict(head=cad, camera=result.camera, slit_reversed=result.slit_reversed))
    base = synth.compare(t, as_cad, plan=plan, robot=robot)
    return got, base


def logged_line_errors(session, head_calibration=None, every=4):
    """How far from where it really was the scan line lands when posed from what the rig logged
    (lines.csv's joints and mirror angles) through a head: headcal's calibration
    (head_calibration.yaml), or the CAD head when None. This is what `scan_to_dataset --urdf`
    gives the trainer, the arm's own errors included. Points along the slit (h = -0.9, 0, 0.9)
    of every `every`th line, met with the board's plane; returns the distances in mm."""
    import csv
    from headcal import geometry as g
    from headcal.model import HeadGeometry
    from so101_scan_sweep.line_log import LinePoser
    from so101_scan_sweep.plan import ARM_JOINTS
    session = Path(session)
    t = truth(session)
    plane_n, plane_p = t.board_in_base[:3, 2], t.board_in_base[:3, 3]
    robot = repo.robot(head_calibration)
    head = HeadGeometry.from_urdf(robot)
    if head_calibration:   # slit_k1 isn't in the URDF
        head.slit_k1 = float(yaml.safe_load(Path(head_calibration).read_text())["slit_k1"])
    poser = LinePoser(robot)
    with open(session / "lines.csv", newline="") as f:
        logged = list(csv.DictReader(f))
    with open(session / "truth" / "lines_true.csv", newline="") as f:
        true = list(csv.DictReader(f))
    hs = np.array([-0.9, 0.0, 0.9])

    def hit(cam, h, geom):
        hu = h.copy()
        for _ in range(4):
            hu = hu - (hu + geom.slit_k1 * hu ** 3 - h) / (1.0 + 3.0 * geom.slit_k1 * hu ** 2)
        d = np.stack([hu * geom.k, np.zeros_like(hu), np.ones_like(hu)], -1) @ cam[:3, :3].T
        s = ((plane_p - cam[:3, 3]) @ plane_n) / (d @ plane_n)
        return cam[:3, 3] + s[:, None] * d

    def pose(row, prefix):
        return g.make(g.quat_matrix([float(row[prefix + k]) for k in ("qw", "qx", "qy", "qz")]),
                      [float(row[prefix + k]) for k in ("x", "y", "z")])

    errs = []
    for lg, tr in list(zip(logged, true))[::every]:
        joints = {j: float(lg[j]) for j in ARM_JOINTS}
        cam = poser.poses(joints, float(lg["mirror_angle"]))[1]
        errs.append(np.linalg.norm(hit(cam, hs, head) - hit(pose(tr, "cam_"), hs, t.head), axis=1))
    return np.concatenate(errs) * 1e3


def run(keep=None, seed=7, head_errors=1.0, errors=1.0, views=None, lines=LINES, steps=STEPS, binning=4.0,
        log=print):
    """Returns (headcal's numbers against the truth, the CAD head's)."""
    from hsical.pipeline import calibrate
    from headcal import solve

    from .binned import bin_session
    from .session import Settings, load_plan, simulate
    base = Path(keep) if keep else Path(tempfile.mkdtemp(prefix="hsisim_handeye_"))
    try:
        t0 = time.time()
        session = base / "scan"
        plan = load_plan("headcal", lines=lines, steps_per_line=steps, views=views or plan_views())
        log(f"Simulating headcal's calibration scan ({len(plan.viewpoints)} viewpoints) into {session} ...")
        simulate(session, plan, Settings(scene="tagboard", pose_camera=True, head_errors=head_errors, errors=errors,
                                         seed=seed, binning=binning), log=lambda m: log("  " + m))
        log("Calibrating the instrument with hsical and binning the lines ...")
        r = calibrate(session / "calibration", base / "cal", log=lambda *m: None)
        if not r.ok:
            raise RuntimeError("hsical's calibration of the simulated instrument failed its checks")
        bin_session(session, base / "cal", log=lambda m: log("  " + m))
        log("Calibrating the head with headcal ...")
        result = solve.run(str(session), str(session / "pose"), None, str(base / "headcal"),
                           log=lambda m: log("  " + m))
        got, cad = compare(session, result)
        logged = {name: logged_line_errors(session, path) for name, path in (
            ("headcal", base / "headcal" / "head_calibration.yaml"), ("CAD", None),
            ("true", session / "truth" / "head_calibration.yaml"))}
        got["logged_mm_median"], cad["logged_mm_median"] = (float(np.median(logged[k])) for k in ("headcal", "CAD"))
        got["logged_mm_true_head"] = float(np.median(logged["true"]))
        log(f"\n{'against the true head':62s} {'headcal':>8s} {'CAD':>8s}")
        rows = [("line_mm_median", "line camera's rays on the board, median (mm)"),
                ("line_mm_max", "line camera's rays on the board, worst (mm)"),
                ("pose_mm", "pose camera on the wrist (mm)"), ("pose_deg", "pose camera on the wrist (deg)"),
                ("k_rel", "scan line length (relative)"),
                ("logged_mm_median", "scan line posed from the logged arm, median (mm)")]
        for key, text in rows:
            log(f"{text:62s} {got[key]:8.3f} {cad[key]:8.3f}")
        log(f"{'  ... and through the true head, for the arm alone (mm)':62s} {got['logged_mm_true_head']:8.3f}")
        log(f"{'pose camera focal length, centre (px)':62s} {got['f_px']:8.3f} {got['c_px']:8.3f}")
        log(f"{'slit order':62s} {'right' if got['slit_reversed_ok'] else 'wrong':>8s}")
        log(f"\n{time.time() - t0:.0f} s" + (f"; results in {base}" if keep else ""))
        return got, cad
    finally:
        if not keep:
            shutil.rmtree(base, ignore_errors=True)
