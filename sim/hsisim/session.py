"""A whole simulated scan session, written the way the rig writes one.

    <out>/
      lines.csv        one row per scan line, exactly as ros2's scan_sweep logs it (same
                       writer): viewpoint, sweep_id, index, stamp_ns, hold_until_ns, settled,
                       mirror_angle, head and line-camera poses in base_link, arm joints
      scan.json        the run, in scan_sweep's format, plus "simulated" and "camera" sections
      robot.urdf       the URDF the poses came from (from ros2/so101_scan_description's xacro)
      plan.yaml        the scan plan as run
      frames/frames.csv
                       one row per line, as the camera node logs it: sweep_id, index, status,
                       file, seq, sof_ns, exposure_start_ns, exposure_end_ns, exposure_us, gain,
                       saturated_px, n_frames
      frames/camera.json
                       the sensor (size, bits, black level, Bayer order, frame rate, row time and
                       the rows that see the slit), exposure, gain, which way the slit runs, and
                       the stamps' offset
      frames/sweep_<sweep_id:03d>/
                       one hsical frame set per sweep: meta.json and frame_<index:04d>.npy, the
                       raw 10-bit frame of each line as the camera gives it
      reference/dark/, reference/white/
                       hsical frame sets at the scan's exposure and gain: darks, and a PTFE sheet
                       (reflectance 0.98) under the scan's light
      calibration/     an hsical calibration session of the same simulated instrument (lamps,
                       flat, wires, laser, darks): `python -m hsical calibrate <out>/calibration`
      pose/            with the pose camera: its stills, as headcal's recorder keeps them
                       (posecam.py)
      truth/           what the frames were made from (see truth.py)

`python -m hsical apply <cal> <out>/frames/sweep_001 --dark <out>/reference/dark
--white <out>/reference/white` turns a sweep into reflectance.
"""

from __future__ import annotations

import csv
import json
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml

from . import repo
from . import spectra
from . import timing as T
from .arm import Arm, PoseErrors
from .instrument import Spectrograph
from .linecam import Objective, SlitRays
from .scenes import CARD, DEFAULT_TARGET, make as make_scene
from hsical.synth import Truth, write_session  # noqa: E402
from so101_scan_camera.session import CSV_COLUMNS as FRAMES_COLUMNS, FRAMES_FORMAT  # noqa: E402
from so101_scan_camera.sources import IMX219_MODES  # noqa: E402
from so101_scan_sweep import plan as plan_mod  # noqa: E402
from so101_scan_sweep.line_log import LinesCsv  # noqa: E402
from so101_scan_sweep.plan import ARM_JOINTS  # noqa: E402

FORMAT = "so101_scan lines v1"
FULL_DN = 1023
TIMING_COLUMNS = ["sweep_id", "index", "seq", "sof_ns", "exposure_start_ns", "exposure_end_ns", "tick_ns",
                  "move_start_ns", "move_end_ns", "moving_share"]
START_NS = 1_791_374_400 * 10 ** 9  # 2026-10-07 12:00 UTC: a fixed start keeps runs repeatable


@dataclass
class Settings:
    scene: str = "relief"
    lighting: str = "uniform"
    binning: float = 4.0           # 1 = the full 3280 x 2464 sensor, 2, 4 (as hsical)
    seed: int = 7
    exposure_us: float = 20000.0
    gain: float = 1.0
    fill: float = 0.8              # the white reference's peak, as a share of full scale
    reference_frames: int = 8
    halogen_k: float = 2850.0
    errors: float = 1.0            # scales every pose error (arm.PoseErrors); 0 = perfect poses
    slices: int = 3                # cuts across the slit's width
    rays_per_point: int = 12       # rays through the objective's aperture per slit point
    objective_blur_um: float = 5.0
    nm_step: float = 2.0
    start_ns: int = START_NS
    calibration: bool = True       # also render the hsical calibration session
    fps: float | None = None       # the camera's frame rate; None: its mode's (timing.camera_mode)
    stamp_error_us: float = 0.0    # how late the camera's frame stamps are against row 0's read-out
    stamp_offset_us: float = 0.0   # line_camera's stamp_offset_us, added to every stamp
    slit_reversed: bool = False    # the camera mounted the other way along the slit
    pose_camera: bool = False      # also render the pose camera's stills into pose/ (needs OpenCV)
    pose_stills: int = 1           # stills the recorder keeps during each sweep
    pose_exposure_us: float = 20000.0
    # the head off its CAD numbers, x what a hand-built one is (as headcal's synthetic head); 0: the CAD head
    head_errors: float = 0.0
    truth: dict = field(default_factory=dict)  # hsical Truth overrides, e.g. {"blur_px": 12.0}

    def instrument_truth(self):
        kw = dict(flip_y=self.slit_reversed)
        kw.update(self.truth)
        return Truth(scale=self.binning, seed=self.seed, halogen_k=self.halogen_k, **kw)

    def frame_rate(self):
        return float(self.fps or T.camera_mode(self.binning)[1])


def load_plan(plan, lines=None, steps_per_line=None, start_angle_deg=None, line_period_s=None, views=None):
    """A plan (path, a name in ros2's plans/ or headcal's, or a dict) with the sweep settings overridden."""
    if isinstance(plan, dict):
        data = json.loads(json.dumps(plan))
    else:
        path = Path(plan)
        for folder in (repo.PLANS, repo.HEADCAL_PLANS):
            if not path.exists() and (folder / f"{plan}.yaml").exists():
                path = folder / f"{plan}.yaml"
        data = yaml.safe_load(path.read_text()) or {}
    over = {k: v for k, v in (("n_lines", lines), ("steps_per_line", steps_per_line),
                               ("start_angle_deg", start_angle_deg), ("line_period_s", line_period_s)) if v is not None}
    if over:
        data["sweep"] = dict(data.get("sweep") or {}, **over)
        for vp in data.get("viewpoints") or []:  # the overrides win over each viewpoint's own
            if vp.get("sweep"):
                vp["sweep"] = {k: v for k, v in vp["sweep"].items() if k not in over}
    if views:
        vps = data.get("viewpoints") or []
        picked = [v for v in vps if v.get("name") in views] or [vps[int(i)] for i in views if str(i).isdigit()]
        if not picked:
            raise ValueError(f"no viewpoints named {views} in the plan")
        data["viewpoints"] = picked
    return plan_mod.from_dict(data)


def _write_set(path, frames, meta, names=None):
    path.mkdir(parents=True, exist_ok=True)
    for i, f in enumerate(frames):
        np.save(path / (names[i] if names else f"frame_{i:03d}.npy"), f)
    h, w = frames[0].shape
    meta = dict(meta, frames=len(frames), width=w, height=h, format="npy", bits=10)
    (path / "meta.json").write_text(json.dumps(meta, indent=2) + "\n")


def _iso(ns):
    return datetime.fromtimestamp(ns * 1e-9, tz=timezone.utc).isoformat()


class _TruthLog:
    """Everything the frames were made from, per line."""

    def __init__(self, path):
        self.path = path
        path.mkdir(parents=True, exist_ok=True)
        self.csv = LinesCsv(str(path / "lines_true.csv"))
        self.weights, self.depth = [], []


def simulate(out_dir, plan, settings: Settings | None = None, calibration_session=None, log=print):
    """Render a scan session into out_dir. Returns scan.json's contents."""
    s = settings or Settings()
    t_start = time.time()
    plan = plan if isinstance(plan, plan_mod.Plan) else load_plan(plan)
    for vp in plan.viewpoints:
        if not vp.joints:
            raise ValueError(f"viewpoint {vp.name} has no joints: the simulator needs every viewpoint's arm pose")
    if s.exposure_us * 1000 >= 1e9 / s.frame_rate():
        raise ValueError(f"a {s.exposure_us / 1000:g} ms exposure doesn't fit in the camera's "
                         f"{1e3 / s.frame_rate():.1f} ms frame at {s.frame_rate():g} fps")
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / "truth").mkdir(exist_ok=True)
    target = tuple(plan.source.get("target_m") or DEFAULT_TARGET)
    scene = make_scene(s.scene, target, s.lighting, **_scene_kw(s, target))
    truth = s.instrument_truth()
    seq = np.random.SeedSequence(s.seed)
    rng_pose, rng_rays = (np.random.default_rng(x) for x in seq.spawn(2))

    log(f"Instrument: binning {s.binning:g}, seed {s.seed}")
    sp = Spectrograph(truth, temp_k=s.halogen_k, nm_step=s.nm_step)
    robot = repo.robot()
    head = _head(s, robot, scene, out / "truth") if s.pose_camera or s.head_errors else None
    true_robot = repo.robot(out / "truth" / "head_calibration.yaml") if s.head_errors else robot
    obj = Objective.from_design(sp.inst.design.config, blur_um=s.objective_blur_um,
                                **(head["objective"] if s.head_errors else {}))
    rays = SlitRays(obj, sp.h, s.slices, s.rays_per_point, seed=int(rng_rays.integers(2 ** 31)))
    names = spectra.NAMES
    table = spectra.table(sp.nm)
    arm = Arm(true_robot, PoseErrors().scaled(s.errors), rng_pose, logged_robot=robot)
    stills = pcam = None
    if s.pose_camera:
        from .posecam import PoseCamera, StillLog
        log(f"Pose camera: {s.pose_stills} still{'s' if s.pose_stills != 1 else ''} a sweep")
        pcam = PoseCamera(head["truth"].camera, truth, temp_k=s.halogen_k, exposure_us=s.pose_exposure_us)
        stills = StillLog(out / "pose", out / "truth", pcam, _iso(s.start_ns))
        rng_stills = np.random.default_rng([s.seed, 2])

    # the white reference sets the exposure: its peak at `fill` of full scale
    white_scene = make_scene("white", target, s.lighting)
    head0, cam0 = arm.poses(plan.viewpoints[0].joints, 0.0)
    w_white, _ = rays.weights(white_scene, cam0, len(names))
    white_sig = sp.signal(w_white @ table, rays.slice_centre)
    level = sp.level_for(white_sig, s.exposure_us, s.gain, s.fill)
    ref_meta = dict(exposure_us=s.exposure_us, gain=s.gain, synthetic=True, design_scale=s.binning)
    _write_set(out / "reference" / "white", sp.expose(white_sig, level, s.exposure_us, s.reference_frames, s.gain),
               dict(ref_meta, kind="white", source="ptfe", reflectance=spectra.PTFE_REFLECTANCE))
    _write_set(out / "reference" / "dark", sp.dark(s.exposure_us, s.reference_frames, s.gain),
               dict(ref_meta, kind="dark", source="capped"))
    camera, rows_of, raw_shape = _camera(sp, s, white_sig)
    mirror = T.Mirror()

    lines = LinesCsv(str(out / "lines.csv"))
    tl = _TruthLog(out / "truth")
    (out / "frames").mkdir(parents=True, exist_ok=True)
    frames_csv = open(out / "frames" / "frames.csv", "w", newline="")
    frames_log = csv.writer(frames_csv)
    frames_log.writerow(FRAMES_COLUMNS)
    timing_csv = open(out / "truth" / "frames_true.csv", "w", newline="")
    timing_log = csv.writer(timing_csv)
    timing_log.writerow(TIMING_COLUMNS)
    info = {"format": FORMAT, "name": plan.name, "started": _iso(s.start_ns), "plan": plan.source,
            "viewpoints": [], "interrupted": False}
    t_ns = s.start_ns
    previous = None
    rad_per_step = 2 * np.pi / 6400
    for i, vp in enumerate(plan.viewpoints):
        sweep_id = i + 1
        furthest = 0.0 if previous is None else max(abs(vp.joints[j] - previous[j]) for j in ARM_JOINTS)
        move_s = max(plan.min_move_time, furthest / plan.max_joint_speed)
        t_ns += int((move_s + plan.settle_time + 0.05) * 1e9)
        previous = vp.joints
        sw = vp.sweep
        start_step = int(round(sw.start_angle / rad_per_step))
        flex = arm.flex()
        sweep_dir = out / "frames" / f"sweep_{sweep_id:03d}"
        sweep_dir.mkdir(parents=True, exist_ok=True)
        # the bridge locks the sweep to the camera's frames, as on the rig
        st = T.plan_sweep(camera, mirror, sweep_id, sw.n_lines, sw.steps_per_line, sw.line_period, t_ns)
        angles = [(start_step + k * sw.steps_per_line) * rad_per_step for k in range(sw.n_lines)]
        q_reads, angles_true = [], []
        for k in range(sw.n_lines):
            q_reads.append(arm.read_joints(vp.joints))
            angles_true.append(arm.mirror_true(angles[k]))
        stamps, logged_joints = [], []
        log(f"  {vp.name}: {sw.n_lines} lines, {st.lock.frames_per_line} camera frame"
            f"{'s' if st.lock.frames_per_line > 1 else ''} a line")
        for k, lt in enumerate(st.lines):
            head_l, cam = arm.logged_poses(q_reads[k], angles[k])
            head_t, cam_t = arm.poses(vp.joints, angles_true[k], flex)
            w, depth = rays.weights(scene, cam_t, len(names))
            row = [sweep_id, k]
            if lt.frames:
                f = lt.frames[0]
                # where the mirror was while each row exposed: still on the line, unless the stamps'
                # error moved the exposure onto a move, and then each row sees its share of each angle
                views = T.mix(camera, st, k, f.sequence, angles_true, np.arange(raw_shape[0]))
                sig = 0.0
                for angle, share in views:
                    wa = w if angle == angles_true[k] else rays.weights(
                        scene, arm.poses(vp.joints, angle, flex)[1], len(names))[0]
                    one = sp.signal(wa @ table, rays.slice_centre)
                    sig = one if len(views) == 1 else sig + share[rows_of] * one
                frame = sp.expose(sig, level, s.exposure_us, 1, s.gain)[0]
                name = f"frame_{k:04d}.npy"
                np.save(sweep_dir / name, frame)
                saturated = int((frame[camera.r0:camera.r1 + 1] >= FULL_DN).sum())
                row += ["ok", f"frames/{sweep_dir.name}/{name}", f.sequence, f.sof_ns, f.start_ns, f.end_ns,
                        "%.1f" % f.exposure_us, "%.4f" % f.gain, saturated, len(lt.frames)]
                still = max((share[camera.r0:camera.r1 + 1] for angle, share in views if angle == angles_true[k]),
                            key=lambda a: a.sum(), default=np.zeros(1))
                start, end = camera.true_window(f.sequence)
                timing_log.writerow([sweep_id, k, f.sequence, camera.sof(f.sequence), start, end, lt.tick_ns,
                                     *(lt.move or ("", "")), f"{1.0 - float(still.min()):.4f}"])
            else:
                row += ["no_frame", "", "", "", "", "", "", "", "", 0]
            frames_log.writerow(row)
            lines.write(i, sweep_id, k, lt.stamp_ns, lt.hold_until_ns, True, angles[k], head_l, cam, q_reads[k])
            tl.csv.write(i, sweep_id, k, lt.stamp_ns, lt.hold_until_ns, True, angles_true[k], head_t, cam_t,
                         vp.joints)
            tl.weights.append(w.mean(0).astype(np.float16))
            tl.depth.append(depth.astype(np.float32))
            stamps.append(lt.stamp_ns)
            logged_joints.append([q_reads[k][j] for j in ARM_JOINTS])
        h, w_px = raw_shape
        (sweep_dir / "meta.json").write_text(json.dumps(dict(
            kind="scene", source=s.scene, exposure_us=s.exposure_us, gain=s.gain, frames=sw.n_lines, width=w_px,
            height=h, format="npy", bits=10, sweep_id=sweep_id, viewpoint=i, synthetic=True,
            design_scale=s.binning), indent=2) + "\n")
        info["viewpoints"].append(_summary(vp, sweep_id, move_s, start_step * rad_per_step, rad_per_step,
                                           stamps, logged_joints, st.lock))
        if pcam is not None:
            from .posecam import still_stamps
            pose = arm.pose_camera(vp.joints, flex)
            signal = pcam.signal(scene, pose)
            for stamp in still_stamps(stamps[0], stamps[-1], s.pose_stills):
                stills.write(pcam.expose(signal, rng_stills), stamp, sweep_id, pose)
        t_ns = st.end_ns + int(0.6e9)
    lines.close()
    if stills is not None:
        stills.close()
    tl.csv.close()
    frames_csv.close()
    timing_csv.close()
    (out / "frames" / "camera.json").write_text(json.dumps(_camera_json(sp, s, camera, mirror), indent=2) + "\n")

    info["frames"] = {
        "base": arm.poser.base, "head": arm.poser.head, "line_camera": arm.poser.camera,
        "line_camera_axes": "z: the view through the mirror; x: along the slit, in pixel order; y = z cross x",
        "scene_distance_m": arm.poser.scene_distance, "scan_line_half_length_m": arm.poser.half_line}
    info["units"] = {"position": "m", "angle": "rad", "quaternion": "x y z w",
                     "stamp_ns": "ROS time in ns at which the mirror settled on the line",
                     "hold_until_ns": "ROS time in ns at which the mirror moved on to the next line"}
    info["finished"] = _iso(t_ns)
    info["lines"] = lines.rows
    info["camera"] = {"frames": "frames/frames.csv: one row per line, naming its raw frame "
                                "(frames/sweep_<sweep_id:03d>/frame_<index:04d>.npy); frames/camera.json: the sensor",
                      "reference": {"dark": "reference/dark", "white": "reference/white"},
                      "exposure_us": s.exposure_us, "gain": s.gain, "white_reflectance": spectra.PTFE_REFLECTANCE}
    cal_dir = None
    if calibration_session is not None:
        cal_dir = Path(calibration_session)
    elif s.calibration:
        cal_dir = out / "calibration"
        log("Calibration session (hsical):")
        write_session(cal_dir, truth, log=lambda m: log("  " + m.strip()))
    if cal_dir is not None:
        info["calibration_session"] = _relative(cal_dir, out)
    info["simulated"] = {
        "by": "sim/ (python -m hsisim)", "scene": scene.name, "lighting": scene.lighting.to_dict(),
        "settings": asdict(s), "instrument": asdict(truth), "objective": obj.to_dict(),
        "exposure_level_e_per_us": level, "seconds": round(time.time() - t_start, 1)}
    if stills is not None:
        info["simulated"]["pose_camera"] = {"stills": "pose", "count": stills.count,
                                            "truth": "truth/pose_camera.json, truth/stills_true.csv"}
    (out / "scan.json").write_text(json.dumps(info, indent=2) + "\n")
    (out / "robot.urdf").write_text(repo.robot_description())
    with open(out / "plan.yaml", "w") as f:
        yaml.safe_dump(plan.source, f, sort_keys=False)
    _write_truth(tl, sp, scene, arm, names, table, head, stills)
    log(f"Wrote {lines.rows} lines in {len(plan.viewpoints)} sweeps to {out} ({time.time() - t_start:.0f} s)")
    return info


def _number(x):
    return int(x) if float(x).is_integer() else float(x)


def _camera(sp, s, white_sig):
    """The camera's frame clock; for every pixel of the rendering, the row of the raw frame it
    lands on; and the raw frames' shape."""
    inst = sp.inst
    raw_shape = inst.orient(np.zeros((sp.H, sp.W))).shape
    # the rows that see the slit, as line_camera is told them (by a calibration's maps, or
    # slit_row_first and slit_row_last): every row the white reference lights
    lit = np.flatnonzero((inst.orient(white_sig) > 0.02 * white_sig.max()).any(1))
    mode, _ = T.camera_mode(s.binning)
    line_time = IMX219_MODES[mode]["line_time_us"] * mode[1] / raw_shape[0]
    camera = T.Camera(t0_ns=s.start_ns, fps=s.frame_rate(), line_time_us=line_time, r0=int(lit[0]),
                      r1=int(lit[-1]), exposure_us=s.exposure_us, gain=s.gain, stamp_error_us=s.stamp_error_us,
                      stamp_offset_us=s.stamp_offset_us)
    idx = inst.orient(np.arange(sp.H * sp.W).reshape(sp.H, sp.W))
    rows_of = np.empty(sp.H * sp.W, dtype=np.intp)
    rows_of[idx.ravel()] = np.repeat(np.arange(idx.shape[0]), idx.shape[1])
    return camera, rows_of.reshape(sp.H, sp.W), raw_shape


def _slit_reversed(sp):
    """True when rectified slit row 0 is the line camera's +x end (h = +1).

    hsical calibrate turns a sideways camera's frames so the spectrum runs along x, and leaves
    the slit running the way the rows do unless told to flip it (--flip-y), so this is whether
    h falls down the rows once the frames are turned."""
    h = sp.inst.orient(sp.inst.h_map)
    if sp.truth.transpose:
        h = h.T
    return bool(np.median(np.diff(h[:, h.shape[1] // 2])) < 0)


def _camera_json(sp, s, camera, mirror):
    """frames/camera.json, in the capture node's format."""
    inst = sp.inst
    cfa = inst.orient(inst.cfa)[:2, :2]
    shape = inst.orient(inst.cfa).shape
    return {
        "source": "simulated",
        "device": None,
        "sensor": {"width": int(shape[1]), "height": int(shape[0]), "bits": 10,
                   "black_level": _number(sp.truth.black_dn), "bayer": "".join("RGB"[int(c)] for c in cfa.ravel()),
                   "fps": camera.fps, "line_time_us": camera.line_time_us, "slit_rows": [camera.r0, camera.r1]},
        "exposure_us": s.exposure_us,
        "gain": s.gain,
        # false when the slit coordinate (h, and hsical's s) grows toward line_camera_optical_frame
        # +x, so that rectified slit row 0 is the dataset's pixel 0
        "slit_reversed": _slit_reversed(sp),
        "calibration": None,
        "binning": None,
        "timing": {"stamps": "ROS time; sof_ns is when a frame's row 0 was read out, and its exposure window "
                             "covers the slit rows from the first one's start to the last one's read-out",
                   "match_margin_us": mirror.match_margin_s * 1e6, "stamp_offset_us": s.stamp_offset_us},
        "format": FRAMES_FORMAT,
        "started": _iso(s.start_ns),
        "raw_every": 1,
        "simulated": {"binning": s.binning, "camera_mode": list(T.camera_mode(s.binning)[0]),
                      "stamp_error_us": s.stamp_error_us, "frame_locked": True, "mirror": asdict(mirror)},
    }


def _relative(path, start):
    try:
        return str(Path(path).resolve().relative_to(Path(start).resolve()))
    except ValueError:
        return str(Path(path).resolve())


def _summary(vp, sweep_id, move_s, start_angle, rad_per_step, stamps, joints, lock):
    """A viewpoint's entry in scan.json, as scan_sweep writes it."""
    periods = np.diff(np.array(stamps, dtype=np.int64)) * 1e-9
    j = np.array(joints)
    out = {"name": vp.name, "joints_goal": vp.joints, "move": {"ok": True, "message": "", "duration_s": move_s},
           "settled": True, "sweep_id": sweep_id, "start_angle": start_angle, "rad_per_step": rad_per_step,
           "line_period_s": lock.line_period_ns * 1e-9, "frame_locked": True,
           "frames_per_line": lock.frames_per_line, "lines_expected": vp.sweep.n_lines,
           "lines_logged": len(stamps), "lines_unsettled": 0}
    if len(stamps) > 1:
        out["line_period_mean_s"] = float(periods.mean())
        out["line_period_max_deviation_s"] = float(np.abs(periods - periods.mean()).max())
    out["joints_mean"] = dict(zip(ARM_JOINTS, j.mean(axis=0).tolist()))
    out["arm_motion_max_rad"] = float((j.max(axis=0) - j.min(axis=0)).max())
    out["joint_sample_gap_max_ms"] = 0.0
    return out


def _scene_kw(s, target):
    """The tag board's place on the table: near the target and turned a little, as a person puts it."""
    if s.scene != "tagboard":
        return {}
    from headcal.board import Board
    from headcal.synth import board_on_table
    rng = np.random.default_rng([s.seed, 3])
    centre = np.asarray(target[:2], float) + rng.normal(0.0, 0.008, 2)
    return dict(pose=board_on_table(Board(), centre, np.radians(rng.uniform(-20.0, 20.0)), target[2] + CARD))


def _head(s, robot, scene, truth_dir):
    """The head as it really is, and the pose camera's lens: headcal's synthetic truth for this seed
    (the CAD head with --head-errors 0). Writes truth/head_calibration.yaml, the true head as
    headcal writes a calibration, for the true URDF and to compare a calibration with."""
    from headcal.model import HeadGeometry
    from headcal.synth import make_truth
    nominal = HeadGeometry.from_urdf(robot)
    t = make_truth(nominal, board=getattr(scene, "board", None), seed=s.seed, errors=s.head_errors,
                   joint_noise_deg=0.0, slit_reversed=s.slit_reversed)
    if hasattr(scene, "board_pose"):
        t.board_in_base = scene.board_pose
    (truth_dir / "head_calibration.yaml").write_text(t.head.calibration_yaml(
        "The simulated head as it really is (hsisim --head-errors %g, seed %d)" % (s.head_errors, s.seed)))
    return dict(truth=t, nominal=nominal,
                objective=dict(line_scale=t.head.half_line / nominal.half_line, slit_k1=t.head.slit_k1))


def _write_truth(tl, sp, scene, arm, names, table, head=None, stills=None):
    p = tl.path
    np.save(p / "weights.npy", np.stack(tl.weights))
    np.save(p / "depth.npy", np.stack(tl.depth))
    np.save(p / "h.npy", sp.h.astype(np.float32))
    np.save(p / "nm.npy", sp.nm.astype(np.float32))
    np.save(p / "materials.npy", table.astype(np.float32))
    nm_map, h_map = sp.pixel_maps()
    np.savez_compressed(p / "pixel_maps.npz", nm=nm_map, h=h_map)
    (p / "scene.json").write_text(json.dumps(scene.to_dict(), indent=1) + "\n")
    (p / "truth.json").write_text(json.dumps(dict(
        materials=names,
        material_descriptions={n: spectra.MATERIALS[n][1] for n in names},
        rare_earth_bands=[dict(nm=c, sigma_nm=sg, depth=d) for c, sg, d in spectra.RARE_EARTH_BANDS],
        arm=arm.to_dict(),
        files={
            "lines_true.csv": "lines.csv's columns with the true values: mirror angle, head and line-camera "
                              "poses, and the joints' true angles (the head pose also has the arm's flex)",
            "weights.npy": "[line, h, material] float16: each material's share of the light at each slit "
                           "position, times its shading (rows sum to 1 under uniform light), across the "
                           "slit's width and the objective's aperture",
            "depth.npy": "[line, h] camera z (m) of what each slit position sees, NaN where nothing",
            "h.npy": "slit positions: -1 and +1 are the slit's ends, +1 at the line camera's +x end",
            "nm.npy": "wavelengths (nm) of materials.npy",
            "materials.npy": "[material, nm] reflectance of each material",
            "pixel_maps.npz": "nm and h of every sensor pixel, in the frames' orientation",
            "frames_true.csv": "when each kept frame really exposed: its true sof_ns and slit rows' window, "
                               "the tick and move (start, end) onto its line, and moving_share, the most "
                               "of any slit row's exposure spent while the mirror was off the line",
            "scene.json": "the scene: solids, textures, lighting",
            **_head_truth_files(p, head, stills, arm),
        }), indent=1) + "\n")


def _head_truth_files(p, head, stills, arm):
    """truth/head.json and pose_camera.json; their entries for truth.json's file list."""
    if head is None:
        return {}
    t = head["truth"]
    (p / "head.json").write_text(json.dumps(dict(
        head=t.head.to_json(), cad=head["nominal"].to_json(), pose_camera_lens=t.camera.to_json(),
        board=t.board.to_json(), board_in_base=np.asarray(t.board_in_base).tolist(),
        slit_reversed=t.slit_reversed, seed=t.seed), indent=1) + "\n")
    files = {"head.json": "the head as it really is and as the CAD model has it (headcal's HeadGeometry: the "
                          "mount, mirror, objective and pose camera in the head frame), the pose camera's lens, "
                          "and the tag board's pose in base_link",
             "head_calibration.yaml": "the true head, written as headcal writes a calibration: the "
                                      "true poses come from the URDF built with it"}
    if stills is not None:
        in_head = arm.robot.fk("pose_camera_optical_frame", {}, arm.poser.head)
        (p / "pose_camera.json").write_text(json.dumps(dict(
            lens=t.camera.to_json(), lens_model="OpenCV plumb bob (k1, k2, p1, p2, k3), pixel centres at integers",
            pose_camera_in_head=in_head.tolist(),
            grey_reflectance=dict(zip(spectra.NAMES, stills.camera.grey.tolist())),
            level_e_per_us=stills.camera.level), indent=1) + "\n")
        files["pose_camera.json"] = "the pose camera's true lens, its pose in the head, and each material's grey"
        files["stills_true.csv"] = "where the pose camera really was (base_link, x y z and quaternion) for each still"
    return files
