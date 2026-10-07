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
                       saturated_px
      frames/camera.json
                       the sensor (size, bits, black level, Bayer order), exposure, gain, which
                       way the slit runs, and the timing model
      frames/sweep_<sweep_id:03d>/
                       one hsical frame set per sweep: meta.json and frame_<index:04d>.npy, the
                       raw 10-bit frame of each line as the camera gives it
      reference/dark/, reference/white/
                       hsical frame sets at the scan's exposure and gain: darks, and a PTFE sheet
                       (reflectance 0.98) under the scan's light
      calibration/     an hsical calibration session of the same simulated instrument (lamps,
                       flat, wires, laser, darks): `python -m hsical calibrate <out>/calibration`
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
from .arm import Arm, PoseErrors
from .instrument import Spectrograph
from .linecam import Objective, SlitRays
from .scenes import DEFAULT_TARGET, make as make_scene
from hsical.synth import Truth, write_session  # noqa: E402
from so101_scan_sweep import plan as plan_mod  # noqa: E402
from so101_scan_sweep.line_log import LinesCsv  # noqa: E402
from so101_scan_sweep.plan import ARM_JOINTS  # noqa: E402

FORMAT = "so101_scan lines v1"
SETTLE_NS = 1_000_000          # the camera starts its exposure 1 ms after the mirror settles
TIMER_NS = 50_000              # the ESP32's 50 us timer step: hold_until is rounded early by it
FRAMES_FORMAT = "so101_scan frames v1"
FRAMES_COLUMNS = ["sweep_id", "index", "status", "file", "seq", "sof_ns", "exposure_start_ns", "exposure_end_ns",
                  "exposure_us", "gain", "saturated_px"]
FULL_DN = 1023
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
    truth: dict = field(default_factory=dict)  # hsical Truth overrides, e.g. {"blur_px": 12.0}

    def instrument_truth(self):
        return Truth(scale=self.binning, seed=self.seed, halogen_k=self.halogen_k, **self.truth)


def load_plan(plan, lines=None, steps_per_line=None, start_angle_deg=None, line_period_s=None, views=None):
    """A plan (path, a name in ros2's plans/, or a dict) with the sweep settings overridden."""
    if isinstance(plan, dict):
        data = json.loads(json.dumps(plan))
    else:
        path = Path(plan)
        if not path.exists() and (repo.PLANS / f"{plan}.yaml").exists():
            path = repo.PLANS / f"{plan}.yaml"
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
        hold_ns = vp.sweep.line_period * 1e9 - TIMER_NS - SETTLE_NS
        if s.exposure_us * 1000 > hold_ns:
            raise ValueError(f"a {s.exposure_us / 1000:g} ms exposure doesn't fit in viewpoint {vp.name}'s "
                             f"{vp.sweep.line_period * 1000:g} ms line period: the mirror holds still for "
                             f"{hold_ns / 1e6:.2f} ms after the camera starts")
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    target = tuple(plan.source.get("target_m") or DEFAULT_TARGET)
    scene = make_scene(s.scene, target, s.lighting)
    truth = s.instrument_truth()
    seq = np.random.SeedSequence(s.seed)
    rng_pose, rng_rays = (np.random.default_rng(x) for x in seq.spawn(2))

    log(f"Instrument: binning {s.binning:g}, seed {s.seed}")
    sp = Spectrograph(truth, temp_k=s.halogen_k, nm_step=s.nm_step)
    obj = Objective.from_design(sp.inst.design.config, blur_um=s.objective_blur_um)
    rays = SlitRays(obj, sp.h, s.slices, s.rays_per_point, seed=int(rng_rays.integers(2 ** 31)))
    names = spectra.NAMES
    table = spectra.table(sp.nm)
    robot = repo.robot()
    arm = Arm(robot, PoseErrors().scaled(s.errors), rng_pose)

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

    lines = LinesCsv(str(out / "lines.csv"))
    tl = _TruthLog(out / "truth")
    (out / "frames").mkdir(parents=True, exist_ok=True)
    frames_csv = open(out / "frames" / "frames.csv", "w", newline="")
    frames_log = csv.writer(frames_csv)
    frames_log.writerow(FRAMES_COLUMNS)
    seq, last_sof = -1, None
    exposure_ns = int(round(s.exposure_us * 1000))
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
        period_ns = int(round(sw.line_period * 1e9))
        flex = arm.flex()
        sweep_dir = out / "frames" / f"sweep_{sweep_id:03d}"
        sweep_dir.mkdir(parents=True, exist_ok=True)
        stamps, logged_joints = [], []
        log(f"  {vp.name}: {sw.n_lines} lines")
        for k in range(sw.n_lines):
            stamp = t_ns + k * period_ns
            hold_until = stamp + period_ns - TIMER_NS
            angle = (start_step + k * sw.steps_per_line) * rad_per_step
            q_read = arm.read_joints(vp.joints)
            head, cam = arm.poses(q_read, angle)
            angle_true = arm.mirror_true(angle)
            head_t, cam_t = arm.poses(vp.joints, angle_true, flex)
            w, depth = rays.weights(scene, cam_t, len(names))
            sig = sp.signal(w @ table, rays.slice_centre)
            frame = sp.expose(sig, level, s.exposure_us, 1, s.gain)[0]
            name = f"frame_{k:04d}.npy"
            np.save(sweep_dir / name, frame)
            # the camera is frame-locked to the mirror: it starts exposing 1 ms after the mirror
            # settles, every row in the same window (the frames are rendered with the mirror
            # still, so the IMX219's rolling shutter is left out), and row 0 is read out as the
            # exposure ends
            start = stamp + SETTLE_NS
            sof = start + exposure_ns
            seq += 1 if last_sof is None else max(1, int(round((sof - last_sof) / period_ns)))
            last_sof = sof
            frames_log.writerow([sweep_id, k, "ok", f"frames/{sweep_dir.name}/{name}", seq, sof, start,
                                 start + exposure_ns, s.exposure_us, s.gain, int((frame >= FULL_DN).sum())])
            lines.write(i, sweep_id, k, stamp, hold_until, True, angle, head, cam, q_read)
            tl.csv.write(i, sweep_id, k, stamp, hold_until, True, angle_true, head_t, cam_t, vp.joints)
            tl.weights.append(w.mean(0).astype(np.float16))
            tl.depth.append(depth.astype(np.float32))
            stamps.append(stamp)
            logged_joints.append([q_read[j] for j in ARM_JOINTS])
        h, w_px = frame.shape
        (sweep_dir / "meta.json").write_text(json.dumps(dict(
            kind="scene", source=s.scene, exposure_us=s.exposure_us, gain=s.gain, frames=sw.n_lines, width=w_px,
            height=h, format="npy", bits=10, sweep_id=sweep_id, viewpoint=i, synthetic=True,
            design_scale=s.binning), indent=2) + "\n")
        info["viewpoints"].append(_summary(vp, sweep_id, move_s, start_step * rad_per_step, rad_per_step,
                                           stamps, logged_joints))
        t_ns = stamps[-1] + period_ns + int(0.6e9)
    lines.close()
    tl.csv.close()
    frames_csv.close()
    (out / "frames" / "camera.json").write_text(json.dumps(_camera_json(sp, s, frame.shape), indent=2) + "\n")

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
    (out / "scan.json").write_text(json.dumps(info, indent=2) + "\n")
    (out / "robot.urdf").write_text(repo.robot_description())
    with open(out / "plan.yaml", "w") as f:
        yaml.safe_dump(plan.source, f, sort_keys=False)
    _write_truth(tl, sp, scene, arm, names, table)
    log(f"Wrote {lines.rows} lines in {len(plan.viewpoints)} sweeps to {out} ({time.time() - t_start:.0f} s)")
    return info


def _number(x):
    return int(x) if float(x).is_integer() else float(x)


def _camera_json(sp, s, shape):
    """frames/camera.json, in the capture node's format."""
    inst = sp.inst
    cfa = inst.orient(inst.cfa)[:2, :2]
    return {
        "format": FRAMES_FORMAT,
        "sensor": {"width": int(shape[1]), "height": int(shape[0]), "bits": 10,
                   "black_level": _number(sp.truth.black_dn), "bayer": "".join("RGB"[int(c)] for c in cfa.ravel())},
        "exposure_us": s.exposure_us,
        "gain": s.gain,
        # the slit coordinate (h, and hsical's s) grows toward line_camera_optical_frame +x,
        # so rectified slit row 0 is the dataset's pixel 0
        "slit_reversed": False,
        "calibration": None,
        "timing": {"model": "simulated", "shutter": "global: every row exposes in the same window",
                   "frame_locked": True, "exposure_start_after_settle_ns": SETTLE_NS,
                   "sof": "row 0 read out as the exposure ends"},
        "simulated": {"binning": s.binning},
    }


def _relative(path, start):
    try:
        return str(Path(path).resolve().relative_to(Path(start).resolve()))
    except ValueError:
        return str(Path(path).resolve())


def _summary(vp, sweep_id, move_s, start_angle, rad_per_step, stamps, joints):
    """A viewpoint's entry in scan.json, as scan_sweep writes it."""
    periods = np.diff(np.array(stamps, dtype=np.int64)) * 1e-9
    j = np.array(joints)
    out = {"name": vp.name, "joints_goal": vp.joints, "move": {"ok": True, "message": "", "duration_s": move_s},
           "settled": True, "sweep_id": sweep_id, "start_angle": start_angle, "rad_per_step": rad_per_step,
           "lines_expected": vp.sweep.n_lines, "lines_logged": len(stamps), "lines_unsettled": 0}
    if len(stamps) > 1:
        out["line_period_mean_s"] = float(periods.mean())
        out["line_period_max_deviation_s"] = float(np.abs(periods - periods.mean()).max())
    out["joints_mean"] = dict(zip(ARM_JOINTS, j.mean(axis=0).tolist()))
    out["arm_motion_max_rad"] = float((j.max(axis=0) - j.min(axis=0)).max())
    out["joint_sample_gap_max_ms"] = 0.0
    return out


def _write_truth(tl, sp, scene, arm, names, table):
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
            "scene.json": "the scene: solids, textures, lighting",
        }), indent=1) + "\n")
