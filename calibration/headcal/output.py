"""What a calibration leaves behind, and the report that explains it.

    head_calibration.yaml   for the URDF: `ros2 launch so101_scan_bringup scan_arm.launch.py
                            head_calibration:=...` (or copy it to ~/so101_scan/, where the launch
                            looks by default)
    robot.urdf              the scan's URDF with the calibration in it, for scan_to_dataset --urdf
                            (scans taken before the calibration)
    calibration.json        everything: the geometry, the trainer's head model, the pose camera's
                            lens, the board on the table, fits and uncertainties
    pose_camera.yaml        the pose camera's lens as ROS camera_info
    report.md               what changed, how well it fits, per viewpoint
    line_corners.png        a few sweeps with the corners the solver found in them
"""

from __future__ import annotations

import json
import math
import os
from datetime import datetime

import numpy as np

from . import geometry as g
from .model import FORMAT, deg

MICROSTEP = 2.0 * math.pi / 6400.0


def changes(result):
    """What moved, part by part, from the scan URDF's geometry to the calibrated one."""
    n, h = result.nominal, result.head
    out = []

    def pose_change(name, a, b, frame_note=""):
        d = g.inv(a) @ b
        out.append(dict(part=name, shift_mm=float(np.linalg.norm(d[:3, 3]) * 1e3),
                        shift_xyz_mm=(d[:3, 3] * 1e3).tolist(),
                        turn_deg=deg(g.angle_between(a[:3, :3], b[:3, :3])),
                        turn_xyz_deg=[deg(v) for v in g.rot_log(d[:3, :3])], note=frame_note))

    pose_change("head on the wrist (mount)", n.mount, h.mount, "head frame: z along the wrist roll axis")
    pose_change("pose camera in the head", n.pose_camera, h.pose_camera, "camera frame: x right, y down, z out")
    pose_change("objective in the head", n.objective, h.objective, "objective frame: x along the slit, z its view")
    d = g.inv(n.mirror) @ h.mirror
    rv = g.rot_log(d[:3, :3])
    out.append(dict(part="mirror shaft", shift_mm=float(np.hypot(d[1, 3], d[2, 3]) * 1e3),
                    shift_xyz_mm=(d[:3, 3] * 1e3).tolist(), turn_deg=deg(float(np.hypot(rv[1], rv[2]))),
                    turn_xyz_deg=[deg(v) for v in rv], note="shaft frame: x the shaft; shift is across it"))
    return out


def home_offset(result):
    """The mirror's home error (rad): calibrated angle = reported angle + this."""
    d = g.inv(result.nominal.mirror) @ result.head.mirror
    return float(g.rot_log(d[:3, :3])[0])


def calibration_json(result, inputs):
    h = result.head
    views = []
    for v in result.views:
        views.append(dict(name=v.name, sweep_id=v.sweep.sweep_id,
                          still=None if v.detection is None else os.path.basename(v.detection.still.path),
                          pose_corners=0 if v.detection is None else int(len(v.detection.ids)),
                          line_corners=0 if not v.line_obs else int(len(v.line_obs["ids"])),
                          arm_mm=None if not np.isfinite(v.arm_mm) else round(v.arm_mm, 4),
                          arm_deg=None if not np.isfinite(v.arm_deg) else round(v.arm_deg, 4)))
    return {
        "format": FORMAT, "created": datetime.now().isoformat(timespec="seconds"), "inputs": inputs,
        "head": h.to_json(),
        "trainer": {"head": h.trainer_head_model(), "scene_distance_m": h.scene_distance,
                    "scan_line_half_length_m": h.half_line, "slit_k1": h.slit_k1,
                    "note": "the trainer's line camera is a pinhole: f_px = width * scene_distance / "
                            "(2 * scan_line_half_length), cu = width / 2; slit_k1 isn't used yet"},
        "nominal_head": result.nominal.to_json(),
        "pose_camera": result.camera.to_json(),
        "board_in_base": g.pose_array(result.board_in_base).tolist(),
        "slit_reversed": result.slit_reversed,
        "mirror_home_offset_rad": home_offset(result),
        "joint_offsets_rad": result.joint_offsets,
        "changes": changes(result),
        "fit": {k: v for k, v in result.stats.items() if k != "per_view"},
        "uncertainty_1sigma": result.sigmas,
        "warnings": result.warnings,
        "viewpoints": views,
    }


def report(result, inputs):
    s = result.stats
    h, n = result.head, result.nominal
    arm = np.array([a for a in s["arm_mm"] if np.isfinite(a)])
    lines = ["# Head calibration", "",
             "Scan `%s`, pose-camera stills `%s`, board %s." % (inputs.get("scan"), inputs.get("pose"),
                                                                inputs.get("board_text")), ""]
    lines += ["## Fit", "",
              "- Pose camera: %d corners in %d stills, %.3f px rms." % (s["n_pose"], sum(v.detection is not None
                                                                                         for v in result.views),
                                                                          s["pose_rms_px"]),
              "- Line camera: %d corners in %d sweeps, %.3f px rms along the slit and %.3f lines rms across it."
              % (s["n_line"], sum(bool(v.line_obs) for v in result.views), s["slit_rms_px"], s["lines_rms"]),
              "- Arm: from each viewpoint the board lands %.2f mm (median, %.2f mm worst) from where it lies; "
              "that is the arm's own repeatability and joint calibration, not the head's." % (
                  float(np.median(arm)) if arm.size else float("nan"), float(arm.max()) if arm.size else float("nan")),
              ""]
    if result.warnings:
        lines += ["## Check these", ""] + ["- " + w for w in result.warnings] + [""]
    lines += ["## What changed from the scan's URDF", "",
              "The scan's URDF is the CAD model's unless it was built with an earlier calibration.", "",
              "| Part | Shift (mm) | Turn (deg) | Shift x, y, z (mm) | Turn about x, y, z (deg) |",
              "|---|---:|---:|---|---|"]
    for c in changes(result):
        lines.append("| %s | %.2f | %.2f | %s | %s |" % (c["part"], c["shift_mm"], c["turn_deg"],
                                                       ", ".join("%.2f" % v for v in c["shift_xyz_mm"]),
                                                       ", ".join("%.2f" % v for v in c["turn_xyz_deg"])))
    ho = home_offset(result)
    lines += ["", "Frames: the mount's is the head's (z along the wrist roll axis), the pose camera's x right, "
              "y down, z out; the objective's x along the slit, z its view; the shaft's x along the shaft.", "",
              "- **Mirror home angle**: the mirror sits %.3f deg from where the firmware says (%+.1f microsteps). The "
              "URDF carries this; leave the firmware's home_pos alone, or calibrate again after changing it."
              % (deg(ho), ho / MICROSTEP),
              "- **Scan line**: %.3f mm long at %.0f mm (CAD %.3f mm, %+.2f%%): the line camera's focal length "
              "follows from it." % (2e3 * h.half_line, 1e3 * h.scene_distance, 2e3 * n.half_line,
                                     100.0 * (h.half_line / n.half_line - 1.0)),
              "- **Distortion along the slit**: k1 = %.4f (a point at the slit's end lands %.2f px from where a "
              "pinhole puts it, at 256 px; the trainer doesn't use it yet)." % (h.slit_k1, abs(h.slit_k1) * 128),
              "- **Slit order**: %s." % ("reversed (pixel 0 at the head's +X end)" if result.slit_reversed
                                         else "as recorded (pixel 0 at the head's -X end)"), ""]
    if result.joint_offsets:
        lines += ["- **Arm joint offsets** (fitted as a check, not applied anywhere; what to add to each "
                  "joint's reading to get its true angle): " + ", ".join(
                      "%s %+.3f deg" % (k, deg(v)) for k, v in result.joint_offsets.items())
                  + ". Offsets over about 0.3 deg mean that joint's zero in sts_calibrate's calibration.yaml "
                    "is off by that much.", ""]
    c = result.camera
    sg = result.sigmas
    lines += ["## Pose camera lens", "", "| | Value | 1 sigma |", "|---|---:|---:|"]
    for name, val in zip(("fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2"),
                         (c.fx, c.fy, c.cx, c.cy, *c.dist[:4])):
        lines.append("| %s | %.5g | %.2g |" % (name, val, sg.get(name, float("nan"))))
    lines += ["", "## Per viewpoint", "",
              "| Viewpoint | Pose corners | Pose rms (px) | Line corners | Slit rms (px) | Across rms (lines) "
              "| Arm (mm) |",
              "|---|---:|---:|---:|---:|---:|---:|"]
    arm_by = {v.name: v.arm_mm for v in result.views}
    for pv in s["per_view"]:
        def f(x, fmt="%.3f"):
            return "" if x is None or (isinstance(x, float) and not np.isfinite(x)) else fmt % x
        lines.append("| %s | %d | %s | %d | %s | %s | %s |" % (pv["name"], pv["pose_corners"], f(pv["pose_rms_px"]),
                                                             pv["line_corners"], f(pv["slit_rms_px"]),
                                                             f(pv["lines_rms"]), f(arm_by.get(pv["name"]), "%.2f")))
    lines += ["", "## Using it", "",
              "- On the Nano, copy `head_calibration.yaml` to `~/so101_scan/head_calibration.yaml`: the launch "
              "file builds the URDF with it from then on, so TF, lines.csv and every new scan use it.",
              "- For scans taken before, convert with the calibrated URDF: `python3 splat/tools/scan_to_dataset.py "
              "SCAN OUT --urdf %s/robot.urdf`." % inputs.get("out", "cal"),
              "- Calibrate again after taking the head off the wrist, re-running sts_calibrate, or changing the "
              "mirror's home_pos.", ""]
    return "\n".join(lines)


def corner_picture(result, path, n=4):
    """A few sweeps side by side: green crosses where the solver found corners, red where the
    final model puts them."""
    import cv2
    views = [v for v in result.views if v.line_obs][:n]
    if not views:
        return
    tiles = []
    for v in views:
        img = v.sweep.image.copy()
        img[~np.isfinite(img)] = np.nanmedian(img)
        lo, hi = np.percentile(img, [1, 99])
        a = np.clip((img - lo) / max(hi - lo, 1e-9) * 255, 0, 255).astype(np.uint8)
        scale = 2
        a = cv2.cvtColor(cv2.resize(a, (a.shape[1] * scale, a.shape[0] * scale), interpolation=cv2.INTER_NEAREST),
                         cv2.COLOR_GRAY2BGR)
        o = v.line_obs
        for ln, u in zip(o["line"], o["u"]):
            c = (int(round((u - 0.5) * scale)), int(round(ln * scale)))
            cv2.circle(a, c, 7, (0, 200, 0), 2)
            cv2.drawMarker(a, c, (0, 200, 0), cv2.MARKER_CROSS, 22, 1)
        cv2.rectangle(a, (0, 0), (a.shape[1], 22), (255, 255, 255), -1)
        cv2.putText(a, "%s: %d corners" % (v.name, len(o["ids"])), (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 0, 0), 1)
        tiles.append(a)
    hmax = max(t.shape[0] for t in tiles)
    tiles = [cv2.copyMakeBorder(t, 0, hmax - t.shape[0], 0, 6, cv2.BORDER_CONSTANT, value=(255, 255, 255))
             for t in tiles]
    cv2.imwrite(path, np.concatenate(tiles, axis=1))


def write(result, out, scan_urdf, inputs, log=print):
    os.makedirs(out, exist_ok=True)
    inputs = dict(inputs, out=out)
    header = ("Head calibration from headcal (calibration/headcal), %s.\nScan: %s\nPose stills: %s\n"
              "Use: head_calibration:=<this file>, or copy it to ~/so101_scan/head_calibration.yaml"
              % (datetime.now().strftime("%Y-%m-%d %H:%M"), inputs.get("scan"), inputs.get("pose")))
    files = {
        "head_calibration.yaml": result.head.calibration_yaml(header),
        "robot.urdf": result.head.patch_urdf(scan_urdf),
        "calibration.json": json.dumps(calibration_json(result, inputs), indent=2) + "\n",
        "pose_camera.yaml": result.camera.camera_info_yaml(),
        "report.md": report(result, inputs),
    }
    for name, text in files.items():
        with open(os.path.join(out, name), "w") as f:
            f.write(text)
    try:
        corner_picture(result, os.path.join(out, "line_corners.png"))
    except Exception as e:  # a picture is a nice-to-have
        log("  (no line_corners.png: %s)" % e)
    log("wrote %s" % ", ".join(sorted(files)))
