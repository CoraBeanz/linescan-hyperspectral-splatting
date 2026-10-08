"""binned/: what line_camera writes as it scans with a calibration, for a simulated session.

With the instrument's hsical calibration, line_camera bins every line on the Jetson as it comes
in (so101_scan_camera.binning.LineBinner) and saves the binned lines next to the raw frames. A
simulated session is rendered before its calibration session has been calibrated, so it comes
out as from a rig scanning without one: raw frames only. This bins them afterwards with the
same code, as line_camera would have:

    binned/binning.npz                 the grid (LineBinner.save)
    binned/sweep_NNN.npy               float32 [lines, slit bins, bands]: mean raw counts per cell,
                                       NaN where a pixel saturated, the cell is empty or the line
                                       got no frame
    binned/reference_dark.npy, reference_white.npy
                                       the references' binned mean
    frames/camera.json                 gains the calibration and binning entries line_camera writes

Tools that read only binned lines need this: headcal solve, for one.

    python -m hsisim bin SESSION CAL
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

from . import repo  # noqa: F401
from so101_scan_camera.binning import LineBinner, load_calibration  # noqa: E402
from so101_scan_camera.session import binned_name  # noqa: E402


def bin_session(session, calibration, slit_bins=256, nm_step=None, nm_range=None, log=print):
    """Bin a session's raw frames with an hsical calibration (its output folder). Returns the
    LineBinner."""
    session = Path(session)
    maps = load_calibration(str(calibration))
    binner = LineBinner(maps, slit_bins, nm_step, nm_range)
    out = session / "binned"
    out.mkdir(exist_ok=True)
    binner.save(str(out / "binning.npz"))
    with open(session / "frames" / "frames.csv", newline="") as f:
        rows = list(csv.DictReader(f))
    sweeps = {}
    for r in rows:
        sweeps.setdefault(int(r["sweep_id"]), []).append(r)
    summary = []
    for sid, rs in sorted(sweeps.items()):
        lines = np.full((max(int(r["index"]) for r in rs) + 1,) + binner.shape, np.nan, np.float32)
        for r in rs:
            if r["status"] == "ok" and r["file"]:
                lines[int(r["index"])] = binner.bin(np.load(session / r["file"]))[0]
        np.save(out / binned_name(sid), lines)
        summary.append(dict(sweep_id=sid, lines=int(lines.shape[0]),
                            lines_with_data=int(np.isfinite(lines).any(axis=(1, 2)).sum())))
    for name in ("dark", "white"):
        ref = session / "reference" / name
        frames = sorted(ref.glob("frame_*.npy"))
        if frames:
            np.save(out / f"reference_{name}.npy",
                    np.mean([binner.bin(np.load(f))[0] for f in frames], axis=0).astype(np.float32))
    cam_path = session / "frames" / "camera.json"
    cam = json.loads(cam_path.read_text())
    cam["calibration"] = dict(path=maps.path, maps_sha256=maps.sha256, temp_k=maps.temp_k)
    cam["binning"] = dict(file="binned/binning.npz", slit_bins=binner.shape[0], bands=binner.shape[1],
                          nm_range=[float(binner.nm_edges[0]), float(binner.nm_edges[-1])],
                          nm_step=float(binner.nm_edges[1] - binner.nm_edges[0]),
                          values="mean raw counts per cell (black level included)")
    cam["sweeps"] = summary
    cam_path.write_text(json.dumps(cam, indent=2) + "\n")
    log(f"Binned {sum(s['lines_with_data'] for s in summary)} lines of {len(summary)} sweeps on "
        f"{binner.shape[0]} slit bins x {binner.shape[1]} bands into {out}")
    return binner
