"""The pose camera: the Pi NoIR camera v2 beside the spectrograph that reads the tag board, and the
stills headcal's recorder keeps from it.

On the rig, `python -m headcal record` grabs raw frames from the pose camera while a scan runs,
averages each 2 x 2 colour cell into one grey pixel and keeps a still whenever the view holds
(calibration/headcal/record.py). The simulator writes the same folder:

    pose/camera.json     "headcal pose frames v1": the sensor mode, the stills' size, exposure, gain
    pose/frames.csv      index, stamp_ns, file, exposure_us, gain
    pose/still_NNNN.npy  uint16 grey, half the 3264 x 2464 mode (1632 x 1232), raw counts with
                         the black level (64) in

Each still is rendered from pose_camera_optical_frame (OpenCV axes: x right, y down, z out) on
the head as it really was, arm flex included, so the pose camera and the line camera move
together as they do on the rig. Every pixel is the mean of 2 x 2 rays through a real-looking
lens (OpenCV's plumb-bob model: headcal's synthetic lens, a Pi camera v2 with its focal length,
centre and distortion a little off nominal), into the scene, so the stills have the scene's
depth, occlusion and shadows. A pixel's grey is what the sensor's four colour cells see of the
material under the scan's light (no IR-cut filter: the NoIR camera sees into the near infrared),
dimmed toward the corners (cos^4), blurred a little for the lens, with shot and read noise.

The true lens and every still's camera pose are in truth/pose_camera.json.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

from . import spectra
from hsical.synth import cfa_transmission, qe  # noqa: E402
from so101_scan_sweep.line_log import pose_fields  # noqa: E402

SENSOR = (3264, 2464)          # the mode the recorder grabs; the stills are half its size
FULL_DN = 1023
STILL_COLUMNS = ["index", "stamp_ns", "file", "exposure_us", "gain"]
TRUE_COLUMNS = ["index", "stamp_ns", "sweep_id", "x", "y", "z", "qx", "qy", "qz", "qw"]


def grey_reflectance(nm, temp_k):
    """[material] what the pose camera's 2 x 2 colour cell sees of each material, relative to a
    perfect white, under a halogen lamp at temp_k (the scan's light)."""
    response = qe(nm) * (cfa_transmission(nm, 0) + 2.0 * cfa_transmission(nm, 1) + cfa_transmission(nm, 2)) / 4.0
    weight = spectra.halogen(nm, temp_k) * response
    return spectra.table(nm) @ weight / weight.sum()


class PoseCamera:
    """Renders grey stills of a scene from a camera pose (4 x 4, camera -> base_link).

    lens: a headcal PinholeCamera (1632 x 1232). instrument: the hsical Truth whose sensor
    numbers (electrons per count, read noise, black level) the IMX219 has. The exposure is set
    the way `record --auto` would: white paper square-on in the middle of the view at `fill` of
    full scale."""

    def __init__(self, lens, instrument, temp_k=2850.0, exposure_us=20000.0, gain=1.0, fill=0.7, sub=2,
                 blur_px=0.6, rows_per_chunk=64):
        self.lens, self.inst = lens, instrument
        self.exposure_us, self.gain, self.fill = float(exposure_us), float(gain), float(fill)
        self.sub, self.blur_px, self.rows_per_chunk = int(sub), float(blur_px), int(rows_per_chunk)
        nm = spectra.wavelength_grid(5.0)
        self.grey = grey_reflectance(nm, temp_k)
        white = float(self.grey[spectra.material_id("white_paper")])
        full_e = (FULL_DN - instrument.black_dn) * instrument.e_per_dn
        self.level = self.fill * full_e / (self.exposure_us * self.gain * white)   # e per us at grey 1
        w, h = lens.width, lens.height
        ys, xs = np.mgrid[0:h, 0:w]
        offsets = (np.arange(self.sub) + 0.5) / self.sub - 0.5
        uv = [np.stack([xs.ravel() + ox, ys.ravel() + oy], axis=-1) for oy in offsets for ox in offsets]
        # [rows, sub x sub, columns, 3]: every pixel's rays, in the camera frame
        rays = np.stack([lens.rays(p).reshape(h, w, 3) for p in uv], axis=1)
        self.rays = rays.astype(np.float32)
        r2 = ((xs - lens.cx) / lens.fx) ** 2 + ((ys - lens.cy) / lens.fy) ** 2
        self.vignette = (1.0 / (1.0 + r2)) ** 2

    @property
    def size(self):
        return self.lens.width, self.lens.height

    def signal(self, scene, pose):
        """The expected grey of every pixel, relative to a perfect white lit as the scan target is
        (float [height, width]), before noise."""
        import cv2
        r, eye = np.asarray(pose, float)[:3, :3], np.asarray(pose, float)[:3, 3]
        h, w = self.lens.height, self.lens.width
        out = np.zeros((h, w))
        n = self.sub * self.sub
        for r0 in range(0, h, self.rows_per_chunk):
            d = self.rays[r0:r0 + self.rows_per_chunk].reshape(-1, 3).astype(np.float64) @ r.T
            o = np.broadcast_to(eye, d.shape)
            hits = scene.cast(o, d, eye=eye)
            g = self.grey[hits.material] * hits.shading
            out[r0:r0 + self.rows_per_chunk] = g.reshape(-1, n, w).mean(axis=1)
        out *= self.vignette
        if self.blur_px > 0:
            out = cv2.GaussianBlur(out, (0, 0), self.blur_px)
        return out

    def expose(self, signal, rng):
        """A still as the recorder keeps it: the four colour cells' raw counts averaged (uint16)."""
        t = self.inst
        e = 4.0 * np.clip(signal, 0, None) * self.level * self.exposure_us      # the cell's four pixels
        e = rng.poisson(e).astype(float) + 2.0 * t.read_e * rng.standard_normal(e.shape)
        dn = e / t.e_per_dn * self.gain / 4.0 + t.black_dn
        return np.clip(np.round(dn), 0, FULL_DN).astype(np.uint16)

    def camera_json(self, started):
        return {"format": "headcal pose frames v1", "device": None, "source": "simulated",
                "sensor_width": SENSOR[0], "sensor_height": SENSOR[1], "width": self.lens.width,
                "height": self.lens.height, "exposure_us": self.exposure_us, "gain": self.gain,
                "grey": "2 x 2 colour cells averaged, raw counts (black level about 64)", "started": started,
                "simulated": {"by": "sim/ (python -m hsisim)", "truth": "truth/pose_camera.json",
                              "rays_per_pixel": self.sub * self.sub, "blur_px": self.blur_px, "fill": self.fill}}


class StillLog:
    """pose/: the stills and frames.csv, and truth/stills_true.csv with where the camera really was."""

    def __init__(self, folder, truth_folder, camera: PoseCamera, started):
        self.dir = Path(folder)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.camera = camera
        (self.dir / "camera.json").write_text(json.dumps(camera.camera_json(started), indent=2) + "\n")
        self._f = open(self.dir / "frames.csv", "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(STILL_COLUMNS)
        self._tf = open(Path(truth_folder) / "stills_true.csv", "w", newline="")
        self._tw = csv.writer(self._tf)
        self._tw.writerow(TRUE_COLUMNS)
        self.poses = []
        self.count = 0

    def write(self, image, stamp_ns, sweep_id, pose):
        name = "still_%04d.npy" % self.count
        np.save(self.dir / name, image)
        self._w.writerow([self.count, int(stamp_ns), name, "%.1f" % self.camera.exposure_us,
                          "%.4f" % self.camera.gain])
        self._tw.writerow([self.count, int(stamp_ns), sweep_id] + ["%.9f" % v for v in pose_fields(pose)])
        self.poses.append(np.asarray(pose, float))
        self.count += 1

    def close(self):
        self._f.close()
        self._tf.close()


def still_stamps(first_ns, last_ns, n, lead_s=0.3):
    """When the recorder keeps n stills during a sweep: spread over it, from lead_s after its first
    line (headcal solve takes the stills from 0.2 s after a sweep's first line to its last)."""
    lo = first_ns + min(int(lead_s * 1e9), (last_ns - first_ns) // 2)
    return [int(lo + (j + 0.5) * (last_ns - lo) / n) for j in range(n)]
