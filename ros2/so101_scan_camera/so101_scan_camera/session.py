"""What line_camera writes into a scan folder (the one scan_sweep writes lines.csv to).

    <scan>/frames/camera.json            the camera: sensor, exposure, calibration, binning, timing
    <scan>/frames/frames.csv             one row per scan line: which frame(s) it got, or why none
    <scan>/frames/sweep_<id>/            raw frames as an hsical frame set: frame_<index>.npy, the
                                         first good frame of each line (uint16, 10-bit), meta.json
    <scan>/binned/binning.npz            the grid lines are binned on (LineBinner.save)
    <scan>/binned/sweep_<id>.npy         float32 [lines, slit bins, bands]: each line's mean raw
                                         counts per cell over its good frames; NaN where a pixel
                                         saturated, the cell is empty, or the line got no frame
    <scan>/reference/<name>/             darks and whites (capture_reference), hsical frame sets
    <scan>/binned/reference_<name>.npy   float32 [slit bins, bands], their binned mean

<id> is the sweep id with three digits and <index> the line index with four. frames.csv:

    sweep_id, index         the line (lines.csv has the same pair)
    status                  ok, no_frame (none was exposed entirely while the mirror held
                            still), unsettled (the mirror never settled), dropped (frames came
                            but couldn't be kept: the disk fell behind)
    file                    the raw frame, relative to the scan folder, or empty
    seq, sof_ns             the first good frame: driver sequence number, start (ROS ns)
    exposure_start_ns, exposure_end_ns    its slit rows' exposure window (ROS ns)
    exposure_us, gain       what the line was taken with
    saturated_px            saturated pixels inside the slit's image, most in any of its frames
    n_frames                how many frames the binned line averages

Writing happens on a thread of its own so the camera never waits for the SD card. Raw frames
are big (4 MB at 1640 x 1232), so if writes fall more than max_queue_bytes behind, raw frames
are skipped (and said so) rather than filling the memory.
"""

import csv
import json
import os
import queue
import threading
from datetime import datetime

import numpy as np

FRAMES_FORMAT = "so101_scan frames v1"
CSV_COLUMNS = ["sweep_id", "index", "status", "file", "seq", "sof_ns", "exposure_start_ns", "exposure_end_ns",
               "exposure_us", "gain", "saturated_px", "n_frames"]


def sweep_dir(sweep_id):
    return "sweep_%03d" % sweep_id


def frame_name(index):
    return "frame_%04d.npy" % index


def binned_name(sweep_id):
    return "sweep_%03d.npy" % sweep_id


def write_json(path, data):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=2)
        f.write("\n")
    os.replace(tmp, path)


def write_frame_set(folder, frames, meta):
    """An hsical frame set: frame_000.npy ... and meta.json."""
    os.makedirs(folder, exist_ok=True)
    for i, f in enumerate(frames):
        np.save(os.path.join(folder, "frame_%03d.npy" % i), np.asarray(f, np.uint16))
    write_json(os.path.join(folder, "meta.json"), dict(meta, frames=len(frames), format="npy", bits=10))


class SessionWriter:
    def __init__(self, directory, camera_info, binner=None, raw_every=1, max_queue_bytes=256 << 20, log=None):
        self.dir = os.path.abspath(os.path.expanduser(directory))
        self.frames_dir = os.path.join(self.dir, "frames")
        self.binned_dir = os.path.join(self.dir, "binned")
        self.info = dict(camera_info)
        self.binner = binner
        self.raw_every = int(raw_every)
        self.max_queue_bytes = int(max_queue_bytes)
        self.log = log or (lambda text: None)
        self.counts = {}
        self.sweeps = []
        self.raw_skipped = 0
        self.errors = []
        self._queue = queue.Queue()
        self._queued_bytes = 0
        self._bytes_lock = threading.Lock()
        self._swept_dirs = set()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._csv = None

    # --- the writer thread ----------------------------------------------------------------

    def _run(self):
        while True:
            job = self._queue.get()
            if job is None:
                return
            fn, size = job
            try:
                fn()
            except Exception as e:   # a full disk: keep going, say so at the end
                if len(self.errors) < 20:
                    self.errors.append(str(e))
                self.log("writing to %s failed: %s" % (self.dir, e))
            finally:
                with self._bytes_lock:
                    self._queued_bytes -= size

    def _put(self, fn, size=0, droppable=False):
        with self._bytes_lock:
            if droppable and self._queued_bytes + size > self.max_queue_bytes:
                return False
            self._queued_bytes += size
        self._queue.put((fn, size))
        return True

    # --- the recording --------------------------------------------------------------------

    def start(self):
        os.makedirs(self.frames_dir, exist_ok=True)
        if self.binner is not None:
            os.makedirs(self.binned_dir, exist_ok=True)
            self.binner.save(os.path.join(self.binned_dir, "binning.npz"))
        self.info.update(format=FRAMES_FORMAT, started=datetime.now().isoformat(), raw_every=self.raw_every)
        write_json(os.path.join(self.frames_dir, "camera.json"), self.info)
        self._csv_file = open(os.path.join(self.frames_dir, "frames.csv"), "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(CSV_COLUMNS)
        self._thread.start()
        return self

    def keep_raw(self, index):
        return self.raw_every > 0 and index % self.raw_every == 0

    def write_line(self, line, saturated_px=0, raw=None, binned=True):
        """One finished line (matcher.Line): its frames.csv row, and its raw frame if given.
        Returns the status written."""
        status = line.status
        rel = ""
        if raw is not None:
            rel = os.path.join("frames", sweep_dir(line.sweep_id), frame_name(line.index))
            if not self._put(lambda p=os.path.join(self.dir, rel), a=raw, s=line.sweep_id: self._save_raw(p, a, s),
                             raw.nbytes, droppable=True):
                self.raw_skipped += 1
                rel = ""
                if not binned:
                    status = "dropped"
        f = line.frames[0] if line.frames else None
        row = [line.sweep_id, line.index, status, rel,
               f.sequence if f else "", f.sof_ns if f else "", f.start_ns if f else "", f.end_ns if f else "",
               "%.1f" % f.exposure_us if f else "", "%.4f" % f.gain if f else "", saturated_px if f else "",
               len(line.frames)]
        self._put(lambda r=row: self._csv.writerow(r))
        self.counts[status] = self.counts.get(status, 0) + 1
        return status

    def _save_raw(self, path, image, sweep_id):
        folder = os.path.dirname(path)
        if folder not in self._swept_dirs:
            os.makedirs(folder, exist_ok=True)
            write_json(os.path.join(folder, "meta.json"), dict(
                kind="scene", sweep_id=sweep_id, width=int(image.shape[1]), height=int(image.shape[0]),
                bits=10, format="npy", exposure_us=self.info.get("exposure_us"), gain=self.info.get("gain"),
                note="one frame per scan line, frame_<line index>.npy; the exposure of each is in frames.csv"))
            self._swept_dirs.add(folder)
        np.save(path, image)

    def write_sweep(self, sweep_id, lines):
        """A sweep's binned lines, float32 [n, slit bins, bands]."""
        arr = np.ascontiguousarray(lines, dtype=np.float32)
        self.sweeps.append(dict(sweep_id=sweep_id, lines=int(arr.shape[0]),
                                lines_with_data=int(np.isfinite(arr).any(axis=(1, 2)).sum())))
        self._put(lambda: np.save(os.path.join(self.binned_dir, binned_name(sweep_id)), arr), arr.nbytes)

    def write_reference(self, name, frames, meta, binned=None):
        """A dark or white: its frames as an hsical frame set, and its binned mean."""
        folder = os.path.join(self.dir, "reference", name)
        write_reference(folder, frames, meta)
        if binned is not None:
            os.makedirs(self.binned_dir, exist_ok=True)
            np.save(os.path.join(self.binned_dir, "reference_%s.npy" % name), np.asarray(binned, np.float32))
        return folder

    def close(self):
        """Wait for the writes, close frames.csv and finish camera.json. Returns a summary."""
        self._put(lambda: self._csv_file.flush())
        self._queue.put(None)
        self._thread.join()
        self._csv_file.close()
        self.info.update(finished=datetime.now().isoformat(), lines=dict(self.counts), sweeps=self.sweeps,
                         raw_skipped=self.raw_skipped)
        if self.errors:
            self.info["write_errors"] = self.errors
        write_json(os.path.join(self.frames_dir, "camera.json"), self.info)
        return dict(self.counts, raw_skipped=self.raw_skipped, errors=len(self.errors))


def write_reference(folder, frames, meta):
    """A reference frame set, replacing whatever was in the folder."""
    if os.path.isdir(folder):
        for name in os.listdir(folder):
            if name.endswith(".npy") or name == "meta.json":
                os.remove(os.path.join(folder, name))
    write_frame_set(folder, frames, meta)
    return folder
