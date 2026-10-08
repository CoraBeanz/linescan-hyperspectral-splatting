"""Record pose-camera stills on the Jetson while a calibration scan runs.

    python3 -m headcal record ~/so101_scan/pose/headcal_1 --device /dev/video1 --auto 0.7
    python3 -m headcal record /tmp/focus --device /dev/video1 --focus     # focusing aid

It grabs a raw frame every --period seconds through v4l2-ctl (as hsical capture does, so this
runs on JetPack 4's Python 3.6 with nothing but numpy), averages each 2 x 2 colour cell into one
grey pixel, and keeps a still whenever the view has stopped changing (the arm holds a
viewpoint while it sweeps) and differs from the last still it kept, and another every --every
seconds while the view holds. The solver later takes the stills whose time falls inside a
sweep; the Jetson's clock stamps both. Ctrl+C stops it.

    <out>/camera.json   format, device, sensor mode, exposure and gain
    <out>/frames.csv    index, stamp_ns, file, exposure_us, gain
    <out>/still_NNNN.npy  uint16 grey, half the sensor mode's width and height (raw counts,
                          black level about 64)
"""

import csv
import json
import os
import time

import numpy as np

FORMAT = "headcal pose frames v1"


def grey(raw):
    """2 x 2 colour cells averaged into one grey pixel (uint16, rounded)."""
    r = raw.astype(np.uint32)
    h, w = (r.shape[0] // 2) * 2, (r.shape[1] // 2) * 2
    s = r[0:h:2, 0:w:2] + r[0:h:2, 1:w:2] + r[1:h:2, 0:w:2] + r[1:h:2, 1:w:2]
    return ((s + 2) // 4).astype(np.uint16)


def change(a, b, black=64.0):
    """Mean absolute difference of two thumbnails, relative to their mean signal."""
    a = a.astype(np.float64)
    b = b.astype(np.float64)
    level = max(0.5 * (a.mean() + b.mean()) - black, 1.0)
    return float(np.abs(a - b).mean() / level)


def sharpness(img, black=64.0):
    """How sharp a grey image is: the mean squared step between neighbouring pixels, relative to
    the mean signal squared. Larger is sharper; only compare numbers of the same view."""
    a = img.astype(np.float64) - black
    level = max(a.mean(), 1.0)
    dx = np.diff(a, axis=1)
    dy = np.diff(a, axis=0)
    return float(1e3 * ((dx ** 2).mean() + (dy ** 2).mean()) / level ** 2)


def focus(cam, out, exposure_us=20000.0, gain=1.0, count=0, log=print):
    """Print the pose camera's sharpness over and over (no stills kept): turn its lens for the
    largest number with the board where the scan will see it."""
    scratch = os.path.join(out, ".grab")
    if not os.path.isdir(scratch):
        os.makedirs(scratch)
    best = None
    i = 0
    log("Sharpness of the pose camera's view; turn the lens for the largest number. Ctrl+C to stop.")
    try:
        while count == 0 or i < count:
            frames, _ = cam.grab(scratch, exposure_us, gain, 1)
            if frames is None:  # dry run
                break
            img = grey(frames[-1])
            s = sharpness(img)
            best = s if best is None else max(best, s)
            peak = 100.0 * float(np.percentile(img, 99.9)) / 1023.0
            log("  sharpness %7.2f   (best %7.2f)   peak %3.0f%%" % (s, best, peak))
            i += 1
    except KeyboardInterrupt:
        pass
    finally:
        for f in os.listdir(scratch):
            os.remove(os.path.join(scratch, f))
        os.rmdir(scratch)
    return best


def auto_exposure(cam, scratch, exposure_us, gain, target, log=print):
    from hsical.capture import levels
    for _ in range(5):
        frames, _ = cam.grab(scratch, exposure_us, gain, 1)
        if frames is None:
            break
        lv = levels(frames[0])
        log("  test shot at %.1f ms: peak %.0f%% of full scale" % (exposure_us / 1000.0, 100 * lv["peak_frac"]))
        if lv["saturated"] > 1e-4:
            exposure_us *= 0.4
        elif lv["peak_frac"] < 0.6 * target or lv["peak_frac"] > 0.95:
            exposure_us *= target / max(lv["peak_frac"], 0.01)
        else:
            break
        exposure_us = min(max(exposure_us, 50.0), 200000.0)
    return exposure_us


def record(cam, out, exposure_us=20000.0, gain=1.0, auto=None, period=1.0, count=0, keep_all=False,
           still_thresh=0.02, new_thresh=0.06, max_grabs=0, every=3.0, log=print):
    """Keep stills in out/ until Ctrl+C, `count` stills, or `max_grabs` grabs. A view that holds
    gets a still as soon as it stops changing and another every `every` seconds, so some land
    inside the sweep (the arm settles for a moment before the mirror starts)."""
    if not os.path.isdir(out):
        os.makedirs(out)
    scratch = os.path.join(out, ".grab")
    if not os.path.isdir(scratch):
        os.makedirs(scratch)
    if auto:
        exposure_us = auto_exposure(cam, scratch, exposure_us, gain, auto, log)
    w, h = cam.size
    info = dict(format=FORMAT, device=cam.cfg.get("device"), sensor_width=w, sensor_height=h,
                width=w // 2, height=h // 2, exposure_us=float(exposure_us), gain=float(gain),
                grey="2 x 2 colour cells averaged, raw counts (black level about 64)",
                started=time.strftime("%Y-%m-%d %H:%M:%S"))
    with open(os.path.join(out, "camera.json"), "w") as f:
        json.dump(info, f, indent=2)
    new_csv = not os.path.exists(os.path.join(out, "frames.csv"))
    fh = open(os.path.join(out, "frames.csv"), "a")
    wr = csv.writer(fh)
    if new_csv:
        wr.writerow(["index", "stamp_ns", "file", "exposure_us", "gain"])
    index = len([f for f in os.listdir(out) if f.startswith("still_")])
    prev = last = None
    last_t = 0.0
    saved = grabs = 0
    log("recording into %s at %.1f ms, gain %.2f; Ctrl+C to stop" % (out, exposure_us / 1000.0, gain))
    try:
        while True:
            t0 = time.time()
            frames, _ = cam.grab(scratch, exposure_us, gain, 1)
            stamp = int(time.time() * 1e9) - 50000000   # the frame came about one frame before the end
            grabs += 1
            if frames is None:  # dry run
                break
            img = grey(frames[-1])
            thumb = img[::8, ::8]
            still = prev is not None and change(thumb, prev) < still_thresh
            new = last is None or change(thumb, last) > new_thresh
            again = time.time() - last_t >= every
            prev = thumb
            if keep_all or (still and (new or again)):
                name = "still_%04d.npy" % index
                np.save(os.path.join(out, name), img)
                wr.writerow([index, stamp, name, float(exposure_us), float(gain)])
                fh.flush()
                log("  still %d (%s)" % (index, time.strftime("%H:%M:%S")))
                index += 1
                saved += 1
                last = thumb
                last_t = time.time()
            if (count and saved >= count) or (max_grabs and grabs >= max_grabs):
                break
            time.sleep(max(0.0, period - (time.time() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        fh.close()
        for f in os.listdir(scratch):
            os.remove(os.path.join(scratch, f))
        os.rmdir(scratch)
    log("kept %d stills" % saved)
    return saved
