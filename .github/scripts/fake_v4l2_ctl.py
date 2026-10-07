"""A stand-in for v4l2-ctl, so CI can run hsical's capture commands without a camera.

It takes the command lines hsical builds for the Jetson's IMX219 and writes
what the driver would: 16-bit raw frames, rows padded to 64 bytes, of a dark
sensor with three lamp lines whose brightness follows the exposure and gain.
Python 3.6 and numpy 1.13, like JetPack 4.
"""
import sys

import numpy as np


def main(argv):
    opts = {}
    for a in argv:
        if a.startswith("--") and "=" in a:
            k, v = a[2:].split("=", 1)
            opts[k] = v
    if "stream-to" not in opts:
        return 0  # --list-formats-ext, --list-ctrls: nothing to show
    fmt = dict(kv.split("=", 1) for kv in opts["set-fmt-video"].split(","))
    w, h = int(fmt["width"]), int(fmt["height"])
    ctrls = dict(kv.split("=", 1) for kv in argv[argv.index("--set-ctrl") + 1].split(","))
    exposure_us = float(ctrls.get("exposure", 10000))
    gain = float(ctrls.get("gain", 16)) / 16.0
    n = int(opts.get("stream-count", 1))
    stride = (2 * w + 63) // 64 * 32  # pixels per row, padded to 64 bytes

    rng = np.random.RandomState(0)
    x = np.arange(w, dtype=float)
    lines = sum(a * np.exp(-0.5 * ((x - c * w) / 3.0) ** 2) for c, a in ((0.2, 1.0), (0.45, 0.6), (0.7, 0.3)))
    rows = np.ones((h, 1))
    rows[: h // 20] = rows[h - h // 20:] = 0.0  # beyond the slit's ends
    signal = 900.0 * (exposure_us / 40000.0) * gain * rows * lines
    frames = np.zeros((n, h, stride), dtype="<u2")
    for i in range(n):
        f = 64.0 + rng.normal(0.0, 2.0, (h, w)) + rng.poisson(np.maximum(signal, 0.0))
        frames[i, :, :w] = np.clip(np.round(f), 0, 1023)
    frames.tofile(opts["stream-to"])
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
