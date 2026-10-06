"""Frame sets on disk: a folder of raw frames plus a meta.json describing them.

    session/
      dark_60000us/   frame_000.npy ... meta.json   {"kind": "dark", "exposure_us": 60000, ...}
      cfl/            frame_000.npy ... meta.json   {"kind": "lamp", "source": "cfl", ...}
      ...

Frames are raw sensor values (10-bit for the IMX219), one 2-D array per file:
.npy, 16-bit .png/.tif, or .raw dumps from v4l2 (width/height in meta.json).
A single .npy holding an (N, H, W) stack also works.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.ndimage import uniform_filter1d

from .profiles import binomial_smooth

KINDS = ("dark", "flat", "lamp", "laser", "wires", "scene", "white")
FRAME_EXT = (".npy", ".png", ".tif", ".tiff", ".raw")
NAME_HINTS = (("dark", "dark", None), ("flat", "flat", "halogen+ptfe"), ("white", "white", None),
              ("wire", "wires", "halogen+ptfe"), ("cfl", "lamp", "cfl"), ("neon", "lamp", "neon"),
              ("laser", "laser", "laser"), ("scene", "scene", None))


def guess_shift(frame, black=64.0):
    """Bit shift that puts a 10-bit sensor's data back in the low bits.

    Some capture paths store 10-bit pixels in the top of a 16-bit word. Every
    frame here has dark pixels (outside the slit image) that sit at the black
    level, about 64 counts for the IMX219, so pick the shift that brings the
    darkest pixels there."""
    low = max(float(np.percentile(np.asarray(frame), 0.5)), 1.0)
    return min((abs(np.log2(max(low / 2 ** s, 0.5) / black)), s) for s in (0, 2, 4, 6))[1]


def row_pixels(n_words, width, height, n_frames=None):
    """Pixels per stored row of a raw dump: the width plus any padding.

    Drivers pad rows to a multiple of 64 bytes or so. With the frame count
    known the answer is exact; otherwise take the smallest padding that splits
    the file into whole frames."""
    if n_frames:
        per = n_words // (n_frames * height)
        if per >= width and per * n_frames * height == n_words:
            return per
    for per in range(width, width + 513):
        if (per == width or (2 * per) % 64 == 0) and n_words % (per * height) == 0:
            return per
    raise ValueError(f"{n_words * 2} bytes is not a whole number of {width}x{height} frames; "
                     "give stride_bytes in meta.json")


def decode_raw16(data, width, height, stride_bytes=None, shift="auto", bits=10, n_frames=None):
    """Frames from a raw dump of 16-bit little-endian pixels (e.g. v4l2-ctl RG10).

    Rows may be padded: stride_bytes is the length of one row in the file,
    worked out from the file size when not given. Some capture paths put the
    10 bits high in the word (and repeat the top bits below them), so the
    words are shifted down and masked. Returns (N, height, width) uint16."""
    words = np.frombuffer(data, dtype="<u2") if isinstance(data, (bytes, bytearray)) \
        else np.asarray(data, dtype="<u2").ravel()
    if stride_bytes is None:
        per_row = row_pixels(words.size, width, height, n_frames)
    else:
        per_row = stride_bytes // 2
    n = words.size // (per_row * height)
    frames = words[: n * per_row * height].reshape(n, height, per_row)[:, :, :width]
    if shift == "auto":
        shift = guess_shift(frames[0])
    return ((frames >> int(shift)) & ((1 << int(bits)) - 1)).astype(np.uint16)


def read_frames(path, meta=None):
    """All frames in one file, as an (N, H, W) array."""
    path = Path(path)
    meta = meta or {}
    ext = path.suffix.lower()
    if ext == ".npy":
        a = np.load(path)
        return a[None] if a.ndim == 2 else a
    if ext == ".raw":
        return decode_raw16(path.read_bytes(), int(meta["width"]), int(meta["height"]),
                            meta.get("stride_bytes"), meta.get("raw_shift", "auto"), meta.get("bits", 10),
                            meta.get("frames"))
    if ext in (".png", ".tif", ".tiff"):
        try:
            import cv2
            a = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        except ImportError:
            from PIL import Image
            a = np.array(Image.open(path))
        if a is None:
            raise ValueError(f"could not read {path}")
        if a.ndim == 3:
            raise ValueError(f"{path} is a colour image; the calibration needs raw sensor frames")
        shift = meta.get("raw_shift", 0)
        return (a >> int(shift))[None] if shift else a[None]
    raise ValueError(f"unknown frame format {path}")


@dataclass
class FrameSet:
    """One folder of frames of the same thing at the same exposure."""
    name: str
    path: Path
    meta: dict
    _summary: tuple = None

    @property
    def kind(self):
        return self.meta.get("kind")

    @property
    def source(self):
        return self.meta.get("source")

    @property
    def exposure_us(self):
        e = self.meta.get("exposure_us")
        return None if e is None else float(e)

    @property
    def gain(self):
        return float(self.meta.get("gain", 1.0))

    @property
    def max_dn(self):
        return 2 ** int(self.meta.get("bits", 10)) - 1

    def files(self):
        return sorted(p for p in self.path.iterdir() if p.suffix.lower() in FRAME_EXT)

    def stack(self):
        arrs = [read_frames(p, self.meta) for p in self.files()]
        if not arrs:
            raise ValueError(f"no frames in {self.path}")
        return np.concatenate(arrs, axis=0)

    def summary(self):
        """Mean frame (float), saturated pixels (True if any frame clipped), frame count.

        Frames are read one file at a time, so a full-size stack never sits in memory."""
        if self._summary is None:
            total, sat, n = None, None, 0
            for p in self.files():
                a = read_frames(p, self.meta)
                s = a.astype(np.float64).sum(0)
                m = (a >= self.max_dn - 2).any(0)
                total = s if total is None else total + s
                sat = m if sat is None else sat | m
                n += a.shape[0]
            if not n:
                raise ValueError(f"no frames in {self.path}")
            self._summary = (total / n, sat, n)
        return self._summary


def load_session(path):
    """Frame sets in a session folder, keyed by folder name, plus session.json."""
    path = Path(path)
    if not path.is_dir():
        raise FileNotFoundError(f"no session folder at {path}")
    sets = {}
    for sub in sorted(p for p in path.iterdir() if p.is_dir()):
        meta_path = sub / "meta.json"
        if meta_path.exists():
            meta = json.loads(meta_path.read_text())
        else:
            low = sub.name.lower()
            hint = next((h for h in NAME_HINTS if low.startswith(h[0])), None)
            if hint is None or not any(p.suffix.lower() in FRAME_EXT for p in sub.iterdir()):
                continue
            meta = dict(kind=hint[1], source=hint[2], guessed=True)
        if meta.get("kind") not in KINDS:
            continue
        sets[sub.name] = FrameSet(sub.name, sub, meta)
    cfg_path = path / "session.json"
    cfg = json.loads(cfg_path.read_text()) if cfg_path.exists() else {}
    return sets, cfg


def match_dark(light, darks, tol=0.02):
    """The dark set taken at the same exposure and gain, or None."""
    if light.exposure_us is None:
        return None
    best = None
    for d in darks:
        if d.exposure_us is None:
            continue
        if abs(d.exposure_us / light.exposure_us - 1) <= tol and abs(d.gain / light.gain - 1) <= tol:
            if best is None or abs(d.exposure_us - light.exposure_us) < abs(best.exposure_us - light.exposure_us):
                best = d
    return best


@dataclass
class Orientation:
    """How to turn a camera frame so the spectrum runs along x, blue on the left."""
    transpose: bool = False
    flip_x: bool = False
    flip_y: bool = False

    def apply(self, a):
        if self.transpose:
            a = np.swapaxes(a, -1, -2)
        if self.flip_x:
            a = a[..., ::-1]
        if self.flip_y:
            a = a[..., ::-1, :]
        return np.ascontiguousarray(a)

    def to_dict(self):
        return dict(transpose=self.transpose, flip_x=self.flip_x, flip_y=self.flip_y)


def structure(profile, width=41):
    """How spiky a 1-D profile is: high-pass RMS over mean level."""
    p = np.asarray(profile, float)
    hp = p - uniform_filter1d(p, width, mode="nearest")
    return float(np.std(hp) / (np.mean(np.abs(p)) + 1e-9))


def spectrum_is_vertical(img):
    """True if a lamp frame's emission lines run across x (spectrum along y)."""
    a = binomial_smooth(img)
    return structure(a.mean(1)) > structure(a.mean(0))
