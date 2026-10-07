"""Streaming raw frames from a V4L2 capture device, in plain Python (ctypes and ioctl).

On JetPack 4 the IMX219 is /dev/video0, driven by the Tegra VI driver. Reading it through V4L2
gives the sensor's own 10-bit Bayer values (RG10), the same frames the calibration kit's
v4l2-ctl captures, where nvarguscamerasrc would demosaic and tone-map them. This module does
what `v4l2-ctl --stream-mmap` does, but keeps each frame's timestamp and sequence number:

    cam = V4l2Device("/dev/video0")
    cam.set_format(1640, 1232, "RG10")
    cam.set_controls({"bypass_mode": 0, "exposure": 5000, "gain": 16, "frame_rate": 30000000})
    cam.start()
    buf = cam.read(timeout=1.0)     # Buffer: .data (bytes), .timestamp_ns (CLOCK_MONOTONIC), .sequence
    cam.stop(); cam.close()

Controls are named as v4l2-ctl names them (lowercase, underscores). The structs below follow
linux/videodev2.h on a 64-bit kernel (the Nano's aarch64 and any x86-64 PC lay them out the
same way); test_v4l2.py checks their sizes and ioctl numbers against the kernel's values.
"""

import ctypes
import errno
import fcntl
import mmap
import os
import re
import select
from dataclasses import dataclass

# --- ioctl numbers (asm-generic/ioctl.h) -----------------------------------------------------

_IOC_NONE, _IOC_WRITE, _IOC_READ = 0, 1, 2


def _ioc(direction, nr, size):
    return (direction << 30) | (size << 16) | (ord("V") << 8) | nr


def _iowr(nr, struct):
    return _ioc(_IOC_READ | _IOC_WRITE, nr, ctypes.sizeof(struct))


def _ior(nr, struct):
    return _ioc(_IOC_READ, nr, ctypes.sizeof(struct))


def _iow(nr, struct):
    return _ioc(_IOC_WRITE, nr, ctypes.sizeof(struct))


def fourcc(code):
    a, b, c, d = (ord(ch) for ch in code)
    return a | (b << 8) | (c << 16) | (d << 24)


def fourcc_text(value):
    return "".join(chr((value >> (8 * i)) & 0xFF) for i in range(4))


BUF_TYPE_VIDEO_CAPTURE = 1
MEMORY_MMAP = 1
FIELD_NONE = 1
CAP_VIDEO_CAPTURE = 0x00000001
CAP_STREAMING = 0x04000000
CAP_DEVICE_CAPS = 0x80000000
BUF_FLAG_TIMESTAMP_MASK = 0x0000E000
BUF_FLAG_TIMESTAMP_MONOTONIC = 0x00002000
BUF_FLAG_TSTAMP_SRC_MASK = 0x00070000
BUF_FLAG_TSTAMP_SRC_SOE = 0x00010000
BUF_FLAG_ERROR = 0x00000040
CTRL_FLAG_NEXT_CTRL = 0x80000000
CTRL_FLAG_DISABLED = 0x0001
CTRL_FLAG_READ_ONLY = 0x0004
CTRL_TYPE_INTEGER, CTRL_TYPE_BOOLEAN, CTRL_TYPE_MENU, CTRL_TYPE_BUTTON = 1, 2, 3, 4
CTRL_TYPE_INTEGER64, CTRL_TYPE_CTRL_CLASS, CTRL_TYPE_INTEGER_MENU = 5, 6, 9
CTRL_WHICH_CUR_VAL = 0


# --- structs (linux/videodev2.h) -------------------------------------------------------------

class Capability(ctypes.Structure):
    _fields_ = [("driver", ctypes.c_char * 16), ("card", ctypes.c_char * 32), ("bus_info", ctypes.c_char * 32),
                ("version", ctypes.c_uint32), ("capabilities", ctypes.c_uint32),
                ("device_caps", ctypes.c_uint32), ("reserved", ctypes.c_uint32 * 3)]


class PixFormat(ctypes.Structure):
    _fields_ = [("width", ctypes.c_uint32), ("height", ctypes.c_uint32), ("pixelformat", ctypes.c_uint32),
                ("field", ctypes.c_uint32), ("bytesperline", ctypes.c_uint32), ("sizeimage", ctypes.c_uint32),
                ("colorspace", ctypes.c_uint32), ("priv", ctypes.c_uint32), ("flags", ctypes.c_uint32),
                ("ycbcr_enc", ctypes.c_uint32), ("quantization", ctypes.c_uint32), ("xfer_func", ctypes.c_uint32)]


class _FormatUnion(ctypes.Union):
    # raw_data[200] in the kernel; the union is 8-byte aligned because v4l2_window holds pointers
    _fields_ = [("pix", PixFormat), ("raw_data", ctypes.c_uint8 * 200), ("_align", ctypes.c_uint64)]


class Format(ctypes.Structure):
    _fields_ = [("type", ctypes.c_uint32), ("fmt", _FormatUnion)]


class RequestBuffers(ctypes.Structure):
    _fields_ = [("count", ctypes.c_uint32), ("type", ctypes.c_uint32), ("memory", ctypes.c_uint32),
                ("capabilities", ctypes.c_uint32), ("flags", ctypes.c_uint8), ("reserved", ctypes.c_uint8 * 3)]


class Timeval(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_usec", ctypes.c_long)]


class Timecode(ctypes.Structure):
    _fields_ = [("type", ctypes.c_uint32), ("flags", ctypes.c_uint32), ("frames", ctypes.c_uint8),
                ("seconds", ctypes.c_uint8), ("minutes", ctypes.c_uint8), ("hours", ctypes.c_uint8),
                ("userbits", ctypes.c_uint8 * 4)]


class _BufferM(ctypes.Union):
    _fields_ = [("offset", ctypes.c_uint32), ("userptr", ctypes.c_ulong), ("planes", ctypes.c_void_p),
                ("fd", ctypes.c_int32)]


class Buffer(ctypes.Structure):
    _fields_ = [("index", ctypes.c_uint32), ("type", ctypes.c_uint32), ("bytesused", ctypes.c_uint32),
                ("flags", ctypes.c_uint32), ("field", ctypes.c_uint32), ("timestamp", Timeval),
                ("timecode", Timecode), ("sequence", ctypes.c_uint32), ("memory", ctypes.c_uint32),
                ("m", _BufferM), ("length", ctypes.c_uint32), ("reserved2", ctypes.c_uint32),
                ("request_fd", ctypes.c_int32)]


class _ExtControlValue(ctypes.Union):
    _fields_ = [("value", ctypes.c_int32), ("value64", ctypes.c_int64), ("ptr", ctypes.c_void_p)]


class ExtControl(ctypes.Structure):
    _pack_ = 1   # __attribute__((packed)) in the kernel
    _fields_ = [("id", ctypes.c_uint32), ("size", ctypes.c_uint32), ("reserved2", ctypes.c_uint32 * 1),
                ("u", _ExtControlValue)]


class ExtControls(ctypes.Structure):
    _fields_ = [("which", ctypes.c_uint32), ("count", ctypes.c_uint32), ("error_idx", ctypes.c_uint32),
                ("request_fd", ctypes.c_int32), ("reserved", ctypes.c_uint32 * 1),
                ("controls", ctypes.POINTER(ExtControl))]


class QueryExtCtrl(ctypes.Structure):
    _fields_ = [("id", ctypes.c_uint32), ("type", ctypes.c_uint32), ("name", ctypes.c_char * 32),
                ("minimum", ctypes.c_int64), ("maximum", ctypes.c_int64), ("step", ctypes.c_uint64),
                ("default_value", ctypes.c_int64), ("flags", ctypes.c_uint32), ("elem_size", ctypes.c_uint32),
                ("elems", ctypes.c_uint32), ("nr_of_dims", ctypes.c_uint32), ("dims", ctypes.c_uint32 * 4),
                ("reserved", ctypes.c_uint32 * 32)]


VIDIOC_QUERYCAP = _ior(0, Capability)
VIDIOC_G_FMT = _iowr(4, Format)
VIDIOC_S_FMT = _iowr(5, Format)
VIDIOC_REQBUFS = _iowr(8, RequestBuffers)
VIDIOC_QUERYBUF = _iowr(9, Buffer)
VIDIOC_QBUF = _iowr(15, Buffer)
VIDIOC_DQBUF = _iowr(17, Buffer)
VIDIOC_STREAMON = _iow(18, ctypes.c_int)
VIDIOC_STREAMOFF = _iow(19, ctypes.c_int)
VIDIOC_G_EXT_CTRLS = _iowr(71, ExtControls)
VIDIOC_S_EXT_CTRLS = _iowr(72, ExtControls)
VIDIOC_QUERY_EXT_CTRL = _iowr(103, QueryExtCtrl)


class V4l2Error(RuntimeError):
    pass


def control_key(name):
    """A control's name the way v4l2-ctl prints it: "Frame Rate" -> "frame_rate"."""
    key = re.sub(r"[^a-z0-9]+", "_", name.lower())
    return key.strip("_")


@dataclass
class Control:
    id: int
    key: str
    type: int
    minimum: int
    maximum: int
    step: int
    default: int
    flags: int


@dataclass
class Frame:
    """One dequeued buffer, copied out of the driver's memory."""
    data: bytes
    timestamp_ns: int      # the driver's timestamp; CLOCK_MONOTONIC when monotonic is True
    sequence: int
    monotonic: bool
    start_of_exposure: bool  # the driver says it stamps the start of exposure, not the end of frame
    error: bool              # the driver flagged the frame as corrupt


class V4l2Device:
    """One V4L2 capture device, streaming through mmap buffers."""

    def __init__(self, path, ioctl=None, mmap_factory=None, open_fn=None):
        self.path = path
        self._ioctl = ioctl or fcntl.ioctl
        self._mmap = mmap_factory or (lambda fd, length, offset: mmap.mmap(
            fd, length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=offset))
        self.fd = (open_fn or os.open)(path, os.O_RDWR | os.O_NONBLOCK)
        self.buffers = []
        self.streaming = False
        self.format = None
        self.controls = {}
        try:
            cap = Capability()
            self.ioctl(VIDIOC_QUERYCAP, cap)
            caps = cap.device_caps if cap.capabilities & CAP_DEVICE_CAPS else cap.capabilities
            if not caps & CAP_VIDEO_CAPTURE or not caps & CAP_STREAMING:
                raise V4l2Error("%s is not a streaming capture device (%s)" % (path, cap.card.decode(errors="replace")))
            self.driver = cap.driver.decode(errors="replace")
            self.card = cap.card.decode(errors="replace")
            self._find_controls()
        except Exception:
            os.close(self.fd)
            raise

    def ioctl(self, request, arg):
        while True:
            try:
                return self._ioctl(self.fd, request, arg)
            except InterruptedError:
                continue

    # --- format and controls ---------------------------------------------------------------

    def set_format(self, width, height, pixfmt="RG10"):
        """Ask for a size and pixel format; returns the format the driver chose."""
        f = Format()
        f.type = BUF_TYPE_VIDEO_CAPTURE
        f.fmt.pix.width, f.fmt.pix.height = int(width), int(height)
        f.fmt.pix.pixelformat = fourcc(pixfmt)
        f.fmt.pix.field = FIELD_NONE
        self.ioctl(VIDIOC_S_FMT, f)
        g = Format()
        g.type = BUF_TYPE_VIDEO_CAPTURE
        self.ioctl(VIDIOC_G_FMT, g)
        p = g.fmt.pix
        if (p.width, p.height) != (int(width), int(height)) or p.pixelformat != fourcc(pixfmt):
            raise V4l2Error("%s gives %dx%d %s, not %dx%d %s; `v4l2-ctl -d %s --list-formats-ext` lists what it has"
                            % (self.path, p.width, p.height, fourcc_text(p.pixelformat), width, height, pixfmt,
                               self.path))
        self.format = dict(width=p.width, height=p.height, pixfmt=fourcc_text(p.pixelformat),
                           bytesperline=p.bytesperline, sizeimage=p.sizeimage)
        return self.format

    def _find_controls(self):
        q = QueryExtCtrl()
        q.id = CTRL_FLAG_NEXT_CTRL
        while True:
            try:
                self.ioctl(VIDIOC_QUERY_EXT_CTRL, q)
            except OSError as e:
                if e.errno in (errno.EINVAL, errno.ENOTTY):
                    break      # no more controls, or a driver without extended queries
                raise
            if q.type != CTRL_TYPE_CTRL_CLASS:
                key = control_key(q.name.decode(errors="replace"))
                self.controls[key] = Control(q.id, key, q.type, q.minimum, q.maximum, q.step, q.default_value,
                                             q.flags)
            q.id |= CTRL_FLAG_NEXT_CTRL

    def set_controls(self, values):
        """Set controls by their v4l2-ctl names, e.g. {"exposure": 5000, "gain": 16}.
        Values outside a control's range are clamped to it; returns what was set."""
        missing = [k for k in values if k not in self.controls]
        if missing:
            raise V4l2Error("%s has no control %s; it has %s" % (self.path, ", ".join(missing),
                                                                 ", ".join(sorted(self.controls))))
        items = (ExtControl * len(values))()
        applied = {}
        for item, (key, value) in zip(items, values.items()):
            c = self.controls[key]
            if c.flags & (CTRL_FLAG_READ_ONLY | CTRL_FLAG_DISABLED):
                raise V4l2Error("control %s on %s is read-only or disabled" % (key, self.path))
            v = min(max(int(round(value)), c.minimum), c.maximum)
            if c.step > 1:
                v = c.minimum + (v - c.minimum) // c.step * c.step
            item.id = c.id
            if c.type == CTRL_TYPE_INTEGER64:
                item.u.value64 = v
            else:
                item.u.value = v
            applied[key] = v
        ctrls = ExtControls()
        ctrls.which = CTRL_WHICH_CUR_VAL
        ctrls.count = len(values)
        ctrls.controls = items
        try:
            self.ioctl(VIDIOC_S_EXT_CTRLS, ctrls)
        except OSError as e:
            bad = list(values)[ctrls.error_idx] if ctrls.error_idx < len(values) else "?"
            raise V4l2Error("setting %s on %s failed: %s" % (bad, self.path, e)) from None
        return applied

    def get_control(self, key):
        c = self.controls[key]
        item = ExtControl()
        item.id = c.id
        ctrls = ExtControls()
        ctrls.which = CTRL_WHICH_CUR_VAL
        ctrls.count = 1
        ctrls.controls = ctypes.pointer(item)
        self.ioctl(VIDIOC_G_EXT_CTRLS, ctrls)
        return item.u.value64 if c.type == CTRL_TYPE_INTEGER64 else item.u.value

    # --- streaming -------------------------------------------------------------------------

    def start(self, n_buffers=4):
        if self.format is None:
            raise V4l2Error("set_format first")
        req = RequestBuffers()
        req.count, req.type, req.memory = n_buffers, BUF_TYPE_VIDEO_CAPTURE, MEMORY_MMAP
        self.ioctl(VIDIOC_REQBUFS, req)
        if req.count < 2:
            raise V4l2Error("%s gave %d buffers; streaming needs 2 or more" % (self.path, req.count))
        for i in range(req.count):
            b = Buffer()
            b.index, b.type, b.memory = i, BUF_TYPE_VIDEO_CAPTURE, MEMORY_MMAP
            self.ioctl(VIDIOC_QUERYBUF, b)
            self.buffers.append(self._mmap(self.fd, b.length, b.m.offset))
            self.ioctl(VIDIOC_QBUF, b)
        self.ioctl(VIDIOC_STREAMON, ctypes.c_int(BUF_TYPE_VIDEO_CAPTURE))
        self.streaming = True

    def read(self, timeout=1.0):
        """The next frame, or None if none came within timeout seconds."""
        ready, _, _ = select.select([self.fd], [], [], timeout)
        if not ready:
            return None
        b = Buffer()
        b.type, b.memory = BUF_TYPE_VIDEO_CAPTURE, MEMORY_MMAP
        try:
            self.ioctl(VIDIOC_DQBUF, b)
        except BlockingIOError:
            return None
        try:
            used = b.bytesused or b.length
            data = bytes(self.buffers[b.index][:used])
        finally:
            self.ioctl(VIDIOC_QBUF, b)
        return Frame(data=data, timestamp_ns=b.timestamp.tv_sec * 1_000_000_000 + b.timestamp.tv_usec * 1000,
                     sequence=b.sequence,
                     monotonic=(b.flags & BUF_FLAG_TIMESTAMP_MASK) == BUF_FLAG_TIMESTAMP_MONOTONIC,
                     start_of_exposure=(b.flags & BUF_FLAG_TSTAMP_SRC_MASK) == BUF_FLAG_TSTAMP_SRC_SOE,
                     error=bool(b.flags & BUF_FLAG_ERROR))

    def stop(self):
        if self.streaming:
            try:
                self.ioctl(VIDIOC_STREAMOFF, ctypes.c_int(BUF_TYPE_VIDEO_CAPTURE))
            finally:
                self.streaming = False
        for m in self.buffers:
            try:
                m.close()
            except Exception:
                pass
        self.buffers = []
        if self.format is not None:
            req = RequestBuffers()
            req.count, req.type, req.memory = 0, BUF_TYPE_VIDEO_CAPTURE, MEMORY_MMAP
            try:
                self.ioctl(VIDIOC_REQBUFS, req)
            except OSError:
                pass

    def close(self):
        self.stop()
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
