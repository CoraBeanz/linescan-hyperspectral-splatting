"""A pretend scan-mirror ESP32 on a pseudo-terminal, for running everything without hardware.

    ros2 run so101_scan_sweep fake_scan_mirror --link /tmp/scan_mirror_fake

It speaks the firmware's protocol (firmware/PROTOCOL.md) as far as the bridge needs: INFO, PING,
STATUS, ENABLE, DISABLE, HOME, MOVE, STOP, stare SCANs and REBOOT, with the replies, events and
errors the firmware sends. Like the real board it starts with the motor off and not homed. Its
microsecond clock starts at a random offset and can run a little fast or slow (--drift-ppm),
and every line it sends arrives after a random USB-like delay, so the bridge's clock mapping is
exercised too. Sweep-mode scans, NUDGE, PERIOD and the settings commands aren't in it.
"""

import argparse
import math
import os
import queue
import random
import select
import signal
import threading
import time
import tty

BUSY = ("moving", "stopping", "homing", "scanning")


class Abort(Exception):
    """The running job was ended by STOP, DISABLE, REBOOT or a newer MOVE."""


def _fmt(v):
    if isinstance(v, float):
        return ("%.3f" % v).rstrip("0").rstrip(".")
    return str(v)


class FakeMirror:
    def __init__(self, link=None, vmax=3200, vstart=1600, settle_us=3000, home_time=0.3, hall=True,
                 latency_ms=(0.2, 2.0), clock_offset_us=None, drift_ppm=0.0, start_pos=None, seed=None):
        self.rng = random.Random(seed)
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        self.path = os.ttyname(self.slave)
        self.link = link
        if link:
            if os.path.lexists(link):
                os.remove(link)
            os.symlink(self.path, link)
        self.vmax, self.vstart, self.settle_us = vmax, vstart, settle_us
        self.home_time, self.hall = home_time, hall
        self.latency = latency_ms
        self.drift = drift_ppm * 1e-6
        self.limits = (-3200, 3200)
        self.drop = set()           # names of events to lose on the way, for tests
        self.lock = threading.RLock()
        self.received = []          # every command line, for tests
        self.line_times = []        # (scan number, line, wall-clock ns when it was ready), for tests
        self.scans = 0
        self._job = None
        self._abort = threading.Event()
        self._out = queue.Queue()
        self.running = False
        self._threads = [threading.Thread(target=self._serve, daemon=True),
                         threading.Thread(target=self._writer, daemon=True)]
        self._power_up(self.rng.randint(10_000_000, 900_000_000) if clock_offset_us is None else clock_offset_us,
                       start_pos, "poweron")

    def _power_up(self, clock_offset_us, start_pos, reset):
        with self.lock:
            self.clock_offset_us, self._t0 = clock_offset_us, time.monotonic()
            # where the motor is before homing, unknown to the host
            self.pos = self.rng.randint(-800, 800) if start_pos is None else start_pos
            self.target, self._motion = self.pos, None
            self.state, self.homed = "disabled", False
            self.line = self.lines = 0
            # the boot ROM's message at 115200 baud reads as a few bytes and no newline at 921600
            self._out.put(b"\x00\x80\x80\x00\xf8\x80")
            self._ev("BOOT", fw="fake", proto=1, reset=reset, drv="ok", t=self.now_us())

    # --- clock and motion -----------------------------------------------------------------
    # The firmware moves the motor and ticks scan lines from a timer interrupt, and every time
    # it reports is when the interrupt acted. The fake works those times out from its schedule
    # instead of reading its clock after a sleep, so a busy machine that wakes it late delays
    # its events but doesn't change the times in them.

    def now_us(self):
        return self.clock_offset_us + int((time.monotonic() - self._t0) * 1e6 * (1.0 + self.drift))

    def wall_ns(self, t_us):
        """The wall-clock time at which the fake's clock reads t_us."""
        return time.time_ns() + int((t_us - self.now_us()) * 1000 / (1.0 + self.drift))

    def _wait_until_us(self, t_us):
        seconds = self._t0 + (t_us - self.clock_offset_us) / (1e6 * (1.0 + self.drift)) - time.monotonic()
        if self._abort.wait(max(0.0, seconds)):
            raise Abort

    def _pos_at(self, t_us):
        if self._motion is None:
            return self.pos
        start, target, t_start, speed = self._motion
        done = int(speed * max(0.0, t_us - t_start) * 1e-6)
        if done >= abs(target - start):
            return target
        return start + (done if target > start else -done)

    def _pos_now(self):
        return self._pos_at(self.now_us())

    def _move(self, target, speed, t_start):
        """Turn to target from t_start at a constant speed (no ramps here). Returns when it
        gets there, on the fake's clock; the job waits for that and then calls _arrive."""
        with self.lock:
            start = self.pos = self._pos_at(t_start)
            self.target, self._motion = target, (start, target, t_start, speed)
        return t_start + abs(target - start) / speed * 1e6

    def _arrive(self):
        """The motor is at its target (call with self.lock held). Raises Abort if the job was ended."""
        if self._abort.is_set():
            raise Abort
        self.pos, self._motion = self.target, None

    # --- jobs: what runs between a command's reply and the event that ends it --------------

    def _start(self, state, job, *args):
        self._abort.clear()
        with self.lock:
            self.state = state

        def run():
            try:
                job(*args)
            except Abort:
                pass

        self._job = threading.Thread(target=run, daemon=True)
        self._job.start()

    def _end_job(self):
        """Abort the running job, wait for it, and stop the motor where it got to. Never call
        with self.lock held: the job needs it."""
        self._abort.set()
        if self._job is not None:
            self._job.join(timeout=2.0)
            self._job = None
        with self.lock:
            self.pos = self.target = self._pos_now()
            self._motion = None

    def _halt(self, why):
        """End the running job as the firmware does, with the events it sends for that."""
        self._end_job()
        with self.lock:
            if self.state == "homing":
                self._ev("HOME_FAILED", reason=why, pos=self.pos, t=self.now_us())
            elif self.state == "scanning":
                self._ev("SCAN_DONE", lines=self.line, t=self.now_us(), pos=self.pos, aborted=1)
            was, self.state = self.state, "idle"
        return was

    def _home(self):
        done = self.now_us() + self.home_time * 1e6
        self._wait_until_us(done)
        with self.lock:
            if self._abort.is_set():
                raise Abort
            self.state = "idle"
            if not self.hall:
                return self._ev("HOME_FAILED", reason="not_found", pos=self.pos, t=round(done))
            # the hall window's middle is home_pos (-711); HOME then parks at 0, the 45 deg rest
            shift, self.pos, self.target, self.homed = -self.pos, 0, 0, True
            self._ev("HOMED", pos=0, t=round(done), width=131, shift=shift)

    def _go(self, target):
        done = self._move(target, self.vmax, self.now_us()) + self.settle_us
        self._wait_until_us(done)
        with self.lock:
            self._arrive()
            self.state = "idle"
            self._ev("MOVED", pos=self.pos, t=round(done))

    def _scan(self, start, step, lines, period, t0, settle_us):
        with self.lock:
            self.line, self.lines = 0, lines
        ready = self.now_us()
        if self.pos != start:
            ready = self._move(start, self.vmax, ready) + settle_us
            self._wait_until_us(ready)
            with self.lock:
                self._arrive()
        # ticks that come before the mirror has arrived and settled are skipped, whole periods
        first = t0 + max(0, math.ceil((ready - t0) / period)) * period
        for n in range(lines):
            tick = first + n * period
            ready = tick   # line 0: already there
            if n:
                self._wait_until_us(tick)
                ready = self._move(start + n * step, self.vstart, tick) + settle_us
            # it reports the line once settled, or at the next tick if it is still moving then
            settled = ready < tick + period
            sent = ready if settled else tick + period
            self._wait_until_us(sent)
            with self.lock:
                if settled:
                    self._arrive()
                elif self._abort.is_set():
                    raise Abort
                self.line = n + 1
                self.line_times.append((self.scans, n, self.wall_ns(ready if settled else tick)))
                self._ev("LINE", n=n, t=round(tick), pos=self._pos_at(sent), ready=round(ready) if settled else -1)
        done = first + lines * period   # the tick after the last line
        self._wait_until_us(done)
        with self.lock:
            if self._abort.is_set():
                raise Abort
            self.pos = self._pos_at(done)
            self.state = "idle"
            self._ev("SCAN_DONE", lines=lines, t=round(done), pos=self.pos, aborted=0)

    # --- commands -------------------------------------------------------------------------

    def _handle(self, text):
        self.received.append(text)
        words = text.split()
        verb, args, cid = words[0].upper(), {}, None
        for w in words[1:]:
            key, eq, value = w.partition("=")
            key = key.lower()
            if not eq or key in args:
                return self._err(verb, "syntax", cid, "not VERB key=value ...")
            if key == "id":
                cid = value
            else:
                args[key] = value
        handler = getattr(self, "_cmd_" + verb.lower(), None)
        if handler is None:
            return self._err(verb, "unknown_cmd", cid, "the fake mirror doesn't do %s" % verb)
        try:
            handler(args, cid)   # one at a time, from this thread, like the firmware's loop()
        except ValueError as e:
            self._err(verb, "bad_arg", cid, str(e))

    def _not_now(self, verb, cid):
        code = "disabled" if self.state in ("disabled", "fault") else "busy"
        self._err(verb, code, cid, "not allowed while %s" % self.state)

    def _cmd_info(self, args, cid):
        self._ok("INFO", cid, fw="fake", proto=1, usteps=32, full_steps=200, tick_us=50, max_rate=10000)

    def _cmd_ping(self, args, cid):
        self._ok("PING", cid, t=self.now_us())

    def _cmd_status(self, args, cid):
        with self.lock:
            self._ok("STATUS", cid, state=self.state, pos=self._pos_now(), target=self.target,
                     homed=int(self.homed), en=int(self.state not in ("disabled", "fault")), hall=0,
                     line=self.line, lines=self.lines, drv="ok", fault="none", dropped=0, t=self.now_us())

    def _cmd_enable(self, args, cid):
        with self.lock:
            if self.state in BUSY:
                return self._ok("ENABLE", cid)
            self.state = "idle"
            self._ok("ENABLE", cid, drv="ok")

    def _cmd_disable(self, args, cid):
        self._ok("DISABLE", cid)
        was = self._halt("disabled")
        with self.lock:
            if was in BUSY:
                self._ev("STOPPED", pos=self.pos, t=self.now_us())
            self.state, self.homed = "disabled", False

    def _cmd_home(self, args, cid):
        if self.state != "idle":
            return self._not_now("HOME", cid)
        self._ok("HOME", cid)
        self._start("homing", self._home)

    def _cmd_move(self, args, cid):
        if sorted(args) not in (["pos"], ["rel"]):
            raise ValueError("give exactly one of pos= or rel= (the fake takes no v= or a=)")
        with self.lock:
            if self.state not in ("idle", "moving"):
                return self._not_now("MOVE", cid)
            target = int(args["pos"]) if "pos" in args else self.target + int(args["rel"])
        if not self.limits[0] <= target <= self.limits[1]:
            return self._err("MOVE", "range", cid, "target %d is outside min..max %d..%d" % ((target,) + self.limits))
        self._end_job()   # a MOVE while moving replaces the old target, quietly
        self._ok("MOVE", cid, pos=target)
        self._start("moving", self._go, target)

    def _cmd_stop(self, args, cid):
        self._ok("STOP", cid)
        if self.state not in ("disabled", "fault"):
            self._halt("stopped")
        with self.lock:
            self._ev("STOPPED", pos=self.pos, t=self.now_us())

    def _cmd_scan(self, args, cid):
        known = {"mode", "start", "step", "lines", "period", "t0", "delay", "settle"}
        if set(args) - known:
            raise ValueError("unknown key '%s'" % sorted(set(args) - known)[0])
        if args.get("mode", "stare") != "stare":
            raise ValueError("the fake mirror only does stare scans")
        if "lines" not in args or "period" not in args:
            raise ValueError("lines= and period= are required")
        if "t0" in args and "delay" in args:
            raise ValueError("give t0= or delay=, not both")
        lines, period = int(args["lines"]), float(args["period"])
        start, step = int(args.get("start", self.pos)), int(args.get("step", 1))
        settle = int(args.get("settle", self.settle_us))
        if lines < 1 or not 1000 <= period <= 60e6:
            raise ValueError("lines must be at least 1 and period 1000..60000000 us")
        if self.state != "idle":
            return self._not_now("SCAN", cid)
        last = start + step * (lines - 1)
        if min(start, last) < self.limits[0] or max(start, last) > self.limits[1]:
            return self._err("SCAN", "range", cid, "scan covers %d..%d, outside min..max %d..%d"
                             % ((min(start, last), max(start, last)) + self.limits))
        now = self.now_us()
        t0 = int(args["t0"]) if "t0" in args else now + int(args.get("delay", 0))
        if t0 < now + 2000:  # too soon: on by whole periods, keeping the phase
            t0 += math.ceil((now + 2000 - t0) / period) * period
        self.scans += 1
        self._ok("SCAN", cid, mode="stare", t0=round(t0), period=period, start=start, step=step, lines=lines)
        self._start("scanning", self._scan, start, step, lines, period, t0, settle)

    def _cmd_reboot(self, args, cid):
        self._ok("REBOOT", cid)
        self.reboot()

    def reboot(self, reset="software"):
        """Restart like the ESP32 does: motor off, not homed, its clock from zero."""
        self._end_job()
        self._power_up(0, None, reset)

    # --- serial ---------------------------------------------------------------------------

    def _send(self, text):
        self._out.put(text)

    def _ok(self, verb, cid, **fields):
        self._send(" ".join(["OK", verb] + ["%s=%s" % (k, _fmt(v)) for k, v in fields.items()]
                            + (["id=" + cid] if cid else [])))

    def _err(self, verb, code, cid, msg):
        self._send(" ".join(["ERR", verb, "code=" + code] + (["id=" + cid] if cid else []) + ["msg=" + msg]))

    def _ev(self, name, **fields):
        if name not in self.drop:
            self._send(" ".join(["EV", name] + ["%s=%s" % (k, _fmt(v)) for k, v in fields.items()]))

    def _writer(self):
        while self.running:
            try:
                text = self._out.get(timeout=0.05)
            except queue.Empty:
                continue
            time.sleep(self.rng.uniform(*self.latency) * 1e-3)
            try:
                os.write(self.master, text if isinstance(text, bytes) else (text + "\n").encode())
            except OSError:
                return

    def _serve(self):
        buf = b""
        while self.running:
            ready, _, _ = select.select([self.master], [], [], 0.05)
            if not ready:
                continue
            try:
                buf += os.read(self.master, 1024)
            except OSError:
                return
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                text = raw.decode(errors="replace").strip()
                if text and not text.startswith("#"):
                    self._handle(text)

    def start(self):
        self.running = True
        for t in self._threads:
            t.start()
        return self

    def stop(self):
        self._end_job()
        self.running = False
        for t in self._threads:
            t.join(timeout=1.0)
        if self.link and os.path.islink(self.link):
            os.remove(self.link)
        os.close(self.master)
        os.close(self.slave)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Pretend scan-mirror ESP32 on a pseudo-terminal")
    ap.add_argument("--link", default="/tmp/scan_mirror_fake", help="symlink to the pseudo-terminal")
    ap.add_argument("--drift-ppm", type=float, default=30.0, help="how much faster its clock runs")
    args, _ = ap.parse_known_args(argv)  # ros2 run adds --ros-args
    mirror = FakeMirror(args.link, drift_ppm=args.drift_ppm).start()
    print("fake scan-mirror ESP32 on %s (link %s)" % (mirror.path, args.link), flush=True)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    stop.wait()
    mirror.stop()


if __name__ == "__main__":
    main()
