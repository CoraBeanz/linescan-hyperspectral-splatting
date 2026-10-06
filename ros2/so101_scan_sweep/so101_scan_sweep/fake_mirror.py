"""A pretend scan-mirror ESP32 on a pseudo-terminal, for running everything without hardware.

    ros2 run so101_scan_sweep fake_scan_mirror --link /tmp/scan_mirror_fake

It speaks mirror_protocol like the firmware: homes, moves, sweeps and stamps every line with
its own microsecond clock, which starts at a random offset and can run a little fast or slow
(--drift-ppm), and it answers after a random USB-like delay, so the bridge's clock mapping is
exercised too.
"""

import argparse
import os
import queue
import random
import select
import signal
import threading
import time
import tty

from so101_scan_sweep import mirror_protocol as mp


class FakeMirror:
    def __init__(self, link=None, steps_per_second=4000.0, home_time=0.3, latency_ms=(0.2, 2.0),
                 clock_offset_us=None, drift_ppm=0.0, start_step=None, seed=None):
        self.rng = random.Random(seed)
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        self.path = os.ttyname(self.slave)
        self.link = link
        if link:
            if os.path.lexists(link):
                os.remove(link)
            os.symlink(self.path, link)
        self.steps_per_second = steps_per_second
        self.home_time = home_time
        self.latency = latency_ms
        self.clock_offset_us = (self.rng.randint(10_000_000, 900_000_000) if clock_offset_us is None
                                else clock_offset_us)
        self.drift = drift_ppm * 1e-6
        self._t0 = time.monotonic()
        # where the motor is before homing, unknown to the host
        self.step = self.rng.randint(-800, 800) if start_step is None else start_step
        self.homed = False
        self.busy = False
        self.sweep_id = 0
        self.received = []          # every command line, for tests
        self.line_times = []        # (sweep_id, index, wall-clock ns) of every LINE sent, for tests
        self.lock = threading.RLock()
        self._job = None            # the running HOME / GOTO / SWEEP thread
        self._cancel = threading.Event()
        self._out = queue.Queue()
        self.running = False
        self._threads = [threading.Thread(target=self._serve, daemon=True),
                         threading.Thread(target=self._writer, daemon=True)]

    # --- clock and motion ---------------------------------------------------------------

    def now_us(self):
        return self.clock_offset_us + int((time.monotonic() - self._t0) * 1e6 * (1.0 + self.drift))

    def _move_to(self, target):
        """Step toward target at steps_per_second; False if cancelled."""
        while True:
            with self.lock:
                if self.step == target:
                    return True
                self.step += 1 if target > self.step else -1
            if self._cancel.wait(1.0 / self.steps_per_second):
                return False

    # --- jobs -----------------------------------------------------------------------------

    def _run(self, fn, *args):
        self._cancel.clear()
        self.busy = True

        def job():
            try:
                fn(*args)
            finally:
                with self.lock:
                    self.busy = False

        self._job = threading.Thread(target=job, daemon=True)
        self._job.start()

    def _home(self):
        if self._cancel.wait(self.home_time):
            return
        with self.lock:
            self.step = 0
            self.homed = True
        self._send("HOMED 0 %d" % self.now_us())

    def _goto(self, target):
        self._move_to(target)

    def _sweep(self, sweep_id, start, steps_per_line, n_lines, period_us):
        if not self._move_to(start):
            self._send("DONE %d %d" % (sweep_id, self.now_us()))
            return
        t_start = time.monotonic()
        for i in range(n_lines):
            wait = t_start + i * period_us * 1e-6 - time.monotonic()
            if wait > 0 and self._cancel.wait(wait):
                break
            if i and not self._move_to(start + i * steps_per_line):
                break
            with self.lock:
                step = self.step
            t_us, wall = self.now_us(), time.time_ns()
            self.line_times.append((sweep_id, i, wall))
            self._send("LINE %d %d %d %d" % (sweep_id, i, step, t_us))
        self._send("DONE %d %d" % (sweep_id, self.now_us()))

    # --- commands -------------------------------------------------------------------------

    def _handle(self, text):
        self.received.append(text)
        word, *f = text.split()
        try:
            args = [int(x) for x in f]
        except ValueError:
            return self._send("ERR %s bad number" % word)
        if word == "PING" and len(args) == 1:
            return self._send("PONG %d %d" % (args[0], self.now_us()))
        if word == "STATUS" and not args:
            with self.lock:
                return self._send("STATUS %d %d %d %d %d" % (self.homed, self.busy, self.step, self.sweep_id,
                                                             self.now_us()))
        if word == "STOP" and not args:
            self._stop_job()
            return self._send("OK STOP")
        if word == "HOME" and not args:
            if self.busy:
                return self._send("ERR HOME busy")
            self._send("OK HOME")
            return self._run(self._home)
        if word == "GOTO" and len(args) == 1:
            if not self.homed:
                return self._send("ERR GOTO not homed")
            if self.busy:
                return self._send("ERR GOTO busy")
            self._send("OK GOTO")
            return self._run(self._goto, args[0])
        if word == "SWEEP" and len(args) == 5:
            sweep_id, start, spl, n, period = args
            if not self.homed:
                return self._send("ERR SWEEP not homed")
            if self.busy:
                return self._send("ERR SWEEP busy")
            if n < 1 or period < 1:
                return self._send("ERR SWEEP bad arguments")
            self.sweep_id = sweep_id
            self._send("OK SWEEP %d" % sweep_id)
            return self._run(self._sweep, sweep_id, start, spl, n, period)
        self._send("ERR %s unknown command" % word)

    def _stop_job(self):
        self._cancel.set()
        if self._job:
            self._job.join(timeout=2.0)

    # --- serial ---------------------------------------------------------------------------

    def _send(self, text):
        self._out.put(text)

    def _writer(self):
        while self.running:
            try:
                text = self._out.get(timeout=0.05)
            except queue.Empty:
                continue
            time.sleep(self.rng.uniform(*self.latency) * 1e-3)
            try:
                os.write(self.master, (text + "\n").encode())
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
                if text:
                    self._handle(text)

    def start(self):
        self.running = True
        for t in self._threads:
            t.start()
        return self

    def stop(self):
        self._stop_job()
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
    ap.add_argument("--steps-per-second", type=float, default=4000.0)
    ap.add_argument("--drift-ppm", type=float, default=30.0, help="how much faster its clock runs")
    args, _ = ap.parse_known_args(argv)  # ros2 run adds --ros-args
    mirror = FakeMirror(args.link, args.steps_per_second, drift_ppm=args.drift_ppm).start()
    print("fake scan-mirror ESP32 on %s (link %s)" % (mirror.path, args.link), flush=True)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    stop.wait()
    mirror.stop()


if __name__ == "__main__":
    main()
