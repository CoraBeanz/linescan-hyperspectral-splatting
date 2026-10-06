#!/usr/bin/env python3
"""Drive the scan-mirror ESP32 from a PC or the Jetson over USB serial.

Each positional argument is one command from PROTOCOL.md, run in order on a
single connection. MOVE, HOME, SCAN and STOP also wait for the event that
ends them, so a whole session fits on one line:

  python mirrorctl.py -p COM5 ENABLE HOME "SCAN start=-100 step=2 lines=100 period=33333.333"
  python mirrorctl.py -p /dev/ttyUSB0 STATUS DRV
  python mirrorctl.py -p COM5 --sync          # device clock vs this computer
  python mirrorctl.py -p COM5 --monitor       # print everything it sends
  python mirrorctl.py -p COM5 --csv lines.csv "SCAN lines=50 period=33333.333"

Needs pyserial (pip install pyserial). Exits non-zero if a command fails.
"""

import argparse
import csv
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial is missing: pip install pyserial")

BAUD = 921600

# Commands that start something, and the events that end it.
FINISH = {
    "MOVE": ("EV MOVED",),
    "HOME": ("EV HOMED", "EV HOME_FAILED"),
    "SCAN": ("EV SCAN_DONE",),
    "STOP": ("EV STOPPED",),
}
FAILURES = ("EV HOME_FAILED", "EV FAULT")


def fields(line):
    """'EV LINE n=3 t=12 pos=-4' -> {'n': '3', 't': '12', 'pos': '-4'}; msg= takes the rest."""
    out = {}
    words = line.split(" ")
    for i, w in enumerate(words):
        if w.startswith("msg="):
            out["msg"] = " ".join(words[i:])[4:]
            break
        key, eq, value = w.partition("=")
        if eq:
            out[key] = value
    return out


class Mirror:
    def __init__(self, port, verbose=True):
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = BAUD
        self.ser.timeout = 0.05
        # DTR and RTS drive the ESP32's reset and boot pins; keep them released.
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        self.buf = b""
        self.next_id = 1
        self.verbose = verbose
        self.lines = []  # EV LINE events seen, as field dicts
        self.events = []  # events that arrived while waiting for a reply

    def readline(self, timeout):
        """One protocol line (OK/ERR/EV ...), or None after timeout seconds."""
        deadline = time.monotonic() + timeout
        while True:
            nl = self.buf.find(b"\n")
            if nl >= 0:
                raw, self.buf = self.buf[:nl], self.buf[nl + 1 :]
                line = raw.decode("ascii", "replace").strip()
                if line.startswith(("OK ", "ERR ", "EV ")):
                    return line
                if line and self.verbose:
                    print("?? " + line)  # e.g. boot ROM output after a reset
                continue
            if time.monotonic() > deadline:
                return None
            self.buf += self.ser.read(self.ser.in_waiting or 1)

    def show(self, line):
        if line.startswith("EV LINE "):
            self.lines.append(fields(line))
            if not self.verbose:
                return
        if line.startswith("EV BOOT"):
            print("note: the ESP32 has just (re)started; it is disabled and not homed")
        print(line)

    def send(self, command, echo=True):
        """Send one command and return its OK/ERR reply (printed if echo)."""
        tag = str(self.next_id)
        self.next_id += 1
        self.ser.write((command.strip() + " id=" + tag + "\n").encode("ascii"))
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            line = self.readline(deadline - time.monotonic())
            if line is None:
                break
            mine = line.startswith(("OK ", "ERR ")) and fields(line).get("id") == tag
            if echo or not mine:
                self.show(line)
            if mine:
                return line
            if line.startswith("EV "):
                self.events.append(line)
        raise TimeoutError("no reply to: " + command)

    def wait_for(self, prefixes, timeout):
        """The first event starting with one of prefixes (or a fault)."""
        prefixes = tuple(prefixes) + ("EV FAULT",)
        while self.events:
            line = self.events.pop(0)
            if line.startswith(prefixes):
                return line
        deadline = time.monotonic() + timeout
        while True:
            line = self.readline(max(0.0, deadline - time.monotonic()))
            if line is None:
                raise TimeoutError("gave up waiting for " + " or ".join(prefixes))
            self.show(line)
            if line.startswith(prefixes):
                return line

    def run(self, command):
        """Run a command to completion. Returns False if it failed."""
        self.events = []
        reply = self.send(command)
        if reply.startswith("ERR "):
            return False
        verb = reply.split(" ")[1]
        if verb not in FINISH:
            return True
        timeout = 75.0 if verb == "HOME" else 30.0
        if verb == "SCAN":
            f = fields(reply)
            lines, period_us = int(f["lines"]), float(f["period"])
            start_in = (int(f["t0"]) - self.device_now()) / 1e6
            timeout = max(0.0, start_in) + lines * period_us / 1e6 + 30.0
        end = self.wait_for(FINISH[verb], timeout)
        return not end.startswith(FAILURES) and fields(end).get("aborted", "0") == "0"

    def device_now(self):
        return int(fields(self.send("PING", echo=False))["t"])

    def sync(self, count=50):
        """Offset from this computer's monotonic clock to device time, from the
        PING with the shortest round trip (its reply is least delayed)."""
        best = None
        for _ in range(count):
            t_send = time.monotonic_ns() // 1000
            reply = self.send("PING", echo=False)
            t_recv = time.monotonic_ns() // 1000
            rtt = t_recv - t_send
            if best is None or rtt < best[0]:
                best = (rtt, int(fields(reply)["t"]) - (t_send + t_recv) // 2)
        return best

    def monitor(self):
        while True:
            line = self.readline(1.0)
            if line is not None:
                print("%.6f %s" % (time.monotonic(), line))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", required=True, help="COM5, /dev/ttyUSB0, ...")
    ap.add_argument("commands", nargs="*", help='commands to run in order, e.g. ENABLE "MOVE pos=100"')
    ap.add_argument("--sync", action="store_true", help="estimate device time minus local time")
    ap.add_argument("--monitor", action="store_true", help="print every line until Ctrl+C")
    ap.add_argument("--csv", metavar="FILE", help="save EV LINE events (n, t, pos, ready) to FILE")
    ap.add_argument("-q", "--quiet", action="store_true", help="do not print each EV LINE")
    args = ap.parse_args()

    m = Mirror(args.port, verbose=not args.quiet)
    ok = True
    try:
        for command in args.commands:
            if not m.run(command):
                ok = False
                break
        if args.sync:
            rtt, offset = m.sync()
            print("device_us = local_monotonic_us %+d  (best round trip %d us)" % (offset, rtt))
        if args.monitor:
            m.monitor()
    except KeyboardInterrupt:
        pass
    except TimeoutError as e:
        print("error: %s" % e, file=sys.stderr)
        ok = False

    if m.lines:
        ready = [int(l["ready"]) - int(l["t"]) for l in m.lines if "ready" in l and l["ready"] != "-1"]
        late = sum(1 for l in m.lines if l.get("ready") == "-1")
        summary = "%d lines" % len(m.lines)
        if ready:
            summary += ", ready %d..%d us after the tick" % (min(ready), max(ready))
        if late:
            summary += ", %d never settled within their period" % late
        print(summary)
    if args.csv and m.lines:
        with open(args.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["n", "t_us", "pos", "ready_us"])
            for l in m.lines:
                w.writerow([l["n"], l["t"], l["pos"], l.get("ready", "")])
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
