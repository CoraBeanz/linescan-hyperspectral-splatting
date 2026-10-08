"""A pretend SO-101 servo bus on a pseudo-terminal, for testing without the arm.

It answers the same packets five STS3215 servos would: ping, read, write, sync
read and sync write, over a register table per servo. With torque on, each
servo moves toward its goal position at a fixed speed; with torque off it stays
where it is (or wherever set_position puts it, which stands in for moving the
arm by hand). The driver and the tools open its pseudo-terminal like a real
serial port.

    ros2 run so101_scan_hardware sts_fake_bus --link /tmp/so101_fake_bus

prints the pseudo-terminal's path and keeps a symlink to it at --link.

For testing the driver's safety checks, a servo's load follows how far it is from
its goal (full load 200 ticks out), and its status packets carry the alarms a
real one would raise: voltage outside its limits, overheating, and overload
after two seconds at full load. A control socket at <link>.ctl (or --control)
takes JSON datagrams that change a servo while the driver runs:

    {"id": 2, "temperature": 66}         deg C
    {"id": 2, "voltage": 4.5}            V
    {"id": 2, "extra_load": 300}         0.1 % of max torque on top, e.g. a weight
    {"id": 2, "obstacle": [0, 2300]}     it can't move outside these ticks; null removes it

control(path, id, **fields) sends one from Python.
"""

import argparse
import json
import os
import socket
import select
import signal
import threading
import time
import tty

from . import sts
from .sts import Reg


# Load per tick between the goal and where the servo is, in 0.1 % of max torque.
LOAD_PER_TICK = 5.0
OVERLOAD_AFTER = 2.0   # s at full load before the overload alarm


class FakeServo:
    def __init__(self, servo_id, ticks=2048, speed=3000.0):
        self.regs = bytearray(256)
        self.speed = speed           # ticks/s with torque on
        self.raw = float(ticks)      # encoder position before the homing offset
        self.extra_load = 0.0        # 0.1 % of max torque, on top of the position error
        self.obstacle = None         # (lo, hi) ticks it can't move outside
        self.full_load_time = 0.0
        r = self.regs
        r[Reg.MODEL_NUMBER:Reg.MODEL_NUMBER + 2] = sts.to_le16(sts.STS3215_MODEL)
        r[Reg.ID] = servo_id
        r[Reg.RETURN_DELAY_TIME] = 250
        r[Reg.MAX_POSITION_LIMIT:Reg.MAX_POSITION_LIMIT + 2] = sts.to_le16(4095)
        r[Reg.P_COEFFICIENT] = 32
        r[Reg.D_COEFFICIENT] = 32
        r[Reg.LOCK] = 1
        r[Reg.MAX_TEMPERATURE_LIMIT] = 70
        r[Reg.MAX_INPUT_VOLTAGE] = 80
        r[Reg.MIN_INPUT_VOLTAGE] = 40
        r[Reg.PRESENT_VOLTAGE] = 74
        r[Reg.PRESENT_TEMPERATURE] = 31
        self._update_present(0.0)
        r[Reg.GOAL_POSITION:Reg.GOAL_POSITION + 2] = r[Reg.PRESENT_POSITION:Reg.PRESENT_POSITION + 2]

    @property
    def id(self):
        return self.regs[Reg.ID]

    def offset(self):
        return sts.decode_signed(sts.le16(self.regs, Reg.HOMING_OFFSET), 11)

    def present(self):
        return int(round(self.raw - self.offset())) % sts.TICKS_PER_TURN

    def _update_present(self, velocity, load=0.0):
        r = self.regs
        r[Reg.PRESENT_POSITION:Reg.PRESENT_POSITION + 2] = sts.to_le16(self.present())
        r[Reg.PRESENT_VELOCITY:Reg.PRESENT_VELOCITY + 2] = sts.to_le16(sts.encode_signed(round(velocity), 15))
        r[Reg.PRESENT_LOAD:Reg.PRESENT_LOAD + 2] = sts.to_le16(sts.encode_signed(round(load), 10))
        r[Reg.MOVING] = 1 if abs(velocity) > 1 else 0

    def error_bits(self):
        """The alarms in its status packets."""
        r = self.regs
        bits = 0
        if not r[Reg.MIN_INPUT_VOLTAGE] <= r[Reg.PRESENT_VOLTAGE] <= r[Reg.MAX_INPUT_VOLTAGE]:
            bits |= sts.ERR_VOLTAGE
        if r[Reg.PRESENT_TEMPERATURE] >= r[Reg.MAX_TEMPERATURE_LIMIT]:
            bits |= sts.ERR_OVERHEAT
        if self.full_load_time >= OVERLOAD_AFTER:
            bits |= sts.ERR_OVERLOAD
        return bits

    def control(self, fields):
        """Apply a control-socket message (see the module docstring)."""
        if "temperature" in fields:
            self.regs[Reg.PRESENT_TEMPERATURE] = int(fields["temperature"])
        if "voltage" in fields:
            self.regs[Reg.PRESENT_VOLTAGE] = int(round(float(fields["voltage"]) * 10))
        if "extra_load" in fields:
            self.extra_load = float(fields["extra_load"])
        if "obstacle" in fields:
            self.obstacle = tuple(fields["obstacle"]) if fields["obstacle"] is not None else None

    def set_position(self, ticks):
        """Move the servo by hand (only sticks while torque is off)."""
        self.raw = float(ticks) + self.offset()
        self._update_present(0.0)

    def step(self, dt):
        velocity, load = 0.0, 0.0
        if self.regs[Reg.TORQUE_ENABLE]:
            goal = sts.le16(self.regs, Reg.GOAL_POSITION) + self.offset()
            err = goal - self.raw
            move = max(-self.speed * dt, min(self.speed * dt, err))
            new = self.raw + move
            if self.obstacle is not None:   # it can't go further outside, only back
                lo, hi = (v + self.offset() for v in self.obstacle)
                if new > hi and move > 0:
                    new = max(self.raw, hi)
                if new < lo and move < 0:
                    new = min(self.raw, lo)
            velocity = (new - self.raw) / dt if dt > 0 else 0.0
            self.raw = new
            load = max(-1000.0, min(1000.0, LOAD_PER_TICK * (goal - self.raw) + self.extra_load))
        self.full_load_time = self.full_load_time + dt if abs(load) >= 1000 else 0.0
        self._update_present(velocity, load)

    def write(self, address, data):
        self.regs[address:address + len(data)] = data


class FakeBus:
    def __init__(self, ids=(1, 2, 3, 4, 5), ticks=None, link=None, rate=200.0, control_path=None):
        ticks = ticks or {}
        self.servos = {i: FakeServo(i, ticks.get(i, 2048)) for i in ids}
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        self.path = os.ttyname(self.slave)
        self.link = link
        if link:
            if os.path.lexists(link):
                os.remove(link)
            os.symlink(self.path, link)
        self.rate = rate
        self.lock = threading.Lock()
        self.running = True
        self.packets = 0
        self._threads = [threading.Thread(target=self._serve, daemon=True),
                         threading.Thread(target=self._physics, daemon=True)]
        self.control_path = control_path or (link + ".ctl" if link else None)
        self._control = None
        if self.control_path:
            if os.path.lexists(self.control_path):
                os.remove(self.control_path)
            self._control = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            self._control.bind(self.control_path)
            self._control.settimeout(0.05)
            self._threads.append(threading.Thread(target=self._controls, daemon=True))

    def start(self):
        for t in self._threads:
            t.start()
        return self

    def stop(self):
        self.running = False
        for t in self._threads:
            t.join(timeout=1.0)
        if self.link and os.path.islink(self.link):
            os.remove(self.link)
        if self._control is not None:
            self._control.close()
            if os.path.lexists(self.control_path):
                os.remove(self.control_path)
        os.close(self.master)
        os.close(self.slave)

    def _controls(self):
        while self.running:
            try:
                data = self._control.recv(4096)
            except (socket.timeout, OSError):
                continue
            try:
                msg = json.loads(data)
                with self.lock:
                    self.servos[int(msg["id"])].control(msg)
            except (ValueError, KeyError, TypeError) as e:
                print("fake bus: bad control message %r: %s" % (data, e), flush=True)

    def _physics(self):
        dt = 1.0 / self.rate
        while self.running:
            with self.lock:
                for s in self.servos.values():
                    s.step(dt)
            time.sleep(dt)

    def _reply(self, servo, params=b""):
        os.write(self.master, sts.packet(servo.id, servo.error_bits(), params))

    def _handle(self, servo_id, code, params):
        self.packets += 1
        with self.lock:
            if code == sts.SYNC_WRITE and servo_id == sts.BROADCAST:
                addr, n = params[0], params[1]
                for k in range(2, len(params), n + 1):
                    s = self.servos.get(params[k])
                    if s:
                        s.write(addr, params[k + 1:k + 1 + n])
                return
            if code == sts.SYNC_READ and servo_id == sts.BROADCAST:
                addr, n = params[0], params[1]
                for i in params[2:]:
                    s = self.servos.get(i)
                    if s:
                        self._reply(s, bytes(s.regs[addr:addr + n]))
                return
            s = self.servos.get(servo_id)
            if s is None:
                return  # nobody at that id: silence, like a real bus
            if code == sts.PING:
                self._reply(s)
            elif code == sts.READ:
                addr, n = params[0], params[1]
                self._reply(s, bytes(s.regs[addr:addr + n]))
            elif code == sts.WRITE:
                addr = params[0]
                old_id = s.id
                s.write(addr, params[1:])
                if s.id != old_id:
                    self.servos[s.id] = self.servos.pop(old_id)
                self._reply(s)

    def _serve(self):
        parser = sts.Parser()
        while self.running:
            ready, _, _ = select.select([self.master], [], [], 0.05)
            if not ready:
                continue
            try:
                data = os.read(self.master, 1024)
            except OSError:
                return
            parser.feed(data)
            for servo_id, code, params, _ in parser.packets():
                self._handle(servo_id, code, params)


def control(path, servo_id, **fields):
    """Change a servo on a running fake bus through its control socket."""
    with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as s:
        s.sendto(json.dumps(dict(fields, id=servo_id)).encode(), path)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Pretend SO-101 servo bus on a pseudo-terminal")
    ap.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3, 4, 5])
    ap.add_argument("--link", default="/tmp/so101_fake_bus", help="symlink to the pseudo-terminal")
    ap.add_argument("--ticks", type=int, nargs="*", default=[],
                    help="start positions in ticks, one per id (default 2048, the middle)")
    ap.add_argument("--control", help="control socket (default: the link with .ctl added)")
    args, _ = ap.parse_known_args(argv)
    ticks = dict(zip(args.ids, args.ticks))
    bus = FakeBus(args.ids, ticks, args.link, control_path=args.control).start()
    print("fake STS bus with servos %s on %s (link %s, control %s)" % (args.ids, bus.path, args.link,
                                                                       bus.control_path), flush=True)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    stop.wait()
    bus.stop()


if __name__ == "__main__":
    main()
