"""A pretend SO-101 servo bus on a pseudo-terminal, for testing without the arm.

It answers the same packets five STS3215 servos would: ping, read, write, sync
read and sync write, over a register table per servo. With torque on, each
servo moves toward its goal position at a fixed speed; with torque off it stays
where it is (or wherever set_position puts it, which stands in for moving the
arm by hand). The driver and the tools open its pseudo-terminal like a real
serial port.

    ros2 run so101_scan_hardware sts_fake_bus --link /tmp/so101_fake_bus

prints the pseudo-terminal's path and keeps a symlink to it at --link.
"""

import argparse
import os
import select
import signal
import threading
import time
import tty

from . import sts
from .sts import Reg


class FakeServo:
    def __init__(self, servo_id, ticks=2048, speed=3000.0):
        self.regs = bytearray(256)
        self.speed = speed           # ticks/s with torque on
        self.raw = float(ticks)      # encoder position before the homing offset
        r = self.regs
        r[Reg.MODEL_NUMBER:Reg.MODEL_NUMBER + 2] = sts.to_le16(sts.STS3215_MODEL)
        r[Reg.ID] = servo_id
        r[Reg.RETURN_DELAY_TIME] = 250
        r[Reg.MAX_POSITION_LIMIT:Reg.MAX_POSITION_LIMIT + 2] = sts.to_le16(4095)
        r[Reg.P_COEFFICIENT] = 32
        r[Reg.D_COEFFICIENT] = 32
        r[Reg.LOCK] = 1
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

    def _update_present(self, velocity):
        r = self.regs
        r[Reg.PRESENT_POSITION:Reg.PRESENT_POSITION + 2] = sts.to_le16(self.present())
        r[Reg.PRESENT_VELOCITY:Reg.PRESENT_VELOCITY + 2] = sts.to_le16(sts.encode_signed(round(velocity), 15))
        r[Reg.MOVING] = 1 if abs(velocity) > 1 else 0

    def set_position(self, ticks):
        """Move the servo by hand (only sticks while torque is off)."""
        self.raw = float(ticks) + self.offset()
        self._update_present(0.0)

    def step(self, dt):
        velocity = 0.0
        if self.regs[Reg.TORQUE_ENABLE]:
            goal = sts.le16(self.regs, Reg.GOAL_POSITION) + self.offset()
            err = goal - self.raw
            move = max(-self.speed * dt, min(self.speed * dt, err))
            self.raw += move
            velocity = move / dt if dt > 0 else 0.0
        self._update_present(velocity)

    def write(self, address, data):
        self.regs[address:address + len(data)] = data


class FakeBus:
    def __init__(self, ids=(1, 2, 3, 4, 5), ticks=None, link=None, rate=200.0):
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
        os.close(self.master)
        os.close(self.slave)

    def _physics(self):
        dt = 1.0 / self.rate
        while self.running:
            with self.lock:
                for s in self.servos.values():
                    s.step(dt)
            time.sleep(dt)

    def _reply(self, servo, params=b""):
        os.write(self.master, sts.packet(servo.id, 0, params))

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


def main(argv=None):
    ap = argparse.ArgumentParser(description="Pretend SO-101 servo bus on a pseudo-terminal")
    ap.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3, 4, 5])
    ap.add_argument("--link", default="/tmp/so101_fake_bus", help="symlink to the pseudo-terminal")
    ap.add_argument("--ticks", type=int, nargs="*", default=[],
                    help="start positions in ticks, one per id (default 2048, the middle)")
    args, _ = ap.parse_known_args(argv)
    ticks = dict(zip(args.ids, args.ticks))
    bus = FakeBus(args.ids, ticks, args.link).start()
    print("fake STS bus with servos %s on %s (link %s)" % (args.ids, bus.path, args.link), flush=True)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    stop.wait()
    bus.stop()


if __name__ == "__main__":
    main()
