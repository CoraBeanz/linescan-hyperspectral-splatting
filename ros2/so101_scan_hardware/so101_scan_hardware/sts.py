"""Feetech STS servo bus protocol in Python.

The same protocol as the C++ driver (src/sts_protocol.cpp), for the tools that
run outside the control loop: the bus scan, the calibration tool and the fake
bus used for testing without the arm.

Packet: 0xFF 0xFF ID LEN INSTR|ERROR PARAMS... CHECKSUM, LEN = len(PARAMS) + 2,
CHECKSUM = ~(ID + LEN + INSTR + sum(PARAMS)) & 0xFF. Two-byte registers are
little-endian.
"""

import math
import time

BROADCAST = 0xFE
TICKS_PER_TURN = 4096
STS3215_MODEL = 777

PING = 0x01
READ = 0x02
WRITE = 0x03
SYNC_READ = 0x82
SYNC_WRITE = 0x83


class Reg:
    """STS3215 control table addresses."""

    MODEL_NUMBER = 3          # 2 bytes
    ID = 5
    BAUD_RATE = 6
    RETURN_DELAY_TIME = 7
    MIN_POSITION_LIMIT = 9    # 2
    MAX_POSITION_LIMIT = 11   # 2
    MAX_TEMPERATURE_LIMIT = 13  # deg C
    MAX_INPUT_VOLTAGE = 14    # 0.1 V
    MIN_INPUT_VOLTAGE = 15    # 0.1 V
    P_COEFFICIENT = 21
    D_COEFFICIENT = 22
    I_COEFFICIENT = 23
    HOMING_OFFSET = 31        # 2, sign bit 11
    OPERATING_MODE = 33
    TORQUE_ENABLE = 40
    ACCELERATION = 41
    GOAL_POSITION = 42        # 2
    GOAL_VELOCITY = 46        # 2, sign bit 15
    LOCK = 55
    PRESENT_POSITION = 56     # 2
    PRESENT_VELOCITY = 58     # 2, sign bit 15
    PRESENT_LOAD = 60         # 2, sign bit 10
    PRESENT_VOLTAGE = 62      # 0.1 V
    PRESENT_TEMPERATURE = 63  # deg C
    MOVING = 66


# Error bits in a status packet.
ERR_VOLTAGE = 0x01
ERR_ANGLE = 0x02
ERR_OVERHEAT = 0x04
ERR_OVERCURRENT = 0x08
ERR_OVERLOAD = 0x20


# The SO-101 follower's servo IDs, as LeRobot sets them up.
SO101_IDS = {"shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3, "wrist_flex": 4, "wrist_roll": 5}


def checksum(body):
    return (~sum(body)) & 0xFF


def packet(servo_id, code, params=b""):
    body = bytes([servo_id, len(params) + 2, code]) + bytes(params)
    return b"\xff\xff" + body + bytes([checksum(body)])


def decode_signed(raw, sign_bit):
    mag = raw & ((1 << sign_bit) - 1)
    return -mag if raw & (1 << sign_bit) else mag


def encode_signed(value, sign_bit):
    mag = min(abs(int(value)), (1 << sign_bit) - 1)
    return mag | ((1 << sign_bit) if value < 0 else 0)


def le16(data, offset=0):
    return data[offset] | (data[offset + 1] << 8)


def to_le16(value):
    return bytes([value & 0xFF, (value >> 8) & 0xFF])


class Parser:
    """Splits a byte stream into (id, code, params, raw) packets, skipping noise."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data

    def packets(self):
        while True:
            i = self.buf.find(b"\xff\xff")
            if i < 0:
                del self.buf[:max(0, len(self.buf) - 1)]
                return
            del self.buf[:i]
            if len(self.buf) < 4:
                return
            if self.buf[2] == 0xFF or self.buf[3] < 2:
                del self.buf[0]
                continue
            total = 4 + self.buf[3]
            if len(self.buf) < total:
                return
            raw = bytes(self.buf[:total])
            del self.buf[:total]
            if raw[-1] != checksum(raw[2:-1]):
                self.buf[:0] = raw[1:]  # resync one byte later
                continue
            yield raw[2], raw[4], raw[5:-1], raw


class Bus:
    """Client side of the bus: talk to servos through a serial port (pyserial)."""

    def __init__(self, port, baud=1_000_000, timeout=0.02):
        import serial  # pyserial; imported here so the fake bus doesn't need it

        self.ser = serial.Serial(port, baud, timeout=0)
        self.timeout = timeout
        self.parser = Parser()
        self.last_sent = b""

    def close(self):
        self.ser.close()

    def _send(self, pkt):
        self.ser.reset_input_buffer()
        self.parser = Parser()
        self.ser.write(pkt)
        self.last_sent = pkt

    def _receive(self, done):
        deadline = time.monotonic() + self.timeout
        while True:
            for servo_id, code, params, raw in self.parser.packets():
                if raw == self.last_sent or servo_id == BROADCAST:
                    continue  # an adapter echoing what was sent
                if done(servo_id, code, params):
                    return
            if time.monotonic() > deadline:
                return
            data = self.ser.read(256)
            if data:
                self.parser.feed(data)
            else:
                time.sleep(0.0005)

    def ping(self, servo_id):
        self._send(packet(servo_id, PING))
        got = []
        self._receive(lambda i, c, p: got.append(i) or i == servo_id)
        return servo_id in got

    def read(self, servo_id, address, length):
        self._send(packet(servo_id, READ, bytes([address, length])))
        out = []

        def done(i, code, params):
            if i == servo_id and len(params) == length:
                out.append(bytes(params))
                return True
            return False

        self._receive(done)
        return out[0] if out else None

    def write(self, servo_id, address, data):
        self._send(packet(servo_id, WRITE, bytes([address]) + bytes(data)))
        got = []
        self._receive(lambda i, c, p: got.append(i) or i == servo_id)
        return servo_id in got

    def read_u8(self, servo_id, address):
        d = self.read(servo_id, address, 1)
        return None if d is None else d[0]

    def read_u16(self, servo_id, address):
        d = self.read(servo_id, address, 2)
        return None if d is None else le16(d)

    def write_u8(self, servo_id, address, value):
        return self.write(servo_id, address, bytes([value & 0xFF]))

    def write_u16(self, servo_id, address, value):
        return self.write(servo_id, address, to_le16(value))

    def write_eeprom(self, servo_id, address, data):
        """EEPROM registers only keep a write while the servo's lock is off."""
        ok = self.write_u8(servo_id, Reg.LOCK, 0)
        ok = self.write(servo_id, address, data) and ok
        return self.write_u8(servo_id, Reg.LOCK, 1) and ok

    def sync_read(self, address, length, ids):
        self._send(packet(BROADCAST, SYNC_READ, bytes([address, length] + list(ids))))
        out = {}

        def done(i, code, params):
            if i in ids and len(params) == length:
                out[i] = bytes(params)
            return len(out) == len(ids)

        self._receive(done)
        return out

    def sync_write(self, address, length, data):
        params = bytearray([address, length])
        for servo_id, value in data.items():
            assert len(value) == length
            params += bytes([servo_id]) + bytes(value)
        self._send(packet(BROADCAST, SYNC_WRITE, bytes(params)))

    def positions(self, ids):
        """Present position in ticks of every servo that answers."""
        return {i: le16(d) for i, d in self.sync_read(Reg.PRESENT_POSITION, 2, ids).items()}

    def set_torque(self, ids, on):
        self.sync_write(Reg.TORQUE_ENABLE, 1, {i: bytes([1 if on else 0]) for i in ids})


def ticks_to_rad(ticks, zero_ticks, sign=1):
    return sign * (ticks - zero_ticks) * 2.0 * math.pi / TICKS_PER_TURN
