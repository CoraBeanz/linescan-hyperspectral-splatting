"""The Python protocol, the fake bus, the bus scan and the calibration tool, without an arm."""

import json
import time

import pytest
import yaml

from so101_scan_hardware import calibrate, scan, sts
from so101_scan_hardware.fake_bus import FakeBus
from so101_scan_hardware.sts import Reg


@pytest.fixture
def fake(tmp_path):
    bus = FakeBus(link=str(tmp_path / "bus")).start()
    yield bus
    bus.stop()


@pytest.fixture
def client(fake):
    c = sts.Bus(fake.path, timeout=0.05)
    yield c
    c.close()


def test_packet_bytes():
    assert sts.packet(1, sts.PING) == bytes([0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB])
    assert sts.packet(1, sts.READ, bytes([56, 2])) == bytes([0xFF, 0xFF, 1, 4, 2, 56, 2, 0xBE])


def test_parser_resyncs():
    p = sts.Parser()
    good = sts.packet(4, 0, b"\x00\x08")
    bad = bytearray(good)
    bad[-1] ^= 0xFF
    p.feed(b"\x00\xff" + bytes(bad) + good)
    out = list(p.packets())
    assert [(i, bytes(params)) for i, _, params, _ in out] == [(4, b"\x00\x08")]


def test_signed():
    assert sts.decode_signed(sts.encode_signed(-700, 11), 11) == -700
    assert sts.encode_signed(-1, 15) == 0x8001


def test_ping_read_write(client):
    assert client.ping(1) and client.ping(5)
    assert not client.ping(7)
    assert client.read_u16(2, Reg.MODEL_NUMBER) == sts.STS3215_MODEL
    assert client.write_u8(2, Reg.ACCELERATION, 77)
    assert client.read_u8(2, Reg.ACCELERATION) == 77


def test_sync_read_and_torque_moves_servo(client, fake):
    assert client.positions([1, 2, 3, 4, 5]) == {i: 2048 for i in range(1, 6)}
    client.sync_write(Reg.GOAL_POSITION, 2, {3: sts.to_le16(2500)})
    time.sleep(0.2)
    assert client.positions([3])[3] == 2048  # torque off: the goal does nothing
    client.set_torque([3], True)
    time.sleep(0.4)
    assert client.positions([3])[3] == 2500


def test_scan_lists_servos(client):
    rows = scan.scan(client, range(1, 8))
    assert [r["id"] for r in rows] == [1, 2, 3, 4, 5]
    assert rows[0]["joint"] == "shoulder_pan" and rows[0]["model"] == sts.STS3215_MODEL


def scripted(fake, steps):
    """A prompt that, instead of waiting for a person, moves the fake servos by hand."""
    it = iter(steps)

    def prompt(_text=""):
        for servo_id, ticks in next(it):
            fake.servos[servo_id].set_position(ticks)
        time.sleep(0.15)
        return ""

    return prompt


def test_calibration_records_zero_range_and_sign(client, fake, tmp_path):
    zero = {1: 2010, 2: 2100, 3: 1990, 4: 2050, 5: 2048}

    # 1: the zero pose; 2: sweep every joint -900..+1000 ticks around it
    def sweep_prompt(_text=""):
        for i, z in zero.items():
            fake.servos[i].set_position(z - 900)
        time.sleep(0.2)
        for i, z in zero.items():
            fake.servos[i].set_position(z + 1000)
        time.sleep(0.2)
        for i, z in zero.items():
            fake.servos[i].set_position(z)
        time.sleep(0.15)
        return ""

    # 3: nudge each joint; elbow_flex (id 3) is wired the other way round
    nudges = [[(i, zero[i] + (-60 if i == 3 else 60))] for i in range(1, 6)]
    zero_prompt = scripted(fake, [list(zero.items())])
    nudge_prompt = scripted(fake, nudges)
    calls = []

    def prompt(text=""):
        calls.append(text)
        if len(calls) == 1:
            return zero_prompt()
        if len(calls) == 2:
            return sweep_prompt()
        return nudge_prompt()

    joints = calibrate.calibrate(client, calibrate.JOINTS, sts.SO101_IDS, prompt=prompt)
    assert joints["shoulder_pan"] == {"id": 1, "zero_ticks": 2010, "sign": 1, "min_ticks": 1110, "max_ticks": 3010}
    assert joints["elbow_flex"]["sign"] == -1
    assert joints["wrist_roll"]["zero_ticks"] == 2048

    path = calibrate.save(tmp_path / "cal.yaml", joints, "test")
    loaded = yaml.safe_load(path.read_text())["joints"]
    assert loaded["elbow_flex"] == joints["elbow_flex"]


def test_homing_offset_centres_zero(client, fake):
    fake.servos[2].set_position(300)  # zero pose near the wrap-around

    def prompt(_text=""):
        time.sleep(0.15)
        return ""

    joints = calibrate.calibrate(client, ["shoulder_lift"], sts.SO101_IDS, write_offset=True,
                                 sign_check=False, prompt=prompt)
    assert joints["shoulder_lift"]["zero_ticks"] == 2048
    assert client.positions([2])[2] == 2048


def test_asks_before_the_motors_go_off(client, fake):
    client.set_torque([1, 2, 3, 4, 5], True)   # the driver leaves the arm held when it stops
    seen = []

    def prompt(text=""):   # looks at the fake itself: the tool's own thread may be using the bus
        time.sleep(0.15)
        seen.append((text, fake.servos[2].regs[Reg.TORQUE_ENABLE]))
        return ""

    calibrate.calibrate(client, ["shoulder_lift"], sts.SO101_IDS, sign_check=False, prompt=prompt)
    assert "goes limp" in seen[0][0] and seen[0][1] == 1   # asked while the motor still held the arm
    assert seen[1][1] == 0 and client.read_u8(2, Reg.TORQUE_ENABLE) == 0


def test_stopped_run_puts_the_homing_offsets_back(fake, tmp_path, monkeypatch, capsys):
    fake.servos[2].set_position(300)
    answers = iter(["", KeyboardInterrupt])   # Ctrl-C at the range step, after the offset was written

    def ask(_text=""):
        time.sleep(0.15)
        answer = next(answers)
        if answer is KeyboardInterrupt:
            assert fake.servos[2].offset() != 0
            raise KeyboardInterrupt
        return answer

    monkeypatch.setattr(calibrate, "ask", ask)
    out = tmp_path / "cal.yaml"
    assert calibrate.main(["--port", fake.path, "--out", str(out), "--joints", "shoulder_lift",
                           "--write-homing-offset", "--no-sign-check"]) == 1
    assert fake.servos[2].offset() == 0 and not out.exists()
    assert "put back the homing offsets" in capsys.readouterr().out


def test_from_lerobot(tmp_path):
    data = {n: {"id": i, "drive_mode": 0, "homing_offset": 0, "range_min": 1000, "range_max": 3000}
            for n, i in sts.SO101_IDS.items()}
    data["wrist_roll"].update(range_min=0, range_max=4095)
    data["gripper"] = {"id": 6, "drive_mode": 0, "homing_offset": 0, "range_min": 2000, "range_max": 3300}
    f = tmp_path / "so101.json"
    f.write_text(json.dumps(data))
    joints = calibrate.from_lerobot(f)
    assert set(joints) == set(sts.SO101_IDS)
    assert joints["elbow_flex"] == {"id": 3, "zero_ticks": 2000, "sign": 1, "min_ticks": 1000, "max_ticks": 3000}
    assert joints["wrist_roll"]["zero_ticks"] == 2048
