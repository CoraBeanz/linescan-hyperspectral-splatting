"""List the servos on the bus and what they report. Read-only: torque stays as it is.

    ros2 run so101_scan_hardware sts_scan --port /dev/so101
"""

import argparse
import sys

from . import sts
from .sts import Reg

NAMES = {i: n for n, i in sts.SO101_IDS.items()}


def scan(bus, ids):
    rows = []
    for i in ids:
        if not bus.ping(i):
            continue
        rd = bus.read_u8
        rows.append({
            "id": i,
            "joint": NAMES.get(i, "?"),
            "model": bus.read_u16(i, Reg.MODEL_NUMBER),
            "position": bus.read_u16(i, Reg.PRESENT_POSITION),
            "offset": sts.decode_signed(bus.read_u16(i, Reg.HOMING_OFFSET) or 0, 11),
            "mode": rd(i, Reg.OPERATING_MODE),
            "torque": rd(i, Reg.TORQUE_ENABLE),
            "volts": (rd(i, Reg.PRESENT_VOLTAGE) or 0) / 10.0,
            "temp": rd(i, Reg.PRESENT_TEMPERATURE),
            "delay": rd(i, Reg.RETURN_DELAY_TIME),
        })
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description="List the STS servos on the bus")
    ap.add_argument("--port", default="/dev/so101")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--ids", type=int, nargs="+", default=list(range(1, 11)))
    args, _ = ap.parse_known_args(argv)
    bus = sts.Bus(args.port, args.baud)
    try:
        rows = scan(bus, args.ids)
    finally:
        bus.close()
    if not rows:
        print("No servos answered on %s at %d baud. Is the servo power on?" % (args.port, args.baud))
        return 1
    print("id  joint          model  position  offset  mode  torque  volts  temp  return delay")
    for r in rows:
        print("%2d  %-13s %6s  %8s  %6s  %4s  %6s  %5.1f  %4s  %s" % (
            r["id"], r["joint"], r["model"], r["position"], r["offset"], r["mode"], r["torque"], r["volts"],
            r["temp"], r["delay"]))
    want = set(sts.SO101_IDS.values())
    have = {r["id"] for r in rows}
    if want - have:
        print("missing: %s" % ", ".join("%s (id %d)" % (NAMES[i], i) for i in sorted(want - have)))
    if any(r["model"] != sts.STS3215_MODEL for r in rows):
        print("note: model %d is the STS3215" % sts.STS3215_MODEL)
    return 0 if want <= have else 1


if __name__ == "__main__":
    sys.exit(main())
