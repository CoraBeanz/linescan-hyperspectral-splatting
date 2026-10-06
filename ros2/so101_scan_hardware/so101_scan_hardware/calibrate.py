"""Calibrate the SO-101 for ROS: where each joint's zero is, which way it turns, how far it goes.

    ros2 run so101_scan_hardware sts_calibrate --port /dev/so101

It switches the motors off (asking you to hold the arm first if they are on),
then asks you to:

  1. put the arm in its zero pose (the URDF's): base turned straight ahead,
     upper arm straight up, forearm level and pointing forward, wrist straight
     in line with the forearm, and the scan head's window facing right as seen
     from behind the arm;
  2. move every joint through its whole range, slowly;
  3. nudge each joint the way its angle increases, so the tool can check the
     direction against the URDF.

It writes a YAML file the launch files read (default
~/so101_scan/calibration.yaml, or $SO101_SCAN_DATA/calibration.yaml):

    joints:
      shoulder_pan: {id: 1, zero_ticks: 2047, sign: 1, min_ticks: 795, max_ticks: 3311}

joint angle [rad] = sign * (ticks - zero_ticks) * 2 pi / 4096.

The servos' homing offsets stay as they are unless you ask (--write-homing-offset),
so a LeRobot calibration on the same arm keeps working; if a run with it is
stopped or fails, the old offsets are put back. The only other thing it writes
to a servo is position mode, for one that isn't in it (the driver needs it).
--joints redoes some joints and keeps the rest of the file. --from-lerobot
converts a LeRobot calibration file instead of moving the arm.
"""

import argparse
import datetime
import json
import os
import sys
import threading
import time
from pathlib import Path

import yaml

from . import sts
from .sts import Reg

JOINTS = list(sts.SO101_IDS)

# How each joint looks when its angle increases, from the URDF, seen standing
# behind the arm with it in the zero pose.
POSITIVE = {
    "shoulder_pan": "swing the whole arm to the RIGHT",
    "shoulder_lift": "tip the upper arm FORWARD",
    "elbow_flex": "tip the forearm DOWN",
    "wrist_flex": "tip the head DOWN",
    "wrist_roll": "roll the head so its window turns from facing right toward facing UP",
}
ZERO_POSE = ("base turned straight ahead, upper arm straight up, forearm level and pointing forward,\n"
             "  wrist straight in line with the forearm, scan window facing right (seen from behind)")
MIN_NUDGE = 25  # ticks, about 2 degrees


def default_path():
    return Path(os.environ.get("SO101_SCAN_DATA", Path.home() / "so101_scan")) / "calibration.yaml"


def load(path):
    path = Path(path)
    if not path.exists():
        return {}
    return (yaml.safe_load(path.read_text()) or {}).get("joints", {})


def save(path, joints, source):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = ["# SO-101 scan arm calibration (%s, %s)." % (source, datetime.date.today().isoformat()),
             "# joint angle [rad] = sign * (ticks - zero_ticks) * 2 pi / 4096; zero is the URDF zero pose.",
             "joints:"]
    for name in JOINTS:
        if name in joints:
            j = joints[name]
            lines.append("  %s: {id: %d, zero_ticks: %d, sign: %d, min_ticks: %d, max_ticks: %d}" % (
                name, j["id"], j["zero_ticks"], j["sign"], j["min_ticks"], j["max_ticks"]))
    tmp = path.with_name(path.name + ".tmp")   # a failed write leaves the old file whole
    tmp.write_text("\n".join(lines) + "\n")
    os.replace(tmp, path)
    return path


def from_lerobot(json_path):
    """LeRobot's degrees are (ticks - middle of the recorded range) * 360 / 4095, and its
    SO-101 URDF (the one in cad/vendor) has its zero at that middle, so zero_ticks is
    the middle of each range."""
    data = json.loads(Path(json_path).read_text())
    out = {}
    for name in JOINTS:
        c = data[name]
        out[name] = {"id": int(c["id"]), "zero_ticks": int(round((c["range_min"] + c["range_max"]) / 2.0)),
                     "sign": -1 if c.get("drive_mode", 0) else 1,
                     "min_ticks": int(c["range_min"]), "max_ticks": int(c["range_max"])}
    return out


def ask(prompt):
    return input(prompt)


def wait_with_readout(bus, ids, show, prompt=None):
    """Poll the servos until Enter, printing a live line; returns the min/max ticks seen."""
    lo, hi = {}, {}
    done = threading.Event()

    def poll():
        while not done.is_set():
            pos = bus.positions(ids)
            for i, t in pos.items():
                lo[i] = min(lo.get(i, t), t)
                hi[i] = max(hi.get(i, t), t)
            if show:
                print("\r  " + "  ".join("%s %4d [%4d..%4d]" % (n[:8], pos.get(i, -1), lo.get(i, -1), hi.get(i, -1))
                                       for n, i in show), end="", flush=True)
            time.sleep(0.05)

    t = threading.Thread(target=poll, daemon=True)
    t.start()
    try:
        (prompt or ask)("")
    finally:   # Ctrl-C too: the bus is needed again to put things back
        done.set()
        t.join()
    print()
    return lo, hi


def torque_off(bus, names, ids, prompt):
    """Switch the motors off, first asking to hold the arm if any of them is holding it up."""
    holding = [n for n in names if bus.read_u8(ids[n], Reg.TORQUE_ENABLE) != 0]
    if holding:
        prompt("The motors are holding the arm (%s). They go off next and the arm goes limp:\n"
               "  hold it, then press Enter. " % ", ".join(holding))
    bus.set_torque([ids[n] for n in names], False)
    print("Torque is off; the arm is limp, so hold it.\n")


def put_back_offsets(bus, old):
    """Write back the homing offsets a stopped run changed; returns the ids it couldn't."""
    return [i for i, offset in old.items() if not bus.write_eeprom(i, Reg.HOMING_OFFSET, offset)]


def calibrate(bus, names, ids, write_offset=False, sign_check=True, prompt=None, old_offsets=None):
    """Interactive part. Returns {joint: {id, zero_ticks, sign, min_ticks, max_ticks}}. Each
    homing offset it changes goes into old_offsets first ({servo id: the old register bytes}),
    so the caller can put them back if the run doesn't finish."""
    prompt = prompt or ask
    old_offsets = {} if old_offsets is None else old_offsets
    joint_ids = [ids[n] for n in names]
    for n in names:
        mode = bus.read_u8(ids[n], Reg.OPERATING_MODE)
        if mode not in (0, None):
            print("  %s: switching servo %d to position mode" % (n, ids[n]))
            bus.write_eeprom(ids[n], Reg.OPERATING_MODE, bytes([0]))
    torque_off(bus, names, ids, prompt)

    prompt("1/3  Put the arm in its zero pose:\n  %s\n  then press Enter. " % ZERO_POSE)
    zero = bus.positions(joint_ids)
    missing = [n for n in names if ids[n] not in zero]
    if missing:
        raise RuntimeError("no answer from %s" % ", ".join(missing))
    if write_offset:
        for n in names:
            i = ids[n]
            old = bus.read_u16(i, Reg.HOMING_OFFSET)
            if old is None:
                raise RuntimeError("can't read %s's homing offset" % n)
            raw = zero[i] + sts.decode_signed(old, 11)
            offset = raw - 2048
            old_offsets.setdefault(i, sts.to_le16(old))
            bus.write_eeprom(i, Reg.HOMING_OFFSET, sts.to_le16(sts.encode_signed(offset, 11)))
        time.sleep(0.05)
        zero = bus.positions(joint_ids)
        print("  wrote homing offsets: the zero pose now reads %s" % ", ".join(str(zero[ids[n]]) for n in names))

    print("\n2/3  Move every joint slowly through its whole range, then press Enter.")
    lo, hi = wait_with_readout(bus, joint_ids, [(n, ids[n]) for n in names], prompt)

    signs = {n: 1 for n in names}
    if sign_check:
        print("\n3/3  Direction check. Put the arm back near the zero pose.")
        for n in names:
            i = ids[n]
            while True:
                before = bus.positions([i]).get(i)
                prompt("  %s: %s by a few degrees, then press Enter. " % (n, POSITIVE[n]))
                after = bus.positions([i]).get(i)
                if before is None or after is None:
                    raise RuntimeError("no answer from %s" % n)
                if abs(after - before) >= MIN_NUDGE:
                    signs[n] = 1 if after > before else -1
                    print("    %s" % ("matches the URDF" if signs[n] > 0 else "turns the other way: sign -1"))
                    break
                print("    it only moved %d ticks; move it a bit more" % (after - before))

    out = {}
    for n in names:
        i = ids[n]
        z, a, b = zero[i], lo.get(i, zero[i]), hi.get(i, zero[i])
        if b - a < 200:
            print("  warning: %s only moved %d ticks during the range step" % (n, b - a))
        if a < 30 or b > 4065:
            print("  warning: %s's range runs into the encoder's wrap-around (%d..%d); rerun with "
                  "--write-homing-offset to centre it" % (n, a, b))
        out[n] = {"id": i, "zero_ticks": int(z), "sign": signs[n], "min_ticks": int(a), "max_ticks": int(b)}
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="Calibrate the SO-101 joints for the ROS driver")
    ap.add_argument("--port", default="/dev/so101")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--out", default=str(default_path()))
    ap.add_argument("--joints", nargs="+", choices=JOINTS, help="only these joints; keep the rest of --out")
    ap.add_argument("--write-homing-offset", action="store_true",
                    help="also write each servo's homing offset so the zero pose reads 2048 "
                         "(changes what LeRobot's calibration expects)")
    ap.add_argument("--no-sign-check", action="store_true")
    ap.add_argument("--from-lerobot", metavar="JSON", help="convert a LeRobot calibration file instead")
    args, _ = ap.parse_known_args(argv)

    if args.from_lerobot:
        joints = from_lerobot(args.from_lerobot)
        path = save(args.out, joints, "converted from LeRobot's %s" % Path(args.from_lerobot).name)
        print("wrote %s\nLeRobot's middle pose was set with the gripper on: check the wrist roll with the "
              "passive check and redo it with --joints wrist_roll if the window isn't facing right at zero." % path)
        return 0

    names = args.joints or JOINTS
    keep = load(args.out) if args.joints else {}
    old_offsets = {}   # servo id -> its homing offset before this run, for each one changed
    bus = sts.Bus(args.port, args.baud)
    try:
        found = [n for n in names if bus.ping(sts.SO101_IDS[n])]
        if len(found) < len(names):
            print("no answer from %s on %s" % (", ".join(set(names) - set(found)), args.port))
            return 1
        joints = calibrate(bus, names, sts.SO101_IDS, args.write_homing_offset, not args.no_sign_check,
                           old_offsets=old_offsets)
        keep.update(joints)
        path = save(args.out, keep, "sts_calibrate")
    except BaseException as e:   # Ctrl-C, a servo that stopped answering, a file that can't be written
        print("\nstopped; %s is unchanged" % args.out)
        if old_offsets:
            try:
                failed = put_back_offsets(bus, old_offsets)
            except Exception:   # the bus itself is gone
                failed = list(old_offsets)
            if failed:
                print("couldn't put back the homing offsets of servos %s, so %s no longer fits them: "
                      "run sts_calibrate again" % (", ".join(map(str, failed)), args.out))
            else:
                print("put back the homing offsets this run had changed")
        if isinstance(e, KeyboardInterrupt):
            return 1
        raise
    finally:
        bus.close()
    print("\nwrote %s" % path)
    for n in JOINTS:
        if n in keep:
            j = keep[n]
            print("  %-14s zero %4d  sign %+d  range %4d..%4d" % (n, j["zero_ticks"], j["sign"],
                                                                 j["min_ticks"], j["max_ticks"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
