#!/usr/bin/env python3
"""Virtual first light: the whole chain from a scan to the web viewer, in one command.

    python3 pipeline/first_light.py                                   # the simulator's ring scan (nightly)
    python3 pipeline/first_light.py pipeline/configs/sim-quick.yaml   # a small one, for pull requests
    python3 pipeline/first_light.py pipeline/configs/rig.yaml --set scan.path=~/so101_scan/scans/<scan>

Stages, each writing into the output folder (--out, default build/first_light/<config name>):

  build      CMake-builds splat_train and splat_export into build/splat if they aren't there
  scan       sim: renders a scan session with hsisim (sim/); rig: uses the scan folder as is
  calibrate  hsical calibrate on the session's calibration (unless one is given or not needed)
  convert    splat/tools/scan_to_dataset.py: the session as the trainer's dataset
  truth      sim only: the simulator's truth as the dataset's gt/, and the conversion checked
             against it (pipeline/sim_truth.py)
  train      splat_train: the splat and the refined sweep poses
  export     splat_export: one .lsplat for the web viewer, and the C++ renderer's reference spectra
  viewer     pipeline/viewer_check.mjs: the viewer's own reader and probe on that file, against
             the reference (needs Node 18+; skipped without it)

Every number a stage measures goes into report.json and report.md, with a limit from the
config where it has one; the command exits 0 only if every check passes. --from STAGE reruns
from that stage, reusing what the earlier ones left in the output folder.
"""

import argparse
import copy
import json
import os
import re
import shutil
import subprocess
import sys
import time

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
STAGES = ["build", "scan", "calibrate", "convert", "truth", "train", "export", "viewer"]

DEFAULTS = {
    "name": "first-light",
    "source": "sim",                # sim: render a scan with hsisim; scan: a scan folder from the rig
    "sim": {                        # python -m hsisim scan's options
        "scene": "relief", "plan": "ring", "views": None, "lines": None, "steps_per_line": None,
        "lighting": "uniform", "binning": 4, "seed": 7, "errors": 1.0,
    },
    "scan": {                       # source: scan
        "path": None,               # the scan folder (scan_sweep + line_camera)
        "calibration": None,        # an hsical calibration (with maps.npz), if the lines need binning
        "calibration_session": None,  # or an hsical session to calibrate first
    },
    "convert": {"slit_bins": 256, "nm_min": 500.0, "nm_max": 950.0, "nm_step": 10.0, "args": []},
    "train": {"iterations": 3000, "batch": 128, "spacing_mm": 1.0, "seed": 7, "cpu": False, "args": []},
    "export": {"title": None},
    "build_dir": "build/splat",
    # check: limit. "_max" limits are upper bounds, "_min" lower bounds; null skips the check.
    "limits": {},
}


# --- the config ------------------------------------------------------------------------------------

def merge(base, over):
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        out[k] = merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


def parse_value(text):
    try:
        return json.loads(text)
    except ValueError:
        return text


def apply_set(cfg, item):
    """--set a.b=value: value is read as JSON where it parses (numbers, true, null, lists), else
    as a string."""
    if "=" not in item:
        raise SystemExit("--set wants key=value, got %r" % item)
    key, value = item.split("=", 1)
    d = cfg
    parts = key.strip().split(".")
    for p in parts[:-1]:
        d = d.setdefault(p, {})
        if not isinstance(d, dict):
            raise SystemExit("--set %s: %s isn't a section" % (key, p))
    d[parts[-1]] = parse_value(value)


def load_config(path, sets=()):
    import yaml
    cfg = copy.deepcopy(DEFAULTS)
    if path:
        with open(path) as f:
            cfg = merge(cfg, yaml.safe_load(f) or {})
    for s in sets:
        apply_set(cfg, s)
    if cfg["source"] not in ("sim", "scan"):
        raise SystemExit("source must be sim or scan, not %r" % cfg["source"])
    return cfg


# --- running things --------------------------------------------------------------------------------

class StageFailed(RuntimeError):
    pass


def run(cmd, log_path, cwd=None):
    """Runs cmd, printing its output indented and keeping it in log_path. Returns the output."""
    print("  $ " + " ".join(cmd), flush=True)
    out = []
    with open(log_path, "a") as log:
        log.write("$ %s\n" % " ".join(cmd))
        p = subprocess.Popen(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        for line in p.stdout:
            out.append(line)
            log.write(line)
            print("    " + line, end="", flush=True)
        p.wait()
    text = "".join(out)
    if p.returncode != 0:
        raise StageFailed("%s exited with %d" % (" ".join(os.path.basename(c) for c in cmd[:3]), p.returncode))
    return text


def tool(build_dir, name):
    for p in (os.path.join(build_dir, name), os.path.join(build_dir, "Release", name + ".exe"),
              os.path.join(build_dir, name + ".exe")):
        if os.path.exists(p):
            return p
    return None


def number(pattern, text, group=1):
    m = re.search(pattern, text)
    return float(m.group(group)) if m else None


def parse_train(text):
    """The numbers splat_train prints."""
    pose = r"pose err\s+([\d.]+) px rms \(along the slit ([\d.]+), across ([\d.]+)\), max ([\d.]+) at the %s poses"
    device = re.search(r"device\s+(.+)", text)
    return {
        "device": device.group(1).strip() if device else None,
        "gaussians": number(r"done\s+(\d+) Gaussians", text),
        "train_seconds": number(r"Gaussians in ([\d.]+) s", text),
        "rmse_measured": number(r"RMSE vs measured lines ([\d.]+)", text),
        "rmse_truth": number(r"vs noise-free lines ([\d.]+)", text),
        "pose_error_recorded_px": number(pose % "recorded", text),
        "pose_error_trained_px": number(pose % "refined", text),
        "pose_error_trained_along_px": number(pose % "refined", text, 2),
        "pose_error_trained_across_px": number(pose % "refined", text, 3),
    }


class Pipeline:
    def __init__(self, cfg, out):
        self.cfg = cfg
        self.out = os.path.abspath(os.path.expanduser(out))
        self.logs = os.path.join(self.out, "logs")
        self.report = {"config": cfg, "stages": {}, "checks": []}

    # paths every stage agrees on
    @property
    def session(self):
        if self.cfg["source"] == "sim":
            return os.path.join(self.out, "scan")
        path = self.cfg["scan"]["path"]
        if not path:
            raise StageFailed("source is scan but scan.path isn't set (--set scan.path=<scan folder>)")
        return os.path.abspath(os.path.expanduser(path))

    @property
    def dataset(self):
        return os.path.join(self.out, "dataset")

    @property
    def build_dir(self):
        return os.path.join(REPO, self.cfg["build_dir"])

    def check(self, stage, name, value, limit_key=None, text=None):
        """Records a number, and checks it against limits[limit_key] if the config has one."""
        limit = self.cfg["limits"].get(limit_key) if limit_key else None
        entry = {"stage": stage, "name": name, "value": value, "text": text}
        if limit is not None:
            upper = limit_key.endswith("_max")
            ok = value is not None and value == value and (value <= limit if upper else value >= limit)
            entry.update(limit=limit, bound="max" if upper else "min", ok=bool(ok))
        self.report["checks"].append(entry)

    def check_true(self, stage, name, ok, text=None):
        self.report["checks"].append({"stage": stage, "name": name, "value": bool(ok), "ok": bool(ok), "text": text})

    # --- stages ---------------------------------------------------------------------------------------

    def stage_build(self, log):
        need = [n for n in ("splat_train", "splat_export") if not tool(self.build_dir, n)]
        if not need:
            return {"built": False}
        run(["cmake", "-S", os.path.join(REPO, "splat"), "-B", self.build_dir, "-DCMAKE_BUILD_TYPE=Release"], log)
        run(["cmake", "--build", self.build_dir, "--config", "Release", "-j", str(os.cpu_count() or 2),
             "--target", "splat_train", "splat_export"], log)
        return {"built": True}

    def stage_scan(self, log):
        if self.cfg["source"] == "scan":
            for need in ("lines.csv", "scan.json", "frames/camera.json"):
                if not os.path.exists(os.path.join(self.session, need)):
                    raise StageFailed("%s has no %s" % (self.session, need))
            return {"session": self.session}
        s = self.cfg["sim"]
        if os.path.exists(self.session):
            shutil.rmtree(self.session)
        cmd = [sys.executable, "-m", "hsisim", "scan", self.session, "--plan", str(s["plan"]),
               "--scene", s["scene"], "--lighting", s["lighting"], "--binning", str(s["binning"]),
               "--seed", str(s["seed"]), "--errors", str(s["errors"])]
        for key, flag in (("views", "--views"), ("lines", "--lines"), ("steps_per_line", "--steps-per-line")):
            if s.get(key) is not None:
                v = s[key]
                cmd += [flag, ",".join(v) if isinstance(v, list) else str(v)]
        text = run(cmd, log, cwd=os.path.join(REPO, "sim"))
        return {"session": self.session, "lines": number(r"Wrote (\d+) lines", text),
                "sweeps": number(r"lines in (\d+) sweeps", text)}

    def calibration_source(self):
        """(calibration folder to use, session to calibrate into it first, or None)."""
        if self.cfg["source"] == "sim":
            return os.path.join(self.out, "cal"), os.path.join(self.session, "calibration")
        sc = self.cfg["scan"]
        if sc.get("calibration"):
            return os.path.abspath(os.path.expanduser(sc["calibration"])), None
        if sc.get("calibration_session"):
            return os.path.join(self.out, "cal"), os.path.abspath(os.path.expanduser(sc["calibration_session"]))
        return None, None

    def stage_calibrate(self, log):
        cal, session = self.calibration_source()
        if session is None:
            return {"calibration": cal, "ran": False}
        if os.path.exists(cal):
            shutil.rmtree(cal)
        try:
            run([sys.executable, "-m", "hsical", "calibrate", session, "-o", cal], log,
                cwd=os.path.join(REPO, "calibration"))
            ok = True
        except StageFailed:
            # hsical exits 2 when it calibrated but some of its own checks failed
            ok = False
            if not os.path.exists(os.path.join(cal, "maps.npz")):
                raise
        self.check_true("calibrate", "hsical's calibration checks pass", ok)
        return {"calibration": cal, "ran": True, "checks_passed": ok}

    def stage_convert(self, log):
        c = self.cfg["convert"]
        if os.path.exists(self.dataset):
            shutil.rmtree(self.dataset)
        cmd = [sys.executable, os.path.join(REPO, "splat", "tools", "scan_to_dataset.py"), self.session, self.dataset,
               "--slit-bins", str(c["slit_bins"]), "--nm-min", str(c["nm_min"]), "--nm-max", str(c["nm_max"]),
               "--nm-step", str(c["nm_step"])]
        cal, _ = self.calibration_source()
        if cal:
            cmd += ["--calibration", cal]
        run(cmd + [str(a) for a in c.get("args") or []], log)
        doc = json.load(open(os.path.join(self.dataset, "dataset.json")))
        meta = doc.get("metadata") or {}
        written = int(meta.get("lines_written", doc["num_lines"]))
        left = sum(int(v) for k, v in (meta.get("lines_left_out") or {}).items() if k != "not_asked")
        res = {"lines": doc["num_lines"], "sweeps": doc["num_sweeps"], "width": doc["width"],
               "bands": doc["num_bands"], "values": doc["values"],
               "lines_left_out_fraction": left / max(1, written + left),
               "values_filled_fraction": float(meta.get("values_filled_fraction", 0.0))}
        self.check("convert", "lines left out (fraction)", res["lines_left_out_fraction"], "lines_left_out_max")
        self.check("convert", "values filled in (fraction)", res["values_filled_fraction"], "values_filled_max")
        return res

    def stage_truth(self, log):
        if self.cfg["source"] != "sim":
            return {"skipped": "no truth for a real scan"}
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from sim_truth import write_truth
        lines = []
        res = write_truth(self.session, self.dataset, log=lambda m: lines.append(m) or print("    " + m, flush=True))
        with open(log, "a") as f:
            f.write("\n".join(lines) + "\n")
        self.check("truth", "reflectance vs truth, |median ratio - 1|", res["reflectance_ratio_median"],
                   "reflectance_ratio_median_max")
        self.check("truth", "reflectance vs truth, rms of ratio - 1", res["reflectance_ratio_rms"],
                   "reflectance_ratio_rms_max")
        self.check("truth", "slit profile correlation with the truth", res["profile_correlation"],
                   "profile_correlation_min")
        self.check("truth", "the same, with the truth mirrored", res["profile_correlation_mirrored"],
                   "profile_correlation_mirrored_max")
        self.check("truth", "slit profile shift (px)", res["profile_shift_px"], "profile_shift_px_max")
        self.check("truth", "logged head poses off the truth (mm, worst sweep)", res["logged_pose_error_mm"])
        return res

    def stage_train(self, log):
        t = self.cfg["train"]
        exe = tool(self.build_dir, "splat_train")
        if not exe:
            raise StageFailed("no splat_train in %s" % self.build_dir)
        train = os.path.join(self.out, "train")
        if os.path.exists(train):
            shutil.rmtree(train)
        cmd = [exe, self.dataset, train, "--iterations", str(t["iterations"]), "--batch", str(t["batch"]),
               "--spacing", str(t["spacing_mm"]), "--seed", str(t["seed"]),
               "--log-every", str(max(1, int(t["iterations"]) // 10))]
        if t.get("cpu"):
            cmd.append("--cpu")
        text = run(cmd + [str(a) for a in t.get("args") or []], log)
        res = parse_train(text)
        self.check("train", "RMSE vs measured lines", res["rmse_measured"], "rmse_measured_max")
        if res["rmse_truth"] is not None:
            self.check("train", "RMSE vs the truth's noise-free lines", res["rmse_truth"], "rmse_truth_max")
        if res["pose_error_trained_px"] is not None:
            self.check("train", "pose error at the logged poses (px rms)", res["pose_error_recorded_px"])
            self.check("train", "pose error after training (px rms)", res["pose_error_trained_px"],
                       "pose_error_px_max")
            ratio = res["pose_error_trained_px"] / max(res["pose_error_recorded_px"], 1e-9)
            res["pose_error_ratio"] = ratio
            self.check("train", "pose error after / before", ratio, "pose_error_ratio_max")
        return res

    def stage_export(self, log):
        exe = tool(self.build_dir, "splat_export")
        if not exe:
            raise StageFailed("no splat_export in %s" % self.build_dir)
        title = self.cfg["export"].get("title") or self.cfg["name"]
        splat, ref = os.path.join(self.out, "splat.lsplat"), os.path.join(self.out, "reference.json")
        text = run([exe, os.path.join(self.out, "train", "scene"), splat, "--dataset", self.dataset,
                    "--poses", os.path.join(self.out, "train", "sweep_head_pose.npy"), "--title", title,
                    "--reference", ref], log)
        res = {"file": splat, "bytes": os.path.getsize(splat),
               "decoded_rms": number(r"differs by ([\d.eE+-]+) rms", text)}
        self.check("export", "the file as the viewer decodes it vs the scene (rms)", res["decoded_rms"],
                   "export_decoded_rms_max")
        return res

    def stage_viewer(self, log):
        node = shutil.which("node")
        if not node:
            return {"skipped": "no node"}
        doc = json.load(open(os.path.join(self.dataset, "dataset.json")))
        out = os.path.join(self.out, "viewer_check.json")
        try:
            run([node, os.path.join(REPO, "pipeline", "viewer_check.mjs"), os.path.join(self.out, "splat.lsplat"),
                 os.path.join(self.out, "reference.json"), "--bands", str(doc["num_bands"]),
                 "--sweeps", str(doc["num_sweeps"]), "--json", out], log)
        except StageFailed:
            if not os.path.exists(out):
                raise
        res = json.load(open(out))
        self.check_true("viewer", "the viewer reads the file and its probe matches the C++ renderer", res["ok"],
                        "; ".join(res["problems"]) or None)
        self.check("viewer", "probe vs C++ renderer, worst band", res["max_spectrum_difference"])
        self.check("viewer", "the default view's middle that the splat covers (fraction)", res["covered_fraction"],
                   "viewer_covered_min")
        return res

    # --- the run ----------------------------------------------------------------------------------------

    def save(self):
        checks = [c for c in self.report["checks"] if "ok" in c]
        self.report["ok"] = all(c["ok"] for c in checks) and not self.report.get("failed")
        with open(os.path.join(self.out, "report.json"), "w") as f:
            json.dump(self.report, f, indent=1, default=str)
            f.write("\n")
        with open(os.path.join(self.out, "report.md"), "w") as f:
            f.write(markdown(self.report))

    def run(self, first="build", last="viewer"):
        os.makedirs(self.logs, exist_ok=True)
        old = os.path.join(self.out, "report.json")
        if first != STAGES[0] and os.path.exists(old):
            prev = json.load(open(old))
            keep = STAGES[:STAGES.index(first)]
            self.report["stages"] = {k: v for k, v in prev.get("stages", {}).items() if k in keep}
            self.report["checks"] = [c for c in prev.get("checks", []) if c["stage"] in keep]
        for name in STAGES[STAGES.index(first):STAGES.index(last) + 1]:
            print("\n== %s" % name, flush=True)
            log = os.path.join(self.logs, name + ".log")
            if os.path.exists(log):
                os.remove(log)
            t0 = time.time()
            try:
                res = getattr(self, "stage_" + name)(log)
            except (StageFailed, OSError, ValueError, KeyError) as e:
                self.report["stages"][name] = {"error": str(e), "seconds": round(time.time() - t0, 1)}
                self.report["failed"] = name
                self.check_true(name, "the %s stage runs" % name, False, str(e))
                print("  ! %s failed: %s" % (name, e), flush=True)
                self.save()
                return False
            res["seconds"] = round(time.time() - t0, 1)
            self.report["stages"][name] = res
            self.save()
        return self.report["ok"]


def fmt(v):
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, float):
        if v != v:
            return "n/a"
        return "%.3g" % v if abs(v) < 1e-3 or abs(v) >= 1e4 else ("%.4f" % v).rstrip("0").rstrip(".")
    return "" if v is None else str(v)


def markdown(report):
    cfg = report["config"]
    src = ("simulated %s scene, %s plan" % (cfg["sim"]["scene"], cfg["sim"]["plan"]) if cfg["source"] == "sim"
           else "scan %s" % cfg["scan"]["path"])
    lines = ["# Virtual first light: %s" % cfg["name"], "",
             "**%s** (%s)" % ("Passed" if report.get("ok") else "FAILED", src), "",
             "| Stage | Check | Value | Limit | |", "|---|---|---|---|---|"]
    for c in report["checks"]:
        limit = "" if "limit" not in c else ("≤ " if c["bound"] == "max" else "≥ ") + fmt(c["limit"])
        mark = "" if "ok" not in c else ("ok" if c["ok"] else "**FAIL**")
        name = c["name"] + (" (%s)" % c["text"] if c.get("text") else "")
        lines.append("| %s | %s | %s | %s | %s |" % (c["stage"], name, fmt(c["value"]), limit, mark))
    lines += ["", "| Stage | Seconds |", "|---|---|"]
    lines += ["| %s | %s |" % (k, fmt(v.get("seconds"))) for k, v in report["stages"].items()]
    return "\n".join(lines) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0],
                                 epilog="Stages: " + ", ".join(STAGES) + ". See pipeline/README.md.")
    ap.add_argument("config", nargs="?", default=os.path.join(REPO, "pipeline", "configs", "sim.yaml"),
                    help="a YAML config (default: pipeline/configs/sim.yaml, the nightly run)")
    ap.add_argument("--out", help="output folder (default: build/first_light/<config name>, which git ignores)")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help="override a config value, e.g. --set sim.scene=board --set train.iterations=500")
    ap.add_argument("--from", dest="first", choices=STAGES, default=STAGES[0],
                    help="start at this stage, reusing the earlier stages' output")
    ap.add_argument("--to", dest="last", choices=STAGES, default=STAGES[-1], help="stop after this stage")
    a = ap.parse_args(argv)
    cfg = load_config(a.config, a.set)
    out = a.out or os.path.join(REPO, "build", "first_light", cfg["name"])
    p = Pipeline(cfg, out)
    ok = p.run(a.first, a.last)
    print("\n" + markdown(p.report))
    print("Report: %s" % os.path.join(p.out, "report.md"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
