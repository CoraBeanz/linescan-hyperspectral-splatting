#!/usr/bin/env python3
"""Time training and count its memory, per dataset preset and training mode.

Makes a synthetic dataset per preset (once), trains on it with splat_train in
each mode, and prints a Markdown table of the time per step, what took it, the
GPU memory by buffer and the peak memory of the process: the numbers in
splat/docs/nano_budget.md. Needs Python 3.6 or newer and nothing else, so it
runs in the Jetson's container:

    jetson/splat.sh python3 /src/splat/tools/nano_budget.py            # on the Nano
    python splat/tools/nano_budget.py --bin build/splat/Release        # on a PC

Modes:
    gpu         the backward pass on the GPU, Adam and densification on the CPU
                (splat_train's default when there is a GPU)
    gpu-adam    everything on the GPU (--gpu-adam)
    basis8      everything on the GPU, 8 features through a learned basis
                (--gpu-adam --basis 8)
    cpu         everything on the CPU (--cpu), for comparison
"""
import argparse
import json
import os
import re
import subprocess
import sys
import time

PRESETS = {  # preset: sweeps to scan it with
    "tiny": 3,
    "small": 16,
    "default": 16,
    "full": 12,
}
MODES = {
    "gpu": [],
    "gpu-adam": ["--gpu-adam"],
    "basis8": ["--gpu-adam", "--basis", "8"],
    "cpu": ["--cpu"],
}


def tool(bin_dir, name):
    for candidate in (name, name + ".exe"):
        path = os.path.join(bin_dir, candidate)
        if os.path.exists(path):
            return path
    sys.exit("no {} in {} (pass --bin)".format(name, bin_dir))


def run(cmd):
    print("$ " + " ".join(cmd), flush=True)
    t0 = time.time()
    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
    sys.stdout.write(p.stdout)
    if p.returncode != 0:
        sys.exit("failed: " + " ".join(cmd))
    return p.stdout, time.time() - t0


def number(pattern, text, default=None):
    m = re.search(pattern, text, re.M)
    return float(m.group(1)) if m else default


def parse(out):
    r = {
        "ms": number(r"^time\s+([\d.]+) ms/step", out),
        "backward_ms": number(r"backward ([\d.]+),", out),
        "update_ms": number(r"Adam and densify ([\d.]+),", out),
        "train_s": number(r"; ([\d.]+) s for \d+ steps", out),
        "gaussians": number(r"^done\s+(\d+) Gaussians", out),
        "rmse": number(r"RMSE vs measured lines ([\d.]+)", out),
        "pose_px": number(r"pose err\s+([\d.]+) px rms \([^)]*\), max [\d.]+ at the refined", out),
        "host_mb": number(r"^host mem\s+([\d.]+) MB", out),
        "gpu_mb": number(r"^gpu mem\s+([\d.]+) MB", out),
        "device_used_mb": number(r"^device\s+([\d.]+) MB used of", out),
        "device_total_mb": number(r"MB used of ([\d.]+) MB", out),
    }
    m = re.search(r"^gpu mem\s+[\d.]+ MB in splat's buffers: (.*)$", out, re.M)
    if m:
        r["gpu_parts_mb"] = {k.strip(): float(v) for k, v in re.findall(r"([a-zA-Z ]+?) ([\d.]+)(?:,|$)", m.group(1))}
    m = re.search(r"^gpu time\s+per step, every step: (.*) in all ([\d.]+) ms", out, re.M)
    if m:
        r["passes_ms"] = {k.strip(): float(v) for k, v in re.findall(r"([a-z ]+?) ([\d.]+),", m.group(1))}
        r["gpu_ms"] = float(m.group(2))
    m = re.search(r"([\d.]+) visible pairs and ([\d.]+) sort keys a step", out)
    if m:
        r["visible_pairs"], r["sort_keys"] = float(m.group(1)), float(m.group(2))
    return r


def fmt(v, digits=1):
    return "–" if v is None else "{:,.{}f}".format(v, digits)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bin", default="/data/build", help="where splat_synth and splat_train are")
    ap.add_argument("--work", default="budget", help="where the datasets and runs go")
    ap.add_argument("--presets", default="small,default", help="from " + ",".join(PRESETS))
    ap.add_argument("--modes", default="gpu,gpu-adam,basis8", help="from " + ",".join(MODES))
    ap.add_argument("--iterations", type=int, default=3000)
    ap.add_argument("--label", default="", help="a name for this machine, for the table")
    a = ap.parse_args()

    synth, train = tool(a.bin, "splat_synth"), tool(a.bin, "splat_train")
    os.makedirs(a.work, exist_ok=True)
    results = []
    for preset in a.presets.split(","):
        sweeps = PRESETS[preset]
        data = os.path.join(a.work, "{}-{}".format(preset, sweeps))
        if not os.path.exists(os.path.join(data, "dataset.json")):
            run([synth, data, "--preset", preset, "--sweeps", str(sweeps)])
        info = json.load(open(os.path.join(data, "dataset.json")))
        for mode in a.modes.split(","):
            out, wall = run([train, data, os.path.join(data, "train-" + mode), "--iterations", str(a.iterations),
                             "--log-every", str(max(1, a.iterations // 10)), "--profile", "--no-preview"]
                            + MODES[mode])
            r = parse(out)
            r.update(preset=preset, sweeps=sweeps, mode=mode, wall_s=wall, iterations=a.iterations,
                     lines=number(r"dataset\s+\d+ sweeps, (\d+) lines", out),
                     width=info.get("intrinsics", {}).get("width"),
                     bands=len(info.get("wavelengths_nm", [])),
                     device=(re.search(r"^device\s+(.*), (?:Adam|the CPU)", out, re.M) or [None, None])[1])
            results.append(r)

    with open(os.path.join(a.work, "results.json"), "w") as f:
        json.dump(results, f, indent=1)
    label = a.label or (results[0]["device"] if results else "")
    print("\n## {} ({} steps)\n".format(label, a.iterations))
    print("| Preset | Mode | Gaussians | ms/step | backward | Adam, densify | GPU buffers MB | device used MB "
          "| host peak MB | pose px |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for r in results:
        print("| {} ({} sweeps) | {} | {} | {} | {} | {} | {} | {} | {} | {} |".format(
            r["preset"], r["sweeps"], r["mode"], fmt(r["gaussians"], 0), fmt(r["ms"], 2), fmt(r["backward_ms"], 2),
            fmt(r["update_ms"], 2), fmt(r["gpu_mb"]), fmt(r["device_used_mb"], 0), fmt(r["host_mb"], 0),
            fmt(r["pose_px"], 2)))
    passes = sorted({p for r in results for p in r.get("passes_ms", {})},
                    key=lambda p: min(list(r.get("passes_ms", {})).index(p) for r in results
                                      if p in r.get("passes_ms", {})))
    if passes:
        print("\nGPU ms per step, by pass (averaged over every step):\n")
        print("| Preset | Mode | " + " | ".join(passes) + " | all | visible pairs | sort keys |")
        print("|---|---|" + "---|" * (len(passes) + 3))
        for r in results:
            pm = r.get("passes_ms", {})
            print("| {} | {} | ".format(r["preset"], r["mode"]) + " | ".join(fmt(pm.get(p), 3) for p in passes)
                  + " | {} | {} | {} |".format(fmt(r.get("gpu_ms"), 2), fmt(r.get("visible_pairs"), 0),
                                              fmt(r.get("sort_keys"), 0)))
    parts = sorted({p for r in results for p in r.get("gpu_parts_mb", {})},
                   key=lambda p: min(list(r.get("gpu_parts_mb", {})).index(p) for r in results
                                     if p in r.get("gpu_parts_mb", {})))
    if parts:
        print("\nGPU buffers, MB:\n")
        print("| Preset | Mode | " + " | ".join(parts) + " |")
        print("|---|---|" + "---|" * len(parts))
        for r in results:
            gp = r.get("gpu_parts_mb", {})
            print("| {} | {} | ".format(r["preset"], r["mode"]) + " | ".join(fmt(gp.get(p)) for p in parts) + " |")
    print("\nWrote " + os.path.join(a.work, "results.json"))


if __name__ == "__main__":
    main()
