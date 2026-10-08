#!/usr/bin/env python3
"""How well do the sweep poses come back, on the relief target and on a flat board?

    python3 pipeline/pose_floor.py                                  # ring plan, relief and board
    python3 pipeline/pose_floor.py --plan pipeline/plans/ring16.yaml

The splat trainer refines one head pose per sweep. On splat_synth's flat board, refining only
the poses against the true scene, from noise-free lines, stops near 0.3 px (splat/README.md,
Training): turning a sweep a little about the board while shifting it to keep the board in place
barely changes what a narrow fan sees of a flat surface. A target with relief should pin that
down. This runs the simulator's relief target and its flat board through the same chain
(first_light.py up to training) and three trainings each:

  joint    splat_train as the chain runs it: the scene and the poses together, from the logged
           poses (what the rig will do)
  scene    the scene alone, at the true poses (--no-poses on a copy of the dataset whose poses and
           mirror angles are the truth's): the best scene this data gives
  frozen   only the poses, from the logged ones, against that scene held fixed (--init-scene
           --freeze-scene): the floor the geometry and the data leave, with no scene error to
           blame

and prints the pose errors, in pixels and in millimetres on the table, into results.md.
"""

import argparse
import json
import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import first_light as fl  # noqa: E402


def true_pose_dataset(dataset, out):
    """A copy of the dataset whose recorded poses and mirror angles are the truth's."""
    if os.path.exists(out):
        shutil.rmtree(out)
    os.makedirs(out)
    doc = json.load(open(os.path.join(dataset, "dataset.json")))
    for key in ("lines", "line_sweep"):
        src, dst = os.path.join(dataset, doc["files"][key]), os.path.join(out, doc["files"][key])
        try:
            os.link(src, dst)
        except OSError:
            shutil.copyfile(src, dst)
    for key in ("sweep_head_pose", "line_mirror_angle"):
        shutil.copyfile(os.path.join(dataset, "gt", key + ".npy"), os.path.join(out, doc["files"][key]))
    shutil.copytree(os.path.join(dataset, "gt"), os.path.join(out, "gt"))
    doc["metadata"] = dict(doc.get("metadata") or {}, poses="the simulator's true poses and mirror angles")
    with open(os.path.join(out, "dataset.json"), "w") as f:
        json.dump(doc, f, indent=2)
        f.write("\n")


def train(p, dataset, out, extra, log):
    exe = fl.tool(p.build_dir, "splat_train")
    t = p.cfg["train"]
    if os.path.exists(out):
        shutil.rmtree(out)
    cmd = [exe, dataset, out, "--iterations", str(t["iterations"]), "--seed", str(t["seed"]),
           "--log-every", str(max(1, int(t["iterations"]) // 10))] + extra
    return fl.parse_train(fl.run(cmd, log))


def run_case(cfg, out, reuse):
    p = fl.Pipeline(cfg, out)
    done = os.path.join(out, "report.json")
    if not (reuse and os.path.exists(done) and "train" in json.load(open(done)).get("stages", {})):
        if not p.run("build", "train") and p.report.get("failed"):
            raise SystemExit("the chain failed at %s; see %s" % (p.report["failed"], os.path.join(out, "report.md")))
    joint = json.load(open(done))["stages"]["train"]
    log = os.path.join(out, "logs", "pose_floor.log")
    true_pose_dataset(p.dataset, os.path.join(out, "dataset_true"))
    scene = train(p, os.path.join(out, "dataset_true"), os.path.join(out, "train_true"), ["--no-poses"], log)
    frozen = train(p, p.dataset, os.path.join(out, "train_frozen"),
                   ["--init-scene", os.path.join(out, "train_true", "scene"), "--freeze-scene"], log)
    doc = json.load(open(os.path.join(p.dataset, "dataset.json")))
    mm_per_px = 150.0 / doc["intrinsics"]["f_px"]   # at the 150 mm working distance
    return dict(scene=cfg["sim"]["scene"], sweeps=doc["num_sweeps"], lines=doc["num_lines"], width=doc["width"],
                mm_per_px=mm_per_px, recorded_px=joint["pose_error_recorded_px"],
                joint_px=joint["pose_error_trained_px"], joint_rmse_truth=joint["rmse_truth"],
                scene_rmse_truth=scene["rmse_truth"], frozen_px=frozen["pose_error_trained_px"],
                frozen_along_px=frozen["pose_error_trained_along_px"],
                frozen_across_px=frozen["pose_error_trained_across_px"])


def table(rows):
    out = ["| Scene | Sweeps | Logged poses | Joint training | Scene at the true poses (RMSE vs truth) | "
           "Poses only, scene frozen |", "|---|---|---|---|---|---|"]
    for r in rows:
        mm = r["mm_per_px"]
        out.append("| %s | %d | %.2f px (%.2f mm) | %.2f px (%.2f mm) | %.4f | %.2f px (%.2f mm): along %.2f, "
                   "across %.2f |" % (r["scene"], r["sweeps"], r["recorded_px"], r["recorded_px"] * mm,
                                      r["joint_px"], r["joint_px"] * mm, r["scene_rmse_truth"], r["frozen_px"],
                                      r["frozen_px"] * mm, r["frozen_along_px"], r["frozen_across_px"]))
    return "\n".join(out) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--config", default=os.path.join(fl.REPO, "pipeline", "configs", "sim.yaml"))
    ap.add_argument("--plan", default="ring", help="a plan name or file for the simulator (default ring)")
    ap.add_argument("--scenes", default="relief,board")
    ap.add_argument("--out", default=os.path.join(fl.REPO, "build", "pose_floor"))
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="as first_light.py's")
    ap.add_argument("--reuse", action="store_true", help="keep a case's scan and joint training if they're there")
    a = ap.parse_args(argv)
    plan_name = os.path.splitext(os.path.basename(a.plan))[0]
    plan = os.path.abspath(a.plan) if os.path.exists(a.plan) else a.plan
    rows = []
    for scene in a.scenes.split(","):
        cfg = fl.load_config(a.config, a.set + ["sim.scene=" + scene, "sim.plan=" + plan,
                                                "name=%s-%s" % (plan_name, scene), "limits={}"])
        rows.append(run_case(cfg, os.path.join(a.out, "%s-%s" % (plan_name, scene)), a.reuse))
        print("\n" + table(rows), flush=True)
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, "results-%s.json" % plan_name), "w") as f:
        json.dump(rows, f, indent=1)
    with open(os.path.join(a.out, "results-%s.md" % plan_name), "w") as f:
        f.write(table(rows))
    print(table(rows))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
