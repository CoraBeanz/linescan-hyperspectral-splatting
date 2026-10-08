# pipeline: virtual first light

Every stage of the rig's software is built and tested on its own. The simulator renders raw
frames, the calibration kit calibrates the spectrograph, the capture side's converter turns a scan
into a dataset, the trainer fits a splat, the exporter packs it and the web viewer opens it. This
folder runs them in a row, the way a real scan will go through them. Each hand-off is checked
against the simulator's truth. Once the parts arrive, first light is the same command with a
different config.

```bash
python3 pipeline/first_light.py                                   # the simulator's ring scan: about 10 minutes on 4 cores
python3 pipeline/first_light.py pipeline/configs/sim-quick.yaml   # a smaller one: about 2 minutes
python3 pipeline/first_light.py pipeline/configs/rig.yaml --set scan.path=~/so101_scan/scans/<scan>
```

It needs Python 3.10 or newer with the simulator's packages (`pip install -r sim/requirements.txt`:
numpy, scipy, PyYAML and xacro), CMake with a C++17 compiler, and Node 18 or newer for the viewer
check, which is skipped without Node. If `build/splat` doesn't have `splat_train` and
`splat_export` yet, the first stage builds them, with CUDA when CMake finds it.

## Contents

- [The chain](#the-chain)
- [The checks](#the-checks)
- [First light with the real parts](#first-light-with-the-real-parts)
- [Relief or flat board: how well the poses come back](#relief-or-flat-board-how-well-the-poses-come-back)
- [Options](#options)
- [Files](#files)

## The chain

| Stage | What runs | What it writes in `build/first_light/<name>/` |
|---|---|---|
| build | CMake, only if the tools are missing | `build/splat/splat_train`, `splat_export` |
| scan | `python -m hsisim scan` from [`sim/`](../sim/README.md) | `scan/`: a scan session in the rig's format, plus `truth/` |
| calibrate | `python -m hsical calibrate` from [`calibration/`](../calibration/README.md) on the session's calibration captures | `cal/` |
| convert | [`splat/tools/scan_to_dataset.py`](../splat/tools/scan_to_dataset.py), the capture side's converter | `dataset/`: the trainer's format |
| truth | [`sim_truth.py`](sim_truth.py) (simulated scans only) | `dataset/gt/`: true poses, mirror angles and noise-free lines |
| train | `splat_train` | `train/`: the splat, the refined poses, `log.csv`, previews |
| export | `splat_export --reference` | `splat.lsplat` for the [viewer](../viewer/README.md), and the C++ renderer's spectra at a grid of pixels |
| viewer | [`viewer_check.mjs`](viewer_check.mjs) | `viewer_check.json` |

Then `report.md` and `report.json` list every number the stages measured, with the config's limit
where it sets one. The command exits with 0 only if every check passes, so CI can run it as is.
`logs/` keeps each stage's output.

`truth` writes the same `gt/` folder that `splat_synth` writes for its own synthetic datasets, so
`splat_train` and `splat_render residual` score a simulated scan without any changes. Lines are
matched to the simulator's by sweep and mirror angle, so it still works if the converter leaves
some out.

The viewer check loads the exported file with the viewer's own `format.js` and `probe.js` in Node,
with no browser. It compares their spectra with the C++ renderer's at the reference pixels
`splat_export --reference` wrote. The viewer's own tests ([`viewer/test`](../viewer/test/run.mjs))
already check the WebGL2 image against the same kind of reference in headless Chromium, so this
check doesn't do that again.

## The checks

Results from the nightly config, [`configs/sim.yaml`](configs/sim.yaml): the relief target on the
ring plan, four viewpoints of 107 lines each, seed 7, 256 pixels by 46 bands, 3000 steps on a
4-core CPU.

| Check | What a failure would mean | Limit | Result |
|---|---|---|---|
| hsical's own calibration checks | the simulated instrument no longer calibrates | pass | pass |
| Lines left out, values filled in | frames lost between the session and the dataset | 0, ≤ 0.1% | 0, 0 |
| Reflectance against the truth: \|median ratio − 1\|, rms | the white reference, darks or band sums are off | ≤ 1%, ≤ 3% | 0.00%, 1.0% |
| Slit profile correlation with the truth, and with the truth mirrored | pixel 0 is at the wrong end of the slit | ≥ 0.99, ≤ 0.5 | 0.999, 0.02 |
| Slit profile shift | the slit bins are off by part of a pixel | ≤ 0.25 px | 0.06 px |
| RMSE against the measured lines, and against the truth's noise-free lines | the splat doesn't fit the lines | ≤ 0.025, ≤ 0.03 | 0.014, 0.019 |
| Pose error after training, and after over before | pose refinement stopped working | ≤ 2.5 px, ≤ 0.7 | 1.69 px, 0.47 |
| The exported file, decoded as the viewer decodes it, against the scene | the export's quantisation went wrong | ≤ 0.005 rms | 0.0002 |
| The viewer's probe against the C++ renderer | the viewer and the trainer disagree on the format or the model | ≤ 5 × 10⁻⁴ | 6 × 10⁻⁵ |
| How much of the default view's middle the splat covers | the viewer opens looking at nothing | ≥ 50% | 81% |

The logged head poses start 2.0 mm and 1.0° from the truth (the simulator's default arm and
mirror errors), which is 3.60 px rms on the line cameras. Training brings that down to 1.69 px,
about 0.28 mm on the table. The quick config ([`configs/sim-quick.yaml`](configs/sim-quick.yaml),
two viewpoints, 128 pixels by 23 bands, 600 steps) runs every stage in about 100 seconds. It keeps
the same limits on the conversion and only asks that training doesn't make the poses worse.

## First light with the real parts

[`configs/rig.yaml`](configs/rig.yaml) runs the same stages on a scan folder from the rig: what
`scan_sweep` and `line_camera` write, as [ros2/README.md](../ros2/README.md#scanning-with-the-spectrograph-camera)
describes.

```bash
python3 pipeline/first_light.py pipeline/configs/rig.yaml --set scan.path=~/so101_scan/scans/ring_20261010-140000
```

If `line_camera` ran with a calibration, its lines are already binned and that's all it needs.
Otherwise `--set scan.calibration=<hsical calibration>` bins the raw frames, or
`--set scan.calibration_session=<hsical session>` calibrates one first. There is no truth for a
real scan, so the checks are the ones a real scan allows: how many lines made it into the
dataset, how many values had to be filled in, how well the splat fits the measured lines, and
whether the viewer reads the file. The two first-light questions in ros2/README.md, which end of
the slit is pixel 0 and whether frames are stamped right, show up in `train/preview/`, where the
measured lines sit beside the trained ones for each sweep. Then open `splat.lsplat` in the viewer.

The simulator's slit-profile check is how this chain proves the converter puts pixel 0 at the head's
−X end. On the rig, scan something asymmetric first.

## Relief or flat board: how well the poses come back

The trainer refines one head pose per sweep. On `splat_synth`'s flat board, refining only the poses
against the true scene, from noise-free lines, stops at about 0.3 px at 128 px (see
[splat/README.md](../splat/README.md#training)). Turning a sweep slightly about the board while
shifting it to keep the board in place barely changes what a narrow fan of lines sees of a flat
surface, so the loss is nearly flat in that direction. The splat README suggested a target with
textured relief to pin that down, and the simulator's default scene is one: a 60 mm plate with
5 × 5 pillars 1 to 12 mm tall, their tops painted, patterned and dotted. Its `board` scene is a
flat target like the splat's.

[`pose_floor.py`](pose_floor.py) runs both scenes through the chain up to training, then trains
each twice more:

- **Joint:** the scene and the poses together, from the logged poses. This is what the chain does
  and what the rig will do.
- **Scene at the true poses:** the scene alone, on a copy of the dataset with the truth's poses
  and mirror angles (`--no-poses`). This is the best scene the lines can give.
- **Poses only:** from the logged poses, against that scene held fixed (`splat_train --init-scene
  --freeze-scene`). This is the floor the geometry leaves, with no error in the scene to blame.

POSE_FLOOR_RESULTS

```bash
python3 pipeline/pose_floor.py                                      # the ring plan
python3 pipeline/pose_floor.py --plan pipeline/plans/ring16.yaml    # sixteen viewpoints
```

[`plans/ring16.yaml`](plans/ring16.yaml) is a plan the arm can reach. It looks at the same spot as
`ring.yaml`: straight down, eight views 15° off vertical every 45° of azimuth, and seven views 25°
off. The arm can't lean 25° away from its base. `make_plan` wrote it, and the rig can run it as it
is.

## Options

```bash
python3 pipeline/first_light.py --help
```

- `CONFIG`, a YAML file: [`sim.yaml`](configs/sim.yaml) (the default), [`sim-quick.yaml`](configs/sim-quick.yaml)
  or [`rig.yaml`](configs/rig.yaml). Any key it leaves out takes the value in `DEFAULTS` at the
  top of `first_light.py`.
- `--set KEY=VALUE` overrides a value, read as JSON where it parses. Examples:
  `--set sim.scene=board`, `--set sim.views='["down"]'`, `--set train.iterations=500`,
  `--set train.cpu=true`, or `--set limits.pose_error_px_max=null` to drop a check.
- `--out DIR`. The default is `build/first_light/<name>`, which git ignores.
- `--from STAGE` and `--to STAGE` rerun part of the chain and reuse what the earlier stages left.
  For example, `--from train --set train.iterations=6000` retrains without scanning again.
- `train.args` and `convert.args` pass extra flags straight to `splat_train` and
  `scan_to_dataset.py`.

## Files

```
first_light.py      the command: config, stages, checks, report
sim_truth.py        the simulator's truth as the dataset's gt/, and the conversion's checks
viewer_check.mjs    the viewer's reader and probe on an exported splat, against the C++ renderer
pose_floor.py       relief against flat board, three ways of training
configs/            sim.yaml (nightly), sim-quick.yaml (pull requests), rig.yaml (first light)
plans/ring16.yaml   sixteen viewpoints of the ring's spot
tests/              unit tests: python -m pytest pipeline/tests
```

[`.github/workflows/first-light.yml`](../.github/workflows/first-light.yml) runs `sim.yaml` every
night and on demand, and `sim-quick.yaml` on pull requests that change this folder, the simulator,
the converter, the splat tools or the viewer's scripts. Each run keeps `report.md`, the logs, the
sweep previews and the splat as an artifact.
