# Training on the Jetson Nano: memory and time

The Nano trains with the same code as a desktop GPU, but on much less: one
Maxwell SM (128 CUDA cores, sm_53) and 4 GB of LPDDR4 that the CPU, the OS and
the GPU share. This page works out what training needs there, per dataset
preset and training mode, from what the code allocates and from TheRig's
measurements scaled by the two machines' specs. The **Nano, measured** columns
stay empty until the commands at the end run on the Nano.

## The training modes

| Mode | `splat_train` flags | Where the work runs |
|---|---|---|
| `gpu` | (none; the default with a GPU) | render and backward on the GPU; Adam and densification on the CPU, so the scene goes up and its gradient comes down every step |
| `gpu-adam` | `--gpu-adam` | everything on the GPU; only the 128 line cameras go up and their pose gradients come down |
| `basis8` | `--gpu-adam --basis 8` | as `gpu-adam`, with 8 features per Gaussian through a learned spectral basis instead of one feature per band |
| `cpu` | `--cpu` | everything on the CPU, for comparison |

## What the GPU holds

Every buffer is sized from a few counts: Gaussians (N), the 128 lines a step
renders (L), pixels per line (W), bands (B), features (K, padded to Kp, a
multiple of 8 or 16 for the raster kernel), and two counts that depend on the
scene: the visible (line, Gaussian) pairs and the tile entries (one per
visible pair per 32-pixel tile it covers). In bytes:

| Buffers | Bytes | Category in `--profile` |
|---|---|---|
| parameters (48 B of covariance and 44 B of means, scales, rotation, opacity) and features | N × (92 + 4 Kp) | scene |
| their gradients, plus the screen-space gradient and pair count densification reads | N × (80 + 4 Kp), + 12 N with `--gpu-adam` | gradients |
| Adam's two moments (`--gpu-adam` only) | N × (88 + 8 Kp) | Adam |
| per (line, Gaussian) pair: packed range, two scan offsets | 12 × L × N | pairs |
| per visible pair: the 1D splat, its Gaussian and line, its gradient | 40 per visible pair | splats |
| per tile entry: 64-bit sort key and 32-bit value, and the line-tile ranges | 12 per entry + 8 L ⌈W/32⌉ | sort keys |
| Thrust's scratch: the radix sort's second copy of keys and values, and the scans | about 12 per entry | Thrust |
| per pixel: rendered features, transmittance, contributor count, error, dL/dband, dL/dfeature | L × W × (8 + 4K + 8B + 4Kp) | pixels |
| the measured lines, uploaded once | 4 × lines × W × B | measured lines |
| densification's new arrays while it rebuilds them (`--gpu-adam` only; freed right after) | 28 N + 12 N′ (11 + Kp), N′ the new count | densify |

Buffers only grow, by at least 1.5× at a time so that a slowly rising count
doesn't reallocate every step, so a buffer can hold up to half again what it
needs. The numbers `--profile` prints are what is allocated.

Two things follow:

- **The measured lines dominate.** Everything per Gaussian is about 1 kB at
  46 bands (N = 13,000 is 13 MB), and a step's pairs, splats and pixels are
  tens of MB. The `default` preset's lines from 16 sweeps are 161 MB, and
  `full`'s from 12 sweeps 479 MB.
- **On the Nano they count twice.** The trainer keeps the dataset in CPU memory
  as well, and the Nano's GPU memory is the same 4 GB. So `full` needs about
  1 GB for its lines alone, next to the OS (about 0.5 GB headless, 1 GB or more
  with the desktop) and CUDA's own context, which `--profile` reports as part
  of "device used". Fewer bands (binning the spectrograph) or fewer lines is
  the lever there; fewer features is not, since the lines keep every band.

THERIG_MEMORY

## Time on TheRig

THERIG_TIME

## What to expect on the Nano

NANO_ESTIMATE

## Measuring it on the Nano

From the repo on the Nano, with the container that [`jetson/splat.sh`](../../jetson/README.md)
builds (JetPack 4.6's CUDA 10.2):

```bash
sudo nvpmodel -m 0 && sudo jetson_clocks       # 10 W mode and fixed clocks, so runs compare
jetson/splat.sh build                          # the CUDA 10.2 build for sm_53
jetson/splat.sh test                           # every test, the GPU ones included
sudo tegrastats --interval 1000 --logfile budget-tegrastats.txt &   # RAM and GPU load once a second
jetson/splat.sh python3 /src/splat/tools/nano_budget.py --label "Jetson Nano"
sudo pkill tegrastats
```

`nano_budget.py` makes the `small` and `default` datasets with 16 sweeps (once,
under `~/linesplat/budget`), trains each in the `gpu`, `gpu-adam` and `basis8`
modes for 3000 steps, and prints the tables above with the Nano's numbers; it
writes them to `~/linesplat/budget/results.json` too. Add `--presets tiny,small,default,full` for all
four, or `--modes gpu,gpu-adam,basis8,cpu` for the CPU as well (slow: expect
minutes per run). Running headless (`sudo systemctl isolate multi-user.target`)
frees about half a GB for `full`.

The `RAM a/b MB` field in tegrastats' log is the whole board's memory, the
number the 4 GB has to cover; its peak while `full` trains is the one to
record.

LEVERS
