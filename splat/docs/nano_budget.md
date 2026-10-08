# Training on the Jetson Nano: memory and time

The Nano trains with the same code as a desktop GPU, but on much less: one
Maxwell SM (128 CUDA cores, sm_53) and 4 GB of LPDDR4 that the CPU, the OS and
the GPU share. This page works out what training needs there, per dataset
preset and training mode, from what the code allocates, TheRig's
measurements, and an estimate of what the Nano's slower GPU and memory make
of the work. The **Nano, measured** columns
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
  tens of MB. The `default` preset's lines from 16 sweeps are 154 MB, and
  `full`'s from 12 sweeps 456 MB (MB here and below are 2²⁰ bytes, as the
  tools print them).
- **On the Nano they count twice.** The trainer keeps the dataset in CPU memory
  as well, and the Nano's GPU memory is the same 4 GB. So `full` needs about
  0.9 GB for its lines alone, next to the OS (about 0.5 GB headless, 1 GB or
  more with the desktop) and CUDA's own context, which the `device` line that
  `splat_train` prints includes. Fewer bands (binning the spectrograph) or
  fewer lines is the lever there; fewer features is not, since the lines keep
  every band.

### Measured on TheRig

TheRig (RTX 4070 SUPER, CUDA 13.4, Windows) ran `nano_budget.py` on all four
presets in the three GPU modes, 3000 steps each. The buffers are the same
size on any GPU, since the code sizes them, except Thrust's scratch, which
depends on the CUDA version's sort. GPU buffers by category, in MB:

| Preset | Mode | scene | gradients | Adam | pairs | splats | sort keys | pixels | measured lines | Thrust | densify | all |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| tiny (3 sweeps) | gpu | 1.3 | 1.2 | 0.0 | 13.1 | 1.8 | 0.6 | 1.7 | 0.5 | 0.6 | 0.0 | 20.8 |
| tiny | gpu-adam | 1.0 | 1.3 | 1.3 | 13.1 | 1.8 | 0.6 | 1.7 | 0.5 | 0.6 | 2.1 | 23.9 |
| tiny | basis8 | 0.9 | 1.1 | 0.9 | 13.2 | 1.8 | 0.6 | 1.3 | 0.5 | 0.6 | 1.5 | 22.3 |
| small (16 sweeps) | gpu | 3.4 | 3.2 | 0.0 | 23.9 | 3.0 | 1.1 | 6.6 | 20.1 | 1.1 | 0.0 | 62.4 |
| small | gpu-adam | 2.8 | 3.4 | 4.1 | 23.9 | 3.0 | 1.1 | 6.6 | 20.1 | 1.1 | 6.5 | 72.5 |
| small | basis8 | 1.7 | 1.9 | 1.9 | 24.0 | 3.0 | 1.0 | 4.1 | 20.1 | 1.1 | 3.1 | 61.9 |
| default (16 sweeps) | gpu | 4.5 | 4.3 | 0.0 | 24.3 | 3.0 | 1.3 | 23.5 | 153.8 | 1.3 | 0.0 | 216.1 |
| default | gpu-adam | 3.7 | 4.5 | 5.9 | 24.3 | 3.0 | 1.3 | 23.5 | 153.8 | 1.3 | 9.1 | 230.4 |
| default | basis8 | 1.7 | 2.0 | 1.9 | 24.4 | 3.0 | 1.3 | 13.8 | 153.8 | 1.3 | 3.3 | 206.5 |
| full (12 sweeps) | gpu | 7.4 | 7.2 | 0.0 | 23.7 | 3.0 | 1.7 | 92.8 | 456.4 | 1.8 | 0.0 | 593.9 |
| full | gpu-adam | 6.6 | 7.3 | 11.6 | 23.7 | 3.0 | 1.7 | 92.8 | 456.4 | 1.8 | 17.7 | 622.7 |
| full | basis8 | 1.8 | 1.9 | 2.1 | 23.8 | 3.0 | 1.7 | 50.0 | 456.4 | 1.7 | 3.5 | 546.0 |

The camera buffers (20 kB) are left out. The pairs stay near 24 MB from
`small` up, since the Gaussian count does (12,000 to 14,500 after
densification), and a step always renders 128 lines.

### What the Nano needs

On the Nano the GPU's buffers and the process's own memory come out of the
same RAM, so the two add up. The process's peak is TheRig's (the CPU side is
the same code), and it comes at the end, when the trainer compares the
trained splat with the measured lines and, for synthetic data, the
noise-free ones, which is one more copy of the lines that real scans don't
have.

| Preset | Mode | GPU buffers | Process peak | Together | Real data (no noise-free lines) | Nano, measured board RAM peak |
|---|---|---|---|---|---|---|
| small | gpu-adam | 73 MB | 184 MB | 257 MB | 237 MB | |
| small | basis8 | 62 MB | 181 MB | 243 MB | 223 MB | |
| default | gpu-adam | 230 MB | 471 MB | 701 MB | 547 MB | |
| default | basis8 | 207 MB | 460 MB | 667 MB | 513 MB | |
| full | gpu-adam | 623 MB | 1,149 MB | 1,772 MB | 1,316 MB | |
| full | basis8 | 546 MB | 1,103 MB | 1,649 MB | 1,193 MB | |

Add CUDA's own context and libraries and the OS. Taking CUDA as about 0.3 GB
and the OS as about 1 GB with the desktop or 0.5 GB headless (both guesses
until the Nano's numbers are in), everything fits the Nano's 4 GB, which
Linux sees as about 3.9: `full` leaves roughly 0.9 GB spare with the desktop
and 1.4 GB headless, and `default` and smaller leave plenty.

The first sweep found two things these numbers no longer show. Thrust's
scratch held 48 to 70 MB, because it kept every block the sort had outgrown;
it now frees them. The process peaked at 5 copies of the lines (763 MB for
`default`, 1,978 MB for `full`), because the final comparison rendered every
line at once and the `.npy` loader held the file twice; it now renders 256
lines at a time and reads float32 files straight into place.

## Time on TheRig

Time per step after the first 300 steps (the buffers grow early on), with
`--profile`, which adds a few tenths of a millisecond to the fast modes (see
below):

| Preset | Mode | Gaussians | ms/step | backward | Adam and densify | 3000 steps | pose error |
|---|---|---|---|---|---|---|---|
| tiny (3 sweeps) | gpu | 6,140 | 2.04 | 1.83 | 0.19 | 6.0 s | 2.00 px |
| tiny | gpu-adam | 6,136 | 1.24 | 1.11 | 0.13 | 3.7 s | 1.99 px |
| tiny | basis8 | 6,240 | 1.28 | 1.11 | 0.16 | 3.8 s | 2.10 px |
| small (16 sweeps) | gpu | 12,479 | 5.00 | 4.70 | 0.27 | 14.3 s | 0.46 px |
| small | gpu-adam | 12,437 | 1.62 | 1.44 | 0.16 | 4.7 s | 0.46 px |
| small | basis8 | 12,890 | 1.18 | 1.05 | 0.12 | 3.5 s | 0.45 px |
| small | cpu (28 threads) | 12,439 | 10.18 | 9.86 | 0.27 | 29.6 s | 0.46 px |
| default (16 sweeps) | gpu | 12,975 | 7.80 | 7.36 | 0.40 | 22.3 s | 0.94 px |
| default | gpu-adam | 12,978 | 2.46 | 2.26 | 0.18 | 7.2 s | 0.96 px |
| default | basis8 | 13,378 | 1.46 | 1.28 | 0.15 | 4.3 s | 0.98 px |
| default | cpu (28 threads) | 12,948 | 23.63 | 23.13 | 0.44 | 68.7 s | 0.96 px |
| full (12 sweeps) | gpu | 14,178 | 15.72 | 14.84 | 0.84 | 44.9 s | 2.10 px |
| full | gpu-adam | 14,161 | 5.27 | 5.00 | 0.24 | 15.5 s | 2.12 px |
| full | basis8 | 14,511 | 1.77 | 1.59 | 0.16 | 5.3 s | 2.15 px |

"Backward" is everything up to the gradients, the GPU passes and the copies
included; "Adam and densify" is the update. The pose error is the same in
every mode, so neither the GPU's optimizer nor 8 features cost accuracy;
`tiny` and `full` end near 2 px because 3 sweeps see too little, and
because `full`'s pixels are half the size of `default`'s (2 px is the same
0.16 mm on the board).

Without `--profile`, `default` takes 7.10, 1.88 and 1.12 ms a step in the
three modes, and `full` 15.75, 6.28 and 1.70. `full` with `--gpu-adam` varies
from about 5.3 to 7.3 ms from run to run, mostly in its two largest passes.

Where the GPU's time goes, in ms per step, with the visible (line, Gaussian)
pairs and sort keys a step:

| Preset | Mode | project | scan | emit | sort | raster | loss | raster back | project back | geometry back | copies | adam | densify | all | visible pairs | sort keys |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| tiny | gpu | 0.029 | 0.107 | 0.060 | 0.204 | 0.078 | 0.080 | 0.263 | 0.021 | 0.022 | 0.754 | – | – | 1.62 | 37,448 | 39,137 |
| tiny | gpu-adam | 0.031 | 0.104 | 0.062 | 0.185 | 0.071 | 0.071 | 0.256 | 0.022 | 0.016 | 0.084 | 0.092 | 0.002 | 1.00 | 37,448 | 39,144 |
| tiny | basis8 | 0.031 | 0.114 | 0.069 | 0.214 | 0.058 | 0.094 | 0.172 | 0.021 | 0.018 | 0.090 | 0.105 | 0.002 | 0.99 | 37,363 | 39,085 |
| small | gpu | 0.065 | 0.195 | 0.111 | 0.218 | 0.099 | 0.119 | 0.674 | 0.052 | 0.016 | 2.347 | – | – | 3.90 | 58,160 | 68,569 |
| small | gpu-adam | 0.050 | 0.110 | 0.075 | 0.200 | 0.067 | 0.107 | 0.436 | 0.040 | 0.016 | 0.095 | 0.105 | 0.004 | 1.30 | 57,954 | 68,339 |
| small | basis8 | 0.050 | 0.097 | 0.071 | 0.182 | 0.053 | 0.092 | 0.175 | 0.031 | 0.016 | 0.066 | 0.085 | 0.003 | 0.92 | 59,859 | 70,505 |
| default | gpu | 0.069 | 0.192 | 0.114 | 0.236 | 0.173 | 0.391 | 1.276 | 0.055 | 0.019 | 3.907 | – | – | 6.43 | 59,724 | 85,704 |
| default | gpu-adam | 0.056 | 0.137 | 0.082 | 0.231 | 0.123 | 0.321 | 0.793 | 0.035 | 0.019 | 0.099 | 0.118 | 0.008 | 2.02 | 59,733 | 85,798 |
| default | basis8 | 0.058 | 0.126 | 0.083 | 0.210 | 0.051 | 0.188 | 0.178 | 0.031 | 0.014 | 0.102 | 0.109 | 0.004 | 1.15 | 61,150 | 87,292 |
| full | gpu | 0.073 | 0.151 | 0.095 | 0.214 | 0.485 | 2.454 | 3.023 | 0.053 | 0.025 | 7.156 | – | – | 13.73 | 62,634 | 120,089 |
| full | gpu-adam | 0.052 | 0.107 | 0.078 | 0.194 | 0.298 | 1.545 | 2.075 | 0.044 | 0.015 | 0.136 | 0.152 | 0.010 | 4.71 | 62,635 | 120,153 |
| full | basis8 | 0.052 | 0.096 | 0.070 | 0.180 | 0.044 | 0.473 | 0.262 | 0.052 | 0.010 | 0.105 | 0.103 | 0.003 | 1.45 | 63,814 | 122,064 |

"Copies" is the scene going up and the gradients coming down in the `gpu`
mode, and only zeroing the gradients and uploading the line cameras with
`--gpu-adam`. What this says:

- **Copying the scene every step costs more than all of the GPU's work.** In
  the `gpu` mode copies are 0.75 to 7.2 ms of a step; `--gpu-adam` cuts them
  to about 0.1 ms, which makes training about 3 times faster from `small` up.
- **The features set the rest.** With one feature per band, the loss and the
  raster passes backwards grow with the bands: at `full`'s 91 bands they are
  3.6 of 4.7 ms. With 8 features through the learned basis they are 0.7 ms,
  and every preset's GPU work takes 1 to 1.5 ms a step.
- **The small passes are mostly fixed costs.** Project, scan, emit and sort
  barely change from `small` to `full`, since a step always projects 128
  lines and sorts 70,000 to 120,000 keys, which take TheRig's GPU only a few
  launches' worth of time. On the Nano they won't be small.

## What to expect on the Nano

The Nano's GPU is one Maxwell SM at 921 MHz: 236 GFLOPS and 25.6 GB/s of
memory bandwidth, shared with the CPU. TheRig's has about 150 times the
arithmetic and 20 times the bandwidth, but neither ratio scales its times
well: its passes are so short that much of each is launch and
synchronisation overhead, and its 48 MB L2 cache holds data that the Nano's
256 KB doesn't. So this estimate works from what each pass moves and
computes in a step instead, at 20 GB/s and 150 GFLOPS (about 80% and 65% of
the peaks), 1.8 billion scattered atomic adds a second, 10 µs per kernel
launch and 50 µs per small blocking copy; each pass gets the larger of its
memory and arithmetic times. Take the totals as good to about a factor of
1.5 either way, until the measured column replaces them.

Per pass for `default` with 16 sweeps (13,000 Gaussians, 128 lines of 256
pixels, 46 bands, and per step 60,000 visible pairs and 86,000 sort keys), in
ms per step:

| Pass | What it costs on the Nano | TheRig, gpu-adam | Nano, gpu-adam | TheRig, basis8 | Nano, basis8 |
|---|---|---|---|---|---|
| project | every Gaussian's 52 bytes, read once per line: 86 MB | 0.06 | 4.3 | 0.06 | 4.3 |
| scan | two prefix sums over 1.7 million pairs: 27 MB | 0.14 | 1.5 | 0.13 | 1.5 |
| emit | the pairs' ranges and the visible splats: 12 MB | 0.08 | 0.6 | 0.08 | 0.6 |
| sort | 11 radix passes over 86,000 keys and values: 30 MB | 0.23 | 1.9 | 0.21 | 1.9 |
| raster | 86,000 entries × 32 pixels × 3 chunks of 16 features: 360 MFLOP (8 features, one chunk: 80) | 0.12 | 2.4 | 0.05 | 0.5 |
| loss | basis × features per pixel and band, and back: 280 MFLOP and 42 MB (8 features: 39 MB) | 0.32 | 2.2 | 0.19 | 2.1 |
| raster back | about twice the raster's arithmetic, and 5 million atomic adds (8 features: 1 million) | 0.79 | 8.0 | 0.18 | 1.7 |
| project back | 60,000 pairs' gradients: 0.8 million atomic adds | 0.04 | 0.7 | 0.03 | 0.7 |
| geometry back | 13,000 Gaussians | 0.02 | 0.1 | 0.01 | 0.1 |
| copies | zeroing the gradients, uploading the cameras | 0.10 | 0.4 | 0.10 | 0.3 |
| adam | 59 values a Gaussian (8 features: 19), 28 bytes each: 21 MB (7 MB) | 0.12 | 1.2 | 0.11 | 0.5 |
| all | | 2.0 | 23 | 1.2 | 14 |

Two things stand out. The passes that are nearly free on TheRig (project,
scan, emit and sort) come to 8 ms on the Nano, a third of the `gpu-adam`
step and over half of the `basis8` one, so that is where to look first if
the Nano is slow (see the end of this page). And the raster passes, with
their atomic adds, grow with the features, so 8 features take about 9 ms
off the Nano's step.

The same arithmetic for every preset, with a column for the Nano's
measurements:

| Preset | Mode | TheRig ms/step | Nano estimate, ms/step | 3000 steps on the Nano | Nano, measured ms/step |
|---|---|---|---|---|---|
| small (16 sweeps) | gpu-adam | 1.62 | 11 to 24 | 0.5 to 1.2 min | |
| small | basis8 | 1.18 | 8 to 18 | 0.4 to 0.9 min | |
| default (16 sweeps) | gpu-adam | 2.46 | 16 to 35 | 0.8 to 1.8 min | |
| default | basis8 | 1.46 | 10 to 21 | 0.5 to 1.1 min | |
| full (12 sweeps) | gpu-adam | 5.27 | 40 to 85 | 2 to 4.3 min | |
| full | basis8 | 1.77 | 15 to 33 | 0.8 to 1.7 min | |

The other two modes, roughly:

- **`gpu`** (no `--gpu-adam`) adds the scene's copies and Adam on the CPU,
  which take TheRig 4.3 ms a step at `default` and are mostly CPU work (the
  copies here are pageable, through the CPU). The Nano's Cortex-A57 cores do
  that about 5 to 10 times slower, so expect 20 to 40 ms a step more than
  `gpu-adam`: use `--gpu-adam` on the Nano.
- **`cpu`** runs on 4 A57 cores instead of TheRig's 28 threads, roughly 30
  times slower: about 15 minutes for `small` and 35 for `default`.

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
four, or `--modes gpu,gpu-adam,basis8,cpu` for the CPU as well (slow: 15 to
35 minutes a run). Running headless (`sudo systemctl isolate multi-user.target`)
frees about half a GB for `full`.

The `RAM a/b MB` field in tegrastats' log is the whole board's memory, the
number the 4 GB has to cover; its peak during each run goes in the memory
table's measured column.

## If the Nano is slower than this

What to try, in the order the estimate above suggests:

- **Read each Gaussian once per block of lines, not once per line.** The
  projection pass runs a thread per (line, Gaussian) pair, and each reads the
  Gaussian's 48 bytes of geometry. TheRig's 48 MB L2 cache holds them all, so
  that costs nothing there; the Nano's 256 KB doesn't, so at the default
  preset it reads about 80 MB a step from memory. A thread that projects one
  Gaussian into 8 lines would read a sixteenth of that.
- **Sort fewer bits.** The keys are 64 bits, but the line and tile in the
  high half need only about 11 (128 lines, up to 16 tiles), so a radix sort
  told to skip the unused bits (CUB's `begin_bit`/`end_bit`, which CUDA 10.2
  ships inside Thrust) makes about a quarter fewer passes.
- **Fewer features.** `--basis 8` is the biggest lever already in the code: the
  raster passes, the loss and Adam all scale with the features.
- **Measured lines as 16-bit floats.** Halves the largest buffer (and its copy
  in CPU memory), rounding each value by at most 0.05%, far below the
  sensor's noise.
- **Drop the CPU's copy of the lines** once they are on the GPU (`--gpu-adam`
  needs them on the CPU only for the final comparison, which could read the
  file again).
