# splat: line-camera Gaussian splatting

This is the reconstruction side of the rig. The spectrograph never takes a
picture: each camera frame is one line of the scene (every pixel along the
slit, every wavelength), the scan mirror sweeps that line across the object,
and the arm moves the head between sweeps. So the training data is thousands of
one-row images, each with its own pose, and the splat has to be rendered
through a camera that has a single row of pixels.

Milestone 1 is here:

- a **line-camera renderer** for hyperspectral Gaussians: a CPU reference (in
  `float` or `double`) and a CUDA rasterizer that matches it, written for the
  Jetson Nano's CUDA 10.2
- a **synthetic pushbroom dataset**: a known scene with realistic spectra, scanned
  through the head's mirror geometry from `cad/`, with the kind of pose errors
  the arm will have, plus the ground truth to score against
- tools to make and inspect datasets, and unit tests that pin the math down

Training (the backward pass, the optimizer and pose refinement) comes next; see
[Next steps](#next-steps).

<table>
  <tr>
    <td width="50%"><img src="docs/scene_rgb.png" width="100%" alt="The synthetic scene in true color: a white board with a checker border, six paint patches, a black panel, a green ball and an orange box, on a wooden table."></td>
    <td width="50%"><img src="docs/scene_cir.png" width="100%" alt="The same scene in color infrared: the black panel shows the word NIR, the green ball turns red and the orange box turns yellow."></td>
  </tr>
</table>

The synthetic scene, rendered by this code from its overview camera (an image
is just a stack of line cameras, one per row). Left: true color, computed from
each pixel's 500–950 nm spectrum. Right: color infrared, with 800–900 nm shown
as red, red as green and green as blue. The black panel hides the word "NIR",
printed in carbon black on a dye that is black to the eye but clear past
740 nm, and the green ball turns red because leaves reflect strongly in the
near infrared.

## What the scanner records

<table>
  <tr>
    <td width="50%"><img src="docs/sweep_rgb.png" width="100%" alt="One sweep looking nearly straight down, in true color: the ball, the box, the paint patches and the black panel."></td>
    <td width="50%"><img src="docs/sweep_cir.png" width="100%" alt="The same sweep in color infrared, with the word NIR readable on the panel."></td>
  </tr>
</table>

One sweep looking nearly straight down: 214 scan lines of 256 pixels, stacked in the
order the mirror took them. The lines are 0.29 mm apart on the board and the
pixels 0.16 mm, so round things look squashed; the renderer knows each line's
pose, so this never has to be resampled into a square-pixel image.

<img src="docs/pose_error.png" width="100%" alt="Three panels of one oblique sweep: the measured lines, the scene rendered at the recorded poses, which is shifted and rotated, and the scene rendered at the true poses, which matches.">

Why poses need refining: one oblique sweep as measured (left), the true scene
rendered at the pose the arm reported (middle), and at the true pose (right).
A head pose that is off by 3.3 mm and a quarter of a degree moves the whole
sweep.

| Lines rendered from the true scene at... | RMSE vs measured |
|---|---|
| the recorded poses | 0.186 |
| the true poses | 0.016 (the noise level) |
| the true poses, vs the noise-free lines | 0.006 |

(Reflectance units, `splat_render residual` on the default dataset. The last
row is what is left from modelling the slit as a Gaussian rather than a box.)

## The model

### A line camera

Each scan line is a pinhole camera with one row of pixels. In camera
coordinates (x along the slit, y across it, z forward, in metres):

```
u = f x / z + cu        along the slit, in pixels; pixel p covers [p, p + 1)
v = f y / z             across the slit; the slit sits at v = v_slit
```

`f` is the objective-to-slit distance measured in pixels, where a pixel is the
slit's length over the pixel count: 17.91 mm over a 5 mm slit binned to 256
pixels gives f = 917 px, so a line covers 41.9 mm at 150 mm, as in the
Optiland model. Each line also carries a
blur: `sigma_u` along the slit (pixel pitch plus optics) and `sigma_v` across it
(the 50 µm slit's width plus optics). See
[`line_camera.hpp`](include/linesplat/line_camera.hpp).

### The mirror and the virtual camera

The objective looks down the head's -Z axis into the 45° scan mirror, and the
scene sees a mirror image of the camera, the *virtual camera*. Turning the
mirror by an angle turns the virtual camera's view by twice that angle. A
reflection flips handedness, so the virtual camera's y axis is negated to keep
it a proper rotation, and `v_slit` flips sign with it.

The pose of line *l* in sweep *s* is

```
camera_in_world(l) = head_in_world(s) * virtual_camera_in_head(mirror_angle(l))
```

`head_in_world` comes from the arm (or the AprilTag board) and is fixed for the
whole sweep; the mirror angle comes from the stepper's step count. The head
geometry is `cad_head_model()` in
[`scan_model.cpp`](src/scan_model.cpp), taken from the head layout in
`cad/rig/params.py` with the Edmund 20 × 20 × 3 mm mirror. Mirror angles follow
the ROS 2 `ScanLine` message: radians, 0 at the 45° rest, right-handed about the
head's +X (shaft) axis.

Splitting the pose this way is what makes training possible. A single line says
almost nothing about where it sat across the slit, but all 214 lines of a sweep
pin down one 6-DoF head pose and one mirror offset between them.

### Drawing a Gaussian on a line

This is the 3DGS projection, cut down to one row. Each 3D Gaussian is projected
to a 2D Gaussian on the image plane (the local affine, or EWA, approximation),
widened by the pixel's footprint, and then read off along the row
`v = v_slit`. A 2D Gaussian cut along a line is a 1D Gaussian, so every
(line, Gaussian) pair becomes a 1D splat:

```
S  = J W Σ Wᵀ Jᵀ                        2D covariance before blur, [a b; b c]
S' = S + diag(sigma_u², sigma_v²)       with the pixel and slit footprint
dv = v_slit - mu_v                      how far the Gaussian's centre is off the slit

u*      = mu_u + (b / c') dv            centre along the line
var_u   = det(S') / c'                  width along the line
alpha_0 = opacity * k * exp(-dv² / 2c') peak alpha on the line
k       = sqrt(det(S) / det(S'))        blur spreads alpha out but keeps its total

alpha(p) = min(0.99, alpha_0 * exp(-(p + 0.5 - u*)² / 2 var_u))
```

Pixels are then composited front to back, the same as 3DGS, with the same
cutoffs (skip alpha below 1/255, stop at transmittance 10⁻⁴). The tests check
this against an independent 2D EWA splat with explicit matrices, to 10⁻⁹.

### Spectral features

Each Gaussian stores K features instead of RGB, and a shared basis
(bands × K) turns features into a spectrum. Compositing is linear in the color,
so the renderers composite K channels and apply the basis once per pixel. The
synthetic ground truth uses K = 46 and an identity basis (the features are the
spectrum); training can use a small K with a learned basis.

## The CUDA rasterizer

It is the 3DGS tile rasterizer with the image collapsed to rows of tiles. Per
batch of lines:

| Pass | What it does |
|---|---|
| count | one thread per (line, Gaussian) pair: project, keep the pixel range if it reaches the line |
| scan | prefix sums give every visible pair a slot and every (pair, 32-pixel tile) overlap a key slot |
| emit | write the 1D splats and the keys: line and tile in the high 32 bits, depth in the low 32 |
| sort | Thrust radix sort, so each tile's splats come out front to back |
| raster | one warp per (line, tile), one lane per pixel; the warp pulls the tile's splats through shared memory 32 at a time and composites until every pixel is opaque |

Some choices, briefly:

- **Tiles of 32 pixels** make one tile one warp, so the 32 pixels share every
  load and stay in step without any synchronisation beyond the warp's.
- **Features in chunks** of 8 or 16 channels (the grid's y dimension) keep the
  accumulators in registers for any number of bands.
- **Batches of lines** cap the scratch memory: about 12 bytes per
  (line, Gaussian) pair, 200 MB at the default 2²⁴ pairs, which leaves room on
  the Nano's 4 GB shared with the CPU.
- **CUDA 10.2 limits** (JetPack 4.6, sm_53): device code is C++14, scans and
  sorts use Thrust (CUB isn't bundled before CUDA 11), only `float` atomics,
  and warp intrinsics use the `_sync` forms. The host code is C++17 that gcc 7
  builds (so no `std::filesystem`).

The tests run both renderers on the same lines and require all but 0.1% of
values to agree to 10⁻⁴ (float rounding can tip a splat across a cutoff).
On a 4-core cloud CPU the reference renders the default dataset (1712 lines ×
256 pixels × 46 bands) in 0.12 s; GPU timings will come from TheRig and the Nano.

## Dataset format

A dataset is a directory. The synthetic generator writes it, and the capture
side will write the same thing from the rig, so the trainer can't tell the two
apart.

```
dataset.json              format "linesplat-dataset", version 1: intrinsics, head
                          model, wavelengths (nm), file names, free-form metadata
lines.npy                 float32 [L, W, B]  one spectrum per pixel per line
line_sweep.npy            int32   [L]        which sweep each line belongs to
line_mirror_angle.npy     float64 [L]        mirror angle per line (rad)
sweep_head_pose.npy       float64 [S, 7]     head -> world per sweep:
                                             tx, ty, tz, qw, qx, qy, qz (m)
gt/                       synthetic only: scene/, true poses and mirror angles,
                          noise-free lines
preview/                  true color and CIR quick looks of each sweep
```

The ROS 2 side's `ScanLinePose` message maps straight onto this: `mirror_angle`
is `line_mirror_angle`, each `sweep_id` becomes one sweep index in `line_sweep`,
and `head_pose` (constant while the arm holds still) is that sweep's
`sweep_head_pose`, as long as its head frame is
the CAD's: origin on the wrist-roll horn face, +Z away from the wrist, +Y out
of the scan window, +X along the mirror shaft. Pixel 0 is at the camera's -x
end of the slit, which is the head's -X end; a sensor that reads the other way
gets flipped on load. Everything is in metres and radians.

## The synthetic dataset

- **Scene** (9741 Gaussians, 1 mm apart): an 80 × 64 mm white board with a
  4 mm checker border, six paint patches, the hidden-word panel, an 18 mm
  leaf-green ball and a 12 × 10 × 10 mm orange box, on a wooden table. The
  spectra are analytic curves shaped like the real materials.
- **Scan**: 8 sweeps from 150 mm, one looking nearly straight down and seven oblique
  ones (48° and 62° elevation) around the board. The mirror covers ±6° (a 24°,
  63 mm fan), one 1/32 microstep of the NEMA 8 per line.
- **Errors**: each sweep's head pose is off by 1 mm and 0.3° per axis (1σ), the
  mirror's homing by 0.05°, and each line's microstep by 0.005°. The values get
  read noise and shot noise (σ = 0.02 on white).
- **Slit**: the measured lines integrate the slit's width as a box (5 sub-lines
  per line); the renderer models it as a Gaussian of the same width.

| Preset | Pixels | Bands | Sweeps | Lines | `lines.npy` |
|---|---|---|---|---|---|
| `tiny` | 64 | 12 | 3 | 162 | 0.5 MB |
| `small` | 128 | 24 | 6 | 642 | 7.5 MB |
| `default` | 256 | 46 | 8 | 1712 | 77 MB |
| `full` | 512 | 91 | 12 | 2568 | 456 MB |

`tiny` and `small` take 4 and 2 microsteps per line; `full` also packs the
Gaussians 0.7 mm apart.

## Building

You need CMake 3.18 or newer and a C++17 compiler. The CUDA rasterizer is built
when CMake finds `nvcc`; without it you get the CPU renderer only.
[nlohmann/json](https://github.com/nlohmann/json) comes from the system if it
has 3.2 or newer, otherwise CMake downloads it.

```bash
# Linux, from the repo root
cmake -S splat -B build/splat -DCMAKE_BUILD_TYPE=Release
cmake --build build/splat -j
build/splat/splat_tests
```

On a desktop GPU the build targets the card in the machine (with CMake 3.24 or
newer; older ones need `-DCMAKE_CUDA_ARCHITECTURES`, such as 86 for an RTX 30
series card). For the
**Jetson Nano** (JetPack 4.6: CUDA 10.2, gcc 7), install a newer CMake first,
since JetPack's 3.10 is too old:

```bash
python3 -m pip install --user --upgrade pip   # JetPack's pip is too old for the cmake wheels
python3 -m pip install --user cmake           # then put ~/.local/bin first on PATH
cmake -S splat -B build/splat -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc -DCMAKE_CUDA_ARCHITECTURES=53
cmake --build build/splat -j4
```

On **Windows** with Visual Studio 2022 and the CUDA toolkit:

```bat
cmake -S splat -B build\splat
cmake --build build\splat --config Release
build\splat\Release\splat_tests.exe
```

## Tools

Run them from the build directory, so the datasets stay out of git (`build/`
is ignored). Either tool prints its flags when run with no arguments.

```bash
cd build/splat

# Make a dataset: on the GPU if there is one, --preset tiny|small|default|full,
# --no-errors for perfect poses
./splat_synth synth --preset small

# CPU vs GPU on every line, with timings
./splat_render compare synth

# How well a scene explains the lines, at the recorded and the true poses;
# --png writes measured | recorded | true images per sweep
./splat_render residual synth --png synth/residual

# True color and CIR images of a scene from the overview camera
./splat_render view synth/gt/scene synth/view
```

## Code map

```
include/linesplat/
├── math.hpp              small vector, matrix and quaternion helpers, host and device
├── line_camera.hpp       the line camera and project_to_line(): Gaussian -> 1D splat
├── scene.hpp             GaussianScene: shapes, opacities, spectral features, basis
├── scan_model.hpp        poses, the scan mirror, the CAD head, intrinsics from the optics
├── render_cpu.hpp        reference renderer, float or double
├── cuda_rasterizer.hpp   the CUDA renderer (src/cuda/rasterizer.cu)
├── dataset.hpp           the dataset format above
├── synthetic.hpp         the synthetic scene and scan
├── spectra.hpp           material spectra, true color (CIE 1931) and CIR
└── preview.hpp, png.hpp, npy.hpp, rng.hpp, util.hpp
src/                      a .cpp per header, and cuda/rasterizer.cu
tools/                    splat_synth, splat_render
tests/                    one file per topic; test_cuda skips without a GPU
```

## Next steps

1. **Backward pass on the CPU** in `double`, checked against finite
   differences, so the CUDA gradients have something exact to match.
2. **CUDA backward pass**: composite back to front per pixel and accumulate
   Gaussian gradients with `float` atomics (sm_53 has no `double` atomicAdd).
3. **Optimizer and densification**: Adam, then split, clone and prune adapted
   to lines, where a Gaussian only gets gradient from the lines that cut it.
4. **Pose refinement**: one SE(3) correction per sweep plus a mirror offset,
   trained with the Gaussians. Success means recovering the synthetic errors
   above to well under a pixel.
5. **Real data**: a converter from the ROS 2 scan logs to this format, using
   the wavelength map and warp from the calibration work.
