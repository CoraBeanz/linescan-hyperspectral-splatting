# Instrument simulator

`hsisim` is a digital twin of the whole scanner: the SO-101 arm, the scan mirror, the objective and slit, the spectrograph and its IMX219 NoIR camera. Give it a scene with known spectra and a scan plan, and it renders the raw frames the camera would give at every scan line, then writes them out as a complete scan session in the rig's own on-disk format, together with the truth they were made from.

That makes every later stage testable before the parts arrive: the capture pipeline can convert a session as if it came off the Jetson, the [calibration kit](../calibration/) can calibrate the simulated instrument and turn its scans into reflectance, and the [splat trainer](../splat/) can be scored against a scene whose geometry, spectra and poses are all known.

<p align="center">
  <img src="img/simulator.png" width="100%" alt="Four panels. The relief target from above: a 5 by 5 block of coloured and patterned pillars inside a checker border, with the first sweep's scan lines drawn across it. A raw sensor frame of one line: a grey band of spectrum with dark absorption bars and horizontal stripes where the slit crosses pillar edges. The first sweep as the slit saw it, in true colour. The same sweep after calibration and rectification, which matches it.">
  <br><sub>Left: the relief target from above, with every second line of the first sweep. Then one raw frame, of the red line, and the whole sweep as the slit really saw it next to what came out of <code>hsical calibrate</code> and <code>hsical apply</code>.</sub>
</p>

## Contents

- [Quick start](#quick-start)
- [What is simulated](#what-is-simulated)
- [The session on disk](#the-session-on-disk)
- [The end-to-end check](#the-end-to-end-check)
- [Options](#options)
- [Limits](#limits)
- [Code map](#code-map)

## Quick start

On any PC with Python 3.10 or newer (no ROS needed), from this folder:

```bash
pip install -r requirements.txt
python -m hsisim selftest          # scan, calibrate, apply, compare with the truth: about 35 s
```

The self-test simulates two 12-line sweeps of the relief target plus a calibration session for the same simulated instrument, calibrates it with `hsical`, turns the sweeps into reflectance and checks the result against the truth. A whole scan, step by step:

```bash
python -m hsisim scan sim_out --plan ring                # 4 viewpoints x 107 lines: under 2 minutes at binning 4
cd ../calibration
python -m hsical calibrate ../sim/sim_out/calibration -o ../sim/sim_out/cal
python -m hsical apply ../sim/sim_out/cal ../sim/sim_out/frames/sweep_001 \
    --dark ../sim/sim_out/reference/dark --white ../sim/sim_out/reference/white -o ../sim/sim_out/spectra
cd ../sim
python -m hsisim evaluate sim_out sim_out/cal             # applies every sweep and compares with the truth
python -m hsisim preview sim_out                          # pictures in sim_out/preview/
python -m pytest                                          # 47 tests, about two minutes
```

## What is simulated

Everything that would make a real scan differ from the scene is in the frames, and the code for most of it is the repo's own, imported rather than copied, so the simulator can't drift from what it simulates.

| Stage | What is modelled | From |
|---|---|---|
| Scene | Solids on the table (a plane, boxes, spheres) with textured matte faces: solid, checker, bitmap and random-dot patterns. Lit evenly, as in a light tent, or by a halogen lamp with distance and angle fall-off and hard shadows. | `scene.py`, `scenes.py` |
| Spectra | The splat's materials, curve for curve, plus PTFE (the white reference) and a rare-earth tile with six narrow absorption bands at known wavelengths, shaped after didymium glass. | `spectra.py`, after `splat/src/spectra.cpp` |
| Objective and slit | The 16 mm f/4 objective focused at 150 mm images the 5 mm × 50 µm slit onto a 41.9 × 0.42 mm line. Every slit point is traced as a bundle of rays over the 4 mm aperture, so depth of field and occlusion at depth edges come out right. The slit is cut into three slices across its width, because light from each edge of the slit lands at a different place along the spectrum. | `linecam.py`, the Optiland design map |
| Spectrograph and camera | `hsical`'s synthetic instrument, deliberately off the design: spectrum shifted, rotated and stretched, barrel distortion, smile and keystone from the Optiland trace, slit taper and dust, camera lens blur, filter, grating and sensor response, the NoIR colour mosaic, pixel response and column pattern noise, hot pixels, black level 64, shot and read noise, 10-bit clipping. | `instrument.py`, `calibration/hsical/synth.py` |
| Arm and mirror | Head and line-camera poses from the URDF, through the ROS 2 package's own forward kinematics. The log differs from the truth as the rig's will: servo calibration offsets, whole encoder ticks with jitter, arm flex at each viewpoint, the mirror's homing offset and step error. | `arm.py`, `ros2/so101_scan_description` |
| Timing | The rig's two clocks, run with its own code. The camera free-runs at 30 fps with the IMX219's rolling shutter: each row stops exposing 18.9 µs after the one before (in the 1640 × 1232 mode), so the slit's rows expose over a window 21 ms longer than the exposure. The bridge locks each sweep to those frames as `frame_lock.py` plans it, the mirror moves at the ESP32's start speed from each tick and settles for 3 ms, and `line_camera`'s matcher keeps the frames whose window falls inside the mirror's hold. The camera's stamps can be given an error; rows that then expose while the mirror moves see each angle along its path for as long as it was there. | `timing.py`, `ros2/so101_scan_sweep`, `ros2/so101_scan_camera` |

The default scene, **relief**, is a 60 × 60 mm plate with a 5 × 5 block of 8 mm pillars from 1 to 12 mm tall, their tops in different paints, papers, checkers, random dots, the rare-earth tile, PTFE and a word hidden in the near infrared. It is the textured relief the [splat README](../splat/README.md#training) suggests for pinning down sweep poses, which a flat board leaves loose, and it is simple to 3D print and paint. **board** is a flat target like the splat's synthetic scene, and **white** is a PTFE sheet.

## The session on disk

```
sim_out/
├── lines.csv          one row per scan line, written by ros2's own LinesCsv: viewpoint,
│                      sweep_id, index, stamps, mirror angle, head and line-camera poses
│                      in base_link, arm joints. What the rig would log.
├── scan.json          the run, in scan_sweep's format, plus "camera", "calibration_session"
│                      and "simulated" (every setting, the instrument's flaws, the objective)
├── robot.urdf         the URDF the poses came from, built from the ROS 2 package's xacro
├── plan.yaml          the scan plan as run
├── frames/
│   ├── frames.csv     one row per line, as the capture node logs it: sweep_id, index, status,
│   │                  file (the raw frame), seq (the camera's frame number), sof_ns (its row 0
│   │                  read out), exposure_start_ns and exposure_end_ns (its slit rows' window),
│   │                  exposure_us, gain, saturated_px, n_frames (frames that fitted the hold)
│   ├── camera.json    in line_camera's format: the sensor (size, bits, black level, Bayer
│   │                  order, fps, line_time_us and slit_rows), exposure, gain, slit_reversed,
│   │                  the stamps' stamp_offset_us, and under "simulated" the stamp error
│   └── sweep_001/     one hsical frame set per sweep: meta.json and frame_0000.npy ..., the
│                      raw 10-bit frame of each line (uint16)
├── reference/         dark/ and white/ (PTFE, reflectance 0.98): hsical frame sets at the
│                      scan's exposure and gain
├── calibration/       an hsical calibration session of the same simulated instrument:
│                      lamps, flat, wires, laser, darks
└── truth/             what the frames were made from
    ├── lines_true.csv   lines.csv's columns with the true joints, mirror angles and poses
    ├── weights.npy      [line, h, material] each material's share of the light at each
    │                    slit position (times its shading)
    ├── materials.npy    [material, nm] their reflectance; nm.npy and h.npy are the axes
    ├── depth.npy        [line, h] distance (camera z) of what each slit position saw
    ├── pixel_maps.npz   the wavelength and slit position of every sensor pixel
    ├── frames_true.csv  when each kept frame really exposed (sof_ns and its slit rows' window),
    │                    its line's tick and the move onto it, and moving_share: the most of any
    │                    slit row's exposure spent while the mirror was off the line
    └── scene.json, truth.json
```

Conventions, the same as the rig's:

- Sweeps are numbered from 1, as the mirror bridge numbers them, and frames by the line's index in its sweep.
- The slit position h runs from −1 to +1 along the line camera's +x axis, and on the sensor the slit row grows with h. So the first rectified slit row is the camera's −x end, which is pixel 0 of the splat's dataset. With `--slit-reversed` the camera is mounted the other way: the frames (and the calibration session's) come out mirrored along the slit, `hsical calibrate` keeps the rows as they are, and camera.json's `slit_reversed: true` tells `scan_to_dataset` to flip them, so the dataset comes out the same.
- Frames are as the camera gives them, so `hsical apply` works on a sweep folder unchanged.
- Times are as the rig logs them. In lines.csv a line's stamp is when the mirror settled (line 0 of a sweep is already there at its tick) and its hold ends 50 µs before the next tick. In frames.csv a frame's stamp is its row 0's read-out plus `stamp_offset_us`, and its window runs from the first slit row's exposure start to the last one's read-out, as `SensorTiming` works it out. The line period is a whole number of frames, enough for one window plus the mirror's move and margins: at a 20 ms exposure, two frames (66.7 ms) a line where the plan asks for one.

`hsisim.truth.ScanTruth` reads the truth back: `ScanTruth("sim_out").reflectance(line, nm)` is what each slit position of a line really saw.

## The end-to-end check

`python -m hsisim evaluate` (and the self-test) turns every sweep into reflectance with `hsical` and compares it with the truth three ways:

- **Reflectance:** every rectified cell that saw a single material, a few blur widths away from any edge, against that material's reflectance at the slit position and wavelength the cell really sees.
- **Wavelength:** the rare-earth tile's six absorption bands, fitted in the scan's own spectra, against the same fit to the true spectrum blurred to the calibrated resolution. No lamp is in the scene, so this checks the calibration's wavelengths on scene light.
- **Geometry:** where each rectified slit row really looks against where its row number puts it, and each line's profile along the slit against the truth's, which also proves the slit runs the right way.

Results at binning 4 (seed 7, uniform light), the self-test and the full `ring` plan:

| Check | Limit | Self-test, 24 lines | Ring, 428 lines |
|---|---|---|---|
| Median reflectance error, as a ratio | 1% | 0.00% | 0.00% |
| RMS reflectance error, as a ratio (one frame per line) | 5% | 1.6% | 1.6% |
| RMS reflectance error | 0.02 | 0.009 | 0.009 |
| Worst rare-earth band centre | 0.30 nm | 0.07 nm | 0.04 nm |
| Worst slit row position | 0.30 px | 0.16 px | 0.16 px |
| Median shift of a line's profile along the slit | 0.5 px | 0.15 px | 0.15 px |
| Correlation with the true profile (mirrored) | 0.9 | 0.997 (0.09) | 0.997 (0.06) |

The slit rows' error is the calibration's: the keystone fit puts rows up to 0.16 px (binned) from where they really look, in a smooth S along the slit, and the profile shifts follow it.

## Options

```bash
python -m hsisim scan --help
```

- `--plan ring`, or any plan file from `make_plan`; `--views down,tilt25_az90` keeps only some viewpoints, and `--lines`, `--steps-per-line`, `--start-angle-deg` and `--line-period-s` override the sweep.
- `--scene relief|board|white` and `--lighting uniform|lamp`.
- `--binning 4` (the default), 2, or 1 for the full 3280 × 2464 sensor. At binning 4 a line takes about 0.17 s and 1 MB on disk. At binning 1 it takes about 6 s and 16 MB, with 1.4 GB of memory, so a full ring plan is most of an hour and 7 GB.
- `--seed`: the instrument's flaws, the pose errors and the noise all follow it; the same seed gives the same lines.csv and frames.
- `--errors 0` for perfect poses (the log is then the truth), or 2 for twice the default errors.
- `--exposure-ms`, `--gain`, `--slices` and `--rays` (rays per slit point).
- `--fps` (30 by default, 21 at binning 1, as the IMX219's modes allow); an exposure must fit in one frame.
- `--stamp-error-us 15000` makes the camera's stamps 15 ms late, as a driver whose timestamp isn't row 0's read-out would. The bridge locks to the stamps, so the frames really expose earlier, and once the error is more than the lock's slack can take (9.3 ms late in the self-test's sweeps of 16 steps a line, 13.6 ms in the ring plan's 2-step lines) the first slit rows catch the end of the mirror's move. `--stamp-offset-us -15000` is `line_camera`'s correction and gives back the exact session. truth/frames_true.csv says how much of each line moved.
- `--slit-reversed`: the camera mounted the other way along the slit.

In Python, `hsisim.session.simulate(out, plan, Settings(...))` takes the same settings, and `Settings.truth` overrides any of hsical's instrument flaws, for example `{"blur_px": 12.0}` for a soft camera lens.

## Limits

- **Matte surfaces only**: no specular highlights, no light bouncing between surfaces, no fluorescence.
- **Ideal objective and mirror**: a thin lens with a Gaussian blur, a flat mirror that reflects every wavelength equally. Stray light and ghosts aren't modelled.
- **Only the mirror moves, and cleanly**: the arm holds still through a sweep, the mirror moves at a steady speed and is still the moment it gets there (the 3 ms settle is margin, not ringing), and the camera's and the ESP32's clocks run at the same rate and agree but for the stamps' error.
- **One camera**: the pose camera and its AprilTags aren't simulated; the arm's poses come from its joints alone.
- **No drift**: the instrument stays as calibrated, with no temperature changes between the calibration session and the scan.

## Code map

```
hsisim/
├── __main__.py     command line: scan, selftest, evaluate, preview
├── repo.py         imports calibration/hsical and the ROS 2 packages' modules; builds robot.urdf
├── spectra.py      the spectral library and preview colours
├── scene.py        solids, textures, lighting, ray casting
├── scenes.py       relief, board, white
├── linecam.py      the objective and the rays one scan line traces
├── instrument.py   hsical's synthetic spectrograph, rendering scene light into raw frames
├── arm.py          true and logged poses
├── session.py      a whole scan session on disk
├── timing.py       the camera's and the mirror's clocks: the frame lock, rolling shutter, stamps
├── truth.py        reading the truth back
├── evaluate.py     the end-to-end check
└── preview.py      pictures of a session
tests/              47 tests: rays, scenes, rendering, the session format, timing, end to end
tools/              readme_figure.py draws img/simulator.png
```
