# Instrument simulator

`hsisim` is a digital twin of the whole scanner: the SO-101 arm, the scan mirror, the objective and slit, the spectrograph and its IMX219 NoIR camera, and the pose camera beside them. Give it a scene with known spectra and a scan plan, and it renders the raw frames the camera would give at every scan line, then writes them out as a complete scan session in the rig's own on-disk format, together with the truth they were made from.

That makes every later stage testable before the parts arrive: the capture pipeline can convert a session as if it came off the Jetson, the [calibration kit](../calibration/) can calibrate the simulated instrument and turn its scans into reflectance, the [hand-eye calibration](../calibration/headcal/) can find a head that is off its CAD numbers, and the [splat trainer](../splat/) can be scored against a scene whose geometry, spectra and poses are all known.

<p align="center">
  <img src="img/simulator.png" width="100%" alt="Four panels. The relief target from above: a 5 by 5 block of coloured and patterned pillars inside a checker border, with the first sweep's scan lines drawn across it. A raw sensor frame of one line: a grey band of spectrum with dark absorption bars and horizontal stripes where the slit crosses pillar edges. The first sweep as the slit saw it, in true colour. The same sweep after calibration and rectification, which matches it.">
  <br><sub>Left: the relief target from above, with every second line of the first sweep. Then one raw frame, of the red line, and the whole sweep as the slit really saw it next to what came out of <code>hsical calibrate</code> and <code>hsical apply</code>.</sub>
</p>

## Contents

- [Quick start](#quick-start)
- [What is simulated](#what-is-simulated)
- [The session on disk](#the-session-on-disk)
- [The end-to-end check](#the-end-to-end-check)
- [A hand-eye calibration scan](#a-hand-eye-calibration-scan)
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
python -m pytest                                          # 53 tests, about four minutes
```

## What is simulated

Everything that would make a real scan differ from the scene is in the frames, and the code for most of it is the repo's own, imported rather than copied, so the simulator can't drift from what it simulates.

| Stage | What is modelled | From |
|---|---|---|
| Scene | Solids on the table (a plane, boxes, spheres) with textured matte faces: solid, checker, bitmap and random-dot patterns. Lit evenly, as in a light tent, or by a halogen lamp with distance and angle fall-off and hard shadows. | `scene.py`, `scenes.py` |
| Spectra | The splat's materials, curve for curve, plus PTFE (the white reference) and a rare-earth tile with six narrow absorption bands at known wavelengths, shaped after didymium glass. | `spectra.py`, after `splat/src/spectra.cpp` |
| Objective and slit | The bought 15.6 mm f/3.9 objective, focused at 150 mm, images the 5 mm × 50 µm slit onto a 43.1 × 0.43 mm line. Every slit point is traced as a bundle of rays over the 4 mm aperture, so depth of field and occlusion at depth edges come out right. The slit is cut into three slices across its width, because light from each edge of the slit lands at a different place along the spectrum. | `linecam.py`, the Optiland design map |
| Spectrograph and camera | `hsical`'s synthetic instrument, deliberately off the design: spectrum shifted, rotated and stretched, barrel distortion, smile and keystone from the Optiland trace, slit taper and dust, camera lens blur, filter, grating and sensor response, the NoIR colour mosaic, pixel response and column pattern noise, hot pixels, black level 64, shot and read noise, 10-bit clipping. | `instrument.py`, `calibration/hsical/synth.py` |
| Arm and mirror | Head and line-camera poses from the URDF, through the ROS 2 package's own forward kinematics. The log differs from the truth as the rig's will: servo calibration offsets, whole encoder ticks with jitter, arm flex at each viewpoint, the mirror's homing offset and step error. With `--head-errors` the head itself is off its CAD numbers as a printed, hand-built one is (headcal's synthetic head: the head turned on the servo horn, the pose camera and the objective off their seats, the mirror's shaft tilted and its home wrong, the scan line a few percent long, some distortion along the slit), and lines.csv still logs the CAD head, as before a hand-eye calibration. | `arm.py`, `ros2/so101_scan_description`, `calibration/headcal` |
| Pose camera | The Pi NoIR camera v2 beside the spectrograph, as `headcal record` keeps its stills while the arm holds a viewpoint: grey, each 2 × 2 colour cell averaged, 1632 × 1232. Every pixel is 2 × 2 rays through a lens with real distortion (headcal's synthetic Pi camera lens) into the scene, so stills have its depth, occlusion and shadows; a pixel's grey is what the sensor's colour cells see of the material under the scan's light, with cos⁴ fall-off, a little lens blur, shot and read noise. | `posecam.py`, `calibration/headcal` |
| Timing | The rig's two clocks, run with its own code. The camera free-runs at 30 fps with the IMX219's rolling shutter: each row stops exposing 18.9 µs after the one before (in the 1640 × 1232 mode), so the slit's rows expose over a window 21 ms longer than the exposure. The bridge locks each sweep to those frames as `frame_lock.py` plans it, the mirror moves at the ESP32's start speed from each tick and settles for 3 ms, and `line_camera`'s matcher keeps the frames whose window falls inside the mirror's hold. The camera's stamps can be given an error; rows that then expose while the mirror moves see each angle along its path for as long as it was there. | `timing.py`, `ros2/so101_scan_sweep`, `ros2/so101_scan_camera` |

The default scene, **relief**, is a 60 × 60 mm plate with a 5 × 5 block of 8 mm pillars from 1 to 12 mm tall, their tops in different paints, papers, checkers, random dots, the rare-earth tile, PTFE and a word hidden in the near infrared. It is the textured relief the [splat README](../splat/README.md#training) suggests for pinning down sweep poses, which a flat board leaves loose, and it is simple to 3D print and paint. **board** is a flat target like the splat's synthetic scene, **white** is a PTFE sheet, and **tagboard** is the hand-eye calibration's ChArUco board (headcal's own definition, drawn as OpenCV draws it) laser-printed on Letter paper on card, lying near the target and turned a little, as a person would put it down.

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
├── pose/              with --pose-camera: the pose camera's stills as `headcal record` keeps
│                      them: camera.json ("headcal pose frames v1"), frames.csv (index,
│                      stamp_ns, file, exposure_us, gain) and still_0000.npy ..., uint16 grey
│                      with black level 64, stamped on lines.csv's clock while a sweep runs
├── binned/            after `python -m hsisim bin`: the lines binned with a calibration, as
│                      line_camera writes them when it has one (binning.npz, sweep_001.npy ...)
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
    ├── head.json        with the pose camera or --head-errors: the head as it really is and as
    │                    the CAD has it (headcal's HeadGeometry), the pose camera's lens, the
    │                    tag board's pose; head_calibration.yaml is the true head as headcal
    │                    writes a calibration
    ├── pose_camera.json, stills_true.csv
    │                    the pose camera's true lens and pose in the head, and where it really
    │                    was for each still
    └── scene.json, truth.json
```

Conventions, the same as the rig's:

- Sweeps are numbered from 1, as the mirror bridge numbers them, and frames by the line's index in its sweep.
- The slit position h runs from −1 to +1 along the line camera's +x axis, and on the sensor the slit row grows with h. So the first rectified slit row is the camera's −x end, which is pixel 0 of the splat's dataset. With `--slit-reversed` the camera is mounted the other way: the frames (and the calibration session's) come out mirrored along the slit, `hsical calibrate` keeps the rows as they are, and camera.json's `slit_reversed: true` tells `scan_to_dataset` to flip them, so the dataset comes out the same.
- Frames are as the camera gives them, so `hsical apply` works on a sweep folder unchanged.
- Times are as the rig logs them. In lines.csv a line's stamp is when the mirror settled (line 0 of a sweep is already there at its tick) and its hold ends 50 µs before the next tick. In frames.csv a frame's stamp is its row 0's read-out plus `stamp_offset_us`, and its window runs from the first slit row's exposure start to the last one's read-out, as `SensorTiming` works it out. The line period is a whole number of frames, enough for one window plus the mirror's move and margins: at a 20 ms exposure, two frames (66.7 ms) a line where the plan asks for one.

`hsisim.truth.ScanTruth` reads the truth back: `ScanTruth("sim_out").reflectance(line, nm)` is what each slit position of a line really saw.

Sessions come out raw, as from a rig scanning without a calibration, because the simulated instrument is calibrated afterwards. `python -m hsisim bin SESSION CAL` then bins the lines with `so101_scan_camera`'s own `LineBinner`, as `line_camera` would have with that calibration, into binned/, and fills in camera.json's calibration and binning. `scan_to_dataset` doesn't need it (it bins raw frames itself, given `--calibration`); `headcal solve` does.

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

## A hand-eye calibration scan

The [hand-eye calibration](../calibration/headcal/) finds where the scanner head really sits on the wrist, and where the pose camera, the objective and the mirror sit in it, from a scan of its tag board and the pose camera's stills. The simulator renders that scan for a head that is off its CAD numbers, as the rig would take it, so a calibration can be checked against the true head:

```bash
python -m hsisim scan he --plan headcal --scene tagboard --pose-camera --head-errors 1   # 22 viewpoints, slow
cd ../calibration
python -m hsical calibrate ../sim/he/calibration -o ../sim/he/cal
cd ../sim
python -m hsisim bin he he/cal                     # the binned lines line_camera would have written
cd ../calibration
python -m headcal solve ../sim/he ../sim/he/pose -o ../sim/he/headcal
```

`python -m hsisim handeye` does all of that on every other viewpoint of the plan, with sweeps of 54 lines four steps apart, and compares the head headcal finds with the true one (truth/head.json) in headcal's own terms: about five minutes. Seed 7:

| Against the true head | Perfect arm (`--errors 0`) | The simulator's arm |
|---|---|---|
| Line camera's rays on the board, median: headcal (CAD head) | 0.03 mm (4.5 mm) | 1.9 mm (4.5 mm) |
| Pose camera on the wrist: headcal (CAD head) | 0.01 mm, 0.005° (2.3 mm, 3.2°) | 1.4 mm, 0.4° (2.3 mm, 3.2°) |
| Pose camera lens: focal length, centre | 0.12 px, 0.10 px | 0.24 px, 0.10 px |
| Scan line posed from the logged arm, median: headcal (CAD head; true head) | 0.02 mm (4.5 mm; 0) | 1.0 mm (3.3 mm; 1.6 mm) |

With a perfect arm headcal finds the head as well as on its own synthetic scans. With the simulator's arm (servo offsets of 0.25°, whole encoder ticks, and 0.5 mm and 0.2° of flex at each viewpoint) it can't tell the servos' constant offsets from the head's place on the wrist, so it puts them into the mount: the head it finds is further from the true one, but the scan line posed from what the rig logs lands nearer where it really was (1.0 mm) than it would through the true head (1.6 mm). What is left is the flex, different at every viewpoint, which the splat trainer's pose correction per sweep is there for. The last row is the error `scan_to_dataset --urdf` hands the trainer.

## Options

```bash
python -m hsisim scan --help
```

- `--plan ring`, any plan file from `make_plan`, or `headcal` (the hand-eye calibration's plan, calibration/headcal/plans); `--views down,tilt25_az90` keeps only some viewpoints, and `--lines`, `--steps-per-line`, `--start-angle-deg` and `--line-period-s` override the sweep.
- `--scene relief|board|white|tagboard` and `--lighting uniform|lamp`.
- `--binning 4` (the default), 2, or 1 for the full 3280 × 2464 sensor. At binning 4 a line takes about 0.17 s and 1 MB on disk. At binning 1 it takes about 6 s and 16 MB, with 1.4 GB of memory, so a full ring plan is most of an hour and 7 GB.
- `--seed`: the instrument's flaws, the pose errors and the noise all follow it; the same seed gives the same lines.csv and frames.
- `--errors 0` for perfect poses (the log is then the truth), or 2 for twice the default errors.
- `--exposure-ms`, `--gain`, `--slices` and `--rays` (rays per slit point).
- `--fps` (30 by default, 21 at binning 1, as the IMX219's modes allow); an exposure must fit in one frame.
- `--stamp-error-us 15000` makes the camera's stamps 15 ms late, as a driver whose timestamp isn't row 0's read-out would. The bridge locks to the stamps, so the frames really expose earlier, and once the error is more than the lock's slack can take (9.3 ms late in the self-test's sweeps of 16 steps a line, 13.6 ms in the ring plan's 2-step lines) the first slit rows catch the end of the mirror's move. `--stamp-offset-us -15000` is `line_camera`'s correction and gives back the exact session. truth/frames_true.csv says how much of each line moved.
- `--slit-reversed`: the camera mounted the other way along the slit.
- `--pose-camera` also renders the pose camera's stills into pose/, `--pose-stills` a sweep (1 by default; the arm holds still through a sweep, so more only adds noise samples), at `--pose-exposure-ms` (20, with the exposure level set as `record --auto 0.7` would). A still takes about 5 s. Needs OpenCV.
- `--head-errors 1` puts the head off its CAD numbers by as much as a hand-built one is likely to be (2 for twice that). The frames and stills come from the true head, lines.csv from the CAD one, and truth/head_calibration.yaml has the true head. Needs OpenCV.

`python -m hsisim bin SESSION CAL [--slit-bins 256]` bins a session's lines with a calibration, as `line_camera` does when it has one.

In Python, `hsisim.session.simulate(out, plan, Settings(...))` takes the same settings, and `Settings.truth` overrides any of hsical's instrument flaws, for example `{"blur_px": 12.0}` for a soft camera lens.

## Limits

- **Matte surfaces only**: no specular highlights, no light bouncing between surfaces, no fluorescence.
- **Ideal objective and mirror**: a thin lens with a Gaussian blur, a flat mirror that reflects every wavelength equally. Stray light and ghosts aren't modelled.
- **Only the mirror moves, and cleanly**: the arm holds still through a sweep, the mirror moves at a steady speed and is still the moment it gets there (the 3 ms settle is margin, not ringing), and the camera's and the ESP32's clocks run at the same rate and agree but for the stamps' error.
- **The pose camera takes stills, not video**: a still is rendered when the recorder would keep one, with the arm holding a viewpoint, so there are none from while the arm moves and no motion blur or rolling shutter in them. The lens is a pinhole with OpenCV's distortion, focused everywhere but for a slight blur.
- **No drift**: the instrument stays as calibrated, with no temperature changes between the calibration session and the scan.

## Code map

```
hsisim/
├── __main__.py     command line: scan, selftest, evaluate, bin, preview
├── repo.py         imports calibration/hsical, headcal and the ROS 2 packages' modules; builds robot.urdf
├── spectra.py      the spectral library and preview colours
├── scene.py        solids, textures, lighting, ray casting
├── scenes.py       relief, board, white, tagboard
├── linecam.py      the objective and the rays one scan line traces
├── instrument.py   hsical's synthetic spectrograph, rendering scene light into raw frames
├── arm.py          true and logged poses
├── session.py      a whole scan session on disk
├── timing.py       the camera's and the mirror's clocks: the frame lock, rolling shutter, stamps
├── posecam.py      the pose camera and its stills
├── binned.py       binned/, as line_camera writes it with a calibration
├── handeye.py      headcal's calibration scan, solved and compared with the true head
├── truth.py        reading the truth back
├── evaluate.py     the end-to-end check
└── preview.py      pictures of a session
tests/              53 tests: rays, scenes, rendering, the session format, timing, the pose camera, end to end
tools/              readme_figure.py draws img/simulator.png
```
