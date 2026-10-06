# CAD: the SO-101 with the scanner head

<p align="center">
  <img src="renders/rig.png" width="100%" alt="Render of the SO-101 arm holding the black scanner head over a table, with a red fan of light from the scan window drawing a line on the table.">
</p>

A parametric FreeCAD model of the whole rig: the SO-101 follower arm, the spectrograph head with every bought part in place, the 3D-printed housing that holds them on the optical axis of [layout C](../optics/spectrograph_model.py), and a puck that bolts the head to the wrist-roll servo where the gripper used to be. One script builds it from a spreadsheet of numbers, checks it, and exports the STLs to print.

<table>
  <tr>
    <td width="50%"><img src="renders/head.png" width="100%" alt="The head from below: scan window with its hood, pose camera board, stepper on the lid, and the round wrist puck."></td>
    <td width="50%"><img src="renders/head_open.png" width="100%" alt="The head with the lid and stepper hidden and the parts labelled: scan mirror on its clamp, objective, slit and field lens, collimator, grating, tilted camera lens and the IMX219 board."></td>
  </tr>
  <tr>
    <td><sub>From below: scan window and hood, pose camera, stepper on the lid, wrist puck.</sub></td>
    <td><sub>Lid and stepper hidden.</sub></td>
  </tr>
</table>

## At a glance

- **Head:** 188 g and 66 × 53 × 135 mm, wrist puck included. The NEMA 8 stepper is 60 g of that.
- **Printed:** eight parts on the head, about 65 g of PETG, plus an optional bench mount.
- **Checks:** no parts overlap. The mirror turns a full circle with at least 0.5 mm to spare. The head clears the wrist at every roll angle (1.4 mm closest) and through ±95° of wrist flex.
- **Watch out:** with the arm stretched out level, the shoulder servo has to hold 1.21 N·m, which is 75% of its stall torque. The stock gripper needs 0.86 N·m there. The scanning pose in the render needs only 0.30 N·m. See [Arm load](#arm-load).

## Files

```
cad/
├── so101_hsi_rig.FCStd     the assembly, driven by its Params spreadsheet
├── build_rig.py            builds the .FCStd from scratch, runs the checks, writes everything below
├── build_report.json       mass budget, servo torques, clearances, optical stations
├── stl/                    printed parts, already turned to their print orientation
├── step/hsi_head.step      the head with every part, for other CAD tools
├── optics_check.py         traces the as-built spacings with the Optiland model
├── optics_as_built.txt     its output
├── renders/                the images on this page
├── render/                 Blender render script and the labelling step
├── rig/                    the model: parameters, printed parts, bought parts, arm, statics
└── vendor/so101/           SO-101 URDF and a script that downloads its meshes
```

## How the head works

The head has its own frame: the origin is on the face of the wrist-roll horn, +Z points out along the roll axis, +Y faces the scene, and X runs along the slit and the mirror shaft.

**Light path.** Light comes in through the scan window on the +Y face and hits the mirror at 45°. It then goes through the filter cap (510 nm long-pass and the 4 mm stop), the objective, the slit with the field lens right behind it, the reversed collimator and the grating film. The camera lens, tilted 22°, focuses it onto the IMX219.

**Mount.** The wrist puck replaces the gripper on the wrist-roll horn. A dovetail rail on the puck runs along X, and the slot in the housing floor is open on the −X side, so the housing slides onto the rail from there until it stops against the closed end. The lid closes the −X side and the end of the slot with it, so the lid screws are what hold the head on.

**Optics carriers.** The objective carrier, slit block and grating carrier are plates that slide into rib slots from the open side. That lets you build and align the optics before the lid goes on. The bench puck has the same rail on a 50 × 40 mm plate with a 1/4"-20 nut, so the head can sit on a tripod or the optics table for the bench steps in the [build order](../docs/parts_list_and_design.md#4-build-order).

**Scan mirror.** A printed clamp grips the stepper's 4 mm shaft with an M3 pinch screw and holds the mirror on a flat pad parallel to the shaft. A 5 × 2 mm magnet in the clamp's tab passes an A3144 hall sensor in the +X wall when the mirror is turned 40° back from its 45° working angle, which gives each scan a home position.

**Camera.** The B0152 board sits outside the end wall on four standoffs. The CIL122 goes in the board's own M12 holder and looks through a 20 mm hole in the wall. Turn the board so the sensor's long side runs across the slit, since that is the axis the spectrum spreads along. Black tape around the board edge keeps light out of the end.

**Pose camera.** The Pi NoIR v2 sits on four standoffs on the +Y face next to the scan window, looking at the scene.

## Parts

### Printed

Print the housing, lid and carriers in black PETG. PETG holds up to the stepper's heat, which PLA may not, and black keeps stray light down. The STLs are already turned to the orientation below.

| Part | File | How to print | Mass |
|---|---|---|---|
| Housing | [`housing.stl`](stl/housing.stl) | Floor (dovetail side) on the bed, camera end up. 0.2 mm layers, 3 perimeters, 20% gyroid, supports only under the camera flange | 36 g |
| Lid | [`lid.stl`](stl/lid.stl) | Inner (flat) face on the bed. 4 perimeters around the motor holes | 13 g |
| Mirror clamp | [`mirror_clamp.stl`](stl/mirror_clamp.stl) | Mirror pad face down. 0.15 mm layers, 100% infill | 2 g |
| Wrist puck | [`wrist_puck.stl`](stl/wrist_puck.stl) | Horn side down (it bridges the 20 mm horn recess). 100% infill | 5 g |
| Objective carrier | [`objective_carrier.stl`](stl/objective_carrier.stl) | Flat | 2 g |
| Slit block | [`slit_block.stl`](stl/slit_block.stl) | Back face down so the blade recess prints flat. 0.12 mm layers | 4 g |
| Grating carrier | [`grating_carrier.stl`](stl/grating_carrier.stl) | Flat, film recess up | 2 g |
| Filter cap | [`filter_cap.stl`](stl/filter_cap.stl) | Stop face down. Check that the sleeve is a snug push fit on the objective | 1 g |
| Bench puck (optional) | [`bench_puck.stl`](stl/bench_puck.stl) | Flat side down. A 1/4"-20 hex nut presses in from below | 19 g |

### Bought

| Part | In the model |
|---|---|
| Objective: Commonlands CIL161, 16 mm, stopped to f/4 | barrel size estimated |
| Collimator: Arducam LN016, 25 mm, used backwards | size from the listing, back focus estimated |
| Camera lens: Commonlands CIL122, 12 mm f/2 | barrel size estimated |
| Camera: Arducam B0152 IMX219 NoIR, M12 mount | 36 × 36 mm from the listing; holes, thickness and holder estimated |
| 2× M12 holder, UCTRONICS U0756M10 (objective, collimator) | listing |
| Field lens: 12.7 mm plano-convex, f = 15 mm | datasheet |
| Slit: two 20 mm pieces of a 9 mm snap-off blade, set 50 µm apart with a feeler gauge | single-edge blades (38 × 19 mm) don't fit inside the head |
| Grating: 15 × 15 mm piece of 500 l/mm film, grooves along X | |
| 510 nm long-pass filter, 17 mm disc | listing |
| Scan mirror: 25 × 20 × 2 mm first-surface | listing |
| Stepper: StepperOnline 8HS11-0204S, NEMA 8, 1.8°, 0.2 A | size, shaft and holes from the listing; boss and thread depth estimated |
| A3144 hall sensor and a 5 × 2 mm magnet | |
| Pose camera: Pi NoIR v2 (owned) | Raspberry Pi drawing |

### Hardware

| Qty | Part | Where |
|---|---|---|
| 4 | M3 × 6 | puck to the servo horn |
| 4 | M2 × 4 | stepper to the lid, from inside (the heads sit flush) |
| 2 + 2 + 2 | M3 × 35, M3 washers and M3 nuts | from the counterbores on the +X face, through the floor and the lid, nutted on the lid's motor pad |
| 2 + 2 | M3 × 8 and M3 heat-set inserts | lid ears to the housing |
| 1 + 1 | M3 × 8 and M3 nut | mirror clamp pinch screw |
| 8 + 8 | M2 × 5 and M2 heat-set inserts | camera board (4) and pose camera (4) |
| 2 | M2 × 5 | objective's M12 holder to its carrier, self-tapping into the printed pilot holes |
| 2 | M2 × 6 | collimator's M12 holder to the back of the slit block, self-tapping |

Plus CA glue for the magnet, mirror and slit blades, and Kapton tape for the grating film. The [NEMA 17 build](#the-nema-17-option) holds its motor with two M3 × 6 instead, and the long bolts thread into the motor rather than into nuts.

## Measure these first

These numbers came from estimates, not datasheets. Check them when the parts arrive, before printing the part they affect. All of them are cells in the Params sheet, marked `est`.

| Value | Assumed | Affects |
|---|---|---|
| B0152 hole spacing (`brd_hole`) | 29 mm square | camera standoffs on the housing |
| B0152 lens holder size (`brd_holder_h`, `brd_holder_w`) | 10 mm tall, 14 mm wide | camera opening in the end wall |
| CIL161 barrel (`obj_od`, `obj_len`) | Ø14 × 18.5 mm | filter cap fit, objective position |
| CIL122 barrel (`cam_od`, `cam_len`) | Ø15 × 19 mm | how close the camera lens can get to the grating |
| Lens back focal lengths (`*_bfl`) | 5–6 mm | carrier positions. Turning a lens in its M12 thread takes up a millimetre or two |
| M12 holder ear thickness (`h12_ear_t`) | 2 mm | holder screw length |
| Stepper pilot boss (`mot_boss_d`, `mot_boss_h`) | Ø15 × 1.5 mm | the pocket in the lid it sits in; the motor has to sit flat on the pad |
| Stepper thread depth (`mot_hole_depth`) | 2.5 mm | the M2 × 4 screws reach 2.2 mm into it |

## Assembly

1. Press in the heat-set inserts: two M3 in the lid bosses on the housing, four M2 in the camera standoffs and four M2 in the pose-camera standoffs.
2. Build the optics on their carriers. The objective's holder screws to the objective carrier, and the filter cap pushes onto the front of the lens. In the slit block, the field lens drops into its pocket from the front with its flat side toward the blades, the two blade pieces go in the recess over it with the 0.05 mm feeler blade setting the gap, and the collimator's holder screws to the back. Tape the grating film into its carrier with the grooves along X. Slide the three carriers into their slots.
3. Fit the camera board on the end wall, the pose camera on the +Y face, and the hall sensor in its pocket in the +X wall.
4. Screw the stepper to the lid with the four M2 × 4 from inside. Put the mirror clamp on the shaft with the magnet and mirror glued in.
5. Take the gripper off the wrist-roll horn and screw the puck on. Slide the housing onto the rail, then fit the lid so the mirror goes in through the open side. The two M3 × 35 bolts go in from the +X face and take a washer and nut each on the lid's motor pad, and the two M3 × 8 go through the lid ears.

## Checks

`build_rig.py` runs these every build and writes the results to `build_report.json`.

- **Overlaps:** every pair of head parts, with up to 0.5 mm³ allowed. None found.
- **Mirror sweep:** the clamp, mirror and magnet turned a full circle in 10° steps. The closest approaches are the lid (0.5 mm), the housing (0.9 mm), the hall sensor (1.1 mm), the filter cap (3.4 mm) and the objective (6.1 mm).
- **Wrist roll:** the head against the wrist bracket at every roll angle. The closest point is 1.4 mm, at 12.5 mm from the roll axis.
- **Wrist flex:** −95° to +95° at roll angles every 30°. No contact with the forearm anywhere.

## Arm load

Gravity torque on each joint, from the link masses in the SO-101 URDF, with the stock gripper swapped for the head at the head's centre of mass. The STS3215 stalls at 16.5 kg·cm (1.62 N·m) at 6 V, and the follower arm runs on 5 V, so treat the percentages as optimistic.

| Shoulder / elbow torque, N·m (% of stall) | Head | Scanning pose (render above) | Worst case: arm stretched out level |
|---|---|---|---|
| Stock gripper | | 0.13 / 0.38 | 0.86 (53%) / 0.45 (28%) |
| **Head as modelled (NEMA 8)** | **188 g** | **0.30 (18%) / 0.61 (37%)** | **1.21 (75%) / 0.70 (43%)** |
| NEMA 17 pancake instead (`RIG_MOTOR=17HM08`) | 281 g | 0.41 (25%) / 0.78 (48%) | 1.52 (94%) / 0.91 (56%) |
| No scan motor, the arm sweeps the line | 128 g | 0.22 (14%) / 0.49 (31%) | 1.02 (63%) / 0.57 (35%) |

The 12 V version of the STS3215 (30 kg·cm) would bring the worst case down to 41%, but it means swapping all six servos and the power supply. In practice: keep the arm folded the way the render shows while scanning, and don't park it stretched out level.

### The NEMA 17 option

The head was first drawn around a 150 g NEMA 17 pancake, which pushed the shoulder to 94% of stall, so the default is now the 60 g NEMA 8. The pancake is still a build option. The housing and every optic stay where they are; only the lid, the mirror clamp and the motor change.

| | 8HS11-0204S (default) | 17HM08-1204S |
|---|---|---|
| Motor | NEMA 8, 1.8°, 0.2 A, 60 g | NEMA 17 pancake, 0.9°, 1.2 A, 150 g |
| Head | 188 g | 281 g |
| Motor to lid | 4× M2 × 4 from inside; the long bolts end in washers and nuts on the lid | 2× M3 × 6 from inside; the long bolts thread into its lower holes |
| Microstepping for 0.29 mm lines at 150 mm | 1/32 | 1/16 |

The NEMA 8's 10 mm shaft reaches 5.5 mm into the clamp, with the pinch screw over it. Gravity on the mirror, clamp and magnet puts at most 0.01 N·cm on the shaft, under 1% of the motor's 1.4 N·cm holding torque. Build the pancake version with `RIG_MOTOR=17HM08 freecadcmd cad/build_rig.py`.

## Optics as built

Layout C puts the grating 22 mm behind the collimator and the camera lens 3 mm behind the grating. The camera lens barrel reaches about 12 mm ahead of its principal plane (estimated), so 3 mm would put the grating inside it. The housing keeps the grating square to the beam on its own carrier, right behind the collimator (6 mm), and brings the camera lens as close as its barrel allows (16 mm). `optics_check.py` traces those spacings with the Optiland model ([`optics_as_built.txt`](optics_as_built.txt)):

- The spectrum length (3.27 mm), smile and keystone don't change.
- At the ends of the band (500 and 1000 nm) about 60% of the light gets through in the model, against 100% in layout C. The wavelengths fan out over the longer gap and partly miss the camera lens's 6 mm aperture. The model puts that aperture at the lens's principal plane, though. In a real M12 lens it usually sits a few millimetres behind the front glass, which would put it 4–8 mm from the grating and let 86% or more through. Flat-fielding against the white reference removes whatever loss remains.
- The 15 mm field lens works as well as the model's 17.9 mm one: 98% at the ends of the line.

## Rebuild or change it

You need FreeCAD 1.0 or newer (tested with 1.1.3). From the repo root:

```
python3 cad/vendor/so101/fetch_assets.py     # SO-101 meshes, once (17 MB, checked by SHA-256)
freecadcmd cad/build_rig.py                  # about 2 min: build, checks, STL/STEP, report, .FCStd
```

On Windows, `freecadcmd` is `FreeCADCmd.exe` in FreeCAD's `bin` folder. Running the script with the GUI instead (`freecad cad/build_rig.py`, from the repo root) builds the same thing and keeps the part colours in the saved file.

To change a number, open `so101_hsi_rig.FCStd`, edit the cell in the **Params** sheet and recompute. Everything that depends on it moves. Each value has a note saying what it is and where it came from. The joint angles (`q_*`) pose the arm. The STLs don't update on a recompute, so after a change either export the changed body by hand or make the same edit in [`rig/params.py`](rig/params.py) and rerun the script.

Renders (Blender 4.x and Pillow):

```
RIG_NO_EXPORT=1 RIG_RENDER_DIR=/tmp/rig_meshes freecadcmd cad/build_rig.py
blender -b -P cad/render/render_blender.py -- /tmp/rig_meshes /tmp/rig_renders
python3 cad/render/finish.py /tmp/rig_renders cad/renders
```

Optics check (needs `optiland`, like the model): `python3 cad/optics_check.py`

## Credits

The SO-101 URDF and meshes are from [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) (`Simulation/SO101`), used under the Apache License 2.0; a copy of the license is in [`docs/img/SO-ARM100_LICENSE.txt`](../docs/img/SO-ARM100_LICENSE.txt). The URDF is unmodified, and the .FCStd holds decimated copies of the meshes.
