# CAD: the SO-101 with the scanner head

<p align="center">
  <img src="renders/rig.png" width="100%" alt="Render of the SO-101 arm holding the black scanner head over a table, with a red fan of light from the scan window drawing a line on the table.">
</p>

A parametric FreeCAD model of the whole rig: the SO-101 follower arm, the spectrograph head with every bought part in place, the 3D-printed housing that holds them on the optical axis of [layout E](../optics/spectrograph_model.py) (the optics model of the parts that were bought), and a puck that bolts the head to the wrist-roll servo where the gripper used to be. A printed tray holds the scan-mirror controller board from [`pcb/`](../pcb/README.md) on the table behind the arm. One script builds it from a spreadsheet of numbers, checks it, and exports the STLs to print.

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

- **Head:** 187 g and 64 × 53 × 136 mm, wrist puck included. The NEMA 8 stepper is 60 g of that.
- **Printed:** nine parts on the head, about 68 g of PETG, plus the controller tray (30 g), an optional bench mount and a spare stop washer.
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

**Light path.** Light comes in through the scan window on the +Y face and hits the mirror at 45°. It then goes through the filter cap (a 495 nm long-pass disc, then a thin washer whose 4 mm hole is the stop), the objective, the slit with the field lens right behind it, curved side first, the reversed collimator and the grating film. The camera lens, tilted 22°, focuses it onto the IMX219.

**Mount.** The wrist puck replaces the gripper on the wrist-roll horn. A dovetail rail on the puck runs along X, and the slot in the housing floor is open on the −X side, so the housing slides onto the rail from there until it stops against the closed end. The lid closes the −X side and the end of the slot with it, so the lid screws are what hold the head on.

**Optics carriers.** The objective carrier, slit block and grating carrier are plates that slide into rib slots from the open side. That lets you build and align the optics before the lid goes on. The bench puck has the same rail on a 50 × 40 mm plate with a 1/4"-20 nut, so the head can sit on a tripod or the optics table for the bench steps in the [build order](../docs/parts_list_and_design.md#4-build-order).

**Scan mirror.** A printed clamp grips the stepper's 4 mm shaft with an M3 pinch screw and holds the mirror on a flat pad parallel to the shaft. The pad is drawn for a 20 × 20 mm piece cut from a 3 mm front-surface mirror sheet, and notches in its edges mark where the mirror's ends go so it sits centred on the optical axis. Along the shaft the piece can be anything from 20 to 25 mm. Across the shaft keep it at 20 mm or a little under, because its corners pass 0.9 mm from the housing as it turns. For thinner glass, set `mirror_t` and rebuild, or glue the mirror onto a printed shim that makes up the difference to 3 mm. A 5 × 1.6 mm magnet in the clamp's tab passes an A3144 hall sensor in the +X wall when the mirror is turned 40° back from its 45° working angle, which gives each scan a home position. The sensor's leads leave the bottom of its body, bend out through a slot under its pocket, and pass through the [hall breakout](../pcb/README.md#hall-sensor-breakout), which is screwed flat on the outside with one M2 × 5.

**Camera.** The B0152 board sits outside the end wall on four standoffs. The CIL122 goes in the board's own M12 holder, which reaches through a 21 mm hole in the wall. The hole is sized from `brd_holder_w` so a square holder passes at any angle. Turn the board so the sensor's long side runs across the slit, since that is the axis the spectrum spreads along. Black tape around the board edge keeps light out of the end.

**Pose camera.** The Pi NoIR v2 sits on four standoffs on the +Y face next to the scan window, looking at the scene.

## Parts

### Printed

Print the housing, lid and carriers in black PETG. PETG holds up to the stepper's heat, which PLA may not, and black keeps stray light down. The STLs are already turned to the orientation below.

| Part | File | How to print | Mass |
|---|---|---|---|
| Housing | [`housing.stl`](stl/housing.stl) | Floor (dovetail side) on the bed, camera end up. 0.2 mm layers, 3 perimeters, 20% gyroid, supports only under the camera flange | 37 g |
| Lid | [`lid.stl`](stl/lid.stl) | Inner (flat) face on the bed. 4 perimeters around the motor holes | 13 g |
| Mirror clamp | [`mirror_clamp.stl`](stl/mirror_clamp.stl) | Mirror pad face down. 0.15 mm layers, 100% infill | 2 g |
| Wrist puck | [`wrist_puck.stl`](stl/wrist_puck.stl) | Horn side down (it bridges the 20 mm horn recess). 100% infill | 5 g |
| Objective carrier | [`objective_carrier.stl`](stl/objective_carrier.stl) | Flat | 2 g |
| Slit block | [`slit_block.stl`](stl/slit_block.stl) | Back face down so the blade recess prints flat. 0.12 mm layers | 4 g |
| Grating carrier | [`grating_carrier.stl`](stl/grating_carrier.stl) | Flat, film recess up | 2 g |
| Filter cap | [`filter_cap.stl`](stl/filter_cap.stl) | Lip face down. Check that the sleeve is a snug push fit on the objective | 1 g |
| Stop washer, 4 mm | [`stop_washer.stl`](stl/stop_washer.stl) | Flat, 0.2 mm layers. Clear any stringing from the hole, since its edge is the stop | 0.1 g |
| Stop washer, 5 mm (optional) | [`stop_washer_5mm.stl`](stl/stop_washer_5mm.stl) | As the 4 mm one. Swap it in at first light to compare: see [Optics as built](#optics-as-built) | 0.1 g |
| Bench puck (optional) | [`bench_puck.stl`](stl/bench_puck.stl) | Flat side down. A 1/4"-20 hex nut presses in from below | 19 g |
| Controller tray | [`controller_tray.stl`](stl/controller_tray.stl) | Floor on the bed. 3 perimeters, 20% infill. It stays on the table, so PLA is fine: see [Controller tray](#controller-tray) | 30 g |

### Bought

| Part | In the model |
|---|---|
| Objective: Commonlands CIL161, 15.6 mm, stopped to f/3.9 by the 4 mm washer | barrel size estimated |
| Collimator: Arducam LN016, 25 mm, used backwards | size from the listing, back focus estimated |
| Camera lens: Commonlands CIL122, 12 mm f/2 | barrel size estimated |
| Camera: Arducam B0152 IMX219 NoIR, M12 mount | 36 × 36 mm from the listing; holes, thickness and holder estimated |
| 2× M12 holder, uxcell, 20 mm screw spacing (objective, collimator), each cut down to 10 mm: see [Assembly](#assembly) | 24 × 17 × 14 mm from the listing; ear thickness estimated |
| Field lens: Edmund #49-840, 12.7 mm plano-convex, f = 15 mm, uncoated | datasheet |
| Slit: two 20 mm pieces of a 9 mm snap-off blade, set 50 µm apart with a feeler gauge | single-edge blades (38 × 19 mm) don't fit inside the head |
| Grating: 15 × 15 mm piece of 500 l/mm film, grooves along X | |
| Long-pass filter: Edmund #54-652, SCHOTT GG-495, Ø12.5 × 3 mm | listing. The cap's seat takes the ±0.38 mm diameter tolerance |
| Scan mirror: 20 × 20 mm piece cut from a RUEHALF 100 × 100 × 3 mm front-surface mirror | thickness from the listing |
| Stepper: StepperOnline 8HS11-0204S, NEMA 8, 1.8°, 0.2 A | size, shaft and holes from the listing; boss and thread depth estimated |
| A3144 hall sensor and a 5 × 1.6 mm magnet, magnetized through its thickness | measured |
| Hall breakout from [`pcb/`](../pcb/README.md#hall-sensor-breakout), 14 × 12.4 × 0.8 mm, with its 3-wire cable soldered in | outline, holes and lead positions from `pcb/` |
| Pose camera: Pi NoIR v2 (owned) | Raspberry Pi drawing |

### Hardware

| Qty | Part | Where |
|---|---|---|
| 4 | M3 × 6 | puck to the servo horn |
| 4 | M2 × 4 | stepper to the lid, from inside (the heads sit flush) |
| 2 + 2 + 2 | M3 × 35, M3 washers and M3 nuts | from the counterbores on the +X face, through the floor and the lid, nutted on the lid's motor pad |
| 2 + 2 | M3 × 8 and M3 × 6 heat-set inserts (Pofsnnx: 5.0 mm knurl, 4.5 mm holes) | lid ears to the housing |
| 1 + 1 | M3 × 8 and M3 nut | mirror clamp pinch screw |
| 8 + 8 | M2 × 5 and M2 × 4 heat-set inserts (Pofsnnx: 3.0 mm knurl, 2.85 mm holes) | camera board (4) and pose camera (4) |
| 2 | M2 × 5 | objective's M12 holder to its carrier, self-tapping into the printed pilot holes |
| 2 | M2 × 5 | collimator's M12 holder to the back of the slit block, self-tapping (3 mm of thread in the block) |
| 1 | M2 × 5 | hall breakout to the +X wall, self-tapping (2.5 mm of thread in the wall) |
| 4 + 4 | M3 × 6 and M3 heat-set inserts | controller board to its tray |
| 2 + 2 | 1/4"-20 or M6 screws, to suit the table, and washers | tray to the optical table |

Plus CA glue for the magnet, mirror and slit blades, and Kapton tape for the grating film. The [NEMA 17 build](#the-nema-17-option) holds its motor with two M3 × 6 instead, and the long bolts thread into the motor rather than into nuts.

## Measure these first

These numbers came from estimates or listings, not datasheets. Check them when the parts arrive, before printing the part they affect. All of them are cells in the Params sheet; the estimates are marked `est`.

| Value | Assumed | Affects |
|---|---|---|
| B0152 hole spacing (`brd_hole` along X, `brd_hole_y` along the dispersion) | 29 mm square | camera standoffs on the housing |
| B0152 lens holder size (`brd_holder_h`, `brd_holder_w`) | 10 mm tall, 14 mm square | camera opening in the end wall |
| B0152 sensor above the PCB (`sensor_above_pcb`) and PCB thickness (`brd_t`) | 1 mm, 1.6 mm | where the end wall sits so the sensor is in focus; camera screw length |
| CIL161 barrel (`obj_od`, `obj_len`), on the M12ANIR version | Ø14 × 18.5 mm | filter cap fit, objective position, and where the stop washer sits: 8.5 mm ahead of the lens's principal plane as drawn, 8.1 mm in layout E |
| CIL122 barrel (`cam_od`, `cam_len`), on the M12ANIR version | Ø15 × 19 mm | how close the camera lens can get to the grating |
| LN016 length (`coll_len`) | 21 mm | grating carrier position |
| Lens back focal lengths (`*_bfl`) | 5–6 mm | carrier positions. Turning a lens in its M12 thread takes up a millimetre or two |
| LN016 back focus (`coll_bfl`): rear of the barrel to the sharp image of a distant lamp | 6.0 mm; a sibling Arducam 25 mm lens lists 5.2 | slit block thickness, which puts the collimator where it focuses on the slit |
| CIL122 aperture: its spec table says f/2.0, the text on the same page f/2.4 | f/2.0, a 6 mm pupil | not the CAD. At f/2.4 (5 mm) the ends of the band lose a little more light; set `d_cam` to 5 in [`spectrograph_model.py`](../optics/spectrograph_model.py) and rerun `optics_check.py` to see how much |
| Lens thread lengths (`obj_thread`, `coll_thread`): rear end to where the barrel widens | 9 mm | where to cut the M12 holders, see [Assembly](#assembly) |
| M12 holder ear thickness (`h12_ear_t`) | 2 mm | holder screw length: M2 × 5 leaves 3 mm of thread in the carrier |
| Snap-off blade thickness (`blade_t`) | 0.38 mm | depth of the blade recess in the slit block |
| Stepper pilot boss (`mot_boss_d`, `mot_boss_h`) | Ø15 × 1.5 mm | the pocket in the lid it sits in; the motor has to sit flat on the pad |
| Stepper thread depth (`mot_hole_depth`) | 2.5 mm | the M2 × 4 screws reach 2.2 mm into it. M2 × 5 reach 3.2 mm, so use them only if the holes are that deep, or file 1 mm off them |

## Assembly

1. Cut two of the uxcell M12 holders down to 10 mm, the lens thread plus 1 mm. Cut from the end the lens screws into, not the ear end, and square the cut on sandpaper. A lens screwed in until its barrel stops against the cut end then has its rear end 1 mm inside the holder, where the model puts it. If a lens's thread isn't 9 mm, cut its holder to the thread plus 1 mm; with 13 mm of thread or more, leave the holder whole. Then score and snap a 20 × 20 mm piece from the mirror sheet, scoring the bare back so the coated face isn't scratched.
2. Press in the heat-set inserts: two M3 in the lid bosses on the housing, four M2 in the camera standoffs and four M2 in the pose-camera standoffs.
3. Build the optics on their carriers. The objective's holder screws to the objective carrier. Drop the long-pass disc into the filter cap and the 4 mm stop washer in on top of it, then push the cap onto the front of the lens until the lens presses them together. In the slit block, the field lens drops into its pocket from the front, curved side up toward the blades, and its flat face sits on the ledge at the bottom of the pocket (a dot of glue on its rim stops it rattling). Turned the other way it would make the slit look 1.35× as wide, and the resolution would drop from 4.1 to 5.4 nm. The two blade pieces go in the recess over it with the 0.05 mm feeler blade setting the gap, and the collimator's holder screws to the back. Tape the grating film into its carrier with the grooves along X. Slide the three carriers into their slots.
4. Fit the camera board on the end wall and the pose camera on the +Y face. Then the hall sensor. Solder the 3-wire cable into the breakout's S, G and + pads from the front, and pass a thread through the two tie holes below them so it runs across the back of the board. Bend the A3144's three leads 90° toward its back, away from the printed face, about 0.5 mm below the body. Push it into its pocket in the +X wall from inside, printed face toward the magnet, so the leads go out through the slot under the pocket and the body sits on the ledges either side of it. Feed the leads through the breakout's three holes from behind; seen from outside, VCC is on the right, at the square pad. Press the board flat on the wall and screw it on with an M2 × 5 through its M2 hole, then solder and trim the leads and tie the thread around the cable. A 0.8 mm recess behind the board's lower half leaves room for the thread and for solder that shows through the pads. The screw's tip comes 1.7 mm through the wall and passes about 1 mm from the mirror clamp as it turns, so don't use a longer one.
5. Screw the stepper to the lid with the four M2 × 4 from inside. The A3144 switches on one pole only, so pass each face of the magnet over the powered sensor first and glue it into the clamp's tab with the face that switches it facing out. Glue the mirror onto the pad between the notches, then put the clamp on the shaft.
6. Take the gripper off the wrist-roll horn and screw the puck on. Slide the housing onto the rail, then fit the lid so the mirror goes in through the open side. The two M3 × 35 bolts go in from the +X face and take a washer and nut each on the lid's motor pad, and the two M3 × 8 go through the lid ears.

## Controller tray

The [scan-mirror controller](../pcb/README.md#scan-mirror-controller) sits on its tray on the table behind the arm, 4 mm behind the Waveshare plate on the back of the base. The board's hall connector faces the arm and its USB edge faces away, toward the Jetson. Four posts with M3 heat-set inserts hold the board 6 mm up, which clears the leads underneath (3 mm at most), and the floor keeps them off a metal table. The walls stop below the board, so the jack, the screw terminal and the USB plug on its edges clear them. The strip behind the board has two slots for 1/4"-20 or M6 table screws. Each slot is a full hole pitch long, so on a 1 in or 25 mm grid there is a hole under both slots wherever the tray sits along them.

Press the four inserts into the posts, screw the board down with four M3 × 6, then screw the tray to the table. `tray_x` in the Params sheet moves it.

The arm reaches the tray only when it leans back over its base, which a scan of something in front of it never does. In the scanning pose it stays 62 mm away.

## Checks

`build_rig.py` runs these every build and writes the results to `build_report.json`.

- **Overlaps:** every pair of head parts, with up to 0.5 mm³ allowed. None found.
- **Mirror sweep:** the clamp, mirror and magnet turned a full circle in 10° steps. The closest approaches are the lid (0.5 mm), the housing (0.9 mm), the hall sensor (1.1 mm), the tip of the hall breakout's screw (1.3 mm; it is drawn at the thread's root, so about 1 mm to the thread), the filter cap (1.4 mm) and the objective (5.9 mm).
- **Wrist roll:** the head against the wrist bracket at every roll angle. The closest point is 1.4 mm, at 12.5 mm from the roll axis.
- **Wrist flex:** −95° to +95° at roll angles every 30°. No contact with the forearm anywhere.

## Arm load

Gravity torque on each joint, from the link masses in the SO-101 URDF, with the stock gripper swapped for the head at the head's centre of mass. The STS3215 stalls at 16.5 kg·cm (1.62 N·m) at 6 V, and the follower arm runs on 5 V, so treat the percentages as optimistic.

| Shoulder / elbow torque, N·m (% of stall) | Head | Scanning pose (render above) | Worst case: arm stretched out level |
|---|---|---|---|
| Stock gripper | | 0.13 / 0.38 | 0.86 (53%) / 0.45 (28%) |
| **Head as modelled (NEMA 8)** | **187 g** | **0.30 (18%) / 0.61 (37%)** | **1.21 (75%) / 0.70 (43%)** |
| NEMA 17 pancake instead (`RIG_MOTOR=17HM08`) | 280 g | 0.41 (25%) / 0.78 (48%) | 1.52 (94%) / 0.91 (56%) |
| No scan motor, the arm sweeps the line | 127 g | 0.22 (14%) / 0.49 (31%) | 1.01 (63%) / 0.57 (35%) |

The 12 V version of the STS3215 (30 kg·cm) would bring the worst case down to 41%, but it means swapping all six servos and the power supply. In practice: keep the arm folded the way the render shows while scanning, and don't park it stretched out level.

### The NEMA 17 option

The head was first drawn around a 150 g NEMA 17 pancake, which pushed the shoulder to 94% of stall, so the default is now the 60 g NEMA 8. The pancake is still a build option. The housing and every optic stay where they are; only the lid, the mirror clamp and the motor change.

| | 8HS11-0204S (default) | 17HM08-1204S |
|---|---|---|
| Motor | NEMA 8, 1.8°, 0.2 A, 60 g | NEMA 17 pancake, 0.9°, 1.2 A, 150 g |
| Head | 187 g | 280 g |
| Motor to lid | 4× M2 × 4 from inside; the long bolts end in washers and nuts on the lid | 2× M3 × 6 from inside; the long bolts thread into its lower holes |
| Microstepping for 0.29 mm lines at 150 mm | 1/32 | 1/16 |

The NEMA 8's 10 mm shaft reaches 5.5 mm into the clamp, with the pinch screw over it. Gravity on the mirror, clamp and magnet puts at most 0.014 N·cm on the shaft, about 1% of the motor's 1.6 N·cm holding torque. Build the pancake version with `RIG_MOTOR=17HM08 freecadcmd cad/build_rig.py`.

## Optics as built

The head holds the optics where [layout E](../optics/spectrograph_model.py) puts them. Layout E is the optics model of the parts that were bought:

- The field lens sits curved side toward the slit, and the collimator 26.76 mm behind the slit, where the slit's image through the field lens is.
- The stop is a 0.6 mm washer with a 4 mm hole on the objective's front face, with the long-pass disc in front of it.
- The grating sits right behind the collimator (6 mm), and the camera lens as close behind the grating as its barrel allows (16 mm).

`optics_check.py` traces the spacings the CAD builds with the Optiland model and prints them next to layout E ([`optics_as_built.txt`](optics_as_built.txt)):

- **Resolution:** 4.1 nm, as in layout E.
- **Spectrum and line:** the spectrum is 3.27 mm long, and the 43 mm line at 150 mm fits on the sensor.
- **Stop position:** the washer ends up 8.5 mm ahead of the objective's principal plane rather than 8.1 mm, because the lens barrel is an estimate. That costs a little light at the ends of the line: 52% gets through at 750 nm instead of 54%.
- **Light at the band ends:** at 500 and 1000 nm about 55% of the light reaches the sensor at the centre of the line, and a third at its ends. The wavelengths fan out over the 16 mm to the camera lens and partly miss its 6 mm aperture.
  - The model puts that aperture at the lens's principal plane. In a real M12 lens it usually sits a few millimetres behind the front glass, 4–8 mm from the grating, which would let 80–95% through at the centre.
  - Flat-fielding against the white reference removes whatever loss remains.

The 5 mm washer is worth printing to compare at first light. In the model it gets 14% more signal to the sensor at the centre of the line and 40% more at its ends, at the same 4.1 nm. But more light also gets through the slit and misses the sensor, and that stray light raises the floor under every spectral line. Compare the two on a neon frame.

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
