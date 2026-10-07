# Line-scan hyperspectral splatting: parts list and first-order design

Status: design draft, 2026-10-05. Model: `optics/spectrograph_model.py` (Optiland 0.6, ideal thin lenses). Results: `optics/model_output/results.txt` and the layout PNGs. Diagram: `docs/optical_train_v2.svg`.

This is the full bill of materials for building the rig from scratch. Every part has a row, including the computer, robot arm and electronics. Where a part has a recommended pick and a cheaper budget pick, both are listed. Skip any row you already have.

## 1. Design findings from the optical model

The obvious cheap build puts a slit, collimator and grating in front of a stock Raspberry Pi NoIR camera. The Optiland model shows that this doesn't work, and it points to three design rules.

1. **Don't use a camera with a tiny fixed lens.** The Pi NoIR v2's stock lens has a 1.5 mm entrance pupil, while the collimated beam from a 25 mm collimator at f/4 is about 6 mm wide. Only 8% of the light reaches the sensor at the center of the line. At the ends of the line, the light walks off the pupil entirely and 0% gets through. Use an IMX219 NoIR board with an **M12 lens mount** and a 12 mm f/2 lens, which has a 6 mm pupil. It is the same sensor and driver, so it still plugs into a Raspberry Pi or Jetson CSI port.
2. **Put a field lens at the slit.** Without one, the bundles from the ends of the slit leave the collimator at an angle and miss the camera lens, so the line ends get 12–14% of the light. A small lens right behind the slit, with a focal length close to the objective-to-slit distance (17.9 mm), makes the chief rays parallel. Every point on the slit then lands on the same patch of the camera lens, and the line ends get 98–100%. The 15 mm Edmund lens in row 6 does as well in the model: 98% at the line ends.
3. **Plan to calibrate smile and keystone.** Expect about 32 px of smile, meaning one wavelength traces a curve across the sensor rather than a straight row. Conical diffraction at the grating causes it, and every flat transmission grating has it. Expect about 11 px of keystone, meaning the line length changes slightly with wavelength. You calibrate both once with lamp lines, then remap every frame with a warp. Writing that warp as a CUDA kernel makes a good first CUDA exercise.

| Config (ideal lenses) | Light at line center | Light at line ends | Dispersion | Spectral res. | Pixels along line |
|---|---|---|---|---|---|
| A: stock Pi NoIR v2 lens | 8% | 0% | 1.5 px/nm | 3.7 nm | 543 |
| B: M12 IMX219 + 12 mm lens | 92–100% | 12–14% | 5.8 px/nm | 3.7 nm | 2143 |
| **C: B + 17.9 mm field lens (recommended)** | **100%** | **98–100%** | **5.8 px/nm** | **3.7 nm** | **2204** |

Spectral resolution is the slit image width divided by the dispersion. Both scale with the camera focal length, so resolution depends only on the slit width and the collimator focal length. A 25 µm slit would give about 1.9 nm. In C the slit image is about 21 px wide, so you can bin 4×4 freely, which buys signal-to-noise for free. "3.7 nm" assumes perfect lenses, so expect real M12 lenses to land around 5–8 nm.

## 2. Final optical train (config C)

```
scene ──150 mm──► [scan mirror, ~20 mm ahead of the objective, folds the path]
      ► objective: M12 16 mm, stopped to f/4 (4 mm printed washer = aperture stop)
      ──17.9 mm──► slit 50 µm × 5 mm (long axis vertical) + field lens f = 15 mm right behind it
      ──25 mm──► collimator: M12 25 mm used backwards (slit where its sensor would be)
      ──22 mm──► 500 l/mm grating film (grooves parallel to the slit), taped onto the camera lens front
      ──3 mm──► camera lens: M12 12 mm f/2, axis tilted 22° (1st order at 750 nm)
      ──12 mm──► IMX219 NoIR, spectrum along the 3280 px axis (3.27 mm of 3.68 mm used),
                 slit along the 2464 px axis (2.4 mm of 2.76 mm used)
495 nm long-pass filter (SCHOTT GG-495) in front of the objective, in the same cap as the f/4 stop
```

The printed head in `cad/` builds this train with one change at the back end. The camera lens barrel doesn't fit 3 mm from the grating, so the grating sits 6 mm behind the collimator and the camera lens 16 mm behind the grating. The spectrum length, smile and keystone stay the same, but at 500 and 1000 nm the model passes about 60% of the light instead of 100%, and flat-fielding against the white reference evens out the difference. Details are under "Optics as built" in `cad/README.md`.

Scene coverage at 150 mm: the line is 42 mm long and 0.42 mm thick. About 150 lines, spaced one line-thickness apart, cover a 63 mm wide patch.

**Why each piece is there**

- **Objective:** Images the scene onto the slit. Any M12 lens works, as long as it has no IR-cut filter. The f/4 stop matches the beam the camera lens can accept, so a faster lens only adds stray light.
- **Slit:** Picks one line of the scene. Narrower gives better spectral resolution and less light.
- **Field lens:** Doesn't change focus. It only steers each bundle so all of them hit the camera lens (rule 2).
- **Collimator:** Turns each slit point into a parallel beam, so the grating sees one clean angle.
- **Grating:** Spreads wavelength by angle. At 500 l/mm, 500–1000 nm spans 14.5°–30°. The camera sits at 22°.
- **Long-pass filter:** The grating also throws a 2nd order at twice the angle-per-nm. Without the filter, 2nd-order 400–500 nm light lands on top of 1st-order 800–1000 nm.

## 3. Parts list

Prices marked ✓ were checked on the vendor page on 2026-10-05 or 2026-10-06. Prices marked ~ are typical street prices that were not checked.

### 3.1 Spectrograph optics

| # | Part | Recommended | Budget | Notes |
|---|---|---|---|---|
| 1 | Spectrograph camera | RobotShop: Arducam **B0152** NoIR 8 MP IMX219 with M12 lens (LS1820), $61.18 ✓ | Same | NoIR IMX219 with an M12 thread and the Pi v2's 15-pin CSI connector. It works on a Raspberry Pi and on Jetson boards with the stock IMX219 driver. Avoid Arducam B0183, which has a fixed IR-cut filter, and B0187, which is discontinued. Experimental alternative: a Pi NoIR v2 ($15.95 ✓ at PiShop) with its lens removed and an M12 holder glued on. It risks dust on the bare sensor and a fiddly alignment. |
| 2 | Camera lens, 12 mm f/2 | Commonlands **CIL122-F2.0-M12ANIR**, $59 ✓ | Arducam **LN065** (M2512ZH03), $12.99 ✓ | The "ANIR" suffix means no IR-cut filter. The Commonlands lens is IR-corrected, so 500 nm and 900 nm focus on nearly the same plane. The LN065 listing doesn't say whether it has an IR-cut filter, so check it on arrival. |
| 3 | Objective, 16 mm f/2 (stop to f/4) | Commonlands **CIL161-F2.0-M12ANIR**, $39 ✓ | Same | The objective's focus has to hold across the whole band, or each wavelength images a slightly different line. Avoid Arducam LN001 16 mm, which has a 650 nm IR-cut filter. |
| 4 | Collimator, 25 mm f/2 | Arducam / UCTRONICS **LN016** (M2025ZH01), $17.99 ✓ | Same | The listing says "without IR filter". The holder is not included. |
| 5 | M12 lens holders ×2 (objective, collimator) | UCTRONICS **U0756M10** metal M12 holder, ~$5 each | 3D-printed M12×0.5 holder, ~$0 | The printed head in `cad/` has screw holes for the U0756M10. |
| 6 | Field lens, f = 15 mm | Edmund Optics **#49-840**, 12.7 mm dia × 15 mm FL plano-convex, CT 5.25 mm, R 7.75 mm, uncoated, $32.75 ✓ | Any 12.7 mm (1/2") PCX with f = 15–18 mm from a surplus lens assortment, ~$10 | Put the flat side toward the slit. The head's slit block has a 12.7 mm pocket for it. To check an unknown lens, measure its focal length by focusing a distant light. |
| 7 | Slit, 50 µm | DIY: two 20 mm pieces of a 9 mm snap-off utility blade + a 0.05 mm (0.002") feeler-gauge blade as a spacer, ~$8 | Same | Set the gap with the feeler gauge, glue the blades, and pull the gauge out once the glue sets. Single-edge razor blades (38 × 19 mm) work on a bench but don't fit inside the printed head. The lab-grade option is in row 29. |
| 8 | Grating, 500 l/mm | Edmund **#54-509** (12,700 lines/inch, 2 sheets), $19.75 ✓ | Bartovation 500 lines/mm linear sheet, 1 ft × 6 in, $12.38 ✓ | Edmund only rates #54-509 to 700 nm, but holographic film keeps working into the NIR at lower efficiency. A 1000 l/mm grating only covers about 400–700 nm in this layout. |
| 9 | Long-pass filter, 495 nm | Edmund Optics **#54-652**, SCHOTT GG-495 colored glass, Ø12.5 × 3 mm, $37.00 ✓ | Same | It blocks 2nd-order light. Colored glass has a soft edge, which is fine here. It drops into the head's filter cap in front of the objective. For a bench build without the printed head, any 530 nm "IR photography" screw-in filter (~$15) does the same job. The lab-grade option is in row 29. |
| 10 | Scan mirror | Edmund Optics **#43-872**, 20 × 20 × 3 mm enhanced-aluminum first-surface mirror, $31.00 ✓ | Any first-surface mirror offcut cut to about 20 × 20 mm, ~$10 | A rear-surface (bathroom-type) mirror creates ghost lines. The head's mirror clamp is drawn for 3 mm glass and takes a mirror up to 25 mm long. Thinner glass needs a printed shim. |

### 3.2 Scanning, lighting, calibration

| # | Part | Pick | Price | Notes |
|---|---|---|---|---|
| 11 | Mirror stepper, NEMA 8 | StepperOnline **8HS11-0204S** (NEMA 8, 1.8°, 0.2 A, 20 × 20 × 28 mm, 60 g, 4 mm D-cut shaft, M2 holes on 16 mm) | $15.98 ✓ | At 1/32 microstepping, one microstep moves the line 0.29 mm at 150 mm. A NEMA 17 also works on a bench, but on the arm the small motor keeps the scanner head at 185 g instead of 278 g. That takes the SO-101 shoulder servo from 94% to 74% of stall with the arm stretched out. |
| 12 | Stepper driver | Adafruit **#6121** TMC2209 breakout | ~$10 | It's quiet and can be controlled over UART. Set the run current to about 0.2 A for the NEMA 8. |
| 13 | Microcontroller | Espressif **ESP32-DevKitC-32E** (Digi-Key) | ~$10 | Runs the [mirror firmware](../firmware/): it drives the stepper through the TMC2209's UART, homes on the hall sensor and stamps every scan line, and the Jetson talks to it in plain-text lines over USB serial. Any ESP32 DevKit with GPIO 16 to 27 works; the firmware relies on the ESP32's hardware timer and second core, so another kind of board would need a port. |
| 14 | Stepper power supply | 12 V, 1 A or larger DC brick with a barrel jack, plus a jack adapter | ~$12 | The NEMA 8 draws only about 0.2 A per phase. |
| 15 | Home switch | A3144 hall sensor + a small magnet | ~$3 | Gives each scan an absolute start angle. |
| 16 | Breadboard and wiring | Half-size breadboard, jumper wires, 100 µF capacitor across the motor supply | ~$10 | |
| 17 | Lights | 2× halogen clamp or work lights (MR16/GU10 halogen) | ~$25 | LEDs have almost no output past 700 nm, and you need the NIR. |
| 18 | Wavelength calibration | NE-2 neon indicator lamp (Digi-Key or Amazon) + any CFL bulb | ~$8 | Neon covers 585–880 nm, and the mercury in a CFL gives 546 and 611 nm. |
| 19 | Alignment laser | 650 nm red laser diode module | ~$8 | For aligning the optical axis. It also adds a third calibration line near 650 nm. |
| 20 | White reference | White PTFE sheet, 1/8" (McMaster-Carr "PTFE sheets", or a 12 × 24 in sheet from Walmart) | ~$15 | Flat-field and reflectance reference. |
| 21 | Housing, stray-light lining and fasteners | Black PLA print of the head in `cad/` + flocking paper (Protostar or similar). Fasteners: M3 and M2 screws, nuts and heat-set inserts, CA glue and Kapton tape, all listed in the hardware table in `cad/README.md`. | ~$20 | Stray light is the main enemy. Make the camera tilt adjustable by ±3°. You need a 3D printer or a print service. |

### 3.3 Compute and pose camera

| # | Part | Recommended | Budget | Notes |
|---|---|---|---|---|
| 22 | GPU computer (capture + CUDA splat renderer) | NVIDIA **Jetson Orin Nano Super Developer Kit**, $399 (NVIDIA raised it from $249 in July 2026) | Used original **Jetson Nano 4 GB** dev kit, ~$100–150 | The original Nano is end-of-life and stuck on JetPack 4.6 (Ubuntu 18.04, CUDA 10.2), so ROS2 needs Docker. A desktop with an NVIDIA GPU trains the splat much faster. If you have one, a Raspberry Pi 4/5 (~$60–80) is enough for capture. |
| 23 | Storage | 256 GB NVMe SSD, ~$25 | 64 GB microSD, ~$10 | The Orin Nano boots from NVMe. The original Nano uses microSD. |
| 24 | CSI cable | 15-pin to 22-pin camera cable, ~$5 | Not needed on an original Nano or Pi 4 | The Orin Nano and Pi 5 have 22-pin CSI connectors, while these cameras ship with 15-pin cables. |
| 25 | Pose camera | **Raspberry Pi Camera Module v2** (IMX219), $16.50 ✓ at PiShop | Any USB webcam (UVC) | It reads the AprilTag board. IMX219 works natively on Jetson. The Camera Module 3 (IMX708) does not work on Jetson without extra drivers. |

### 3.4 Viewpoints

| # | Part | Recommended | Budget | Notes |
|---|---|---|---|---|
| 26 | Robot arm | **SO-ARM101 follower only** (PartaBot), $299 ✓, but sold out when checked. Alternative: Seeed **SO-ARM101 Pro motor kit**, $277.99 ✓, which includes leader + follower servos and the driver board but no printed parts (Seeed sells those for $29.90). | Tripod or camera slider, moved by hand, ~$25 | Pose comes from the tag board, not the arm, so manual viewpoints work. The arm adds repeatable, automated viewpoints and the ROS2 side of the project. Check whether your kit includes the 12 V servo power supply. |
| 27 | Tag board | AprilTag grid printed and glued to foam board, ~$5 | Same | Gives one pose per viewpoint, then splatting refines it. |

### 3.5 Optional test gear

| # | Part | Pick | Price | Notes |
|---|---|---|---|---|
| 28 | Scope / logic analyzer | Digilent **Analog Discovery 3** | $379 ✓ | For checking the camera exposure strobe against the mirror step timing. A ~$15 8-channel USB logic analyzer covers the timing check alone. |
| 29 | Lab-grade slit and filter | Thorlabs **S50LK** (Ø1" mounted slit, 50 µm × 10 mm) + Thorlabs **FELH0500** (Ø25 mm long-pass) | about $100 each, not checked | These replace rows 7 and 9 on a bench build. Neither fits inside the printed head. Don't buy the Thorlabs S50RD slit, which is only 3 mm long. |

### 3.6 Totals

| Build | Recommended | Budget |
|---|---|---|
| Spectrograph + scanner (3.1 + 3.2) | ~$455 | ~$345 |
| + compute and pose camera (3.3) | ~$900 | ~$495 |
| + viewpoints (3.4), full rig | **~$1,200** | **~$525** |

The spectrograph and scanner rows are the core of the project. Everything else is common hobby gear, so check what you already have before buying. The totals leave out section 3.5. The bench spectrometer stage (build step 1 below) needs only rows 1–2 and 4–9, the lamps in row 18, the laser in row 19, the housing in row 21, and a Raspberry Pi or Jetson to read the camera.

## 4. Build order

1. **Bench spectrometer.** Build the slit, field lens, collimator, grating and camera with no objective, pointed at the neon and CFL lamps. Fit the wavelength map and the smile/keystone warp. This is where you learn alignment.
2. **Line imager.** Add the objective and focus it on a printed target. Measure the line thickness on a knife edge.
3. **Scanner.** Add the mirror and stepper. Scan a flat Macbeth-style color card, then assemble the 2D datacube on the GPU (first CUDA kernel: warp + bin).
4. **3D.** Capture from several viewpoints (arm or tripod), get poses from the AprilTag board, and train the splat with the line-camera renderer.

## 5. Modeling notes and limits

- All lenses are ideal (paraxial). The model is right about geometry, dispersion, smile, keystone and vignetting, but it can't predict blur, distortion or chromatic focus shift from the real M12 lenses, since their prescriptions aren't published. M12 lenses that aren't NIR-corrected will refocus by 50–100 µm between 500 and 1000 nm, so you'll likely see a soft red or soft blue end. If that happens, tilt the sensor slightly (a Scheimpflug tilt).
- A good next Optiland exercise is swapping the paraxial collimator for a real catalog achromat (e.g. Thorlabs AC127-025-B) from Optiland's lens database and comparing the spot sizes.
- Silicon response falls steeply past 900 nm, so plan on a usable range of about 500–950 nm.
- To rerun or explore: `pip install optiland`, edit `CONFIGS` near the bottom of `optics/spectrograph_model.py`, then run `python3 optics/spectrograph_model.py`. It writes `results.txt` and the layout PNGs to `optics/model_output/`.

## Sources

- [Arducam NoIR IMX219 with M12 lens (RobotShop)](https://www.robotshop.com/products/arducam-noir-8-mp-sony-imx219-camera-module-m12-lens-ls1820)
- [Arducam B0183 (IR-cut, lens not swappable)](https://www.arducam.com/b0183-arducam-imx219-distortioin-m12-mount-camera-module-raspberry-pi-compute-module.html)
- [Arducam B0187 NoIR M12 for Jetson (discontinued)](https://arducam.com/product/b0187-arducam-imx219-low-distortion-ir-sensitive-noir-m12-mount-camera-module-for-nvidia-jetson-board)
- [Commonlands CIL161 16 mm IR-corrected](https://commonlands.com/products/ir-corrected-16mm-m12-lens-cil161), [CIL122 12 mm IR-corrected](https://commonlands.com/products/ir-corrected-12mm-m12-lens)
- [Arducam LN016 25 mm (UCTRONICS)](https://www.uctronics.com/m12-mount-mtv2520b-25mm-focal-length-camera-lens-2219.html), [LN065 12 mm](https://www.uctronics.com/arducam-12mm-m12-lens-m2512zh03-for-usb-camera.html), [LN001 16 mm (has IR-cut, avoid)](https://www.uctronics.com/arducam-1/2.5-m12-mount-16mm-focal-length-camera-lens-m2016zh01.html)
- [Edmund #49-840 field lens](https://www.edmundoptics.com/p/127mm-dia-x-150mm-fl-uncoated-plano-convex-lens/10312/), [Edmund #54-652 GG-495 filter](https://www.edmundoptics.com/p/gg-495-125mm-dia-longpass-filter/11321/), [Edmund #43-872 mirror](https://www.edmundoptics.com/p/20-x-20mm-enhanced-aluminum-4-6lambda-mirror/5359/), [Edmund #54-509 grating film](https://www.edmundoptics.com/p/12700-linesinch-6quot-x-12quot-sheets-2pack/11226/)
- [Bartovation 500 l/mm grating sheet](https://bartovation.com/product/other-lab-supplies/diffraction-grating-sheets/500-lines-mm-linear-diffraction-grating-sheet/)
- [Thorlabs precision slits](https://www.thorlabs.com/newgrouppage9.cfm?objectgroup_id=1464), [S50LK](https://www.thorlabs.com/item/S50LK)
- [Thorlabs hard-coated edgepass filters (FELH0500)](https://www.thorlabs.com/hard-coated-edgepass-filters?pn=FEL+H0500)
- [StepperOnline 8HS11-0204S (NEMA 8)](https://www.omc-stepperonline.com/nema-8-bipolar-1-8deg-1-4ncm-1-98oz-in-0-2a-20x20x28mm-4-wires-8hs11-0204s)
- [Adafruit TMC2209 breakout](https://www.adafruit.com/product/6121)
- [Espressif ESP32-DevKitC-32E (Digi-Key)](https://www.digikey.com/en/products/detail/espressif-systems/ESP32-DEVKITC-32E/12091810)
- [Jetson price increase, Hardware Busters, July 2026](https://hwbusters.com/news/nvidia-jetson-prices-jump-up-to-101-the-249-orin-nano-super-is-now-399/), [NVIDIA: buy Jetson](https://developer.nvidia.com/buy-jetson)
- [Raspberry Pi Camera Module v2 (PiShop)](https://www.pishop.us/product/raspberry-pi-camera-module-v2/)
- [SO-ARM101 follower only (PartaBot)](https://partabot.com/products/so-arm101-follower-only), [Seeed SO-ARM101 Pro motor kit](https://www.seeedstudio.com/SO-101-Low-Cost-AI-Arm-Kit-Pro-p-6427.html)
- [Digilent Analog Discovery 3](https://digilent.com/shop/analog-discovery-3/)
