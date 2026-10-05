# Line-scan hyperspectral splatting: parts list and first-order design

Status: design draft, 2026-10-05. Model: `optics/spectrograph_model.py` (Optiland 0.6, ideal thin lenses). Results: `optics/model_output/results.txt` and the layout PNGs. Diagram: `docs/optical_train_v2.svg`.

## 1. What the model changed

I built the earlier optical train in Optiland and traced it. Three findings change the parts list.

1. **The stock Pi NoIR v2 lens doesn't work as the spectrograph camera.** Its entrance pupil is 1.5 mm wide, while the collimated beam from a 25 mm collimator at f/4 is about 6 mm. Only 8% of the light reaches the sensor at the center of the line. At the ends of the line the light walks off the pupil entirely, so 0% gets through. The 650-pixel line I quoted earlier would really be a short bright stub. The fix is an IMX219 NoIR board with an **M12 lens mount** and a 12 mm f/2 lens, which has a 6 mm pupil. It is the same sensor and driver, so it still plugs into the Jetson Nano CSI port.
2. **A field lens at the slit is needed.** Without one, the bundles from the ends of the slit leave the collimator at an angle and miss the camera lens, so the line ends get 12–14% of the light. A small lens with f ≈ 18 mm right behind the slit makes the chief rays parallel. Every point on the slit then lands on the same patch of the camera lens, which brings the line ends up to 98–100%.
3. **Expect about 32 px of smile and about 11 px of keystone.** Smile means a single wavelength traces a curve rather than a straight row across the sensor. It comes from conical diffraction at the grating and is unavoidable with a flat transmission grating. Keystone means the line length changes slightly with wavelength. You calibrate both once with lamp lines, then remap every frame with a warp. Writing that warp as a CUDA kernel is a good first CUDA exercise.

| Config (ideal lenses) | Light at line center | Light at line ends | Dispersion | Spectral res. | Pixels along line |
|---|---|---|---|---|---|
| A: stock Pi NoIR v2 lens | 8% | 0% | 1.5 px/nm | 3.7 nm | 543 |
| B: M12 IMX219 + 12 mm lens | 92–100% | 12–14% | 5.8 px/nm | 3.7 nm | 2143 |
| **C: B + 18 mm field lens (recommended)** | **100%** | **98–100%** | **5.8 px/nm** | **3.7 nm** | **2204** |

Spectral resolution is the slit image width divided by the dispersion. Both scale with the camera focal length, so resolution depends only on the slit width and the collimator focal length. A 25 µm slit would give about 1.9 nm. In C the slit image is about 21 px wide, so you can bin 4×4 freely, which buys signal-to-noise for free. "3.7 nm" assumes perfect lenses, so expect real M12 lenses to land around 5–8 nm.

## 2. Final optical train (config C)

```
scene ──150 mm──► [scan mirror, ~20 mm ahead of the objective, folds the path]
      ► objective: M12 16 mm, stopped to f/4 (4 mm printed washer = aperture stop)
      ──17.9 mm──► slit 50 µm × 5 mm (long axis vertical) + field lens f≈18 mm right behind it
      ──25 mm──► collimator: M12 25 mm used backwards (slit where its sensor would be)
      ──22 mm──► 500 l/mm grating film (grooves parallel to the slit), taped onto the camera lens front
      ──3 mm──► camera lens: M12 12 mm f/2, axis tilted 22° (1st order at 750 nm)
      ──12 mm──► IMX219 NoIR, spectrum along the 3280 px axis (3.27 mm of 3.68 mm used),
                 slit along the 2464 px axis (2.4 mm of 2.76 mm used)
long-pass filter (≥500 nm) anywhere in the collimated space or in front of the objective
```

Scene coverage at 150 mm: the line is 42 mm long and 0.42 mm thick. About 150 lines, spaced one line-thickness apart, cover a 63 mm wide patch.

**Why each piece is there**

- **Objective:** Images the scene onto the slit. Any M12 lens works, as long as it has no IR-cut filter. The f/4 stop matches the beam the camera lens can accept, so a faster lens only adds stray light.
- **Slit:** Picks one line of the scene. Narrower gives better spectral resolution and less light.
- **Field lens:** Doesn't change focus. It only steers each bundle so all of them hit the camera lens (finding 2).
- **Collimator:** Turns each slit point into a parallel beam, so the grating sees one clean angle.
- **Grating:** Spreads wavelength by angle. At 500 l/mm, 500–1000 nm spans 14.5°–30°. The camera sits at 22°.
- **Long-pass filter:** The grating also throws a 2nd order at twice the angle-per-nm. Without the filter, 2nd-order 400–500 nm light lands on top of 1st-order 800–1000 nm.

## 3. Parts list

Every row names a vendor and a part number. Prices marked ✓ were checked on the vendor page on 2026-10-05. Prices marked ~ are typical street prices I did not check.

### Spectrograph optics

| # | Part | Vendor and part number | Price | Why this one |
|---|---|---|---|---|
| 1 | Spectrograph camera | RobotShop: Arducam NoIR 8 MP IMX219 with M12 lens (LS1820) | $61.18 ✓ | NoIR IMX219 with an M12 thread, and the same 15-pin CSI as the Pi v2, so the Nano's stock IMX219 driver works. Avoid Arducam B0183 (fixed IR-cut filter) and B0187 (discontinued). |
| 2 | Camera lens, 12 mm f/2 | Commonlands **CIL122-F2.0-M12ANIR** (the "ANIR" suffix means no IR-cut filter) | $59 ✓ | IR-corrected, so 500 nm and 900 nm focus on nearly the same plane. Budget option: Arducam LN065 (M2512ZH03), 12 mm f/2, $12.99 ✓, but its listing doesn't say whether it has an IR-cut filter, so check on arrival. |
| 3 | Objective, 16 mm f/2 (stop to f/4) | Commonlands **CIL161-F2.0-M12ANIR** | $39 ✓ | The objective's focus has to hold across the whole band, otherwise each wavelength images a slightly different line. Avoid Arducam LN001 16 mm: it has a 650 nm IR-cut filter. |
| 4 | Collimator, 25 mm f/2 | Arducam / UCTRONICS **LN016** (M2025ZH01) | $17.99 ✓ | The listing says "without IR filter". Its holder is not included. |
| 5 | M12 lens holders ×2 (objective, collimator) | Arducam M12 lens holder, from the "compatible holders" list on the LN016 page, or 3D-print one | ~$5 each | Any metal M12×0.5 holder works. |
| 6 | Field lens, f = 18 mm | Edmund Optics **#32-008**, 9 mm dia × 18 mm FL plano-convex, uncoated N-BK7 | $30.50 ✓ | Exactly the f ≈ 18 mm the model wants. Put the flat side toward the slit. |
| 7 | Slit, 50 µm | DIY: 2 single-edge razor blades + a 0.05 mm (0.002") blade from a feeler-gauge set (any hardware store) | ~$8 | Upgrade: Thorlabs **S50LK** (Ø1" mounted, 50 µm × 10 mm). I couldn't load its price, so check it before ordering. Don't buy the S50RD: it's only 3 mm long. |
| 8 | Grating, 500 l/mm | Bartovation 500 lines/mm linear sheet, 1 ft × 6 in | $12.38 ✓ | Alternative: Edmund **#54-509** (12,700 lines/inch = 500 l/mm, 2 sheets), $19.75 ✓. EO only rates it to 700 nm, but holographic film works further into the NIR at lower efficiency. |
| 9 | Long-pass filter ≥500 nm | Cheap: any 530 nm "IR photography" screw-in filter (K&F Concept or Zomei, 37 mm). Lab grade: Thorlabs **FELH0500** (Ø25 mm) | ~$15 / price not checked | Blocks 2nd-order light. |
| 10 | Scan mirror | Edmund Optics **#43-792**, 25 × 25 mm enhanced-aluminum first-surface mirror | $31.00 ✓ | A first-surface mirror has no ghost reflection. EO #15-490 (rhodium) costs $47.75 and isn't needed. |

### Scanning, lighting, calibration

| # | Part | Vendor and part number | Price | Why this one |
|---|---|---|---|---|
| 11 | Mirror stepper, 0.9° | StepperOnline **17HM19-1684S** (NEMA17, 0.9°, 1.68 A) | $11.41 ✓ | At 1/16 microstepping, one microstep moves the line 0.29 mm at 150 mm. |
| 12 | Stepper driver | Adafruit **#6121** TMC2209 breakout | ~$10 | Quiet, with UART control from one of your ESP32s. Run the ESP32 as a micro-ROS node. |
| 13 | Home switch | Any A3144 hall sensor + a small magnet | ~$3 | Gives each scan an absolute start angle. |
| 14 | Lights | 2× halogen clamp or work lights (any MR16/GU10 halogen) from a hardware store | ~$25 | You need the NIR output, and LEDs have almost none past 700 nm. |
| 15 | Wavelength calibration | NE-2 neon indicator lamp (Digi-Key or Amazon) + any CFL bulb + your red diode | ~$8 | Neon covers 585–880 nm, mercury in the CFL gives 546/611 nm, and the diode gives about 650 nm. |
| 16 | White reference | White PTFE sheet, 1/8" (McMaster-Carr "PTFE sheets", or the Walmart-listed 12×24 in sheet) | ~$15 | Flat-field and reflectance reference. |
| 17 | Stray-light lining | Black PLA for the housing + Protostar or Acktar-style flocking paper | ~$15 | Stray light is the main enemy. |

### Pose and robot (mostly owned)

| # | Part | Vendor and part number | Price | Notes |
|---|---|---|---|---|
| 18 | Pose camera | Any camera you own (second CSI port on a Nano B01, else USB) | $0 | |
| 19 | Tag board | AprilTag grid printed and glued to foam board | ~$5 | |
| 20 | Arm, Jetson Nano, ESP32, AD3 | Owned | $0 | |

**Totals:**
- **Recommended: about $375.** This includes the two Commonlands IR-corrected lenses, which are the biggest optical-quality upgrade.
- **Budget: about $330**, using Arducam LN065 instead of CIL122 for the camera lens.
- Each of the Thorlabs slit and filter adds about $100.

To save more, check your Amazon gratings first (−$12). You could also try mirrors you already own on your optics table, if any are first-surface (−$31).

## 4. Build order

1. **Bench spectrometer.** Build the slit, field lens, collimator, grating and camera with no objective, pointed at the neon and CFL lamps. Fit the wavelength map and the smile/keystone warp. This is where you learn alignment.
2. **Line imager.** Add the objective and focus it on a printed target. Measure the line thickness on a knife edge.
3. **Scanner.** Add the mirror and stepper. Scan a flat Macbeth-style color card, then assemble the 2D datacube on the Jetson (first CUDA kernel: warp + bin).
4. **3D.** Use arm viewpoints, AprilTag poses, and the line-camera splat renderer.

## 5. Modeling notes and limits

- All lenses are ideal (paraxial). The model is right about geometry, dispersion, smile, keystone and vignetting, but it can't predict blur, distortion or chromatic focus shift from the real M12 lenses, since their prescriptions aren't published. M12 lenses that aren't NIR-corrected will refocus by 50–100 µm between 500 and 1000 nm, so you'll likely see a soft red or soft blue end. If that happens, tilt the sensor slightly (a Scheimpflug tilt).
- A good next Optiland exercise is swapping the paraxial collimator for a real catalog achromat (e.g. Thorlabs AC127-025-B) from Optiland's lens database and comparing the spot sizes.
- Silicon response falls steeply past 900 nm, so plan on a usable range of about 500–950 nm.
- To rerun or explore: `pip install optiland`, edit `CONFIGS` near the bottom of `optics/spectrograph_model.py`, then run `python3 optics/spectrograph_model.py`. It writes `results.txt` and the layout PNGs to `optics/model_output/`.

## Sources

- [Arducam NoIR IMX219 with M12 lens (RobotShop)](https://www.robotshop.com/products/arducam-noir-8-mp-sony-imx219-camera-module-m12-lens-ls1820)
- [Arducam B0183 (IR-cut, lens not swappable)](https://www.arducam.com/b0183-arducam-imx219-distortioin-m12-mount-camera-module-raspberry-pi-compute-module.html)
- [Arducam B0187 NoIR M12 for Jetson (discontinued)](https://arducam.com/product/b0187-arducam-imx219-low-distortion-ir-sensitive-noir-m12-mount-camera-module-for-nvidia-jetson-board)
- [Bartovation 500 l/mm grating sheet](https://bartovation.com/product/other-lab-supplies/diffraction-grating-sheets/500-lines-mm-linear-diffraction-grating-sheet/)
- [Thorlabs precision slits](https://www.thorlabs.com/newgrouppage9.cfm?objectgroup_id=1464), [S50LK](https://www.thorlabs.com/item/S50LK)
- [Thorlabs hard-coated edgepass filters (FELH0500)](https://www.thorlabs.com/hard-coated-edgepass-filters?pn=FEL+H0500)
- [Adafruit TMC2209 breakout](https://www.adafruit.com/product/6121)
- [Commonlands CIL161 16 mm IR-corrected](https://commonlands.com/products/ir-corrected-16mm-m12-lens-cil161), [CIL122 12 mm IR-corrected](https://commonlands.com/products/ir-corrected-12mm-m12-lens)
- [Arducam LN016 25 mm (UCTRONICS)](https://www.uctronics.com/m12-mount-mtv2520b-25mm-focal-length-camera-lens-2219.html), [LN065 12 mm](https://www.uctronics.com/arducam-12mm-m12-lens-m2512zh03-for-usb-camera.html), [LN001 16 mm (has IR-cut, avoid)](https://www.uctronics.com/arducam-1/2.5-m12-mount-16mm-focal-length-camera-lens-m2016zh01.html)
- [Edmund #32-008 field lens](https://www.edmundoptics.com/p/90mm-dia-x-180mm-fl-uncoated-plano-convex-lens/2084/), [Edmund #43-792 mirror](https://www.edmundoptics.com/p/25-x-25mm-enhanced-aluminum-4-6lambda-mirror/5288/), [Edmund #54-509 grating film](https://www.edmundoptics.com/p/12700-linesinch-6quot-x-12quot-sheets-2pack/11226/)
- [StepperOnline 17HM19-1684S](https://www.omc-stepperonline.com/nema-17-bipolar-0-9deg-44ncm-62-3oz-in-1-68a-2-8v-42x42x47mm-4-wires-17hm19-1684s)
