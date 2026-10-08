# Line-scan hyperspectral splatting: parts list and first-order design

Status: parts list updated to what was bought, 2026-10-08. Nothing has arrived yet; the Amazon order and the long-pass filter are due Friday, October 9. Model: `optics/spectrograph_model.py` (Optiland 0.6.2): configs A to C are the design study with ideal lenses, and D and E use the bought parts. Results: `optics/model_output/results.txt` and the layout PNGs. Diagram: `docs/optical_train_v2.svg`.

Section 3 is the bill of materials for this build. Each row names the exact part that was bought or is already owned, with its part number or Amazon ASIN, what it cost and where it stands. The notes keep the earlier generic picks where they help someone building from scratch. Section 6 lists the orders with each listing's full name.

## 1. Design findings from the optical model

The obvious cheap build puts a slit, collimator and grating in front of a stock Raspberry Pi NoIR camera. The Optiland model shows that this doesn't work, and it points to four design rules.

1. **Don't use a camera with a tiny fixed lens.** The Pi NoIR v2's stock lens has a 1.5 mm entrance pupil, while the collimated beam from a 25 mm collimator at f/4 is about 6 mm wide. Only 8% of the light reaches the sensor at the center of the line. At the ends of the line, the light walks off the pupil entirely and 0% gets through. Use an IMX219 NoIR board with an **M12 lens mount** and a 12 mm f/2 lens, which has a 6 mm pupil. It is the same sensor and driver, so it still plugs into a Raspberry Pi or Jetson CSI port.
2. **Put a field lens at the slit.** Without one, the bundles from the ends of the slit leave the collimator at an angle and miss the camera lens, so the line ends get 12–14% of the light. A small lens right behind the slit, with a focal length close to the objective-to-slit distance (17.9 mm), makes the chief rays parallel. Every point on the slit then lands on the same patch of the camera lens, and the line ends get 98–100%. The 15 mm Edmund lens in row 6 does as well in the model: 98% at the line ends.
3. **Plan to calibrate smile and keystone.** Expect about 32 px of smile, meaning one wavelength traces a curve across the sensor rather than a straight row. Conical diffraction at the grating causes it, and every flat transmission grating has it. Expect about 11 px of keystone, meaning the line length changes slightly with wavelength. You calibrate both once with lamp lines, then remap every frame with a warp. Writing that warp as a CUDA kernel makes a good first CUDA exercise.
4. **Turn the field lens's curved side toward the slit.** A plano-convex lens does its bending at its curved face. With the flat side toward the slit, the bought lens acts as if it sat 3.9 mm behind the slit, and from there it magnifies the slit 1.35× for the collimator. That makes the slit image 31 px wide instead of 24 (5.4 nm resolution instead of 4.1), and the image of the 5 mm slit grows to 3.26 mm, longer than the sensor's 2.76 mm side, so 15% of the line falls off the sensor. With the curved side toward the slit it acts from 0.43 mm behind the slit and magnifies 1.03×. Section 2 has the numbers.

| Config (ideal lenses) | Light at line center | Light at line ends | Dispersion | Spectral res. | Pixels along line |
|---|---|---|---|---|---|
| A: stock Pi NoIR v2 lens | 8% | 0% | 1.5 px/nm | 3.7 nm | 543 |
| B: M12 IMX219 + 12 mm lens | 92–100% | 12–14% | 5.8 px/nm | 3.7 nm | 2143 |
| **C: B + 17.9 mm field lens (recommended)** | **100%** | **98–100%** | **5.8 px/nm** | **3.7 nm** | **2204** |

Spectral resolution is the slit image width divided by the dispersion. Both scale with the camera focal length, so resolution depends only on the slit width and the collimator focal length. A 25 µm slit would give about 1.9 nm. In C the slit image is about 21 px wide, so you can bin 4×4 freely, which buys signal-to-noise for free. "3.7 nm" assumes perfect lenses, so expect real M12 lenses to land around 5–8 nm.

The table's slit widths are the paraxial estimate, slit width × 12/25. Tracing the slit's two edges through the grating gives 4.1 nm for C. The beam leaves the grating at 22°, which stretches the slit image 1.08× across the dispersion, and C's thin field lens 0.5 mm behind the slit adds 1.03×. The bought-parts numbers in section 2 are traced the same way.

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
- **Field lens:** It steers each bundle so all of them hit the camera lens (rule 2). An ideal thin lens at the slit wouldn't change the focus; the real one is 5.25 mm thick, so the collimator focuses a little further back.
- **Collimator:** Turns each slit point into a parallel beam, so the grating sees one clean angle.
- **Grating:** Spreads wavelength by angle. At 500 l/mm, 500–1000 nm spans 14.5°–30°. The camera sits at 22°.
- **Long-pass filter:** The grating also throws a 2nd order at twice the angle-per-nm. Without the filter, 2nd-order 400–500 nm light lands on top of 1st-order 800–1000 nm.

### With the parts that were bought (configs D and E)

Configs D and E trace the bought parts wherever the vendor publishes enough to model them. The Edmund field lens is real N-BK7 surfaces (R 7.75 mm, 5.25 mm thick). The GG-495 is a 3 mm glass plate, and the printed stop and the 20 × 20 mm mirror are apertures. The objective uses the CIL161's published 15.6 mm focal length. The M12 lenses are still ideal, since Commonlands and Arducam don't publish prescriptions. Both configs use the spacings `cad/` builds, with the grating 6 mm behind the collimator and the camera lens 16 mm behind the grating, and the collimator focused on the slit as the field lens shows it. D puts every part where `cad/` draws it today. E turns the field lens round and moves the stop onto the lens.

| Traced, ideal M12 lenses | D: as `cad/` draws it | E: field lens flipped, stop on the lens |
|---|---|---|
| Field lens faces the slit with its | flat side | curved side |
| Slit as the collimator sees it | 1.35× as big, 0.42 mm further back | 1.03× as big, 1.76 mm further back |
| Collimator principal plane behind the slit | 25.4 mm | 26.8 mm |
| Slit image width | 35 µm (31 px) | 27 µm (24 px) |
| Spectral resolution | 5.4 nm | 4.1 nm |
| Line on the sensor's 2.76 mm side | 3.26 mm, 85% of it on the sensor | 2.56 mm, all of it |
| Scene line at 150 mm that reaches the sensor | 36.5 mm | 43.1 mm |
| Smile / keystone | 58 / 20 px | 36 / 12 px |
| Light at the line end, 750 nm / 500 nm | 37% / 21% | 54% / 35% |
| RMS spot at the line end | 5 µm | 5 µm |

What the trace decides, part by part:

- **Field lens.** Turn it curved side toward the slit, as in E (rule 4). The lens's principal plane is where it acts as a thin lens. With the flat side first, that plane is 3.46 mm inside the glass (the thickness divided by the glass index, 5.25 / 1.52), so 3.9 mm behind the slit once the 0.43 mm blades are added. With the curved side first it is at the curved vertex, right behind the blades. Either way the collimator has to sit a little further back than 25 mm, because the slit seen through the glass looks further away: 0.42 mm with the flat side first, 1.76 mm with the curved side first.
- **Objective and stop.** Commonlands lists the CIL161 at 15.6 mm and f/2.0, so the 4 mm printed stop makes it f/3.9 and the line 43 mm long at 150 mm. Keep a 4 mm stop. The CIL122 camera lens's 6 mm pupil only takes about an f/4 beam. Opening the objective to f/2 lets in 3.8 times the light, but only 1.27 times E's signal reaches the sensor at the line center. Light worth another 2.6 times E's signal gets through the slit and misses the sensor. That light lands on the walls of the head as stray light, which raises the floor under every spectral line. Where the stop sits matters as much as its size, because the field lens is meant to image the stop onto the camera lens. At the front of the filter cap, 11.7 mm ahead of the objective's principal plane by `cad/`'s estimate, the line ends get 38% of the center's light. A thin washer on the lens's front face (8.1 mm ahead, filter in front of it) gets 54%. The lens's own internal stop would give 95%, but a washer can't go there. A 5 mm washer on the lens face gives 1.14 times E's center signal at the center and 0.76 times at the line ends, but at the ends about as much light again misses the sensor. Printing 4 and 5 mm washers and comparing them on a neon frame at first light is a cheap test.
- **Slit width.** Keep 50 µm, set with the feeler gauge's 0.05 mm leaf. With E that gives 4.1 nm before lens blur. The 0.04 mm leaf gives 3.3 nm for 20% less light. If the field lens stays flat side first, the 0.04 mm leaf brings D back to about 4.3 nm, but the line still overfills the sensor.
- **Camera tilt.** No change with a 500 l/mm grating: the camera axis stays at 22.0°, the 750 nm first-order angle. The sensor's 3.68 mm then covers 460–1022 nm at the line center, so 500–1000 nm fits with about 35 nm to spare at each end. Each degree of tilt error moves the band about 33 nm, so keep the tilt within about −0.5° and +1°. The pitch of the grating Ryan owns isn't confirmed. A 600 l/mm film would need the camera at 26.7° and would cover only 516–967 nm. A 1000 l/mm film can't work, because 1000 nm would leave it at 90°. A 300 l/mm film fits the band easily but halves the dispersion, giving 6.9 nm.
- **Filter and second order.** SCHOTT's data for 3 mm of GG-495, including the two surface reflections, gives 20% at 490 nm, 67% at 500 nm, 88% at 520 nm and 86–89% from 550 to 1000 nm. The edge is soft, so second-order light from about 485–511 nm still gets through, and it lands on the first-order 970–1022 nm end. Treat the spectrum past about 970 nm with care. A rough estimate puts that light at up to about a tenth of the signal there under a halogen lamp, and more under daylight.
- **Spot sizes.** With ideal M12 lenses the only aberrations left come from the field lens. Spots are about 1 µm RMS at the line center and about 5 µm at the line ends, well under the 27 µm slit image. Real M12 lens blur isn't published and will likely dominate.
- **Scan mirror.** A 20 × 20 mm piece clears the beam with room to spare: taking the mirror out of the model changes no number.

`cad/` draws config D today. The changes for E (the slit block's lens seat, the collimator 1.8 mm further back with everything behind it, and the stop washer on the lens face) go to the CAD pass that follows Friday's measurements.

## 3. Parts list

Statuses as of 2026-10-08: **ordered** (paid, not arrived), **owned** (Ryan's own stock), **to buy**, **unconfirmed** (owned, but a detail still needs checking). Paid prices come from the order confirmations, before tax. Prices marked ✓ on parts still to buy were read on the seller's page on October 6 or 7.

### 3.1 Spectrograph optics

| # | Part | Bought | Paid | Status | Notes |
|---|---|---|---|---|---|
| 1 | Spectrograph camera | Arducam **B0152** NoIR 8 MP IMX219 M12-mount board, from [Arducam's store](https://www.arducam.com/arducam-raspberry-pi-camera-module-v2-8mp-imx219-m12-noir-picam-b0152.html) | $35.00 | ordered, express | No IR-cut filter, 36 × 36 mm, 15-pin CSI. Its stock 1.8 mm LS1820 lens comes off for row 2. The same board is on RobotShop as the LS1820 listing. Avoid Arducam B0183, which has a fixed IR-cut filter. |
| 2 | Camera lens, 12 mm f/2 | Commonlands **CIL122-F2.0-M12ANIR**, [IR Corrected 12mm M12 Lens, F/2.0, Without Filter](https://commonlands.com/products/ir-corrected-12mm-m12-lens) | $59.00 | ordered | First ordered as the -M12A650 version with a 650 nm IR-cut filter; Commonlands swapped it to no filter on October 7 at the same price. Look into the back on arrival: a pink or blue-green tint means the IR-cut version came anyway. Budget alternative: Arducam LN065, $12.99. |
| 3 | Objective, 16 mm f/2 | Commonlands **CIL161-F2.0-M12ANIR**, [IR Corrected 16mm M12 Lens, F/2.0, Without Filter](https://commonlands.com/products/ir-corrected-16mm-m12-lens-cil161) | $39.00 | ordered | Swapped from -M12A650 the same way. Commonlands lists its focal length as 15.6 mm. The 4 mm printed stop makes it f/3.9 (section 2). Avoid Arducam LN001, which has a 650 nm IR-cut filter. |
| 4 | Collimator, 25 mm f/2 | Arducam **LN016** (lens M2025ZH01), 1/2" format, no IR filter, from [Arducam's store](https://www.arducam.com/m2025zh01-m12-mount-25mm-focal-length-camera-lens-mtv2520b-for-raspberry-pi-camera.html) | $17.99 | ordered, express | 17 × 21 mm, 12.5 mm aperture. Back focus isn't published, and a sibling Arducam 25 mm lens lists 5.2 mm, so measure it. |
| 5 | M12 holders ×2 (objective, collimator) | uxcell 10-pack, 20 mm screw spacing, Amazon [B00R1J42T8](https://www.amazon.com/dp/B00R1J42T8) | $7.79 | ordered, due Oct 9 | Plastic and 14 mm tall: cut two down to the lens thread + 1 mm, as in step 1 of the [Assembly](../cad/README.md#assembly). From scratch: UCTRONICS U0756M10 metal holders. |
| 6 | Field lens, f = 15 mm | Edmund Optics **#49-840**, [12.7 mm dia. × 15.0 mm FL, uncoated plano-convex](https://www.edmundoptics.com/p/127mm-dia-x-150mm-fl-uncoated-plano-convex-lens/10312/), N-BK7, CT 5.25 mm | $32.75 + $9.99 ground shipping | ordered, 1–7 business days | Turn its curved side toward the slit (section 2). Until the slit block is redrawn, `cad/` seats it flat side toward the blades. |
| 7 | Slit, 50 µm | 9 mm snap-off blades, 25-pack, Amazon [B0FKTN8WJZ](https://www.amazon.com/dp/B0FKTN8WJZ), set apart with the 0.05 mm leaf of a Spurtar 32-blade feeler gauge set, Amazon [B081SYZ1YV](https://www.amazon.com/dp/B081SYZ1YV) | $4.49 + $4.97 | ordered, due Oct 9 | Two 20 mm pieces of blade. Set the gap with the leaf, glue the blades, and pull the leaf out once the glue sets. The blade thickness isn't listed and `cad/` assumes 0.38 mm, so measure one. The 0.04 mm leaf gives a 40 µm slit. Single-edge razor blades don't fit the printed head. The lab-grade option is in row 32. |
| 8 | Grating, 500 l/mm | Ryan's own diffraction gratings | — | unconfirmed | Pitch not confirmed. If none is 500 l/mm, buy Rainbow Symphony 500 l/mm slides, Amazon [B00PBFTQMM](https://www.amazon.com/dp/B00PBFTQMM), $15.60 ✓. Other 500 l/mm film: Edmund #54-509 ($19.75 ✓) or a [Bartovation sheet](https://bartovation.com/product/other-lab-supplies/diffraction-grating-sheets/500-lines-mm-linear-diffraction-grating-sheet/) ($12.38 ✓). Section 2 says what other pitches do. |
| 9 | Long-pass filter, 495 nm | Edmund Optics **#54-652**, [GG-495 12.5 mm dia. longpass filter](https://www.edmundoptics.com/p/gg-495-125mm-dia-longpass-filter/11321/), SCHOTT GG495, 3 mm | $37.00 | ordered, due Oct 9 | Blocks 2nd-order light; its soft edge passes 67% at 500 nm and 88% at 520 nm (section 2). It sits in the head's filter cap. For a bench build without the head, any 530 nm screw-in filter does the same job. The lab-grade option is in row 32. |
| 10 | Scan mirror | RUEHALF 100 × 100 × 3 mm front-surface mirror, Amazon [B0GW4WT69F](https://www.amazon.com/dp/B0GW4WT69F), cut to 20 × 20 mm with a glass scorer | $20.90 | ordered, due Oct 9 | Score the bare back, not the coated face. Keep the side across the shaft at 20 mm or just under; along the shaft, 20 to 25 mm fits the clamp. Edmund's precut #43-872 ($31.00) was backordered. A rear-surface mirror would create ghost lines. |

### 3.2 Scanning, electronics, lighting, calibration

| # | Part | Bought | Paid | Status | Notes |
|---|---|---|---|---|---|
| 11 | Mirror stepper, NEMA 8 | StepperOnline **8HS11-0204S** (1.8°, 0.2 A, 20 × 20 × 28 mm, 60 g, 4 mm D-cut shaft, M2 holes on 16 mm), Amazon [B00PJYL6BY](https://www.amazon.com/dp/B00PJYL6BY), sold by StepperOnline | $24.48 | ordered, due Oct 9 | At 1/32 microstepping, one microstep moves the line 0.29 mm at 150 mm. On the arm the small motor keeps the head at 185 g instead of 278 g with a NEMA 17, which takes the shoulder servo from 94% to 74% of stall with the arm stretched out. Measure how deep its M2 holes are threaded: the M2 × 5 screws need 3.2 mm, or file 1 mm off four of them. |
| 12 | Stepper driver | BIGTREETECH **TMC2209 V1.3**, Amazon [B07ZPYKL46](https://www.amazon.com/dp/B07ZPYKL46) | $17.47 | ordered, due Oct 9 | Quiet, and set over UART. It ships at VREF ≈ 1.2 V (0.85 A): turn it to about 0.28 V (0.2 A) with the motor unplugged before the first run ([pcb/README.md](../pcb/README.md#before-the-first-power-up)). From scratch: Adafruit #6121 breakout. |
| 13 | Microcontroller | Ryan's own ESP32 dev board | — | unconfirmed | Runs the [mirror firmware](../firmware/). The controller board's socket is for the 38-pin ESP32-DevKitC with rows 25.4 mm apart, so check the pin count and row spacing before ordering it. From scratch: Espressif ESP32-DevKitC-32E, ~$10. |
| 14 | Stepper power supply | ALITOVE 12 V 3 A adapter, 5.5 × 2.1 mm, with a screw-terminal jack adapter, Amazon [B07VQGHSWY](https://www.amazon.com/dp/B07VQGHSWY) | $9.97 | ordered, due Oct 9 | The NEMA 8 draws only about 0.2 A per phase. |
| 15 | Home switch | Gikfun A3144 hall sensors, 20-pack, Amazon [B07QS6PN3B](https://www.amazon.com/dp/B07QS6PN3B), and one of Ryan's own Ø5 × 1.6 mm disc magnets | $7.58 | ordered, due Oct 9; magnets owned | Gives each scan an absolute start angle. The clamp's magnet pocket is cut for 1.6 mm magnets. The A3144 switches on one pole only, so glue the magnet with the face that trips it facing out. |
| 16 | Controller boards | Scan-mirror controller and hall-sensor breakout, KiCad in [`pcb/`](../pcb/) | not priced | to order | They replace the breadboard wiring. Parts and the JLCPCB order steps are in [pcb/README.md](../pcb/README.md#parts). For breadboard tests: Ryan's breadboard, jumpers, two 10 kΩ, one 1 kΩ and a 100 µF 25 V capacitor (owned, unconfirmed). |
| 17 | Lights | HDX 250 W portable halogen work light, LG351-250W, [Home Depot 334046804](https://www.homedepot.com/p/HDX-250-Watt-Portable-Halogen-Work-Light-LG351-250W/334046804) | $17.98 ✓ | to buy, unless owned | LEDs have almost no output past 700 nm, and you need the NIR. A second light evens out shadows. Skip "cool-beam" GU10 and MR16 bulbs, whose reflectors send the NIR out the back. |
| 18 | Wavelength calibration | EcoSmart 13 W CFL daylight 4-pack, ESL13T2-5K-4-ESM, [Home Depot 313261174](https://www.homedepot.com/p/60-Watt-Equivalent-A19-Spiral-Non-Dimmable-E26-Medium-Base-CFL-Compact-Fluorescent-Light-Bulb-Daylight-5000K-4-Pack-ESL13T2-5K-4-ESM/313261174), and a Sperry GFI6302N outlet tester, [Home Depot 301961462](https://www.homedepot.com/p/Sperry-GFCI-Outlet-Tester-GFI6302N/301961462) | $9.97 ✓ + $14.50 ✓ | to buy, unless owned | The CFL's mercury and phosphor give 546, 578 and 611 nm. The tester's lamps are neon, about 585–880 nm, and it is the safe way to run neon off the wall. Any fluorescent bulb, or any tester whose lamps glow orange, works. |
| 19 | Alignment laser | Ryan's red diode lasers | — | owned | For aligning the optical axis, and the calibration kit's last check. Not a wavelength reference: a diode drifts a few nm as it warms. |
| 20 | White reference | Sasylvia virgin PTFE sheet, 12 × 6 in, 1/16 in thick, Amazon [B0GKGHX1W6](https://www.amazon.com/dp/B0GKGHX1W6) | $11.99 | ordered, due Oct 9 | Flat-field and reflectance reference. At 1.6 mm it lets some light through, so cut it in half and stack the halves, or back it with white card. |
| 21 | Housing and stray-light lining | The head in `cad/`, printed in black PETG on Ryan's printer; Protostar Hi-tack flocking is optional ($7.50 ✓) | — | owned printer | Some black filaments look grey in NIR, so check a test print through a NoIR camera. Stray light is the main enemy. |
| 22 | Screws and heat-set inserts | HEXPHANT M2 × 5 socket head, 50-pack, Amazon [B0GCT7TQCH](https://www.amazon.com/dp/B0GCT7TQCH); VGBUY 440-piece M3 kit, 25–45 mm, with nuts and washers, Amazon [B0D1XQCJPR](https://www.amazon.com/dp/B0D1XQCJPR); Pofsnnx 350 heat-set inserts, M2–M6, Amazon [B0FVKYJ83G](https://www.amazon.com/dp/B0FVKYJ83G) | $6.19 + $7.99 + $9.99 | ordered, due Oct 9 | The head's full hardware list is in [`cad/README.md`](../cad/README.md). The M3 kit starts at 25 mm, so the four M3 × 6 and three M3 × 8 come from row 23 or a parts bin. Insert holes in `cad/` are sized for these inserts (M2 2.85 mm, M3 4.5 mm). |
| 23 | Short M3 screws | VGBUY 600-piece M3 kit, 6–30 mm, Amazon [B0D1XJ766V](https://www.amazon.com/dp/B0D1XJ766V) | $9.99 ✓ | to buy, unless the bin has them | For the M3 × 6 and M3 × 8. |
| 24 | Glue and tape | Loctite Super Glue Gel Control, Amazon [B0006HUJCQ](https://www.amazon.com/dp/B0006HUJCQ); ½ in × 33 m polyimide (Kapton) tape, Amazon [B0DZCR4TKM](https://www.amazon.com/dp/B0DZCR4TKM) | $3.82 + $5.35 | ordered, due Oct 9 | Glue for the magnet, mirror and slit blades, tape for the grating film. Cure CA in open air: its fumes can haze the mirror. |

### 3.3 Compute, cameras and cables

| # | Part | Bought | Paid | Status | Notes |
|---|---|---|---|---|---|
| 25 | GPU computer (capture + CUDA splat renderer) | Ryan's Jetson Nano 4 GB developer kit | — | unconfirmed | The B01 board has a second camera port for the pose camera; the A02 has one. Which one isn't confirmed. The Nano is stuck on JetPack 4.6 (Ubuntu 18.04, CUDA 10.2), so ROS 2 runs in Docker ([`jetson/`](../jetson/)). From scratch: Jetson Orin Nano Super Developer Kit, $399 (NVIDIA raised it from $249 in July 2026), or a desktop GPU for training and a Raspberry Pi for capture. |
| 26 | Storage | The Nano's microSD card | — | owned, unconfirmed | From scratch: 64 GB microSD, ~$10, or a 256 GB NVMe SSD on an Orin Nano. |
| 27 | Spectrograph camera cable | Arducam 100 cm 15-pin ribbon, Amazon [B087FDJ2RP](https://www.amazon.com/dp/B087FDJ2RP), sold by UCTRONICS | $5.38 | ordered, due Oct 9 | Reaches from the head along the arm. Add strain relief where the arm bends. An Orin Nano or Pi 5 needs a 15-to-22-pin cable instead. |
| 28 | Pose camera cable | A second Arducam 100 cm ribbon, Amazon [B087FDJ2RP](https://www.amazon.com/dp/B087FDJ2RP) | $5.38 ✓ | to buy | |
| 29 | Pose camera | Ryan's Raspberry Pi NoIR camera v2 (IMX219) | — | owned | Reads the AprilTag board, which needs no color. IMX219 works natively on Jetson; the Camera Module 3 (IMX708) doesn't without extra drivers. |

### 3.4 Viewpoints

| # | Part | Bought | Paid | Status | Notes |
|---|---|---|---|---|---|
| 30 | Robot arm | Ryan's LeRobot SO-101 follower arm | — | owned | The scanner head replaces the gripper on the wrist-roll horn ([`cad/`](../cad/)). Pose comes from the tag board, not the arm, so a tripod moved by hand also works. From scratch: SO-ARM101 follower only (PartaBot, $299 ✓, sold out when checked) or the Seeed SO-ARM101 Pro motor kit ($277.99 ✓, no printed parts). |
| 31 | Tag board | AprilTag grid, printed and glued to foam board | ~$5 | to make | Gives one pose per viewpoint, then splatting refines it. |

### 3.5 Optional test gear

| # | Part | Bought | Paid | Status | Notes |
|---|---|---|---|---|---|
| — | Scope / logic analyzer | Ryan's Digilent Analog Discovery 3 | — | owned | Checks the camera exposure strobe against the mirror step timing. |
| 32 | Lab-grade slit and filter | Thorlabs **S50LK** (Ø1" mounted slit, 50 µm × 10 mm) and **FELH0500** (Ø25 mm long-pass) | — | not bought | About $100 each. They replace rows 7 and 9 on a bench build, but neither fits the printed head. Don't buy the S50RD slit, which is only 3 mm long. |

### 3.6 Totals

| Order | What | Paid |
|---|---|---|
| Amazon, due October 9 | Rows 5, 7, 10, 11, 12, 14, 15, 20, 22, 24 and 27 | $148.36 |
| Arducam's store, express | Rows 1 and 4, plus $47.00 express shipping | $99.99 |
| Commonlands | Rows 2 and 3, plus a $9.80 tariff fee; free shipping | $107.80 |
| Edmund Optics | Rows 6 and 9 | $69.75, plus $9.99 shipping on the field lens and tax |
| **Ordered** | | **$425.90** before tax and Edmund's shipping |
| Still to buy | Rows 23 and 28 from Amazon ($15.37); the lamps in rows 17 and 18 from Home Depot ($42.45) | about $58 |
| Only if needed | The grating in row 8 ($15.60), a second halogen light ($17.98), flocking ($7.50) | |

The controller boards in row 16 aren't priced yet. Everything else comes from Ryan's own stock. Building the whole rig from scratch with the generic picks in the notes comes to roughly $1,200, or about $525 with the budget picks and a tripod instead of the arm. The bench spectrometer stage (build step 1 below) needs only rows 1–2 and 4–9, the lamps in row 18, the laser in row 19, the housing in row 21, and a Raspberry Pi or Jetson to read the camera.

## 4. Build order

1. **Bench spectrometer.** Build the slit, field lens, collimator, grating and camera with no objective, pointed at the neon and CFL lamps. Fit the wavelength map and the smile/keystone warp. This is where you learn alignment.
2. **Line imager.** Add the objective and focus it on a printed target. Measure the line thickness on a knife edge.
3. **Scanner.** Add the mirror and stepper. Scan a flat Macbeth-style color card, then assemble the 2D datacube on the GPU (first CUDA kernel: warp + bin).
4. **3D.** Capture from several viewpoints (arm or tripod), get poses from the AprilTag board, and train the splat with the line-camera renderer.

## 5. Modeling notes and limits

- The M12 lenses are ideal (paraxial) in every config, since none of their prescriptions is published. The model is right about geometry, dispersion, smile, keystone and vignetting, but it can't predict blur, distortion or chromatic focus shift from the real M12 lenses. Commonlands says the CIL161 and CIL122 are IR-corrected, but gives no focus-shift figure. The LN016 makes no such claim. If one end of the spectrum comes out soft, tilt the sensor slightly (a Scheimpflug tilt).
- Configs D and E model the field lens as real N-BK7 surfaces and the GG-495 as a 1.52-index plate, so their numbers include those parts' aberrations. The spot sizes in `results.txt` come only from these parts.
- The stop and lens positions in D and E are `cad/`'s estimates, since the lens barrels and back focal distances aren't published. The light-at-the-line-end numbers depend on where the objective's real pupil sits, so treat them as a comparison between layouts, not a prediction. The paraxial objective puts its pupil at its principal plane. Measuring the lenses will move them.
- The CIL122's spec table says f/2.0, but the text on the same page says f/2.4. At f/2.4 its pupil would be 5 mm, not 6, and the band ends would lose a little more light.
- The IMX219's microlenses are shifted for a short lens whose rays reach the sensor corners at about 30°. Here the rays arrive within about 9° of square-on, so expect the sensor's edges to read darker; the flat field removes it.
- Spectral resolution in configs A to C is the paraxial estimate; D and E trace it (section 1 explains the difference).
- A good next Optiland exercise is swapping the paraxial collimator for a real catalog achromat (e.g. Thorlabs AC127-025-B) from Optiland's lens database and comparing the spot sizes.
- Silicon response falls steeply past 900 nm, so plan on a usable range of about 500–950 nm.
- To rerun or explore: `pip install optiland==0.6.2`, edit `CONFIGS` near the bottom of `optics/spectrograph_model.py`, then run `python3 optics/spectrograph_model.py`. It writes `results.txt` and the layout PNGs to `optics/model_output/`. CI checks that `results.txt` matches the script.

## 6. Orders

Each Amazon ASIN is the listing the shopping list linked; its title, seller and price match the order confirmation. Quantity is one of each listing.

**Amazon, ordered 2026-10-06, due 2026-10-09**

| Row | ASIN | Listing name, as ordered | Sold by | Paid |
|---|---|---|---|---|
| 10 | B0GW4WT69F | RUEHALF Front Surface Mirrors, 100 * 100 * 3mm First 1st Surface Mirror, Surface Mirror Optics Quality for Engineering and Scientific Project | PTG Merchant | $20.90 |
| 11 | B00PJYL6BY | STEPPERONLINE Nema 8 Bipolar Smallest Stepper Motor 1.6Ncm/2.3oz.in 1.8deg 28mm 0.2A 4 Leads | StepperOnline | $24.48 |
| 7 | B0FKTN8WJZ | 9mm Utility Knife Blades, Carbon Steel Snap Off Blade Refills, 25 PK | Mendota Merchants | $4.49 |
| 12 | B07ZPYKL46 | BIGTREETECH TMC2209 V1.3 Stepper Motor Driver, 2.8A UART/DIR/Step Mode | BIGTREETECH | $17.47 |
| 22 | B0GCT7TQCH | M2 x 5mm (50 Pack) Socket Head Cap Screws Bolts, 304 Stainless Steel 18-8, Allen Socket Drive, Full Thread, Bright Finish, with Hex Spanner (HEXPHANT) | hexphant US | $6.19 |
| 22 | B0D1XQCJPR | VGBUY 440pcs M3 Screw Assortment Kit, M3 Screws Bolts for 3D Printer DIY, Hex Socket Head Cap Screws Bolts Nuts Washer Kit, 304 Stainless Steel, 25/30/35/40/45mm | Valued Global BUY | $7.99 |
| 24 | B0DZCR4TKM | 1/2''×33M High Temperature Resistant Polyimide Tape, No Residue Heat Tape | XINHONG-TAPE | $5.35 |
| 27 | B087FDJ2RP | Arducam Pi Camera Cable, Octoprint Octopi Webcam, Monitor 3D Printer, 3.28FT/100CM Long Extension Flex Ribbon Cable for Raspberry Pi, Black | UCTRONICS | $5.38 |
| 15 | B07QS6PN3B | Gikfun A3144/OH3144/AH3144E Hall Effect Sensor Magnetic Detector for Arduino (Pack of 20pcs) EK1325 | Gikfun_Official_Store | $7.58 |
| 22 | B0FVKYJ83G | Pofsnnx 350Pcs Threaded Inserts, M2-M6 Heat Set Inserts for 3D Printing | Pofsnnx Direct | $9.99 |
| 14 | B07VQGHSWY | ALITOVE 12V DC Power Supply 3A 36W Universal AC Adapter 100-240V 50-60hz to 12 Volt Power Adapter Cord 3Amp 2.5A 2A 5.5mm x 2.5mm 2.1mm for LED Strip Light CCTV Security Camera PC Monitor and More | ALITOVE | $9.97 |
| 7 | B081SYZ1YV | Spurtar Feeler Gauges 0.0015-0.035'' (0.04-0.88mm) 32-Blade Stainless Steel | Wittyware | $4.97 |
| 20 | B0GKGHX1W6 | Sasylvia 1 Pcs 12 x 6 Inch Virgin PTFE Sheet, 1/16" Thick | Chadusil | $11.99 |
| 5 | B00R1J42T8 | uxcell 10 Pcs S Mount M12 Board Lens Holder 20mm Screw Spacing Black for CCTV Camera | uxcell | $7.79 |
| 24 | B0006HUJCQ | Loctite Super Glue Gel Control, Clear, 0.14 fl oz Bottle, 1 Pack | Amazon.com | $3.82 |

**Arducam's store, ordered 2026-10-06, express**

| Row | Part no. | Listing name | Paid |
|---|---|---|---|
| 1 | B0152 | Arducam NOIR 8 MP Sony IMX219 camera module with M12 lens LS1820 for Raspberry Pi 4/3B+/3 | $35.00 |
| 4 | LN016 (M2025ZH01) | Arducam 1/2" M12 mount 25mm Focal Length Camera Lens M2025ZH01 | $17.99 |
| | | Express shipping | $47.00 |

**Commonlands, ordered 2026-10-06, free USPS shipping**

| Row | Part no. | Listing name | Paid |
|---|---|---|---|
| 3 | CIL161-F2.0-M12ANIR (ordered as -M12A650, swapped 2026-10-07) | IR Corrected 16mm M12 Lens, F/2.0 / M12A / Without Filter | $39.00 |
| 2 | CIL122-F2.0-M12ANIR (ordered as -M12A650, swapped 2026-10-07) | IR Corrected 12mm M12 Lens, F/2.0 / M12A / Without Filter | $59.00 |
| | | Tariff fee | $9.80 |

**Edmund Optics, ordered by 2026-10-07**

| Row | Part no. | Listing name | Paid |
|---|---|---|---|
| 6 | #49-840 | 12.7mm Dia. x 15.0mm FL, Uncoated, Plano-Convex Lens | $32.75, plus $9.99 ground shipping |
| 9 | #54-652 | GG-495 12.5mm Dia. Longpass Filter | $37.00 |

**Planned once, then replaced**

| Planned | Replaced by | Why |
|---|---|---|
| Commonlands CIL161 and CIL122 -M12A650 | -M12ANIR, no filter | The 650 nm IR-cut filter would block 650–1000 nm. |
| Edmund #43-872 precut 20 × 20 mm mirror | RUEHALF sheet, cut by hand | Backordered. |
| Pibiger B0CGPT72NS board and Commonlands CIL251 collimator | Arducam B0152 and LN016 from Arducam's store | UCTRONICS quoted 4–6 weeks for the B0152. |
| UCTRONICS U0756M10 metal M12 holders | uxcell B00R1J42T8 plastic holders | Amazon Prime. |
| 5 × 2 mm magnets | Ryan's own Ø5 × 1.6 mm magnets | Already owned. |
| M2 × 4 and M2 × 6 screws | M2 × 5, filed 1 mm shorter if the motor's holes are under 3.2 mm deep | One pack covers every M2 screw. |

## Sources

- [Arducam B0152 (UCTRONICS)](https://www.uctronics.com/arducam-noir-8-mp-sony-imx219-camera-module-with-m12-lens-ls1820-for-raspberry-pi.html), [Arducam NoIR IMX219 with M12 lens (RobotShop)](https://www.robotshop.com/products/arducam-noir-8-mp-sony-imx219-camera-module-m12-lens-ls1820)
- [Arducam B0183 (IR-cut, lens not swappable)](https://www.arducam.com/b0183-arducam-imx219-distortioin-m12-mount-camera-module-raspberry-pi-compute-module.html)
- [Commonlands CIL161 16 mm IR-corrected](https://commonlands.com/products/ir-corrected-16mm-m12-lens-cil161), [CIL122 12 mm IR-corrected](https://commonlands.com/products/ir-corrected-12mm-m12-lens)
- [Arducam LN016 25 mm (UCTRONICS)](https://www.uctronics.com/m12-mount-mtv2520b-25mm-focal-length-camera-lens-2219.html), [LN065 12 mm](https://www.uctronics.com/arducam-12mm-m12-lens-m2512zh03-for-usb-camera.html), [LN001 16 mm (has IR-cut, avoid)](https://www.uctronics.com/arducam-1/2.5-m12-mount-16mm-focal-length-camera-lens-m2016zh01.html)
- [Edmund #49-840 field lens](https://www.edmundoptics.com/p/127mm-dia-x-150mm-fl-uncoated-plano-convex-lens/10312/), [Edmund #54-652 GG-495 filter](https://www.edmundoptics.com/p/gg-495-125mm-dia-longpass-filter/11321/), [SCHOTT GG495 data](https://www.schott.com/shop/advanced-optics/en/Matt-Filter-Plates/GG495/c/glass-GG495), [Edmund #43-872 mirror](https://www.edmundoptics.com/p/20-x-20mm-enhanced-aluminum-4-6lambda-mirror/5359/), [Edmund #54-509 grating film](https://www.edmundoptics.com/p/12700-linesinch-6quot-x-12quot-sheets-2pack/11226/)
- [Raspberry Pi camera hardware (IMX219)](https://www.raspberrypi.com/documentation/accessories/camera.html)
- [Bartovation 500 l/mm grating sheet](https://bartovation.com/product/other-lab-supplies/diffraction-grating-sheets/500-lines-mm-linear-diffraction-grating-sheet/)
- [Thorlabs precision slits](https://www.thorlabs.com/newgrouppage9.cfm?objectgroup_id=1464), [S50LK](https://www.thorlabs.com/item/S50LK)
- [Thorlabs hard-coated edgepass filters (FELH0500)](https://www.thorlabs.com/hard-coated-edgepass-filters?pn=FEL+H0500)
- [StepperOnline 8HS11-0204S (NEMA 8)](https://www.omc-stepperonline.com/nema-8-bipolar-1-8deg-1-4ncm-1-98oz-in-0-2a-20x20x28mm-4-wires-8hs11-0204s)
- [Adafruit TMC2209 breakout](https://www.adafruit.com/product/6121)
- [Espressif ESP32-DevKitC-32E (Digi-Key)](https://www.digikey.com/en/products/detail/espressif-systems/ESP32-DEVKITC-32E/12091810)
- [Jetson price increase, Hardware Busters, July 2026](https://hwbusters.com/news/nvidia-jetson-prices-jump-up-to-101-the-249-orin-nano-super-is-now-399/), [NVIDIA: buy Jetson](https://developer.nvidia.com/buy-jetson)
- [SO-ARM101 follower only (PartaBot)](https://partabot.com/products/so-arm101-follower-only), [Seeed SO-ARM101 Pro motor kit](https://www.seeedstudio.com/SO-101-Low-Cost-AI-Arm-Kit-Pro-p-6427.html)
- [Digilent Analog Discovery 3](https://digilent.com/shop/analog-discovery-3/)
