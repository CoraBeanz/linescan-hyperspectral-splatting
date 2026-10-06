<p align="center">
  <img src="docs/img/hero.svg" width="100%" alt="Line-Scan Hyperspectral 3D Gaussian Splatting. An SO-101 robot arm holds a spectrograph whose scan line sweeps across a potted plant; behind the line the plant turns into Gaussians, and one Gaussian's stored spectrum is shown in a callout.">
</p>

<h3 align="center">A homebuilt pushbroom spectrograph rides a robot arm, and the 3D Gaussian splat it trains stores a full 500–950 nm spectrum in every Gaussian instead of an RGB color.</h3>

<p align="center">
  <img alt="status: design and modeling" src="https://img.shields.io/badge/status-design%20%26%20modeling-f59e0b?style=for-the-badge">
  <img alt="band: 500 to 950 nm" src="https://img.shields.io/badge/band-500%E2%80%93950%20nm-a855f7?style=for-the-badge">
  <img alt="optics: Optiland" src="https://img.shields.io/badge/optics-Optiland-0ea5e9?style=for-the-badge&logo=python&logoColor=white">
  <img alt="renderer: C++ and CUDA" src="https://img.shields.io/badge/renderer-C%2B%2B%20%2F%20CUDA-76b900?style=for-the-badge&logo=nvidia&logoColor=white">
  <img alt="robotics: ROS 2 and SO-101" src="https://img.shields.io/badge/robotics-ROS%202%20%2B%20SO--101-22314e?style=for-the-badge&logo=ros&logoColor=white">
</p>

<p align="center">
  <a href="#how-it-works"><b>How it works</b></a> ·
  <a href="#the-rig"><b>The rig</b></a> ·
  <a href="#optical-path"><b>Optical path</b></a> ·
  <a href="#optiland-model"><b>Optiland model</b></a> ·
  <a href="#reconstruction"><b>Reconstruction</b></a> ·
  <a href="#roadmap"><b>Roadmap</b></a> ·
  <a href="docs/parts_list_and_design.md"><b>Parts list</b></a>
</p>

## How it works

A hyperspectral camera records a whole spectrum at every pixel instead of three color channels, which shows things RGB can't, like the sharp rise in a leaf's reflectance just past red. This one is built from M12 board lenses, grating film and a Raspberry Pi camera sensor, for under $500 in optics and scanner parts.

| 1 · Pick a viewpoint | 2 · Sweep the slit | 3 · Train the splat |
|---|---|---|
| The SO-101 arm carries the scanner head to a pose and holds still. An RGB camera on the head reads an AprilTag board to get that pose. | A 0.9° stepper turns the scan mirror one step per frame. Each frame is one line of the object by its spectrum, and about 150 lines cover a 63 × 42 mm patch. | Every scan line is a training sample. A C++/CUDA renderer draws the splat through a line-camera model at that line's pose and compares it with the measured line. |

## The rig

<p align="center">
  <img src="docs/img/rig.svg" width="100%" alt="Side view of the setup: the SO-101 arm holds the scanner head above a potted plant on an AprilTag board, a scan mirror sweeps a line across the plant, halogen lamps light it, and a Jetson Nano and an ESP32 sit on the table. An inset shows the parts inside the scanner head.">
</p>

<table>
  <tr>
    <td width="42%" valign="top">
      <img src="docs/img/SO101_Follower.webp" width="100%" alt="Photo of an assembled SO-101 follower arm">
      <br><sub>SO-101 follower arm. Photo: <a href="https://github.com/TheRobotStudio/SO-ARM100">TheRobotStudio/SO-ARM100</a>, Apache-2.0.</sub>
    </td>
    <td valign="top">

**Arm.** An [SO-101](https://huggingface.co/docs/lerobot/so101) follower from [LeRobot](https://github.com/huggingface/lerobot), with six STS3215 servos. It only sets the viewpoint, then holds still while the mirror scans.

**Scanner head.** The spectrograph below, plus a first-surface scan mirror on a 0.9° NEMA17 stepper. A TMC2209 driver runs it from an ESP32 on micro-ROS.

**Pose.** An RGB camera on the head sees an AprilTag board under the object.

**Light.** Halogen lamps, because LEDs give almost nothing past 700 nm.

**Compute.** A Jetson Nano for capture, the calibration warp and the CUDA splat renderer.

  </td>
  </tr>
</table>

## Optical path

<p align="center">
  <img src="docs/img/optical_path.svg" width="100%" alt="Optical path, side view in the dispersion plane: light from one line of the object reflects off the scan mirror, passes a 500 nm long-pass filter and a 16 mm objective, focuses on a 50 micron slit with a field lens behind it, is collimated by a 25 mm lens, fanned out by a 500 lines per mm grating and focused by a 12 mm lens tilted 22 degrees onto an IMX219 NoIR sensor. A strip at the bottom gives the real distances between the parts.">
</p>

A 16 mm f/4 objective images the object onto a 50 µm slit, so the instrument sees one line at a time. A field lens right behind the slit steers every point of that line toward the camera. A 25 mm lens, used backward, collimates the light, and a 500 lines/mm transmission grating fans it out by wavelength. A 12 mm f/2 lens, tilted 22° to the first diffraction order, refocuses the fan onto an IMX219 NoIR sensor, so each frame is one line of the object by its spectrum. A 500 nm long-pass filter in front keeps second-order light off the near-infrared end. Why each part is there: [final optical train](docs/parts_list_and_design.md#2-final-optical-train-config-c).

### What the camera sees

<p align="center">
  <img src="docs/img/sensor_frame.png" width="100%" alt="Simulated IMX219 frames from the Optiland model: a white target under a halogen lamp shows a smooth band from green to near-infrared, a neon lamp shows curved emission lines, and a zoom shows the smile of the 703 nm neon line.">
</p>

Each frame is the scan line (vertical) by its spectrum (horizontal), at about 5.8 px per nm. Spectral lines bow slightly across the frame (smile, up to about 33 px at the line ends) and the line length shifts a little with wavelength (keystone, about 11 px). Both get calibrated once with lamp lines and removed by a warp.

## Optiland model

[`optics/spectrograph_model.py`](optics/spectrograph_model.py) builds the whole train in [Optiland](https://github.com/HarrisonKramer/optiland) with ideal thin lenses and traces it. Tracing changed the design twice: the stock Pi NoIR lens was far too small to catch the collimated beam, and without a field lens most of the light from the ends of the line missed the camera lens.

<p align="center">
  <img src="docs/img/optiland_dispersion.png" width="100%" alt="Optiland ray trace of config C in the dispersion plane: white light from the slit is collimated, fanned out by the grating from 500 to 1000 nm and refocused onto the sensor by the camera lens tilted 22 degrees.">
</p>

<p align="center">
  <img src="docs/img/optiland_field_lens.png" width="100%" alt="Optiland traces of configs A, B and C in the slit plane. Light from the ends of the scan line gets clipped in A and B; with the field lens in C, 98 to 99 percent of it reaches the sensor.">
</p>

| Config (ideal lenses) | Light at line center | Light at line ends | Dispersion | Spectral resolution | Pixels along the line |
|---|---|---|---|---|---|
| A: stock Pi NoIR v2 lens | 8% | 0% | 1.5 px/nm | 3.7 nm | 543 |
| B: M12 IMX219 + 12 mm f/2 lens | 92–100% | 12–14% | 5.8 px/nm | 3.7 nm | 2,143 |
| **C: B + 18 mm field lens** | **100%** | **98–99%** | **5.8 px/nm** | **3.7 nm** | **2,204** |

Resolution is geometric, with perfect lenses; real M12 lenses will likely land around 5–8 nm. More in the [modeling notes](docs/parts_list_and_design.md#5-modeling-notes-and-limits).

```bash
pip install optiland                   # tested with 0.6.2
python optics/spectrograph_model.py    # prints the table, writes optics/model_output/
python optics/readme_figures.py        # redraws the Optiland figures in docs/img/
```

Full output: [`optics/model_output/results.txt`](optics/model_output/results.txt) and the [ray layouts](optics/model_output/).

## Reconstruction

Published hyperspectral splatting, such as [HyperGS](https://openaccess.thecvf.com/content/CVPR2025/papers/Thirgood_HyperGS_Hyperspectral_3D_Gaussian_Splatting_CVPR_2025_paper.pdf) and [DD-HGS](https://arxiv.org/abs/2505.21890), trains on full multi-view hyperspectral images. Here the splat trains straight from raw scan lines, through a model of the instrument itself.

- **Spectral Gaussians.** Each Gaussian keeps the usual position, shape and opacity, and stores a spectrum over 500–950 nm where a normal splat stores RGB.
- **A line camera, not a pinhole.** The slit only sees a thin fan of rays, and the mirror angle tilts that fan. For each scan line, the renderer draws the splat along that one fan and compares the resulting line of spectra with the measured line.
- **Poses get refined too.** The arm and the tag board give each viewpoint a pose, and the optimizer can adjust those poses along with the Gaussians, so small arm errors don't blur the result.
- **Calibrated input.** Neon and CFL lamp lines fix the wavelength scale, the warp removes smile and keystone, and a white PTFE reference turns counts into reflectance.

Splatting through a non-pinhole camera has precedent in satellite imagery ([RPC-GS](https://arxiv.org/abs/2606.06690)), and close-range pushbroom cameras have been calibrated for plant phenotyping ([Behmann et al. 2015](https://doi.org/10.1016/j.isprsjprs.2015.05.010)). This project puts the two together on hobby hardware.

## Roadmap

- [x] First-order optical design and Optiland model (configs A, B, C)
- [ ] **Bench spectrometer:** slit, field lens, collimator, grating and camera aimed at neon and CFL lamps; fit the wavelength map and the smile/keystone warp
- [ ] **Line imager:** add the objective, focus it on a printed target, measure the line on a knife edge
- [ ] **Scanner:** add the mirror and stepper, scan a color card, assemble the datacube on the Jetson (first CUDA kernel: warp + bin)
- [ ] **3D:** arm viewpoints, AprilTag poses and the line-camera splat renderer

Details for each step are in the [build order](docs/parts_list_and_design.md#4-build-order).

## Stack

| Layer | Plan |
|---|---|
| **Optics** | Python and [Optiland](https://github.com/HarrisonKramer/optiland): ideal-lens model now, catalog lenses next |
| **Capture** | Python, IMX219 NoIR on the Jetson Nano's CSI port |
| **Robotics** | ROS 2 for the arm and the scan mirror; micro-ROS on an ESP32 driving a TMC2209 |
| **Reconstruction** | C++/CUDA line-camera Gaussian splat renderer on the Jetson Nano |

## Repo map

```
docs/
├── parts_list_and_design.md   parts with vendors and prices, first-order design, build order
├── optical_train_v2.svg       optical train drawing (v1 kept alongside)
└── img/                       README figures; img/src/ holds the scripts that draw the SVGs
optics/
├── spectrograph_model.py      Optiland model of the spectrograph, configs A–C
├── readme_figures.py          traces the model and renders the Optiland figures
└── model_output/              results.txt and ray layouts
```

## Background reading

- Kerbl et al., [3D Gaussian Splatting for Real-Time Radiance Field Rendering](https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/), ACM TOG (SIGGRAPH) 2023. [arXiv:2308.04079](https://arxiv.org/abs/2308.04079)
- Thirgood et al., [HyperGS: Hyperspectral 3D Gaussian Splatting](https://openaccess.thecvf.com/content/CVPR2025/papers/Thirgood_HyperGS_Hyperspectral_3D_Gaussian_Splatting_CVPR_2025_paper.pdf), CVPR 2025
- Narayanan et al., [Diffusion-Denoised Hyperspectral Gaussian Splatting](https://arxiv.org/abs/2505.21890) (first posted as *Hyperspectral Gaussian Splatting*), arXiv 2025
- Wagner et al., [RPC-GS: Gaussian Splatting with native RPC Rendering for Satellite Imagery](https://arxiv.org/abs/2606.06690), arXiv 2026
- Behmann et al., [Calibration of hyperspectral close-range pushbroom cameras for plant phenotyping](https://doi.org/10.1016/j.isprsjprs.2015.05.010), ISPRS Journal of Photogrammetry and Remote Sensing 106, 2015

## Credits

- SO-101 photo: [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) (`media/SO101_Follower.webp`), used unmodified under the Apache License 2.0; a copy of the license is in [`docs/img/SO-ARM100_LICENSE.txt`](docs/img/SO-ARM100_LICENSE.txt). The SO-101 is designed by The Robot Studio in collaboration with Hugging Face.
- Ray tracing: [Optiland](https://github.com/HarrisonKramer/optiland) (MIT).
