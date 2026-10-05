# Line-Scan Hyperspectral 3D Gaussian Splatting

A low-cost pushbroom hyperspectral scanner on a robot arm, feeding a 3D Gaussian splat where every Gaussian carries a full spectrum instead of an RGB color.

## Hardware

**Pushbroom spectrograph.** Light enters through a slit, is collimated by a 16 mm f/4 M12 objective, dispersed by a 500 lines/mm transmission grating, and refocused by a 25 mm M12 lens used backward as the camera-side collimator. A 500 nm long-pass filter blocks second-order overlap. A Raspberry Pi NoIR camera sits tilted to the first diffraction order, so each frame is one spatial line by many wavelengths.

**Line sweep.** A stepper-driven scan mirror sweeps the slit across the scene to build up a hyperspectral cube one line at a time.

**Viewpoints.** The scanner rides on a LeRobot SO-101 arm, which moves it around the object. An RGB camera and a fiducial tag board give the pose of each scan.

## Reconstruction

The splat is trained directly from scan lines rather than from assembled image cubes. Each Gaussian stores a spectrum, and the renderer models the instrument as a line camera: for each pose it draws a single line, which is compared against the measured scan line.

## Planned stack

- **Renderer:** C++/CUDA line-camera Gaussian splat renderer running on a Jetson Nano
- **Robotics:** ROS2 for the arm and the scan mirror
- **Optics:** Optiland for the optical model of the spectrograph
- **Capture:** Python for the Pi camera
