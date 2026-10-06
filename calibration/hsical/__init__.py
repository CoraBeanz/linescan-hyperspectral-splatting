"""hsical: calibration kit for the line-scan hyperspectral spectrograph.

Turns raw IMX219 frames of a few lamps into a wavelength map, a smile and
keystone warp, and a spectral response, then applies them to scan frames.
Run ``python -m hsical --help`` from the calibration/ folder.

This file stays importable on Python 3.6 (JetPack 4's Python), so that
``python -m hsical capture`` works on the Jetson Nano.
"""

__version__ = "0.1.0"
