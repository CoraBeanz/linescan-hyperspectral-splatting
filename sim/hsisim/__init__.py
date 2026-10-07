"""hsisim: a digital twin of the line-scan spectrograph on the SO-101 arm.

It renders the raw IMX219 frames the spectrograph would see while the arm and the scan
mirror sweep a scan line over a known hyperspectral scene, and writes them as a whole scan
session in the rig's on-disk format, with the truth they were made from.

    python -m hsisim scan sim_out              # a scan session of the relief target
    python -m hsisim selftest                  # scan, calibrate with hsical, compare with the truth

Modules, from the scene to the session:

  spectra      the spectral library: the splat's materials, PTFE, a rare-earth tile
  scene        solids, textures, lighting and ray casting
  scenes       the built-in scenes (relief, board, white)
  linecam      the objective and slit: the rays one scan line traces into the scene
  instrument   hsical's synthetic spectrograph and IMX219, rendering scene light
  arm          true and logged poses: servo offsets and ticks, arm flex, mirror errors
  session      a whole scan session on disk
  truth        reading back what a session was made from
  evaluate     the end-to-end check against the truth
  preview      pictures of a session
"""

__version__ = "0.1.0"
