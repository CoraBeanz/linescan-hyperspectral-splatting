"""How the code fits together (docs/img/software.svg): firmware, ROS 2, calibration, the
splat trainer and the viewer, what each runs on, what passes between them, and what CI
tests; below them, the design models and the simulator.

Run: python docs/img/src/software.py  [out.svg]   (default: docs/img/software.svg)
"""
import sys
from pathlib import Path
from common import *

W, H = 1280, 728
OUT = sys.argv[1] if len(sys.argv) > 1 else str(Path(__file__).resolve().parent.parent / "software.svg")
def f(x): return f"{x:.1f}".rstrip('0').rstrip('.')

FW_C, ROS_C, CAL_C, SPLAT_C, VIEW_C = "#fbbf24", LENS, GRATING, SENSOR, "#fb7185"
ARROW = "#6b7699"

# columns and rows
C1, C2, C3 = (48, 290), (466, 340), (934, 298)
R1, R2, RH = 196, 420, 168

def card(x, y, w, h, color, folder, where, title, lines, test, accent=None):
    out = [f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="16" fill="#0e1631" fill-opacity="0.96" stroke="#2c3b6e"/>',
           f'<path d="M{x + 1.5} {y + 22} L{x + 1.5} {y + h - 22}" stroke="{accent or color}" stroke-width="3" stroke-linecap="round"/>',
           f'<text x="{x + 22}" y="{y + 32}" class="mono" style="fill:{color}">{folder}<tspan class="ls" dx="8">· {where}</tspan></text>',
           f'<text x="{x + 22}" y="{y + 62}" class="lt">{title}</text>']
    for i, line in enumerate(lines):
        out.append(f'<text x="{x + 22}" y="{y + 92 + 21 * i}" class="desc">{line}</text>')
    ty = y + h - 24
    out.append(f'<circle cx="{x + 30}" cy="{ty - 5}" r="8" fill="{SENSOR}" fill-opacity="0.16" stroke="{SENSOR}" stroke-width="1.4"/>'
               f'<path d="M{x + 26} {ty - 5} L{x + 29} {ty - 2} L{x + 34.5} {ty - 8.5}" fill="none" stroke="{SENSOR}" '
               f'stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"/>'
               f'<text x="{x + 46}" y="{ty}" class="test">{test}</text>')
    return "\n  ".join(out)

def arrow(d, both=False):
    start = ' marker-start="url(#ah)"' if both else ''
    return f'<path d="{d}" fill="none" stroke="{ARROW}" stroke-width="2"{start} marker-end="url(#ah)"/>'

def camera(x, y, w, h):
    cx, cy = x + 52, y + h / 2
    pins = "".join(f'<rect x="{f(cx - 21 + 9 * i)}" y="{f(cy - 30)}" width="4" height="6" fill="#4b5f95"/>'
                   f'<rect x="{f(cx - 21 + 9 * i)}" y="{f(cy + 24)}" width="4" height="6" fill="#4b5f95"/>' for i in range(5))
    return "\n  ".join([
        f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="16" fill="#0e1631" fill-opacity="0.6" stroke="#2c3b6e" stroke-dasharray="5 5"/>',
        pins,
        f'<rect x="{f(cx - 26)}" y="{f(cy - 24)}" width="52" height="48" rx="6" fill="#151d38" stroke="#4b5f95" stroke-width="1.5"/>',
        f'<rect x="{f(cx - 14)}" y="{f(cy - 12)}" width="28" height="24" rx="2" fill="{SENSOR}" fill-opacity="0.28" stroke="{SENSOR}" stroke-width="1.2"/>',
        f'<text x="{x + 98}" y="{f(cy - 4)}" class="lt">IMX219 camera</text>',
        f'<text x="{x + 98}" y="{f(cy + 19)}" class="ls">behind the grating</text>',
    ])

def design_band(x, y, w, h):
    def pill(px, folder, name):
        return (f'<rect x="{px}" y="{y + 18}" width="150" height="{h - 36}" rx="12" fill="#121a33" stroke="#3c4c7c"/>'
                f'<text x="{px + 16}" y="{y + 37}" class="mono" style="fill:#c9d3f5">{folder}</text>'
                f'<text x="{px + 16}" y="{y + 56}" class="ls">{name}</text>')
    mid = y + h / 2
    return "\n  ".join([
        f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="16" fill="#0e1631" fill-opacity="0.5" stroke="#2c3b6e" stroke-dasharray="5 5"/>',
        pill(x + 22, "optics/", "Optiland model"),
        arrow(f"M{x + 178} {mid} L{x + 214} {mid}"),
        pill(x + 220, "cad/", "FreeCAD model"),
        arrow(f"M{x + 376} {mid} L{x + 412} {mid}"),
        pill(x + 418, "sim/", "digital twin"),
        f'<text x="{x + 600}" y="{y + 25}" class="band">The code takes its numbers from the design: the URDF and the scan</text>',
        f'<text x="{x + 600}" y="{y + 45}" class="band">model use cad/\'s head, calibration/ starts from the Optiland map, and</text>',
        f'<text x="{x + 600}" y="{y + 65}" class="band">sim/ renders whole scans from all of it, with known truth, to test on.</text>',
    ])

fw_mid = R1 + RH / 2
cam = (C1[0], R2 + 32, C1[1], RH - 64)
cam_hi, cam_lo = cam[1] + 28, cam[1] + cam[3] - 28
gap1 = (C1[0] + C1[1] + C2[0]) / 2
gap2 = (C2[0] + C2[1] + C3[0]) / 2
col2_mid = C2[0] + C2[1] / 2
col3_mid = C3[0] + C3[1] / 2

svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" role="img" aria-labelledby="t d">
  <title id="t">How the software fits together</title>
  <desc id="d">Block diagram of the code. The scan-mirror firmware on the ESP32 talks to the ROS 2 scan arm on the Jetson Nano in plain text over USB serial and reports every scan line. The IMX219 camera's frames go to the ROS 2 side during scans and to the calibration kit as lamp frames; the calibration kit gives the ROS 2 side a wavelength map and a warp. The ROS 2 side passes calibrated lines, each with a pose, to the line-camera splat trainer, which stores a spectrum in every Gaussian. The splat goes to a browser viewer as one file. Each module lists what its tests cover. A band at the bottom shows the Optiland and FreeCAD models whose numbers the code uses, and the simulator that renders whole scans from them to test on.</desc>
  <style>
    text {{ font-family: {FONT}; }}
    .h1 {{ font-size: 32px; font-weight: 800; fill: #f1f5ff; }}
    .sub {{ font-size: 17px; fill: #a3acc9; }}
    .lt {{ font-size: 19px; font-weight: 700; fill: #e6e9f2; }}
    .ls {{ font-size: 15px; fill: #8b93b0; font-family: {FONT}; }}
    .desc {{ font-size: 15.5px; fill: #c9d3f5; }}
    .test {{ font-size: 14.5px; fill: #8fd9b6; }}
    .small {{ font-size: 14px; fill: #8b93b0; font-style: italic; }}
    .mono {{ font-size: 14.5px; font-weight: 700; fill: #e6e9f2; font-family: {MONO}; }}
    .data {{ font-size: 14px; fill: #e6e9f2; font-family: {MONO}; }}
    .band {{ font-size: 15px; fill: #a3acc9; }}
  </style>
  <defs>
    <linearGradient id="bg" x1="0" y1="0" x2="1" y2="1">
      <stop offset="0" stop-color="#101a3a"/><stop offset="0.55" stop-color="#0a1024"/><stop offset="1" stop-color="#060914"/>
    </linearGradient>
    <pattern id="holes" width="28" height="28" patternUnits="userSpaceOnUse"><circle cx="14" cy="14" r="1.7" fill="#1d2955"/></pattern>
    <radialGradient id="fade" cx="0.5" cy="0.55" r="0.75"><stop offset="0" stop-color="#fff"/><stop offset="1" stop-color="#fff" stop-opacity="0"/></radialGradient>
    <mask id="holesMask"><rect width="{W}" height="{H}" fill="url(#fade)"/></mask>
    <marker id="ah" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0 0 L10 5 L0 10z" fill="{ARROW}"/></marker>
    <linearGradient id="specV" x1="0" y1="{R2 + 22}" x2="0" y2="{R2 + RH - 22}" gradientUnits="userSpaceOnUse">{spectrum_stops(500, 950)}</linearGradient>
  </defs>

  <rect width="{W}" height="{H}" rx="26" fill="url(#bg)"/>
  <rect width="{W}" height="{H}" rx="26" fill="url(#holes)" mask="url(#holesMask)"/>

  <text x="48" y="68" class="h1">The software</text>
  <text x="48" y="102" class="sub">Five modules take a scan from the mirror to a splat you can open in a browser.</text>
  <text x="48" y="127" class="sub">CI builds each one and runs its tests on every pull request, on simulated hardware.</text>

  {card(C1[0], R1, C1[1], RH, FW_C, "firmware/", "ESP32", "Scan-mirror firmware",
        ["Steps the mirror on the camera's", "frame clock, stamps every line."], "a simulated rig, ASan and UBSan")}
  {card(C2[0], R1, C2[1], RH, ROS_C, "ros2/", "Jetson Nano, Humble", "ROS 2 scan arm",
        ["Moves the arm, sweeps the mirror,", "and gives every scan line a pose."], "fake servos and mirror, a whole scan")}
  {card(C2[0], R2, C2[1], RH, CAL_C, "calibration/", "Jetson and PC", "Calibration kit",
        ["Lamp lines give each pixel's wavelength,", "and a warp removes smile and keystone."], "synthetic sessions with known answers")}
  {card(C3[0], R1, C3[1], RH, SPLAT_C, "splat/", "C++ and CUDA", "Line-camera splat",
        ["Trains spectral Gaussians on the", "scan lines and refines the poses."], "gradients, CUDA against the CPU")}
  {card(C3[0], R2, C3[1], RH, VIEW_C, "viewer/", "WebGL2", "Splat viewer",
        ["Any wavelength, true color or NDVI,", "and the spectrum of any point."], "against the C++ renderer", accent="url(#specV)")}
  {camera(*cam)}

  {arrow(f"M{C1[0] + C1[1] + 4} {fw_mid} L{C2[0] - 4} {fw_mid}", both=True)}
  <text x="{f(gap1)}" y="{fw_mid - 12}" text-anchor="middle" class="data">EV LINE</text>
  <text x="{f(gap1)}" y="{fw_mid + 26}" text-anchor="middle" class="small">serial, text</text>

  {arrow(f"M{C2[0] + C2[1] + 4} {fw_mid} L{C3[0] - 4} {fw_mid}")}
  <text x="{f(gap2)}" y="{fw_mid - 12}" text-anchor="middle" class="data">lines</text>
  <text x="{f(gap2)}" y="{fw_mid + 26}" text-anchor="middle" class="small">with poses</text>

  {arrow(f"M{C1[0] + C1[1] + 4} {cam_hi} L{f(gap1)} {cam_hi} L{f(gap1)} {R1 + RH - 34} L{C2[0] - 4} {R1 + RH - 34}")}
  <text x="{f(gap1 - 10)}" y="{f((cam_hi + R1 + RH) / 2 + 4)}" text-anchor="end" class="small">scan frames</text>
  {arrow(f"M{C1[0] + C1[1] + 4} {cam_lo} L{C2[0] - 4} {cam_lo}")}
  <text x="{f(gap1)}" y="{cam_lo - 10}" text-anchor="middle" class="small">lamp frames</text>

  {arrow(f"M{f(col2_mid)} {R2 - 4} L{f(col2_mid)} {R1 + RH + 4}")}
  <text x="{f(col2_mid + 14)}" y="{f((R1 + RH + R2) / 2 + 5)}" class="small">wavelength map and warp</text>

  {arrow(f"M{f(col3_mid)} {R1 + RH + 4} L{f(col3_mid)} {R2 - 4}")}
  <text x="{f(col3_mid + 14)}" y="{f((R1 + RH + R2) / 2 + 5)}" class="data">.lsplat</text>

  {design_band(48, 620, 1184, 80)}
</svg>
'''
open(OUT, "w", encoding="utf-8", newline="\n").write(svg)
print("wrote", OUT, len(svg))
