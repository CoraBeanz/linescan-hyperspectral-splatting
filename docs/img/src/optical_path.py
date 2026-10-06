"""Optical path schematic (config C): object -> scan mirror -> long-pass -> objective -> slit + field
lens -> collimator -> grating -> tilted camera lens -> IMX219. Dispersion-plane view, not to scale.
Rays are traced through ideal thin lenses in drawing units so the picture is self-consistent;
the dispersion angles after the grating are exaggerated 1.7x so the fan is readable."""
import math, sys
from pathlib import Path
from common import *

W, H = 1280, 736
OUT = sys.argv[1] if len(sys.argv) > 1 else str(Path(__file__).resolve().parent.parent / "optical_path.svg")
def f(x): return f"{x:.1f}".rstrip('0').rstrip('.')

# ---------- layout (drawing units) ----------
AY = 300                       # optical axis
XM = 185                       # scan mirror (also x of the object line)
YO = 548                       # object line (seen end-on: it runs into the page)
XF, XO, XS, XC, XG = 268, 352, 500, 650, 790   # filter, objective, slit, collimator, grating
XFL = XS + 12                  # field lens right behind the slit
THETA_C = math.degrees(math.asin(0.75 / 2.0))   # camera axis: 1st order at 750 nm, 500 l/mm
L_CAM, F_CAM = 70, 190         # grating -> camera lens, camera lens -> sensor
EXAG = 1.7
WLS = [500, 550, 600, 650, 700, 750, 800, 850, 900, 950]

def norm(v):
    n = math.hypot(*v); return (v[0] / n, v[1] / n)
def add(a, b, s=1.0): return (a[0] + s * b[0], a[1] + s * b[1])
def dot(a, b): return a[0] * b[0] + a[1] * b[1]

def hit_plane(p, d, c, a):
    """Ray p + t d meets the plane through c with normal a."""
    t = dot((c[0] - p[0], c[1] - p[1]), a) / dot(d, a)
    return add(p, d, t)

def thin_lens(p, d, c, a, fl):
    """Ideal thin lens at c with axis a: rays sharing a direction meet on the focal plane."""
    q = hit_plane(p, d, c, a)
    focus = add(c, d, fl / dot(d, a))
    return q, norm((focus[0] - q[0], focus[1] - q[1]))

def reflect(d, n):
    k = 2 * dot(d, n); return (d[0] - k * n[0], d[1] - k * n[1])

AX = (1.0, 0.0)
D_OBJ = (YO - AY) + (XO - XM)                     # object -> objective, along the chief ray
F_OBJ = 1 / (1 / D_OBJ + 1 / (XS - XO))           # images the object line onto the slit
F_COL = XC - XS                                   # slit sits in the collimator's front focal plane
HALF_AP = 25                                      # bundle half-width at the objective
S_MAX = HALF_AP / D_OBJ
MIRROR_N = norm((1, 1))                           # "/" mirror, reflecting side faces down-right

ta = math.radians(THETA_C)
A_CAM = (math.cos(ta), -math.sin(ta))
B_CAM = (math.sin(ta), math.cos(ta))              # along the sensor
C_CAM = add((XG, AY), A_CAM, L_CAM)
SEN = add(C_CAM, A_CAM, F_CAM)

def trace_white(s):
    """One ray from the object point with lateral slope s; returns the polyline up to the grating."""
    p, d = (XM, YO), norm((s, -1))
    m = hit_plane(p, d, (XM, AY), MIRROR_N)
    d = reflect(d, MIRROR_N)
    q1, d = thin_lens(m, d, (XO, AY), AX, F_OBJ)
    slit = hit_plane(q1, d, (XS, AY), AX)
    q2, d = thin_lens(slit, d, (XC, AY), AX, F_COL)
    g = hit_plane(q2, d, (XG, AY), AX)
    return [p, m, q1, slit, q2, g]

def disp_dir(nm):
    th = math.degrees(math.asin(nm / 2000.0))
    phi = math.radians(THETA_C + EXAG * (th - THETA_C))
    return (math.cos(phi), -math.sin(phi))

def trace_color(g, nm):
    d = disp_dir(nm)
    q, d2 = thin_lens(g, d, C_CAM, A_CAM, F_CAM)
    s = hit_plane(q, d2, SEN, A_CAM)
    return [g, q, s]

def P(pts): return "M" + " L".join(f"{f(x)} {f(y)}" for x, y in pts)

# ---------- rays ----------
top, mid, bot = trace_white(-S_MAX), trace_white(0.0), trace_white(S_MAX)
white_poly = top + bot[::-1]
H_COL = abs(top[-1][1] - AY)                      # collimated half-height at the grating

def color_layers():
    fills, chiefs, flows = [], [], []
    for nm in WLS:
        c = wl_hex(nm)
        a = trace_color((XG, AY - H_COL), nm)
        b = trace_color((XG, AY + H_COL), nm)
        m = trace_color((XG, AY), nm)
        poly = [a[0], a[1], a[2], b[1], b[0]]
        fills.append(f'<path d="{P(poly)}Z" fill="{c}" fill-opacity="0.16"/>')
        chiefs.append(f'<path d="{P(a)}" stroke="{c}" stroke-opacity="0.55"/><path d="{P(b)}" stroke="{c}" stroke-opacity="0.55"/>'
                      f'<path d="{P(m)}" stroke="{c}" stroke-opacity="0.95" stroke-width="1.6"/>')
        flows.append(P(m))
    return "".join(fills), "".join(chiefs), flows

FILLS, CHIEFS, COLOR_FLOWS = color_layers()
SPOT = {nm: trace_color((XG, AY), nm)[2] for nm in WLS}

# ---------- element drawings ----------
def lens(x, y, h, w=7, color=LENS, rot=0.0, fill_op=0.16):
    return (f'<g transform="translate({f(x)} {f(y)}) rotate({f(rot)})">'
            f'<path d="M0 {-h} Q{w} 0 0 {h} Q{-w} 0 0 {-h}Z" fill="{color}" fill-opacity="{fill_op}" '
            f'stroke="{color}" stroke-width="1.8"/></g>')

def mirror():
    u = norm((1, -1))
    a, b = add((XM, AY), u, 44), add((XM, AY), u, -44)
    back = (-MIRROR_N[0] * 5, -MIRROR_N[1] * 5)
    a2, b2 = add(a, back), add(b, back)
    motor = add((XM, AY), MIRROR_N, -52)
    return (f'<line x1="{f(XM)}" y1="{f(AY)}" x2="{f(motor[0])}" y2="{f(motor[1])}" stroke="#4b5f95" stroke-width="4"/>'
            f'<rect x="{f(motor[0] - 24)}" y="{f(motor[1] - 24)}" width="48" height="48" rx="7" fill="#121a33" stroke="#4b5f95" stroke-width="1.5"/>'
            f'<circle cx="{f(motor[0])}" cy="{f(motor[1])}" r="8" fill="#2b3a63" stroke="#4b5f95"/>'
            f'<path d="M{f(a[0])} {f(a[1])} L{f(b[0])} {f(b[1])} L{f(b2[0])} {f(b2[1])} L{f(a2[0])} {f(a2[1])}Z" fill="#334155"/>'
            f'<line x1="{f(a[0])}" y1="{f(a[1])}" x2="{f(b[0])}" y2="{f(b[1])}" stroke="#f1f5f9" stroke-width="3.2" stroke-linecap="round"/>'
            # rocking arrow around the mirror's upper end
            f'<path d="M{f(a[0] - 16)} {f(a[1] + 2)} A 24 24 0 0 1 {f(a[0] + 8)} {f(a[1] + 20)}" fill="none" stroke="#e6e9f2" '
            f'stroke-width="1.4" marker-start="url(#ah)" marker-end="url(#ah)"/>')

def obj_sheet():
    """The object, drawn as a sheet in oblique view; the scan line runs into the page."""
    dx, dy = 34, -26
    x0, x1 = XM - 92, XM + 64
    y = YO + 13
    sheet = f'M{x0} {y} L{x1} {y} L{x1 + dx} {y + dy} L{x0 + dx} {y + dy}Z'
    stripes = "".join(
        f'<path d="M{f(XM - 17 - 8 * k - dx / 2)} {f(y)} L{f(XM - 17 - 8 * k + dx / 2)} {f(y + dy)}" stroke="{wl_hex(520 + 45 * k)}" '
        f'stroke-opacity="{0.55 - 0.05 * k:.2f}" stroke-width="5"/>' for k in range(8))
    line = (f'<path d="M{f(XM - dx / 2)} {f(y)} L{f(XM + dx / 2)} {f(y + dy)}" stroke="#fff" stroke-width="3.2" '
            f'stroke-linecap="round" filter="url(#glow)"/>')
    return (f'<path d="{sheet}" fill="#16203f" stroke="#33427a" stroke-width="1.4"/>'
            f'<g clip-path="url(#sheetClip)">{stripes}</g>{line}'
            f'<path d="M{x0 + 6} {y + 22} L{x1 - 6} {y + 22}" stroke="#8b93b0" stroke-width="1.3" marker-start="url(#ahm)" marker-end="url(#ahm)"/>'
            f'<text x="{f((x0 + x1) / 2)}" y="{y + 42}" text-anchor="middle" class="small">sweep</text>'), sheet

SHEET, SHEET_PATH = obj_sheet()

def grating():
    hatch = "".join(f'<line x1="{XG - 4}" y1="{AY - 54 + 6 * k}" x2="{XG + 4}" y2="{AY - 48 + 6 * k}" stroke="#c4b5fd" stroke-opacity="0.55" stroke-width="1"/>'
                    for k in range(17))
    return (f'<rect x="{XG - 4}" y="{AY - 56}" width="8" height="112" fill="#c4b5fd" fill-opacity="0.22" stroke="#c4b5fd" stroke-width="1.4"/>'
            + hatch)

def sensor():
    a, b = add(SEN, B_CAM, -58), add(SEN, B_CAM, 58)
    back = add(SEN, A_CAM, 14)
    return (f'<g transform="translate({f(back[0])} {f(back[1])}) rotate({f(-THETA_C)})">'
            f'<rect x="-8" y="-66" width="16" height="132" rx="3" fill="#0f2a22" stroke="#34d399" stroke-opacity="0.5"/></g>'
            f'<line x1="{f(a[0])}" y1="{f(a[1])}" x2="{f(b[0])}" y2="{f(b[1])}" stroke="#34d399" stroke-width="6" stroke-linecap="round"/>')

def spots():
    return "".join(f'<circle cx="{f(p[0])}" cy="{f(p[1])}" r="3.4" fill="{wl_hex(nm)}" filter="url(#glow)"/>' for nm, p in SPOT.items())

def angle_mark():
    r = 118
    p0 = (XG + r, AY)
    p1 = add((XG, AY), A_CAM, r)
    return (f'<line x1="{XG + 6}" y1="{AY}" x2="{XG + 168}" y2="{AY}" stroke="#8b93b0" stroke-width="1.1" stroke-dasharray="3 4"/>'
            f'<path d="M{f(p0[0])} {f(p0[1])} A {r} {r} 0 0 0 {f(p1[0])} {f(p1[1])}" fill="none" stroke="#8b93b0" stroke-width="1.1"/>'
            f'<text x="{XG + 172}" y="{AY + 4}" class="tiny">grating normal</text>'
            f'<text x="{f(XG + r + 8)}" y="{f(AY - 18)}" class="ang">22°</text>')

# ---------- labels ----------
def lab(x, y, title, lines, anchor="middle", color="#e6e9f2"):
    out = [f'<text x="{f(x)}" y="{f(y)}" text-anchor="{anchor}" class="lt" fill="{color}">{title}</text>']
    for k, (cls, txt) in enumerate(lines):
        out.append(f'<text x="{f(x)}" y="{f(y + 22 + 19 * k)}" text-anchor="{anchor}" class="{cls}">{txt}</text>')
    return "".join(out)

def tick(x, y0, y1):
    return f'<line x1="{f(x)}" y1="{f(y0)}" x2="{f(x)}" y2="{f(y1)}" stroke="#3b4a7a" stroke-width="1.2"/>'

LABELS = "".join([
    lab(XM - 8, 160, "Scan mirror", [("spec", "NEMA 8 stepper"), ("ls", "steps the line"), ("ls", "across the object")]),
    lab(XO + 8, 160, "Objective", [("spec", "16 mm f/4"), ("ls", "images the object"), ("ls", "onto the slit")]),
    lab(XC, 160, "Collimator", [("spec", "25 mm, used backward"), ("ls", "turns each slit point"), ("ls", "into a parallel beam")]),
    lab(XF + 26, 404, "Long-pass", [("spec", "≥ 500 nm"), ("ls", "blocks 2nd-order"), ("ls", "overlap")]),
    lab(XS + 6, 404, "Slit + field lens", [("spec", "50 µm slit, f 18 mm lens"), ("ls", "lets one line through; the"),
                                         ("ls", "field lens steers its ends"), ("ls", "into the camera lens")]),
    lab(XG - 8, 404, "Grating", [("spec", "500 lines/mm"), ("ls", "each wavelength"), ("ls", "leaves at its own angle")]),
    lab(966, 404, "Camera lens", [("spec", "12 mm f/2, tilted 22°"), ("ls", "focuses each wavelength"), ("ls", "onto its own column")]),
    lab(1110, 136, "IMX219 NoIR", [("spec", "no IR-cut filter")], anchor="start"),
    # leader ticks from labels to parts
    tick(XM - 8, 230, 248), tick(XO + 8, 230, AY - 44), tick(XC, 230, AY - 52),
    f'<path d="M{XF} {AY + 40} L{XF} 368 L{XF + 26} 384" fill="none" stroke="#3b4a7a" stroke-width="1.2"/>',
    tick(XS + 6, AY + 46, 384), tick(XG, AY + 62, 368),
    f'<path d="M{XG} 368 L{XG - 8} 384" fill="none" stroke="#3b4a7a" stroke-width="1.2"/>',
    f'<path d="M{f(C_CAM[0] + 54 * B_CAM[0] + 3)} {f(C_CAM[1] + 54 * B_CAM[1] + 6)} L{f(C_CAM[0] + 54 * B_CAM[0] + 3)} 368 L966 384" fill="none" stroke="#3b4a7a" stroke-width="1.2"/>',
])

# one camera frame: spatial line (vertical) x spectrum (horizontal)
FX, FY, FW, FH = 1110, 196, 132, 92
def frame_chip():
    rows = []
    for k in range(7):
        y = FY + 10 + k * (FH - 20) / 6
        rows.append(f'<line x1="{FX + 8}" y1="{f(y)}" x2="{FX + FW - 8}" y2="{f(y)}" stroke="#000" stroke-opacity="0.12"/>')
    return (f'<rect x="{FX - 1}" y="{FY - 1}" width="{FW + 2}" height="{FH + 2}" rx="5" fill="#000" stroke="#2c3b6e"/>'
            f'<rect x="{FX + 8}" y="{FY + 8}" width="{FW - 16}" height="{FH - 16}" rx="2" fill="url(#spectrumH)"/>'
            f'<rect x="{FX + 8}" y="{FY + 8}" width="{FW - 16}" height="{FH - 16}" rx="2" fill="url(#frameShade)"/>'
            + "".join(rows) +
            f'<text x="{FX}" y="{FY + FH + 20}" class="ls">→ wavelength</text>'
            f'<text x="{FX}" y="{FY + FH + 38}" class="ls">↓ along the line</text>'
            f'<text x="{FX}" y="{FY - 12}" class="spec">one frame</text>')

# light-path strip with real distances (mm), from docs/parts_list_and_design.md
STOPS = ["object", "mirror", "objective", "slit", "collimator", "grating", "camera lens", "sensor"]
GAPS = ["≈130", "≈20", "17.9", "25", "22", "3", "12"]
def strip():
    y, x0, x1 = 676, 64, 1050
    step = (x1 - x0) / (len(STOPS) - 1)
    out = [f'<text x="{x0 - 16}" y="{y - 30}" class="spec">Light path, real distances in mm</text>',
           f'<line x1="{x0}" y1="{y}" x2="{x1}" y2="{y}" stroke="url(#pathGrad)" stroke-width="3" stroke-linecap="round"/>']
    for k, name in enumerate(STOPS):
        x = x0 + k * step
        out.append(f'<circle cx="{f(x)}" cy="{y}" r="5.5" fill="#0b1020" stroke="#e6e9f2" stroke-width="1.8"/>'
                   f'<text x="{f(x)}" y="{y + 26}" text-anchor="middle" class="ls">{name}</text>')
        if k < len(GAPS):
            out.append(f'<text x="{f(x + step / 2)}" y="{y - 9}" text-anchor="middle" class="mono">{GAPS[k]}</text>')
    out.append(f'<text x="{x1 + 36}" y="{y - 6}" class="spec">≈ 80 mm</text>'
               f'<text x="{x1 + 36}" y="{y + 13}" class="ls">objective to sensor</text>')
    return "".join(out)

def flows():
    """Light pulses running along the chief rays (SMIL; static viewers just see faint dots)."""
    white = P(mid)
    out = [f'<path d="{white}" fill="none" stroke="#fff" stroke-width="3" stroke-linecap="round" stroke-dasharray="2 46" '
           f'stroke-opacity="0.85"><animate attributeName="stroke-dashoffset" values="48;0" dur="0.9s" repeatCount="indefinite"/></path>']
    for d, nm in zip(COLOR_FLOWS, WLS):
        out.append(f'<path d="{d}" fill="none" stroke="{wl_hex(nm)}" stroke-width="3" stroke-linecap="round" stroke-dasharray="2 46" '
                   f'stroke-opacity="0.9"><animate attributeName="stroke-dashoffset" values="48;0" dur="0.9s" repeatCount="indefinite"/></path>')
    return "".join(out)

grad_h = "".join(f'<stop offset="{(nm - 500) / 450:.3f}" stop-color="{c}"/>' for nm, c in SPECTRUM if nm <= 950)

svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" role="img" aria-labelledby="t d">
  <title id="t">Optical path of the pushbroom spectrograph</title>
  <desc id="d">Schematic side view in the dispersion plane. Light from one line of the object goes up to a scan mirror on a stepper, through a 500 nm long-pass filter and a 16 mm objective onto a 50 micron slit with a field lens behind it. A 25 mm lens collimates it, a 500 lines per mm grating fans it out by wavelength, and a 12 mm lens tilted 22 degrees focuses each wavelength onto its own column of an IMX219 NoIR sensor. A strip at the bottom gives the real distances between the parts.</desc>
  <style>
    text {{ font-family: {FONT}; }}
    .h1 {{ font-size: 32px; font-weight: 800; fill: #f1f5ff; }}
    .sub {{ font-size: 17px; fill: #a3acc9; }}
    .lt {{ font-size: 18px; font-weight: 700; }}
    .spec {{ font-size: 15px; font-weight: 600; fill: #7dd3fc; }}
    .ls {{ font-size: 15px; fill: #8b93b0; }}
    .small {{ font-size: 14px; fill: #8b93b0; font-style: italic; }}
    .tiny {{ font-size: 13px; fill: #6b7394; font-style: italic; }}
    .ang {{ font-size: 15px; font-weight: 700; fill: #e6e9f2; }}
    .mono {{ font-size: 14.5px; fill: #e6e9f2; font-family: {MONO}; }}
  </style>
  <defs>
    <linearGradient id="bg" x1="0" y1="0" x2="1" y2="1">
      <stop offset="0" stop-color="#101a3a"/><stop offset="0.55" stop-color="#0a1024"/><stop offset="1" stop-color="#060914"/>
    </linearGradient>
    <pattern id="holes" width="28" height="28" patternUnits="userSpaceOnUse"><circle cx="14" cy="14" r="1.7" fill="#1d2955"/></pattern>
    <radialGradient id="fade" cx="0.55" cy="0.45" r="0.75"><stop offset="0" stop-color="#fff"/><stop offset="1" stop-color="#fff" stop-opacity="0"/></radialGradient>
    <mask id="holesMask"><rect width="{W}" height="{H}" fill="url(#fade)"/></mask>
    <linearGradient id="spectrumH" x1="0" x2="1">{grad_h}</linearGradient>
    <linearGradient id="frameShade" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0" stop-color="#000" stop-opacity="0.55"/><stop offset="0.12" stop-color="#000" stop-opacity="0"/>
      <stop offset="0.88" stop-color="#000" stop-opacity="0"/><stop offset="1" stop-color="#000" stop-opacity="0.55"/>
    </linearGradient>
    <linearGradient id="pathGrad" gradientUnits="userSpaceOnUse" x1="64" y1="0" x2="1050" y2="0">
      <stop offset="0" stop-color="#e6e9f2" stop-opacity="0.7"/><stop offset="0.7" stop-color="#e6e9f2" stop-opacity="0.7"/>
      <stop offset="0.78" stop-color="#00f5a0"/><stop offset="0.86" stop-color="#ffe600"/><stop offset="0.93" stop-color="#ff2a1f"/><stop offset="1" stop-color="#a61a6e"/>
    </linearGradient>
    <linearGradient id="beam" gradientUnits="userSpaceOnUse" x1="{XM}" y1="{YO}" x2="{XG}" y2="{AY}">
      <stop offset="0" stop-color="#fff" stop-opacity="0.10"/><stop offset="1" stop-color="#fff" stop-opacity="0.22"/>
    </linearGradient>
    <filter id="glow" x="-50%" y="-50%" width="200%" height="200%"><feGaussianBlur stdDeviation="2.6" result="b"/><feMerge><feMergeNode in="b"/><feMergeNode in="SourceGraphic"/></feMerge></filter>
    <filter id="soft" x="-20%" y="-20%" width="140%" height="140%"><feGaussianBlur stdDeviation="5"/></filter>
    <clipPath id="sheetClip"><path d="{SHEET_PATH}"/></clipPath>
    <marker id="ah" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse"><path d="M0 0 L10 5 L0 10z" fill="#e6e9f2"/></marker>
    <marker id="ahm" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse"><path d="M0 0 L10 5 L0 10z" fill="#8b93b0"/></marker>
  </defs>

  <rect width="{W}" height="{H}" rx="26" fill="url(#bg)"/>
  <rect width="{W}" height="{H}" rx="26" fill="url(#holes)" mask="url(#holesMask)"/>

  <text x="48" y="68" class="h1">Optical path</text>
  <text x="48" y="102" class="sub">One line of the object goes in; every point on it comes out as a spectrum on the sensor.</text>
  <text x="48" y="127" class="sub">Side view in the dispersion plane, config C. Schematic: not to scale, color fan exaggerated.</text>

  <!-- object -->
  {SHEET}
  <text x="{XM + 112}" y="{YO - 4}" class="lt" fill="#e6e9f2">Object</text>
  <text x="{XM + 112}" y="{YO + 16}" class="spec">one line at a time, 42 × 0.42 mm at 150 mm</text>
  <text x="{XM + 112}" y="{YO + 34}" class="ls">The line runs into the page; the mirror sweeps it sideways.</text>

  <!-- white light up to the grating -->
  <path d="{P(white_poly)}Z" fill="url(#beam)"/>
  <g fill="none" stroke="#fff" stroke-width="1.1" stroke-opacity="0.55" stroke-linejoin="round">
    <path d="{P(top)}"/><path d="{P(bot)}"/>
  </g>
  <path d="{P(mid)}" fill="none" stroke="#fff" stroke-width="1.3" stroke-opacity="0.8" stroke-linejoin="round"/>

  <!-- color fan after the grating -->
  <g style="mix-blend-mode:screen">{FILLS}</g>
  <g fill="none" stroke-width="1" stroke-linejoin="round">{CHIEFS}</g>
  {flows()}

  <!-- parts -->
  {mirror()}
  <rect x="{XF - 5}" y="{AY - 36}" width="10" height="72" rx="2" fill="#f97316" fill-opacity="0.28" stroke="#fb923c" stroke-width="1.4"/>
  {lens(XO, AY, 40)}
  <line x1="{XS}" y1="{AY - 42}" x2="{XS}" y2="{AY - 3}" stroke="{SLIT}" stroke-width="4"/>
  <line x1="{XS}" y1="{AY + 3}" x2="{XS}" y2="{AY + 42}" stroke="{SLIT}" stroke-width="4"/>
  {lens(XFL, AY, 32, w=6, color="#34d399")}
  {lens(XC, AY, 48)}
  {grating()}
  {lens(C_CAM[0], C_CAM[1], 54, w=8, rot=-THETA_C)}
  {sensor()}
  {spots()}
  {angle_mark()}
  {frame_chip()}
  <path d="M{f(SEN[0] + 26)} {f(SEN[1] - 4)} L{FX - 22} {FY + FH / 2 - 4}" stroke="#8b93b0" stroke-width="1.2" stroke-dasharray="3 4" marker-end="url(#ahm)"/>

  {LABELS}
  {strip()}
</svg>
'''
open(OUT, "w", encoding="utf-8", newline="\n").write(svg)
print("wrote", OUT, len(svg))
