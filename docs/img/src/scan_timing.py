"""Timing of a stare scan (docs/img/scan_timing.svg): line clock, mirror steps, MOVING and
TRIG pins, and the camera frames that fit between the moves.

Run: python docs/img/src/scan_timing.py  [out.svg]   (default: docs/img/scan_timing.svg)
Times follow firmware/PROTOCOL.md's example: 2 microsteps per line at 30 fps take 0.65 ms
to move and 3 ms to settle. The camera's exposure and readout are illustrative.
"""
import sys
from pathlib import Path
from common import *

W, H = 1280, 672
OUT = sys.argv[1] if len(sys.argv) > 1 else str(Path(__file__).resolve().parent.parent / "scan_timing.svg")
def f(x): return f"{x:.1f}".rstrip('0').rstrip('.')

P = 1000 / 30                  # line period, ms
MOVE, SETTLE = 0.65, 3.0       # 2 microsteps, then the settle time
EXPOSE, READOUT = 14.0, 13.0   # rolling shutter: first row to last row
STEP_C, MOVING_C, TRIG_C, MIRROR_C, CLOCK_C = "#e6e9f2", "#fbbf24", "#f472b6", LENS, GRATING

def ready(n): return n * P if n == 0 else n * P + MOVE + SETTLE

# ---------- main timeline ----------
X0, X1, T0, T1 = 262, 1232, -5.0, 105.0
def tx(t): return X0 + (t - T0) * (X1 - X0) / (T1 - T0)

Y_CLOCK, Y_MIRROR, Y_STEP, Y_MOVING, Y_TRIG, Y_CAM = 306, 382, 436, 482, 528, 592
PULSE = 15

def row_label(y, title, sub):
    return (f'<text x="232" y="{y - 2}" text-anchor="end" class="lt">{title}</text>'
            f'<text x="232" y="{y + 17}" text-anchor="end" class="ls">{sub}</text>')

def digital(y, highs, color, fill=False):
    """A logic trace low at y, high (y - PULSE) over each (t_on, t_off)."""
    d = [f"M{f(tx(T0))} {y}"]
    for a, b in highs:
        xa, xb = tx(a), max(tx(b), tx(a) + 2.5)
        d.append(f"L{f(xa)} {y} L{f(xa)} {y - PULSE} L{f(xb)} {y - PULSE} L{f(xb)} {y}")
    d.append(f"L{f(tx(T1))} {y}")
    path = " ".join(d)
    out = ""
    if fill:
        out = "".join(f'<rect x="{f(tx(a))}" y="{y - PULSE}" width="{f(tx(b) - tx(a))}" height="{PULSE}" '
                      f'fill="{color}" fill-opacity="0.22"/>' for a, b in highs)
    return out + f'<path d="{path}" fill="none" stroke="{color}" stroke-width="2" stroke-linejoin="round"/>'

def clock():
    out = [f'<line x1="{tx(T0)}" y1="{Y_CLOCK}" x2="{tx(T1)}" y2="{Y_CLOCK}" stroke="{FAINT}" stroke-width="2"/>']
    for n in range(4):
        x = tx(n * P)
        out.append(f'<line x1="{f(x)}" y1="{Y_CLOCK - 16}" x2="{f(x)}" y2="{Y_CAM + 26}" stroke="{CLOCK_C}" '
                   f'stroke-opacity="0.22" stroke-dasharray="3 5"/>'
                   f'<circle cx="{f(x)}" cy="{Y_CLOCK}" r="6" fill="{BG}" stroke="{CLOCK_C}" stroke-width="2.2"/>'
                   f'<text x="{f(x)}" y="{Y_CLOCK - 14}" text-anchor="middle" class="tag">line {n}</text>'
                   f'<text x="{f(x)}" y="{Y_CLOCK + 24}" text-anchor="middle" class="mono">t0{" + %dP" % n if n > 1 else " + P" if n else ""}</text>')
    # the period, between ticks 1 and 2
    a, b, y = tx(P), tx(2 * P), Y_CLOCK - 44
    out.append(f'<path d="M{f(a)} {y} L{f(b)} {y}" stroke="#8b93b0" stroke-width="1.3" marker-start="url(#ahm)" marker-end="url(#ahm)"/>'
               f'<text x="{f((a + b) / 2)}" y="{y - 9}" text-anchor="middle" class="small">period P = 33 333.333 µs</text>')
    return "".join(out)

def mirror():
    pos = [-20, -18, -16, -14]
    def py(p): return Y_MIRROR + 16 - (p + 20) * 7
    d = [f"M{f(tx(T0))} {f(py(pos[0]))}"]
    for n in range(1, 4):
        a = n * P
        d.append(f"L{f(tx(a))} {f(py(pos[n - 1]))} L{f(tx(a + MOVE))} {f(py(pos[n]))}")
    d.append(f"L{f(tx(T1))} {f(py(pos[3]))}")
    out = [f'<path d="{" ".join(d)}" fill="none" stroke="{MIRROR_C}" stroke-width="2.4" stroke-linejoin="round"/>']
    for n in range(4):
        x, anchor = ((tx(ready(n)) + tx((n + 1) * P)) / 2, "middle") if n < 3 else (X1, "end")
        out.append(f'<text x="{f(x)}" y="{f(py(pos[n]) - 8)}" text-anchor="{anchor}" class="mono">{pos[n]}</text>')
    return "".join(out)

def frames():
    out = []
    top, bot = Y_CAM - 22, Y_CAM + 22
    for n in range(3):
        a = ready(n) + 1.4
        xa, xb = tx(a), tx(a + EXPOSE)
        xc, xd = tx(a + READOUT + EXPOSE), tx(a + READOUT)
        out.append(f'<path d="M{f(xa)} {top} L{f(xb)} {top} L{f(xc)} {bot} L{f(xd)} {bot}Z" fill="{SENSOR}" '
                   f'fill-opacity="0.16" stroke="{SENSOR}" stroke-width="1.6"/>'
                   f'<text x="{f((xa + xc) / 2)}" y="{Y_CAM + 5}" text-anchor="middle" class="tag" style="fill:{SENSOR}">frame for line {n}</text>')
    # the hold window of line 1: from ready to the next tick
    a, b, y = tx(ready(1)), tx(2 * P), bot + 18
    out.append(f'<path d="M{f(a)} {y - 6} L{f(a)} {y} L{f(b)} {y} L{f(b)} {y - 6}" fill="none" stroke="#8b93b0" stroke-width="1.3"/>'
               f'<text x="{f((a + b) / 2)}" y="{y + 20}" text-anchor="middle" class="small">the mirror holds still from ready to the next tick</text>')
    return "".join(out)

# ---------- zoom on one tick ----------
IX, IY, IW, IH = 812, 30, 420, 200
ZX0, ZX1, ZT0, ZT1 = IX + 104, IX + 396, -0.5, 4.6
def zx(t): return ZX0 + (t - ZT0) * (ZX1 - ZX0) / (ZT1 - ZT0)

def zoom():
    ys, ym, yt = IY + 82, IY + 128, IY + 174
    h = 14
    def trace(y, highs, color, fill=False):
        d = [f"M{f(ZX0)} {y}"]
        for a, b in highs:
            xa, xb = zx(a), max(zx(b), zx(a) + 2.5)
            d.append(f"L{f(xa)} {y} L{f(xa)} {y - h} L{f(xb)} {y - h} L{f(xb)} {y}")
        d.append(f"L{f(ZX1)} {y}")
        rect = "".join(f'<rect x="{f(zx(a))}" y="{y - h}" width="{f(zx(b) - zx(a))}" height="{h}" fill="{color}" '
                       f'fill-opacity="0.22"/>' for a, b in highs) if fill else ""
        return rect + f'<path d="{" ".join(d)}" fill="none" stroke="{color}" stroke-width="2" stroke-linejoin="round"/>'
    r = MOVE + SETTLE
    return "\n  ".join([
        f'<rect x="{IX}" y="{IY}" width="{IW}" height="{IH}" rx="16" fill="#0e1631" fill-opacity="0.96" stroke="#2c3b6e"/>',
        f'<text x="{IX + 20}" y="{IY + 30}" class="lt">One tick, close up</text>',
        f'<text x="{IX + IW - 20}" y="{IY + 30}" text-anchor="end" class="ls">line 1, from t0 + P</text>',
        f'<line x1="{f(zx(0))}" y1="{IY + 52}" x2="{f(zx(0))}" y2="{yt + 8}" stroke="{CLOCK_C}" stroke-opacity="0.5" stroke-dasharray="3 4"/>',
        f'<text x="{IX + 20}" y="{ys - 2}" class="tag" style="fill:{STEP_C}">STEP</text>',
        f'<text x="{IX + 20}" y="{ym - 2}" class="tag" style="fill:{MOVING_C}">MOVING</text>',
        f'<text x="{IX + 20}" y="{yt - 2}" class="tag" style="fill:{TRIG_C}">TRIG</text>',
        trace(ys, [(0, 0.03), (MOVE, MOVE + 0.03)], STEP_C),
        trace(ym, [(0, r)], MOVING_C, fill=True),
        trace(yt, [(r, r + 0.1)], TRIG_C),
        f'<path d="M{f(zx(0))} {ys - 22} L{f(zx(MOVE))} {ys - 22}" stroke="#8b93b0" stroke-width="1.2" marker-start="url(#ahm)" marker-end="url(#ahm)"/>',
        f'<text x="{f(zx(MOVE) + 8)}" y="{ys - 17}" class="small">0.65 ms: 2 microsteps</text>',
        f'<path d="M{f(zx(MOVE))} {ym - 22} L{f(zx(r))} {ym - 22}" stroke="#8b93b0" stroke-width="1.2" marker-start="url(#ahm)" marker-end="url(#ahm)"/>',
        f'<text x="{f((zx(MOVE) + zx(r)) / 2)}" y="{ym - 28}" text-anchor="middle" class="small">settle 3 ms</text>',
        f'<text x="{f(zx(r) - 8)}" y="{yt - 18}" text-anchor="end" class="small">ready: TRIG and EV LINE</text>',
    ])

svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" role="img" aria-labelledby="t d">
  <title id="t">Timing of a stare scan</title>
  <desc id="d">Timing diagram of three scan lines. A line clock ticks once per camera frame period, 33.3 ms at 30 frames per second. At each tick the ESP32 sends two step pulses 0.65 ms apart, so the mirror moves two microsteps; the MOVING pin stays high until the mirror has been still for 3 ms, then the TRIG pin pulses and the ESP32 reports the line as ready. Each camera frame, drawn as a rolling-shutter exposure from first row to last, fits in the hold between ready and the next tick. An inset zooms in on one tick.</desc>
  <style>
    text {{ font-family: {FONT}; }}
    .h1 {{ font-size: 32px; font-weight: 800; fill: #f1f5ff; }}
    .sub {{ font-size: 17px; fill: #a3acc9; }}
    .lt {{ font-size: 18px; font-weight: 700; fill: #e6e9f2; }}
    .ls {{ font-size: 15px; fill: #8b93b0; }}
    .tag {{ font-size: 14px; font-weight: 700; fill: #c9d3f5; }}
    .small {{ font-size: 14px; fill: #8b93b0; font-style: italic; }}
    .mono {{ font-size: 13.5px; fill: #e6e9f2; font-family: {MONO}; }}
  </style>
  <defs>
    <linearGradient id="bg" x1="0" y1="0" x2="1" y2="1">
      <stop offset="0" stop-color="#101a3a"/><stop offset="0.55" stop-color="#0a1024"/><stop offset="1" stop-color="#060914"/>
    </linearGradient>
    <pattern id="holes" width="28" height="28" patternUnits="userSpaceOnUse"><circle cx="14" cy="14" r="1.7" fill="#1d2955"/></pattern>
    <radialGradient id="fade" cx="0.5" cy="0.55" r="0.75"><stop offset="0" stop-color="#fff"/><stop offset="1" stop-color="#fff" stop-opacity="0"/></radialGradient>
    <mask id="holesMask"><rect width="{W}" height="{H}" fill="url(#fade)"/></mask>
    <marker id="ahm" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse"><path d="M0 0 L10 5 L0 10z" fill="#8b93b0"/></marker>
  </defs>

  <rect width="{W}" height="{H}" rx="26" fill="url(#bg)"/>
  <rect width="{W}" height="{H}" rx="26" fill="url(#holes)" mask="url(#holesMask)"/>

  <text x="48" y="68" class="h1">Frame sync</text>
  <text x="48" y="102" class="sub">The ESP32 runs a line clock that follows the camera's frames.</text>
  <text x="48" y="127" class="sub">On each tick it steps the mirror to the next line, waits for it</text>
  <text x="48" y="152" class="sub">to settle, then holds it still while the camera takes the frame.</text>
  <text x="48" y="196" class="ls">Stare scan, 2 microsteps per line at 30 fps.</text>
  <text x="48" y="217" class="ls">NUDGE and PERIOD keep the ticks between exposures.</text>

  {row_label(Y_CLOCK, "Line clock", "ESP32 time")}
  {row_label(Y_MIRROR, "Mirror", "position, 1/32 steps")}
  {row_label(Y_STEP, "STEP", "to the TMC2209")}
  {row_label(Y_MOVING, "MOVING pin", "moving or settling")}
  {row_label(Y_TRIG, "TRIG pin", "line ready")}
  {row_label(Y_CAM, "Camera", "first row to last")}

  {clock()}
  {mirror()}
  {digital(Y_STEP, [(n * P + k * MOVE, n * P + k * MOVE + 0.03) for n in (1, 2, 3) for k in (0, 1)], STEP_C)}
  {digital(Y_MOVING, [(n * P, ready(n)) for n in (1, 2, 3)], MOVING_C, fill=True)}
  {digital(Y_TRIG, [(ready(n), ready(n) + 0.1) for n in range(4)], TRIG_C)}
  {frames()}
  {zoom()}
</svg>
'''
open(OUT, "w", encoding="utf-8", newline="\n").write(svg)
print("wrote", OUT, len(svg))
