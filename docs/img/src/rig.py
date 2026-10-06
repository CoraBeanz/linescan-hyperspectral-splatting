"""Side-view diagram of the rig (docs/img/rig.svg): SO-101 arm, scanner head, tag board, lamps.

Run: python docs/img/src/rig.py  [out.svg]   (default: docs/img/rig.svg)
A drawing, not a CAD model: proportions are approximate.
"""
import math, sys
from pathlib import Path
from common import *

W, H = 1280, 784
OUT = sys.argv[1] if len(sys.argv) > 1 else str(Path(__file__).resolve().parent.parent / "rig.svg")
def f(x): return f"{x:.1f}".rstrip('0').rstrip('.')

def elbow(S, Wr, L1, L2, up=True):
    """Elbow position for a two-link arm from shoulder S to wrist Wr (elbow up)."""
    dx, dy = Wr[0] - S[0], Wr[1] - S[1]
    d = math.hypot(dx, dy)
    a = (L1**2 - L2**2 + d**2) / (2 * d)
    h = math.sqrt(max(L1**2 - a**2, 0))
    px, py = S[0] + a * dx / d, S[1] + a * dy / d
    ux, uy = dx / d, dy / d
    s = 1 if up else -1
    return (px + s * h * uy, py - s * h * ux)

TABLE_Y = 652
S = (305, 552)
L1 = L2 = 200
HEAD_LEN = 136

def arm_pose(M, ang):
    a = math.radians(ang)
    Wr = (M[0] - HEAD_LEN * math.cos(a), M[1] - HEAD_LEN * math.sin(a))
    E = elbow(S, Wr, L1, L2)
    return E, Wr

def arm(E, Wr, ang, ghost=False):
    g = 'opacity="0.28"' if ghost else ''
    dash = ' stroke-dasharray="6 5"' if ghost else ''
    fill_link = "none" if ghost else "url(#armGrad)"
    out = [f'<g {g} stroke-linecap="round" stroke-linejoin="round">']
    if ghost:
        for (a, b, w) in ((S, E, 34), (E, Wr, 30)):
            out.append(f'<line x1="{f(a[0])}" y1="{f(a[1])}" x2="{f(b[0])}" y2="{f(b[1])}" stroke="#7d8db8" stroke-width="{w}" stroke-opacity="0.35"/>')
    else:
        for (a, b, w) in ((S, E, 36), (E, Wr, 31)):
            out.append(f'<line x1="{f(a[0])}" y1="{f(a[1])}" x2="{f(b[0])}" y2="{f(b[1])}" stroke="#3c4c7c" stroke-width="{w}"/>')
            out.append(f'<line x1="{f(a[0])}" y1="{f(a[1])}" x2="{f(b[0])}" y2="{f(b[1])}" stroke="url(#armGrad)" stroke-width="{w-3}"/>')
    for (x, y), r in ((S, 21), (E, 21), (Wr, 18)):
        out.append(f'<circle cx="{f(x)}" cy="{f(y)}" r="{r}" fill="#121a33" stroke="#4b5f95" stroke-width="2"/>'
                   f'<circle cx="{f(x)}" cy="{f(y)}" r="{r*0.42:.1f}" fill="#2b3a63"/>')
    out.append(f'''<g transform="translate({f(Wr[0])} {f(Wr[1])}) rotate({ang})">
      <rect x="12" y="-17" width="16" height="34" rx="4" fill="#151d38" stroke="#3c4c7c"/>
      <rect x="26" y="-21" width="80" height="42" rx="8" fill="url(#armGrad)" stroke="#4b5f95" stroke-width="1.5"/>
      <rect x="34" y="-10" width="64" height="20" rx="4" fill="#0a0f22" stroke="#2c3966"/>
      <line x1="38" y1="0" x2="92" y2="0" stroke="#e2e8f0" stroke-opacity="0.5" stroke-width="1.2"/>
      <ellipse cx="45" cy="0" rx="2" ry="7" fill="none" stroke="#7dd3fc" stroke-width="1.4"/>
      <line x1="57" y1="-7" x2="57" y2="-1.5" stroke="#e2e8f0" stroke-width="1.6"/><line x1="57" y1="1.5" x2="57" y2="7" stroke="#e2e8f0" stroke-width="1.6"/>
      <ellipse cx="71" cy="0" rx="2" ry="7" fill="none" stroke="#7dd3fc" stroke-width="1.4"/>
      <rect x="81" y="-7" width="2.5" height="14" fill="#c4b5fd"/>
      <path d="M84 0 L95 -6 L95 6Z" fill="url(#spectrum)" opacity="0.9"/>
      <rect x="40" y="-34" width="30" height="14" rx="3" fill="#151d38" stroke="#4b5f95"/>
      <circle cx="72" cy="-27" r="5" fill="#0a0f22" stroke="#7dd3fc" stroke-width="1.4"/>
      <rect x="106" y="-15" width="20" height="30" rx="3" fill="#121a33" stroke="#4b5f95"/>
      <line x1="{HEAD_LEN - 14}" y1="-14" x2="{HEAD_LEN}" y2="12" stroke="#e2e8f0" stroke-width="3.2"/>
    </g>''')
    out.append('</g>')
    return "\n".join(out)

# ---------- plant (plain colors) ----------
LEAVES = [(724, 588, 166, 104, 24), (736, 580, 16, 96, 23), (726, 552, 128, 96, 22),
          (734, 546, 56, 100, 22), (730, 526, 93, 78, 17)]
def leaf_path(leaf):
    bx, by, ang, L, hw = leaf
    a = math.radians(ang); ax, ay = math.cos(a), -math.sin(a); nx, ny = -ay, ax
    tx, ty = bx + ax * L, by + ay * L; mx, my = bx + ax * L / 2, by + ay * L / 2
    return (f'<path d="M{f(bx)} {f(by)} Q{f(mx + nx*2*hw)} {f(my + ny*2*hw)} {f(tx)} {f(ty)} '
            f'Q{f(mx - nx*2*hw)} {f(my - ny*2*hw)} {f(bx)} {f(by)}Z" fill="#1d6b40" stroke="#2f8a57" stroke-width="1.2"/>'
            f'<line x1="{f(bx)}" y1="{f(by)}" x2="{f(tx)}" y2="{f(ty)}" stroke="#3aa56a" stroke-width="1.4" stroke-opacity="0.8"/>')
POT = (696, 600, 764, 648)  # top-left x, top y, top-right x, bottom y
def plant():
    x0, y0, x1, y1 = POT
    leaves = "".join(leaf_path(l) for l in LEAVES)
    return (f'<path d="M730 646 C728 616 732 576 730 526" fill="none" stroke="#2f8a57" stroke-width="5" stroke-linecap="round"/>'
            f'{leaves}<path d="M{x0} {y0} L{x1} {y0} L{x1-6} {y1} L{x0+6} {y1}Z" fill="#7d4430" stroke="#9a5a3f"/>'
            f'<rect x="{x0-6}" y="{y0-8}" width="{x1-x0+12}" height="12" rx="3" fill="#8f4f37" stroke="#a8644a"/>')
def plant_clip():
    x0, y0, x1, y1 = POT
    out = []
    for bx, by, ang, L, hw in LEAVES:
        a = math.radians(ang); ax, ay = math.cos(a), -math.sin(a); nx, ny = -ay, ax
        tx, ty = bx + ax * L, by + ay * L; mx, my = bx + ax * L / 2, by + ay * L / 2
        out.append(f'<path d="M{f(bx)} {f(by)} Q{f(mx + nx*2*hw)} {f(my + ny*2*hw)} {f(tx)} {f(ty)} Q{f(mx - nx*2*hw)} {f(my - ny*2*hw)} {f(bx)} {f(by)}Z"/>')
    out.append(f'<path d="M{x0-6} {y0-8} L{x1+6} {y0-8} L{x1-6} {y1} L{x0+6} {y1}Z"/>')
    out.append('<rect x="727" y="520" width="6" height="90"/>')
    return "".join(out)

def tag(x, y, s=1.0):
    return (f'<g transform="translate({f(x)} {f(y)}) skewX(-48) scale(1 0.55)">'
            f'<rect width="{18*s}" height="{18*s}" fill="#e6e9f2" fill-opacity="0.9"/>'
            f'<rect x="{3*s}" y="{3*s}" width="{12*s}" height="{12*s}" fill="#141c38"/>'
            f'<rect x="{5*s}" y="{5*s}" width="{4*s}" height="{4*s}" fill="#e6e9f2"/>'
            f'<rect x="{9*s}" y="{9*s}" width="{4*s}" height="{4*s}" fill="#e6e9f2"/></g>')

M_MAIN, ANG_MAIN = (612, 382), 40
E1, W1 = arm_pose(M_MAIN, ANG_MAIN)
GHOST_M, GHOST_ANG = (676, 318), 78   # a later viewpoint, looking down at the plant

X0, X1, XS = 662, 806, 742
LINE_TOP, LINE_BOT = 470, 650
DUR, KT = "7s", "0;0.7;0.85;0.92;1"
def sheet(x): return f"{f(M_MAIN[0])},{f(M_MAIN[1])} {x},{LINE_TOP} {x},{LINE_BOT}"

def label(n, x, y, title, sub, anchor="start"):
    cx = x + 13 if anchor == "start" else x - 13
    tx = x + 36 if anchor == "start" else x - 36
    return (f'<g><circle cx="{cx}" cy="{y - 6}" r="13" fill="#7dd3fc" fill-opacity="0.14" stroke="#7dd3fc" stroke-width="1.4"/>'
            f'<text x="{cx}" y="{y - 1}" text-anchor="middle" class="num">{n}</text>'
            f'<text x="{tx}" y="{y}" text-anchor="{anchor}" class="lt">{title}</text>'
            + "".join(f'<text x="{tx}" y="{y + 23 + 20*k}" text-anchor="{anchor}" class="ls">{line}</text>' for k, line in enumerate(sub))
            + '</g>')

def leader(pts):
    d = "M" + " L".join(f"{f(x)} {f(y)}" for x, y in pts)
    return f'<path d="{d}" fill="none" stroke="#8b93b0" stroke-width="1.2" stroke-opacity="0.8"/><circle cx="{f(pts[-1][0])}" cy="{f(pts[-1][1])}" r="3" fill="#8b93b0"/>'

# head close-up inset
IX, IY, IW, IH = 812, 34, 432, 268
def inset():
    yA = IY + 118   # optical axis
    xs = dict(lp=IX + 132, obj=IX + 156, slit=IX + 200, fl=IX + 208, coll=IX + 256, gr=IX + 306,
              cam=IX + 326, sen=IX + 366)
    return "\n  ".join([
        f'<rect x="{IX}" y="{IY}" width="{IW}" height="{IH}" rx="16" fill="#0e1631" fill-opacity="0.96" stroke="#2c3b6e"/>',
        f'<text x="{IX + 20}" y="{IY + 30}" class="lt">Scanner head, close up</text>',
        # housing
        f'<rect x="{IX + 112}" y="{yA - 34}" width="282" height="68" rx="10" fill="#18213f" stroke="#4b5f95" stroke-width="1.5"/>',
        f'<rect x="{IX + 120}" y="{yA - 24}" width="266" height="48" rx="6" fill="#070b18" stroke="#2c3966"/>',
        # pose camera on top
        f'<rect x="{IX + 270}" y="{yA - 58}" width="54" height="24" rx="5" fill="#121a33" stroke="#4b5f95"/>',
        f'<circle cx="{IX + 324}" cy="{yA - 46}" r="8" fill="#0a0f22" stroke="#7dd3fc" stroke-width="1.6"/>',
        f'<text x="{IX + 412}" y="{IY + 30}" class="ls" text-anchor="end">RGB pose camera</text>',
        f'<path d="M{IX + 340} {IY + 38} L{IX + 330} {yA - 54}" stroke="#8b93b0" stroke-width="1.1"/>',
        # stepper + mirror; light comes up from the object and folds into the train
        f'<rect x="{IX + 24}" y="{yA - 26}" width="52" height="52" rx="6" fill="#121a33" stroke="#4b5f95"/>',
        f'<circle cx="{IX + 50}" cy="{yA}" r="7" fill="#2b3a63" stroke="#4b5f95"/>',
        f'<line x1="{IX + 78}" y1="{yA + 22}" x2="{IX + 112}" y2="{yA - 12}" stroke="#e2e8f0" stroke-width="4" stroke-linecap="round"/>',
        f'<path d="M{IX + 88} {IY + IH - 12} L{IX + 88} {yA + 12} L{IX + 124} {yA + 4}" fill="none" stroke="#fff" stroke-opacity="0.75" stroke-width="1.6"/>',
        f'<path d="M{IX + 100} {IY + IH - 12} L{IX + 100} {yA} L{IX + 124} {yA - 4}" fill="none" stroke="#fff" stroke-opacity="0.45" stroke-width="1.2"/>',
        f'<text x="{IX + 80}" y="{IY + IH - 18}" class="small" text-anchor="end">from the</text>',
        f'<text x="{IX + 80}" y="{IY + IH - 2}" class="small" text-anchor="end">object</text>',
        f'<text x="{IX + 20}" y="{IY + 58}" class="ls">scan mirror on a</text>',
        f'<text x="{IX + 20}" y="{IY + 76}" class="ls">NEMA 8 stepper</text>',
        # train inside the housing
        f'<line x1="{IX + 124}" y1="{yA}" x2="{xs["gr"]}" y2="{yA}" stroke="#fff" stroke-opacity="0.7" stroke-width="1.4"/>',
        f'<rect x="{xs["lp"] - 2}" y="{yA - 16}" width="4" height="32" fill="#f6c46b" fill-opacity="0.85"/>',
        f'<ellipse cx="{xs["obj"]}" cy="{yA}" rx="4" ry="16" fill="#7dd3fc" fill-opacity="0.18" stroke="#7dd3fc" stroke-width="1.5"/>',
        f'<line x1="{xs["slit"]}" y1="{yA - 18}" x2="{xs["slit"]}" y2="{yA - 2}" stroke="#e2e8f0" stroke-width="2.4"/><line x1="{xs["slit"]}" y1="{yA + 2}" x2="{xs["slit"]}" y2="{yA + 18}" stroke="#e2e8f0" stroke-width="2.4"/>',
        f'<ellipse cx="{xs["fl"]}" cy="{yA}" rx="3" ry="12" fill="none" stroke="#34d399" stroke-width="1.5"/>',
        f'<ellipse cx="{xs["coll"]}" cy="{yA}" rx="4" ry="18" fill="#7dd3fc" fill-opacity="0.18" stroke="#7dd3fc" stroke-width="1.5"/>',
        f'<rect x="{xs["gr"] - 2}" y="{yA - 18}" width="4" height="36" fill="#c4b5fd" fill-opacity="0.8"/>',
        f'<path d="M{xs["gr"] + 2} {yA} L{xs["sen"] - 4} {yA - 22} L{xs["sen"] - 4} {yA - 6}Z" fill="url(#spectrum)" fill-opacity="0.85"/>',
        f'<g transform="translate({xs["cam"] + 14} {yA - 12}) rotate(-22)"><ellipse cx="0" cy="0" rx="4" ry="14" fill="#7dd3fc" fill-opacity="0.18" stroke="#7dd3fc" stroke-width="1.5"/></g>',
        f'<g transform="translate({xs["sen"]} {yA - 14}) rotate(-22)"><rect x="-3" y="-15" width="6" height="30" fill="#34d399"/></g>',
        f'<path d="M{IX + 124} {yA + 44} L{IX + 124} {yA + 52} L{IX + 386} {yA + 52} L{IX + 386} {yA + 44}" fill="none" stroke="#8b93b0" stroke-width="1.1"/>',
        f'<text x="{IX + 255}" y="{yA + 74}" class="ls" text-anchor="middle">spectrograph: filter, objective, slit,</text>',
        f'<text x="{IX + 255}" y="{yA + 93}" class="ls" text-anchor="middle">collimator, grating, 12 mm lens + IMX219</text>',
    ])

svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" role="img" aria-labelledby="t d">
  <title id="t">The rig: SO-101 arm with the scanner head</title>
  <desc id="d">Side view of the setup. An SO-101 arm holds the scanner head above a potted plant on an AprilTag board. A scan mirror on the head sweeps a line of light across the plant, an RGB camera on the head sees the tags, halogen lamps light the scene, and a Jetson Nano and an ESP32 sit on the table. A faded second arm pose shows the next viewpoint. An inset shows the head's internals.</desc>
  <style>
    text {{ font-family: {FONT}; }}
    .h1 {{ font-size: 30px; font-weight: 800; fill: #f1f5ff; }}
    .sub {{ font-size: 17px; fill: #a3acc9; }}
    .lt {{ font-size: 18px; font-weight: 700; fill: #e6e9f2; }}
    .ls {{ font-size: 15.5px; fill: #8b93b0; }}
    .num {{ font-size: 14px; font-weight: 800; fill: #7dd3fc; }}
    .small {{ font-size: 14px; fill: #8b93b0; font-style: italic; }}
  </style>
  <defs>
    <linearGradient id="bg" x1="0" y1="0" x2="1" y2="1">
      <stop offset="0" stop-color="#101a3a"/><stop offset="0.55" stop-color="#0a1024"/><stop offset="1" stop-color="#060914"/>
    </linearGradient>
    <pattern id="holes" width="28" height="28" patternUnits="userSpaceOnUse"><circle cx="14" cy="14" r="1.7" fill="#1d2955"/></pattern>
    <radialGradient id="fade" cx="0.5" cy="0.6" r="0.75"><stop offset="0" stop-color="#fff"/><stop offset="1" stop-color="#fff" stop-opacity="0"/></radialGradient>
    <mask id="holesMask"><rect width="{W}" height="{H}" fill="url(#fade)"/></mask>
    <linearGradient id="spectrum" x1="0" x2="1">{spectrum_stops()}</linearGradient>
    <linearGradient id="armGrad" x1="0" y1="0" x2="1" y2="1"><stop offset="0" stop-color="#2b3a63"/><stop offset="1" stop-color="#18213f"/></linearGradient>
    <linearGradient id="sheet" gradientUnits="userSpaceOnUse" x1="{f(M_MAIN[0])}" y1="{f(M_MAIN[1])}" x2="{XS}" y2="600">
      <stop offset="0" stop-color="#fff" stop-opacity="0.5"/><stop offset="1" stop-color="#fff" stop-opacity="0.06"/>
    </linearGradient>
    <radialGradient id="lampGlow" cx="1052" cy="372" r="330" gradientUnits="userSpaceOnUse">
      <stop offset="0" stop-color="#fbbf24" stop-opacity="0.28"/><stop offset="1" stop-color="#fbbf24" stop-opacity="0"/>
    </radialGradient>
    <linearGradient id="tableFace" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#141c38"/><stop offset="1" stop-color="#141c38" stop-opacity="0"/></linearGradient>
    <filter id="glow" x="-50%" y="-50%" width="200%" height="200%"><feGaussianBlur stdDeviation="3.5" result="b"/><feMerge><feMergeNode in="b"/><feMergeNode in="SourceGraphic"/></feMerge></filter>
    <clipPath id="plantClip">{plant_clip()}</clipPath>
    <marker id="ah" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0 0 L10 5 L0 10z" fill="#e6e9f2"/></marker>
  </defs>

  <rect width="{W}" height="{H}" rx="26" fill="url(#bg)"/>
  <rect width="{W}" height="{H}" rx="26" fill="url(#holes)" mask="url(#holesMask)"/>

  <text x="48" y="68" class="h1">The rig</text>
  <text x="48" y="100" class="sub">The arm carries the scanner to a viewpoint and holds still.</text>
  <text x="48" y="124" class="sub">A mirror then sweeps the slit across the object, one line per frame.</text>

  <!-- lamp light -->
  <path d="M1052 372 L640 470 L700 652 L1000 652Z" fill="url(#lampGlow)"/>

  <!-- table -->
  <rect x="36" y="{TABLE_Y}" width="1208" height="12" rx="3" fill="#18213f" stroke="#2c3966"/>
  <rect x="36" y="{TABLE_Y + 12}" width="1208" height="60" fill="url(#tableFace)"/>

  <!-- AprilTag board -->
  <path d="M520 {TABLE_Y} L900 {TABLE_Y} L930 {TABLE_Y - 26} L550 {TABLE_Y - 26}Z" fill="#141c38" stroke="#34436f"/>
  {"".join(tag(x, TABLE_Y - 22) for x in (548, 588, 832, 872))}
  {"".join(tag(x, TABLE_Y - 11) for x in (538, 578, 822, 862))}

  <!-- RGB pose camera view -->
  <path d="M{f(W1[0] + 64)} {f(W1[1] - 4)} L548 {TABLE_Y - 20} M{f(W1[0] + 64)} {f(W1[1] - 4)} L900 {TABLE_Y - 14}" stroke="#7dd3fc" stroke-opacity="0.45" stroke-width="1.3" stroke-dasharray="5 5" fill="none"/>

  <!-- plant -->
  {plant()}

  <!-- next viewpoint: a faded head -->
  <g opacity="0.35" transform="translate({f(GHOST_M[0])} {f(GHOST_M[1])}) rotate({GHOST_ANG}) translate({-HEAD_LEN} 0)">
    <rect x="26" y="-21" width="80" height="42" rx="8" fill="none" stroke="#c9d3f5" stroke-width="1.5" stroke-dasharray="5 4"/>
    <rect x="106" y="-15" width="20" height="30" rx="3" fill="none" stroke="#c9d3f5" stroke-dasharray="4 3"/>
  </g>
  <path d="M{f(M_MAIN[0] - 48)} {f(M_MAIN[1] - 96)} Q{f(M_MAIN[0] - 34)} {f(M_MAIN[1] - 160)} {f(GHOST_M[0] - 50)} {f(GHOST_M[1] - 112)}" fill="none" stroke="#e6e9f2" stroke-opacity="0.6" stroke-width="1.4" stroke-dasharray="4 4" marker-end="url(#ah)"/>
  <text x="{f(GHOST_M[0] - 20)}" y="{f(GHOST_M[1] - 150)}" class="small" text-anchor="middle">next viewpoint</text>

  <!-- light sheet + scan line -->
  <polygon points="{sheet(XS)}" fill="url(#sheet)">
    <animate attributeName="points" values="{";".join(sheet(v) for v in (X0, X1, X1, X1, X0))}" keyTimes="{KT}" dur="{DUR}" repeatCount="indefinite"/>
    <animate attributeName="opacity" values="1;1;1;0;0" keyTimes="{KT}" dur="{DUR}" repeatCount="indefinite"/>
  </polygon>
  <g>
    <line x1="{XS}" y1="{LINE_TOP}" x2="{XS}" y2="{LINE_BOT}" stroke="#fff" stroke-opacity="0.15" stroke-width="2"/>
    <g clip-path="url(#plantClip)" filter="url(#glow)"><rect x="{XS - 2}" y="{LINE_TOP}" width="4" height="{LINE_BOT - LINE_TOP}" fill="#fff"/></g>
    <animateTransform attributeName="transform" type="translate" values="{";".join(f"{v - XS} 0" for v in (X0, X1, X1, X1, X0))}" keyTimes="{KT}" dur="{DUR}" repeatCount="indefinite"/>
    <animate attributeName="opacity" values="1;1;1;0;0" keyTimes="{KT}" dur="{DUR}" repeatCount="indefinite"/>
  </g>
  <path d="M{f(M_MAIN[0] + 52)} {f(M_MAIN[1] + 30)} A60 60 0 0 1 {f(M_MAIN[0] + 92)} {f(M_MAIN[1] + 92)}" fill="none" stroke="#e6e9f2" stroke-width="1.6" marker-start="url(#ah)" marker-end="url(#ah)"/>

  <!-- arm base, electronics -->
  <rect x="248" y="{TABLE_Y - 12}" width="114" height="12" rx="4" fill="#1a2340" stroke="#34436f"/>
  <rect x="262" y="{TABLE_Y - 60}" width="86" height="50" rx="10" fill="url(#armGrad)" stroke="#3c4c7c"/>
  <rect x="276" y="{TABLE_Y - 98}" width="58" height="40" rx="8" fill="#151d38" stroke="#3c4c7c"/>
  {arm(E1, W1, ANG_MAIN)}

  <!-- ESP32 + driver -->
  <rect x="400" y="{TABLE_Y - 18}" width="70" height="18" rx="3" fill="#123b2a" stroke="#2f8a57"/>
  <rect x="412" y="{TABLE_Y - 24}" width="24" height="8" rx="2" fill="#94a3b8"/>
  <rect x="444" y="{TABLE_Y - 26}" width="18" height="10" rx="2" fill="#7c3aed" fill-opacity="0.8"/>

  <!-- Jetson Nano -->
  <rect x="1000" y="{TABLE_Y - 40}" width="124" height="40" rx="5" fill="#123b2a" stroke="#2f8a57"/>
  <rect x="1030" y="{TABLE_Y - 66}" width="64" height="28" rx="3" fill="#2b3550" stroke="#4b5f95"/>
  {"".join(f'<line x1="{1034 + 6*k}" y1="{TABLE_Y - 64}" x2="{1034 + 6*k}" y2="{TABLE_Y - 40}" stroke="#4b5f95"/>' for k in range(10))}

  <!-- halogen lamp -->
  <line x1="1150" y1="{TABLE_Y}" x2="1150" y2="372" stroke="#4b5f95" stroke-width="5"/>
  <rect x="1126" y="{TABLE_Y - 8}" width="48" height="8" rx="3" fill="#1a2340" stroke="#34436f"/>
  <g transform="translate(1084 368) rotate(18)">
    <path d="M-34 -18 L18 -24 L18 24 L-34 18Z" fill="#232c4a" stroke="#4b5f95"/>
    <ellipse cx="-34" cy="0" rx="6" ry="18" fill="#fde68a"/>
  </g>
  <line x1="1150" y1="378" x2="1100" y2="372" stroke="#4b5f95" stroke-width="4"/>

  <!-- labels -->
  {label(1, 60, 300, "SO-101 follower arm", ["LeRobot, six STS3215 servos.", "Moves to a viewpoint,", "then holds still."])}
  {leader([(250, 352), (296, 400)])}
  {label(5, 60, 520, "ESP32 + TMC2209", ["micro-ROS node that", "steps the mirror."])}
  {leader([(220, 560), (398, TABLE_Y - 12)])}
  {label(2, 470, 712, "AprilTag board", ["Pose of every viewpoint."])}
  {leader([(500, 690), (556, TABLE_Y - 14)])}
  {label(4, 900, 712, "Jetson Nano", ["Capture, calibration warp", "and the CUDA splat renderer."])}
  {leader([(930, 690), (1004, TABLE_Y - 8)])}
  {label(3, 1138, 474, "Halogen lamps", ["Fixed, NIR-rich light."], anchor="end")}
  <text x="540" y="{f(M_MAIN[1] + 76)}" class="small" text-anchor="end">mirror sweep:</text>
  <text x="540" y="{f(M_MAIN[1] + 94)}" class="small" text-anchor="end">~150 lines per viewpoint</text>
  {inset()}
</svg>
'''
open(OUT, "w", encoding="utf-8", newline="\n").write(svg)
print("wrote", OUT, len(svg))
