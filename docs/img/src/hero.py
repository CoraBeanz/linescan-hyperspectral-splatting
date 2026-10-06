"""Animated hero banner for the README (docs/img/hero.svg).

Run: python docs/img/src/hero.py  [out.svg]   (default: docs/img/hero.svg)
The scan line, the splat reveal and the spectrum plot are SMIL animations, which GitHub plays
in README images. The plant reflectance curve is illustrative, not measured.
"""
import math, random, sys
from pathlib import Path
from common import *

random.seed(11)
W, H = 1280, 600
OUT = sys.argv[1] if len(sys.argv) > 1 else str(Path(__file__).resolve().parent.parent / "hero.svg")

def f(x): return f"{x:.1f}".rstrip('0').rstrip('.')

# ---------------- plant geometry ----------------
LEAVES = [  # base x, base y, angle (deg, 0 = right, 90 = up), length, half-width
    (1124, 428, 166, 118, 27),
    (1136, 420, 16, 106, 26),
    (1126, 390, 128, 112, 25),
    (1134, 384, 56, 114, 25),
    (1130, 362, 93, 92, 19),
]
POT = dict(top=446, bottom=506, tl=1094, tr=1166, bl=1101, br=1159)

def leaf_point(leaf, u, v):
    bx, by, ang, L, hw = leaf
    a = math.radians(ang)
    ax, ay = math.cos(a), -math.sin(a)          # along the leaf (SVG y down)
    nx, ny = -ay, ax                              # normal
    w = 4 * hw * u * (1 - u) * v
    return bx + ax * L * u + nx * w, by + ay * L * u + ny * w

def leaf_path(leaf):
    bx, by, ang, L, hw = leaf
    a = math.radians(ang)
    ax, ay = math.cos(a), -math.sin(a)
    nx, ny = -ay, ax
    tx, ty = bx + ax * L, by + ay * L
    mx, my = bx + ax * L / 2, by + ay * L / 2
    c1 = (mx + nx * 2 * hw, my + ny * 2 * hw)
    c2 = (mx - nx * 2 * hw, my - ny * 2 * hw)
    return (f"M{f(bx)} {f(by)} Q{f(c1[0])} {f(c1[1])} {f(tx)} {f(ty)} "
            f"Q{f(c2[0])} {f(c2[1])} {f(bx)} {f(by)}Z"), (bx, by, tx, ty)

def pot_path():
    p = POT
    return f"M{p['tl']} {p['top']} L{p['tr']} {p['top']} L{p['br']} {p['bottom']} L{p['bl']} {p['bottom']}Z"

STEM = "M1130 504 C1128 470 1132 420 1130 362"

LEAF_COLORS = ["#ff3d6e", "#ff4f86", "#f2366a", "#ff6b9a", "#e02d63", "#ff5577", "#ff7aa8"]
POT_COLORS = ["#ffb547", "#ffa53a", "#f59e0b", "#ffc561", "#f7a634"]

def gaussians():
    out = []
    for leaf in LEAVES:
        n = int(leaf[3] * leaf[4] / 30)
        for _ in range(n):
            u = random.uniform(0.06, 0.94)
            v = random.uniform(-0.85, 0.85)
            x, y = leaf_point(leaf, u, v)
            rx, ry = random.uniform(4.0, 9.0), random.uniform(2.2, 4.6)
            rot = -leaf[2] + random.uniform(-28, 28)
            c = random.choice(LEAF_COLORS)
            op = random.uniform(0.45, 0.85)
            out.append(f'<ellipse cx="{f(x)}" cy="{f(y)}" rx="{f(rx)}" ry="{f(ry)}" '
                       f'transform="rotate({f(rot)} {f(x)} {f(y)})" fill="{c}" fill-opacity="{op:.2f}"/>')
    p = POT
    for _ in range(60):
        t = random.uniform(0.04, 0.96)
        y = p['top'] + t * (p['bottom'] - p['top'])
        xl = p['tl'] + t * (p['bl'] - p['tl']); xr = p['tr'] + t * (p['br'] - p['tr'])
        x = random.uniform(xl + 4, xr - 4)
        rx, ry = random.uniform(5, 11), random.uniform(3, 6)
        rot = random.uniform(-20, 20)
        out.append(f'<ellipse cx="{f(x)}" cy="{f(y)}" rx="{f(rx)}" ry="{f(ry)}" '
                   f'transform="rotate({f(rot)} {f(x)} {f(y)})" fill="{random.choice(POT_COLORS)}" '
                   f'fill-opacity="{random.uniform(0.5, 0.85):.2f}"/>')
    for k in range(8):  # stem
        y = 368 + k * 10 + random.uniform(-3, 3)
        out.append(f'<ellipse cx="{f(1130 + random.uniform(-1.5, 1.5))}" cy="{f(y)}" rx="2.6" ry="7" '
                   f'fill="#ff5c7a" fill-opacity="0.75"/>')
    return "\n      ".join(out)

# ---------------- arm geometry ----------------
S, E, Wr = (722, 432), (742, 246), (872, 128)
HEAD_ANGLE = 35  # degrees, clockwise from +x (SVG)
def head_pt(lx, ly):
    a = math.radians(HEAD_ANGLE)
    return (Wr[0] + lx * math.cos(a) - ly * math.sin(a), Wr[1] + lx * math.sin(a) + ly * math.cos(a))
M = head_pt(128, 0)  # mirror / scan apex

# ---------------- scan animation ----------------
X0, X1, XS = 1000, 1246, 1150       # sweep range, static (no-animation) position
DUR = "9s"
KT = "0;0.6;0.78;0.84;1"             # sweep, hold, fade, idle
def xs_vals(): return f"{X0};{X1};{X1};{X1};{X0}"
LINE_TOP, LINE_BOT = 262, 512

def sheet_pts(x):
    return f"{f(M[0])},{f(M[1])} {x},{LINE_TOP} {x},{LINE_BOT}"

# ---------------- callout chart ----------------
CARD = (1028, 30, 224, 176)   # x, y, w, h
REFL = [(500, .06), (530, .09), (550, .14), (570, .11), (600, .08), (640, .06), (670, .045),
        (690, .07), (705, .14), (720, .27), (735, .39), (750, .45), (780, .49), (850, .51),
        (950, .50)]
def chart():
    x, y, w, h = CARD
    px0, px1 = x + 22, x + w - 14
    py0, py1 = y + h - 34, y + 50            # bottom (0), top (0.55)
    sx = lambda nm: px0 + (nm - 500) / 450 * (px1 - px0)
    sy = lambda r: py0 - r / 0.55 * (py0 - py1)
    pts = [(sx(nm), sy(r)) for nm, r in REFL]
    d = "M" + " L".join(f"{f(a)} {f(b)}" for a, b in pts)
    area = d + f" L{f(px1)} {f(py0)} L{f(px0)} {f(py0)}Z"
    ticks = "".join(
        f'<text x="{f(sx(nm))}" y="{f(py0 + 17)}" text-anchor="middle" class="tick">{nm}</text>'
        for nm in (500, 700))
    return f'''
    <rect x="{x}" y="{y}" width="{w}" height="{h}" rx="14" fill="#0e1631" fill-opacity="0.94" stroke="#2c3b6e"/>
    <text x="{x + 16}" y="{y + 26}" class="cardtitle">one Gaussian's spectrum</text>
    <line x1="{f(px0)}" y1="{f(py0)}" x2="{f(px1)}" y2="{f(py0)}" stroke="#2c3b6e"/>
    <path d="{area}" fill="url(#chartFill)" opacity="0.35"/>
    <path d="{d}" fill="none" stroke="url(#chartLine)" stroke-width="3" stroke-linejoin="round" stroke-linecap="round"
          pathLength="1" stroke-dasharray="1" stroke-dashoffset="0">
      <animate attributeName="stroke-dashoffset" values="1;0;0" keyTimes="0;0.25;1" dur="{DUR}" repeatCount="indefinite"/>
    </path>
    <text x="{f(sx(712))}" y="{f(sy(0.30))}" text-anchor="end" class="anno">red edge</text>
    {ticks}
    <text x="{f(sx(900))}" y="{f(py0 + 17)}" text-anchor="middle" class="tick">900 nm</text>'''

def chart_grads():
    x, y, w, h = CARD
    px0, px1 = x + 22, x + w - 14
    stops = "".join(f'<stop offset="{(nm - 500) / 450:.3f}" stop-color="{c}"/>'
                    for nm, c in SPECTRUM if nm <= 950)
    return (f'<linearGradient id="chartLine" gradientUnits="userSpaceOnUse" x1="{px0}" y1="0" x2="{px1}" y2="0">{stops}'
            f'<stop offset="1" stop-color="{wl_hex(950)}"/></linearGradient>'
            f'<linearGradient id="chartFill" gradientUnits="userSpaceOnUse" x1="{px0}" y1="0" x2="{px1}" y2="0">{stops}'
            f'<stop offset="1" stop-color="{wl_hex(950)}"/></linearGradient>')

# ---------------- stat chips ----------------
CHIPS = [("500–950 nm", "usable band"), ("3.7 nm", "resolution*"),
         ("2,204 px", "per scan line"), ("&lt; $500", "optics + scanner")]
def chips(x0=64, y0=356):
    out, w, gap = [], 128, 12
    for k, (big, small) in enumerate(CHIPS):
        x = x0 + k * (w + gap)
        out.append(f'''
    <g transform="translate({x} {y0})">
      <rect width="{w}" height="78" rx="14" fill="#0f1833" stroke="#25325e"/>
      <text x="13" y="35" class="chipbig">{big}</text>
      <text x="13" y="59" class="chipsmall">{small}</text>
    </g>''')
    return "".join(out)

leaf_paths = [leaf_path(l) for l in LEAVES]
plant_shapes = "".join(f'<path d="{p}"/>' for p, _ in leaf_paths) + f'<path d="{pot_path()}"/>' \
               + f'<path d="{STEM}" fill="none" stroke="#000" stroke-width="5"/>'
plain_leaves = "".join(
    f'<path d="{p}" fill="#1d6b40" stroke="#2f8a57" stroke-width="1.2"/>'
    f'<line x1="{f(b[0])}" y1="{f(b[1])}" x2="{f(b[2])}" y2="{f(b[3])}" stroke="#3aa56a" stroke-width="1.4" stroke-opacity="0.8"/>'
    for p, b in leaf_paths)

# highlighted Gaussian with a leader to the card
HX, HY = leaf_point(LEAVES[2], 0.58, 0.15)

svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" role="img" aria-labelledby="t d">
  <title id="t">Line-Scan Hyperspectral 3D Gaussian Splatting</title>
  <desc id="d">An SO-101 robot arm carries a pushbroom spectrograph. A scan mirror sweeps a line of light across a potted plant; behind the line the plant turns into colored 3D Gaussians, and one Gaussian's stored spectrum is shown in a callout.</desc>
  <style>
    text {{ font-family: {FONT}; }}
    .eyebrow {{ font-size: 13.5px; letter-spacing: 2.6px; font-weight: 700; fill: #7dd3fc; }}
    .title {{ font-size: 46px; font-weight: 800; letter-spacing: -0.6px; }}
    .tag {{ font-size: 19px; fill: #a3acc9; }}
    .chipbig {{ font-size: 19px; font-weight: 800; fill: #f1f5ff; }}
    .chipsmall {{ font-size: 13.5px; fill: #8b93b0; }}
    .foot {{ font-size: 13.5px; fill: #6b7394; }}
    .cardtitle {{ font-size: 15px; font-weight: 700; fill: #e6e9f2; }}
    .tick {{ font-size: 12.5px; fill: #8b93b0; font-family: {MONO}; }}
    .anno {{ font-size: 12.5px; fill: #c9cfe6; font-style: italic; }}
    .pill {{ font-size: 13px; font-weight: 700; fill: #fbbf24; letter-spacing: 0.4px; }}
  </style>
  <defs>
    <linearGradient id="bg" x1="0" y1="0" x2="1" y2="1">
      <stop offset="0" stop-color="#101a3a"/><stop offset="0.55" stop-color="#0a1024"/><stop offset="1" stop-color="#060914"/>
    </linearGradient>
    <pattern id="holes" width="28" height="28" patternUnits="userSpaceOnUse">
      <circle cx="14" cy="14" r="1.7" fill="#1d2955"/>
    </pattern>
    <radialGradient id="fade" cx="0.72" cy="0.55" r="0.7">
      <stop offset="0" stop-color="#fff" stop-opacity="1"/><stop offset="1" stop-color="#fff" stop-opacity="0"/>
    </radialGradient>
    <mask id="holesMask"><rect width="{W}" height="{H}" fill="url(#fade)"/></mask>
    <linearGradient id="titleGrad" x1="0" x2="1">
      <stop offset="0" stop-color="#00f5a0"/><stop offset="0.22" stop-color="#c8ff00"/>
      <stop offset="0.38" stop-color="#ffe600"/><stop offset="0.55" stop-color="#ffa600"/>
      <stop offset="0.72" stop-color="#ff5e00"/><stop offset="0.86" stop-color="#ff2a55"/>
      <stop offset="1" stop-color="#e040a0"/>
    </linearGradient>
    <linearGradient id="spectrum" x1="0" x2="1">{spectrum_stops()}</linearGradient>
    {chart_grads()}
    <linearGradient id="sheet" gradientUnits="userSpaceOnUse" x1="{f(M[0])}" y1="{f(M[1])}" x2="{X1}" y2="420">
      <stop offset="0" stop-color="#ffffff" stop-opacity="0.55"/><stop offset="1" stop-color="#ffffff" stop-opacity="0.06"/>
    </linearGradient>
    <linearGradient id="armGrad" x1="0" y1="0" x2="1" y2="1">
      <stop offset="0" stop-color="#2b3a63"/><stop offset="1" stop-color="#18213f"/>
    </linearGradient>
    <filter id="glow" x="-50%" y="-50%" width="200%" height="200%">
      <feGaussianBlur stdDeviation="4" result="b"/>
      <feMerge><feMergeNode in="b"/><feMergeNode in="SourceGraphic"/></feMerge>
    </filter>
    <filter id="soft" x="-20%" y="-20%" width="140%" height="140%"><feGaussianBlur stdDeviation="0.7"/></filter>
    <clipPath id="plantClip">{plant_shapes}</clipPath>
    <clipPath id="scanned">
      <rect x="960" y="230" width="{XS - 960}" height="300">
        <animate attributeName="width" values="{";".join(str(v - 960) for v in (X0, X1, X1, X1, X0))}" keyTimes="{KT}" dur="{DUR}" repeatCount="indefinite"/>
      </rect>
    </clipPath>
    <clipPath id="unscanned">
      <rect x="{XS}" y="230" width="{1270 - XS}" height="300">
        <animate attributeName="x" values="{xs_vals()}" keyTimes="{KT}" dur="{DUR}" repeatCount="indefinite"/>
      </rect>
    </clipPath>
  </defs>

  <rect width="{W}" height="{H}" rx="26" fill="url(#bg)"/>
  <rect width="{W}" height="{H}" rx="26" fill="url(#holes)" mask="url(#holesMask)"/>

  <!-- left column: pitch -->
  <g transform="translate(64 0)">
    <rect x="0" y="62" width="196" height="30" rx="15" fill="#fbbf24" fill-opacity="0.12" stroke="#fbbf24" stroke-opacity="0.45"/>
    <circle cx="18" cy="77" r="4.5" fill="#fbbf24">
      <animate attributeName="opacity" values="1;0.25;1" dur="2.4s" repeatCount="indefinite"/>
    </circle>
    <text x="31" y="82" class="pill">design &amp; modeling</text>
    <text x="0" y="132" class="eyebrow">HYPERSPECTRAL × ROBOTICS × CUDA</text>
    <text x="0" y="190" class="title" fill="#f1f5ff">Line-Scan Hyperspectral</text>
    <text x="0" y="246" class="title" fill="url(#titleGrad)">3D Gaussian Splatting</text>
    <text x="0" y="292" class="tag">A pushbroom spectrograph rides an SO-101 arm,</text>
    <text x="0" y="320" class="tag">and every Gaussian in the splat stores a spectrum.</text>
  </g>
  {chips()}
  <g transform="translate(64 470)">
    <rect width="548" height="10" rx="5" fill="url(#spectrum)"/>
    <text x="0" y="33" class="tick">500 nm</text>
    <line x1="219" y1="-4" x2="219" y2="16" stroke="#e6e9f2" stroke-opacity="0.5"/>
    <text x="211" y="33" class="tick" text-anchor="end">visible</text>
    <text x="227" y="33" class="tick">near-infrared</text>
    <text x="548" y="33" class="tick" text-anchor="end">1000 nm</text>
  </g>
  <text x="64" y="552" class="foot">* geometric, with ideal lenses (Optiland model); real M12 lenses will land around 5–8 nm</text>

  <!-- table + tag board -->
  <line x1="652" y1="522" x2="1250" y2="522" stroke="#26315a" stroke-width="2"/>
  <path d="M968 522 L1236 522 L1252 500 L986 500Z" fill="#141c38" stroke="#2c3966"/>
  {"".join(f'<rect x="{x}" y="505" width="13" height="12" fill="#e6e9f2" fill-opacity="0.85"/><rect x="{x+3}" y="508" width="4" height="4" fill="#141c38"/><rect x="{x+7}" y="511" width="3" height="3" fill="#141c38"/>' for x in (992, 1022, 1166, 1196, 1226) )}

  <!-- the plant: plain where not scanned yet -->
  <g clip-path="url(#unscanned)">
    <path d="{STEM}" fill="none" stroke="#2f8a57" stroke-width="5" stroke-linecap="round"/>
    {plain_leaves}
    <path d="{pot_path()}" fill="#7d4430" stroke="#9a5a3f"/>
    <rect x="1088" y="438" width="84" height="12" rx="3" fill="#8f4f37" stroke="#a8644a"/>
  </g>

  <!-- the plant: Gaussians where already scanned -->
  <g clip-path="url(#scanned)">
    <g filter="url(#soft)">
      {gaussians()}
    </g>
    <ellipse cx="{f(HX)}" cy="{f(HY)}" rx="9" ry="5" transform="rotate({-LEAVES[2][2]} {f(HX)} {f(HY)})" fill="#ff7aa8" stroke="#fff" stroke-width="2"/>
  </g>
  <path d="M{f(HX)} {f(HY - 6)} C{f(HX)} {f(HY - 70)} {f(CARD[0] + 40)} {CARD[1] + CARD[3] + 50} {f(CARD[0] + 40)} {CARD[1] + CARD[3]}"
        fill="none" stroke="#c9cfe6" stroke-width="1.3" stroke-dasharray="4 4" opacity="0.8"/>

  <!-- light sheet and scan line -->
  <polygon points="{sheet_pts(XS)}" fill="url(#sheet)">
    <animate attributeName="points" values="{";".join(sheet_pts(v) for v in (X0, X1, X1, X1, X0))}" keyTimes="{KT}" dur="{DUR}" repeatCount="indefinite"/>
    <animate attributeName="opacity" values="1;1;1;0;0" keyTimes="{KT}" dur="{DUR}" repeatCount="indefinite"/>
  </polygon>
  <g>
    <line x1="{XS}" y1="{LINE_TOP}" x2="{XS}" y2="{LINE_BOT}" stroke="#ffffff" stroke-opacity="0.18" stroke-width="2"/>
    <g clip-path="url(#plantClip)" filter="url(#glow)">
      <rect x="{XS - 2}" y="{LINE_TOP}" width="4" height="{LINE_BOT - LINE_TOP}" fill="#ffffff"/>
    </g>
    <animateTransform attributeName="transform" type="translate" values="{";".join(f"{v - XS} 0" for v in (X0, X1, X1, X1, X0))}" keyTimes="{KT}" dur="{DUR}" repeatCount="indefinite"/>
    <animate attributeName="opacity" values="1;1;1;0;0" keyTimes="{KT}" dur="{DUR}" repeatCount="indefinite"/>
  </g>

  <!-- SO-101 arm (stylized) -->
  <g stroke-linecap="round" stroke-linejoin="round">
    <rect x="684" y="508" width="104" height="14" rx="4" fill="#1a2340" stroke="#34436f"/>
    <rect x="696" y="462" width="72" height="48" rx="10" fill="url(#armGrad)" stroke="#3c4c7c"/>
    <rect x="708" y="430" width="48" height="36" rx="8" fill="#151d38" stroke="#3c4c7c"/>
    <line x1="{S[0]}" y1="{S[1]}" x2="{E[0]}" y2="{E[1]}" stroke="#3c4c7c" stroke-width="36"/>
    <line x1="{S[0]}" y1="{S[1]}" x2="{E[0]}" y2="{E[1]}" stroke="url(#armGrad)" stroke-width="33"/>
    <line x1="{S[0] - 6}" y1="{S[1] - 20}" x2="{E[0] - 6}" y2="{E[1] + 20}" stroke="#4b5f95" stroke-width="2" opacity="0.6"/>
    <line x1="{E[0]}" y1="{E[1]}" x2="{Wr[0]}" y2="{Wr[1]}" stroke="#3c4c7c" stroke-width="31"/>
    <line x1="{E[0]}" y1="{E[1]}" x2="{Wr[0]}" y2="{Wr[1]}" stroke="url(#armGrad)" stroke-width="28"/>
    <line x1="{E[0] + 18}" y1="{E[1] - 18}" x2="{Wr[0] - 14}" y2="{Wr[1] - 8}" stroke="#4b5f95" stroke-width="2" opacity="0.6"/>
    {"".join(f'<circle cx="{x}" cy="{y}" r="{r}" fill="#121a33" stroke="#4b5f95" stroke-width="2"/><circle cx="{x}" cy="{y}" r="{r*0.42:.1f}" fill="#2b3a63"/>' for (x, y), r in ((S, 21), (E, 21), (Wr, 18)))}
    <g transform="translate({Wr[0]} {Wr[1]}) rotate({HEAD_ANGLE})">
      <rect x="12" y="-17" width="16" height="34" rx="4" fill="#151d38" stroke="#3c4c7c"/>
      <rect x="26" y="-21" width="78" height="42" rx="8" fill="url(#armGrad)" stroke="#4b5f95" stroke-width="1.5"/>
      <rect x="34" y="-10" width="62" height="20" rx="4" fill="#0a0f22" stroke="#2c3966"/>
      <line x1="38" y1="0" x2="90" y2="0" stroke="#e2e8f0" stroke-opacity="0.55" stroke-width="1.2"/>
      <ellipse cx="44" cy="0" rx="2" ry="7" fill="none" stroke="#7dd3fc" stroke-width="1.4"/>
      <line x1="56" y1="-7" x2="56" y2="-1.5" stroke="#e2e8f0" stroke-width="1.6"/><line x1="56" y1="1.5" x2="56" y2="7" stroke="#e2e8f0" stroke-width="1.6"/>
      <ellipse cx="70" cy="0" rx="2" ry="7" fill="none" stroke="#7dd3fc" stroke-width="1.4"/>
      <rect x="80" y="-7" width="2.5" height="14" fill="#c4b5fd"/>
      <path d="M83 0 L94 -6 L94 6Z" fill="url(#spectrum)" opacity="0.9"/>
      <rect x="40" y="-34" width="30" height="14" rx="3" fill="#151d38" stroke="#4b5f95"/>
      <circle cx="72" cy="-27" r="5" fill="#0a0f22" stroke="#7dd3fc" stroke-width="1.4"/>
      <rect x="104" y="-15" width="20" height="30" rx="3" fill="#121a33" stroke="#4b5f95"/>
      <line x1="120" y1="-14" x2="134" y2="12" stroke="#e2e8f0" stroke-width="3.2"/>
    </g>
  </g>
  {chart()}
</svg>
'''
open(OUT, "w", encoding="utf-8", newline="\n").write(svg)
print("wrote", OUT, len(svg))
