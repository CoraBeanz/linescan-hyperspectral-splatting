"""Shared palette and helpers for the SVG diagram scripts in this folder."""
SPECTRUM = [(500, "#00f5a0"), (530, "#5cff3a"), (560, "#c8ff00"), (580, "#ffe600"),
            (600, "#ffa600"), (630, "#ff5e00"), (660, "#ff2a1f"), (700, "#e8173a"),
            (800, "#d41a55"), (900, "#a61a6e"), (1000, "#7a1c80")]
FONT = "ui-sans-serif, -apple-system, 'Segoe UI', 'Helvetica Neue', Helvetica, Arial, sans-serif"
MONO = "ui-monospace, SFMono-Regular, Menlo, Consolas, 'Liberation Mono', monospace"
BG, FG, MUTED, FAINT = "#0b1020", "#e6e9f2", "#8b93b0", "#2a3358"
LENS, SLIT, GRATING, SENSOR, CLIP = "#7dd3fc", "#e2e8f0", "#c4b5fd", "#34d399", "#f87171"

def spectrum_stops(lo=500, hi=1000):
    out = []
    for nm, c in SPECTRUM:
        if lo <= nm <= hi:
            out.append(f'<stop offset="{(nm-lo)/(hi-lo):.3f}" stop-color="{c}"/>')
    return "".join(out)

def hex2rgb(h):
    h = h.lstrip('#'); return tuple(int(h[i:i+2], 16) for i in (0, 2, 4))

def wl_hex(nm):
    import bisect
    xs = [s[0] for s in SPECTRUM]
    nm = min(max(nm, xs[0]), xs[-1])
    i = max(1, bisect.bisect_left(xs, nm)); i = min(i, len(xs)-1)
    x0, c0 = SPECTRUM[i-1]; x1, c1 = SPECTRUM[i]
    t = 0 if x1 == x0 else (nm - x0) / (x1 - x0)
    a, b = hex2rgb(c0), hex2rgb(c1)
    return '#%02x%02x%02x' % tuple(round(a[k] + (b[k]-a[k])*t) for k in range(3))
