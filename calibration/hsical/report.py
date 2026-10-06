"""report.md plus a few plots, written next to the calibration.

The plots need matplotlib; without it the report is text only.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np


def write_report(result, out_dir):
    out = Path(out_dir)
    cal = result.calibration
    info = cal.info
    figs = []
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        figs = _plots(result, out, plt)
    except ImportError:
        pass
    d = info["dispersion"]
    lsf = info["lsf"]
    sm = info["smile_fit"]
    ks = cal.keystone
    lines = [
        "# Spectrometer calibration",
        "",
        f"Session `{info['session']}`, calibrated {info['created']}.",
        "",
        "## Checks",
        "",
        "| check | result | target | |",
        "|---|---|---|---|",
    ]
    for c in result.checks:
        verdict = {"pass": "pass", "FAIL": "**FAIL**: ", "note": "note: "}[c.status]
        lines.append(f"| {c.name} | {c.value} | {c.target} | {verdict}{'' if c.ok else c.advice} |")
    lines += [
        "",
        "## Numbers",
        "",
        f"* Frames: {cal.width} x {cal.height} after turning "
        f"(transpose {cal.orientation.transpose}, mirror x {cal.orientation.flip_x}, "
        f"mirror y {cal.orientation.flip_y}), binning {info['design_scale']:g}.",
        f"* Wavelength: {cal.wavelength(0):.1f} nm at column 0 to {cal.wavelength(cal.width - 1):.1f} nm "
        f"at column {cal.width - 1}; {d['px_per_nm_750']:.2f} px/nm at 750 nm.",
        f"* Fit: {sum(1 for f in info['lines'] if f['used'])} lines from {d['nm_covered'][0]:.1f} to "
        f"{d['nm_covered'][1]:.1f} nm, rms {d['rms_nm']:.3f} nm (extra scatter {d['extra_nm']:.3f} nm "
        "beyond noise, from line shapes and blends).",
        f"* Resolution: lines are {lsf['fwhm_px']:.1f} px wide = {lsf['resolution_nm']:.2f} nm FWHM "
        f"(slit-limited design value {lsf['design_resolution_nm']:.1f} nm); slit image "
        f"{lsf['w0']:.1f} px, blur sigma {lsf['s0']:.2f} px.",
        f"* Smile: {sm['n_lines']} lines followed, fit rms {sm['rms_px']:.2f} px; lines bow "
        f"{sm['bow_at_end_px']:+.1f} px at the slit's end.",
        f"* Slit: ends at rows {ks.s_top:.1f} and {ks.s_bottom:.1f} at column {ks.x_ref:.0f} (750 nm); "
        f"keystone fit rms {ks.rms_px:.2f} px from {len(ks.tracks)} tracks.",
    ]
    if "response_range_nm" in info:
        a, b = info["response_range_nm"]
        lines.append(f"* Response: above 5% of its peak from {a:.0f} to {b:.0f} nm "
                     f"(halogen taken as {cal.temp_k:.0f} K).")
    if "slit" in info:
        s = info["slit"]
        dust = ", ".join(f"row {x['s']:.0f} ({100 * x['depth']:.0f}%)" for x in s["dust"]) or "none"
        lines.append(f"* Slit evenness: dips {dust}; {100 * s['taper']:+.1f}% end to end.")
    if info.get("laser"):
        lz = info["laser"]
        lines.append(f"* Laser: {lz['nm']:.2f} nm, {lz['spread_nm']:.3f} nm rms along the slit, "
                     f"{lz['fwhm_nm']:.2f} nm wide.")
    lines.append(f"* Rectified output: {cal.nm_grid.size} wavelengths "
                 f"({cal.nm_grid[0]:.0f}-{cal.nm_grid[-1]:.0f} nm) x {cal.s_grid.size} slit rows.")
    if figs:
        lines += ["", "## Plots", ""]
        lines += [f"![{t}]({f})" for f, t in figs]
    lines += ["", "## Lamp lines", "",
              "| lamp | line | nm | column | +- px | SNR | lines in fit | used | residual nm |",
              "|---|---|---|---|---|---|---|---|---|"]
    for f in sorted(info["lines"], key=lambda f: f["nm"]):
        lines.append(f"| {f['lamp']} | {f['label']} | {f['nm']:.3f} | {f['x']:.2f} | {f['x_sigma']:.2f} | "
                     f"{f['snr']:.0f} | {f['n_members']} | {'yes' if f['used'] else 'no'} | "
                     f"{f['resid_nm']:+.3f} |")
    lines += ["", "## Log", "", "```"] + result.log + ["```", ""]
    (out / "report.md").write_text("\n".join(lines))
    return out / "report.md"


def _plots(result, out, plt):
    cal = result.calibration
    P = result.plots
    info = cal.info
    W = cal.width
    x = np.arange(W)
    nm = cal.wavelength(x)
    figs = []

    # lamp spectra with the lines that were fitted
    fig, axes = plt.subplots(len(P["spectra"]), 1, figsize=(11, 2.4 * len(P["spectra"])), squeeze=False)
    for ax, (name, spec) in zip(axes[:, 0], P["spectra"].items()):
        spec = np.asarray(spec, float)
        top = float(np.nanmax(spec))
        # log scale, so the faint near-infrared lines show next to the bright ones
        ax.semilogy(nm, np.clip(spec, 3e-4 * top, None), lw=0.7, color="k")
        for f in P["fits"]:
            if f["lamp"] == name:
                ax.axvline(f["nm"], color="tab:green" if f["used"] else "tab:red", lw=0.9, alpha=0.8,
                           zorder=0)
        ax.set_ylim(3e-4 * top, 2 * top)
        ax.set_ylabel(name)
        ax.set_xlim(nm[0], nm[-1])
    axes[-1, 0].set_xlabel("wavelength (nm)")
    axes[0, 0].set_title("Lamp spectra along the middle of the slit, log scale "
                         "(green: lines used, red: rejected)")
    figs.append(_save(fig, out, "lamps.png", "lamp spectra", plt))

    # dispersion and residuals
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    a1.plot(nm, 1.0 / cal.wavelength.deriv(x), color="k")
    a1.set_ylabel("px per nm")
    a1.set_title("Dispersion, and how far each line sits from the fit")
    sig = np.asarray(P["sigma"])
    a2.fill_between(nm, -sig, sig, color="0.85", label="fit uncertainty")
    for used, c in ((True, "tab:blue"), (False, "tab:red")):
        fs = [f for f in P["fits"] if f["used"] == used]
        if fs:
            err = [np.hypot(f["x_sigma"] * abs(float(cal.wavelength.deriv(f["x"]))), f["blend_nm"])
                   for f in fs]
            a2.errorbar([f["nm"] for f in fs], [f["resid_nm"] for f in fs], yerr=err, fmt="o", ms=3,
                        color=c, lw=0.8, label="used" if used else "rejected")
    a2.axhline(0, color="k", lw=0.5)
    a2.set_ylabel("line - fit (nm)")
    a2.set_xlabel("wavelength (nm)")
    a2.legend(fontsize=8)
    figs.append(_save(fig, out, "dispersion.png", "dispersion and residuals", plt))

    # smile traces
    fig, ax = plt.subplots(figsize=(7, 6))
    cmap = plt.get_cmap("turbo")
    for t in P["traces"]:
        y = np.asarray(t["y"])
        xx = np.asarray(t["x"])
        lam = float(cal.wavelength(t["x0"]))
        col = cmap((lam - 480) / 560)
        ax.plot(xx - t["x0"], y, ".", ms=2, color=col)
        yy = np.linspace(y.min(), y.max(), 50)
        ax.plot(cal.smile(np.full_like(yy, t["x0"]), yy), yy, "-", lw=0.6, color=col)
    ax.invert_yaxis()
    ax.set_xlabel("line position - position at the slit's middle (px)")
    ax.set_ylabel("row")
    ax.set_title("Smile: lamp lines along the slit (colour = wavelength)")
    figs.append(_save(fig, out, "smile.png", "smile", plt))

    # keystone tracks
    if cal.keystone.tracks and "x" in cal.keystone.tracks[0]:
        fig, ax = plt.subplots(figsize=(9, 4.5))
        for t in cal.keystone.tracks:
            tx, ty = np.asarray(t["x"]), np.asarray(t["y"])
            ax.plot(cal.wavelength(tx), ty - t["s"], ".", ms=2,
                    label=f"{t['kind']} at row {t['s']:.0f}")
            ax.plot(cal.wavelength(tx), -cal.keystone.delta(tx, ty), "-", lw=0.6, color="k")
        ax.set_xlabel("wavelength (nm)")
        ax.set_ylabel("row - row at 750 nm (px)")
        ax.set_title("Keystone: slit ends and wire shadows across the spectrum (black: fit)")
        ax.legend(fontsize=7, ncol=3)
        figs.append(_save(fig, out, "keystone.png", "keystone", plt))

    # response and slit evenness
    if "response" in P:
        fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4), gridspec_kw=dict(width_ratios=[3, 2]))
        r = P["response"]
        peak = np.nanmax(r["all"])
        for k, c in (("R", "tab:red"), ("G", "tab:green"), ("B", "tab:blue"), ("all", "k")):
            a1.plot(nm, np.asarray(r[k]) / peak, color=c, lw=1.2 if k == "all" else 0.8,
                    label="all pixels" if k == "all" else f"{k} pixels")
        a1.set_xlabel("wavelength (nm)")
        a1.set_ylabel("relative response")
        a1.set_title(f"Spectral response (halogen at {cal.temp_k:.0f} K)")
        a1.legend(fontsize=8)
        a1.set_xlim(nm[0], nm[-1])
        if "slit_profile" in P:
            a2.plot(cal.s_grid, P["slit_profile"], color="k", lw=0.8)
            for dct in info.get("slit", {}).get("dust", []):
                a2.axvline(dct["s"], color="tab:red", lw=0.6)
            a2.set_xlabel("slit row")
            a2.set_ylabel("relative brightness")
            a2.set_title("Along the slit (red: dips, likely dust)")
        figs.append(_save(fig, out, "response.png", "response and slit evenness", plt))

    # rectified lamp
    if "rectified_lamp" in P:
        img = np.asarray(P["rectified_lamp"]["img"], float)
        fig, ax = plt.subplots(figsize=(10, 5))
        v = np.nanpercentile(img, 99.5)
        ax.imshow(np.sqrt(np.clip(img / v, 0, 1)), aspect="auto", cmap="gray",
                  extent=[cal.nm_grid[0], cal.nm_grid[-1], cal.s_grid[-1], cal.s_grid[0]])
        ax.set_xlabel("wavelength (nm)")
        ax.set_ylabel("slit row")
        ax.set_title(f"{P['rectified_lamp']['name']} after rectification: lines should be straight and upright")
        figs.append(_save(fig, out, "rectified.png", "rectified lamp frame", plt))

    if "laser" in P:
        lz = P["laser"]
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.plot(lz["y"], lz["nm"], ".", ms=3)
        ax.set_xlabel("row")
        ax.set_ylabel("laser wavelength (nm)")
        ax.set_title("Laser along the slit after calibration (should be flat)")
        figs.append(_save(fig, out, "laser.png", "laser along the slit", plt))
    return figs


def _save(fig, out, name, title, plt):
    fig.tight_layout()
    fig.savefig(out / name, dpi=110)
    plt.close(fig)
    return name, title
