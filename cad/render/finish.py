"""Turn raw Cycles renders into the images in cad/renders.

    python3 cad/render/finish.py <render_dir> cad/renders

A 3x3 median takes out the speckle Cycles leaves without a denoiser, every
image is scaled to 1500 px wide, and head_open gets part labels from the
head_open_labels.json that render_blender.py writes next to it. Needs Pillow.
"""

import json
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFilter, ImageFont

WIDTH = 1500
FONTS = ["/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "DejaVuSans.ttf", "Arial.ttf"]


def font(size):
    for f in FONTS:
        try:
            return ImageFont.truetype(f, size)
        except OSError:
            pass
    return ImageFont.load_default()


def label(im, anchors):
    """Labels in a row above and a row below the part, leader lines to the anchors.

    anchors: {text: [x, y]} with x, y as fractions of the width and height."""
    W, H = im.size
    f = font(round(W / 54))
    pad, gap, margin = round(W / 150), round(W / 120), round(W / 75)
    over = Image.new("RGBA", im.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(over)
    items = sorted(anchors.items(), key=lambda kv: kv[1][0])
    rows = {0: [], 1: []}
    for i, (text, (x, y)) in enumerate(items):
        left, top, right, bottom = d.textbbox((0, 0), text, font=f)
        rows[i % 2].append({"text": text, "ax": x * W, "ay": y * H, "w": right - left + 2 * pad,
                            "h": bottom - top + 2 * pad, "dy": top})
    for row, y_row in ((rows[0], 0.12 * H), (rows[1], 0.88 * H)):
        # centre each box over its anchor, then push right / left so none overlap
        for it in row:
            it["x0"] = min(max(it["ax"] - it["w"] / 2, margin), W - margin - it["w"])
        for a, b in zip(row, row[1:]):
            b["x0"] = max(b["x0"], a["x0"] + a["w"] + gap)
        for a, b in zip(reversed(row[:-1]), reversed(row[1:])):
            a["x0"] = min(a["x0"], b["x0"] - gap - a["w"])
        for it in row:
            x0, y0 = it["x0"], y_row - it["h"] / 2
            x1, y1 = x0 + it["w"], y0 + it["h"]
            edge = y1 if y_row < H / 2 else y0
            end = (min(max(it["ax"], x0 + pad), x1 - pad), edge)
            d.line([(it["ax"], it["ay"]), end], fill=(20, 20, 24, 255), width=5)
            d.line([(it["ax"], it["ay"]), end], fill=(255, 255, 255, 255), width=2)
            r = 5
            d.ellipse([it["ax"] - r, it["ay"] - r, it["ax"] + r, it["ay"] + r],
                      fill=(255, 255, 255, 255), outline=(20, 20, 24, 255), width=2)
            d.rounded_rectangle([x0, y0, x1, y1], radius=pad, fill=(24, 24, 28, 220))
            d.text((x0 + pad, y0 + pad - it["dy"]), it["text"], font=f, fill=(255, 255, 255, 255))
    return Image.alpha_composite(im.convert("RGBA"), over).convert("RGB")


def main():
    src, dst = Path(sys.argv[1]), Path(sys.argv[2])
    dst.mkdir(parents=True, exist_ok=True)
    for p in sorted(src.glob("*.png")):
        im = Image.open(p).convert("RGB").filter(ImageFilter.MedianFilter(3))
        im = im.resize((WIDTH, round(im.height * WIDTH / im.width)), Image.LANCZOS)
        labels = p.with_name(p.stem + "_labels.json")
        if labels.exists():
            im = label(im, json.loads(labels.read_text()))
        im.save(dst / p.name, optimize=True)
        print("wrote", dst / p.name)


if __name__ == "__main__":
    main()
