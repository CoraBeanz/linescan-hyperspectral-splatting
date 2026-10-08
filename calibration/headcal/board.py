"""The calibration target: a ChArUco board (a checkerboard with an ArUco tag in each white square).

The pose camera finds the board with OpenCV's ChArUco detector: the tags say which corner is
which, so a board that is only partly in view still gives named corners. The line camera can't
read tags in its scan lines, so headcal matches the board's known pattern there instead
(linecam.py); the tags make that pattern unique, so the match can't slip by a square.

Board frame (OpenCV's): origin at the board's outer top-left corner, x along the rows of
squares, y down the columns, z = x cross y into the paper. Inner corner id k sits at
((k % (nx - 1) + 1) * square, (k // (nx - 1) + 1) * square, 0).

The default board, 24 x 17 squares of 10 mm with 7 mm DICT_4X4_250 tags, fits on Letter or A4
paper. 10 mm squares are a compromise between the cameras: the line camera's 42 mm scan line
sees about four squares across, and the pose camera, 15 to 25 cm away, still gets 6 or more
pixels per tag cell.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass

import numpy as np

FORMAT = "headcal board v1"

PAPER = {"letter": (279.4, 215.9), "a4": (297.0, 210.0)}

# reflectances used when the board is rendered (synthetic data and the line camera's model)
WHITE = 0.85     # office paper
BLACK = 0.05     # laser toner (carbon black: dark in the near infrared too)
TABLE = 0.35     # what lies around the paper


@dataclass
class Board:
    squares_x: int = 24
    squares_y: int = 17
    square_mm: float = 10.0
    marker_mm: float = 7.0
    dictionary: str = "DICT_4X4_250"

    # --- OpenCV --------------------------------------------------------------------------

    def cv_dictionary(self):
        import cv2
        return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.dictionary))

    def cv_board(self):
        import cv2
        b = cv2.aruco.CharucoBoard((self.squares_x, self.squares_y), self.square_mm * 1e-3, self.marker_mm * 1e-3,
                                   self.cv_dictionary())
        b.setLegacyPattern(False)
        return b

    # --- geometry --------------------------------------------------------------------------

    @property
    def size_m(self):
        return self.squares_x * self.square_mm * 1e-3, self.squares_y * self.square_mm * 1e-3

    @property
    def n_corners(self):
        return (self.squares_x - 1) * (self.squares_y - 1)

    def corners(self, ids=None):
        """Inner corners (N, 3) in the board frame, metres."""
        k = np.arange(self.n_corners) if ids is None else np.asarray(ids, int)
        nx = self.squares_x - 1
        s = self.square_mm * 1e-3
        return np.stack([(k % nx + 1) * s, (k // nx + 1) * s, np.zeros(len(k))], axis=-1)

    @property
    def centre(self):
        w, h = self.size_m
        return np.array([w / 2, h / 2, 0.0])

    # --- files -------------------------------------------------------------------------------

    def to_json(self):
        return dict(format=FORMAT, **asdict(self))

    def save(self, path):
        with open(path, "w") as f:
            json.dump(self.to_json(), f, indent=2)
            f.write("\n")

    @classmethod
    def load(cls, path):
        with open(path) as f:
            d = json.load(f)
        if d.get("format") != FORMAT:
            raise ValueError("%s isn't a headcal board file" % path)
        return cls(**{k: d[k] for k in ("squares_x", "squares_y", "square_mm", "marker_mm", "dictionary")})

    def scaled(self, span_mm, squares=20):
        """The board as printed, from a measured span of `squares` squares."""
        f = span_mm / (squares * self.square_mm)
        return Board(self.squares_x, self.squares_y, self.square_mm * f, self.marker_mm * f, self.dictionary)

    # --- pictures ------------------------------------------------------------------------------

    def black_rects(self):
        """Every black area as (x, y, w, h) in mm, board frame: the black squares, then the tags'
        black cells with each row's runs merged. Taken from OpenCV's own drawing, so it is the
        board the detector expects."""
        import cv2
        sq = self.square_mm
        img = self.cv_board().generateImage((self.squares_x * 40, self.squares_y * 40), marginSize=0, borderBits=1)
        rects = []
        for r in range(self.squares_y):
            for c in range(self.squares_x):
                if img[r * 40 + 2, c * 40 + 2] < 128:   # just inside the square's corner: no tag there
                    rects.append((c * sq, r * sq, sq, sq))
        d = self.cv_dictionary()
        bits = d.markerSize + 2
        cell = self.marker_mm / bits
        for i, quad in enumerate(self.cv_board().getObjPoints()):
            x0, y0 = float(quad[0][0]) * 1e3, float(quad[0][1]) * 1e3
            m = cv2.aruco.generateImageMarker(d, int(self.cv_board().getIds()[i]), bits * 10, borderBits=1)
            grid = m[5::10, 5::10] < 128
            for r in range(bits):
                c = 0
                while c < bits:
                    if grid[r, c]:
                        c1 = c
                        while c1 < bits and grid[r, c1]:
                            c1 += 1
                        rects.append((x0 + c * cell, y0 + r * cell, (c1 - c) * cell, cell))
                        c = c1
                    else:
                        c += 1
        return rects

    def raster(self, px_per_mm):
        """The board drawn from black_rects at px_per_mm, 1 = white, 0 = black (anti-aliased by
        area). Used by the tests to check black_rects against OpenCV's drawing."""
        w = int(round(self.squares_x * self.square_mm * px_per_mm))
        h = int(round(self.squares_y * self.square_mm * px_per_mm))
        img = np.ones((h, w))
        for x, y, rw, rh in self.black_rects():
            x0, x1 = x * px_per_mm, (x + rw) * px_per_mm
            y0, y1 = y * px_per_mm, (y + rh) * px_per_mm
            c = np.arange(max(int(x0), 0), min(int(np.ceil(x1)), w))
            r = np.arange(max(int(y0), 0), min(int(np.ceil(y1)), h))
            cols = np.clip(np.minimum(c + 1, x1) - np.maximum(c, x0), 0, 1)
            rows = np.clip(np.minimum(r + 1, y1) - np.maximum(r, y0), 0, 1)
            img[r[0]:r[-1] + 1, c[0]:c[-1] + 1] -= np.outer(rows, cols)
        return np.clip(img, 0, 1)

    def texture(self, px_per_mm=10, margin_mm=15.0):
        """The board as seen on the table, for rendering: float32 reflectance image with a white
        paper margin. Returns (image, to_pixels) where to_pixels(x, y) maps board-frame metres
        to image coordinates (pixel centres at integers, as cv2.remap wants)."""
        img = self.raster(px_per_mm)
        m = int(round(margin_mm * px_per_mm))
        out = np.pad(BLACK + (WHITE - BLACK) * img, m, constant_values=WHITE).astype(np.float32)
        scale = px_per_mm * 1e3

        def to_pixels(x, y):
            return x * scale + m - 0.5, y * scale + m - 0.5

        return out, to_pixels

    def svg(self, paper="letter"):
        """A printable SVG at the board's real size on landscape paper, with a 100 mm check bar."""
        pw, ph = PAPER[paper]
        bw, bh = self.squares_x * self.square_mm, self.squares_y * self.square_mm
        if bw > pw - 10 or bh > ph - 24:
            raise ValueError("a %.0f x %.0f mm board doesn't fit on %s paper" % (bw, bh, paper))
        ox, oy = (pw - bw) / 2, (ph - bh) / 2 + 2
        out = ['<?xml version="1.0" encoding="UTF-8"?>',
               '<svg xmlns="http://www.w3.org/2000/svg" width="%gmm" height="%gmm" viewBox="0 0 %g %g">'
               % (pw, ph, pw, ph),
               '<rect width="%g" height="%g" fill="#fff"/>' % (pw, ph),
               '<g transform="translate(%.4f %.4f)" fill="#000" shape-rendering="crispEdges">' % (ox, oy)]
        out += ['<rect x="%.4f" y="%.4f" width="%.4f" height="%.4f"/>' % r for r in self.black_rects()]
        out.append("</g>")
        # centre ticks on the paper's edges, to line the board up with a mark on the table
        for x in (ox + bw / 2,):
            out.append('<path d="M%.3f %.3f v-4 M%.3f %.3f v4" stroke="#000" stroke-width="0.3"/>'
                       % (x, oy - 1, x, oy + bh + 1))
        y = oy + bh / 2
        out.append('<path d="M%.3f %.3f h-4 M%.3f %.3f h4" stroke="#000" stroke-width="0.3"/>'
                   % (ox - 1, y, ox + bw + 1, y))
        # 100 mm check bar and the label
        bx, by = ox, oy - 7
        out.append('<path d="M%.3f %.3f h100" stroke="#000" stroke-width="0.4"/>' % (bx, by))
        for i in range(0, 101, 10):
            out.append('<path d="M%.3f %.3f v%g" stroke="#000" stroke-width="0.3"/>'
                       % (bx + i, by, -3 if i % 50 == 0 else -1.5))
        font = 'font-family="Helvetica, Arial, sans-serif" font-size="3"'
        out.append('<text x="%.3f" y="%.3f" %s>100 mm: print at 100%% (actual size) and check this bar</text>'
                   % (bx + 103, by + 1, font))
        out.append('<text x="%.3f" y="%.3f" %s>headcal ChArUco board: %d x %d squares of %g mm, %g mm %s tags. '
                   'Laser printer (toner stays black in near infrared); glue flat on card.</text>'
                   % (ox, oy + bh + 6, font, self.squares_x, self.squares_y, self.square_mm, self.marker_mm,
                      self.dictionary))
        out.append("</svg>")
        return "\n".join(out) + "\n"
