"""Built-in scenes, each laid out around a scan target on the table.

relief  the default: a 3-D relief target, a 5 x 5 block of square pillars of different
        heights on a base plate, their tops in different materials and textures. The
        splat's pose refinement stalls on a flat board (a sweep turned slightly about the
        board and shifted to keep it in place looks nearly the same); detail at many
        depths pins every sweep's pose down. It is a sensible thing to 3-D print: 60 x 60
        mm, the tallest pillar 12 mm, tops painted or covered with paper.
board   a flat target board like the splat's synthetic scene: a checker border, paint
        patches, a panel with a word hidden in the near infrared, a rare-earth tile, a
        PTFE patch, a leaf-green ball and an orange box.
white   a PTFE sheet: what the white reference frames look at.

`target` is the plan's scan target in base_link (the table top under it, metres).
"""

from __future__ import annotations

import numpy as np

from .scene import Bitmap, Box, Checker, Lamp, Plane, Scene, Speckle, Sphere, Texture, Uniform

MM = 1e-3
DEFAULT_TARGET = (0.26, 0.0, -0.0024)  # make_plan's default: 26 cm in front of the base, on the table

NIR = ("#...#.###.###.",  # the word NIR, 14 x 5 cells: '#' is carbon black, the rest the IR dye
       "##..#..#..#..#",
       "#.#.#..#..###.",
       "#..##..#..#.#.",
       "#...#.###.#..#")


def _lighting(lighting, target):
    if lighting in (None, "uniform"):
        return Uniform()
    if lighting == "lamp":  # the halogen work light, 35 cm up, off to the side of the arm
        tx, ty, tz = target
        return Lamp(position=(tx - 0.10, ty + 0.20, tz + 0.35), target=target)
    raise ValueError(f"unknown lighting '{lighting}' (uniform or lamp)")


def _bordered(material, size, border=2.0 * MM, square=2.0 * MM):
    """A face of `material` inside a checker border `border` wide."""
    half = 0.5 * size
    t = Texture().add(Checker(square, ("carbon_black", "white_paper"), origin=(-half, -half)))
    return t.add(Checker(1.0, (material, material)), (-half + border, -half + border, half - border, half - border))


# 5 x 5 pillar heights (mm) and tops, row by row from -y to +y, columns from -x to +x
RELIEF_HEIGHTS = ((3.0, 9.0, 5.0, 12.0, 2.0),
                  (7.0, 1.5, 10.0, 4.0, 8.0),
                  (11.0, 6.0, 2.5, 9.5, 3.5),
                  (4.5, 12.0, 7.5, 1.0, 10.5),
                  (2.0, 5.5, 11.5, 6.5, 4.0))


def _relief_tops():
    """Top texture of each pillar, in the same order as RELIEF_HEIGHTS."""
    s = 8.0 * MM
    solid = Texture.solid
    check = lambda a, b="white_paper", q=1.6 * MM: Texture().add(Checker(q, (a, b)))  # noqa: E731
    dots = lambda base, dot, seed: Texture().add(Speckle(1.2 * MM, 0.45 * MM, base, dot, 0.65, seed))  # noqa: E731
    nir = Texture().add(Checker(1.0, ("ir_black_dye", "ir_black_dye"))).add(
        Bitmap(NIR, 0.5 * MM, "carbon_black", "ir_black_dye"), (-0.45 * s, -0.2 * s, 0.45 * s, 0.2 * s))
    return (
        (solid("rare_earth"), check("red_paint"), dots("white_paper", "carbon_black", 1), solid("leaf"),
         check("blue_paint", "yellow_paint")),
        (check("carbon_black"), solid("ptfe"), dots("yellow_paint", "leaf", 2), check("green_paint"),
         solid("orange_plastic")),
        (dots("white_paper", "red_paint", 3), check("leaf", "white_paper", 1.2 * MM), nir,
         solid("gray18"), check("orange_plastic", "blue_paint")),
        (solid("green_paint"), check("gray18", "white_paper", 2.0 * MM), solid("rare_earth"),
         dots("leaf", "white_paper", 4), check("yellow_paint", "carbon_black")),
        (check("red_paint", "leaf"), solid("blue_paint"), dots("gray18", "white_paper", 5),
         check("ir_black_dye", "white_paper"), solid("yellow_paint")),
    )


def relief(target=DEFAULT_TARGET, lighting=None):
    tx, ty, tz = (float(v) for v in target)
    plate_h = 2.0 * MM
    solids = [Plane(tz, Texture.solid("wood"), name="table"),
              Box((tx, ty, tz + plate_h / 2), (60 * MM, 60 * MM, plate_h), _bordered("white_paper", 60 * MM),
                  sides=Texture.solid("gray18"), name="base plate")]
    pitch, side = 10.0 * MM, 8.0 * MM
    tops = _relief_tops()
    for r in range(5):
        for c in range(5):
            h = RELIEF_HEIGHTS[r][c] * MM
            x, y = tx + (c - 2) * pitch, ty + (r - 2) * pitch
            solids.append(Box((x, y, tz + plate_h + h / 2), (side, side, h), tops[r][c],
                              sides=Texture().add(Checker(2.0 * MM, ("white_paper", "gray18"))),
                              name=f"pillar r{r}c{c}"))
    return Scene(solids, _lighting(lighting, (tx, ty, tz)), name="relief", target=(tx, ty, tz),
                 description="5 x 5 pillars, 8 mm square on a 10 mm pitch, 1 to 12 mm tall, on a 60 x 60 x 2 mm "
                             "plate with a checker border, on a wooden table")


def board(target=DEFAULT_TARGET, lighting=None):
    tx, ty, tz = (float(v) for v in target)
    t = 2.0 * MM
    top = _bordered("white_paper", 60 * MM)
    patches = [("red_paint", -20, 12), ("orange_plastic", -10, 12), ("yellow_paint", 0, 12),
               ("green_paint", 10, 12), ("blue_paint", 20, 12), ("gray18", -20, 2), ("leaf", -10, 2),
               ("ptfe", 20, 2)]
    for m, x, y in patches:
        top.add(Checker(1.0, (m, m)), ((x - 4) * MM, (y - 4) * MM, (x + 4) * MM, (y + 4) * MM))
    top.add(Checker(1.0, ("rare_earth", "rare_earth")), (6 * MM, -2 * MM, 14 * MM, 6 * MM))
    top.add(Checker(1.0, ("ir_black_dye", "ir_black_dye")), (-22 * MM, -22 * MM, 22 * MM, -10 * MM))
    top.add(Bitmap(NIR, 2.0 * MM, "carbon_black", "ir_black_dye", centre=(0.0, -16 * MM)),
            (-22 * MM, -22 * MM, 22 * MM, -10 * MM))
    solids = [Plane(tz, Texture.solid("wood"), name="table"),
              Box((tx, ty, tz + t / 2), (60 * MM, 60 * MM, t), top, sides=Texture.solid("white_paper"), name="board"),
              Sphere((tx - 15 * MM, ty - 2 * MM, tz + t + 7 * MM), 7 * MM, Texture.solid("leaf"), name="ball"),
              Box((tx + 2 * MM, ty - 2 * MM, tz + t + 5 * MM), (12 * MM, 10 * MM, 10 * MM),
                  Texture.solid("orange_plastic"), name="orange box")]
    return Scene(solids, _lighting(lighting, (tx, ty, tz)), name="board", target=(tx, ty, tz),
                 description="a 60 x 60 mm board with a checker border, paint patches, a rare-earth tile, a PTFE "
                             "patch and a word hidden in the near infrared; a leaf-green ball and an orange box")


def white(target=DEFAULT_TARGET, lighting=None):
    tx, ty, tz = (float(v) for v in target)
    return Scene([Plane(tz, Texture.solid("ptfe"), name="PTFE sheet")], _lighting(lighting, (tx, ty, tz)),
                 name="white", target=(tx, ty, tz), description="a PTFE sheet filling the view")


SCENES = {"relief": relief, "board": board, "white": white}


def make(name, target=DEFAULT_TARGET, lighting=None):
    try:
        return SCENES[name](target, lighting)
    except KeyError:
        raise ValueError(f"unknown scene '{name}'; built in: {', '.join(SCENES)}") from None


def heights_mm():
    return np.array(RELIEF_HEIGHTS)
