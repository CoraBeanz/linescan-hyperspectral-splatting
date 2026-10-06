"""Read the chosen optical layout straight out of optics/spectrograph_model.py.

The CAD positions every optical station (objective, slit, collimator, grating,
camera) from the same numbers the Optiland model traces, so the two can't
drift apart. This parses the model file with `ast` instead of importing it,
because importing would need Optiland installed inside FreeCAD's Python.
"""

import ast
from pathlib import Path

MODEL = Path(__file__).resolve().parents[2] / "optics" / "spectrograph_model.py"


def read_layout(tag="C", path=MODEL):
    """Return the Config fields of layout `tag`, plus the module constants."""
    tree = ast.parse(Path(path).read_text())
    consts, defaults, configs = {}, {}, {}
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == "Config":
            for st in node.body:
                if isinstance(st, ast.AnnAssign) and st.value is not None:
                    defaults[st.target.id] = ast.literal_eval(st.value)
        elif isinstance(node, ast.Assign) and len(node.targets) == 1:
            name = getattr(node.targets[0], "id", None)
            if name == "CONFIGS":
                for call in node.value.elts:
                    label = ast.literal_eval(call.args[0])
                    configs[label.split(":")[0].strip()] = {
                        "name": label,
                        **{k.arg: ast.literal_eval(k.value) for k in call.keywords},
                    }
            elif name:
                try:
                    consts[name] = ast.literal_eval(node.value)
                except ValueError:
                    pass
    if tag not in configs:
        raise KeyError("layout %r not found in %s (have %s)" % (tag, path, sorted(configs)))
    cfg = dict(defaults)
    cfg.update(configs[tag])
    cfg["center_wl_um"] = consts["CENTER_WL"]
    cfg["pixel_mm"] = consts["PIXEL_MM"]
    cfg["sensor_w"], cfg["sensor_h"] = consts["SENSOR_MM"]
    return cfg


if __name__ == "__main__":
    for k, v in read_layout().items():
        print("%-16s %s" % (k, v))
