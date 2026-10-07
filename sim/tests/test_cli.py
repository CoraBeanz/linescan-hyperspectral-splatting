"""The command line."""

import json

import pytest

from hsisim.__main__ import main
from hsisim.truth import read_lines_csv


def test_scan_command(tmp_path):
    out = tmp_path / "scan"
    assert main(["scan", str(out), "--views", "down", "--lines", "2", "--steps-per-line", "16",
                 "--scene", "board", "--lighting", "lamp", "--errors", "0", "--no-calibration"]) == 0
    assert len(read_lines_csv(out / "lines.csv")["index"]) == 2
    sim = json.loads((out / "scan.json").read_text())["simulated"]
    assert sim["scene"] == "board" and sim["lighting"]["type"] == "lamp"
    assert not (out / "calibration").exists()
    # perfect poses: the log is the truth
    assert (out / "lines.csv").read_text() == (out / "truth" / "lines_true.csv").read_text()


def test_bad_plan_is_an_error(tmp_path, capsys):
    assert main(["scan", str(tmp_path / "x"), "--views", "nowhere", "--no-calibration"]) == 1
    assert "nowhere" in capsys.readouterr().err


def test_preview_command(scan, tmp_path):
    pytest.importorskip("matplotlib")
    base, _ = scan
    assert main(["preview", str(base), "-o", str(tmp_path)]) == 0
    names = sorted(p.name for p in tmp_path.glob("*.png"))
    assert names == ["frame.png", "scene.png", "sweep_001.png", "sweep_002.png"]
