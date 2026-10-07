"""A scan from the instrument to the trainer's dataset, on a simulated spectrograph.

The arm looks at a patterned table from two viewpoints and the mirror sweeps a few lines at
each, logged as scan_sweep logs them (lines.csv, poses from the URDF). Every line's frame is
rendered by a made-up spectrograph (synthetic.Instrument) from where that line's pixels see the
table, then binned and written as line_camera writes it (SessionWriter), with darks and a
white. scan_to_dataset makes the dataset, and the trainer's own camera model (linesplat: the
head model, sweep poses, mirror angles and intrinsics from dataset.json) has to find the
table's reflectance in every pixel of every line.
"""

import csv
import json
import math

import numpy as np
import pytest

from so101_scan_camera import linesplat as ls
from so101_scan_camera import scan_to_dataset as s2d
from so101_scan_camera.binning import LineBinner, load_calibration
from so101_scan_camera.matcher import FrameInfo, Line
from so101_scan_camera.session import SessionWriter
from so101_scan_camera.synthetic import Instrument, halogen
from so101_scan_description.kinematics import Robot
from so101_scan_sweep.line_log import LinePoser, LinesCsv
from so101_scan_sweep.plan import ARM_JOINTS

VIEWS = {"down": [0.0, -8.3, 60.5, -52.2, -87.2], "side": [14.0, -4.1, 56.9, -46.3, -111.4]}
N_LINES = 24
RAD_PER_STEP = 2 * math.pi / 6400
EXPOSURE_US, GAIN = 5000.0, 1.0
WHITE_EXPOSURE_US, WHITE_GAIN = 3000.0, 1.5
SLIT_BINS = 64
NO_FRAME = (0, 7)                 # a line no frame came for
PERIOD_NS = 33_333_333


def reflectance(x, y, nm):
    """The table's pattern (base_link x, y in m): coloured stripes, 4 to 96 %."""
    u, v = (x - 0.26) / 0.012, y / 0.017
    spatial = 0.5 + 0.25 * np.sin(2 * np.pi * u + 1.3) * np.cos(2 * np.pi * v) + 0.12 * np.sin(np.pi * (u + v))
    colour = 0.75 + 0.25 * np.cos((np.asarray(nm) - 500.0) / 110.0 + 2.0 * v)
    return np.clip(spatial * colour, 0.04, 0.96)


def table_hits(cam, slope):
    """Where the rays of a line camera (4x4 in base_link) with slopes x/z meet the table top."""
    d = np.stack([slope, np.zeros_like(slope), np.ones_like(slope)], axis=-1) @ cam[:3, :3].T
    s = (s2d.TABLE_Z - cam[2, 3]) / d[..., 2]
    return cam[:3, 3] + s[..., None] * d


@pytest.fixture(scope="module")
def rig(urdf, tmp_path_factory):
    """The instrument, its calibration, and every frame of a scan of the table, rendered once."""
    inst = Instrument(seed=3)
    cal = inst.save_calibration(str(tmp_path_factory.mktemp("cal")))
    maps = load_calibration(cal)
    poser = LinePoser(Robot(urdf))
    k = poser.half_line / poser.scene_distance

    def frames(slit_reversed):
        """{(sweep, line): (head, camera, angle, joints, raw frame)}."""
        out = {}
        for sid, q in enumerate(VIEWS.values()):
            joints = dict(zip(ARM_JOINTS, np.radians(q)))
            for i in range(N_LINES):
                angle = (-12 + 2 * i) * RAD_PER_STEP
                head, cam = poser.poses(joints, angle)

                def radiance(t, nm, cam=cam):
                    # pixel order runs along the line camera's x unless the slit reads backwards
                    p = table_hits(cam, ((1.0 - 2.0 * t) if slit_reversed else (2.0 * t - 1.0)) * k)
                    return reflectance(p[:, 0], p[:, 1], nm) * halogen(nm)

                out[(sid, i)] = (head, cam, angle, joints, inst.render(radiance, EXPOSURE_US, GAIN, noise=False))
        return out

    refs = {
        "white": ("white", WHITE_EXPOSURE_US, WHITE_GAIN,
                  [inst.render(lambda t, nm: 0.98 * halogen(nm), WHITE_EXPOSURE_US, WHITE_GAIN, noise=False)] * 4),
        "dark": ("dark", EXPOSURE_US, GAIN, [inst.render(0.0, EXPOSURE_US, GAIN, noise=False)] * 4),
        "dark_white": ("dark", WHITE_EXPOSURE_US, WHITE_GAIN,
                       [inst.render(0.0, WHITE_EXPOSURE_US, WHITE_GAIN, noise=False)] * 4),
    }
    return dict(cal=cal, maps=maps, poser=poser, frames=frames, refs=refs, rendered={})


def write_scan(folder, urdf, rig, slit_reversed=False, binned=True):
    """A scan folder as scan_sweep and line_camera write it."""
    maps, poser = rig["maps"], rig["poser"]
    if slit_reversed not in rig["rendered"]:
        rig["rendered"][slit_reversed] = rig["frames"](slit_reversed)
    frames = rig["rendered"][slit_reversed]
    binner = LineBinner(maps, SLIT_BINS) if binned else None
    folder.mkdir()
    (folder / "robot.urdf").write_text(urdf)
    camera = dict(source="fake", exposure_us=EXPOSURE_US, gain=GAIN, slit_reversed=slit_reversed,
                  calibration=dict(path=maps.path, maps_sha256=maps.sha256))
    writer = SessionWriter(str(folder), camera, binner, raw_every=5 if binned else 1).start()
    lines_csv = LinesCsv(str(folder / "lines.csv"))
    t = 1_791_000_000_000_000_000
    for sid in range(len(VIEWS)):
        sweep = np.full((N_LINES,) + (binner.shape if binner else (1, 1)), np.nan, np.float32)
        for i in range(N_LINES):
            head, cam, angle, joints, raw = frames[(sid, i)]
            lines_csv.write(sid, sid, i, t, t + PERIOD_NS - 50_000, True, angle, head, cam, joints)
            line = Line(sid, i, t, t + PERIOD_NS - 50_000, True, i == N_LINES - 1, angle)
            saturated = 0
            if (sid, i) != NO_FRAME:
                line.frames.append(FrameInfo(100 * sid + i, t + 4_000_000, t + 1_000_000, t + 6_000_000,
                                             EXPOSURE_US, GAIN))
                if binner:
                    sweep[i], saturated = binner.bin(raw)
            keep = (sid, i) != NO_FRAME and writer.keep_raw(i)
            writer.write_line(line, saturated, raw if keep else None, binned=binner is not None)
            t += PERIOD_NS
        if binner:
            writer.write_sweep(sid, sweep)
    for name, (kind, exposure, gain, ref_frames) in rig["refs"].items():
        mean = np.mean([binner.bin(f)[0] for f in ref_frames], axis=0) if binner else None
        writer.write_reference(name, ref_frames, dict(kind=kind, exposure_us=exposure, gain=gain), mean)
    lines_csv.close()
    writer.close()
    info = {"format": "so101_scan lines v1", "name": "replay", "lines": lines_csv.rows,
            "frames": {"base": poser.base, "head": poser.head, "line_camera": poser.camera,
                       "scene_distance_m": poser.scene_distance, "scan_line_half_length_m": poser.half_line}}
    (folder / "scan.json").write_text(json.dumps(info))
    return folder


def load_dataset(out):
    doc = json.loads((out / "dataset.json").read_text())
    arrays = {k: np.load(out / v) for k, v in doc["files"].items()}
    return doc, arrays


def seen_reflectance(doc, arrays):
    """The table's reflectance in each pixel of each line, from the dataset's camera model alone:
    the head model, each line's sweep pose and mirror angle, and the intrinsics."""
    h = doc["head"]
    head = ls.HeadModel(ls.pose_matrix(h["camera_in_head"]), np.array(h["mirror"]["axis_point"]),
                        np.array(h["mirror"]["axis_dir"]), np.array(h["mirror"]["normal_at_zero"]),
                        h["mirror"]["face_offset"])
    nm = np.array(doc["wavelengths_nm"])
    out = []
    for sweep, angle in zip(arrays["line_sweep"], arrays["line_mirror_angle"]):
        cam = ls.pose_matrix(arrays["sweep_head_pose"][sweep]) @ ls.virtual_camera_in_head(head, angle)
        o, d = ls.pixel_rays(doc["intrinsics"], cam)
        p = o + (-o[2] / d[:, 2])[:, None] * d        # the table top is the dataset's z = 0
        out.append(reflectance(p[:, 0, None], p[:, 1, None], nm[None, :]))
    return np.array(out)


@pytest.mark.parametrize("slit_reversed", [False, True])
def test_replay_gives_the_table_through_the_trainers_camera(urdf, rig, tmp_path, slit_reversed):
    scan = write_scan(tmp_path / "scan", urdf, rig, slit_reversed)
    s2d.convert(str(scan), str(tmp_path / "dataset"))
    doc, a = load_dataset(tmp_path / "dataset")

    n = 2 * N_LINES - 1
    assert doc["format"] == "linesplat-dataset" and doc["version"] == 1 and doc["values"] == "reflectance"
    assert a["lines"].shape == (n, SLIT_BINS, 46) and a["lines"].dtype == np.float32
    assert (doc["num_lines"], doc["num_sweeps"], doc["width"], doc["num_bands"]) == (n, 2, SLIT_BINS, 46)
    assert doc["wavelengths_nm"] == pytest.approx(np.arange(500.0, 951.0, 10.0))
    assert a["line_sweep"].dtype == np.int32 and list(a["line_sweep"]) == [0] * (N_LINES - 1) + [1] * N_LINES
    with open(scan / "lines.csv") as f:
        rows = [r for r in csv.DictReader(f) if (int(r["sweep_id"]), int(r["index"])) != NO_FRAME]
    np.testing.assert_allclose(a["line_mirror_angle"], [float(r["mirror_angle"]) for r in rows], atol=1e-9)
    assert a["sweep_head_pose"].shape == (2, 7)
    meta = doc["metadata"]
    assert meta["lines_left_out"]["no_frame"] == 1 and meta["darks"] == {
        "5000 us, gain 1": "dark", "3000 us, gain 1.5": "dark_white"}
    assert meta["white"]["name"] == "white" and meta["slit_reversed"] == slit_reversed
    # the head in the dataset is the CAD head the synthetic datasets use
    cad = ls.cad_head_model().to_json()
    np.testing.assert_allclose(doc["head"]["camera_in_head"], cad["camera_in_head"], atol=1e-5)
    np.testing.assert_allclose(doc["head"]["mirror"]["axis_point"], cad["mirror"]["axis_point"], atol=1e-5)

    truth = seen_reflectance(doc, a)
    err = np.abs(a["lines"] - truth)
    assert np.isfinite(a["lines"]).all()
    # what is left is rounding to whole counts (the blue end gets a few counts a pixel) and the
    # pattern changing inside a pixel and a band
    assert np.median(err) < 0.004 and np.percentile(err, 99) < 0.03, (np.median(err), np.percentile(err, 99))
    # and the check can tell: the slit read the wrong way round is far off
    assert np.median(np.abs(a["lines"][:, ::-1] - truth)) > 0.05


def test_radiance_and_raw_only_scans(urdf, rig, tmp_path):
    """Without a white the values are radiance (relative to the calibration's lamp), and a scan
    recorded without a calibration (raw frames only) converts to the same dataset once binned."""
    scan = write_scan(tmp_path / "scan", urdf, rig)
    s2d.convert(str(scan), str(tmp_path / "reflectance"))
    s2d.convert(str(scan), str(tmp_path / "radiance"), values="radiance")
    doc, a = load_dataset(tmp_path / "radiance")
    assert doc["values"] == "radiance"
    nm = np.array(doc["wavelengths_nm"])
    rel = np.abs(a["lines"] / (seen_reflectance(doc, a) * halogen(nm)) - 1)
    assert np.median(rel) < 0.015 and np.percentile(rel, 99) < 0.1, (np.median(rel), np.percentile(rel, 99))

    raw = write_scan(tmp_path / "raw", urdf, rig, binned=False)
    with pytest.raises(s2d.ConvertError, match="calibration"):
        s2d.convert(str(raw), str(tmp_path / "nope"), calibration=str(tmp_path / "missing"))
    s2d.convert(str(raw), str(tmp_path / "from_raw"), slit_bins=SLIT_BINS)
    _, b = load_dataset(tmp_path / "from_raw")
    _, c = load_dataset(tmp_path / "reflectance")
    for k in c:
        np.testing.assert_allclose(b[k], c[k], rtol=1e-5, atol=1e-6, err_msg=k)


def test_lines_without_values_are_left_out(urdf, rig, tmp_path):
    scan = write_scan(tmp_path / "scan", urdf, rig)
    rows = (scan / "frames" / "frames.csv").read_text().splitlines()
    assert rows[1 + N_LINES + 3].startswith("1,3,ok,")
    rows[1 + N_LINES + 3] = rows[1 + N_LINES + 3].replace(",ok,", ",unsettled,")
    (scan / "frames" / "frames.csv").write_text("\n".join(rows) + "\n")
    binned = np.load(scan / "binned" / "sweep_001.npy")
    binned[5, :, : binned.shape[2] // 2] = np.nan       # half the spectrum saturated
    binned[6, :3, :] = np.nan                           # three pixels' spectra gone
    np.save(scan / "binned" / "sweep_001.npy", binned)
    doc = s2d.convert(str(scan), str(tmp_path / "dataset"), sweeps=["1"])
    left = doc["metadata"]["lines_left_out"]
    assert left == {"no_frame": 1, "missing_values": 1, "not_asked": N_LINES}
    assert doc["num_lines"] == N_LINES - 2 and doc["num_sweeps"] == 1
    assert doc["metadata"]["values_filled"] > 0
    _, a = load_dataset(tmp_path / "dataset")
    assert np.isfinite(a["lines"]).all()
    with pytest.raises(s2d.ConvertError, match="isn't empty"):
        (tmp_path / "busy").mkdir()
        (tmp_path / "busy" / "notes.txt").write_text("mine")
        s2d.convert(str(scan), str(tmp_path / "busy"))
