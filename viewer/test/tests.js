// The viewer's checks, run in a browser by test/index.html. They compare the
// file reader, the color math, the CPU probe and the WebGL2 renderer with
// what the C++ code computes for the same scene: reference.json holds the
// C++ renderer's spectra at a grid of pixels of two cameras, and its color
// weights, written by
//
//   splat_export SCENE trained.lsplat ... --reference reference.json
//
// for the trained sample in data/.

import { decodeSplat, fetchSplat, gaussianFeatures } from '../js/format.js';
import { bandMeanWeights, colorInfraredWeights, dot, trueColorWeights, wavelengthWeights } from '../js/spectral.js';
import { OrbitCamera, focalForFov, lookAt, projectPoint, unproject } from '../js/camera.js';
import { NEAR, probe } from '../js/probe.js';
import { SplatRenderer } from '../js/renderer.js';

const tests = [];
const test = (name, fn) => tests.push({ name, fn });

function check(ok, message) {
  if (!ok) throw new Error(message);
}

function near(a, b, tol, what) {
  check(Math.abs(a - b) <= tol, `${what}: ${a} vs ${b} (allowed ${tol})`);
}

// The largest difference between two arrays, and where it is.
function maxDiff(a, b) {
  let worst = 0, at = -1;
  for (let i = 0; i < a.length; i++) {
    const d = Math.abs(a[i] - b[i]);
    if (d > worst) {
      worst = d;
      at = i;
    }
  }
  return { worst, at };
}

const fmt = (x) => x.toExponential(1);

let cache = null;
async function fixtures() {
  if (cache) return cache;
  const [reference, trained, truth] = await Promise.all([
    fetch('reference.json').then((r) => r.json()),
    fetchSplat('../data/trained-16-sweeps.lsplat'),
    fetchSplat('../data/ground-truth.lsplat.gz'),
  ]);
  return (cache = { reference, trained, truth });
}

// A reference camera as the viewer's renderer and probe take it.
function referenceCamera(c) {
  return { ...lookAt(c.view.eye, c.view.target, c.view.up), f: c.f_px, width: c.width, height: c.height };
}

// --- The file ----------------------------------------------------------------

test('reads both bundled samples', async () => {
  const { trained, truth, reference } = await fixtures();
  for (const [s, count, title] of [[trained, 12943, 'Trained from 16 sweeps'], [truth, 9741, 'Ground truth']]) {
    check(s.count === count, `${s.title}: ${s.count} Gaussians, expected ${count}`);
    check(s.title === title, `title "${s.title}", expected "${title}"`);
    check(s.numFeatures === 46 && s.numBands === 46, `${s.numFeatures} features, ${s.numBands} bands`);
    check(Array.from(s.wavelengths).every((nm, b) => nm === reference.wavelengths_nm[b]), 'wavelengths differ from the reference');
    check(s.sweeps.length === 16 && s.sweeps.every((w) => w.corners.length === 4 && w.lines > 0), 'expected 16 sweep fans');
    check(s.header.source.synthetic === true, 'the samples come from the synthetic dataset');
    for (let i = 0; i < s.count; i++) {
      const c = s.cov.subarray(6 * i, 6 * i + 6);
      check(c[0] > 0 && c[3] > 0 && c[5] > 0, `Gaussian ${i} has a covariance that isn't positive`);
      check(s.opacity[i] >= 0 && s.opacity[i] <= 1, `Gaussian ${i} has opacity ${s.opacity[i]}`);
    }
  }
  return `${trained.count} + ${truth.count} Gaussians`;
});

test('decodes features between each Gaussian\'s offset and offset + scale', async () => {
  const { trained } = await fixtures();
  const f = new Float64Array(trained.numFeatures);
  for (let i = 0; i < trained.count; i += 97) {
    gaussianFeatures(trained, i, f);
    const off = trained.featureRange[2 * i], scale = trained.featureRange[2 * i + 1];
    for (const v of f) check(v >= off - 1e-6 && v <= off + scale + 1e-6, `Gaussian ${i}: feature ${v} outside its range`);
  }
});

test('rejects files that are not splats', async () => {
  const bad = [new Uint8Array([1, 2, 3]), new TextEncoder().encode('LSPX\u0001\u0000\u0000\u0000\u0004\u0000\u0000\u0000{}  ')];
  for (const bytes of bad) {
    let threw = false;
    try {
      await decodeSplat(bytes.buffer);
    } catch {
      threw = true;
    }
    check(threw, 'a bad file was accepted');
  }
});

// --- Color -------------------------------------------------------------------

test('true color and color infrared weights match the C++ previews', async () => {
  const { reference, trained } = await fixtures();
  let worst = 0;
  for (const [name, js] of [['true_color_weights', trueColorWeights(trained.wavelengths)],
    ['color_infrared_weights', colorInfraredWeights(trained.wavelengths)]]) {
    reference[name].forEach((rgb, b) => {
      for (let c = 0; c < 3; c++) {
        const d = Math.abs(js[c][b] - rgb[c]);
        worst = Math.max(worst, d);
        check(d < 1e-6, `${name}, band ${b}, channel ${c}: ${js[c][b]} vs ${rgb[c]}`);
      }
    });
  }
  return `largest difference ${fmt(worst)}`;
});

test('white stays white and a wavelength interpolates between bands', async () => {
  const { trained } = await fixtures();
  const wl = trained.wavelengths;
  const ones = new Float64Array(wl.length).fill(1);
  for (const w of trueColorWeights(wl)) near(dot(w, ones), 1, 1e-9, 'true color of reflectance 1');
  for (const w of colorInfraredWeights(wl)) near(dot(w, ones), 1, 1e-9, 'CIR of reflectance 1');
  const w = wavelengthWeights(wl, 805);
  near(w[30], 0.5, 1e-12, '805 nm from 800 nm');
  near(w[31], 0.5, 1e-12, '805 nm from 810 nm');
  near(wavelengthWeights(wl, 400)[0], 1, 0, 'below the first band');
  near(bandMeanWeights(wl, 790, 810).reduce((a, b) => a + b, 0), 1, 1e-12, 'a band mean');
});

// --- Cameras -----------------------------------------------------------------

test('the viewing camera agrees with the C++ look_at and focal length', async () => {
  const { reference, trained } = await fixtures();
  for (const c of reference.cameras) {
    const cam = referenceCamera(c);
    const centre = projectPoint(cam, c.view.target);
    near(centre[0], c.width / 2, 1e-9, 'target column');
    near(centre[1], c.height / 2, 1e-9, 'target row');
    near(focalForFov(c.view.fov_deg, c.width, c.height), c.f_px, 1e-9, 'focal length');
    const back = unproject(cam, 17.25, 42.5, 0.123);
    const again = projectPoint(cam, back);
    near(again[0], 17.25, 1e-9, 'unproject then project, u');
    near(again[1], 42.5, 1e-9, 'unproject then project, v');
    near(again[2], 0.123, 1e-12, 'unproject then project, depth');
  }
  // The orbit camera starts where the file's view says.
  const orbit = new OrbitCamera(trained.view);
  const f = orbit.frame(480, 360);
  const ref = referenceCamera(reference.cameras[0]);
  for (let k = 0; k < 9; k++) near(f.R[k], ref.R[k], 1e-6, `rotation ${k}`);
  for (let k = 0; k < 3; k++) near(f.t[k], ref.t[k], 1e-6, `translation ${k}`);
  near(f.f, reference.cameras[0].f_px, 1e-9, 'orbit focal length');
});

// --- Rendering ---------------------------------------------------------------

test('the CPU probe matches the C++ renderer\'s spectra', async () => {
  const { reference, trained } = await fixtures();
  let worst = 0, worstCov = 0, n = 0;
  for (const c of reference.cameras) {
    const cam = referenceCamera(c);
    for (const p of c.pixels) {
      const r = probe(trained, cam, p.x, p.y);
      const { worst: d, at } = maxDiff(r.spectrum, p.spectrum);
      // The reference is rounded to 4 decimals.
      check(d < 5e-4, `pixel (${p.x}, ${p.y}) of the ${c.width} x ${c.height} camera, band ${at}: ` +
        `${r.spectrum[at].toFixed(5)} vs ${p.spectrum[at]}`);
      near(r.coverage, p.coverage, 5e-4, `coverage at (${p.x}, ${p.y})`);
      worst = Math.max(worst, d);
      worstCov = Math.max(worstCov, Math.abs(r.coverage - p.coverage));
      n++;
    }
  }
  return `${n} pixels, largest difference ${fmt(worst)} in a band, ${fmt(worstCov)} in coverage`;
});

test('a probe\'s point lands back on its pixel', async () => {
  const { reference, trained } = await fixtures();
  const c = reference.cameras[0];
  const cam = referenceCamera(c);
  for (const p of c.pixels) {
    const r = probe(trained, cam, p.x, p.y);
    if (r.coverage < 0.5) continue;
    const at = projectPoint(cam, r.point, NEAR);
    near(at[0], p.x + 0.5, 1e-6, 'column');
    near(at[1], p.y + 0.5, 1e-6, 'row');
  }
});

// Renders a reference camera on the GPU with `weights` as the channels and
// returns each reference pixel's three channels, background included, and
// its coverage.
function gpuPixels(renderer, scene, c, weights) {
  const canvas = renderer.canvas;
  canvas.width = c.width;
  canvas.height = c.height;
  renderer.setChannels(weights);
  renderer.render(referenceCamera(c), { mode: 0, gain: 1 });
  const img = renderer.readLinear();
  const bg = renderer.channels.background;
  return c.pixels.map((p) => {
    const o = 4 * (p.y * img.width + p.x);
    const a = img.data[o + 3];
    return { rgb: [0, 1, 2].map((k) => img.data[o + k] + (1 - a) * bg[k]), coverage: a };
  });
}

let gpu = null;
async function gpuRenderer() {
  if (gpu) return gpu;
  const { trained } = await fixtures();
  const canvas = document.createElement('canvas');
  canvas.className = 'gpu';
  document.body.appendChild(canvas);
  gpu = new SplatRenderer(canvas);
  gpu.setScene(trained);
  return gpu;
}

// Every `step`-th pixel of a camera's image, as [x, y].
function grid(c, step) {
  const out = [];
  for (let y = step >> 1; y < c.height; y += step)
    for (let x = step >> 1; x < c.width; x += step) out.push({ x, y });
  return out;
}

test('WebGL2 draws what the CPU probe computes, pixel by pixel', async () => {
  const { reference, trained } = await fixtures();
  const renderer = await gpuRenderer();
  const weights = trueColorWeights(trained.wavelengths);
  const diffs = [];
  let worst = { d: 0 };
  for (const c of reference.cameras) {
    const pixels = grid(c, 4);
    const got = gpuPixels(renderer, trained, { ...c, pixels }, weights);
    const cam = referenceCamera(c);
    pixels.forEach((p, j) => {
      // The GPU blends every Gaussian; so does the probe without its early stop.
      const r = probe(trained, cam, p.x, p.y, { stopEarly: false });
      const want = [...weights.map((w) => dot(w, r.spectrum)), r.coverage];
      const have = [...got[j].rgb, got[j].coverage];
      for (let k = 0; k < 4; k++) {
        const d = Math.abs(have[k] - want[k]);
        diffs.push(d);
        if (d > worst.d) worst = { d, p, k, c, have: have[k], want: want[k] };
      }
    });
  }
  diffs.sort((a, b) => a - b);
  const q = (f) => diffs[Math.min(diffs.length - 1, Math.floor(f * diffs.length))];
  const w = worst.d ? `, worst at (${worst.p.x}, ${worst.p.y}) of ${worst.c.width} x ${worst.c.height}, ` +
    `${['R', 'G', 'B', 'coverage'][worst.k]} ${worst.have.toFixed(4)} vs ${worst.want.toFixed(4)}` : '';
  // A Gaussian right at a cutoff (alpha 1/255, or 3 sigma along the row) can
  // land on either side of it in float32, so allow a few small outliers.
  check(q(0.999) < 1e-4, `0.1% of values differ by more than ${fmt(q(0.999))}`);
  check(worst.d < 5e-3, `the largest difference is ${fmt(worst.d)}${w}`);
  return `${diffs.length / 4} pixels: median ${fmt(q(0.5))}, 99.9% within ${fmt(q(0.999))}, largest ${fmt(worst.d)}`;
});

// The C++ renderer stops a pixel at the Gaussian that would leave less than
// 1e-4 of its light, and leaves that Gaussian out (as 3DGS does); the GPU
// draws back to front and can't, so where a pixel turns opaque it keeps up to
// 1% more of the last Gaussians.
test('WebGL2 agrees with the C++ renderer at its reference pixels', async () => {
  const { reference, trained } = await fixtures();
  const renderer = await gpuRenderer();
  const wl = trained.wavelengths;
  let worst = 0, worstCov = 0;
  for (const weights of [trueColorWeights(wl), [0, 15, 30].map((b) => wavelengthWeights(wl, wl[b])),
    [45, 25, 5].map((b) => wavelengthWeights(wl, wl[b]))]) {
    for (const c of reference.cameras) {
      const got = gpuPixels(renderer, trained, c, weights);
      c.pixels.forEach((p, j) => {
        for (let k = 0; k < 3; k++) {
          const want = dot(weights[k], p.spectrum);
          const d = Math.abs(got[j].rgb[k] - want);
          check(d < 1e-2, `pixel (${p.x}, ${p.y}) of the ${c.width} x ${c.height} camera, channel ${k}: ` +
            `${got[j].rgb[k].toFixed(4)} vs ${want.toFixed(4)}`);
          worst = Math.max(worst, d);
        }
        const dc = Math.abs(got[j].coverage - p.coverage);
        check(dc < 1e-2, `coverage at (${p.x}, ${p.y}): ${got[j].coverage.toFixed(4)} vs ${p.coverage}`);
        worstCov = Math.max(worstCov, dc);
      });
    }
  }
  return `${renderer.accFormat.name} target, largest difference ${fmt(worst)}, coverage ${fmt(worstCov)}`;
});

test('Gaussians are drawn back to front, file order breaking ties', async () => {
  const { reference, trained } = await fixtures();
  const renderer = await gpuRenderer();
  const cam = referenceCamera(reference.cameras[1]);
  renderer.sortedFor = null;
  renderer.sort(cam);
  const m = trained.means, R = cam.R;
  const depth = (i) => R[6] * m[3 * i] + R[7] * m[3 * i + 1] + R[8] * m[3 * i + 2] + cam.t[2];
  let expected = 0;
  for (let i = 0; i < trained.count; i++) if (depth(i) > NEAR) expected++;
  check(renderer.visible === expected, `${renderer.visible} Gaussians drawn, ${expected} in front of the camera`);
  const order = renderer.order;
  for (let j = 1; j < renderer.visible; j++) {
    const a = order[j - 1], b = order[j];
    const da = Math.fround(depth(a)), db = Math.fround(depth(b));
    check(da > db || (da === db && a > b), `drawn out of order at ${j}: ${a} (${da}) before ${b} (${db})`);
  }
  return `${renderer.visible} Gaussians`;
});

// --- Running -----------------------------------------------------------------

async function run() {
  const results = [];
  const list = document.getElementById('results');
  for (const t of tests) {
    const start = performance.now();
    const r = { name: t.name, ok: true, detail: '', ms: 0 };
    try {
      r.detail = (await t.fn()) || '';
    } catch (e) {
      r.ok = false;
      r.detail = e && e.message ? e.message : String(e);
    }
    r.ms = Math.round(performance.now() - start);
    results.push(r);
    const li = document.createElement('li');
    li.className = r.ok ? 'pass' : 'fail';
    li.textContent = `${r.ok ? 'PASS' : 'FAIL'}  ${r.name}${r.detail ? `: ${r.detail}` : ''} (${r.ms} ms)`;
    list.appendChild(li);
  }
  const failed = results.filter((r) => !r.ok).length;
  document.getElementById('summary').textContent =
    failed ? `${failed} of ${results.length} checks failed` : `All ${results.length} checks passed`;
  window.testResults = { done: true, failed, results };
}

run();
