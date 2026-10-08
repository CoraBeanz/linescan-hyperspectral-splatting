// The viewer's checks, run in a browser by test/index.html. They compare the
// file reader, the color math, the CPU probe and the WebGL2 renderer with
// what the C++ code computes for the same scene: reference.json holds the
// C++ renderer's spectra at a grid of pixels of two cameras, and its color
// weights, written by
//
//   splat_export SCENE trained.lsplat ... --reference reference.json
//
// for the trained sample in data/. Both samples also carry material maps from
// splat_materials, checked against the probe and the GPU below.

import { decodeSplat, fetchSplat, gaussianFeatures } from '../js/format.js';
import { bandMeanWeights, colorInfraredWeights, dot, trueColorWeights, wavelengthWeights } from '../js/spectral.js';
import { OrbitCamera, focalForFov, lookAt, projectPoint, unproject } from '../js/camera.js';
import { NEAR, probe } from '../js/probe.js';
import { SplatRenderer, UNKNOWN } from '../js/renderer.js';

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

test('reads base64 text the same as the binary file', async () => {
  const { trained } = await fixtures();
  for (const file of ['../data/trained-16-sweeps.lsplat', '../data/ground-truth.lsplat.gz']) {
    const bytes = new Uint8Array(await (await fetch(file)).arrayBuffer());
    let text = '';
    for (let i = 0; i < bytes.length; i += 0x8000) text += String.fromCharCode(...bytes.subarray(i, i + 0x8000));
    const scene = await decodeSplat(new TextEncoder().encode(btoa(text) + '\n').buffer);
    const direct = await decodeSplat(bytes.buffer);
    check(scene.count === direct.count, `${file}: ${scene.count} Gaussians from base64, ${direct.count} from binary`);
    check(scene.means.every((v, i) => v === direct.means[i]), `${file}: positions differ`);
    check(scene.features.every((v, i) => v === direct.features[i]), `${file}: features differ`);
  }
  return `${trained.count} Gaussians`;
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

// --- Material maps -------------------------------------------------------------

// The same file without its material maps: the header without "materials" and
// the four blocks, which splat_materials appends after the others.
function withoutMaterials(buffer) {
  const bytes = new Uint8Array(buffer);
  const length = new DataView(buffer).getUint32(8, true);
  const header = JSON.parse(new TextDecoder().decode(bytes.subarray(12, 12 + length)));
  delete header.materials;
  for (const name of ['material_label', 'material_angle', 'cluster', 'abundances']) delete header.blocks[name];
  const end = Math.max(...Object.values(header.blocks).map((b) => b.offset + b.bytes));
  let text = JSON.stringify(header);
  while (new TextEncoder().encode(text).length % 4) text += ' ';
  const h = new TextEncoder().encode(text);
  const out = new Uint8Array(12 + h.length + end);
  out.set(bytes.subarray(0, 12));
  new DataView(out.buffer).setUint32(8, h.length, true);
  out.set(h, 12);
  out.set(bytes.subarray(12 + length, 12 + length + end), 12 + h.length);
  return out.buffer;
}

test('reads the material maps of both samples', async () => {
  const { trained, truth } = await fixtures();
  const out = [];
  for (const s of [trained, truth]) {
    const m = s.materials;
    check(m, `${s.title} has no material maps`);
    check(m.classes.length === 11 && m.classes[0].name === 'white paper', `${m.classes.length} library materials`);
    check(m.classes.every((c) => c.spectrum.length === s.numBands), 'a library spectrum has the wrong length');
    const E = m.endmembers.length;
    const perClass = new Array(m.classes.length).fill(0), perCluster = new Array(m.clusters.length).fill(0);
    let unknown = 0;
    for (let i = 0; i < s.count; i++) {
      const l = m.label[i];
      if (l === UNKNOWN) unknown++;
      else {
        check(l < m.classes.length, `Gaussian ${i} has label ${l}`);
        perClass[l]++;
      }
      check(m.cluster[i] < m.clusters.length, `Gaussian ${i} is in cluster ${m.cluster[i]}`);
      perCluster[m.cluster[i]]++;
      // Each share is rounded to 1/255, so the sum is off by at most E/2 of those.
      let sum = 0;
      for (let e = 0; e < E; e++) sum += m.abundances[i * E + e];
      check(Math.abs(sum - 255) <= E / 2, `Gaussian ${i}'s abundances sum to ${sum}/255`);
    }
    m.classes.forEach((c, k) => check(c.count === perClass[k], `${c.name}: header says ${c.count}, labels count ${perClass[k]}`));
    m.clusters.forEach((c, k) => check(c.count === perCluster[k], `${c.name}: header says ${c.count}, labels count ${perCluster[k]}`));
    check(unknown === m.unknown, `${unknown} unknown, header says ${m.unknown}`);
    check(m.score && m.score.view.label_accuracy > 0, `${s.title} has no score against the truth`);
    out.push(`${s.title}: ${(100 * m.score.label_accuracy).toFixed(1)}% of Gaussians, ` +
      `${(100 * m.score.view.label_accuracy).toFixed(1)}% of map pixels`);
  }
  check(truth.materials.score.label_accuracy === 1, 'the ground truth should match its own library exactly');
  return out.join('; ');
});

test('a file without material maps still loads', async () => {
  const { trained } = await fixtures();
  const bytes = await (await fetch('../data/trained-16-sweeps.lsplat')).arrayBuffer();
  const plain = await decodeSplat(withoutMaterials(bytes));
  check(plain.materials === null, 'the stripped file still has material maps');
  check(plain.count === trained.count, `${plain.count} Gaussians, expected ${trained.count}`);
  check(plain.means.every((v, i) => v === trained.means[i]), 'positions differ');
  check(plain.features.every((v, i) => v === trained.features[i]), 'features differ');
  const cam = referenceCamera((await fixtures()).reference.cameras[0]);
  check(probe(plain, cam, 240, 180).materials === null, 'the probe returned material shares');
  return `${(bytes.byteLength / 1024).toFixed(0)} KB with, ${(plain.bytes / 1024).toFixed(0)} KB without`;
});

test('a probe\'s material shares add up to its coverage', async () => {
  const { reference, trained } = await fixtures();
  const m = trained.materials;
  let worst = 0, n = 0;
  for (const c of reference.cameras) {
    const cam = referenceCamera(c);
    for (const p of c.pixels) {
      const r = probe(trained, cam, p.x, p.y);
      const sum = (a) => a.reduce((x, y) => x + y, 0);
      near(sum(r.materials.labels), r.coverage, 1e-9, `material shares at (${p.x}, ${p.y})`);
      near(sum(r.materials.clusters), r.coverage, 1e-9, `cluster shares at (${p.x}, ${p.y})`);
      const d = Math.abs(sum(r.materials.abundances) - r.coverage);
      check(d <= (m.endmembers.length / 2 / 255) * r.coverage + 1e-9, `abundances at (${p.x}, ${p.y}) sum to ` +
        `${sum(r.materials.abundances).toFixed(4)}, coverage ${r.coverage.toFixed(4)}`);
      worst = Math.max(worst, d);
      n++;
    }
  }
  return `${n} pixels, abundances within ${fmt(worst)} of the coverage`;
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

// With the spectral channels at zero, a Gaussian that isn't shown in color
// adds nothing, so a palette of red for one category and green for the rest
// draws, in the red and green channels, exactly the shares the probe blends.
test('WebGL2 draws the material maps and abundances the CPU probe blends', async () => {
  const { reference, trained } = await fixtures();
  const renderer = await gpuRenderer();
  const m = trained.materials;
  const c = reference.cameras[0];
  const cam = referenceCamera(c);
  const pixels = grid(c, 6);
  const shares = pixels.map((p) => probe(trained, cam, p.x, p.y, { stopEarly: false }));
  const leaf = m.classes.findIndex((k) => k.name === 'leaf');
  const cases = [
    ['library match', { by: 'category', category: 'label', highlight: -1 }, m.classes.length, leaf,
      (r) => [r.materials.labels[leaf], r.materials.labels.reduce((a, b) => a + b, 0) - r.materials.labels[leaf] - r.materials.labels[m.classes.length]]],
    ['leaf alone', { by: 'category', category: 'label', highlight: leaf }, m.classes.length, leaf,
      (r) => [r.materials.labels[leaf], 0]],
    ['cluster 2', { by: 'category', category: 'cluster', highlight: -1 }, m.clusters.length, 1,
      (r) => [r.materials.clusters[1], r.coverage - r.materials.clusters[1]]],
    ['leaf abundance', { by: 'abundance', endmember: leaf }, 0, -1,
      (r) => [r.materials.abundances[leaf], r.materials.abundances[leaf]]],
  ];
  renderer.canvas.width = c.width;
  renderer.canvas.height = c.height;
  renderer.setChannels([]);
  const out = [];
  let worst = 0;
  try {
    for (const [name, colorBy, n, red, want] of cases) {
      const palette = Array.from({ length: n }, (_, k) => (k === red ? [1, 0, 0] : [0, 1, 0]));
      renderer.setColorBy(colorBy, palette);
      renderer.render(cam, { mode: 0, gain: 1 });
      const img = renderer.readLinear();
      let caseWorst = 0;
      pixels.forEach((p, j) => {
        const o = 4 * (p.y * img.width + p.x);
        const [r, g] = want(shares[j]);
        const d = Math.max(Math.abs(img.data[o] - r), Math.abs(img.data[o + 1] - g));
        check(d < 5e-3, `${name} at (${p.x}, ${p.y}): ${img.data[o].toFixed(4)}, ${img.data[o + 1].toFixed(4)} ` +
          `vs ${r.toFixed(4)}, ${g.toFixed(4)}`);
        caseWorst = Math.max(caseWorst, d);
      });
      worst = Math.max(worst, caseWorst);
      out.push(name);
    }
  } finally {
    renderer.setColorBy({ by: 'spectra' });
  }
  return `${pixels.length} pixels each for ${out.join(', ')}: largest difference ${fmt(worst)}`;
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
