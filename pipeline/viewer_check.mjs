// Opens an exported splat the way the web viewer does, with its own reader and
// CPU probe (viewer/js), and checks the spectra it gives against the C++
// renderer's, which `splat_export --reference` wrote for the same file.
//
//   node pipeline/viewer_check.mjs SPLAT.lsplat REFERENCE.json [--bands B] [--sweeps S] [--json OUT.json]
//
// It needs Node 18 or newer and nothing else: no browser and no npm install.
// The page's WebGL2 image is checked against the same reference by the
// viewer's own tests (viewer/test), which this doesn't repeat.

import { readFile, writeFile } from 'node:fs/promises';
import { decodeSplat } from '../viewer/js/format.js';
import { probe } from '../viewer/js/probe.js';
import { lookAt } from '../viewer/js/camera.js';

// The reference is rounded to 4 decimals, as in viewer/test/tests.js.
const TOLERANCE = 5e-4;

function usage() {
  console.error('usage: node pipeline/viewer_check.mjs SPLAT.lsplat REFERENCE.json [--bands B] [--sweeps S] [--json OUT.json]');
  process.exit(2);
}

const args = process.argv.slice(2);
if (args.length < 2 || args[0].startsWith('-') || args[1].startsWith('-')) usage();
const [splatPath, referencePath] = args;
const option = (name) => {
  const at = args.indexOf(name);
  if (at < 0) return null;
  if (at + 1 >= args.length) usage();
  return args[at + 1];
};
const wantBands = option('--bands');
const wantSweeps = option('--sweeps');
const jsonOut = option('--json');

const problems = [];
const check = (ok, text) => {
  if (!ok) problems.push(text);
};

const bytes = await readFile(splatPath);
const scene = await decodeSplat(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength));
const reference = JSON.parse(await readFile(referencePath, 'utf8'));

// The file.
check(scene.count > 0, 'the splat has no Gaussians');
check(scene.wavelengths.length === reference.wavelengths_nm.length &&
  scene.wavelengths.every((nm, b) => Math.abs(nm - reference.wavelengths_nm[b]) < 1e-9),
'its wavelengths differ from the reference\'s');
if (wantBands !== null) check(scene.numBands === Number(wantBands), `${scene.numBands} bands, expected ${wantBands}`);
if (wantSweeps !== null)
  check(scene.sweeps.length === Number(wantSweeps), `${scene.sweeps.length} sweep fans, expected ${wantSweeps}`);
let badShapes = 0;
for (let i = 0; i < scene.count; i++) {
  const c = scene.cov.subarray(6 * i, 6 * i + 6);
  if (!(c[0] > 0 && c[3] > 0 && c[5] > 0) || !(scene.opacity[i] >= 0 && scene.opacity[i] <= 1)) badShapes++;
}
check(badShapes === 0, `${badShapes} Gaussians have a covariance that isn't positive or an opacity outside 0..1`);

// The probe against the C++ renderer, at every reference pixel.
let worst = 0, worstCoverage = 0, pixels = 0;
const covered = [];
for (const c of reference.cameras) {
  let n = 0;
  const cam = { ...lookAt(c.view.eye, c.view.target, c.view.up), f: c.f_px, width: c.width, height: c.height };
  for (const p of c.pixels) {
    const r = probe(scene, cam, p.x, p.y);
    let d = 0;
    for (let b = 0; b < r.spectrum.length; b++) d = Math.max(d, Math.abs(r.spectrum[b] - p.spectrum[b]));
    worst = Math.max(worst, d);
    worstCoverage = Math.max(worstCoverage, Math.abs(r.coverage - p.coverage));
    if (r.coverage > 0.5) n++;
    pixels++;
  }
  covered.push(c.pixels.length ? n / c.pixels.length : 0);
}
check(pixels > 0, 'the reference has no pixels');
check(worst < TOLERANCE, `the probe's spectra differ from the C++ renderer's by up to ${worst.toExponential(2)}`);
check(worstCoverage < TOLERANCE, `the probe's coverage differs from the C++ renderer's by up to ${worstCoverage.toExponential(2)}`);

const result = {
  ok: problems.length === 0,
  problems,
  title: scene.title,
  gaussians: scene.count,
  bands: scene.numBands,
  features: scene.numFeatures,
  sweeps: scene.sweeps.length,
  bytes: scene.bytes,
  pixels_compared: pixels,
  max_spectrum_difference: worst,
  max_coverage_difference: worstCoverage,
  // How much of the default view's middle the splat fills (the first reference camera is the
  // view the page opens with, which should frame the scene), then the other views'.
  covered_fraction: covered[0] || 0,
  covered_fractions: covered,
};
if (jsonOut) await writeFile(jsonOut, JSON.stringify(result, null, 1) + '\n');
console.log(`viewer    ${scene.count} Gaussians, ${scene.numBands} bands, ${scene.sweeps.length} sweep fans; ` +
  `probe vs C++ at ${pixels} pixels: ${worst.toExponential(1)} in a band, ${worstCoverage.toExponential(1)} in coverage; ` +
  `${Math.round(100 * result.covered_fraction)}% of the default view's covered`);
for (const p of problems) console.log(`  ! ${p}`);
process.exit(result.ok ? 0 : 1);
