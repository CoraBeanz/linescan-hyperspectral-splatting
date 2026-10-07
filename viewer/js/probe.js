// The full spectrum at one pixel, on the CPU.
//
// The GPU renderer only blends the three channels on screen. To plot every
// band at a clicked pixel, this projects every Gaussian the way the C++
// reference renderer does (project_to_line in line_camera.hpp, with an image
// row as one line camera, as pinhole_rows in preview.cpp makes it), sorts the
// ones that reach the pixel front to back, and blends their features until
// the pixel is opaque (render_cpu.cpp). The tests compare its spectra with
// the C++ renderer's.

import { gaussianFeatures } from './format.js';
import { unproject } from './camera.js';

// The renderers' shared thresholds (line_camera.hpp).
export const MIN_ALPHA = 1 / 255;
export const MAX_ALPHA = 0.99;
export const MIN_TRANSMITTANCE = 1e-4;
export const JACOBIAN_CLAMP = 1.3;
// The pinhole preview's settings (preview.cpp): a pixel's footprint is a
// box one pixel wide, which has variance 1/12, and the near plane is 5 mm.
export const PIXEL_VARIANCE = 1 / 12;
export const NEAR = 0.005;

// cam: {R, t, f, width, height} from camera.js. (px, py) is a pixel's index,
// counted from the top-left; its centre is at (px + 0.5, py + 0.5).
// Returns the spectrum (one value per band), how much of the pixel the
// Gaussians cover, and the 3D point they cover it at (null if nothing does).
//
// Like the C++ renderer, it stops at the Gaussian that would leave less than
// 1e-4 of the pixel's light, and leaves that one out. Drawing back to front,
// the GPU can't stop early, so where a pixel turns opaque its image keeps up
// to 1% more of the last Gaussians; stopEarly: false blends them all, as the
// GPU does, and the tests use it to compare the two.
export function probe(scene, cam, px, py, { stopEarly = true } = {}) {
  const { R, t, f, width, height } = cam;
  const N = scene.count, K = scene.numFeatures, B = scene.numBands;
  const { means, cov, opacity } = scene;
  const cu = width / 2;
  const vSlit = py + 0.5 - height / 2;
  const limX = JACOBIAN_CLAMP * Math.max(cu, width - cu) / f;
  const limY = limX + Math.abs(vSlit) / f;
  const last = width - 1;
  const hits = [];

  for (let i = 0; i < N; i++) {
    const op = opacity[i];
    if (!(op >= MIN_ALPHA)) continue;
    const mx = means[3 * i], my = means[3 * i + 1], mz = means[3 * i + 2];
    const xc = R[0] * mx + R[1] * my + R[2] * mz + t[0];
    const yc = R[3] * mx + R[4] * my + R[5] * mz + t[1];
    const zc = R[6] * mx + R[7] * my + R[8] * mz + t[2];
    if (!(zc > NEAR)) continue;
    const tx = xc / zc, ty = yc / zc;
    const muU = f * tx + cu;
    const dv = vSlit - f * ty;
    // The Jacobian at a clamped direction, so Gaussians far off to the side
    // don't get huge footprints (as in 3DGS).
    const txc = Math.min(Math.max(tx, -limX), limX);
    const tyc = Math.min(Math.max(ty, -limY), limY);
    const a = f / zc;
    const j0x = a * (R[0] - txc * R[6]), j0y = a * (R[1] - txc * R[7]), j0z = a * (R[2] - txc * R[8]);
    const j1x = a * (R[3] - tyc * R[6]), j1y = a * (R[4] - tyc * R[7]), j1z = a * (R[5] - tyc * R[8]);
    const c = 6 * i;
    const sxx = cov[c], sxy = cov[c + 1], sxz = cov[c + 2], syy = cov[c + 3], syz = cov[c + 4], szz = cov[c + 5];
    // S j1, then the projected 2D covariance [p q; q r].
    const s1x = sxx * j1x + sxy * j1y + sxz * j1z;
    const s1y = sxy * j1x + syy * j1y + syz * j1z;
    const s1z = sxz * j1x + syz * j1y + szz * j1z;
    const pRaw = j0x * (sxx * j0x + sxy * j0y + sxz * j0z) + j0y * (sxy * j0x + syy * j0y + syz * j0z) +
      j0z * (sxz * j0x + syz * j0y + szz * j0z);
    const q = j0x * s1x + j0y * s1y + j0z * s1z;
    const rRaw = j1x * s1x + j1y * s1y + j1z * s1z;
    const p = pRaw + PIXEL_VARIANCE;
    const r = rRaw + PIXEL_VARIANCE;
    const det = p * r - q * q;
    if (!(det > 0)) continue;
    // Blurring spreads a Gaussian's alpha out but keeps its total.
    const k = Math.sqrt(Math.max(pRaw * rRaw - q * q, 0) / det);
    const alphaRow = op * k * Math.exp(-0.5 * dv * dv / r);
    if (!(alphaRow >= MIN_ALPHA)) continue;
    // Cut along the row: a 1D Gaussian with this centre and variance.
    const varU = det / r;
    const u = muU + (q / r) * dv;
    const n2 = Math.min(9, 2 * Math.log(255 * alphaRow));
    const rad = Math.sqrt(varU * n2);
    const lo = u - rad - 0.5, hi = u + rad - 0.5;
    if (!(hi >= 0) || !(lo <= last)) continue;
    const p0 = lo <= 0 ? 0 : Math.ceil(lo);
    const p1 = hi >= last ? width : Math.floor(hi) + 1;
    if (px < p0 || px >= p1) continue;
    const d = px + 0.5 - u;
    const alpha = Math.min(MAX_ALPHA, alphaRow * Math.exp(-0.5 * d * d / varU));
    if (alpha < MIN_ALPHA) continue;
    hits.push({ depth: zc, alpha, i });
  }

  // Front to back; the sort is stable, so equal depths keep the file's order
  // as they do in the C++ renderer.
  hits.sort((x, y) => x.depth - y.depth);
  const feats = new Float64Array(K);
  const acc = new Float64Array(K);
  let trans = 1, weight = 0, depth = 0;
  for (const h of hits) {
    const next = trans * (1 - h.alpha);
    if (stopEarly && next < MIN_TRANSMITTANCE) break;
    const w = h.alpha * trans;
    gaussianFeatures(scene, h.i, feats);
    for (let c = 0; c < K; c++) acc[c] += w * feats[c];
    weight += w;
    depth += w * h.depth;
    trans = next;
  }
  for (let c = 0; c < K; c++) acc[c] += trans * scene.background[c];
  const spectrum = new Float64Array(B);
  for (let b = 0; b < B; b++) {
    let s = 0;
    for (let c = 0; c < K; c++) s += scene.basis[b * K + c] * acc[c];
    spectrum[b] = s;
  }
  const point = weight > 0 ? unproject(cam, px + 0.5, py + 0.5, depth / weight) : null;
  return { spectrum, coverage: 1 - trans, point };
}
