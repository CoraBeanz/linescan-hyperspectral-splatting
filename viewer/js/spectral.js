// Spectra to display values.
//
// Everything the viewer shows is linear in the spectrum: one wavelength, the
// mean over a band, true color, color infrared. So each is a vector of
// weights over the bands, and a pixel's value is the weighted sum of its
// band values. The renderer blends three such channels per Gaussian.
//
// The color math is ported from splat/src/spectra.cpp, so the viewer's true
// color and CIR match the C++ previews; the tests check the two agree.

// Adds `scale` times the weights that interpolate a spectrum at `nm` to `out`:
// linear between band centres, held at the end values past either end.
export function addSampleWeights(wl, nm, out, scale = 1) {
  const n = wl.length;
  if (nm <= wl[0]) {
    out[0] += scale;
    return out;
  }
  if (nm >= wl[n - 1]) {
    out[n - 1] += scale;
    return out;
  }
  let i = 1;
  while (wl[i] <= nm) i++;  // the first centre above nm
  const t = (nm - wl[i - 1]) / (wl[i] - wl[i - 1]);
  out[i - 1] += scale * (1 - t);
  out[i] += scale * t;
  return out;
}

// The mean over [lo, hi], sampled every nanometre (band_mean in spectra.cpp).
export function bandMeanWeights(wl, lo, hi) {
  const out = new Float64Array(wl.length);
  let n = 0;
  for (let nm = lo; nm <= hi + 1e-9; nm += 1) n++;
  for (let nm = lo; nm <= hi + 1e-9; nm += 1) addSampleWeights(wl, nm, out, 1 / n);
  return out;
}

// One wavelength (width 0) or the mean over a band `width` nm wide.
export function wavelengthWeights(wl, centre, width = 0) {
  if (width <= 0) return addSampleWeights(wl, centre, new Float64Array(wl.length));
  return bandMeanWeights(wl, centre - width / 2, centre + width / 2);
}

// The CIE 1931 color matching functions, as the analytic fit of Wyman, Sloan
// and Shirley, "Simple Analytic Approximations to the CIE XYZ Color Matching
// Functions" (JCGT 2013).
function lobe(x, mu, s1, s2) {
  const s = x < mu ? s1 : s2;
  return Math.exp(-0.5 * (x - mu) * (x - mu) / (s * s));
}

export function cieXYZ(nm) {
  return [
    1.056 * lobe(nm, 599.8, 37.9, 31.0) + 0.362 * lobe(nm, 442.0, 16.0, 26.7) - 0.065 * lobe(nm, 501.1, 20.4, 26.2),
    0.821 * lobe(nm, 568.8, 46.9, 40.5) + 0.286 * lobe(nm, 530.9, 16.3, 31.1),
    1.217 * lobe(nm, 437.0, 11.8, 36.0) + 0.681 * lobe(nm, 459.0, 26.0, 13.8),
  ];
}

export function xyzToLinearSrgb(X, Y, Z) {
  return [
    3.2406 * X - 1.5372 * Y - 0.4986 * Z,
    -0.9689 * X + 1.8758 * Y + 0.0415 * Z,
    0.0557 * X - 0.2040 * Y + 1.0570 * Z,
  ];
}

// True color: the CIE 1931 observer under equal-energy light, white balanced
// so reflectance 1 is white. Below the first band (the GG-495 filter cuts
// there) the first band's value is held, so blues are only approximate.
// Returns weights for linear R, G and B.
export function trueColorWeights(wl) {
  const xyz = [new Float64Array(wl.length), new Float64Array(wl.length), new Float64Array(wl.length)];
  let Xw = 0, Yw = 0, Zw = 0;
  for (let nm = 380; nm <= 780; nm += 2) {
    const [x, y, z] = cieXYZ(nm);
    addSampleWeights(wl, nm, xyz[0], x);
    addSampleWeights(wl, nm, xyz[1], y);
    addSampleWeights(wl, nm, xyz[2], z);
    Xw += x;
    Yw += y;
    Zw += z;
  }
  const white = xyzToLinearSrgb(Xw / Yw, 1, Zw / Yw);
  const rgb = [new Float64Array(wl.length), new Float64Array(wl.length), new Float64Array(wl.length)];
  for (let b = 0; b < wl.length; b++) {
    const c = xyzToLinearSrgb(xyz[0][b] / Yw, xyz[1][b] / Yw, xyz[2][b] / Yw);
    for (let k = 0; k < 3; k++) rgb[k][b] = c[k] / white[k];
  }
  return rgb;
}

// The classic color-infrared false color: red shows 800-900 nm, green
// 620-680 nm and blue 520-580 nm, so vegetation comes out red.
export const CIR_BANDS = [[800, 900], [620, 680], [520, 580]];

export function colorInfraredWeights(wl) {
  return CIR_BANDS.map(([lo, hi]) => bandMeanWeights(wl, lo, hi));
}

export function dot(weights, spectrum) {
  let s = 0;
  for (let b = 0; b < weights.length; b++) s += weights[b] * spectrum[b];
  return s;
}

export function srgbEncode(x) {
  x = Math.min(Math.max(x, 0), 1);
  return x <= 0.0031308 ? 12.92 * x : 1.055 * Math.pow(x, 1 / 2.4) - 0.055;
}

// How light of one wavelength looks, as an sRGB triple in 0..1 for the
// interface (the slider tracks, the chart's strip, the readout's accent):
// Dan Bruton's piecewise spectrum, whose hues read as the familiar rainbow
// where the CIE colors, clipped into sRGB, turn reds pink. Past the red end
// of vision it fades to a dim violet-gray, a stand-in for "nothing to see".
export const NIR_COLOR = [0.36, 0.3, 0.42];

export function spectralColor(nm) {
  let r = 0, g = 0, b = 0;
  if (nm < 440) {
    r = (440 - nm) / 60;
    b = 1;
  } else if (nm < 490) {
    g = (nm - 440) / 50;
    b = 1;
  } else if (nm < 510) {
    g = 1;
    b = (510 - nm) / 20;
  } else if (nm < 580) {
    r = (nm - 510) / 70;
    g = 1;
  } else if (nm < 645) {
    r = 1;
    g = (645 - nm) / 65;
  } else {
    r = 1;
  }
  const t = Math.min(Math.max((nm - 680) / 100, 0), 1);
  return [r, g, b].map((v, k) => Math.max(0, Math.min(1, v)) * (1 - t) + NIR_COLOR[k] * t);
}

export function cssColor([r, g, b], alpha = 1) {
  const to8 = (v) => Math.round(Math.min(Math.max(v, 0), 1) * 255);
  return alpha >= 1 ? `rgb(${to8(r)} ${to8(g)} ${to8(b)})` : `rgb(${to8(r)} ${to8(g)} ${to8(b)} / ${alpha})`;
}

// Where a wavelength sits, in words.
export function describeWavelength(nm) {
  if (nm < 450) return 'Violet';
  if (nm < 490) return 'Blue';
  if (nm < 520) return 'Cyan';
  if (nm < 565) return 'Green';
  if (nm < 590) return 'Yellow';
  if (nm < 625) return 'Orange';
  if (nm < 700) return 'Red';
  if (nm < 750) return 'Edge of vision';
  return 'Near infrared, invisible to the eye';
}
