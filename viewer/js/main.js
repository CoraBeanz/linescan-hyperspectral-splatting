// The page: loading scenes, the controls, orbiting, sampling spectra.

import { decodeSplat, fetchSplat } from './format.js';
import { OrbitCamera, projectPoint } from './camera.js';
import { SplatRenderer } from './renderer.js';
import { NEAR, probe } from './probe.js';
import { SpectrumChart } from './chart.js';
import {
  CIR_BANDS, colorInfraredWeights, cssColor, describeWavelength, dot, spectralColor, srgbEncode,
  trueColorWeights, wavelengthWeights,
} from './spectral.js';

const SAMPLES = [
  {
    id: 'trained',
    file: 'data/trained-16-sweeps.lsplat',
    label: 'Trained from 16 sweeps',
    about: 'This splat was trained by splat_train on 3,424 synthetic scan lines: 16 sweeps of 256 pixels and 46 bands, ' +
      'taken from head poses that were off by up to 3.3 mm. Training brought the pose error from 10.9 px to 0.97 px.',
  },
  {
    id: 'truth',
    file: 'data/ground-truth.lsplat.gz',
    label: 'Ground truth',
    about: 'The synthetic scene the scan lines were rendered from: a board with paint patches, a panel hiding a word in ' +
      'infrared-clear dye, a leaf-green ball and an orange box, on a wooden table. Compare it with the trained splat.',
  },
];

// Points worth sampling in the synthetic scene, from its layout in
// splat/src/synthetic.cpp: the ball's centre (which lands on its near side),
// the middle of the box's lid, and a patch of bare dye inside the hidden word.
const SYNTHETIC_POINTS = [
  { label: 'Leaf-green ball', point: [0.014, -0.012, 0.009] },
  { label: 'Orange box', point: [0.016, 0.004, 0.01] },
  { label: 'Black panel', point: [-0.0188, -0.014, 0] },
];

// Categorical colors for the sampled points, in a fixed order that stays
// apart for color-blind readers on this panel.
const PROBE_COLORS = ['#3987e5', '#d95926', '#199e70', '#c98500'];

const MODE_NOTES = {
  true: 'What your eye would see, computed from every band with the CIE 1931 color matching functions.',
  band: 'One wavelength, or the mean over a band, in gray. Past 740 nm the black panel shows a hidden word.',
  cir: 'Near infrared shown as red, red as green and green as blue. Leaves turn red, and so does the black panel, whose dye is clear past 740 nm.',
  index: 'The normalized difference (A − B) / (A + B) of two wavelengths. With A in the near infrared and B in the red, it is NDVI.',
  materials: 'Each Gaussian in the color of the material its spectrum matches, or of its cluster. Pick a material in the list or click one in the scene to show it alone.',
  abundance: 'How much of one endmember each Gaussian is made of, from linear unmixing: its spectrum as a mix of the endmembers, with shares that sum to one.',
};

const MATERIAL_MODES = ['materials', 'abundance'];

const $ = (id) => document.getElementById(id);
const view = $('view');
const overlay = $('overlay');
const stage = $('stage');

const state = {
  mode: 'true',
  wavelength: 800,
  bandwidth: 0,
  indexA: 800,
  indexB: 670,
  exposure: 0,
  showSweeps: false,
  scene: null,
  sceneId: null,
  probes: [],
  nextProbe: 1,
  group: 'label',   // color materials by the library match ('label') or the cluster
  highlight: -1,    // the one material or cluster shown in color, or -1 for all
  endmember: 0,     // whose abundance is shown
};

let renderer = null;
let camera = null;
let weights = null;
let trueColorCache = null;
let pending = false;
let animation = null;
let toastTimer = 0;

const chart = new SpectrumChart($('chart'), { onPick: pickWavelength });

function setStatus(text) {
  const s = $('status');
  s.textContent = text || '';
  s.hidden = !text;
}

function toast(text) {
  const t = $('toast');
  t.textContent = text;
  t.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { t.hidden = true; }, 2600);
}

// --- Drawing ----------------------------------------------------------------

function pixelRatio() { return Math.min(window.devicePixelRatio || 1, 2); }

function frame() { return camera.frame(view.width, view.height); }

function requestDraw() {
  if (pending) return;
  pending = true;
  requestAnimationFrame(() => {
    pending = false;
    draw();
  });
}

function draw() {
  if (!renderer || !state.scene || !camera) return;
  const cam = frame();
  const material = MATERIAL_MODES.includes(state.mode);
  renderer.render(cam, {
    mode: displayMode(),
    gain: material ? 1 : 2 ** state.exposure,
    background: material ? [0, 0, 0] : undefined,
  });
  drawOverlay(cam);
}

function displayMode() {
  return { band: 1, index: 2, abundance: 3 }[state.mode] || 0;
}

// Points in camera space, clipped to the near plane, then projected.
function projectSegment(cam, a, b) {
  const toCam = (p) => [
    cam.R[0] * p[0] + cam.R[1] * p[1] + cam.R[2] * p[2] + cam.t[0],
    cam.R[3] * p[0] + cam.R[4] * p[1] + cam.R[5] * p[2] + cam.t[1],
    cam.R[6] * p[0] + cam.R[7] * p[1] + cam.R[8] * p[2] + cam.t[2],
  ];
  let p = toCam(a), q = toCam(b);
  const near = 0.01;
  if (p[2] < near && q[2] < near) return null;
  if (p[2] < near || q[2] < near) {
    const s = (near - p[2]) / (q[2] - p[2]);
    const m = [p[0] + s * (q[0] - p[0]), p[1] + s * (q[1] - p[1]), near];
    if (p[2] < near) p = m; else q = m;
  }
  const px = (c) => [cam.f * c[0] / c[2] + cam.width / 2, cam.f * c[1] / c[2] + cam.height / 2];
  return [px(p), px(q)];
}

function drawOverlay(cam) {
  const ctx = overlay.getContext('2d');
  ctx.clearRect(0, 0, overlay.width, overlay.height);
  const r = pixelRatio();
  if (state.showSweeps) {
    ctx.lineWidth = 1 * r;
    for (const s of state.scene.sweeps) {
      const lines = [];
      for (const c of s.corners) lines.push([s.apex, c, 0.28]);
      s.corners.forEach((c, k) => lines.push([c, s.corners[(k + 1) % 4], 0.7]));
      for (const [a, b, alpha] of lines) {
        const seg = projectSegment(cam, a, b);
        if (!seg) continue;
        ctx.strokeStyle = `rgb(214 222 236 / ${alpha})`;
        ctx.beginPath();
        ctx.moveTo(seg[0][0], seg[0][1]);
        ctx.lineTo(seg[1][0], seg[1][1]);
        ctx.stroke();
      }
      const apex = projectPoint(cam, s.apex, 0.01);
      if (apex) {
        ctx.fillStyle = 'rgb(214 222 236 / 0.9)';
        ctx.beginPath();
        ctx.arc(apex[0], apex[1], 2.5 * r, 0, 2 * Math.PI);
        ctx.fill();
      }
    }
  }
  for (const p of state.probes) {
    const at = projectPoint(cam, p.point, NEAR);
    if (!at) continue;
    const [x, y] = at;
    ctx.lineWidth = 4 * r;
    ctx.strokeStyle = 'rgb(11 13 16 / 0.85)';
    ctx.beginPath();
    ctx.arc(x, y, 7 * r, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.lineWidth = 2 * r;
    ctx.strokeStyle = p.color;
    ctx.stroke();
    ctx.font = `600 ${13 * r}px "Martian Mono", ui-monospace, monospace`;
    ctx.textBaseline = 'middle';
    ctx.lineWidth = 3 * r;
    ctx.strokeStyle = 'rgb(11 13 16 / 0.9)';
    ctx.strokeText(String(p.number), x + 11 * r, y);
    ctx.fillStyle = '#e8e6e1';
    ctx.fillText(String(p.number), x + 11 * r, y);
  }
}

// --- What's shown -------------------------------------------------------------

function channelWeights() {
  const wl = state.scene.wavelengths;
  switch (state.mode) {
    case 'band':
      return [wavelengthWeights(wl, state.wavelength, state.bandwidth)];
    case 'cir':
      return colorInfraredWeights(wl);
    case 'index':
      return [wavelengthWeights(wl, state.indexA), wavelengthWeights(wl, state.indexB)];
    default:  // true color, which the material views also use for their grays
      return trueColorCache || (trueColorCache = trueColorWeights(wl));
  }
}

function updateChannels() {
  if (!state.scene) return;
  weights = channelWeights();
  renderer.setChannels(weights);
  applyColorBy();
  updateReadout();
  updateLegend();
  requestDraw();
}

// --- Material maps ------------------------------------------------------------

// The materials or clusters the material view colors by.
function categories() {
  const m = state.scene?.materials;
  if (!m) return [];
  return state.group === 'cluster' ? m.clusters : m.classes;
}

function applyColorBy() {
  const m = state.scene?.materials;
  if (!renderer) return;
  if (m && state.mode === 'materials')
    renderer.setColorBy({ by: 'category', category: state.group, highlight: state.highlight }, categories().map((c) => c.color));
  else if (m && state.mode === 'abundance')
    renderer.setColorBy({ by: 'abundance', endmember: state.endmember });
  else
    renderer.setColorBy({ by: 'spectra' });
}

function percent(x) {
  const p = 100 * x;
  return p > 0 && p < 1 ? '<1%' : `${Math.round(p)}%`;
}

// The material (or cluster) covering most of a sampled point, and its share:
// {name, index, share}, index -1 where no library material matches.
function dominantMaterial(result) {
  const m = state.scene?.materials;
  if (!m || !result.materials || !(result.coverage > 0)) return null;
  const shares = state.group === 'cluster' ? result.materials.clusters : result.materials.labels;
  let best = 0;
  for (let k = 1; k < shares.length; k++) if (shares[k] > shares[best]) best = k;
  const known = best < categories().length;
  return { index: known ? best : -1, name: known ? categories()[best].name : 'No match', share: shares[best] / result.coverage };
}

function setHighlight(index) {
  state.highlight = index;
  applyColorBy();
  updateMaterialList();
  updateReadout();
  requestDraw();
}

// The list of materials or clusters, largest first: a color, a name, the
// share of Gaussians, and for a cluster the library material nearest it.
function updateMaterialList() {
  const list = $('material-list');
  list.replaceChildren();
  const m = state.scene?.materials;
  if (!m) return;
  const N = state.scene.count;
  const rows = categories().map((c, k) => ({ c, k })).sort((a, b) => b.c.count - a.c.count);
  for (const { c, k } of rows) {
    const li = document.createElement('li');
    const button = document.createElement('button');
    button.type = 'button';
    button.dataset.index = String(k);
    button.setAttribute('aria-pressed', String(state.highlight === k));
    button.disabled = c.count === 0;
    const chip = document.createElement('span');
    chip.className = 'chip';
    chip.style.background = c.css;
    const name = document.createElement('span');
    name.className = 'name';
    name.textContent = c.name;
    if (state.group === 'cluster') {
      const sub = document.createElement('span');
      sub.className = 'sub';
      const near = c.nearest >= 0 ? m.classes[c.nearest].name : '';
      sub.textContent = !near ? 'Like no library spectrum'
        : c.brightnessMatches ? `Nearest: ${near}, ${c.angleDeg.toFixed(1)}°`
        : `Shaped like ${near}, ${c.angleDeg.toFixed(1)}°, but brighter or darker`;
      name.appendChild(sub);
    }
    const share = document.createElement('span');
    share.className = 'share';
    share.textContent = percent(c.count / N);
    share.title = `${formatCount(c.count)} Gaussians`;
    button.append(chip, name, share);
    button.addEventListener('click', () => setHighlight(state.highlight === k ? -1 : k));
    li.appendChild(button);
    list.appendChild(li);
  }
  if (state.group === 'label' && m.unknown) {
    const li = document.createElement('li');
    li.className = 'unmatched';
    li.textContent = `No match: ${percent(m.unknown / N)} of Gaussians, shown in gray`;
    list.appendChild(li);
  }
  $('show-all').hidden = state.highlight < 0;
  const score = m.score?.view;
  const scored = score ? ` Against the known materials, ${percent(score.label_accuracy)} of the opening view's pixels that show one material get the right one (${percent(m.score.label_accuracy)} of Gaussians).` : '';
  $('material-note').textContent = state.group === 'cluster'
    ? `k-means on the spectra with ${m.clusters.length} clusters, weighted by opacity, found without the library; each is named after the library spectrum nearest its mean, if one is within ${m.labels.max_angle_deg}°.`
    : `Spectral angle against ${m.classes.length} library spectra, among those within ${m.labels.brightness_window}× in brightness, up to ${m.labels.max_angle_deg}°.${scored}`;
}

function updateEndmembers() {
  const select = $('endmember');
  select.replaceChildren();
  const m = state.scene?.materials;
  if (!m) return;
  m.endmembers.forEach((e, k) => {
    const o = document.createElement('option');
    o.value = String(k);
    o.textContent = e.name;
    select.appendChild(o);
  });
  select.value = String(state.endmember);
}

function wavelengthLabel(nm) {
  return `${Math.round(nm)} nm`;
}

function updateReadout() {
  const main = $('readout-main');
  const swatch = $('readout-swatch');
  const scale = $('readout-scale');
  const bar = $('readout-scale-bar');
  const ticks = $('readout-scale-ticks');
  const caption = $('readout-caption');
  const B = state.scene ? state.scene.numBands : 0;
  main.className = 'readout-main';
  swatch.hidden = true;
  scale.hidden = true;
  let accent = '#d9d4c7';
  const gain = 2 ** state.exposure;
  const fmt = (v) => String(+v.toFixed(2));
  const m = state.scene?.materials;
  if (state.mode === 'materials' && m) {
    $('readout-kicker').textContent = state.group === 'cluster' ? 'Clusters' : 'Materials';
    const picked = state.highlight >= 0 ? categories()[state.highlight] : null;
    main.textContent = picked ? picked.name : state.group === 'cluster' ? `${m.clusters.length} clusters` : 'Library match';
    if (picked) {
      accent = picked.css;
      swatch.hidden = false;
      caption.textContent = `${formatCount(picked.count)} Gaussians · ${percent(picked.count / state.scene.count)}`;
    } else {
      caption.textContent = state.group === 'cluster' ? 'k-means on the spectra' : `Spectral angle, ${m.classes.length} library spectra`;
    }
  } else if (state.mode === 'abundance' && m) {
    const e = m.endmembers[state.endmember];
    $('readout-kicker').textContent = 'Abundance';
    main.textContent = e.name;
    accent = e.css;
    caption.textContent = `Linear unmixing, ${m.endmembers.length} endmembers`;
    scale.hidden = false;
    bar.style.background = 'linear-gradient(to right, #440154, #3b528b, #21918c, #5ec962, #fde725)';
    ticks.replaceChildren(...['0', '0.5', '1'].map(textSpan));
  } else if (state.mode === 'band') {
    $('readout-kicker').textContent = state.bandwidth > 0 ? 'Band' : 'Wavelength';
    main.className = 'readout-main number';
    main.replaceChildren(String(Math.round(state.wavelength)));
    const unit = document.createElement('span');
    unit.className = 'unit';
    unit.textContent = 'nm';
    main.appendChild(unit);
    accent = cssColor(spectralColor(state.wavelength));
    swatch.hidden = false;
    const what = describeWavelength(state.wavelength);
    caption.textContent = state.bandwidth > 0 ? `Mean over ${state.bandwidth} nm · ${what}` : what;
    scale.hidden = false;
    bar.style.background = `linear-gradient(to right, #000, #fff)`;
    ticks.replaceChildren(...['0', fmt(0.5 / gain), `${fmt(1 / gain)} ${state.scene?.values || ''}`].map(textSpan));
  } else if (state.mode === 'cir') {
    $('readout-kicker').textContent = 'Showing';
    main.textContent = 'Color infrared';
    caption.textContent = CIR_BANDS.map(([lo, hi], k) => `${['Red', 'green', 'blue'][k]} ${lo}–${hi}`).join(' · ') + ' nm';
  } else if (state.mode === 'index') {
    $('readout-kicker').textContent = 'Band index';
    main.textContent = '(A − B) / (A + B)';
    caption.textContent = `A ${wavelengthLabel(state.indexA)} · B ${wavelengthLabel(state.indexB)}`;
    scale.hidden = false;
    bar.style.background = 'linear-gradient(to right, #a9cdf6, #2f7ad8, #44443f, #c94a48, #f3a19b)';
    ticks.replaceChildren(...['−1', '0', '+1'].map(textSpan));
  } else {
    $('readout-kicker').textContent = 'Showing';
    main.textContent = 'True color';
    caption.textContent = `CIE 1931 observer, from ${B} bands`;
  }
  document.documentElement.style.setProperty('--accent', accent);
  $('mode-note').textContent = MODE_NOTES[state.mode];
  for (const [id, modes] of [['ctl-wavelength', ['band']], ['ctl-bandwidth', ['band']], ['ctl-index-a', ['index']],
    ['ctl-index-b', ['index']], ['ctl-exposure', ['true', 'band', 'cir']], ['ctl-materials', ['materials']],
    ['ctl-endmember', ['abundance']]])
    $(id).hidden = !modes.includes(state.mode);
  $('wavelength').value = Math.round(state.wavelength);
  $('wavelength-out').textContent = wavelengthLabel(state.wavelength);
  $('bandwidth-out').textContent = state.bandwidth > 0 ? `${state.bandwidth} nm` : 'Single wavelength';
  $('index-a-out').textContent = wavelengthLabel(state.indexA);
  $('index-b-out').textContent = wavelengthLabel(state.indexB);
  $('exposure-out').textContent = `${state.exposure > 0 ? '+' : state.exposure < 0 ? '−' : ''}${Math.abs(state.exposure)} EV`;
  chart.setMarkers(chartMarkers());
  chart.setReference(chartReference());
  chart.render();
}

// The picked material's or the endmember's own spectrum, dashed on the chart.
function chartReference() {
  const m = state.scene?.materials;
  if (!m) return null;
  if (state.mode === 'materials' && state.highlight >= 0) {
    const c = categories()[state.highlight];
    return { label: state.group === 'cluster' ? `${c.name} mean` : `${c.name} (library)`, color: c.css, spectrum: c.spectrum };
  }
  if (state.mode === 'abundance') {
    const e = m.endmembers[state.endmember];
    return { label: `${e.name} (endmember)`, color: e.css, spectrum: e.spectrum };
  }
  return null;
}

function textSpan(text) {
  const s = document.createElement('span');
  s.textContent = text;
  return s;
}

function chartMarkers() {
  if (state.mode === 'band') {
    if (state.bandwidth > 0)
      return [{ kind: 'band', lo: state.wavelength - state.bandwidth / 2, hi: state.wavelength + state.bandwidth / 2, label: wavelengthLabel(state.wavelength) }];
    return [{ kind: 'line', nm: state.wavelength, label: wavelengthLabel(state.wavelength) }];
  }
  if (state.mode === 'cir') return CIR_BANDS.map(([lo, hi], k) => ({ kind: 'band', lo, hi, label: 'RGB'[k] }));
  if (state.mode === 'index') return [{ kind: 'line', nm: state.indexA, label: 'A' }, { kind: 'line', nm: state.indexB, label: 'B' }];
  return [];
}

// --- Sampled points -------------------------------------------------------------

function freeColor() {
  const used = new Set(state.probes.map((p) => p.color));
  return PROBE_COLORS.find((c) => !used.has(c));
}

// Samples the scene at a world point, as seen from the current view.
function sampleWorldPoint(point) {
  const cam = frame();
  const at = projectPoint(cam, point, NEAR);
  if (!at || at[0] < 0 || at[1] < 0 || at[0] >= cam.width || at[1] >= cam.height) return null;
  const r = probe(state.scene, cam, Math.floor(at[0]), Math.floor(at[1]));
  return r.coverage > 0.2 ? r : null;
}

function addProbe(label, result, anchor) {
  if (state.probes.length >= PROBE_COLORS.length) state.probes.shift();
  const number = state.nextProbe++;
  state.probes.push({
    number,
    label: label || `Point ${number}`,
    color: freeColor(),
    spectrum: result.spectrum,
    coverage: result.coverage,
    materials: result.materials,
    point: anchor || result.point,
  });
  updateProbeViews();
  requestDraw();
}

function sampleAt(clientX, clientY) {
  const rect = view.getBoundingClientRect();
  const px = Math.floor(((clientX - rect.left) / rect.width) * view.width);
  const py = Math.floor(((clientY - rect.top) / rect.height) * view.height);
  const r = probe(state.scene, frame(), px, py);
  if (r.coverage < 0.2 || !r.point) {
    toast('Nothing to sample there. Click on the scene itself.');
    return;
  }
  addProbe(null, r);
  // In the material view, a click also picks the material there.
  if (state.mode === 'materials') {
    const d = dominantMaterial(r);
    if (d && d.index >= 0) setHighlight(d.index);
    else if (d) toast('No library material matches the spectrum there.');
  }
}

function addPresetProbes() {
  if (!state.scene.header.source?.synthetic) return;
  for (const p of SYNTHETIC_POINTS) {
    const r = sampleWorldPoint(p.point);
    if (r) addProbe(p.label, r, p.point);
  }
}

// After a scene change, sample the same places in the new scene.
function resampleProbes() {
  for (const p of state.probes) {
    const r = sampleWorldPoint(p.point);
    p.spectrum = r ? r.spectrum : new Float64Array(state.scene.numBands);
    p.coverage = r ? r.coverage : 0;
    p.materials = r ? r.materials : null;
  }
  updateProbeViews();
}

function probeValueText(p) {
  if (!weights) return '';
  if (state.mode === 'materials' && state.scene?.materials) {
    const d = dominantMaterial(p);
    return d ? `${d.name} ${percent(d.share)}` : '–';
  }
  if (state.mode === 'abundance' && p.materials)
    return p.coverage > 0 ? (p.materials.abundances[state.endmember] / p.coverage).toFixed(2) : '–';
  if (state.mode === 'band') return dot(weights[0], p.spectrum).toFixed(3);
  if (state.mode === 'index') {
    const a = dot(weights[0], p.spectrum), b = dot(weights[1], p.spectrum);
    const nd = Math.abs(a + b) > 1e-6 ? (a - b) / (a + b) : 0;
    return `${nd >= 0 ? '+' : '−'}${Math.abs(nd).toFixed(2)}`;
  }
  return null;
}

// The legend: each point's name, and its value (or color) in this view.
function updateLegend() {
  const legend = $('legend');
  legend.replaceChildren();
  const gain = 2 ** state.exposure;
  for (const p of state.probes) {
    const li = document.createElement('li');
    const key = document.createElement('span');
    key.className = 'line-key';
    key.style.background = p.color;
    const name = document.createElement('span');
    name.className = 'name';
    const num = document.createElement('span');
    num.className = 'num';
    num.textContent = String(p.number);
    name.append(num, document.createTextNode(p.label));
    const value = document.createElement('span');
    value.className = 'value';
    const text = probeValueText(p);
    if (text !== null) {
      value.textContent = text;
    } else if (weights) {
      // What the point looks like in this view.
      const chip = document.createElement('span');
      const rgb = weights.map((w) => srgbEncode(gain * dot(w, p.spectrum)));
      chip.style.cssText = `display:inline-block;width:22px;height:14px;border-radius:3px;vertical-align:middle;background:${cssColor(rgb)}`;
      chip.title = 'How this point looks in the current view';
      value.appendChild(chip);
    }
    const remove = document.createElement('button');
    remove.type = 'button';
    remove.textContent = '×';
    remove.setAttribute('aria-label', `Remove ${p.label}`);
    remove.addEventListener('click', () => {
      state.probes = state.probes.filter((q) => q !== p);
      updateProbeViews();
      requestDraw();
    });
    li.append(key, name, value, remove);
    legend.appendChild(li);
  }
  $('clear-probes').hidden = !state.probes.length;
}

// Everything that shows the sampled spectra: the legend, the chart and the table.
function updateProbeViews() {
  updateLegend();
  chart.setWavelengths(state.scene ? state.scene.wavelengths : []);
  chart.valueLabel = state.scene ? state.scene.values : 'reflectance';
  chart.setProbes(state.probes);
  chart.render();

  const table = $('values');
  table.replaceChildren();
  if (!state.scene) return;
  const head = table.createTHead().insertRow();
  const th = (text) => {
    const c = document.createElement('th');
    c.textContent = text;
    head.appendChild(c);
  };
  th('nm');
  for (const p of state.probes) th(`${p.number} ${p.label}`);
  const body = table.createTBody();
  state.scene.wavelengths.forEach((nm, b) => {
    const row = body.insertRow();
    row.insertCell().textContent = String(Math.round(nm * 10) / 10);
    for (const p of state.probes) row.insertCell().textContent = p.spectrum[b].toFixed(4);
  });
}

// --- Scenes -------------------------------------------------------------------

function formatCount(n) { return n.toLocaleString('en-US'); }

function useScene(scene, { keepView }) {
  renderer.setScene(scene);
  const sameFrame = keepView && camera && state.scene;
  state.scene = scene;
  trueColorCache = null;
  if (!sameFrame) camera = new OrbitCamera(scene.view);
  const wl = scene.wavelengths;
  const lo = Math.ceil(wl[0]), hi = Math.floor(wl[wl.length - 1]);
  for (const id of ['wavelength', 'index-a', 'index-b']) {
    const input = $(id);
    input.min = lo;
    input.max = hi;
  }
  state.wavelength = Math.min(Math.max(state.wavelength, lo), hi);
  state.indexA = Math.min(Math.max(state.indexA, lo), hi);
  state.indexB = Math.min(Math.max(state.indexB, lo), hi);
  $('index-a').value = state.indexA;
  $('index-b').value = state.indexB;
  if (!animation) $('play').textContent = playLabel();
  paintSpectralTracks(lo, hi);

  const sweeps = scene.sweeps.length;
  $('stats').textContent = [
    `${formatCount(scene.count)} Gaussians`,
    `${scene.numBands} bands, ${Math.round(wl[0])}–${Math.round(wl[wl.length - 1])} nm`,
    sweeps ? `${sweeps} sweeps` : null,
    `${(scene.bytes / 1048576).toFixed(1)} MB`,
  ].filter(Boolean).join(' · ');
  $('show-sweeps').disabled = !sweeps;
  const sample = SAMPLES.find((s) => s.id === state.sceneId);
  $('about-sample').textContent = sample ? sample.about
    : `${scene.title}: ${formatCount(scene.count)} Gaussians${scene.header.source?.scene ? ` from ${scene.header.source.scene}` : ''}.`;

  // Material views, for files that have material maps.
  const m = scene.materials;
  for (const id of MATERIAL_MODES) {
    $(`mode-${id}`).disabled = !m;
    document.querySelector(`label[for="mode-${id}"]`).hidden = !m;
  }
  state.highlight = -1;
  // Vegetation is the classic abundance map; otherwise the first endmember.
  state.endmember = m ? Math.max(0, m.endmembers.findIndex((e) => e.name === 'leaf')) : 0;
  updateMaterialList();
  updateEndmembers();
  if (!m && MATERIAL_MODES.includes(state.mode)) {
    state.mode = 'true';
    $('mode-true').checked = true;
  }

  weights = channelWeights();
  renderer.setChannels(weights);
  applyColorBy();
  if (sameFrame) {
    resampleProbes();
  } else {
    state.probes = [];
    state.nextProbe = 1;
    addPresetProbes();
  }
  updateReadout();
  updateProbeViews();
  setStatus('');
  requestDraw();
}

function paintSpectralTracks(lo, hi) {
  const stops = [];
  for (let nm = lo; nm <= hi; nm += 10) stops.push(`${cssColor(spectralColor(nm))} ${(((nm - lo) / (hi - lo)) * 100).toFixed(1)}%`);
  const track = `linear-gradient(to right, ${stops.join(', ')})`;
  for (const input of document.querySelectorAll('input.spectral')) input.style.setProperty('--track', track);
  const ticks = document.querySelector('#ctl-wavelength .ticks');
  ticks.replaceChildren();
  ticks.style.position = 'relative';
  ticks.style.height = '12px';
  for (let nm = Math.ceil(lo / 100) * 100; nm <= hi; nm += 100) {
    const s = textSpan(String(nm));
    const at = (nm - lo) / (hi - lo);
    s.style.cssText = `position:absolute;left:${(at * 100).toFixed(2)}%;transform:translateX(${at < 0.02 ? 0 : at > 0.98 ? -100 : -50}%)`;
    ticks.appendChild(s);
  }
}

async function loadSample(id) {
  const sample = SAMPLES.find((s) => s.id === id) || SAMPLES[0];
  stopAnimation();
  setStatus(`Loading ${sample.label.toLowerCase()}…`);
  try {
    const scene = await fetchSplat(sample.file, (loaded, total) => {
      setStatus(`Loading ${sample.label.toLowerCase()}… ${(loaded / 1048576).toFixed(1)}${total ? ` of ${(total / 1048576).toFixed(1)}` : ''} MB`);
    });
    state.sceneId = sample.id;
    $('scene-select').value = sample.id;
    useScene(scene, { keepView: true });
    try { history.replaceState(null, '', `#${sample.id}`); } catch { /* not allowed in some frames */ }
  } catch (e) {
    const local = location.protocol === 'file:';
    setStatus(local
      ? 'Browsers block loading files from a page opened straight from disk. Serve this folder (python3 -m http.server) or open a .lsplat file.'
      : `Couldn't load the sample: ${e.message}`);
    document.documentElement.dataset.error = e.message;
  }
}

async function openFile(file) {
  stopAnimation();
  setStatus(`Reading ${file.name}…`);
  try {
    const scene = await decodeSplat(await file.arrayBuffer());
    const select = $('scene-select');
    let option = select.querySelector('option[value="file"]');
    if (!option) {
      option = document.createElement('option');
      option.value = 'file';
      select.appendChild(option);
    }
    option.textContent = file.name;
    select.value = 'file';
    state.sceneId = 'file';
    useScene(scene, { keepView: false });
  } catch (e) {
    setStatus(`Couldn't open ${file.name}: ${e.message}`);
    if (state.scene) setTimeout(() => setStatus(''), 4000);
  }
}

// --- Controls -------------------------------------------------------------------

function setMode(mode) {
  state.mode = mode;
  $(`mode-${mode}`).checked = true;
  updateChannels();
}

function pickWavelength(nm) {
  state.wavelength = Math.round(nm);
  if (state.mode !== 'band') setMode('band');
  else updateChannels();
}

function playLabel() {
  return `Sweep ${$('wavelength').min} to ${$('wavelength').max} nm`;
}

function stopAnimation() {
  if (!animation) return;
  cancelAnimationFrame(animation.frame);
  animation = null;
  $('play').textContent = playLabel();
}

function toggleAnimation() {
  if (animation) {
    stopAnimation();
    return;
  }
  if (state.mode !== 'band') setMode('band');
  const lo = Number($('wavelength').min), hi = Number($('wavelength').max);
  const period = 9000;  // ms for up and back
  const phase0 = (state.wavelength - lo) / (hi - lo) / 2;
  const t0 = performance.now();
  const step = (now) => {
    const ph = (phase0 + (now - t0) / period) % 1;
    const tri = ph < 0.5 ? 2 * ph : 2 - 2 * ph;
    state.wavelength = lo + tri * (hi - lo);
    updateChannels();
    animation.frame = requestAnimationFrame(step);
  };
  animation = { frame: requestAnimationFrame(step) };
  $('play').textContent = 'Stop';
}

function bindControls() {
  for (const input of document.querySelectorAll('input[name="mode"]'))
    input.addEventListener('change', () => {
      stopAnimation();
      setMode(input.value);
    });
  const range = (id, key, parse = Number) => {
    $(id).addEventListener('input', (e) => {
      if (id === 'wavelength') stopAnimation();
      state[key] = parse(e.target.value);
      updateChannels();
    });
  };
  range('wavelength', 'wavelength');
  range('bandwidth', 'bandwidth');
  range('index-a', 'indexA');
  range('index-b', 'indexB');
  $('exposure').addEventListener('input', (e) => {
    state.exposure = Number(e.target.value);
    updateReadout();
    updateLegend();
    requestDraw();
  });
  $('play').addEventListener('click', toggleAnimation);
  $('reset-view').addEventListener('click', () => {
    camera.reset(state.scene.view);
    requestDraw();
  });
  $('show-sweeps').addEventListener('change', (e) => {
    state.showSweeps = e.target.checked;
    // The fans start 15 cm up; back off far enough to see them.
    if (state.showSweeps && camera.distance < 0.42) camera.dolly(0.42 / camera.distance);
    requestDraw();
  });
  for (const input of document.querySelectorAll('input[name="group"]'))
    input.addEventListener('change', () => {
      state.group = input.value;
      state.highlight = -1;
      applyColorBy();
      updateMaterialList();
      updateReadout();
      updateLegend();
      requestDraw();
    });
  $('show-all').addEventListener('click', () => setHighlight(-1));
  $('endmember').addEventListener('change', (e) => {
    state.endmember = Number(e.target.value);
    applyColorBy();
    updateReadout();
    updateLegend();
    requestDraw();
  });
  $('clear-probes').addEventListener('click', () => {
    state.probes = [];
    updateProbeViews();
    requestDraw();
  });
  const select = $('scene-select');
  for (const s of SAMPLES) {
    const o = document.createElement('option');
    o.value = s.id;
    o.textContent = s.label;
    select.appendChild(o);
  }
  select.addEventListener('change', () => {
    if (select.value !== 'file') loadSample(select.value);
  });
  $('open-file').addEventListener('click', () => $('file-input').click());
  $('file-input').addEventListener('change', (e) => {
    if (e.target.files[0]) openFile(e.target.files[0]);
    e.target.value = '';
  });
  stage.addEventListener('dragover', (e) => {
    e.preventDefault();
    $('drop').hidden = false;
  });
  stage.addEventListener('dragleave', (e) => {
    if (!stage.contains(e.relatedTarget)) $('drop').hidden = true;
  });
  stage.addEventListener('drop', (e) => {
    e.preventDefault();
    $('drop').hidden = true;
    const file = e.dataTransfer.files[0];
    if (file) openFile(file);
  });
}

// --- Orbiting -------------------------------------------------------------------

function bindView() {
  const pointers = new Map();
  let gesture = null;

  const pinchState = () => {
    const [a, b] = [...pointers.values()];
    return { dist: Math.hypot(a.x - b.x, a.y - b.y), mx: (a.x + b.x) / 2, my: (a.y + b.y) / 2 };
  };

  view.addEventListener('contextmenu', (e) => e.preventDefault());
  view.addEventListener('pointerdown', (e) => {
    if (!camera) return;
    view.setPointerCapture(e.pointerId);
    pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
    if (pointers.size === 1) {
      const pan = e.button === 1 || e.button === 2 || e.shiftKey || e.ctrlKey || e.metaKey;
      gesture = { kind: pan ? 'pan' : 'orbit', moved: 0 };
    } else if (pointers.size === 2) {
      gesture = { kind: 'pinch', moved: Infinity, ...pinchState() };
    }
  });
  view.addEventListener('pointermove', (e) => {
    const prev = pointers.get(e.pointerId);
    if (!prev || !gesture) return;
    const dx = e.clientX - prev.x, dy = e.clientY - prev.y;
    prev.x = e.clientX;
    prev.y = e.clientY;
    const r = pixelRatio();
    if (gesture.kind === 'pinch' && pointers.size === 2) {
      const now = pinchState();
      if (now.dist > 0) camera.dolly(gesture.dist / now.dist);
      camera.pan((now.mx - gesture.mx) * r, (now.my - gesture.my) * r, frame().f);
      Object.assign(gesture, now);
    } else if (gesture.kind === 'orbit') {
      gesture.moved += Math.abs(dx) + Math.abs(dy);
      camera.orbit(-dx * 0.008, dy * 0.008);
    } else if (gesture.kind === 'pan') {
      gesture.moved += Math.abs(dx) + Math.abs(dy);
      camera.pan(dx * r, dy * r, frame().f);
    }
    requestDraw();
  });
  const end = (e) => {
    if (!pointers.has(e.pointerId)) return;
    if (e.type === 'pointerup' && gesture && pointers.size === 1 && gesture.moved < 5 && state.scene)
      sampleAt(e.clientX, e.clientY);
    pointers.delete(e.pointerId);
    // Lifting one finger of a pinch leaves an orbit that never counts as a tap.
    gesture = pointers.size ? { kind: 'orbit', moved: Infinity } : null;
  };
  view.addEventListener('pointerup', end);
  view.addEventListener('pointercancel', end);
  view.addEventListener('wheel', (e) => {
    if (!camera) return;
    e.preventDefault();
    camera.dolly(Math.exp(e.deltaY * (e.deltaMode === 1 ? 0.05 : 0.0015)));
    requestDraw();
  }, { passive: false });
  view.addEventListener('keydown', (e) => {
    if (!camera) return;
    const step = 5 * Math.PI / 180, f = frame().f, r = pixelRatio();
    const moves = {
      ArrowLeft: () => (e.shiftKey ? camera.pan(-20 * r, 0, f) : camera.orbit(step, 0)),
      ArrowRight: () => (e.shiftKey ? camera.pan(20 * r, 0, f) : camera.orbit(-step, 0)),
      ArrowUp: () => (e.shiftKey ? camera.pan(0, -20 * r, f) : camera.orbit(0, -step)),
      ArrowDown: () => (e.shiftKey ? camera.pan(0, 20 * r, f) : camera.orbit(0, step)),
      '+': () => camera.dolly(0.9),
      '=': () => camera.dolly(0.9),
      '-': () => camera.dolly(1 / 0.9),
    };
    if (!moves[e.key]) return;
    e.preventDefault();
    moves[e.key]();
    requestDraw();
  });
}

// --- Start ----------------------------------------------------------------------

function resize() {
  const r = pixelRatio();
  const w = Math.max(1, Math.round(stage.clientWidth * r));
  const h = Math.max(1, Math.round(stage.clientHeight * r));
  if (view.width !== w || view.height !== h) {
    view.width = overlay.width = w;
    view.height = overlay.height = h;
  }
  requestDraw();
}

function start() {
  if (matchMedia('(pointer: coarse)').matches)
    $('hint').textContent = 'Drag to turn · Pinch to zoom · Tap to sample a spectrum';
  bindControls();
  updateProbeViews();
  try {
    renderer = new SplatRenderer(view);
  } catch (e) {
    setStatus(`This viewer needs WebGL2, which isn't available here (${e.message}). Try a current Chrome, Edge, Firefox or Safari.`);
    document.documentElement.dataset.error = e.message;
    return;
  }
  view.addEventListener('webglcontextlost', (e) => {
    e.preventDefault();
    setStatus('The graphics card dropped this page. Reload to continue.');
  });
  bindView();
  new ResizeObserver(() => {
    resize();
    chart.render();
  }).observe(stage);
  resize();
  const wanted = location.hash.slice(1);
  loadSample(SAMPLES.some((s) => s.id === wanted) ? wanted : SAMPLES[0].id).then(() => {
    if (state.scene) document.documentElement.dataset.ready = 'true';
  });
}

window.viewer = {
  state, get camera() { return camera; }, get renderer() { return renderer; }, frame, setMode, draw, setHighlight, openFile,
};
start();
