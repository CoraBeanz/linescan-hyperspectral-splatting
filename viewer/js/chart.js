// The spectrum chart: the sampled points' spectra against wavelength, with
// the bands the current view uses marked, a crosshair readout, and a click
// to pick the wavelength shown.

import { cssColor, spectralColor } from './spectral.js';

const NS = 'http://www.w3.org/2000/svg';

function el(name, attrs = {}, parent) {
  const node = document.createElementNS(NS, name);
  for (const [k, v] of Object.entries(attrs)) node.setAttribute(k, v);
  if (parent) parent.appendChild(node);
  return node;
}

function niceStep(span) {
  const raw = span / 5;
  const pow = Math.pow(10, Math.floor(Math.log10(raw)));
  const m = raw / pow;
  return (m < 1.5 ? 1 : m < 3.5 ? 2 : m < 7.5 ? 5 : 10) * pow;
}

export class SpectrumChart {
  // root holds the chart; onPick(nm) is called when the plot is clicked.
  constructor(root, { onPick, height = 200 } = {}) {
    this.root = root;
    this.onPick = onPick;
    this.height = height;
    this.wl = [];
    this.probes = [];
    this.markers = [];
    this.svg = el('svg', { class: 'chart-svg', role: 'img' }, root);
    this.tip = document.createElement('div');
    this.tip.className = 'chart-tip';
    this.tip.hidden = true;
    root.appendChild(this.tip);
    this.svg.addEventListener('pointermove', (e) => this.hover(e));
    this.svg.addEventListener('pointerleave', () => this.hover(null));
    this.svg.addEventListener('click', (e) => {
      const nm = this.nmAt(e);
      if (nm !== null && this.onPick) this.onPick(nm);
    });
  }

  setWavelengths(wl) { this.wl = Array.from(wl); }
  // probes: [{color, label, spectrum}]
  setProbes(probes) { this.probes = probes; }
  // markers: [{kind: 'line', nm, label} | {kind: 'band', lo, hi, label}]
  setMarkers(markers) { this.markers = markers; }

  layout() {
    const width = Math.max(240, this.root.clientWidth);
    const m = { top: 18, right: 16, bottom: 42, left: 38 };
    const lo = this.wl[0] ?? 500, hi = this.wl[this.wl.length - 1] ?? 950;
    let vmin = 0, vmax = 1;
    for (const p of this.probes)
      for (const v of p.spectrum) {
        vmin = Math.min(vmin, v);
        vmax = Math.max(vmax, v);
      }
    // Noise can take a trained spectrum a little below zero; don't give
    // that a grid line of its own.
    if (vmin > -0.02) vmin = 0;
    const step = niceStep(vmax - vmin);
    vmin = Math.floor(vmin / step + 1e-9) * step;
    vmax = Math.ceil(vmax / step - 1e-9) * step;
    const x = (nm) => m.left + ((nm - lo) / (hi - lo)) * (width - m.left - m.right);
    const y = (v) => m.top + ((vmax - v) / (vmax - vmin)) * (this.height - m.top - m.bottom);
    return { width, height: this.height, m, lo, hi, vmin, vmax, step, x, y };
  }

  render() {
    const L = (this.L = this.layout());
    const { width, height, m, lo, hi, x, y } = L;
    const svg = this.svg;
    svg.replaceChildren();
    svg.setAttribute('viewBox', `0 0 ${width} ${height}`);
    svg.setAttribute('width', width);
    svg.setAttribute('height', height);
    const names = this.probes.map((p) => p.label).join(', ');
    svg.setAttribute('aria-label', this.probes.length
      ? `Reflectance from ${lo} to ${hi} nm at ${this.probes.length} sampled points: ${names}`
      : 'No spectra sampled yet');
    const bottom = height - m.bottom;

    // The spectrum under the axis: what each wavelength looks like, fading
    // to a stand-in color where the eye stops seeing.
    const defs = el('defs', {}, svg);
    const grad = el('linearGradient', { id: 'spectrum-strip', x1: 0, x2: 1, y1: 0, y2: 0 }, defs);
    for (let nm = lo; nm <= hi + 1e-6; nm += 10)
      el('stop', { offset: (nm - lo) / (hi - lo), 'stop-color': cssColor(spectralColor(nm)) }, grad);
    el('rect', { x: m.left, y: bottom + 6, width: width - m.left - m.right, height: 5, rx: 1, fill: 'url(#spectrum-strip)' }, svg);
    if (hi > 760) {
      const xv = x(Math.max(lo, 750));
      el('text', { x: xv + 4, y: bottom + 37, class: 'chart-note' }, svg).textContent = 'near infrared';
      el('line', { x1: xv, x2: xv, y1: bottom + 4, y2: bottom + 13, class: 'chart-axis' }, svg);
    }

    // Grid and axes.
    for (let v = L.vmin; v <= L.vmax + 1e-9; v += L.step) {
      const yy = y(v);
      el('line', { x1: m.left, x2: width - m.right, y1: yy, y2: yy, class: Math.abs(v) < 1e-9 ? 'chart-axis' : 'chart-grid' }, svg);
      const label = el('text', { x: m.left - 6, y: yy + 3.5, class: 'chart-tick', 'text-anchor': 'end' }, svg);
      label.textContent = +v.toFixed(3);
    }
    for (let nm = Math.ceil(lo / 50) * 50; nm <= hi; nm += 50) {
      const xx = x(nm);
      el('line', { x1: xx, x2: xx, y1: bottom, y2: bottom + 4, class: 'chart-axis' }, svg);
      if (nm % 100 === 0) {
        const t = el('text', { x: xx, y: bottom + 24, class: 'chart-tick', 'text-anchor': 'middle' }, svg);
        t.textContent = nm;
      }
    }
    el('text', { x: width - m.right, y: bottom + 24, class: 'chart-tick', 'text-anchor': 'end' }, svg).textContent = 'nm';
    el('text', { x: 2, y: 10, class: 'chart-note' }, svg).textContent = this.valueLabel || 'reflectance';

    // What the image shows, marked on the wavelength axis.
    for (const mk of this.markers) {
      if (mk.kind === 'band') {
        const x0 = x(Math.max(lo, mk.lo)), x1 = x(Math.min(hi, mk.hi));
        el('rect', { x: x0, y: m.top, width: Math.max(1, x1 - x0), height: bottom - m.top, class: 'chart-band' }, svg);
        el('text', { x: (x0 + x1) / 2, y: m.top - 6, class: 'chart-marker-label', 'text-anchor': 'middle' }, svg)
          .textContent = mk.label;
      } else {
        const xx = x(mk.nm);
        el('line', { x1: xx, x2: xx, y1: m.top, y2: bottom, class: 'chart-marker' }, svg);
        el('text', { x: xx, y: m.top - 6, class: 'chart-marker-label', 'text-anchor': 'middle' }, svg)
          .textContent = mk.label;
      }
    }

    if (!this.probes.length) {
      el('text', { x: (m.left + width - m.right) / 2, y: (m.top + bottom) / 2, class: 'chart-empty', 'text-anchor': 'middle' }, svg)
        .textContent = 'Click the scene to sample a spectrum';
    }

    // The spectra, with each one's number at its right end unless that would
    // collide with another; the legend below always names them.
    const ends = [];
    this.probes.forEach((p, n) => {
      const pts = this.wl.map((nm, b) => `${x(nm).toFixed(1)},${y(p.spectrum[b]).toFixed(1)}`).join(' ');
      el('polyline', { points: pts, class: 'chart-line', style: `stroke: ${p.color}` }, svg);
      ends.push({ y: y(p.spectrum[p.spectrum.length - 1]), n, color: p.color });
    });
    ends.sort((a, b) => a.y - b.y);
    ends.forEach((e, j) => {
      const crowded = (j > 0 && e.y - ends[j - 1].y < 13) || (j + 1 < ends.length && ends[j + 1].y - e.y < 13);
      el('circle', { cx: x(hi), cy: e.y, r: 4, class: 'chart-dot', style: `fill: ${e.color}` }, svg);
      if (!crowded)
        el('text', { x: x(hi) + 7, y: e.y + 3.5, class: 'chart-end' }, svg).textContent = String(this.probes[e.n].number);
    });

    this.cross = el('line', { x1: 0, x2: 0, y1: m.top, y2: bottom, class: 'chart-cross', visibility: 'hidden' }, svg);
    this.hitbox = el('rect', { x: m.left, y: 0, width: width - m.left - m.right, height: bottom + 14, fill: 'transparent', class: 'chart-hit' }, svg);
  }

  // The band centre nearest the pointer, or null off the plot.
  nmAt(e) {
    const L = this.L;
    if (!L || !this.wl.length) return null;
    const box = this.svg.getBoundingClientRect();
    const px = ((e.clientX - box.left) / box.width) * L.width;
    if (px < L.m.left - 4 || px > L.width - L.m.right + 4) return null;
    const nm = L.lo + ((px - L.m.left) / (L.width - L.m.left - L.m.right)) * (L.hi - L.lo);
    return Math.min(Math.max(nm, L.lo), L.hi);
  }

  hover(e) {
    const nm = e ? this.nmAt(e) : null;
    if (nm === null || !this.L) {
      if (this.cross) this.cross.setAttribute('visibility', 'hidden');
      this.tip.hidden = true;
      return;
    }
    let b = 0;
    for (let i = 1; i < this.wl.length; i++) if (Math.abs(this.wl[i] - nm) < Math.abs(this.wl[b] - nm)) b = i;
    const xx = this.L.x(this.wl[b]);
    this.cross.setAttribute('x1', xx);
    this.cross.setAttribute('x2', xx);
    this.cross.setAttribute('visibility', 'visible');

    this.tip.replaceChildren();
    const head = document.createElement('div');
    head.className = 'chart-tip-head';
    head.textContent = `${Math.round(this.wl[b])} nm`;
    this.tip.appendChild(head);
    for (const p of this.probes) {
      const row = document.createElement('div');
      row.className = 'chart-tip-row';
      const key = document.createElement('span');
      key.className = 'line-key';
      key.style.background = p.color;
      const value = document.createElement('strong');
      value.textContent = p.spectrum[b].toFixed(3);
      const label = document.createElement('span');
      label.textContent = p.label;
      row.append(key, value, label);
      this.tip.appendChild(row);
    }
    if (!this.probes.length) {
      const hint = document.createElement('div');
      hint.textContent = 'Click to show this wavelength';
      this.tip.appendChild(hint);
    }
    this.tip.hidden = false;
    const box = this.svg.getBoundingClientRect();
    const left = (xx / this.L.width) * box.width;
    const tipWidth = this.tip.offsetWidth;
    this.tip.style.left = `${Math.min(Math.max(left - tipWidth / 2, 0), box.width - tipWidth)}px`;
  }
}
