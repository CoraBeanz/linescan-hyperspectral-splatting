// Runs the viewer's checks in headless Chromium:
//
//   npm install && npx playwright install chromium   (once)
//   npm test                                         (or: node test/run.mjs)
//   node test/run.mjs --screenshots shots            (also saves screenshots)
//
// It serves viewer/ on a local port, runs test/index.html (the comparisons
// with the C++ renderer), then drives the page itself: loading, the modes,
// sampling, orbiting, switching scenes, the material views, and a
// phone-sized layout.

import { createServer } from 'node:http';
import { mkdir, readFile, stat } from 'node:fs/promises';
import { dirname, extname, join, normalize, resolve, sep } from 'node:path';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const args = process.argv.slice(2);
const shotsAt = args.indexOf('--screenshots');
const shots = shotsAt >= 0 ? resolve(args[shotsAt + 1] || 'screenshots') : null;

const TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.json': 'application/json',
  '.css': 'text/css',
  '.png': 'image/png',
  '.svg': 'image/svg+xml',
  '.lsplat': 'application/octet-stream',
  '.gz': 'application/gzip',
};

function serve() {
  const server = createServer(async (req, res) => {
    const path = decodeURIComponent(new URL(req.url, 'http://localhost').pathname);
    const file = normalize(join(root, path.endsWith('/') ? `${path}index.html` : path));
    if (file !== root && !file.startsWith(root + sep)) {
      res.writeHead(403).end();
      return;
    }
    try {
      if (!(await stat(file)).isFile()) throw new Error('not a file');
      const body = await readFile(file);
      res.writeHead(200, {
        'content-type': TYPES[extname(file)] || 'application/octet-stream',
        'content-length': body.length,
        'cache-control': 'no-store',
      });
      res.end(body);
    } catch {
      res.writeHead(404).end();
    }
  });
  return new Promise((ok) => server.listen(0, '127.0.0.1', () => ok(server)));
}

let failures = 0;
function report(ok, name, detail = '') {
  if (!ok) failures++;
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? `: ${detail}` : ''}`);
}

// Runs one check against the page; a thrown error fails it.
async function step(name, fn) {
  try {
    const detail = await fn();
    report(true, name, detail || '');
  } catch (e) {
    report(false, name, e.message.split('\n')[0]);
  }
}

function expect(ok, message) {
  if (!ok) throw new Error(message);
}

// A page that records its errors. Google Fonts are blocked so the checks
// don't depend on the network; the page falls back to system fonts.
async function openPage(browser, base, path, viewport) {
  const page = await browser.newPage({ viewport });
  const errors = [];
  page.on('pageerror', (e) => errors.push(e.message));
  page.on('console', (m) => {
    if (m.type() === 'error' && !/fonts\.(googleapis|gstatic)\.com/.test(m.location().url || '')) errors.push(m.text());
  });
  await page.route(/fonts\.(googleapis|gstatic)\.com/, (r) => r.abort());
  await page.goto(base + path);
  return { page, errors };
}

async function ready(page) {
  await page.waitForFunction(() => document.documentElement.dataset.ready || document.documentElement.dataset.error,
    null, { timeout: 60000 });
  const error = await page.evaluate(() => document.documentElement.dataset.error);
  expect(!error, `the page reported: ${error}`);
}

// Draws now rather than on the next animation frame, then reads a value.
const settle = (page) => page.evaluate(() => new Promise((ok) => requestAnimationFrame(() => requestAnimationFrame(ok))));

async function screenshot(page, name) {
  if (!shots) return;
  await mkdir(shots, { recursive: true });
  await settle(page);
  await page.screenshot({ path: join(shots, `${name}.png`) });
}

// The middle of the image as shown, through a 2D canvas so the WebGL buffer is read as drawn.
const centrePixel = (page) => page.evaluate(() => {
  const v = document.getElementById('view');
  const c = document.createElement('canvas');
  c.width = v.width;
  c.height = v.height;
  const ctx = c.getContext('2d');
  window.viewer.draw();
  ctx.drawImage(v, 0, 0);
  return Array.from(ctx.getImageData(v.width >> 1, v.height >> 1, 1, 1).data.slice(0, 3)).join(',');
});

// How many pixels of the image are strongly colored (not gray).
const colorfulPixels = (page) => page.evaluate(() => {
  const v = document.getElementById('view');
  const c = document.createElement('canvas');
  c.width = v.width;
  c.height = v.height;
  const ctx = c.getContext('2d');
  window.viewer.draw();
  ctx.drawImage(v, 0, 0);
  const d = ctx.getImageData(0, 0, c.width, c.height).data;
  let n = 0;
  for (let i = 0; i < d.length; i += 4) if (Math.max(d[i], d[i + 1], d[i + 2]) - Math.min(d[i], d[i + 1], d[i + 2]) > 40) n++;
  return n;
});

// Where a sampled point is on the screen, in page coordinates.
const probeOnScreen = (page, label) => page.evaluate(async (label) => {
  const { projectPoint } = await import(new URL('js/camera.js', location.href).href);
  const p = window.viewer.state.probes.find((q) => q.label === label);
  const cam = window.viewer.frame();
  const [u, v] = projectPoint(cam, p.point);
  const r = document.getElementById('view').getBoundingClientRect();
  return { x: r.left + (u * r.width) / cam.width, y: r.top + (v * r.height) / cam.height };
}, label);

// The trained sample without its material maps, as splat_export alone writes it.
async function sampleWithoutMaterials() {
  const bytes = await readFile(join(root, 'data', 'trained-16-sweeps.lsplat'));
  const length = bytes.readUInt32LE(8);
  const header = JSON.parse(bytes.subarray(12, 12 + length).toString('utf8'));
  delete header.materials;
  for (const name of ['material_label', 'material_angle', 'cluster', 'abundances']) delete header.blocks[name];
  const end = Math.max(...Object.values(header.blocks).map((b) => b.offset + b.bytes));
  let h = Buffer.from(JSON.stringify(header), 'utf8');
  h = Buffer.concat([h, Buffer.alloc((4 - (h.length % 4)) % 4, ' ')]);
  const top = Buffer.from(bytes.subarray(0, 12));
  top.writeUInt32LE(h.length, 8);
  return Buffer.concat([top, h, bytes.subarray(12 + length, 12 + length + end)]);
}

const server = await serve();
const base = `http://127.0.0.1:${server.address().port}/`;
const browser = await chromium.launch();
try {
  // The comparisons with the C++ renderer.
  {
    const { page, errors } = await openPage(browser, base, 'test/index.html', { width: 900, height: 700 });
    await page.waitForFunction(() => window.testResults?.done, null, { timeout: 120000 });
    const { results } = await page.evaluate(() => window.testResults);
    for (const r of results) report(r.ok, r.name, r.detail);
    report(errors.length === 0, 'the checks page ran without errors', errors.join(' | '));
    await page.close();
  }

  // The page itself, at a desktop size.
  {
    const { page, errors } = await openPage(browser, base, 'index.html', { width: 1280, height: 800 });
    await step('the page loads the trained sample', async () => {
      await ready(page);
      const stats = await page.textContent('#stats');
      expect(stats.includes('12,943 Gaussians'), `stats read "${stats}"`);
      expect(await page.isHidden('#status'), 'the loading message is still up');
      return stats;
    });
    await step('it samples three points of the synthetic scene to start', async () => {
      const names = await page.$$eval('#legend li .name', (els) => els.map((e) => e.lastChild.textContent));
      expect(names.length === 3, `the legend has ${names.length} entries: ${names.join(', ')}`);
      const lines = await page.$$eval('#chart polyline', (els) => els.length);
      expect(lines === 3, `the chart has ${lines} lines`);
      return names.join(', ');
    });
    await screenshot(page, 'desktop-true-color');

    await step('the modes change the image and the readout', async () => {
      const seen = [];
      const pixels = new Set();
      for (const [id, kicker, main] of [['band', 'Wavelength', '800nm'], ['cir', 'Showing', 'Color infrared'],
        ['index', 'Band index', '(A − B) / (A + B)'], ['true', 'Showing', 'True color']]) {
        await page.click(`label[for="mode-${id}"]`);
        await settle(page);
        const k = await page.textContent('#readout-kicker');
        const m = (await page.textContent('#readout-main')).replace(/\s+/g, '');
        expect(k === kicker && m === main.replace(/\s+/g, ''), `${id}: readout "${k}" / "${m}"`);
        pixels.add(await centrePixel(page));
        seen.push(id);
        if (id !== 'true') await screenshot(page, `desktop-${id}`);
      }
      expect(pixels.size >= 3, `the image centre looked the same in several modes: ${[...pixels].join(' / ')}`);
      return seen.join(', ');
    });

    await step('the wavelength slider and the chart pick wavelengths', async () => {
      await page.click('label[for="mode-band"]');
      await page.fill('#wavelength', '650');
      await page.dispatchEvent('#wavelength', 'input');
      expect((await page.textContent('#wavelength-out')) === '650 nm', 'the slider did not set 650 nm');
      const chart = page.locator('#chart svg');
      await chart.scrollIntoViewIfNeeded();
      const box = await chart.boundingBox();
      await page.mouse.click(box.x + box.width * 0.75, box.y + box.height * 0.4);
      const nm = await page.evaluate(() => window.viewer.state.wavelength);
      expect(nm > 800 && nm < 900, `clicking the chart at three quarters picked ${nm} nm`);
      return `picked ${nm} nm from the chart`;
    });

    await step('clicking the scene samples a spectrum there', async () => {
      const box = await page.locator('#view').boundingBox();
      await page.mouse.click(box.x + box.width / 2, box.y + box.height / 2);
      const n = await page.$$eval('#legend li', (els) => els.length);
      expect(n === 4, `the legend has ${n} entries after a click`);
      const rows = await page.$$eval('#values tbody tr', (els) => els.length);
      expect(rows === 46, `the table has ${rows} rows`);
      return 'Point 4 added';
    });

    await step('dragging turns the view and the wheel zooms', async () => {
      const before = await page.evaluate(() => ({ az: window.viewer.camera.azimuth, el: window.viewer.camera.elevation, d: window.viewer.camera.distance }));
      const box = await page.locator('#view').boundingBox();
      const x = box.x + box.width * 0.5, y = box.y + box.height * 0.5;
      await page.mouse.move(x, y);
      await page.mouse.down();
      await page.mouse.move(x + 60, y + 30, { steps: 6 });
      await page.mouse.up();
      await page.mouse.wheel(0, 200);
      const after = await page.evaluate(() => ({ az: window.viewer.camera.azimuth, el: window.viewer.camera.elevation, d: window.viewer.camera.distance, n: window.viewer.state.probes.length }));
      // Dragging right swings the camera to its left, so the scene follows the pointer.
      expect(after.az < before.az, 'dragging right did not turn the view');
      expect(after.el > before.el, 'dragging down did not raise the view');
      expect(after.d > before.d, 'scrolling down did not zoom out');
      expect(after.n === 4, 'a drag added a sample');
      return `azimuth ${(((after.az - before.az) * 180) / Math.PI).toFixed(0)} deg, distance x${(after.d / before.d).toFixed(2)}`;
    });
    await screenshot(page, 'desktop-turned');

    await step('switching to the ground truth keeps the view and the sampled points', async () => {
      await page.click('#reset-view');
      await page.selectOption('#scene-select', 'truth');
      await page.waitForFunction(() => document.getElementById('stats').textContent.includes('9,741'), null, { timeout: 30000 });
      const n = await page.$$eval('#legend li', (els) => els.length);
      expect(n === 4, `the legend has ${n} entries after switching`);
      return await page.textContent('#stats');
    });
    await page.click('label[for="mode-cir"]');
    await screenshot(page, 'desktop-truth-cir');

    await step('the scan sweeps overlay draws', async () => {
      await page.check('#show-sweeps');
      await settle(page);
      const lit = await page.evaluate(() => {
        const o = document.getElementById('overlay');
        const d = o.getContext('2d').getImageData(0, 0, o.width, o.height).data;
        let n = 0;
        for (let i = 3; i < d.length; i += 4) if (d[i] > 0) n++;
        return n;
      });
      expect(lit > 1000, `only ${lit} overlay pixels drawn`);
      return `${lit} pixels`;
    });
    await screenshot(page, 'desktop-sweeps');

    await step('the sweep animation runs and stops', async () => {
      await page.uncheck('#show-sweeps');
      await page.click('#play');
      const a = await page.evaluate(() => window.viewer.state.wavelength);
      await page.waitForTimeout(400);
      const b = await page.evaluate(() => window.viewer.state.wavelength);
      await page.click('#play');
      expect(a !== b, 'the wavelength did not move');
      expect((await page.textContent('#play')).startsWith('Sweep'), 'the button did not reset');
      return `${a.toFixed(0)} to ${b.toFixed(0)} nm`;
    });

    await step('the Materials view colors each Gaussian by its library match', async () => {
      await page.selectOption('#scene-select', 'trained');
      await page.waitForFunction(() => document.getElementById('stats').textContent.includes('12,943'), null, { timeout: 30000 });
      const before = await colorfulPixels(page);
      await page.click('label[for="mode-materials"]');
      await settle(page);
      expect((await page.textContent('#readout-kicker')) === 'Materials', 'the readout does not say Materials');
      expect(await page.isVisible('#ctl-materials'), 'the material list is hidden');
      expect(await page.isHidden('#ctl-exposure'), 'the exposure control is still up');
      const rows = await page.$$eval('#material-list button', (els) => els.map((e) => e.querySelector('.name').textContent));
      expect(rows.length === 11, `the list has ${rows.length} materials`);
      expect(rows[0] === 'white paper', `the largest material is ${rows[0]}`);
      const unmatched = await page.textContent('#material-list .unmatched');
      expect(/^No match: \d+% of Gaussians/.test(unmatched), `the list ends "${unmatched}"`);
      const after = await colorfulPixels(page);
      expect(after > 2 * before, `${after} colorful pixels, ${before} in true color`);
      return `${rows.length} materials, ${(await page.textContent('#material-note')).match(/\d+% of the opening view's pixels/)?.[0]}`;
    });
    await screenshot(page, 'desktop-materials');

    await step('picking a material in the list shows it alone', async () => {
      const all = await colorfulPixels(page);
      await page.click('#material-list button:has(.name:text-is("leaf"))');
      await settle(page);
      const st = await page.evaluate(() => ({ h: window.viewer.state.highlight, name: window.viewer.state.scene.materials.classes[window.viewer.state.highlight]?.name }));
      expect(st.name === 'leaf', `picked ${st.name}`);
      expect((await page.textContent('#readout-main')) === 'leaf', 'the readout does not name the leaf');
      const pressed = await page.$$eval('#material-list button[aria-pressed="true"]', (els) => els.length);
      expect(pressed === 1, `${pressed} buttons pressed`);
      expect(await page.isVisible('#show-all'), 'Show all is hidden');
      expect((await page.$$eval('#chart .chart-ref', (els) => els.length)) === 1, 'the chart lacks the library spectrum');
      const alone = await colorfulPixels(page);
      expect(alone < all / 2 && alone > 100, `${alone} colorful pixels with the leaf alone, ${all} with all`);
      await screenshot(page, 'desktop-materials-leaf');
      await page.click('#show-all');
      expect((await page.evaluate(() => window.viewer.state.highlight)) === -1, 'Show all did not clear the pick');
      return `${alone} of ${all} colorful pixels left`;
    });

    await step('clicking the scene picks the material there', async () => {
      const at = await probeOnScreen(page, 'Orange box');
      const next = await page.evaluate(() => window.viewer.state.nextProbe);
      await page.mouse.click(at.x, at.y);
      await settle(page);
      const name = await page.textContent('#readout-main');
      expect(name === 'orange plastic', `clicking the orange box picked "${name}"`);
      const legend = await page.$$eval('#legend li', (els) => els.map((e) => e.textContent));
      expect(legend.at(-1).includes(`Point ${next}`), `the legend ends "${legend.at(-1)}", not with Point ${next}`);
      expect(/orange plastic \d+%/.test(legend.at(-1)), `the new point reads "${legend.at(-1)}"`);
      await page.click('#show-all');
      return `Point ${next} reads ${legend.at(-1).match(/orange plastic \d+%/)[0]}`;
    });

    await step('clusters color by k-means and name their nearest material', async () => {
      await page.click('label[for="group-cluster"]');
      await settle(page);
      expect((await page.textContent('#readout-kicker')) === 'Clusters', 'the readout does not say Clusters');
      const subs = await page.$$eval('#material-list button .sub', (els) => els.map((e) => e.textContent));
      expect(subs.length === 12, `the list has ${subs.length} clusters`);
      expect(subs[0].startsWith('Nearest: white paper'), `the largest cluster reads "${subs[0]}"`);
      await screenshot(page, 'desktop-clusters');
      await page.click('label[for="group-label"]');
      return subs.slice(0, 3).join('; ');
    });

    await step('the Abundance view maps one endmember at a time', async () => {
      await page.click('label[for="mode-abundance"]');
      await settle(page);
      expect((await page.textContent('#readout-kicker')) === 'Abundance', 'the readout does not say Abundance');
      expect((await page.textContent('#readout-main')) === 'leaf', 'leaf is not the first endmember shown');
      expect(await page.isVisible('#ctl-endmember'), 'the endmember picker is hidden');
      const leaf = await centrePixel(page);
      const values = await page.$$eval('#legend li .value', (els) => els.map((e) => e.textContent));
      expect(values.every((v) => /^(\d\.\d\d|–)$/.test(v)), `the legend reads ${values.join(', ')}`);
      await screenshot(page, 'desktop-abundance');
      await page.selectOption('#endmember', { label: 'white paper' });
      await settle(page);
      expect((await page.textContent('#readout-main')) === 'white paper', 'the endmember did not change');
      const paper = await centrePixel(page);
      expect(paper !== leaf, `the image centre stayed ${leaf}`);
      return `centre ${leaf} for leaf, ${paper} for white paper`;
    });

    await step('a file without material maps hides the material views', async () => {
      await page.setInputFiles('#file-input', {
        name: 'no-materials.lsplat', mimeType: 'application/octet-stream', buffer: await sampleWithoutMaterials(),
      });
      await page.waitForFunction(() => window.viewer.state.scene?.materials === null, null, { timeout: 30000 });
      await settle(page);
      expect(await page.isHidden('label[for="mode-materials"]'), 'the Materials mode is still offered');
      expect(await page.isHidden('label[for="mode-abundance"]'), 'the Abundance mode is still offered');
      const mode = await page.evaluate(() => window.viewer.state.mode);
      expect(mode === 'true', `the view stayed in ${mode}`);
      expect(await page.isHidden('#ctl-endmember'), 'the endmember picker is still up');
      return await page.textContent('#stats');
    });

    report(errors.length === 0, 'the page ran without errors', errors.join(' | '));
    await page.close();
  }

  // A phone-sized screen.
  {
    const { page, errors } = await openPage(browser, base, 'index.html#truth', { width: 390, height: 844 });
    await step('the page fits a phone screen', async () => {
      await ready(page);
      const stats = await page.textContent('#stats');
      expect(stats.includes('9,741'), `the #truth link loaded "${stats}"`);
      const overflow = await page.evaluate(() => document.documentElement.scrollWidth - window.innerWidth);
      expect(overflow <= 0, `the page scrolls sideways by ${overflow}px`);
      return stats;
    });
    await screenshot(page, 'phone');
    await page.evaluate(() => window.scrollTo(0, document.querySelector('.panel').offsetTop));
    await screenshot(page, 'phone-panel');
    await step('the material list fits a phone screen', async () => {
      await page.click('label[for="mode-materials"]');
      await settle(page);
      const rows = await page.$$eval('#material-list button', (els) => els.length);
      expect(rows === 11, `the list has ${rows} materials`);
      const overflow = await page.evaluate(() => document.documentElement.scrollWidth - window.innerWidth);
      expect(overflow <= 0, `the page scrolls sideways by ${overflow}px`);
      return `${rows} materials`;
    });
    await page.evaluate(() => window.scrollTo(0, 0));
    await screenshot(page, 'phone-materials');
    report(errors.length === 0, 'the phone layout ran without errors', errors.join(' | '));
    await page.close();
  }
} finally {
  await browser.close();
  server.close();
}

console.log(failures ? `\n${failures} failed` : '\nAll passed');
process.exitCode = failures ? 1 : 0;
