// Checks the assembled GitHub Pages site in headless Chromium, the way Pages
// serves it (static files, .lsplat and .gz as plain binary):
//
//   docs/site/build.sh _site
//   node docs/site/check.mjs _site      (needs the playwright package, as viewer/ uses)
//
// The front page's pictures load and its links resolve inside the site, it fits
// a phone's width, and the viewer opens its sample splat from the site's layout.
// The viewer's own workflow tests the page itself in depth.

import { createServer } from 'node:http';
import { readFile, stat } from 'node:fs/promises';
import { extname, join, normalize, resolve, sep } from 'node:path';
import { chromium } from 'playwright';

const root = resolve(process.argv[2] || '_site');
const TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.svg': 'image/svg+xml',
  '.jpg': 'image/jpeg',
  '.lsplat': 'application/octet-stream',
  '.gz': 'application/gzip',
};

function serve() {
  const server = createServer(async (req, res) => {
    const path = decodeURIComponent(new URL(req.url, 'http://localhost').pathname);
    const file = normalize(join(root, path.endsWith('/') ? `${path}index.html` : path));
    if (file !== root && !file.startsWith(root + sep)) return res.writeHead(403).end();
    try {
      if (!(await stat(file)).isFile()) throw new Error('not a file');
      const body = await readFile(file);
      res.writeHead(200, { 'content-type': TYPES[extname(file)] || 'application/octet-stream' });
      res.end(body);
    } catch {
      res.writeHead(404).end();
    }
  });
  return new Promise((ok) => server.listen(0, '127.0.0.1', () => ok(server)));
}

let failures = 0;
async function check(name, fn) {
  try {
    const detail = await fn();
    console.log(`PASS  ${name}${detail ? `: ${detail}` : ''}`);
  } catch (e) {
    failures++;
    console.log(`FAIL  ${name}: ${e.message.split('\n')[0]}`);
  }
}
function expect(ok, message) {
  if (!ok) throw new Error(message);
}

// A page that records its errors and the requests that failed. Google Fonts
// are blocked so the checks don't depend on the network.
async function open(browser, url, viewport) {
  const page = await browser.newPage({ viewport });
  const errors = [];
  page.on('pageerror', (e) => errors.push(e.message));
  page.on('console', (m) => {
    if (m.type() === 'error' && !/fonts\.(googleapis|gstatic)\.com/.test(m.location().url || '')) errors.push(m.text());
  });
  page.on('response', (r) => {
    if (r.status() >= 400) errors.push(`${r.status()} ${r.url()}`);
  });
  await page.route(/fonts\.(googleapis|gstatic)\.com/, (r) => r.abort());
  await page.goto(url, { waitUntil: 'load' });
  return { page, errors };
}

const server = await serve();
const base = `http://127.0.0.1:${server.address().port}/`;
const browser = await chromium.launch();
try {
  await check('front page: pictures load, links resolve', async () => {
    const { page, errors } = await open(browser, base, { width: 1280, height: 900 });
    const broken = await page.evaluate(() => [...document.images].filter((i) => !i.complete || !i.naturalWidth).map((i) => i.src));
    expect(!broken.length, `pictures that didn't load: ${broken.join(', ')}`);
    const local = await page.evaluate(() => [...document.links].map((a) => a.href).filter((h) => h.startsWith(location.origin)));
    for (const href of local) {
      const r = await page.request.get(href);
      expect(r.ok(), `${href} gives ${r.status()}`);
    }
    expect(!errors.length, errors.join('; '));
    return `${await page.evaluate(() => document.images.length)} pictures, ${local.length} links within the site`;
  });

  await check('front page: fits a phone', async () => {
    const { page } = await open(browser, base, { width: 390, height: 844 });
    const overflow = await page.evaluate(() => document.documentElement.scrollWidth - window.innerWidth);
    expect(overflow <= 0, `${overflow} px wider than the screen`);
  });

  await check('viewer: opens its sample splat', async () => {
    const { page, errors } = await open(browser, `${base}viewer/`, { width: 1280, height: 800 });
    await page.waitForFunction(() => document.documentElement.dataset.ready || document.documentElement.dataset.error,
      null, { timeout: 60000 });
    const error = await page.evaluate(() => document.documentElement.dataset.error);
    expect(!error, `the viewer reported: ${error}`);
    expect(!errors.length, errors.join('; '));
    return (await page.textContent('#stats')).trim().replace(/\s+/g, ' ');
  });

  await check('viewer: opens the gzipped ground truth', async () => {
    const { page, errors } = await open(browser, `${base}viewer/#truth`, { width: 1280, height: 800 });
    await page.waitForFunction(() => document.getElementById('stats')?.textContent.includes('9,741')
      || document.documentElement.dataset.error, null, { timeout: 60000 });
    const error = await page.evaluate(() => document.documentElement.dataset.error);
    expect(!error, `the viewer reported: ${error}`);
    expect(!errors.length, errors.join('; '));
  });
} finally {
  await browser.close();
  server.close();
}
console.log(failures ? `${failures} check(s) failed` : 'Site checks passed');
process.exit(failures ? 1 : 0);
