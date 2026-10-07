// Reads the files splat_export writes (format "linesplat-view", version 1).
//
//   bytes 0-3    "LSPV"
//   bytes 4-7    version, uint32 little-endian
//   bytes 8-11   length of the JSON header in bytes, a multiple of 4
//   header       counts, wavelengths, basis, background, default view, scan
//                sweeps, and "blocks": where each array starts, counted from
//                the end of the header
//   arrays       little-endian, each starting on a 4-byte boundary:
//                  means          float32 [N, 3]  metres
//                  log_scales     uint16  [N, 3]  lo + q * (hi - lo) / 65535
//                  rotations      int16   [N, 4]  unit quaternion (w, x, y, z) * 32767
//                  opacity        uint8   [N]     opacity * 255
//                  feature_range  float32 [N, 2]  offset and scale of the features
//                  features       uint8   [N, K]  feature = offset + scale * q / 255
//
// A spectrum is basis [B, K] times the features. A file may also be gzipped,
// and either form may be base64 text, for hosts that only serve text.

const MAGIC = 'LSPV';

// Fetches and decodes a file. onProgress(loadedBytes, totalBytes) is called
// as it downloads (totalBytes is 0 when the server doesn't say).
export async function fetchSplat(url, onProgress) {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`the server answered ${response.status} for ${url}`);
  const total = Number(response.headers.get('content-length')) || 0;
  if (!response.body || !onProgress) return decodeSplat(await response.arrayBuffer());
  const reader = response.body.getReader();
  const chunks = [];
  let loaded = 0;
  for (;;) {
    const { done, value } = await reader.read();
    if (done) break;
    chunks.push(value);
    loaded += value.length;
    onProgress(loaded, total);
  }
  const bytes = new Uint8Array(loaded);
  let at = 0;
  for (const c of chunks) {
    bytes.set(c, at);
    at += c.length;
  }
  return decodeSplat(bytes.buffer);
}

// How a file and a gzipped file start, in base64.
const BASE64_STARTS = ['TFNQ', 'H4sI'];

export async function decodeSplat(buffer) {
  let bytes = new Uint8Array(buffer);
  if (BASE64_STARTS.includes(String.fromCharCode(...bytes.subarray(0, 4)))) {
    const text = atob(new TextDecoder().decode(bytes).trim());
    bytes = new Uint8Array(text.length);
    for (let i = 0; i < text.length; i++) bytes[i] = text.charCodeAt(i);
  }
  if (bytes[0] === 0x1f && bytes[1] === 0x8b) {
    if (typeof DecompressionStream === 'undefined')
      throw new Error("this browser can't unzip files; gunzip it first and open the .lsplat");
    const stream = new Blob([bytes]).stream().pipeThrough(new DecompressionStream('gzip'));
    bytes = new Uint8Array(await new Response(stream).arrayBuffer());
  }
  return parseSplat(bytes.buffer);
}

const TYPES = {
  float32: Float32Array,
  uint16: Uint16Array,
  int16: Int16Array,
  uint8: Uint8Array,
};

export function parseSplat(buffer) {
  if (buffer.byteLength < 12) throw new Error('the file is too short to be a splat');
  const magic = String.fromCharCode(...new Uint8Array(buffer, 0, 4));
  if (magic !== MAGIC) throw new Error('this is not a splat_export file (it should start with LSPV)');
  const words = new DataView(buffer);
  const version = words.getUint32(4, true);
  if (version !== 1) throw new Error(`this viewer reads version 1 files; this one is version ${version}`);
  const headerBytes = words.getUint32(8, true);
  const header = JSON.parse(new TextDecoder().decode(new Uint8Array(buffer, 12, headerBytes)));
  if (header.format !== 'linesplat-view') throw new Error(`unknown format "${header.format}"`);
  const start = 12 + headerBytes;

  const N = header.count;
  const K = header.num_features;
  const B = header.num_bands;
  const block = (name, perItem) => {
    const b = header.blocks[name];
    const Type = TYPES[b?.type];
    if (!Type) throw new Error(`the file has no readable "${name}" array`);
    const length = b.bytes / Type.BYTES_PER_ELEMENT;
    if (length !== N * perItem || start + b.offset + b.bytes > buffer.byteLength)
      throw new Error(`the "${name}" array has the wrong size`);
    return new Type(buffer, start + b.offset, length);
  };

  const means = block('means', 3);
  const logScales = block('log_scales', 3);
  const rotations = block('rotations', 4);
  const opacityQ = block('opacity', 1);
  const range = block('feature_range', 2);
  const features = block('features', K);

  // Covariances R diag(s^2) R^T, as math.hpp builds them.
  const [lo, hi] = header.blocks.log_scales.range;
  const step = (hi - lo) / 65535;
  const cov = new Float32Array(6 * N);
  const opacity = new Float32Array(N);
  for (let i = 0; i < N; i++) {
    const sx = Math.exp(2 * (lo + logScales[3 * i] * step));
    const sy = Math.exp(2 * (lo + logScales[3 * i + 1] * step));
    const sz = Math.exp(2 * (lo + logScales[3 * i + 2] * step));
    let w = rotations[4 * i] / 32767;
    let x = rotations[4 * i + 1] / 32767;
    let y = rotations[4 * i + 2] / 32767;
    let z = rotations[4 * i + 3] / 32767;
    const n = Math.hypot(w, x, y, z) || 1;
    w /= n; x /= n; y /= n; z /= n;
    const r00 = 1 - 2 * (y * y + z * z), r01 = 2 * (x * y - w * z), r02 = 2 * (x * z + w * y);
    const r10 = 2 * (x * y + w * z), r11 = 1 - 2 * (x * x + z * z), r12 = 2 * (y * z - w * x);
    const r20 = 2 * (x * z - w * y), r21 = 2 * (y * z + w * x), r22 = 1 - 2 * (x * x + y * y);
    cov[6 * i] = r00 * r00 * sx + r01 * r01 * sy + r02 * r02 * sz;
    cov[6 * i + 1] = r00 * r10 * sx + r01 * r11 * sy + r02 * r12 * sz;
    cov[6 * i + 2] = r00 * r20 * sx + r01 * r21 * sy + r02 * r22 * sz;
    cov[6 * i + 3] = r10 * r10 * sx + r11 * r11 * sy + r12 * r12 * sz;
    cov[6 * i + 4] = r10 * r20 * sx + r11 * r21 * sy + r12 * r22 * sz;
    cov[6 * i + 5] = r20 * r20 * sx + r21 * r21 * sy + r22 * r22 * sz;
    opacity[i] = opacityQ[i] / 255;
  }

  const basis = new Float64Array(B * K);
  if (header.basis === 'identity') {
    if (B !== K) throw new Error('an identity basis needs as many features as bands');
    for (let b = 0; b < B; b++) basis[b * K + b] = 1;
  } else {
    header.basis.forEach((row, b) => row.forEach((v, k) => { basis[b * K + k] = v; }));
  }

  return {
    header,
    title: header.title || 'Untitled splat',
    count: N,
    numFeatures: K,
    numBands: B,
    wavelengths: Float64Array.from(header.wavelengths_nm),
    values: header.values || 'reflectance',
    means,
    cov,
    opacity,
    featureRange: range,
    features,
    basis,
    background: Float64Array.from(header.background),
    view: header.view,
    sweeps: header.sweeps || [],
    bytes: buffer.byteLength,
  };
}

// The K features of Gaussian i, decoded.
export function gaussianFeatures(scene, i, out = new Float64Array(scene.numFeatures)) {
  const K = scene.numFeatures;
  const off = scene.featureRange[2 * i];
  const scale = scene.featureRange[2 * i + 1] / 255;
  for (let k = 0; k < K; k++) out[k] = off + scale * scene.features[i * K + k];
  return out;
}
