// The WebGL2 renderer.
//
// Each Gaussian is drawn as a screen-space quad around its projected 2D
// Gaussian (the 3DGS projection, with the same blur, thresholds and cutoffs
// as the C++ renderer). The quads are sorted by depth on the CPU, drawn back
// to front, and blended into a floating-point image with premultiplied alpha,
// which gives the same result as the C++ front-to-back loop. A second pass
// turns that linear image into display colors.
//
// A Gaussian's color is three channels, each a weighted sum of its features
// (spectral.js makes the weights), computed in the vertex shader from the
// features in a texture. Changing what's shown only changes the weights.

import { JACOBIAN_CLAMP, NEAR, PIXEL_VARIANCE } from './probe.js';

const TEX_WIDTH = 2048;  // texels per row of the data textures; WebGL2 guarantees at least 2048
const MAX_FEATURE_TEXELS = 32;  // up to 128 features per Gaussian

const splatVertexShader = (texels) => `#version 300 es
precision highp float;
precision highp int;
precision highp sampler2D;

#define TEXELS ${texels}

uniform sampler2D uGeom;   // per Gaussian, 3 RGBA32F texels: (mean, opacity), (xx, xy, xz, yy), (yz, zz, offset, scale)
uniform sampler2D uFeat;   // per Gaussian, TEXELS RGBA8 texels: features as 0..1 between offset and offset + scale
uniform vec3 uRow0, uRow1, uRow2;  // world -> camera rotation, by rows
uniform vec3 uTrans;
uniform float uFocal;      // px
uniform vec2 uSize;        // image size, px
uniform vec2 uLimit;       // where the Jacobian's x/z and y/z are clamped
uniform float uNear;
uniform float uPixelVar;   // variance of a pixel's footprint, px^2
uniform vec4 uWeights[3 * TEXELS];  // per channel, weights on the 0..1 features
uniform vec3 uOffsetWeight;         // per channel, the sum of the feature weights

layout(location = 0) in uint aIndex;

flat out vec3 vColor;
flat out vec2 vMean;   // projected centre, px from the top-left
flat out vec3 vConic;  // inverse of the 2D covariance: (a, b, c) for a dx^2 + 2 b dx dy + c dy^2
flat out vec3 vCut;    // peak alpha, and the row-wise centre's slope q / r and variance det / r

ivec2 texel(int i) { return ivec2(i & ${TEX_WIDTH - 1}, i >> ${Math.log2(TEX_WIDTH)}); }

void main() {
  gl_Position = vec4(2.0, 2.0, 2.0, 1.0);  // off screen, unless the Gaussian is drawn
  int i = int(aIndex);
  vec4 g0 = texelFetch(uGeom, texel(3 * i), 0);
  vec4 g1 = texelFetch(uGeom, texel(3 * i + 1), 0);
  vec4 g2 = texelFetch(uGeom, texel(3 * i + 2), 0);
  vec3 pc = vec3(dot(uRow0, g0.xyz), dot(uRow1, g0.xyz), dot(uRow2, g0.xyz)) + uTrans;
  if (!(pc.z > uNear) || !(g0.w >= 1.0 / 255.0)) return;

  float invZ = 1.0 / pc.z;
  float tx = pc.x * invZ, ty = pc.y * invZ;
  vec2 mu = uFocal * vec2(tx, ty) + 0.5 * uSize;
  // The Jacobian of the projection, at a clamped direction so Gaussians far
  // off to the side don't get huge footprints.
  float a = uFocal * invZ;
  vec3 j0 = a * (uRow0 - clamp(tx, -uLimit.x, uLimit.x) * uRow2);
  vec3 j1 = a * (uRow1 - clamp(ty, -uLimit.y, uLimit.y) * uRow2);
  mat3 S = mat3(g1.x, g1.y, g1.z, g1.y, g1.w, g2.x, g1.z, g2.x, g2.y);
  vec3 s1 = S * j1;
  float pRaw = dot(j0, S * j0), q = dot(j0, s1), rRaw = dot(j1, s1);
  float p = pRaw + uPixelVar, r = rRaw + uPixelVar;
  float det = p * r - q * q;
  if (!(det > 0.0)) return;
  // Blurring spreads the alpha out but keeps its total.
  float alpha0 = g0.w * sqrt(max(pRaw * rRaw - q * q, 0.0) / det);
  if (!(alpha0 >= 1.0 / 255.0)) return;

  // A box around the ellipse where alpha >= 1/255, with a pixel to spare.
  vec2 halfSize = sqrt(2.0 * log(255.0 * alpha0) * vec2(p, r)) + 1.0;
  vec2 corner = vec2(float(gl_VertexID & 1), float(gl_VertexID >> 1)) * 2.0 - 1.0;
  vec2 pix = mu + corner * halfSize;
  gl_Position = vec4(2.0 * pix.x / uSize.x - 1.0, 1.0 - 2.0 * pix.y / uSize.y, 0.0, 1.0);
  vMean = mu;
  vConic = vec3(r, -q, p) / det;
  vCut = vec3(alpha0, q / r, det / r);

  vec3 c = vec3(0.0);
  for (int j = 0; j < TEXELS; ++j) {
    vec4 f = texelFetch(uFeat, texel(TEXELS * i + j), 0);
    c += vec3(dot(uWeights[j], f), dot(uWeights[TEXELS + j], f), dot(uWeights[2 * TEXELS + j], f));
  }
  vColor = g2.z * uOffsetWeight + g2.w * c;
}`;

const SPLAT_FRAGMENT_SHADER = `#version 300 es
precision highp float;

uniform vec2 uSize;
flat in vec3 vColor;
flat in vec2 vMean;
flat in vec3 vConic;
flat in vec3 vCut;
out vec4 outColor;

void main() {
  // The pixel's centre, px from the top-left, like the C++ cameras count.
  vec2 d = vec2(gl_FragCoord.x, uSize.y - gl_FragCoord.y) - vMean;
  float alpha = vCut.x * exp(-0.5 * (vConic.x * d.x * d.x + 2.0 * vConic.y * d.x * d.y + vConic.z * d.y * d.y));
  if (alpha < 1.0 / 255.0) discard;
  // The C++ renderer draws an image as rows of line cameras and stops each
  // row's 1D splat 3 sigma from its centre; this keeps the same edge.
  float du = d.x - vCut.y * d.y;
  if (du * du > 9.0 * vCut.z) discard;
  alpha = min(alpha, 0.99);
  outColor = vec4(vColor * alpha, alpha);
}`;

const FULLSCREEN_VERTEX_SHADER = `#version 300 es
void main() {
  vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));
  gl_Position = vec4(2.0 * p - 1.0, 0.0, 1.0);
}`;

// Modes: 0 shows the three channels as linear RGB, 1 shows the first as
// gray, 2 shows (first - second) / (first + second) on a diverging scale.
const DISPLAY_FRAGMENT_SHADER = `#version 300 es
precision highp float;

uniform sampler2D uAcc;
uniform int uMode;
uniform float uGain;
uniform vec3 uBackground;  // the scene's background spectrum, through the channel weights
out vec4 outColor;

vec3 srgb(vec3 c) {
  c = clamp(c, 0.0, 1.0);
  return mix(12.92 * c, 1.055 * pow(c, vec3(1.0 / 2.4)) - 0.055, step(0.0031308, c));
}

// Blue below zero, red above, through a dark neutral gray at zero; the ends
// are the lightest, so the strongest values stand out.
vec3 diverging(float x) {
  vec3 c0 = vec3(0.663, 0.804, 0.965), c1 = vec3(0.184, 0.478, 0.847), c2 = vec3(0.267, 0.267, 0.247);
  vec3 c3 = vec3(0.788, 0.290, 0.282), c4 = vec3(0.953, 0.631, 0.608);
  float t = clamp(x, -1.0, 1.0) * 2.0 + 2.0;
  if (t < 1.0) return mix(c0, c1, t);
  if (t < 2.0) return mix(c1, c2, t - 1.0);
  if (t < 3.0) return mix(c2, c3, t - 2.0);
  return mix(c3, c4, t - 3.0);
}

void main() {
  vec4 acc = texelFetch(uAcc, ivec2(gl_FragCoord.xy), 0);
  vec3 c = acc.rgb + (1.0 - acc.a) * uBackground;
  vec3 rgb;
  if (uMode == 0) {
    rgb = srgb(uGain * c);
  } else if (uMode == 1) {
    rgb = srgb(vec3(uGain * c.r));
  } else {
    float sum = c.r + c.g;
    float nd = abs(sum) > 1e-6 ? (c.r - c.g) / sum : 0.0;
    rgb = mix(vec3(0.05), diverging(nd), clamp(acc.a, 0.0, 1.0));
  }
  outColor = vec4(rgb, 1.0);
}`;

function compile(gl, type, source) {
  const shader = gl.createShader(type);
  gl.shaderSource(shader, source);
  gl.compileShader(shader);
  if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
    const log = gl.getShaderInfoLog(shader);
    gl.deleteShader(shader);
    throw new Error(`shader didn't compile: ${log}`);
  }
  return shader;
}

function link(gl, vertexSource, fragmentSource) {
  const program = gl.createProgram();
  gl.attachShader(program, compile(gl, gl.VERTEX_SHADER, vertexSource));
  gl.attachShader(program, compile(gl, gl.FRAGMENT_SHADER, fragmentSource));
  gl.linkProgram(program);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw new Error(`shaders didn't link: ${gl.getProgramInfoLog(program)}`);
  const uniforms = {};
  const n = gl.getProgramParameter(program, gl.ACTIVE_UNIFORMS);
  for (let i = 0; i < n; i++) {
    const name = gl.getActiveUniform(program, i).name.replace(/\[0\]$/, '');
    uniforms[name] = gl.getUniformLocation(program, name);
  }
  return { program, uniforms };
}

function dataTexture(gl, internalFormat, format, type, data, rows) {
  const tex = gl.createTexture();
  gl.bindTexture(gl.TEXTURE_2D, tex);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
  gl.pixelStorei(gl.UNPACK_ALIGNMENT, 1);
  gl.texImage2D(gl.TEXTURE_2D, 0, internalFormat, TEX_WIDTH, rows, 0, format, type, data);
  return tex;
}

// Sorts `count` indices by their uint32 keys, smallest first, keeping the
// order of equal keys: a least-significant-digit radix sort, a byte at a time.
function radixSort(keys, idx, count, keysTmp, idxTmp, histogram) {
  let k0 = keys, i0 = idx, k1 = keysTmp, i1 = idxTmp;
  for (let shift = 0; shift < 32; shift += 8) {
    histogram.fill(0);
    for (let j = 0; j < count; j++) histogram[(k0[j] >>> shift) & 255]++;
    if (histogram[(k0[0] >>> shift) & 255] === count) continue;  // every key has this byte
    let sum = 0;
    for (let b = 0; b < 256; b++) {
      const c = histogram[b];
      histogram[b] = sum;
      sum += c;
    }
    for (let j = 0; j < count; j++) {
      const at = histogram[(k0[j] >>> shift) & 255]++;
      k1[at] = k0[j];
      i1[at] = i0[j];
    }
    [k0, k1] = [k1, k0];
    [i0, i1] = [i1, i0];
  }
  return i0;
}

export class SplatRenderer {
  constructor(canvas) {
    const gl = canvas.getContext('webgl2', { antialias: false, alpha: false, depth: false, stencil: false });
    if (!gl) throw new Error('this browser has no WebGL2');
    this.gl = gl;
    this.canvas = canvas;
    this.display = link(gl, FULLSCREEN_VERTEX_SHADER, DISPLAY_FRAGMENT_SHADER);
    this.accFormat = this.pickAccumulationFormat();
    this.acc = null;
    this.scene = null;
    this.channels = { weights: [], offset: [0, 0, 0], background: [0, 0, 0] };
    this.sortedFor = null;
    this.visible = 0;
    this.vao = gl.createVertexArray();
    this.emptyVao = gl.createVertexArray();
  }

  // The most precise image format the GPU can blend into: 32-bit float, then
  // 16-bit float, then 8 bits as a last resort.
  pickAccumulationFormat() {
    const gl = this.gl;
    const formats = [];
    const colorFloat = gl.getExtension('EXT_color_buffer_float');
    if (colorFloat && gl.getExtension('EXT_float_blend'))
      formats.push({ name: 'RGBA32F', internal: gl.RGBA32F, type: gl.FLOAT, read: gl.FLOAT });
    if (colorFloat || gl.getExtension('EXT_color_buffer_half_float'))
      formats.push({ name: 'RGBA16F', internal: gl.RGBA16F, type: gl.HALF_FLOAT, read: gl.FLOAT });
    formats.push({ name: 'RGBA8', internal: gl.RGBA8, type: gl.UNSIGNED_BYTE, read: gl.UNSIGNED_BYTE });
    for (const f of formats) {
      const target = this.makeTarget(f, 4, 4);
      const ok = gl.checkFramebufferStatus(gl.FRAMEBUFFER) === gl.FRAMEBUFFER_COMPLETE;
      gl.bindFramebuffer(gl.FRAMEBUFFER, null);
      gl.deleteFramebuffer(target.fbo);
      gl.deleteTexture(target.tex);
      if (ok) return f;
    }
    throw new Error("the GPU can't render to any image format this viewer needs");
  }

  makeTarget(format, width, height) {
    const gl = this.gl;
    const tex = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, tex);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    gl.texImage2D(gl.TEXTURE_2D, 0, format.internal, width, height, 0, gl.RGBA, format.type, null);
    const fbo = gl.createFramebuffer();
    gl.bindFramebuffer(gl.FRAMEBUFFER, fbo);
    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, tex, 0);
    return { tex, fbo, width, height };
  }

  setScene(scene) {
    const gl = this.gl;
    const N = scene.count, K = scene.numFeatures;
    const texels = Math.max(1, Math.ceil(K / 4));
    if (texels > MAX_FEATURE_TEXELS)
      throw new Error(`this viewer handles up to ${4 * MAX_FEATURE_TEXELS} features; the file has ${K}`);
    this.dispose();

    const geomRows = Math.max(1, Math.ceil((3 * N) / TEX_WIDTH));
    const geom = new Float32Array(TEX_WIDTH * geomRows * 4);
    for (let i = 0; i < N; i++) {
      const o = 12 * i, c = 6 * i;
      geom.set([scene.means[3 * i], scene.means[3 * i + 1], scene.means[3 * i + 2], scene.opacity[i]], o);
      geom.set([scene.cov[c], scene.cov[c + 1], scene.cov[c + 2], scene.cov[c + 3]], o + 4);
      geom.set([scene.cov[c + 4], scene.cov[c + 5], scene.featureRange[2 * i], scene.featureRange[2 * i + 1]], o + 8);
    }
    const featRows = Math.max(1, Math.ceil((texels * N) / TEX_WIDTH));
    const feat = new Uint8Array(TEX_WIDTH * featRows * 4);
    for (let i = 0; i < N; i++) feat.set(scene.features.subarray(i * K, (i + 1) * K), 4 * texels * i);

    this.geomTex = dataTexture(gl, gl.RGBA32F, gl.RGBA, gl.FLOAT, geom, geomRows);
    this.featTex = dataTexture(gl, gl.RGBA8, gl.RGBA, gl.UNSIGNED_BYTE, feat, featRows);
    this.splat = link(gl, splatVertexShader(texels), SPLAT_FRAGMENT_SHADER);
    this.texels = texels;

    this.order = new Uint32Array(N);
    this.keys = new Uint32Array(N);
    this.keysTmp = new Uint32Array(N);
    this.idx = new Uint32Array(N);
    this.idxTmp = new Uint32Array(N);
    this.histogram = new Uint32Array(256);
    this.depthBits = new Float32Array(1);
    this.depthWord = new Uint32Array(this.depthBits.buffer);
    this.indexBuffer = gl.createBuffer();
    gl.bindVertexArray(this.vao);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.indexBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, this.order.byteLength, gl.DYNAMIC_DRAW);
    gl.enableVertexAttribArray(0);
    gl.vertexAttribIPointer(0, 1, gl.UNSIGNED_INT, 0, 0);
    gl.vertexAttribDivisor(0, 1);
    gl.bindVertexArray(null);

    this.scene = scene;
    this.sortedFor = null;
    this.setChannels(this.bandWeights || []);
  }

  dispose() {
    const gl = this.gl;
    if (this.geomTex) gl.deleteTexture(this.geomTex);
    if (this.featTex) gl.deleteTexture(this.featTex);
    if (this.indexBuffer) gl.deleteBuffer(this.indexBuffer);
    if (this.splat) gl.deleteProgram(this.splat.program);
    this.geomTex = this.featTex = this.indexBuffer = this.splat = null;
  }

  // Up to three channels, each a Float64Array of weights over the bands.
  // They're carried through the basis onto the features here, once, so the
  // shader only takes one dot product per four features.
  setChannels(bandWeights) {
    this.bandWeights = bandWeights;
    const s = this.scene;
    if (!s) return;
    const K = s.numFeatures, B = s.numBands, T = this.texels;
    const weights = new Float32Array(3 * T * 4);
    const offset = [0, 0, 0], background = [0, 0, 0];
    bandWeights.slice(0, 3).forEach((w, c) => {
      for (let k = 0; k < K; k++) {
        let v = 0;
        for (let b = 0; b < B; b++) v += w[b] * s.basis[b * K + k];
        weights[(c * T) * 4 + k] = v;
        offset[c] += v;
        background[c] += v * s.background[k];
      }
    });
    this.channels = { weights, offset, background };
  }

  resize(width, height) {
    if (this.acc && this.acc.width === width && this.acc.height === height) return;
    const gl = this.gl;
    if (this.acc) {
      gl.deleteFramebuffer(this.acc.fbo);
      gl.deleteTexture(this.acc.tex);
    }
    this.acc = this.makeTarget(this.accFormat, width, height);
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
  }

  // Back to front: sort by camera depth with the file order breaking ties
  // (as the C++ renderer's stable sort does front to back), then draw the
  // list in reverse.
  sort(cam) {
    const s = this.scene, N = s.count, m = s.means;
    const { R, t } = cam;
    let n = 0;
    for (let i = 0; i < N; i++) {
      const z = R[6] * m[3 * i] + R[7] * m[3 * i + 1] + R[8] * m[3 * i + 2] + t[2];
      if (!(z > NEAR)) continue;
      this.depthBits[0] = z;  // a positive float's bits sort like the float
      this.keys[n] = this.depthWord[0];
      this.idx[n] = i;
      n++;
    }
    const sorted = radixSort(this.keys, this.idx, n, this.keysTmp, this.idxTmp, this.histogram);
    for (let j = 0; j < n; j++) this.order[n - 1 - j] = sorted[j];
    const gl = this.gl;
    gl.bindBuffer(gl.ARRAY_BUFFER, this.indexBuffer);
    gl.bufferSubData(gl.ARRAY_BUFFER, 0, this.order, 0, n);
    this.visible = n;
  }

  // cam: {R, t, f, width, height} (camera.js). display: {mode, gain}.
  render(cam, display) {
    const gl = this.gl;
    const { width, height } = cam;
    this.resize(width, height);
    gl.bindFramebuffer(gl.FRAMEBUFFER, this.acc.fbo);
    gl.viewport(0, 0, width, height);
    gl.clearColor(0, 0, 0, 0);
    gl.clear(gl.COLOR_BUFFER_BIT);
    if (this.scene) {
      const key = [...cam.R, ...cam.t].join(',');
      if (this.sortedFor !== key) {
        this.sort(cam);
        this.sortedFor = key;
      }
      const { program, uniforms: u } = this.splat;
      gl.useProgram(program);
      gl.activeTexture(gl.TEXTURE0);
      gl.bindTexture(gl.TEXTURE_2D, this.geomTex);
      gl.uniform1i(u.uGeom, 0);
      gl.activeTexture(gl.TEXTURE1);
      gl.bindTexture(gl.TEXTURE_2D, this.featTex);
      gl.uniform1i(u.uFeat, 1);
      gl.uniform3f(u.uRow0, cam.R[0], cam.R[1], cam.R[2]);
      gl.uniform3f(u.uRow1, cam.R[3], cam.R[4], cam.R[5]);
      gl.uniform3f(u.uRow2, cam.R[6], cam.R[7], cam.R[8]);
      gl.uniform3f(u.uTrans, cam.t[0], cam.t[1], cam.t[2]);
      gl.uniform1f(u.uFocal, cam.f);
      gl.uniform2f(u.uSize, width, height);
      // As the C++ cameras clamp it, taking the bottom row's slit offset for
      // the whole image (the difference only touches Gaussians well off screen).
      const limX = JACOBIAN_CLAMP * (width / 2) / cam.f;
      gl.uniform2f(u.uLimit, limX, limX + (height / 2) / cam.f);
      gl.uniform1f(u.uNear, NEAR);
      gl.uniform1f(u.uPixelVar, PIXEL_VARIANCE);
      gl.uniform4fv(u.uWeights, this.channels.weights);
      gl.uniform3fv(u.uOffsetWeight, this.channels.offset);
      gl.enable(gl.BLEND);
      gl.blendFunc(gl.ONE, gl.ONE_MINUS_SRC_ALPHA);
      gl.bindVertexArray(this.vao);
      gl.drawArraysInstanced(gl.TRIANGLE_STRIP, 0, 4, this.visible);
      gl.bindVertexArray(null);
      gl.disable(gl.BLEND);
    }

    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    gl.viewport(0, 0, width, height);
    const { program, uniforms: u } = this.display;
    gl.useProgram(program);
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, this.acc.tex);
    gl.uniform1i(u.uAcc, 0);
    gl.uniform1i(u.uMode, display.mode);
    gl.uniform1f(u.uGain, display.gain);
    gl.uniform3fv(u.uBackground, this.channels.background);
    gl.bindVertexArray(this.emptyVao);
    gl.drawArrays(gl.TRIANGLES, 0, 3);
    gl.bindVertexArray(null);
  }

  // The blended linear channels and coverage of the last render, as
  // [r, g, b, coverage] per pixel, top row first.
  readLinear() {
    const gl = this.gl;
    const { width, height } = this.acc;
    const f = this.accFormat;
    const raw = f.read === gl.FLOAT ? new Float32Array(width * height * 4) : new Uint8Array(width * height * 4);
    gl.bindFramebuffer(gl.FRAMEBUFFER, this.acc.fbo);
    gl.readPixels(0, 0, width, height, gl.RGBA, f.read, raw);
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    const out = new Float32Array(width * height * 4);
    const scale = f.read === gl.FLOAT ? 1 : 1 / 255;
    for (let y = 0; y < height; y++) {
      const src = (height - 1 - y) * width * 4;  // GL counts rows from the bottom
      for (let j = 0; j < width * 4; j++) out[y * width * 4 + j] = raw[src + j] * scale;
    }
    return { width, height, data: out };
  }
}
