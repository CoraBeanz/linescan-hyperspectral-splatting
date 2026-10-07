// The viewing camera.
//
// A pinhole camera with the C++ code's axes (scan_model.cpp's look_at): x to
// the right, y down, z forward. A world point p lands on pixel
//
//   [x, y, z] = R p + t,   u = f x / z + width / 2,   v = f y / z + height / 2
//
// with pixel (i, j) covering u in [i, i + 1) and v in [j, j + 1), counted
// from the top-left. The orbit controls move the eye around a target point
// with +Z up, which is how the scenes are laid out (the board is z = 0).

const DEG = Math.PI / 180;

function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
function normalize(a) {
  const n = Math.hypot(a[0], a[1], a[2]);
  return [a[0] / n, a[1] / n, a[2] / n];
}

// World -> camera rotation (row-major) and translation of a camera at `eye`
// looking at `target`.
export function lookAt(eye, target, up) {
  const z = normalize(sub(target, eye));
  const x = normalize(cross(z, up));
  const y = cross(z, x);
  const R = [x[0], x[1], x[2], y[0], y[1], y[2], z[0], z[1], z[2]];
  const t = [
    -(R[0] * eye[0] + R[1] * eye[1] + R[2] * eye[2]),
    -(R[3] * eye[0] + R[4] * eye[1] + R[5] * eye[2]),
    -(R[6] * eye[0] + R[7] * eye[1] + R[8] * eye[2]),
  ];
  return { R, t, eye: [...eye] };
}

// Focal length in pixels for a field of view (degrees) across the shorter side.
export function focalForFov(fovDeg, width, height) {
  return 0.5 * Math.min(width, height) / Math.tan(0.5 * fovDeg * DEG);
}

// Where a world point lands: [u, v, z] in pixels and metres, or null behind
// the camera.
export function projectPoint(cam, p, near = 1e-3) {
  const { R, t, f, width, height } = cam;
  const x = R[0] * p[0] + R[1] * p[1] + R[2] * p[2] + t[0];
  const y = R[3] * p[0] + R[4] * p[1] + R[5] * p[2] + t[1];
  const z = R[6] * p[0] + R[7] * p[1] + R[8] * p[2] + t[2];
  if (z <= near) return null;
  return [f * x / z + width / 2, f * y / z + height / 2, z];
}

// The world point at depth z (metres along the view axis) behind pixel position (u, v).
export function unproject(cam, u, v, z) {
  const { R, t, f, width, height } = cam;
  const c = [z * (u - width / 2) / f - t[0], z * (v - height / 2) / f - t[1], z - t[2]];
  // R is a rotation, so its inverse is its transpose.
  return [
    R[0] * c[0] + R[3] * c[1] + R[6] * c[2],
    R[1] * c[0] + R[4] * c[1] + R[7] * c[2],
    R[2] * c[0] + R[5] * c[1] + R[8] * c[2],
  ];
}

export class OrbitCamera {
  constructor(view) {
    this.reset(view);
  }

  // view: {eye, target, fov_deg}, as splat_export writes it.
  reset(view) {
    this.target = [...view.target];
    const d = sub(view.eye, view.target);
    this.distance = Math.hypot(d[0], d[1], d[2]);
    this.azimuth = Math.atan2(d[1], d[0]);
    this.elevation = Math.asin(d[2] / this.distance);
    this.fovDeg = view.fov_deg;
    this.version = (this.version || 0) + 1;
  }

  get eye() {
    const ce = Math.cos(this.elevation);
    return [
      this.target[0] + this.distance * ce * Math.cos(this.azimuth),
      this.target[1] + this.distance * ce * Math.sin(this.azimuth),
      this.target[2] + this.distance * Math.sin(this.elevation),
    ];
  }

  // Everything the renderer needs for a width x height pixel image.
  frame(width, height) {
    const pose = lookAt(this.eye, this.target, [0, 0, 1]);
    return { ...pose, f: focalForFov(this.fovDeg, width, height), width, height };
  }

  // Turns the view by angles in radians; elevation stops short of straight
  // up or down, where "up" stops meaning anything.
  orbit(dAzimuth, dElevation) {
    this.azimuth += dAzimuth;
    const limit = 89.5 * DEG;
    this.elevation = Math.min(Math.max(this.elevation + dElevation, -limit), limit);
    this.version++;
  }

  // Slides the target with the image, by a pixel offset at focal length f.
  pan(dxPx, dyPx, f) {
    const { R } = lookAt(this.eye, this.target, [0, 0, 1]);
    const s = this.distance / f;
    for (let k = 0; k < 3; k++) this.target[k] -= s * (dxPx * R[k] + dyPx * R[3 + k]);
    this.version++;
  }

  // Moves toward (factor < 1) or away from (factor > 1) the target.
  dolly(factor) {
    this.distance = Math.min(Math.max(this.distance * factor, 0.01), 3);
    this.version++;
  }
}
