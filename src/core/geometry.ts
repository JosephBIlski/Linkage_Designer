/** Small 3-D vector helpers on plain tuples (kept dependency-free so the solver can run in tests / workers). */
import type { Vec3 } from './types';

export const v3 = (x = 0, y = 0, z = 0): Vec3 => [x, y, z];
export const add = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const sub = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const scale = (a: Vec3, s: number): Vec3 => [a[0] * s, a[1] * s, a[2] * s];
export const dot = (a: Vec3, b: Vec3): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross = (a: Vec3, b: Vec3): Vec3 => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
export const len = (a: Vec3): number => Math.hypot(a[0], a[1], a[2]);
export const dist = (a: Vec3, b: Vec3): number => len(sub(a, b));
export const neg = (a: Vec3): Vec3 => [-a[0], -a[1], -a[2]];
export const copy = (a: Vec3): Vec3 => [a[0], a[1], a[2]];
export const lerp = (a: Vec3, b: Vec3, t: number): Vec3 => add(a, scale(sub(b, a), t));
export const eq = (a: Vec3, b: Vec3, tol = 1e-9): boolean => dist(a, b) <= tol;

export function normalize(a: Vec3): Vec3 {
  const l = len(a);
  return l > 1e-15 ? scale(a, 1 / l) : [0, 0, 0];
}

/**
 * A unit vector perpendicular to `n`: the first world axis not parallel to n,
 * made orthogonal to n. For n = +Z this is +X, so planeBasis(+Z) = (+X, +Y).
 */
export function perpendicular(n: Vec3): Vec3 {
  const nn = normalize(n);
  const candidates: Vec3[] = [v3(1, 0, 0), v3(0, 1, 0), v3(0, 0, 1)];
  for (const a of candidates) {
    if (Math.abs(dot(a, nn)) < 0.9) return normalize(sub(a, scale(nn, dot(a, nn))));
  }
  return v3(1, 0, 0);
}

/** Orthonormal basis (u, v) spanning the plane perpendicular to unit normal n. */
export function planeBasis(n: Vec3): [Vec3, Vec3] {
  const u = perpendicular(n);
  const v = normalize(cross(n, u));
  return [u, v];
}

/** Rotate vector `p` about unit axis `k` by angle `theta` (Rodrigues). */
export function rotateAbout(p: Vec3, k: Vec3, theta: number): Vec3 {
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  return add(add(scale(p, c), scale(cross(k, p), s)), scale(k, dot(k, p) * (1 - c)));
}

/** Signed angle of vector w measured in the plane with normal n from reference u (v = n × u). */
export function angleInPlane(w: Vec3, n: Vec3, u: Vec3): number {
  const v = cross(n, u);
  return Math.atan2(dot(w, v), dot(w, u));
}

/** Wrap angle to (-pi, pi]. */
export function wrapAngle(a: number): number {
  let r = a % (2 * Math.PI);
  if (r > Math.PI) r -= 2 * Math.PI;
  if (r <= -Math.PI) r += 2 * Math.PI;
  return r;
}

export const deg2rad = (d: number): number => (d * Math.PI) / 180;
export const rad2deg = (r: number): number => (r * 180) / Math.PI;

/** Triple product (a × b) · c */
export const triple = (a: Vec3, b: Vec3, c: Vec3): number => dot(cross(a, b), c);

/** Centroid of a list of points. */
export function centroid(pts: Vec3[]): Vec3 {
  if (pts.length === 0) return [0, 0, 0];
  let s: Vec3 = [0, 0, 0];
  for (const p of pts) s = add(s, p);
  return scale(s, 1 / pts.length);
}

/**
 * Newell normal of a polygon given by its vertices in order: the area-weighted
 * normal, oriented by the winding (counter-clockwise seen from the side the
 * normal points to). Not normalised; zero for degenerate input.
 */
export function newellNormal(pts: Vec3[]): Vec3 {
  let n: Vec3 = [0, 0, 0];
  for (let i = 0; i < pts.length; i++) {
    const a = pts[i];
    const b = pts[(i + 1) % pts.length];
    n = add(n, [(a[1] - b[1]) * (a[2] + b[2]), (a[2] - b[2]) * (a[0] + b[0]), (a[0] - b[0]) * (a[1] + b[1])]);
  }
  return n;
}

/** Dihedral (fold) angle about edge a->b between faces containing p and q.
 *  Positive when q is rotated from p by the right-hand rule about (b - a). */
export function dihedralAngle(a: Vec3, b: Vec3, p: Vec3, q: Vec3): number {
  const e = normalize(sub(b, a));
  const n1 = cross(e, sub(p, a));
  const n2 = cross(e, sub(q, a));
  return Math.atan2(dot(cross(n1, n2), e), dot(n1, n2));
}
