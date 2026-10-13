/**
 * Constraint primitives for the point-based solver.
 *
 * Every constraint maps the variable vector x to a residual vector r (length m)
 * and provides its Jacobian ∂r/∂x as sparse (row, col, value) entries. All
 * residuals are expressed in *length units* so they can be mixed in one
 * least-squares problem (angles are multiplied by a characteristic length).
 *
 * A point reference (PRef) carries one column index per coordinate; a column of
 * -1 means that coordinate is a constant (ground point, frozen coordinate on an
 * axis-aligned sketch plane, construction geometry).
 */
import type { Vec3 } from './types';
import { cross, dot, len, normalize, sub, wrapAngle, dihedralAngle, perpendicular } from './geometry';

export interface PRef {
  cols: [number, number, number];
  /** Constant values for components whose column is -1 (also the initial value otherwise). */
  c: Vec3;
}

export type JacPush = (row: number, col: number, val: number) => void;

export interface Constraint {
  /** Number of scalar residuals. */
  m: number;
  /** Multiplies residual and Jacobian rows (1 = hard constraint, small = soft / compliant). */
  weight: number;
  /** Hard constraints count toward the Jacobian rank used for degree-of-freedom analysis. */
  hard: boolean;
  /** Debug / grouping tag, e.g. "dist:link_3", "joint:joint_2", "target:t_1". */
  tag: string;
  eval(x: Float64Array, out: Float64Array, off: number): void;
  jac(x: Float64Array, row0: number, push: JacPush): void;
}

export const constRef = (c: Vec3): PRef => ({ cols: [-1, -1, -1], c: [c[0], c[1], c[2]] });

export function getP(x: Float64Array, r: PRef): Vec3 {
  const c = r.cols;
  return [c[0] >= 0 ? x[c[0]] : r.c[0], c[1] >= 0 ? x[c[1]] : r.c[1], c[2] >= 0 ? x[c[2]] : r.c[2]];
}

export function isConstRef(r: PRef): boolean {
  return r.cols[0] < 0 && r.cols[1] < 0 && r.cols[2] < 0;
}

function pushGrad(push: JacPush, row: number, r: PRef, g: Vec3, s = 1): void {
  if (r.cols[0] >= 0) push(row, r.cols[0], g[0] * s);
  if (r.cols[1] >= 0) push(row, r.cols[1], g[1] * s);
  if (r.cols[2] >= 0) push(row, r.cols[2], g[2] * s);
}

/** Skew-symmetric matrix action helpers: [u]× w = u × w. Returns the 3x3 matrix rows of [u]×. */
function skewRows(u: Vec3): [Vec3, Vec3, Vec3] {
  return [
    [0, -u[2], u[1]],
    [u[2], 0, -u[0]],
    [-u[1], u[0], 0],
  ];
}

// ---------------------------------------------------------------------------
// Distance constraints
// ---------------------------------------------------------------------------

/** |p - q| - L = 0 */
export function distance(p: PRef, q: PRef, L: number, weight = 1, hard = true, tag = 'dist'): Constraint {
  return {
    m: 1,
    weight,
    hard,
    tag,
    eval(x, out, off) {
      out[off] = len(sub(getP(x, p), getP(x, q))) - L;
    },
    jac(x, row, push) {
      const d = sub(getP(x, p), getP(x, q));
      const l = len(d);
      const g: Vec3 = l > 1e-12 ? [d[0] / l, d[1] / l, d[2] / l] : [1, 0, 0];
      pushGrad(push, row, p, g, 1);
      pushGrad(push, row, q, g, -1);
    },
  };
}

/** |p - q| - |p0 - q0| = 0  (same length in two poses; the length itself is a free design variable). */
export function sharedDistance(p: PRef, q: PRef, p0: PRef, q0: PRef, weight = 1, tag = 'shared'): Constraint {
  return {
    m: 1,
    weight,
    hard: true,
    tag,
    eval(x, out, off) {
      out[off] = len(sub(getP(x, p), getP(x, q))) - len(sub(getP(x, p0), getP(x, q0)));
    },
    jac(x, row, push) {
      const d = sub(getP(x, p), getP(x, q));
      const l = len(d);
      const g: Vec3 = l > 1e-12 ? [d[0] / l, d[1] / l, d[2] / l] : [1, 0, 0];
      pushGrad(push, row, p, g, 1);
      pushGrad(push, row, q, g, -1);
      const d0 = sub(getP(x, p0), getP(x, q0));
      const l0 = len(d0);
      const g0: Vec3 = l0 > 1e-12 ? [d0[0] / l0, d0[1] / l0, d0[2] / l0] : [1, 0, 0];
      pushGrad(push, row, p0, g0, -1);
      pushGrad(push, row, q0, g0, 1);
    },
  };
}

// ---------------------------------------------------------------------------
// Position constraints
// ---------------------------------------------------------------------------

/** p - q = 0 (3 residuals). */
export function coincide(p: PRef, q: PRef, weight = 1, hard = true, tag = 'coincide'): Constraint {
  return {
    m: 3,
    weight,
    hard,
    tag,
    eval(x, out, off) {
      const a = getP(x, p);
      const b = getP(x, q);
      out[off] = a[0] - b[0];
      out[off + 1] = a[1] - b[1];
      out[off + 2] = a[2] - b[2];
    },
    jac(_x, row, push) {
      for (let i = 0; i < 3; i++) {
        if (p.cols[i] >= 0) push(row + i, p.cols[i], 1);
        if (q.cols[i] >= 0) push(row + i, q.cols[i], -1);
      }
    },
  };
}

/** p - T = 0 for a constant target T (3 residuals). */
export function fixPosition(p: PRef, target: Vec3, weight = 1, hard = true, tag = 'fix'): Constraint {
  return coincide(p, constRef(target), weight, hard, tag);
}

/** n · (p - o) = 0 for a world plane (origin o, unit normal n). */
export function pointOnPlane(p: PRef, o: Vec3, n: Vec3, weight = 1, hard = true, tag = 'onPlane'): Constraint {
  const nn = normalize(n);
  return {
    m: 1,
    weight,
    hard,
    tag,
    eval(x, out, off) {
      out[off] = dot(nn, sub(getP(x, p), o));
    },
    jac(_x, row, push) {
      pushGrad(push, row, p, nn, 1);
    },
  };
}

/** p lies in the plane through body points a, b, c:  ((b-a) × (c-a)) · (p-a) / A0 = 0. */
export function pointOnBodyPlane(p: PRef, a: PRef, b: PRef, c: PRef, A0: number, weight = 1, hard = true, tag = 'onBodyPlane'): Constraint {
  const inv = 1 / Math.max(A0, 1e-12);
  return {
    m: 1,
    weight,
    hard,
    tag,
    eval(x, out, off) {
      const pa = getP(x, a);
      const u = sub(getP(x, b), pa);
      const v = sub(getP(x, c), pa);
      const w = sub(getP(x, p), pa);
      out[off] = dot(cross(u, v), w) * inv;
    },
    jac(x, row, push) {
      const pa = getP(x, a);
      const u = sub(getP(x, b), pa);
      const v = sub(getP(x, c), pa);
      const w = sub(getP(x, p), pa);
      const gp = cross(u, v); // ∂/∂p
      const gb = cross(v, w); // ∂/∂b : (v × w)
      const gc = cross(w, u); // ∂/∂c : (w × u)
      const ga: Vec3 = [-(gp[0] + gb[0] + gc[0]), -(gp[1] + gb[1] + gc[1]), -(gp[2] + gb[2] + gc[2])];
      pushGrad(push, row, p, gp, inv);
      pushGrad(push, row, b, gb, inv);
      pushGrad(push, row, c, gc, inv);
      pushGrad(push, row, a, ga, inv);
    },
  };
}

/** p lies on the world line (o, unit d): (p-o) - ((p-o)·d) d = 0 (3 residuals, rank 2). */
export function pointOnLine(p: PRef, o: Vec3, d: Vec3, weight = 1, hard = true, tag = 'onLine'): Constraint {
  const dd = normalize(d);
  return {
    m: 3,
    weight,
    hard,
    tag,
    eval(x, out, off) {
      const w = sub(getP(x, p), o);
      const t = dot(w, dd);
      out[off] = w[0] - t * dd[0];
      out[off + 1] = w[1] - t * dd[1];
      out[off + 2] = w[2] - t * dd[2];
    },
    jac(_x, row, push) {
      for (let i = 0; i < 3; i++) {
        for (let j = 0; j < 3; j++) {
          if (p.cols[j] >= 0) push(row + i, p.cols[j], (i === j ? 1 : 0) - dd[i] * dd[j]);
        }
      }
    },
  };
}

/** p lies on the line through body points a, b:  ((p-a) × (b-a)) / L0 = 0 (3 residuals, rank 2). */
export function pointOnBodyLine(p: PRef, a: PRef, b: PRef, L0: number, weight = 1, hard = true, tag = 'onBodyLine'): Constraint {
  const inv = 1 / Math.max(L0, 1e-12);
  return {
    m: 3,
    weight,
    hard,
    tag,
    eval(x, out, off) {
      const pa = getP(x, a);
      const w = sub(getP(x, p), pa);
      const u = sub(getP(x, b), pa);
      const r = cross(w, u);
      out[off] = r[0] * inv;
      out[off + 1] = r[1] * inv;
      out[off + 2] = r[2] * inv;
    },
    jac(x, row, push) {
      const pa = getP(x, a);
      const w = sub(getP(x, p), pa);
      const u = sub(getP(x, b), pa);
      // r = w × u.  ∂r/∂w = -[u]×,  ∂r/∂u = [w]×
      const Su = skewRows(u);
      const Sw = skewRows(w);
      for (let i = 0; i < 3; i++) {
        for (let j = 0; j < 3; j++) {
          const dw = -Su[i][j]; // ∂r_i/∂w_j
          const du = Sw[i][j]; // ∂r_i/∂u_j
          if (p.cols[j] >= 0) push(row + i, p.cols[j], dw * inv); // w = p - a
          if (b.cols[j] >= 0) push(row + i, b.cols[j], du * inv); // u = b - a
          if (a.cols[j] >= 0) push(row + i, a.cols[j], (-dw - du) * inv);
        }
      }
    },
  };
}

/** Projection of (pB - pA) onto the unit direction of (qA - pA) equals s0 (prevents sliding along a body axis). */
export function slideLock(pB: PRef, pA: PRef, qA: PRef, s0: number, weight = 1, hard = true, tag = 'slideLock'): Constraint {
  return {
    m: 1,
    weight,
    hard,
    tag,
    eval(x, out, off) {
      const a = getP(x, pA);
      const d = sub(getP(x, qA), a);
      const L = len(d);
      const w = sub(getP(x, pB), a);
      out[off] = (L > 1e-12 ? dot(w, d) / L : 0) - s0;
    },
    jac(x, row, push) {
      const a = getP(x, pA);
      const d = sub(getP(x, qA), a);
      const L = Math.max(len(d), 1e-12);
      const w = sub(getP(x, pB), a);
      const wd = dot(w, d);
      const gB: Vec3 = [d[0] / L, d[1] / L, d[2] / L];
      const gQ: Vec3 = [w[0] / L - (wd * d[0]) / L ** 3, w[1] / L - (wd * d[1]) / L ** 3, w[2] / L - (wd * d[2]) / L ** 3];
      const gA: Vec3 = [-gB[0] - gQ[0], -gB[1] - gQ[1], -gB[2] - gQ[2]];
      pushGrad(push, row, pB, gB);
      pushGrad(push, row, qA, gQ);
      pushGrad(push, row, pA, gA);
    },
  };
}

/** (p - o) · d = s0 for a world line (o, unit d). */
export function projectionLock(p: PRef, o: Vec3, d: Vec3, s0: number, weight = 1, hard = true, tag = 'projLock'): Constraint {
  const dd = normalize(d);
  return {
    m: 1,
    weight,
    hard,
    tag,
    eval(x, out, off) {
      out[off] = dot(sub(getP(x, p), o), dd) - s0;
    },
    jac(_x, row, push) {
      pushGrad(push, row, p, dd);
    },
  };
}

// ---------------------------------------------------------------------------
// Angle constraints
// ---------------------------------------------------------------------------

/**
 * Driver: the angle of (tip - pivot), measured in the plane with unit normal n
 * from the reference direction u (v = n × u), equals theta (radians).
 * Residual is scaled by the characteristic length Lc so it has length units.
 */
export function angleDriver(tip: PRef, pivot: PRef, n: Vec3, u: Vec3, theta: number, Lc: number, weight = 1, hard = true, tag = 'driver'): Constraint {
  const nn = normalize(n);
  const uu = normalize(sub(u, [nn[0] * dot(u, nn), nn[1] * dot(u, nn), nn[2] * dot(u, nn)]));
  const vv = cross(nn, uu);
  return {
    m: 1,
    weight,
    hard,
    tag,
    eval(x, out, off) {
      const w = sub(getP(x, tip), getP(x, pivot));
      out[off] = wrapAngle(Math.atan2(dot(w, vv), dot(w, uu)) - theta) * Lc;
    },
    jac(x, row, push) {
      const w = sub(getP(x, tip), getP(x, pivot));
      const a = dot(w, uu);
      const b = dot(w, vv);
      const den = Math.max(a * a + b * b, 1e-12);
      const g: Vec3 = [((a * vv[0] - b * uu[0]) / den) * Lc, ((a * vv[1] - b * uu[1]) / den) * Lc, ((a * vv[2] - b * uu[2]) / den) * Lc];
      pushGrad(push, row, tip, g, 1);
      pushGrad(push, row, pivot, g, -1);
    },
  };
}

/**
 * Generic numeric Jacobian (central differences) over the variable columns touched by refs. For a `periodic` f (an
 * angle wrapped to (−π, π]) the difference is wrapped as well, so the derivative stays finite when the two samples
 * straddle the ±π discontinuity instead of blowing up to ±2π/2h.
 */
function numericJac(refs: PRef[], f: (x: Float64Array) => number, Lc: number, periodic = false) {
  const cols = new Set<number>();
  for (const r of refs) for (const c of r.cols) if (c >= 0) cols.add(c);
  const colList = [...cols];
  return (x: Float64Array, row: number, push: JacPush) => {
    const h = 1e-6 * Math.max(1, Lc);
    for (const c of colList) {
      const old = x[c];
      x[c] = old + h;
      const fp = f(x);
      x[c] = old - h;
      const fm = f(x);
      x[c] = old;
      const df = periodic ? wrapAngle(fp - fm) : fp - fm;
      push(row, c, (df / (2 * h)) * Lc);
    }
  };
}

/** Dihedral (fold) angle about edge a->b between the faces containing p and q equals theta (radians). */
export function dihedral(a: PRef, b: PRef, p: PRef, q: PRef, theta: number, Lc: number, weight = 1, hard = true, tag = 'fold'): Constraint {
  const f = (x: Float64Array) => wrapAngle(dihedralAngle(getP(x, a), getP(x, b), getP(x, p), getP(x, q)) - theta);
  return {
    m: 1,
    weight,
    hard,
    tag,
    eval(x, out, off) {
      out[off] = f(x) * Lc;
    },
    jac: numericJac([a, b, p, q], f, Lc, true),
  };
}

/** Rotation of body B about the axis (a,b) of body A, measured between refA and refB projected on the axis plane. */
export function axisRotation(a: PRef, b: PRef, refA: PRef, refB: PRef, x: Float64Array): number {
  const pa = getP(x, a);
  const e = normalize(sub(getP(x, b), pa));
  const ra = sub(getP(x, refA), pa);
  const rb = sub(getP(x, refB), pa);
  const pra = sub(ra, [e[0] * dot(ra, e), e[1] * dot(ra, e), e[2] * dot(ra, e)]);
  const prb = sub(rb, [e[0] * dot(rb, e), e[1] * dot(rb, e), e[2] * dot(rb, e)]);
  return Math.atan2(dot(cross(pra, prb), e), dot(pra, prb));
}

/**
 * Screw coupling: translation of B along A's axis minus pitch * rotation/(2π) is constant.
 * (slide - s0) - pitch * unwrap(angle - phi0) / (2π) = 0.
 * The rotation is unwrapped relative to the previous evaluation so multi-turn screws work.
 */
export function screwCoupling(a: PRef, b: PRef, pB: PRef, refA: PRef, refB: PRef, pitch: number, s0: number, phi0: number, Lc: number, weight = 1, hard = true, tag = 'screw'): Constraint {
  let lastPhi = 0;
  const f = (x: Float64Array) => {
    const pa = getP(x, a);
    const e = normalize(sub(getP(x, b), pa));
    const s = dot(sub(getP(x, pB), pa), e);
    let phi = axisRotation(a, b, refA, refB, x) - phi0;
    // unwrap toward lastPhi
    while (phi - lastPhi > Math.PI) phi -= 2 * Math.PI;
    while (phi - lastPhi < -Math.PI) phi += 2 * Math.PI;
    return s - s0 - (pitch * phi) / (2 * Math.PI);
  };
  return {
    m: 1,
    weight,
    hard,
    tag,
    eval(x, out, off) {
      out[off] = f(x);
      // update unwrap reference
      let phi = axisRotation(a, b, refA, refB, x) - phi0;
      while (phi - lastPhi > Math.PI) phi -= 2 * Math.PI;
      while (phi - lastPhi < -Math.PI) phi += 2 * Math.PI;
      lastPhi = phi;
    },
    jac: numericJac([a, b, pB, refA, refB], f, 1),
  };
}

/** Helper: a constant reference point perpendicular to a world axis (for screw joints against construction axes). */
export function worldAxisRef(o: Vec3, d: Vec3, r = 1): Vec3 {
  const p = perpendicular(normalize(d));
  return [o[0] + p[0] * r, o[1] + p[1] * r, o[2] + p[2] * r];
}

// ---------------------------------------------------------------------------
// Length-independent body attachments (used for joint-axis helper points)
// ---------------------------------------------------------------------------

/** cos(angle between (h-p) and (q-p)) - value = 0, scaled by Lc. */
export function cosAngle(h: PRef, p: PRef, q: PRef, value: number, Lc: number, weight = 1, hard = true, tag = 'cos'): Constraint {
  return {
    m: 1,
    weight,
    hard,
    tag,
    eval(x, out, off) {
      const pp = getP(x, p);
      const u = sub(getP(x, h), pp);
      const w = sub(getP(x, q), pp);
      const lu = Math.max(len(u), 1e-12);
      const lw = Math.max(len(w), 1e-12);
      out[off] = (dot(u, w) / (lu * lw) - value) * Lc;
    },
    jac(x, row, push) {
      const pp = getP(x, p);
      const u = sub(getP(x, h), pp);
      const w = sub(getP(x, q), pp);
      const lu = Math.max(len(u), 1e-12);
      const lw = Math.max(len(w), 1e-12);
      const uw = dot(u, w);
      const gu: Vec3 = [
        (w[0] / (lu * lw) - (uw * u[0]) / (lu ** 3 * lw)) * Lc,
        (w[1] / (lu * lw) - (uw * u[1]) / (lu ** 3 * lw)) * Lc,
        (w[2] / (lu * lw) - (uw * u[2]) / (lu ** 3 * lw)) * Lc,
      ];
      const gw: Vec3 = [
        (u[0] / (lu * lw) - (uw * w[0]) / (lu * lw ** 3)) * Lc,
        (u[1] / (lu * lw) - (uw * w[1]) / (lu * lw ** 3)) * Lc,
        (u[2] / (lu * lw) - (uw * w[2]) / (lu * lw ** 3)) * Lc,
      ];
      pushGrad(push, row, h, gu);
      pushGrad(push, row, q, gw);
      pushGrad(push, row, p, [-(gu[0] + gw[0]), -(gu[1] + gw[1]), -(gu[2] + gw[2])]);
    },
  };
}

/** (b-a) × (d-c) / scale = 0 (3 residuals, rank 2): the two vectors stay parallel. */
export function parallelVectors(a: PRef, b: PRef, c: PRef, d: PRef, scale: number, weight = 1, hard = true, tag = 'parallel'): Constraint {
  const inv = 1 / Math.max(scale, 1e-12);
  return {
    m: 3,
    weight,
    hard,
    tag,
    eval(x, out, off) {
      const u = sub(getP(x, b), getP(x, a));
      const w = sub(getP(x, d), getP(x, c));
      const r = cross(u, w);
      out[off] = r[0] * inv;
      out[off + 1] = r[1] * inv;
      out[off + 2] = r[2] * inv;
    },
    jac(x, row, push) {
      const u = sub(getP(x, b), getP(x, a));
      const w = sub(getP(x, d), getP(x, c));
      const Su = skewRows(u);
      const Sw = skewRows(w);
      for (let i = 0; i < 3; i++) {
        for (let j = 0; j < 3; j++) {
          const du = -Sw[i][j]; // ∂(u×w)_i/∂u_j = -[w]×
          const dw = Su[i][j]; // ∂(u×w)_i/∂w_j = [u]×
          if (b.cols[j] >= 0) push(row + i, b.cols[j], du * inv);
          if (a.cols[j] >= 0) push(row + i, a.cols[j], -du * inv);
          if (d.cols[j] >= 0) push(row + i, d.cols[j], dw * inv);
          if (c.cols[j] >= 0) push(row + i, c.cols[j], -dw * inv);
        }
      }
    },
  };
}

/** Normalised triple product ((b-a) × (d-c)) · (c-a) - value = 0 (fixes the twist between two helper offsets). */
export function twist(a: PRef, b: PRef, c: PRef, d: PRef, value: number, Lc: number, weight = 1, hard = true, tag = 'twist'): Constraint {
  const f = (x: Float64Array) => {
    const pa = getP(x, a);
    const pc = getP(x, c);
    const u = sub(getP(x, b), pa);
    const w = sub(getP(x, d), pc);
    const e = sub(pc, pa);
    const den = Math.max(len(u) * len(w) * len(e), 1e-12);
    return dot(cross(u, w), e) / den - value;
  };
  return {
    m: 1,
    weight,
    hard,
    tag,
    eval(x, out, off) {
      out[off] = f(x) * Lc;
    },
    jac: numericJac([a, b, c, d], f, Lc),
  };
}

/** Current normalised twist value (used when capturing rigidity at creation). */
export function twistValue(a: Vec3, b: Vec3, c: Vec3, d: Vec3): number {
  const u = sub(b, a);
  const w = sub(d, c);
  const e = sub(c, a);
  const den = Math.max(len(u) * len(w) * len(e), 1e-12);
  return dot(cross(u, w), e) / den;
}

/** Current cosine between (h-p) and (q-p). */
export function cosValue(h: Vec3, p: Vec3, q: Vec3): number {
  const u = sub(h, p);
  const w = sub(q, p);
  return dot(u, w) / Math.max(len(u) * len(w), 1e-12);
}
