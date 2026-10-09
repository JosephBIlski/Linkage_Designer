/**
 * Dense numerical linear algebra and the Levenberg–Marquardt solver used to
 * project configurations onto the constraint manifold.
 *
 * Design notes (see docs/DESIGN_DECISIONS.md):
 *  - Newton-type projection with a damped pseudo-inverse of the constraint
 *    Jacobian, in the spirit of Tachi's Rigid Origami Simulator and Crane.
 *  - Degrees of freedom are computed numerically as (#variables − rank J),
 *    which, unlike the Grübler–Kutzbach count, is correct for over-constrained
 *    and paradoxical mechanisms (Bennett linkage, flat-foldable vertices, …).
 *  - Systems are small (tens to a few hundred variables), so dense Cholesky
 *    and Householder QR are fast enough and much simpler than sparse solvers.
 */
import type { Constraint } from './constraints';

export interface CompiledSystemLike {
  n: number;
  constraints: Constraint[];
}

export interface Triplet {
  row: number;
  col: number;
  val: number;
}

export interface Evaluation {
  r: Float64Array;
  rows: { cols: number[]; vals: number[] }[];
  m: number;
  /** Row index ranges that belong to hard constraints. */
  hardRows: Uint8Array;
}

export function totalResiduals(cs: Constraint[]): number {
  let m = 0;
  for (const c of cs) m += c.m;
  return m;
}

/** Evaluate residuals and sparse Jacobian (weights applied). */
export function evaluate(sys: CompiledSystemLike, x: Float64Array): Evaluation {
  const m = totalResiduals(sys.constraints);
  const r = new Float64Array(m);
  const rows: { cols: number[]; vals: number[] }[] = new Array(m);
  const hardRows = new Uint8Array(m);
  for (let i = 0; i < m; i++) rows[i] = { cols: [], vals: [] };
  let off = 0;
  for (const c of sys.constraints) {
    c.eval(x, r, off);
    if (c.weight !== 1) for (let i = 0; i < c.m; i++) r[off + i] *= c.weight;
    const w = c.weight;
    c.jac(x, off, (row, col, val) => {
      const rr = rows[row];
      rr.cols.push(col);
      rr.vals.push(val * w);
    });
    if (c.hard) for (let i = 0; i < c.m; i++) hardRows[off + i] = 1;
    off += c.m;
  }
  return { r, rows, m, hardRows };
}

export function residualOnly(sys: CompiledSystemLike, x: Float64Array): Float64Array {
  const m = totalResiduals(sys.constraints);
  const r = new Float64Array(m);
  let off = 0;
  for (const c of sys.constraints) {
    c.eval(x, r, off);
    if (c.weight !== 1) for (let i = 0; i < c.m; i++) r[off + i] *= c.weight;
    off += c.m;
  }
  return r;
}

export function maxAbs(r: Float64Array, mask?: Uint8Array): number {
  let mx = 0;
  for (let i = 0; i < r.length; i++) {
    if (mask && !mask[i]) continue;
    const a = Math.abs(r[i]);
    if (a > mx) mx = a;
  }
  return mx;
}

// ---------------------------------------------------------------------------
// Dense helpers
// ---------------------------------------------------------------------------

/** Build A = JᵀJ (n×n row-major) and g = Jᵀr from the sparse rows. */
export function normalEquations(ev: Evaluation, n: number): { A: Float64Array; g: Float64Array } {
  const A = new Float64Array(n * n);
  const g = new Float64Array(n);
  for (let i = 0; i < ev.m; i++) {
    const { cols, vals } = ev.rows[i];
    const ri = ev.r[i];
    for (let a = 0; a < cols.length; a++) {
      const ca = cols[a];
      const va = vals[a];
      g[ca] += va * ri;
      for (let b = 0; b < cols.length; b++) {
        A[ca * n + cols[b]] += va * vals[b];
      }
    }
  }
  return { A, g };
}

/** In-place Cholesky solve of (A) x = b. Returns null if A is not positive definite. */
export function choleskySolve(A: Float64Array, b: Float64Array, n: number): Float64Array | null {
  const L = new Float64Array(n * n);
  for (let j = 0; j < n; j++) {
    let s = A[j * n + j];
    for (let k = 0; k < j; k++) s -= L[j * n + k] * L[j * n + k];
    if (!(s > 1e-18)) return null;
    const ljj = Math.sqrt(s);
    L[j * n + j] = ljj;
    for (let i = j + 1; i < n; i++) {
      let t = A[i * n + j];
      for (let k = 0; k < j; k++) t -= L[i * n + k] * L[j * n + k];
      L[i * n + j] = t / ljj;
    }
  }
  // forward: L y = b
  const y = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    let s = b[i];
    for (let k = 0; k < i; k++) s -= L[i * n + k] * y[k];
    y[i] = s / L[i * n + i];
  }
  // backward: Lᵀ x = y
  const xo = new Float64Array(n);
  for (let i = n - 1; i >= 0; i--) {
    let s = y[i];
    for (let k = i + 1; k < n; k++) s -= L[k * n + i] * xo[k];
    xo[i] = s / L[i * n + i];
  }
  return xo;
}

/** Dense m×n row-major matrix from sparse rows (optionally only hard rows). */
export function denseJacobian(ev: Evaluation, n: number, hardOnly = false): { J: Float64Array; m: number } {
  const rowsIdx: number[] = [];
  for (let i = 0; i < ev.m; i++) if (!hardOnly || ev.hardRows[i]) rowsIdx.push(i);
  const m = rowsIdx.length;
  const J = new Float64Array(m * n);
  for (let k = 0; k < m; k++) {
    const { cols, vals } = ev.rows[rowsIdx[k]];
    for (let a = 0; a < cols.length; a++) J[k * n + cols[a]] += vals[a];
  }
  return { J, m };
}

/**
 * Rank-revealing Householder QR with column pivoting.
 * Returns rank, the permutation and the upper-triangular factor R (m×n, row-major).
 */
export function pivotedQR(Jin: Float64Array, m: number, n: number, relTol = 1e-9): { rank: number; perm: number[]; R: Float64Array } {
  const R = Float64Array.from(Jin);
  const perm = Array.from({ length: n }, (_, i) => i);
  const colNorm = new Float64Array(n);
  for (let j = 0; j < n; j++) {
    let s = 0;
    for (let i = 0; i < m; i++) s += R[i * n + j] ** 2;
    colNorm[j] = s;
  }
  let maxNorm0 = 0;
  for (let j = 0; j < n; j++) maxNorm0 = Math.max(maxNorm0, colNorm[j]);
  const tol2 = Math.max(maxNorm0, 1e-300) * relTol * relTol;
  const kmax = Math.min(m, n);
  let rank = 0;
  for (let k = 0; k < kmax; k++) {
    // pivot: column with largest remaining norm
    let best = k;
    let bestNorm = -1;
    for (let j = k; j < n; j++) {
      let s = 0;
      for (let i = k; i < m; i++) s += R[i * n + j] ** 2;
      colNorm[j] = s;
      if (s > bestNorm) {
        bestNorm = s;
        best = j;
      }
    }
    if (bestNorm <= tol2) break;
    if (best !== k) {
      for (let i = 0; i < m; i++) {
        const t = R[i * n + k];
        R[i * n + k] = R[i * n + best];
        R[i * n + best] = t;
      }
      const tp = perm[k];
      perm[k] = perm[best];
      perm[best] = tp;
    }
    // Householder on column k, rows k..m-1
    let norm = 0;
    for (let i = k; i < m; i++) norm += R[i * n + k] ** 2;
    norm = Math.sqrt(norm);
    const alpha = R[k * n + k] > 0 ? -norm : norm;
    const v = new Float64Array(m - k);
    for (let i = k; i < m; i++) v[i - k] = R[i * n + k];
    v[0] -= alpha;
    let vnorm = 0;
    for (let i = 0; i < v.length; i++) vnorm += v[i] ** 2;
    if (vnorm > 1e-300) {
      for (let j = k; j < n; j++) {
        let s = 0;
        for (let i = k; i < m; i++) s += v[i - k] * R[i * n + j];
        const f = (2 * s) / vnorm;
        for (let i = k; i < m; i++) R[i * n + j] -= f * v[i - k];
      }
    }
    rank++;
  }
  return { rank, perm, R };
}

/** Numerical rank of the dense m×n matrix. */
export function matrixRank(J: Float64Array, m: number, n: number, relTol = 1e-9): number {
  if (m === 0 || n === 0) return 0;
  return pivotedQR(J, m, n, relTol).rank;
}

/**
 * Orthonormal basis of the null space of the dense m×n matrix J (columns of the
 * returned n×k row-major matrix). Uses the pivoted QR factor: with J Π = Q R and
 * R = [R11 R12; 0 0], the null space is Π [ -R11⁻¹ R12 ; I ].
 */
export function nullSpace(J: Float64Array, m: number, n: number, relTol = 1e-9): { N: Float64Array; k: number } {
  if (n === 0) return { N: new Float64Array(0), k: 0 };
  if (m === 0) {
    const N = new Float64Array(n * n);
    for (let i = 0; i < n; i++) N[i * n + i] = 1;
    return { N, k: n };
  }
  const { rank, perm, R } = pivotedQR(J, m, n, relTol);
  const k = n - rank;
  const N = new Float64Array(n * k);
  if (k === 0) return { N, k };
  // For each free column f (rank..n-1), solve R11 z = -R12[:, f]
  for (let f = 0; f < k; f++) {
    const col = rank + f;
    const z = new Float64Array(rank);
    for (let i = rank - 1; i >= 0; i--) {
      let s = -R[i * n + col];
      for (let j = i + 1; j < rank; j++) s -= R[i * n + j] * z[j];
      const d = R[i * n + i];
      z[i] = Math.abs(d) > 1e-300 ? s / d : 0;
    }
    // assemble permuted vector
    const v = new Float64Array(n);
    for (let i = 0; i < rank; i++) v[perm[i]] = z[i];
    v[perm[col]] = 1;
    // Gram–Schmidt against previous columns
    for (let p = 0; p < f; p++) {
      let d = 0;
      for (let i = 0; i < n; i++) d += v[i] * N[i * k + p];
      for (let i = 0; i < n; i++) v[i] -= d * N[i * k + p];
    }
    let nv = 0;
    for (let i = 0; i < n; i++) nv += v[i] * v[i];
    nv = Math.sqrt(nv) || 1;
    for (let i = 0; i < n; i++) N[i * k + f] = v[i] / nv;
  }
  return { N, k };
}

// ---------------------------------------------------------------------------
// Levenberg–Marquardt
// ---------------------------------------------------------------------------

export interface SolveOptions {
  maxIter?: number;
  /** Convergence tolerance on the max hard residual (length units). */
  tol?: number;
  /** Initial damping. */
  lambda?: number;
  /** Maximum allowed step length per iteration (guards against jumping branches). */
  maxStep?: number;
}

export interface SolveResult {
  x: Float64Array;
  /** 0.5 ||r||² at the solution. */
  cost: number;
  /** Max |r_i| over hard constraints (unweighted scale ≈ length units). */
  hardResidual: number;
  converged: boolean;
  iterations: number;
}

/** Fraction of the largest diagonal entry of JᵀJ below which the Levenberg–Marquardt damping of a column is floored. */
export const DAMPING_FLOOR = 1e-3;

export function solve(sys: CompiledSystemLike, x0: Float64Array, opts: SolveOptions = {}): SolveResult {
  const n = sys.n;
  const maxIter = opts.maxIter ?? 50;
  const tol = opts.tol ?? 1e-8;
  let lambda = opts.lambda ?? 1e-3;
  const maxStep = opts.maxStep ?? Infinity;
  let x = Float64Array.from(x0);
  if (n === 0) {
    // nothing to solve for, but the constraints can still be violated (e.g. a rest length between two frozen
    // points): converged only when the hard residual meets the same criterion as a solved system
    const ev0 = evaluate(sys, x);
    const hard = maxAbs(ev0.r, ev0.hardRows);
    return { x, cost: 0.5 * dot(ev0.r, ev0.r), hardResidual: hard, converged: hard < tol * 10, iterations: 0 };
  }
  let ev = evaluate(sys, x);
  let cost = 0.5 * dot(ev.r, ev.r);
  let iterations = 0;
  // Converged when the hard constraints are satisfied AND the least-squares
  // gradient Jᵀr (which includes soft / compliant terms) vanishes.
  const gtol = tol * 1e-2;
  let { A, g } = normalEquations(ev, n);
  let converged = maxAbs(ev.r, ev.hardRows) < tol && maxAbs(g) < gtol;
  // Damping: Marquardt's diagonal scaling (λ·A_ii) keeps the step scale-aware, but a column whose Jacobian entries
  // are nearly zero (a coordinate the residuals hardly depend on, e.g. the x of a point on an almost vertical
  // distance) would be left almost undamped and the step would move it by a large amount for no gain, so the
  // scaling is floored at a small fraction of the largest diagonal entry: in such directions the step is then the
  // minimum-norm (damped pseudo-inverse) step, which is what a projection onto the constraint manifold wants.
  const dampFloor = () => {
    let mx = 0;
    for (let i = 0; i < n; i++) mx = Math.max(mx, A[i * n + i]);
    return DAMPING_FLOOR * mx + 1e-9;
  };
  let floor = dampFloor();
  while (!converged && iterations < maxIter) {
    iterations++;
    let accepted = false;
    for (let attempt = 0; attempt < 12 && !accepted; attempt++) {
      const Ad = Float64Array.from(A);
      for (let i = 0; i < n; i++) Ad[i * n + i] += lambda * (A[i * n + i] + floor) + 1e-12;
      const negg = new Float64Array(n);
      for (let i = 0; i < n; i++) negg[i] = -g[i];
      const dx = choleskySolve(Ad, negg, n);
      if (!dx) {
        lambda *= 10;
        continue;
      }
      // step limiting
      let stepMax = 0;
      for (let i = 0; i < n; i++) stepMax = Math.max(stepMax, Math.abs(dx[i]));
      if (stepMax > maxStep) {
        const s = maxStep / stepMax;
        for (let i = 0; i < n; i++) dx[i] *= s;
      }
      const xn = new Float64Array(n);
      for (let i = 0; i < n; i++) xn[i] = x[i] + dx[i];
      const rn = residualOnly(sys, xn);
      const costn = 0.5 * dot(rn, rn);
      if (costn <= cost || stepMax < 1e-14) {
        x = xn;
        cost = costn;
        lambda = Math.max(lambda / 3, 1e-12);
        accepted = true;
        if (stepMax < 1e-12) {
          ev = evaluate(sys, x);
          converged = true;
        }
      } else {
        lambda *= 4;
      }
    }
    if (!accepted) break;
    if (!converged) {
      ev = evaluate(sys, x);
      ({ A, g } = normalEquations(ev, n));
      floor = dampFloor();
      converged = maxAbs(ev.r, ev.hardRows) < tol && maxAbs(g) < gtol;
    }
  }
  const evFinal = evaluate(sys, x);
  return { x, cost, hardResidual: maxAbs(evFinal.r, evFinal.hardRows), converged: maxAbs(evFinal.r, evFinal.hardRows) < tol * 10, iterations };
}

function dot(a: Float64Array, b: Float64Array): number {
  let s = 0;
  for (let i = 0; i < a.length; i++) s += a[i] * b[i];
  return s;
}

/** Degrees of freedom = n − rank(J_hard) at x. */
export function degreesOfFreedom(sys: CompiledSystemLike, x: Float64Array, relTol = 1e-7): { dof: number; rank: number; n: number } {
  const ev = evaluate(sys, x);
  const { J, m } = denseJacobian(ev, sys.n, true);
  const rank = matrixRank(J, m, sys.n, relTol);
  return { dof: sys.n - rank, rank, n: sys.n };
}

/** Null-space basis of the hard constraints at x (n×k row-major). */
export function motionNullSpace(sys: CompiledSystemLike, x: Float64Array, relTol = 1e-7): { N: Float64Array; k: number } {
  const ev = evaluate(sys, x);
  const { J, m } = denseJacobian(ev, sys.n, true);
  return nullSpace(J, m, sys.n, relTol);
}
