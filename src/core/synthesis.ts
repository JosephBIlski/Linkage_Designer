/**
 * Inverse design ("synthesis") of mechanisms from editing-point constraints.
 *
 * Formulation (see docs/DESIGN_DECISIONS.md): all sampled poses of the motion
 * are solved simultaneously as one constraint system in which the design
 * parameters (unlocked link lengths / polygon shapes, unlocked ground pivots)
 * are shared across poses and the input (driver) value of each pose is
 * prescribed. Editing-point targets are hard constraints on individual poses.
 * The remaining design freedom is the nullity of the stacked Jacobian.
 *
 * This is the "multi-pose / stacked" approach used by Coros et al. 2013 and
 * Thomaszewski et al. 2014 for linkage characters, and corresponds to path
 * generation with prescribed timing in classical kinematic synthesis.
 */
import { compile, type CompiledSystem } from './compile';
import { centroid, dist } from './geometry';
import { groundLink, linkAllPointIds } from './model';
import { degreesOfFreedom, denseJacobian, evaluate, nullSpace, solve } from './solver';
import type { ID, Model, Target, Vec3 } from './types';
import type { Pose, Positions } from './kinematics';
import { computeMobility, modelSize } from './kinematics';

export interface DesignInput {
  model: Model;
  /** Driver values per pose (driverValues[pose][driverIndex]). */
  poseValues: number[][];
  /** Warm-start positions per pose. */
  initPositions: Positions[];
  targets?: Target[];
  soft: boolean;
  maxIter?: number;
}

export interface DesignResult {
  converged: boolean;
  residual: number;
  iterations: number;
  poses: Pose[];
  sys: CompiledSystem;
  x: Float64Array;
}

/** Solve the stacked inverse-design problem. Does not modify the model; use applyDesign(). */
export function solveDesign(input: DesignInput): DesignResult {
  const m = input.model;
  const P = input.poseValues.length;
  const sys = compile(m, {
    mode: 'design',
    poses: P,
    driverValues: input.poseValues,
    posePositions: input.initPositions,
    soft: input.soft,
    targets: input.targets,
    includeTargets: true,
  });
  const maxIter = input.maxIter ?? 60;
  // Phase 1: regularised solve (soft assumptions pull toward the current design).
  let x = sys.x0;
  let iterations = 0;
  if (input.soft) {
    const r1 = solve(sys, x, { maxIter, tol: 1e-7 });
    x = r1.x;
    iterations += r1.iterations;
  }
  // Phase 2: exact projection onto the hard constraints (minimum-norm step from the regularised point).
  const hardSys = compile(m, {
    mode: 'design',
    poses: P,
    driverValues: input.poseValues,
    posePositions: input.initPositions,
    soft: false,
    targets: input.targets,
    includeTargets: true,
  });
  const res = solve(hardSys, x, { maxIter, tol: 1e-7 });
  iterations += res.iterations;
  const poses: Pose[] = [];
  for (let k = 0; k < P; k++) {
    poses.push({ value: input.poseValues[k][0] ?? 0, positions: hardSys.extract(res.x, k), converged: res.converged });
  }
  return { converged: res.converged, residual: res.hardResidual, iterations, poses, sys: hardSys, x: res.x };
}

/**
 * Write the design found by solveDesign back into the model: rest lengths of
 * unlocked links from pose 0 and ground pivot positions from the shared
 * variables. Returns the ids of links whose geometry changed.
 */
export function applyDesign(m: Model, result: DesignResult): ID[] {
  const changed: ID[] = [];
  const pose0 = result.poses[0].positions;
  const ground = groundLink(m);
  for (const link of Object.values(m.links)) {
    if (link.locked) continue;
    if (link.ground) {
      if (!ground) continue;
      let moved = false;
      for (const id of linkAllPointIds(link)) {
        const p = pose0.get(id);
        if (p && dist(p, m.points[id].pos) > 1e-12) {
          m.points[id].pos = [p[0], p[1], p[2]];
          moved = true;
        }
      }
      if (moved) changed.push(link.id);
      continue;
    }
    let touched = false;
    for (const r of link.rigidity) {
      if (r.kind === 'dist' && !r.fixed) {
        const a = pose0.get(r.a);
        const b = pose0.get(r.b);
        if (a && b) {
          const L = dist(a, b);
          if (Math.abs(L - r.length) > 1e-12) {
            r.length = L;
            touched = true;
          }
        }
      }
    }
    if (touched) changed.push(link.id);
  }
  return changed;
}

// ---------------------------------------------------------------------------
// Design freedom analysis
// ---------------------------------------------------------------------------

export interface PointDesignSpace {
  pointId: ID;
  /** Dimension of the (first-order) design space of this point: 0 = fully determined, 1 = curve, 2 = surface, 3 = volume. */
  dim: number;
  /** Principal directions spanning the design space (unit vectors, length = dim). */
  dirs: Vec3[];
  /** Per-pose local freedom (dimension + directions) used for the small indicators at editing points. */
  perPose: { dim: number; dirs: Vec3[] }[];
  /** Suggested display centre and radius for the design-space surface. */
  center: Vec3;
  radius: number;
}

export interface DesignAnalysis {
  /** Nullity of the stacked hard Jacobian minus un-driven motion freedom. */
  designDOF: number;
  nullity: number;
  motionDOF: number;
  driverCount: number;
  targetCount: number;
  /** Targets are inconsistent / unreachable (residual did not converge). */
  overConstrained: boolean;
  spaces: PointDesignSpace[];
}

/** Analyse remaining design freedom and the design space of the given points. */
export function analyseDesign(m: Model, result: DesignResult, pointIds: ID[]): DesignAnalysis {
  const sys = result.sys;
  const mob = computeMobility(m);
  const undriven = Math.max(0, mob.dof - m.drivers.length);
  const { dof: nullity } = degreesOfFreedom(sys, result.x, 1e-7);
  const designDOF = Math.max(0, nullity - sys.poses * undriven);
  const spaces: PointDesignSpace[] = [];
  if (pointIds.length > 0 && sys.n > 0) {
    const ev = evaluate(sys, result.x);
    const { J, m: rows } = denseJacobian(ev, sys.n, true);
    const { N, k } = nullSpace(J, rows, sys.n, 1e-7);
    const size = modelSize(m);
    const ground = groundLink(m);
    const groundPts = ground ? linkAllPointIds(ground).map((id) => m.points[id].pos) : sys.groups.filter((g) => g.constant && g.constPos).map((g) => g.constPos!);
    for (const pid of pointIds) {
      if (!m.points[pid]) continue;
      const perPose: { dim: number; dirs: Vec3[] }[] = [];
      const G = new Float64Array(9); // accumulated Gram matrix over poses
      const path: Vec3[] = [];
      for (let pose = 0; pose < sys.poses; pose++) {
        const ref = sys.ref(pose, pid);
        path.push(result.poses[pose].positions.get(pid)!);
        // M = rows of N for this point's columns (3 × k)
        const Mg = new Float64Array(9);
        for (let a = 0; a < 3; a++) {
          const ca = ref.cols[a];
          if (ca < 0) continue;
          for (let b = 0; b < 3; b++) {
            const cb = ref.cols[b];
            if (cb < 0) continue;
            let s = 0;
            for (let j = 0; j < k; j++) s += N[ca * k + j] * N[cb * k + j];
            Mg[a * 3 + b] = s;
          }
        }
        const eig = symmetricEigen3(Mg);
        const dirs: Vec3[] = [];
        for (let i = 0; i < 3; i++) if (eig.values[i] > 1e-8) dirs.push(eig.vectors[i]);
        perPose.push({ dim: dirs.length, dirs });
        for (let i = 0; i < 9; i++) G[i] += Mg[i];
      }
      const eigAll = symmetricEigen3(G);
      const dirs: Vec3[] = [];
      for (let i = 0; i < 3; i++) if (eigAll.values[i] > 1e-8) dirs.push(eigAll.vectors[i]);
      const center = groundPts.length ? centroid(groundPts) : centroid(path);
      let reach = 0;
      for (const p of path) reach = Math.max(reach, dist(p, center));
      const radius = Math.max(reach * 1.1, size * 0.2);
      spaces.push({ pointId: pid, dim: dirs.length, dirs, perPose, center, radius });
    }
  }
  return {
    designDOF,
    nullity,
    motionDOF: mob.dof,
    driverCount: m.drivers.length,
    targetCount: (m.targets ?? []).length,
    overConstrained: !result.converged,
    spaces,
  };
}

/** Jacobi eigen-decomposition of a symmetric 3×3 matrix (row-major). Values sorted descending. */
export function symmetricEigen3(A: Float64Array): { values: number[]; vectors: Vec3[] } {
  const a = Float64Array.from(A);
  const V = new Float64Array([1, 0, 0, 0, 1, 0, 0, 0, 1]);
  for (let sweep = 0; sweep < 50; sweep++) {
    let off = 0;
    for (let p = 0; p < 3; p++) for (let q = p + 1; q < 3; q++) off += a[p * 3 + q] ** 2;
    if (off < 1e-24) break;
    for (let p = 0; p < 3; p++) {
      for (let q = p + 1; q < 3; q++) {
        const apq = a[p * 3 + q];
        if (Math.abs(apq) < 1e-30) continue;
        const theta = (a[q * 3 + q] - a[p * 3 + p]) / (2 * apq);
        const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
        const c = 1 / Math.sqrt(t * t + 1);
        const s = t * c;
        for (let k = 0; k < 3; k++) {
          const akp = a[k * 3 + p];
          const akq = a[k * 3 + q];
          a[k * 3 + p] = c * akp - s * akq;
          a[k * 3 + q] = s * akp + c * akq;
        }
        for (let k = 0; k < 3; k++) {
          const apk = a[p * 3 + k];
          const aqk = a[q * 3 + k];
          a[p * 3 + k] = c * apk - s * aqk;
          a[q * 3 + k] = s * apk + c * aqk;
        }
        for (let k = 0; k < 3; k++) {
          const vkp = V[k * 3 + p];
          const vkq = V[k * 3 + q];
          V[k * 3 + p] = c * vkp - s * vkq;
          V[k * 3 + q] = s * vkp + c * vkq;
        }
      }
    }
  }
  const idx = [0, 1, 2].sort((i, j) => a[j * 3 + j] - a[i * 3 + i]);
  return {
    values: idx.map((i) => a[i * 3 + i]),
    vectors: idx.map((i) => [V[i], V[3 + i], V[6 + i]] as Vec3),
  };
}

