/**
 * Forward kinematics: constraint projection for a given set of driver values,
 * numerical mobility (DOF) analysis and driver sweeps that discover the
 * reachable range of an input (crank vs. rocker) and record output paths.
 */
import { compile, type CompiledSystem, type DragTarget } from './compile';
import { dist, sub, len } from './geometry';
import { groundLink, refreshRigidity } from './model';
import { degreesOfFreedom, solve } from './solver';
import type { ID, Model, Vec3 } from './types';
import * as jointMeasure from './jointMeasure';

export type Positions = Map<ID, Vec3>;

export function positionsFromModel(m: Model): Positions {
  const out: Positions = new Map();
  for (const p of Object.values(m.points)) out.set(p.id, [p.pos[0], p.pos[1], p.pos[2]]);
  return out;
}

export function applyPositions(m: Model, pos: Positions): void {
  for (const [id, p] of pos) if (m.points[id]) m.points[id].pos = [p[0], p[1], p[2]];
}

/** Approximate size of the mechanism (bounding-box diagonal of all points). */
export function modelSize(m: Model, pos?: Positions): number {
  let lo: Vec3 = [Infinity, Infinity, Infinity];
  let hi: Vec3 = [-Infinity, -Infinity, -Infinity];
  const src = pos ?? positionsFromModel(m);
  for (const p of src.values()) {
    for (let i = 0; i < 3; i++) {
      lo[i] = Math.min(lo[i], p[i]);
      hi[i] = Math.max(hi[i], p[i]);
    }
  }
  if (!isFinite(lo[0])) return 1;
  return Math.max(len(sub(hi, lo)), 1e-3);
}

export interface ForwardResult {
  positions: Positions;
  converged: boolean;
  residual: number;
  iterations: number;
  sys: CompiledSystem;
}

/** Solve the assembly for the given driver values (degrees / units) starting from `init`. */
export function solveForward(m: Model, driverValues: number[], init?: Positions, opts: { maxIter?: number; maxStep?: number } = {}): ForwardResult {
  const sys = compile(m, { mode: 'forward', poses: 1, driverValues: [driverValues], posePositions: init ? [init] : undefined });
  const res = solve(sys, sys.x0, { maxIter: opts.maxIter ?? 60, tol: 1e-8, maxStep: opts.maxStep });
  return { positions: sys.extract(res.x, 0), converged: res.converged, residual: res.hardResidual, iterations: res.iterations, sys };
}

export interface MobilityResult {
  /** Degrees of freedom of the mechanism (not counting drivers). */
  dof: number;
  rank: number;
  variables: number;
  grounded: boolean;
  /** With no ground link the count includes the rigid-body modes of the floating assembly. */
  rigidBodyModes: number;
}

export function computeMobility(m: Model): MobilityResult {
  const sys = compile(m, { mode: 'forward', poses: 1, includeDrivers: false, flexibility: false });
  const { dof, rank, n } = degreesOfFreedom(sys, sys.x0);
  // grounded when a ground link exists or some point is pinned to construction geometry
  const grounded = groundLink(m) !== null || sys.groups.some((g) => g.constant);
  let rigidBodyModes = 0;
  if (!grounded && Object.keys(m.links).length > 0) {
    // planar sketches have 3 rigid-body modes, spatial assemblies 6 — detect by frozen z of every group
    const allFrozen = sys.groups.filter((g) => !g.constant).every((g) => g.frozen.some(Boolean));
    rigidBodyModes = allFrozen ? 3 : 6;
  }
  return { dof, rank, variables: n, grounded, rigidBodyModes };
}

/** Constraint violation of the current construction pose (max hard residual, length units). */
export function currentViolation(m: Model): number {
  const sys = compile(m, { mode: 'forward', poses: 1, includeDrivers: false, flexibility: false });
  const res = solve(sys, sys.x0, { maxIter: 0 });
  return res.hardResidual;
}

// ---------------------------------------------------------------------------
// Sketch (construction-mode) editing
// ---------------------------------------------------------------------------

export interface SketchOptions {
  dragTargets?: DragTarget[];
  freePointIds?: Set<ID>;
  allowGroundMove?: boolean;
  maxIter?: number;
}

/**
 * Re-satisfy all constraints after a construction-mode edit, pulling dragged
 * points toward their targets. Returns the solved positions (not applied).
 */
export function solveSketch(m: Model, opts: SketchOptions = {}): ForwardResult {
  const sys = compile(m, {
    mode: 'sketch',
    poses: 1,
    dragTargets: opts.dragTargets,
    freePointIds: opts.freePointIds,
    allowGroundMove: opts.allowGroundMove,
  });
  const res = solve(sys, sys.x0, { maxIter: opts.maxIter ?? 40, tol: 1e-8 });
  return { positions: sys.extract(res.x, 0), converged: res.converged, residual: res.hardResidual, iterations: res.iterations, sys };
}

/** Apply a sketch solve to the model and refresh the rest geometry of links whose points were released. */
export function commitSketch(m: Model, result: ForwardResult, freePointIds?: Set<ID>): void {
  applyPositions(m, result.positions);
  if (freePointIds) {
    const links = new Set<ID>();
    for (const pid of freePointIds) if (m.points[pid]) links.add(m.points[pid].linkId);
    for (const lid of links) refreshRigidity(m, m.links[lid]);
  }
  // keep driver values in sync with the construction pose
  syncDriverValues(m);
}

/** Update each driver's stored value from the current construction pose. */
export function syncDriverValues(m: Model): void {
  for (const d of m.drivers) {
    const v = measureDriver(m, d.id, positionsFromModel(m));
    if (v !== null) d.value = v;
  }
}

/** Measure a driver's value (deg / units) in a given set of positions. */
export function measureDriver(m: Model, driverId: ID, pos: Positions): number | null {
  const d = m.drivers.find((x) => x.id === driverId);
  if (!d) return null;
  if (d.kind === 'angle' && d.pivotId && d.tipId && d.axis && d.ref) {
    const tip = pos.get(d.tipId);
    const piv = pos.get(d.pivotId);
    if (!tip || !piv) return null;
    const w = sub(tip, piv);
    const n = d.axis;
    const u = normalizeAgainst(d.ref, n);
    const v: Vec3 = [n[1] * u[2] - n[2] * u[1], n[2] * u[0] - n[0] * u[2], n[0] * u[1] - n[1] * u[0]];
    return (Math.atan2(w[0] * v[0] + w[1] * v[1] + w[2] * v[2], w[0] * u[0] + w[1] * u[1] + w[2] * u[2]) * 180) / Math.PI;
  }
  // fold / slide: evaluate via a compiled driver constraint at value 0 would be circular; use geometry helpers
  const { measureJointDriver } = jointMeasure;
  return measureJointDriver(m, d, pos);
}

function normalizeAgainst(ref: Vec3, n: Vec3): Vec3 {
  const d = ref[0] * n[0] + ref[1] * n[1] + ref[2] * n[2];
  const u: Vec3 = [ref[0] - n[0] * d, ref[1] - n[1] * d, ref[2] - n[2] * d];
  const l = Math.hypot(u[0], u[1], u[2]) || 1;
  return [u[0] / l, u[1] / l, u[2] / l];
}

// ---------------------------------------------------------------------------
// Driver sweeps
// ---------------------------------------------------------------------------

export interface Pose {
  value: number;
  positions: Positions;
  converged: boolean;
}

export interface SweepResult {
  driverIndex: number;
  poses: Pose[]; // sorted by value
  range: [number, number];
  isCrank: boolean;
  /** Values of the other drivers held constant during the sweep. */
  heldValues: number[];
}

export interface SweepOptions {
  /** Step in degrees (angle/fold) or in world units (slide). */
  step?: number;
  maxSteps?: number;
  /** Maximum allowed displacement of any point in one step, as a fraction of the model size (branch-jump guard). */
  jumpFraction?: number;
}

/** Sweep driver `driverIndex` forward and backward from the current pose until assembly fails. */
export function sweepDriver(m: Model, driverIndex: number, opts: SweepOptions = {}): SweepResult {
  const d = m.drivers[driverIndex];
  const isAngle = d.kind !== 'slide';
  const size = modelSize(m);
  const step = opts.step ?? (isAngle ? 3 : size / 40);
  const maxSteps = opts.maxSteps ?? (isAngle ? Math.ceil(360 / step) + 2 : 200);
  const jumpTol = (opts.jumpFraction ?? 0.35) * size;
  const held = m.drivers.map((x) => x.value);
  const base = positionsFromModel(m);
  const v0 = d.value;

  const tryPose = (value: number, init: Positions, ref: Positions): Pose | null => {
    const vals = [...held];
    vals[driverIndex] = value;
    const res = solveForward(m, vals, init, { maxIter: 40, maxStep: jumpTol });
    if (!res.converged) return null;
    let maxMove = 0;
    for (const [id, p] of res.positions) {
      const q = ref.get(id);
      if (q) maxMove = Math.max(maxMove, dist(p, q));
    }
    if (maxMove > jumpTol) return null;
    return { value, positions: res.positions, converged: true };
  };

  /**
   * Secant extrapolation of the last two poses: warm-starting from the
   * extrapolated configuration keeps the sweep on its current branch when the
   * motion passes through a bifurcation (e.g. the flat state of an origami
   * vertex), instead of hopping onto a degenerate branch.
   */
  const extrapolate = (prev: Positions, prevprev: Positions | null, s: number, sPrev: number): Positions => {
    if (!prevprev || Math.abs(sPrev) < 1e-12) return prev;
    const f = s / sPrev;
    const out: Positions = new Map();
    for (const [id, p] of prev) {
      const q = prevprev.get(id);
      if (!q) {
        out.set(id, p);
        continue;
      }
      out.set(id, [p[0] + (p[0] - q[0]) * f, p[1] + (p[1] - q[1]) * f, p[2] + (p[2] - q[2]) * f]);
    }
    return out;
  };

  const sweepDir = (dir: 1 | -1): { poses: Pose[]; closed: boolean } => {
    const poses: Pose[] = [];
    let prev: Pose = { value: v0, positions: base, converged: true };
    let prevprev: Pose | null = null;
    let v = v0;
    let closed = false;
    for (let i = 0; i < maxSteps; i++) {
      let s = step * dir;
      let next: Pose | null = null;
      // bisection toward the limit when a step fails
      for (let h = 0; h < 5; h++) {
        const init = extrapolate(prev.positions, prevprev?.positions ?? null, s, prevprev ? prev.value - prevprev.value : 0);
        next = tryPose(v + s, init, prev.positions);
        if (next) break;
        s /= 2;
      }
      if (!next) break;
      poses.push(next);
      prevprev = prev;
      prev = next;
      v = next.value;
      if (Math.abs(s) < step * 0.9) {
        // we are at a limit: stop
        break;
      }
      if (isAngle && Math.abs(v - v0) >= 360 - 1e-9) {
        // full revolution: check we returned to the same branch
        let maxDev = 0;
        for (const [id, p] of next.positions) maxDev = Math.max(maxDev, dist(p, base.get(id)!));
        closed = maxDev < 1e-3 * size + 1e-6;
        break;
      }
    }
    return { poses, closed };
  };

  const fwd = sweepDir(1);
  let isCrank = fwd.closed;
  let all: Pose[] = [{ value: v0, positions: base, converged: true }, ...fwd.poses];
  if (!isCrank) {
    const bwd = sweepDir(-1);
    all = [...bwd.poses.reverse(), ...all];
    if (bwd.closed) isCrank = true;
  }
  if (isCrank) {
    // keep exactly one revolution starting at v0
    all = all.filter((p) => p.value >= v0 - 1e-9 && p.value < v0 + 360 - 1e-9);
    if (all.length === 0) all = [{ value: v0, positions: base, converged: true }];
  }
  all.sort((a, b) => a.value - b.value);
  const range: [number, number] = isCrank ? [v0, v0 + 360] : [all[0].value, all[all.length - 1].value];
  return { driverIndex, poses: all, range, isCrank, heldValues: held };
}

/** Evenly spaced driver values across the sweep range (a small margin keeps rockers away from their dead points). */
export function samplePoseValues(sweep: SweepResult, count: number, margin = 0.02): number[] {
  const [lo, hi] = sweep.range;
  const out: number[] = [];
  if (sweep.isCrank) {
    for (let i = 0; i < count; i++) out.push(lo + ((hi - lo) * i) / count);
  } else {
    const span = hi - lo;
    const a = lo + span * margin;
    const b = hi - span * margin;
    for (let i = 0; i < count; i++) out.push(count === 1 ? a : a + ((b - a) * i) / (count - 1));
  }
  return out;
}

/** Nearest sweep pose (by driver value), used as warm start. */
export function nearestPose(sweep: SweepResult, value: number): Pose {
  let best = sweep.poses[0];
  let bestD = Infinity;
  for (const p of sweep.poses) {
    let dv = Math.abs(p.value - value);
    if (sweep.isCrank) dv = Math.min(dv, Math.abs(dv - 360));
    if (dv < bestD) {
      bestD = dv;
      best = p;
    }
  }
  return best;
}

/** Solve the exact poses at the given driver values, warm-starting from the sweep. */
export function solvePosesAt(m: Model, sweep: SweepResult, values: number[]): Pose[] {
  return values.map((v) => {
    const init = nearestPose(sweep, v).positions;
    const vals = [...sweep.heldValues];
    vals[sweep.driverIndex] = v;
    const res = solveForward(m, vals, init, { maxIter: 60 });
    return { value: v, positions: res.positions, converged: res.converged };
  });
}

/**
 * Output *surface* for 2-DOF mechanisms: sweep driver A at several values of
 * driver B. Returns a grid of poses [iB][iA]. The model is restored afterwards.
 */
export function sweepGrid(m: Model, driverA: number, driverB: number, countB = 7, opts: SweepOptions = {}): Pose[][] {
  const saved = positionsFromModel(m);
  const savedValues = m.drivers.map((d) => d.value);
  const sweepB = sweepDriver(m, driverB, opts);
  const valuesB = samplePoseValues(sweepB, countB, 0.05);
  const posesB = solvePosesAt(m, sweepB, valuesB);
  const grid: Pose[][] = [];
  for (const pb of posesB) {
    if (!pb.converged) continue;
    applyPositions(m, pb.positions);
    m.drivers[driverB].value = pb.value;
    const sw = sweepDriver(m, driverA, opts);
    grid.push(sw.poses);
  }
  applyPositions(m, saved);
  m.drivers.forEach((d, i) => (d.value = savedValues[i]));
  return grid;
}

/** Trajectory of a point through a list of poses. */
export function pathOf(poses: Pose[], pointId: ID): Vec3[] {
  const out: Vec3[] = [];
  for (const p of poses) {
    const q = p.positions.get(pointId);
    if (q && p.converged) out.push(q);
  }
  return out;
}

