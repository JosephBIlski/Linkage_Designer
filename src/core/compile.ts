/**
 * Compile a Model into a solver System.
 *
 * Three modes share one code path:
 *  - 'forward': design fixed, ground constant, drivers prescribed → kinematic simulation.
 *  - 'sketch':  design fixed except released points, soft drag targets → construction-mode editing.
 *  - 'design':  multi-pose ("stacked") system for inverse design. Unlocked link
 *               lengths are shared free variables across poses, unlocked ground
 *               pivots are shared variables, drivers are prescribed per pose and
 *               editing-point targets are hard constraints.
 *
 * Coincident points (joint pairs) are merged into one variable; coordinates
 * constrained to axis-aligned construction planes are frozen (removed from the
 * variable set). This keeps systems small enough for dense linear algebra.
 */
import type { Constraint, PRef } from './constraints';
import {
  angleDriver,
  constRef,
  cosAngle,
  dihedral,
  distance,
  fixPosition,
  getP,
  parallelVectors,
  pointOnBodyLine,
  pointOnBodyPlane,
  pointOnLine,
  pointOnPlane,
  projectionLock,
  screwCoupling,
  sharedDistance,
  slideLock,
  twist,
  worldAxisRef,
  axisRotation,
} from './constraints';
import { cross, deg2rad, dist, dot, len, normalize, sub } from './geometry';
import { groundLink, jointAxisPoints, linkAllPointIds, offAxisPoint } from './model';
import type { ID, Joint, Model, Target, Vec3 } from './types';
import { isConstructionRef } from './types';

export type CompileMode = 'forward' | 'sketch' | 'design';

export interface DragTarget {
  pointId: ID;
  pos: Vec3;
  weight: number;
}

export interface CompileOptions {
  mode: CompileMode;
  poses?: number;
  /** driverValues[pose][driverIndex]; missing entries fall back to the model's current driver values. */
  driverValues?: number[][];
  /** Initial positions per pose (warm start). Defaults to the model's construction positions. */
  posePositions?: Map<ID, Vec3>[];
  /** sketch: points whose owning link's design distances are released (rubber-band editing). */
  freePointIds?: Set<ID>;
  /** sketch: soft position targets for dragged points. */
  dragTargets?: DragTarget[];
  /** sketch: let ground points move (dragging the ground link). */
  allowGroundMove?: boolean;
  /** design: add weak regularisers pulling toward the current design ("soft assumptions"). */
  soft?: boolean;
  softWeight?: number;
  /** design: include editing-point targets (default true). */
  includeTargets?: boolean;
  /** design: explicit target list (defaults to model.targets). */
  targets?: Target[];
  /** forward/sketch: treat flexible links / compliant hinges as springs (default true). */
  flexibility?: boolean;
  /** include driver constraints (default: forward & design yes, sketch no). */
  includeDrivers?: boolean;
}

export interface Group {
  index: number;
  members: ID[];
  rep: ID;
  constant: boolean;
  constPos: Vec3 | null;
  frozen: [boolean, boolean, boolean];
  frozenVal: Vec3;
  shared: boolean;
}

export interface CompiledSystem {
  n: number;
  constraints: Constraint[];
  x0: Float64Array;
  poses: number;
  mode: CompileMode;
  ref(pose: number, pointId: ID): PRef;
  getPos(x: Float64Array, pose: number, pointId: ID): Vec3;
  /** Variable columns (>= 0) of a point in a pose. */
  columnsOf(pose: number, pointId: ID): number[];
  groupOf(pointId: ID): Group;
  groups: Group[];
  /** Columns that are design variables (pose 0 and shared groups) in design mode. */
  designColumns: number[];
  /** Map of all positions of all points (per pose). */
  extract(x: Float64Array, pose: number): Map<ID, Vec3>;
}

/** Weight applied to compliant (flexible) constraints relative to hard ones. */
export const FLEX_WEIGHT_SCALE = 0.05;

class UnionFind {
  parent = new Map<ID, ID>();
  find(a: ID): ID {
    let r = a;
    while (this.parent.get(r) !== undefined && this.parent.get(r) !== r) r = this.parent.get(r)!;
    // path compression
    let c = a;
    while (this.parent.get(c) !== undefined && this.parent.get(c) !== r) {
      const next = this.parent.get(c)!;
      this.parent.set(c, r);
      c = next;
    }
    return r;
  }
  add(a: ID): void {
    if (!this.parent.has(a)) this.parent.set(a, a);
  }
  union(a: ID, b: ID): void {
    this.add(a);
    this.add(b);
    const ra = this.find(a);
    const rb = this.find(b);
    if (ra !== rb) this.parent.set(rb, ra);
  }
}

const idNumber = (id: ID): number => Number(id.split('_')[1] ?? 0);

function axisAlignedIndex(n: Vec3): number {
  for (let i = 0; i < 3; i++) if (Math.abs(Math.abs(n[i]) - 1) < 1e-9) return i;
  return -1;
}

export function compile(m: Model, opts: CompileOptions): CompiledSystem {
  const mode = opts.mode;
  const P = Math.max(1, opts.poses ?? 1);
  const includeDrivers = opts.includeDrivers ?? mode !== 'sketch';
  const flexibility = opts.flexibility ?? mode !== 'design';
  const ground = groundLink(m);
  const ws = opts.softWeight ?? 0.02;

  // ---- 1. merge coincident points ------------------------------------------------
  const uf = new UnionFind();
  for (const id of Object.keys(m.points)) uf.add(id);
  for (const j of Object.values(m.joints)) for (const [a, b] of j.pairs ?? []) if (m.points[a] && m.points[b]) uf.union(a, b);

  const constPoint = new Map<ID, Vec3>(); // point -> constant world position
  const groundConstant = ground ? (mode === 'forward' ? true : mode === 'sketch' ? !opts.allowGroundMove : ground.locked) : false;
  if (ground && groundConstant) for (const id of linkAllPointIds(ground)) constPoint.set(id, m.points[id].pos);
  for (const j of Object.values(m.joints)) {
    if (!isConstructionRef(j.b)) continue;
    const c = m.construction[j.b.constructionId];
    if (!c) continue;
    if ((j.type === 'spherical' || j.type === 'revolute') && c.kind === 'point' && j.a.kind === 'vertex') constPoint.set(j.a.pointIds[0], c.origin);
  }

  // frozen coordinates from axis-aligned construction planes
  const frozenByPoint = new Map<ID, { axis: number; value: number }[]>();
  for (const j of Object.values(m.joints)) {
    if (j.type !== 'planar' || !isConstructionRef(j.b)) continue;
    const c = m.construction[j.b.constructionId];
    if (!c || c.kind !== 'plane' || !c.dir) continue;
    const ax = axisAlignedIndex(c.dir);
    if (ax < 0) continue;
    for (const pid of j.a.pointIds) {
      if (!m.points[pid]) continue;
      const arr = frozenByPoint.get(pid) ?? [];
      arr.push({ axis: ax, value: c.origin[ax] });
      frozenByPoint.set(pid, arr);
    }
  }

  const groupsByRoot = new Map<ID, Group>();
  const groups: Group[] = [];
  const groupOfPoint = new Map<ID, Group>();
  const sortedIds = Object.keys(m.points).sort((a, b) => idNumber(a) - idNumber(b));
  for (const id of sortedIds) {
    const root = uf.find(id);
    let g = groupsByRoot.get(root);
    if (!g) {
      g = { index: groups.length, members: [], rep: id, constant: false, constPos: null, frozen: [false, false, false], frozenVal: [0, 0, 0], shared: false };
      groupsByRoot.set(root, g);
      groups.push(g);
    }
    g.members.push(id);
    groupOfPoint.set(id, g);
  }
  for (const g of groups) {
    // representative: prefer a ground point, then the earliest-created point
    const groundMember = ground ? g.members.find((id) => m.points[id].linkId === ground.id) : undefined;
    if (groundMember) g.rep = groundMember;
    for (const id of g.members) {
      const cp = constPoint.get(id);
      if (cp) {
        g.constant = true;
        g.constPos = cp;
        break;
      }
    }
    for (const id of g.members) {
      for (const f of frozenByPoint.get(id) ?? []) {
        g.frozen[f.axis] = true;
        g.frozenVal[f.axis] = f.value;
      }
    }
    if (mode === 'design' && ground && !groundConstant && g.members.some((id) => m.points[id].linkId === ground.id)) g.shared = true;
  }

  // ---- 2. allocate columns ----------------------------------------------------------
  let n = 0;
  const colIndex: Int32Array[] = []; // colIndex[pose][group*3+c]
  const sharedCols = new Int32Array(groups.length * 3).fill(-2);
  const x0Vals: number[] = [];
  const posPositionFor = (k: number, id: ID): Vec3 => opts.posePositions?.[k]?.get(id) ?? m.points[id].pos;
  for (let k = 0; k < P; k++) {
    const ci = new Int32Array(groups.length * 3).fill(-1);
    for (const g of groups) {
      if (g.constant) continue;
      if (g.shared) {
        if (sharedCols[g.index * 3] === -2) {
          const p0 = posPositionFor(0, g.rep);
          for (let c = 0; c < 3; c++) {
            if (g.frozen[c]) sharedCols[g.index * 3 + c] = -1;
            else {
              sharedCols[g.index * 3 + c] = n++;
              x0Vals.push(p0[c]);
            }
          }
        }
        for (let c = 0; c < 3; c++) ci[g.index * 3 + c] = sharedCols[g.index * 3 + c];
        continue;
      }
      const p = posPositionFor(k, g.rep);
      for (let c = 0; c < 3; c++) {
        if (g.frozen[c]) continue;
        ci[g.index * 3 + c] = n++;
        x0Vals.push(p[c]);
      }
    }
    colIndex.push(ci);
  }
  const x0 = Float64Array.from(x0Vals);

  const ref = (k: number, pid: ID): PRef => {
    const g = groupOfPoint.get(pid);
    if (!g) throw new Error(`Unknown point ${pid}`);
    if (g.constant) return constRef(g.constPos ?? m.points[g.rep].pos);
    const ci = colIndex[k];
    const base = posPositionFor(k, g.rep);
    const c: Vec3 = [g.frozen[0] ? g.frozenVal[0] : base[0], g.frozen[1] ? g.frozenVal[1] : base[1], g.frozen[2] ? g.frozenVal[2] : base[2]];
    return { cols: [ci[g.index * 3], ci[g.index * 3 + 1], ci[g.index * 3 + 2]], c };
  };
  const R = ref;

  const constraints: Constraint[] = [];
  const isConst = (r: PRef) => r.cols[0] < 0 && r.cols[1] < 0 && r.cols[2] < 0;
  const sameGroup = (a: ID, b: ID) => groupOfPoint.get(a) === groupOfPoint.get(b);
  const flexWeight = (stiffness: number) => FLEX_WEIGHT_SCALE * Math.min(1, Math.max(0.01, stiffness));
  const pos = (id: ID) => m.points[id].pos;

  // ---- 3. link rigidity --------------------------------------------------------------------
  for (const link of Object.values(m.links)) {
    const gConst = link.ground && groundConstant;
    if (gConst) continue;
    const posesForLink = link.ground ? 1 : P; // shared ground geometry needs constraints once
    const designFree = mode === 'design' && !link.locked;
    const flex = flexibility && link.flexible && !link.ground;
    for (let k = 0; k < posesForLink; k++) {
      for (const r of link.rigidity) {
        const tag = `rigid:${link.id}`;
        switch (r.kind) {
          case 'dist': {
            if (sameGroup(r.a, r.b)) break;
            const pa = R(k, r.a);
            const pb = R(k, r.b);
            if (isConst(pa) && isConst(pb)) break;
            if (r.fixed) {
              constraints.push(distance(pa, pb, r.length, 1, true, tag));
              break;
            }
            if (mode === 'sketch' && (opts.freePointIds?.has(r.a) || opts.freePointIds?.has(r.b))) break;
            if (designFree) {
              if (link.ground) break; // shared pivots: shape is free
              if (k === 0) {
                if (opts.soft) constraints.push(distance(pa, pb, r.length, ws, false, `soft:${link.id}`));
              } else {
                constraints.push(sharedDistance(pa, pb, R(0, r.a), R(0, r.b), 1, tag));
              }
              break;
            }
            constraints.push(distance(pa, pb, r.length, flex ? flexWeight(link.stiffness) : 1, true, tag));
            break;
          }
          case 'coplanar': {
            if (mode === 'sketch' && opts.freePointIds?.has(r.p)) break; // released vertex may leave its face plane
            const A0 = len(cross(sub(pos(r.b), pos(r.a)), sub(pos(r.c), pos(r.a))));
            constraints.push(pointOnBodyPlane(R(k, r.p), R(k, r.a), R(k, r.b), R(k, r.c), Math.max(A0, 1e-9), 1, true, tag));
            break;
          }
          case 'cos': {
            const Lc = Math.max(dist(pos(r.h), pos(r.p)), 1e-6);
            constraints.push(cosAngle(R(k, r.h), R(k, r.p), R(k, r.q), r.value, Lc, 1, true, tag));
            break;
          }
          case 'parallel': {
            const s = Math.max(dist(pos(r.a), pos(r.b)) * dist(pos(r.c), pos(r.d)), 1e-9);
            constraints.push(parallelVectors(R(k, r.a), R(k, r.b), R(k, r.c), R(k, r.d), s / Math.max(dist(pos(r.a), pos(r.b)), 1e-6), 1, true, tag));
            break;
          }
          case 'twist': {
            const Lc = Math.max(dist(pos(r.a), pos(r.b)), 1e-6);
            constraints.push(twist(R(k, r.a), R(k, r.b), R(k, r.c), R(k, r.d), r.value, Lc, 1, true, tag));
            break;
          }
        }
      }
    }
  }

  // ---- 4. joints -------------------------------------------------------------------------------
  const emitJoint = (j: Joint, k: number) => {
    const tag = `joint:${j.id}`;
    const linkA = m.links[j.a.linkId];
    if (!linkA) return;
    if (isConstructionRef(j.b)) {
      const c = m.construction[j.b.constructionId];
      if (!c) return;
      if (j.type === 'planar') {
        if (c.kind !== 'plane' || !c.dir) return;
        if (axisAlignedIndex(c.dir) >= 0) return; // frozen coordinates handle it
        for (const pid of j.a.pointIds) {
          const r = R(k, pid);
          if (!isConst(r)) constraints.push(pointOnPlane(r, c.origin, c.dir, 1, true, tag));
        }
        return;
      }
      if (j.type === 'spherical') return; // constant group
      const ax = jointAxisPoints(m, j, 'a');
      if (!ax) return;
      if (c.kind === 'point') {
        // revolute about a point with the joint axis: vertex constant, helper on the line
        const dirv = j.axis ?? [0, 0, 1];
        const r1 = R(k, ax[1]);
        if (!isConst(r1)) constraints.push(pointOnLine(r1, c.origin, dirv, 1, true, tag));
        return;
      }
      if (c.kind !== 'axis' || !c.dir) return;
      for (const pid of ax) {
        const r = R(k, pid);
        if (!isConst(r)) constraints.push(pointOnLine(r, c.origin, c.dir, 1, true, tag));
      }
      if (j.type === 'revolute') {
        constraints.push(projectionLock(R(k, ax[0]), c.origin, c.dir, j.offsets?.slide ?? dot(sub(pos(ax[0]), c.origin), c.dir), 1, true, tag));
      }
      if ((j.type === 'prismatic' || j.type === 'screw') && j.refs) {
        const refA = R(k, j.refs[0]);
        if (j.type === 'prismatic') {
          const nrm = j.offsets?.planeNormal ?? normalize(cross(c.dir, sub(pos(j.refs[0]), c.origin)));
          constraints.push(pointOnPlane(refA, c.origin, nrm, 1, true, tag));
        } else {
          const a0 = constRef(c.origin);
          const a1 = constRef([c.origin[0] + c.dir[0], c.origin[1] + c.dir[1], c.origin[2] + c.dir[2]]);
          const wref = constRef(worldAxisRef(c.origin, c.dir));
          const s0 = j.offsets?.slide ?? dot(sub(pos(ax[0]), c.origin), c.dir);
          const phi0 = j.offsets?.angle ?? axisRotation(a0, a1, wref, constRef(pos(j.refs[0])), x0);
          constraints.push(screwCoupling(a0, a1, R(k, ax[0]), wref, refA, j.pitch ?? 1, s0, phi0, Math.max(1, len(sub(pos(j.refs[0]), c.origin))), 1, true, tag));
        }
      }
      return;
    }
    const fb = j.b;
    const linkB = m.links[fb.linkId];
    if (!linkB) return;
    if (j.type === 'spherical') return; // merged
    if (j.type === 'planar') {
      const faceSide = j.a.kind === 'face' ? j.a : fb;
      const otherSide = j.a.kind === 'face' ? fb : j.a;
      if (faceSide.kind !== 'face' || faceSide.pointIds.length < 3) return;
      const [fa, fb2, fc] = faceSide.pointIds;
      const A0 = len(cross(sub(pos(fb2), pos(fa)), sub(pos(fc), pos(fa))));
      for (const pid of otherSide.pointIds) {
        if (faceSide.pointIds.includes(pid)) continue;
        constraints.push(pointOnBodyPlane(R(k, pid), R(k, fa), R(k, fb2), R(k, fc), Math.max(A0, 1e-9), 1, true, tag));
      }
      return;
    }
    const axA = jointAxisPoints(m, j, 'a');
    const axB = jointAxisPoints(m, j, 'b');
    if (!axA || !axB) return;
    const pairs = j.pairs ?? [];
    const merged = (pid: ID) => pairs.some(([a, b]) => a === pid || b === pid);
    const L0 = Math.max(dist(pos(axA[0]), pos(axA[1])), 1e-6);
    if (pairs.length < 2) {
      for (const pid of axB) {
        if (merged(pid)) continue;
        constraints.push(pointOnBodyLine(R(k, pid), R(k, axA[0]), R(k, axA[1]), L0, 1, true, tag));
      }
    }
    if (j.type === 'revolute' && pairs.length === 0) {
      constraints.push(slideLock(R(k, axB[0]), R(k, axA[0]), R(k, axA[1]), j.offsets?.slide ?? 0, 1, true, tag));
    }
    if ((j.type === 'prismatic' || j.type === 'screw') && j.refs) {
      const [refA, refB] = j.refs;
      if (j.type === 'prismatic') {
        const A0 = len(cross(sub(pos(axA[1]), pos(axA[0])), sub(pos(refA), pos(axA[0]))));
        constraints.push(pointOnBodyPlane(R(k, refB), R(k, axA[0]), R(k, axA[1]), R(k, refA), Math.max(A0, 1e-9), 1, true, tag));
      } else {
        const s0 = j.offsets?.slide ?? dot(sub(pos(axB[0]), pos(axA[0])), normalize(sub(pos(axA[1]), pos(axA[0]))));
        const phi0 = j.offsets?.angle ?? axisRotation(constRef(pos(axA[0])), constRef(pos(axA[1])), constRef(pos(refA)), constRef(pos(refB)), x0);
        constraints.push(screwCoupling(R(k, axA[0]), R(k, axA[1]), R(k, axB[0]), R(k, refA), R(k, refB), j.pitch ?? 1, s0, phi0, L0, 1, true, tag));
      }
    }
    // compliant hinge (torsional spring)
    if (j.type === 'revolute' && j.hinge?.enabled && flexibility) {
      const offA = offAxisPoint(m, linkA, axA);
      const offB = offAxisPoint(m, linkB, axB);
      if (offA && offB) {
        constraints.push(dihedral(R(k, axA[0]), R(k, axA[1]), R(k, offA), R(k, offB), deg2rad(j.hinge.restAngle), L0, flexWeight(j.hinge.stiffness), false, `hinge:${j.id}`));
      }
    }
  };
  for (let k = 0; k < P; k++) for (const j of Object.values(m.joints)) emitJoint(j, k);

  // ---- 5. drivers ----------------------------------------------------------------------------------
  if (includeDrivers) {
    for (let k = 0; k < P; k++) {
      m.drivers.forEach((d, i) => {
        const value = opts.driverValues?.[k]?.[i] ?? d.value;
        const tag = `driver:${d.id}`;
        if (d.kind === 'angle' && d.pivotId && d.tipId && d.axis && d.ref) {
          if (!m.points[d.pivotId] || !m.points[d.tipId]) return;
          const Lc = Math.max(dist(pos(d.tipId), pos(d.pivotId)), 1e-6);
          constraints.push(angleDriver(R(k, d.tipId), R(k, d.pivotId), d.axis, d.ref, deg2rad(value), Lc, 1, true, tag));
        } else if (d.kind === 'fold' && d.jointId) {
          const j = m.joints[d.jointId];
          if (!j || isConstructionRef(j.b)) return;
          const axA = jointAxisPoints(m, j, 'a');
          const axB = jointAxisPoints(m, j, 'b');
          if (!axA || !axB) return;
          const offA = offAxisPoint(m, m.links[j.a.linkId], axA);
          const offB = offAxisPoint(m, m.links[j.b.linkId], axB);
          if (!offA || !offB) return;
          const Lc = Math.max(dist(pos(axA[0]), pos(axA[1])), 1e-6);
          constraints.push(dihedral(R(k, axA[0]), R(k, axA[1]), R(k, offA), R(k, offB), deg2rad(value), Lc, 1, true, tag));
        } else if (d.kind === 'slide' && d.jointId) {
          const j = m.joints[d.jointId];
          if (!j) return;
          const axA = jointAxisPoints(m, j, 'a');
          if (!axA) return;
          if (isConstructionRef(j.b)) {
            const c = m.construction[j.b.constructionId];
            if (c?.kind === 'axis' && c.dir) constraints.push(projectionLock(R(k, axA[0]), c.origin, c.dir, value, 1, true, tag));
          } else {
            const axB = jointAxisPoints(m, j, 'b');
            if (axB) constraints.push(slideLock(R(k, axB[0]), R(k, axA[0]), R(k, axA[1]), value, 1, true, tag));
          }
        }
      });
    }
  }

  // ---- 6. targets (inverse design) ---------------------------------------------------------------
  if (mode === 'design' && (opts.includeTargets ?? true)) {
    for (const t of opts.targets ?? m.targets) {
      if (t.pose < 0 || t.pose >= P || !m.points[t.pointId]) continue;
      const r = R(t.pose, t.pointId);
      const tag = `target:${t.id}`;
      if (t.kind === 'position' && t.position) constraints.push(fixPosition(r, t.position, 1, true, tag));
      else if (t.constructionId) {
        const c = m.construction[t.constructionId];
        if (!c) continue;
        if (t.kind === 'onPoint') constraints.push(fixPosition(r, c.origin, 1, true, tag));
        else if (t.kind === 'onAxis' && c.dir) constraints.push(pointOnLine(r, c.origin, c.dir, 1, true, tag));
        else if (t.kind === 'onPlane' && c.dir) constraints.push(pointOnPlane(r, c.origin, c.dir, 1, true, tag));
      }
    }
  }

  // ---- 7. sketch drag targets ---------------------------------------------------------------------------
  if (mode === 'sketch') {
    for (const t of opts.dragTargets ?? []) {
      if (!m.points[t.pointId]) continue;
      constraints.push(fixPosition(R(0, t.pointId), t.pos, t.weight, false, `drag:${t.pointId}`));
    }
  }

  // ---- 8. soft regularisers for shared ground pivots ----------------------------------------------
  if (mode === 'design' && opts.soft) {
    for (const g of groups) {
      if (!g.shared) continue;
      constraints.push(fixPosition(R(0, g.rep), m.points[g.rep].pos, ws, false, 'soft:ground'));
    }
  }

  const designColumns: number[] = [];
  if (mode === 'design') {
    for (const g of groups) {
      if (g.constant) continue;
      for (let c = 0; c < 3; c++) {
        const col = colIndex[0][g.index * 3 + c];
        if (col >= 0) designColumns.push(col);
      }
    }
  }

  const sys: CompiledSystem = {
    n,
    constraints,
    x0,
    poses: P,
    mode,
    ref,
    getPos: (x, k, pid) => getP(x, ref(k, pid)),
    columnsOf: (k, pid) => ref(k, pid).cols.filter((c) => c >= 0),
    groupOf: (pid) => groupOfPoint.get(pid)!,
    groups,
    designColumns,
    extract: (x, k) => {
      const out = new Map<ID, Vec3>();
      for (const pid of Object.keys(m.points)) out.set(pid, getP(x, ref(k, pid)));
      return out;
    },
  };
  return sys;
}
