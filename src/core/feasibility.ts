/**
 * Pre-flight solve, rollback and diagnostics for construction-mode edits.
 *
 * Every edit that has to be re-solved (a new joint, a joint type change, a bar
 * length, a drag release) goes through one of the helpers here: the model is
 * snapshotted, the edit is applied, the sketch solve runs, and the result is
 * committed only when it is accepted. Otherwise the model is restored byte for
 * byte from the snapshot and the caller receives a diagnosis to show. No tool
 * path commits a non-converged pose (docs/CONSTRUCTION_PLAN.md, item 0b).
 */
import { dist, dot, normalize, sub } from './geometry';
import { commitSketch, currentViolation, feasibilityTolerance, isAccepted, modelSize, positionsFromModel, solveSketch, solveSketchWithRelease, type Positions, type SketchOptions } from './kinematics';
import { EDGE_LENGTH_TOLERANCE, addJoint, bodyPlaneJoint, changeJointType, cloneModel, creaseReleasePoints, parseModel, refreshRigidity, removeJoint, serializeModel, type JointOptions } from './model';
import type { ConstructionRef, Feature, ID, Joint, JointType, Link, Model, Vec3 } from './types';
import { isConstructionRef } from './types';

// ---------------------------------------------------------------------------
// Acceptance and pose consistency
// ---------------------------------------------------------------------------

// feasibilityTolerance and isAccepted live in kinematics.ts (commitSketch needs them); they are part of this
// module's API as well.
export { feasibilityTolerance, isAccepted };

/**
 * Largest hard residual a pose produced by any solve the app reports as
 * converged can carry: sketch and forward solves run with tol 1e-8 and flag
 * convergence below 10× that (1e-7, solver.ts); the inverse-design solve
 * (synthesis.ts) runs with tol 1e-7, so the pose it writes back can carry up
 * to 1e-6. A pose that the app itself just accepted must never be called
 * inconsistent, so this is the absolute floor of consistencyTolerance.
 */
export const CONVERGED_RESIDUAL = 1e-6;

/**
 * Violation above which a static construction pose counts as inconsistent:
 * the larger of CONVERGED_RESIDUAL and the model-size tolerance. Unlike
 * isAccepted, which judges a solve result together with the solver's
 * converged flag, a pose at rest has no flag, so the floor has to be the
 * residual a converged solve may leave rather than 1e-9. The DOF chip's
 * "Constraints violated" flag, Solidify and Save use this one tolerance.
 */
export function consistencyTolerance(m: Model): number {
  return Math.max(CONVERGED_RESIDUAL, feasibilityTolerance(m));
}

/**
 * Does the current construction pose satisfy every hard constraint to within
 * `tol` (currentViolation, the maximum hard residual in length units)? An
 * empty model is consistent. Rest geometry may only be refreshed ("baked")
 * from a consistent pose; otherwise the violation would become the design.
 */
export function poseIsConsistent(m: Model, tol: number = consistencyTolerance(m)): boolean {
  if (Object.keys(m.points).length === 0) return true;
  return currentViolation(m) <= tol;
}

/**
 * Solidify the design: make the current positions the rest geometry of every
 * link (refreshRigidity on each). Refused, with nothing changed, when the pose
 * is not consistent (poseIsConsistent with `tol`), since baking a violated
 * pose would silently turn the violation into the design. Returns whether the
 * rest geometry was refreshed. The app's Solidify button and Save use this.
 */
export function solidifyRestGeometry(m: Model, tol: number = consistencyTolerance(m)): boolean {
  if (!poseIsConsistent(m, tol)) return false;
  for (const link of Object.values(m.links)) refreshRigidity(m, link);
  return true;
}

// ---------------------------------------------------------------------------
// Diagnoses
// ---------------------------------------------------------------------------

export type Diagnosis =
  /** The joint type cannot connect these features (addJoint returned null). */
  | { kind: 'incompatible' }
  /** Edge–edge revolute between edges whose lengths differ by more than the merge tolerance. */
  | { kind: 'edgeLengths'; la: number; lb: number }
  /**
   * The joint closes a loop of panels around one vertex whose interior angles do not add up to 360°.
   * `constrained2d` is true when every contributing panel carries a sketch-plane (2-D) constraint.
   */
  | { kind: 'sectorSum'; vertexPointId: ID; vertexName: string; sumDeg: number; linkIds: ID[]; constrained2d: boolean }
  /** The loop can be closed once the sketch-plane constraints of the involved links are removed. */
  | { kind: 'needs3d'; linkIds: ID[] }
  /** Nothing more specific could be said; `residual` is the remaining gap in length units. */
  | { kind: 'infeasible'; residual: number };

export interface FeasibilityResult {
  ok: boolean;
  /** The created joint (ok only). */
  joint?: Joint;
  /** Why the edit was refused (failure only). */
  diagnosis?: Diagnosis;
  /** Hard residual of the verifying solve (length units). */
  residual: number;
}

/** Angle sum tolerance for the sector diagnosis (degrees). */
export const SECTOR_SUM_TOLERANCE_DEG = 0.5;

/** Minimum number of panels around a vertex for the sector diagnosis. */
export const SECTOR_MIN_PANELS = 3;

// ---------------------------------------------------------------------------
// Joint creation and type change
// ---------------------------------------------------------------------------

/**
 * Add a joint, re-solve, and keep it only when the solve is accepted. On
 * failure the model is restored to its state before the call (serializeModel
 * equality) and a diagnosis is returned, most specific first: sector sum at a
 * shared vertex, closure possible only out of the sketch plane, edge lengths
 * that differ, or the generic residual. Creases between edges of slightly
 * different length let one link's edge adapt (creaseReleasePoints) before the
 * result is judged, as the plain Joint-tool path did.
 */
export function tryAddJoint(m: Model, type: JointType, a: Feature, b: Feature | ConstructionRef, opts: JointOptions = {}): FeasibilityResult {
  const snapshot = serializeModel(m);
  const loop = isConstructionRef(b) ? null : linkPath(m, a.linkId, b.linkId);
  const joint = addJoint(m, type, a, b, opts);
  if (!joint) {
    restore(m, snapshot);
    return { ok: false, diagnosis: { kind: 'incompatible' }, residual: 0 };
  }
  return verifyJoint(m, joint, snapshot, loop);
}

/**
 * Change a joint's type with the same pre-flight as tryAddJoint. An
 * incompatible type leaves the original joint in place (changeJointType alone
 * removes it before finding out), a refused solve restores the model.
 */
export function tryChangeJointType(m: Model, jointId: ID, type: JointType, opts: JointOptions = {}): FeasibilityResult {
  const snapshot = serializeModel(m);
  const old = m.joints[jointId];
  if (!old) return { ok: false, diagnosis: { kind: 'incompatible' }, residual: 0 };
  const loop = isConstructionRef(old.b) ? null : linkPath(m, old.a.linkId, old.b.linkId, jointId);
  const joint = changeJointType(m, jointId, type, opts);
  if (!joint) {
    restore(m, snapshot);
    return { ok: false, diagnosis: { kind: 'incompatible' }, residual: 0 };
  }
  return verifyJoint(m, joint, snapshot, loop);
}

/** Solve for a freshly added joint; commit on acceptance, otherwise diagnose and restore. */
function verifyJoint(m: Model, joint: Joint, snapshot: string, loop: ID[] | null): FeasibilityResult {
  const free = new Set(creaseReleasePoints(m, joint));
  const res = solveSketchWithRelease(m, free);
  if (isAccepted(res, feasibilityTolerance(m))) {
    commitSketch(m, res, free);
    return { ok: true, joint, residual: res.residual };
  }
  const diagnosis = diagnoseJoint(m, joint, res.residual, loop);
  restore(m, snapshot);
  return { ok: false, diagnosis, residual: res.residual };
}

// ---------------------------------------------------------------------------
// Other re-solved edits
// ---------------------------------------------------------------------------

/**
 * Re-solve after an edit that is not a joint creation (bar length, typed
 * position, drag release) and commit only an accepted result. On failure the
 * WHOLE model is restored from `snapshot`, which defaults to the state at the
 * call; pass the serialisation taken before your own edit (e.g. before the
 * rest length was changed) so that a refusal undoes that edit as well.
 */
export function trySolveCommit(m: Model, opts: SketchOptions = {}, snapshot: string = serializeModel(m)): { ok: boolean; residual: number } {
  const res = solveSketch(m, opts);
  if (isAccepted(res, feasibilityTolerance(m))) {
    commitSketch(m, res, opts.freePointIds);
    return { ok: true, residual: res.residual };
  }
  restore(m, snapshot);
  return { ok: false, residual: res.residual };
}

/** Restore a model in place from a serializeModel snapshot (the object identity of `m` is kept). */
export function restore(m: Model, snapshot: string): void {
  Object.assign(m, parseModel(snapshot));
}

// ---------------------------------------------------------------------------
// Diagnosis
// ---------------------------------------------------------------------------

/**
 * Explain why the solve for `joint` was refused. `loop` is the chain of links
 * that already connected the two sides before the joint was added (null when
 * they were not connected, so the joint closes no loop).
 */
export function diagnoseJoint(m: Model, joint: Joint, residual: number, loop: ID[] | null): Diagnosis {
  if (loop) {
    const sector = sectorSumAt(m, joint);
    if (sector && Math.abs(sector.sumDeg - 360) > SECTOR_SUM_TOLERANCE_DEG) return { kind: 'sectorSum', ...sector };
  }
  const linkIds = loop ?? [joint.a.linkId, ...(isConstructionRef(joint.b) ? [] : [joint.b.linkId])];
  if (closesWithout2d(m, joint, linkIds)) return { kind: 'needs3d', linkIds };
  const lengths = unequalEdgeLengths(m, joint);
  if (lengths) return { kind: 'edgeLengths', ...lengths };
  return { kind: 'infeasible', residual };
}

export interface SectorSum {
  vertexPointId: ID;
  /** "<link name> <point name>" of the vertex on the joint's first link. */
  vertexName: string;
  /** Sum of the panels' interior angles at the vertex, rounded to 0.1°. */
  sumDeg: number;
  /** Panels (polygons / prisms) contributing an angle. */
  linkIds: ID[];
  /** Every contributing panel carries a sketch-plane (2-D) constraint. */
  constrained2d: boolean;
}

/**
 * Sum of the interior angles of the panels meeting at the vertex the joint
 * closes a loop around: the merged-point group (union-find over all joints'
 * pairs, including this joint) that contains one of the joint's end points and
 * touches the most links is the vertex; every polygon or prism with a point in
 * it contributes the angle between its two neighbouring vertices there (the
 * ring of the prism face the point belongs to). Returns null when fewer than
 * SECTOR_MIN_PANELS panels meet there (a plain crease or pin). Angles are
 * measured in the current pose, which for rigid panels is their design.
 */
export function sectorSumAt(m: Model, joint: Joint): SectorSum | null {
  const parent = new Map<ID, ID>();
  const find = (x: ID): ID => {
    let r = x;
    while (parent.has(r) && parent.get(r) !== r) r = parent.get(r)!;
    return r;
  };
  const union = (x: ID, y: ID) => {
    if (!parent.has(x)) parent.set(x, x);
    if (!parent.has(y)) parent.set(y, y);
    const rx = find(x);
    const ry = find(y);
    if (rx !== ry) parent.set(ry, rx);
  };
  for (const j of Object.values(m.joints)) for (const [x, y] of j.pairs ?? []) if (m.points[x] && m.points[y]) union(x, y);
  const endPoints = [...joint.a.pointIds, ...(isConstructionRef(joint.b) ? [] : joint.b.pointIds)].filter((id) => m.points[id]);
  let best: { members: ID[]; links: Set<ID> } | null = null;
  for (const pid of endPoints) {
    if (!parent.has(pid)) continue;
    const root = find(pid);
    const members = [...parent.keys()].filter((id) => find(id) === root);
    const links = new Set(members.map((id) => m.points[id].linkId));
    if (!best || links.size > best.links.size) best = { members, links };
  }
  if (!best) return null;
  let sum = 0;
  const linkIds: ID[] = [];
  for (const pid of best.members) {
    const link = m.links[m.points[pid].linkId];
    if (!link || (link.kind !== 'polygon' && link.kind !== 'prism') || linkIds.includes(link.id)) continue;
    const angle = interiorAngleDeg(m, link, pid);
    if (angle === null) continue;
    sum += angle;
    linkIds.push(link.id);
  }
  if (linkIds.length < SECTOR_MIN_PANELS) return null;
  const onA = best.members.find((id) => m.points[id].linkId === joint.a.linkId) ?? best.members[0];
  const pt = m.points[onA];
  return {
    vertexPointId: onA,
    vertexName: `${m.links[pt.linkId]?.name ?? ''} ${pt.name}`.trim(),
    sumDeg: Math.round(sum * 10) / 10,
    linkIds,
    constrained2d: linkIds.every((id) => bodyPlaneJoint(m, id) !== null),
  };
}

/** Interior angle (degrees) of a polygon / prism-face ring at one of its vertices, null if the point is not on a ring. */
export function interiorAngleDeg(m: Model, link: Link, pointId: ID): number | null {
  const ring = vertexRing(link, pointId);
  if (!ring || ring.length < 3) return null;
  const i = ring.indexOf(pointId);
  const v = m.points[pointId].pos;
  const prev = m.points[ring[(i + ring.length - 1) % ring.length]].pos;
  const next = m.points[ring[(i + 1) % ring.length]].pos;
  const u = sub(prev, v);
  const w = sub(next, v);
  const lu = Math.hypot(u[0], u[1], u[2]);
  const lw = Math.hypot(w[0], w[1], w[2]);
  if (lu < 1e-12 || lw < 1e-12) return null;
  const c = Math.max(-1, Math.min(1, dot(u, w) / (lu * lw)));
  return (Math.acos(c) * 180) / Math.PI;
}

/** The ordered vertex ring a point belongs to: the polygon itself, or the bottom / top face of a prism. */
function vertexRing(link: Link, pointId: ID): ID[] | null {
  if (link.kind === 'polygon') return link.pointIds.includes(pointId) ? link.pointIds : null;
  if (link.kind === 'prism') {
    const n = link.params.sides ?? link.pointIds.length / 2;
    const i = link.pointIds.indexOf(pointId);
    if (i < 0) return null;
    return i < n ? link.pointIds.slice(0, n) : link.pointIds.slice(n, 2 * n);
  }
  return null;
}

/**
 * Would the loop close if the sketch-plane (2-D) constraints of `linkIds` were
 * removed? Tested on a clone, first from the current pose and then from a pose
 * nudged out of the sketch plane: an exactly flat start is a bifurcation where
 * every out-of-plane gradient vanishes, so the solver cannot leave the plane
 * from it on its own. The clone is discarded.
 */
export function closesWithout2d(m: Model, joint: Joint, linkIds: ID[]): boolean {
  const clone = cloneModel(m);
  const removed: { link: Link; normal: Vec3 }[] = [];
  for (const id of linkIds) {
    const bp = bodyPlaneJoint(clone, id);
    if (!bp || !isConstructionRef(bp.b)) continue;
    const plane = clone.construction[bp.b.constructionId];
    removeJoint(clone, bp.id);
    removed.push({ link: clone.links[id], normal: plane?.dir ? normalize(plane.dir) : [0, 0, 1] });
  }
  if (removed.length === 0) return false;
  const cj = clone.joints[joint.id];
  const free = new Set(cj ? creaseReleasePoints(clone, cj) : []);
  const tol = feasibilityTolerance(clone);
  if (isAccepted(solveSketchWithRelease(clone, free), tol)) return true;
  const init: Positions = positionsFromModel(clone);
  const h = 0.05 * modelSize(clone);
  for (const { link, normal } of removed) {
    if (link.ground) continue;
    for (const id of [...link.pointIds, ...link.helperIds]) {
      const p = init.get(id);
      if (p) init.set(id, [p[0] + normal[0] * h, p[1] + normal[1] * h, p[2] + normal[2] * h]);
    }
  }
  return isAccepted(solveSketchWithRelease(clone, free, { init }), tol);
}

/** Lengths of an edge/axis–edge/axis revolute whose edges differ by more than the merge tolerance, else null. */
export function unequalEdgeLengths(m: Model, joint: Joint): { la: number; lb: number } | null {
  if (joint.type !== 'revolute' || isConstructionRef(joint.b)) return null;
  const axisLike = (f: Feature) => f.kind === 'edge' || f.kind === 'axis';
  if (!axisLike(joint.a) || !axisLike(joint.b)) return null;
  const la = dist(m.points[joint.a.pointIds[0]].pos, m.points[joint.a.pointIds[1]].pos);
  const lb = dist(m.points[joint.b.pointIds[0]].pos, m.points[joint.b.pointIds[1]].pos);
  return Math.abs(la - lb) > EDGE_LENGTH_TOLERANCE * Math.max(la, lb) ? { la, lb } : null;
}

// ---------------------------------------------------------------------------
// Link connectivity
// ---------------------------------------------------------------------------

/**
 * Shortest chain of links from `from` to `to` through link–link joints
 * (joints to construction geometry do not connect links), or null when they
 * are not connected. `excludeJointId` ignores one joint (the one being
 * replaced by a type change). A joint added between connected links closes a
 * loop made of the returned links.
 */
export function linkPath(m: Model, from: ID, to: ID, excludeJointId?: ID): ID[] | null {
  if (from === to) return [from];
  const adj = new Map<ID, Set<ID>>();
  const connect = (x: ID, y: ID) => {
    if (!adj.has(x)) adj.set(x, new Set());
    adj.get(x)!.add(y);
  };
  for (const j of Object.values(m.joints)) {
    if (j.id === excludeJointId || isConstructionRef(j.b)) continue;
    connect(j.a.linkId, j.b.linkId);
    connect(j.b.linkId, j.a.linkId);
  }
  const prev = new Map<ID, ID | null>([[from, null]]);
  const queue: ID[] = [from];
  while (queue.length) {
    const cur = queue.shift()!;
    if (cur === to) {
      const path: ID[] = [];
      for (let x: ID | null = to; x !== null; x = prev.get(x) ?? null) path.unshift(x);
      return path;
    }
    for (const nxt of adj.get(cur) ?? []) {
      if (prev.has(nxt)) continue;
      prev.set(nxt, cur);
      queue.push(nxt);
    }
  }
  return null;
}
