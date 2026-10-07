/**
 * Fold (pre-fold) command for rigid-origami vertices.
 *
 * A flat degree-n vertex is a bifurcation of the constraint manifold: every
 * crease is a hinge, yet the numerical DOF of the flat state exceeds that of
 * the folded state (2 instead of 1 for a degree-4 vertex) and a sweep started
 * there falls onto the degenerate straight-hinge branch, where two creases
 * never move (docs/DESIGN_DECISIONS.md §8, docs/CONSTRUCTION_PLAN.md §2).
 * This module detects crease loops around a vertex, tells whether they are
 * flat, and leaves the flat state by prescribing a modest dihedral on one
 * crease and forward-solving the rest of the mechanism along with it. The
 * branch is selected by the choice of crease: driving a crease that is NOT
 * collinear with another crease at the vertex lands on the generic branch
 * (all creases fold); driving a collinear crease lands on the hinge branch,
 * which the acceptance test ("no crease of the loop stays flat") rejects.
 *
 * Pure core: no DOM or three.js imports; everything here is unit-testable.
 */
import { add, cross, dot, len, normalize, scale, sub } from './geometry';
import { jointDihedralDeg } from './jointMeasure';
import { applyPositions, computeMobility, feasibilityTolerance, isAccepted, measureDriver, positionsFromModel, solveForward, syncDriverValues, type Positions } from './kinematics';
import { addFoldDriver, bodyPlaneJoint, jointAxisPoints, offAxisPoint, removeJoint, serializeModel, sketchNormal } from './model';
import { restore } from './feasibility';
import type { Driver, FeatureKind, ID, Joint, Model, Vec3 } from './types';
import { isConstructionRef } from './types';

/** A crease counts as flat (unfolded) when its dihedral is within this many degrees of ±180°. */
export const FLAT_TOLERANCE_DEG = 1;

/** Two creases at a vertex count as collinear when their outward directions are antiparallel within this angle. */
export const COLLINEAR_TOLERANCE_DEG = 1;

/** Default dihedral prescribed by the pre-fold (20° away from flat). */
export const DEFAULT_PREFOLD_DEG = 160;

const axisLike = (k: FeatureKind): boolean => k === 'edge' || k === 'axis';

// ---------------------------------------------------------------------------
// Creases
// ---------------------------------------------------------------------------

/**
 * Is the joint an origami crease: a revolute between two edge/axis features
 * whose two end-point pairs are merged (the shared edge of two panels)? Hinges
 * between edges of different length (slide-locked, no pairs), vertex pins and
 * joints to construction geometry are not creases.
 */
export function isCrease(m: Model, j: Joint): boolean {
  if (j.type !== 'revolute' || isConstructionRef(j.b) || j.pairs?.length !== 2) return false;
  if (!axisLike(j.a.kind) || !axisLike(j.b.kind) || !m.links[j.a.linkId] || !m.links[j.b.linkId]) return false;
  return [...j.a.pointIds, ...j.b.pointIds].every((id) => !!m.points[id]);
}

/**
 * Signed dihedral angle (degrees) between the two panels across a crease, the
 * quantity a fold driver on the crease prescribes (jointDihedralDeg): ±180°
 * when the panels are coplanar and unfolded, |angle| → 0 as they close. Null
 * when the joint is not a crease or a panel has no point off the axis. `pos`
 * overrides the construction pose.
 */
export function creaseDihedralDeg(m: Model, j: Joint, pos?: Positions): number | null {
  return isCrease(m, j) ? jointDihedralDeg(m, j, pos) : null;
}

/** Is a dihedral within FLAT_TOLERANCE_DEG of ±180° (the crease is unfolded)? */
export function isFlatDihedral(deg: number, tolDeg: number = FLAT_TOLERANCE_DEG): boolean {
  return 180 - Math.abs(deg) < tolDeg;
}

/**
 * Mountain / valley assignment of a crease in a pose, seen from the side the
 * unit vector `up` points to: the two panels' in-plane directions away from
 * the axis are added; for a mountain fold (a ridge toward the viewer) both
 * panels dip away from `up`, so the sum points against it, for a valley it
 * points along it. Null when the crease is flat (the sum vanishes) or cannot
 * be measured. Independent of the orientation of the stored edge features;
 * the Fold command uses the normal of the flat vertex as `up` (see
 * loopNormal) and the crease display can use the same convention.
 */
export function creaseMountainValley(m: Model, j: Joint, up: Vec3, pos?: Positions): 'M' | 'V' | null {
  if (isConstructionRef(j.b)) return null;
  const axA = jointAxisPoints(m, j, 'a');
  const axB = jointAxisPoints(m, j, 'b');
  if (!axA || !axB) return null;
  const offA = offAxisPoint(m, m.links[j.a.linkId], axA);
  const offB = offAxisPoint(m, m.links[j.b.linkId], axB);
  if (!offA || !offB) return null;
  const get = (id: ID): Vec3 => pos?.get(id) ?? m.points[id].pos;
  const a0 = get(axA[0]);
  const e = normalize(sub(get(axA[1]), a0));
  const perp = (p: Vec3): Vec3 => {
    const w = sub(p, a0);
    return normalize(sub(w, scale(e, dot(w, e))));
  };
  const s = dot(add(perp(get(offA)), perp(get(offB))), normalize(up));
  if (Math.abs(s) < 1e-6) return null;
  return s < 0 ? 'M' : 'V';
}

// ---------------------------------------------------------------------------
// Crease loops
// ---------------------------------------------------------------------------

export interface CreaseLoop {
  /** A point of the shared vertex (on the ground panel when one is in the loop). */
  vertexPointId: ID;
  /** The creases in cyclic order around the vertex, starting from the earliest-created one. */
  creaseIds: ID[];
  /** The panels in the same cyclic order: linkIds[i] is shared by creaseIds[i] and creaseIds[(i + 1) % n]. */
  linkIds: ID[];
  /** Every crease of the loop is within FLAT_TOLERANCE_DEG of ±180° (the vertex is unfolded: a bifurcation). */
  flat: boolean;
  /** Pairs of creases whose outward directions from the vertex are antiparallel within COLLINEAR_TOLERANCE_DEG. */
  collinearPairs: [ID, ID][];
}

/**
 * Crease loops of the model: for every merged vertex (union-find over the
 * point pairs of all joints) the creases ending there are collected, and they
 * form a loop when there are at least three of them and their panels form one
 * closed cycle around the vertex (every panel carries exactly two of the
 * creases and the cycle is connected). An open fan of panels is not a loop:
 * it folds freely without a pre-fold. The Miura example is one non-flat loop.
 */
export function findCreaseLoops(m: Model): CreaseLoop[] {
  const parent = new Map<ID, ID>();
  const find = (x: ID): ID => {
    let r = x;
    while (parent.has(r) && parent.get(r) !== r) r = parent.get(r)!;
    return r;
  };
  const union = (x: ID, y: ID): void => {
    if (!parent.has(x)) parent.set(x, x);
    if (!parent.has(y)) parent.set(y, y);
    const rx = find(x);
    const ry = find(y);
    if (rx !== ry) parent.set(ry, rx);
  };
  for (const j of Object.values(m.joints)) for (const [x, y] of j.pairs ?? []) if (m.points[x] && m.points[y]) union(x, y);
  const rootOf = (id: ID): ID => (parent.has(id) ? find(id) : id);

  const creases = Object.values(m.joints).filter((j) => isCrease(m, j));
  const byVertex = new Map<ID, Joint[]>();
  for (const j of creases) {
    for (const pid of j.a.pointIds) {
      const r = rootOf(pid);
      if (!byVertex.has(r)) byVertex.set(r, []);
      byVertex.get(r)!.push(j);
    }
  }

  const loops: CreaseLoop[] = [];
  for (const [root, js] of byVertex) {
    if (js.length < 3) continue;
    const linksOf = (j: Joint): [ID, ID] => [j.a.linkId, (j.b as { linkId: ID }).linkId];
    const degree = new Map<ID, Joint[]>();
    for (const j of js) for (const l of linksOf(j)) degree.set(l, [...(degree.get(l) ?? []), j]);
    if (degree.size !== js.length || [...degree.values()].some((list) => list.length !== 2)) continue;
    // walk the cycle: from the earliest crease, step to the panel on its b side and leave it by its other crease
    const order: Joint[] = [js[0]];
    const linkOrder: ID[] = [];
    let link = linksOf(js[0])[1];
    let prev = js[0];
    while (order.length <= js.length) {
      linkOrder.push(link);
      const next = degree.get(link)!.find((j) => j !== prev);
      if (!next || next === js[0]) break;
      order.push(next);
      prev = next;
      link = linksOf(next)[0] === link ? linksOf(next)[1] : linksOf(next)[0];
    }
    if (order.length !== js.length) continue; // more than one cycle shares the vertex
    const members = [...parent.keys()].filter((id) => find(id) === root && m.points[id]);
    const vertexPointId = members.find((id) => m.links[m.points[id].linkId]?.ground) ?? members[0] ?? js[0].a.pointIds[0];
    const dihedrals = order.map((j) => creaseDihedralDeg(m, j));
    const flat = dihedrals.every((d) => d !== null && isFlatDihedral(d));
    const dirs = order.map((j) => creaseDirection(m, j, root, rootOf));
    const collinearPairs: [ID, ID][] = [];
    const cosTol = -Math.cos((COLLINEAR_TOLERANCE_DEG * Math.PI) / 180);
    for (let i = 0; i < order.length; i++) {
      for (let k = i + 1; k < order.length; k++) {
        if (dirs[i] && dirs[k] && dot(dirs[i]!, dirs[k]!) < cosTol) collinearPairs.push([order[i].id, order[k].id]);
      }
    }
    loops.push({ vertexPointId, creaseIds: order.map((j) => j.id), linkIds: linkOrder, flat, collinearPairs });
  }
  return loops;
}

/** Unit direction of a crease from the vertex (merged group `root`) outward, in the construction pose. */
function creaseDirection(m: Model, j: Joint, root: ID, rootOf: (id: ID) => ID): Vec3 | null {
  const [p0, p1] = j.a.pointIds;
  const atVertex = rootOf(p0) === root ? p0 : rootOf(p1) === root ? p1 : null;
  if (!atVertex) return null;
  const other = atVertex === p0 ? p1 : p0;
  const d = sub(m.points[other].pos, m.points[atVertex].pos);
  return len(d) > 1e-12 ? normalize(d) : null;
}

/**
 * Unit normal of a flat crease loop, oriented so that mountain / valley are
 * judged from a consistent side: the normal of the construction plane one of
 * its panels is kept on when there is one, otherwise the plane spanned by the
 * crease directions, oriented along the active sketch plane's normal (or the
 * first non-zero world axis when that is ambiguous).
 */
export function loopNormal(m: Model, loop: CreaseLoop): Vec3 {
  for (const id of loop.linkIds) {
    const bp = bodyPlaneJoint(m, id);
    const plane = bp && isConstructionRef(bp.b) ? m.construction[bp.b.constructionId] : null;
    if (plane?.dir) return normalize(plane.dir);
  }
  const v = m.points[loop.vertexPointId]?.pos ?? [0, 0, 0];
  const dirs = loop.creaseIds
    .map((id) => m.joints[id])
    .filter((j): j is Joint => !!j)
    .map((j) => {
      const [p0, p1] = j.a.pointIds;
      const far = len(sub(m.points[p0].pos, v)) > len(sub(m.points[p1].pos, v)) ? p0 : p1;
      return normalize(sub(m.points[far].pos, v));
    });
  let n: Vec3 = [0, 0, 0];
  for (let i = 0; i < dirs.length; i++) n = add(n, cross(dirs[i], dirs[(i + 1) % dirs.length]));
  n = normalize(n);
  if (len(n) < 0.5) n = sketchNormal(m);
  const ref = sketchNormal(m);
  const s = dot(n, ref);
  if (s < -1e-9) return scale(n, -1);
  if (s > 1e-9) return n;
  const k = [2, 1, 0].find((i) => Math.abs(n[i]) > 1e-9);
  return k !== undefined && n[k] < 0 ? scale(n, -1) : n;
}

// ---------------------------------------------------------------------------
// Folding a single crease
// ---------------------------------------------------------------------------

/**
 * Forward-solve the mechanism so that the dihedral of crease `jointId` is
 * `targetDeg` (degrees, signed like creaseDihedralDeg), keeping every other
 * driver at its current measured value. A fold driver the model already has
 * on this crease is given the target; otherwise a temporary driver prescribes
 * it and is removed again (the model's id counter is rewound, so a failed call
 * leaves the model byte-identical). On an accepted solve (isAccepted with the
 * model-size tolerance) the positions are applied and the driver values
 * re-measured; on failure nothing changes. `init` warm-starts the solve.
 */
export function foldCreaseTo(m: Model, jointId: ID, targetDeg: number, opts: { init?: Positions } = {}): { ok: boolean; residual: number } {
  const j = m.joints[jointId];
  if (!j || j.type !== 'revolute' || isConstructionRef(j.b)) return { ok: false, residual: Infinity };
  const pos0 = positionsFromModel(m);
  const onThisCrease = (d: Driver): boolean => d.kind === 'fold' && d.jointId === jointId;
  const values = m.drivers.map((d) => (onThisCrease(d) ? targetDeg : (measureDriver(m, d.id, pos0) ?? d.value)));
  const nextId = m.nextId;
  let temp: Driver | null = null;
  if (!m.drivers.some(onThisCrease)) {
    temp = addFoldDriver(m, jointId);
    if (!temp) return { ok: false, residual: Infinity };
    values.push(targetDeg);
  }
  const res = solveForward(m, values, opts.init ?? pos0);
  if (temp) {
    m.drivers = m.drivers.filter((d) => d !== temp);
    m.nextId = nextId;
  }
  const ok = isAccepted(res, feasibilityTolerance(m));
  if (ok) {
    applyPositions(m, res.positions);
    syncDriverValues(m);
  }
  return { ok, residual: res.residual };
}

// ---------------------------------------------------------------------------
// Pre-folding a flat vertex
// ---------------------------------------------------------------------------

/**
 * Why a pre-fold did nothing:
 *  noLoop: the model has no crease loop at all;
 *  notFlat: the chosen loop (or every loop, when none was chosen) is already folded;
 *  locked: a panel of the loop other than the ground is locked (its pose may not be edited);
 *  noBranch: no candidate crease and sign produced a converged, fully folded pose with fewer DOF.
 */
export type FoldReason = 'noLoop' | 'notFlat' | 'noBranch' | 'locked';

export interface FoldResult {
  ok: boolean;
  reason?: FoldReason;
  /** The crease whose dihedral was prescribed (ok only). */
  creaseId?: ID;
  /** Dihedral of every crease of the loop after the fold, by joint id (ok only). */
  dihedrals?: Record<ID, number>;
  /** Mechanism DOF after the fold (ok only). */
  dof?: number;
  /** Ids of the body planar (sketch-plane) joints removed from the loop's panels (ok only; undo restores them). */
  removedPlanes?: ID[];
}

export interface PrefoldOptions {
  /** The loop to fold (default: the loop containing preferCreaseId, else the first flat loop). */
  loop?: CreaseLoop;
  /** Crease to drive first (when it is not collinear with another crease of the vertex). */
  preferCreaseId?: ID;
  /** Dihedral prescribed on the driven crease (default DEFAULT_PREFOLD_DEG = 160°). */
  angleDeg?: number;
  /** Sign of the prescribed dihedral; both are tried when omitted. */
  sign?: 1 | -1;
  /** Mountain / valley preference per crease (creaseMountainValley seen from loopNormal); the matching sign is tried first and the other sign is rejected. */
  mountainValley?: Record<ID, 'M' | 'V'>;
}

/**
 * Leave the flat (bifurcation) state of a crease loop: the given loop, else
 * the loop containing `preferCreaseId`, else the first flat loop. The sketch-plane
 * constraints of the loop's panels are removed (a flat vertex drawn in 2-D is
 * held flat by them), then for each candidate crease, non-collinear creases
 * first because driving a collinear crease lands on the straight-hinge
 * branch, the dihedral ±angleDeg is prescribed (foldCreaseTo) from the flat
 * pose. A result is accepted when the solve converged, no crease of the loop
 * is still within FLAT_TOLERANCE_DEG of flat, the mountain / valley
 * preference (if any) is met and the mechanism DOF is lower than in the flat
 * state without the planar joints (2 → 1 for a degree-4 vertex). The pose is
 * then committed and the driver values re-measured; a model that had no
 * driver keeps the fold driver on the chosen crease so the vertex can be
 * previewed at once (docs/CONSTRUCTION_PLAN.md §8). Drivers acting on the
 * loop itself (fold drivers on its creases, angle drivers on its panels) are
 * released during the search, since holding them at their flat values would
 * forbid the fold; every other driver is held. When no candidate works the
 * model is restored byte for byte (planar joints included).
 */
export function prefoldVertex(m: Model, opts: PrefoldOptions = {}): FoldResult {
  const loops = opts.loop ? [opts.loop] : findCreaseLoops(m);
  if (loops.length === 0) return { ok: false, reason: 'noLoop' };
  const preferred = opts.preferCreaseId !== undefined ? loops.find((l) => l.creaseIds.includes(opts.preferCreaseId!)) : undefined;
  const loop = opts.loop ?? preferred ?? loops.find((l) => l.flat) ?? loops[0];
  if (!loop.flat) return { ok: false, reason: 'notFlat' };
  if (loop.linkIds.some((id) => m.links[id]?.locked && !m.links[id].ground)) return { ok: false, reason: 'locked' };

  const snapshot = serializeModel(m);
  const up = loopNormal(m, loop);
  const removedPlanes: ID[] = [];
  for (const id of loop.linkIds) {
    const bp = bodyPlaneJoint(m, id);
    if (bp) {
      removedPlanes.push(bp.id);
      removeJoint(m, bp.id);
    }
  }
  const dofBefore = computeMobility(m).dof;
  const originalDrivers = m.drivers;
  const onLoop = (d: Driver): boolean => (d.jointId !== undefined && loop.creaseIds.includes(d.jointId)) || (d.linkId !== undefined && loop.linkIds.includes(d.linkId));
  m.drivers = originalDrivers.filter((d) => !onLoop(d));

  const collinear = new Set(loop.collinearPairs.flat());
  const prefer = opts.preferCreaseId !== undefined && loop.creaseIds.includes(opts.preferCreaseId) ? opts.preferCreaseId : undefined;
  const candidates = [
    ...(prefer !== undefined && !collinear.has(prefer) ? [prefer] : []),
    ...loop.creaseIds.filter((id) => id !== prefer && !collinear.has(id)),
    ...(prefer !== undefined && collinear.has(prefer) ? [prefer] : []),
    ...loop.creaseIds.filter((id) => id !== prefer && collinear.has(id)),
  ];
  const angle = Math.abs(opts.angleDeg ?? DEFAULT_PREFOLD_DEG);

  for (const cid of candidates) {
    const j = m.joints[cid];
    const pref = opts.mountainValley?.[cid];
    const first = opts.sign ?? (pref ? predictedSign(m, j, up, pref) : 1);
    const signs: (1 | -1)[] = opts.sign ? [opts.sign] : [first, first === 1 ? -1 : 1];
    for (const s of signs) {
      const saved = positionsFromModel(m);
      if (!foldCreaseTo(m, cid, s * angle).ok) continue;
      const dihedrals: Record<ID, number> = {};
      let folded = true;
      for (const id of loop.creaseIds) {
        const d = creaseDihedralDeg(m, m.joints[id]);
        if (d === null || isFlatDihedral(d)) folded = false;
        else dihedrals[id] = d;
      }
      const mvOk = !pref || creaseMountainValley(m, j, up) === pref;
      const dof = folded && mvOk ? computeMobility(m).dof : dofBefore;
      if (folded && mvOk && dof < dofBefore) {
        m.drivers = originalDrivers;
        if (originalDrivers.length === 0) addFoldDriver(m, cid);
        syncDriverValues(m);
        return { ok: true, creaseId: cid, dihedrals, dof, removedPlanes };
      }
      applyPositions(m, saved);
      syncDriverValues(m);
    }
  }
  restore(m, snapshot);
  return { ok: false, reason: 'noBranch' };
}

/**
 * Sign of the prescribed dihedral expected to give the mountain / valley
 * assignment `pref` on crease `j` when folded from flat, seen from `up`: for a
 * small fold the dihedral's sign is that of the b panel's displacement along
 * `up` times the sign of (e × (offA − a0)) · up, and a mountain fold moves the
 * panels away from `up`. Only an ordering hint; prefoldVertex verifies the
 * result with creaseMountainValley.
 */
function predictedSign(m: Model, j: Joint, up: Vec3, pref: 'M' | 'V'): 1 | -1 {
  const axA = jointAxisPoints(m, j, 'a');
  const offA = axA ? offAxisPoint(m, m.links[j.a.linkId], axA) : null;
  if (!axA || !offA) return 1;
  const a0 = m.points[axA[0]].pos;
  const e = normalize(sub(m.points[axA[1]].pos, a0));
  const side = dot(cross(e, sub(m.points[offA].pos, a0)), up);
  const mountain = side < 0 ? 1 : -1;
  return pref === 'M' ? mountain : mountain === 1 ? -1 : 1;
}
