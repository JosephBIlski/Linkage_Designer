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
import { add, cross, dot, len, newellNormal, normalize, scale, sub } from './geometry';
import { jointDihedralDeg } from './jointMeasure';
import { applyPositions, computeMobility, feasibilityTolerance, isAccepted, measureDriver, positionsFromModel, solveForward, syncDriverValues, type Positions } from './kinematics';
import { addFoldDriver, bodyPlaneJoint, jointAxisPoints, linkFaces, offAxisPoint, removeJoint, serializeModel, sketchNormal } from './model';
import { mergedPointGroups, restore } from './feasibility';
import type { Driver, FeatureKind, ID, Joint, Model, Vec3 } from './types';
import { isConstructionRef } from './types';

/** A crease counts as flat (unfolded) when its dihedral is within this many degrees of ±180°. */
export const FLAT_TOLERANCE_DEG = 1;

/** Two creases at a vertex count as collinear when their outward directions are antiparallel within this angle. */
export const COLLINEAR_TOLERANCE_DEG = 1;

/** Default dihedral prescribed by the pre-fold (20° away from flat). */
export const DEFAULT_PREFOLD_DEG = 160;

/** Largest change of a prescribed dihedral in one forward solve of "Fold to target" (continuation step, degrees). */
export const MAX_PRESCRIBE_STEP_DEG = 120;

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
// Mountain / valley convention
// ---------------------------------------------------------------------------

/**
 * Oriented face normal of the first panel of a crease: the normal given by the
 * vertex winding (newellNormal) of the face of link a that contains the shared
 * edge, in the given pose. For a polygon drawn counter-clockwise on the sketch
 * plane, as the Polygon and Sketch tools draw them, this is the sketch-plane
 * normal, whatever the direction in which the edge was stored. Links without a
 * face containing the edge (bars, cylinder axes) fall back to
 * (a1 − a0) × (offA − a0), the normal the dihedral's sign is measured against.
 * Null when the crease cannot be measured.
 */
export function creaseNormal(m: Model, j: Joint, pos?: Positions): Vec3 | null {
  if (isConstructionRef(j.b)) return null;
  const axA = jointAxisPoints(m, j, 'a');
  const linkA = m.links[j.a.linkId];
  if (!axA || !linkA || !m.points[axA[0]] || !m.points[axA[1]]) return null;
  const get = (id: ID): Vec3 => pos?.get(id) ?? m.points[id].pos;
  const face = linkFaces(m, linkA).find((f) => f.length >= 3 && f.includes(axA[0]) && f.includes(axA[1]) && f.every((id) => !!m.points[id]));
  if (face) {
    const n = newellNormal(face.map(get));
    if (len(n) > 1e-12) return normalize(n);
  }
  const offA = offAxisPoint(m, linkA, axA);
  if (!offA) return null;
  const a0 = get(axA[0]);
  const n = cross(sub(get(axA[1]), a0), sub(get(offA), a0));
  return len(n) > 1e-12 ? normalize(n) : null;
}

/**
 * Which side of the first panel the positive dihedral lies on: +1 when
 * (a1 − a0) × (offA − a0), the side a positive creaseDihedralDeg bends panel b
 * to, points along creaseNormal, −1 when against it, 0 when unmeasurable.
 */
function creaseSide(m: Model, j: Joint, pos?: Positions): 1 | -1 | 0 {
  const n = creaseNormal(m, j, pos);
  const axA = jointAxisPoints(m, j, 'a');
  if (!n || !axA) return 0;
  const offA = offAxisPoint(m, m.links[j.a.linkId], axA);
  if (!offA) return 0;
  const get = (id: ID): Vec3 => pos?.get(id) ?? m.points[id].pos;
  const a0 = get(axA[0]);
  const s = dot(cross(sub(get(axA[1]), a0), sub(get(offA), a0)), n);
  return s > 1e-12 ? 1 : s < -1e-12 ? -1 : 0;
}

/**
 * Mountain / valley class of a crease in a pose, from the sign of its dihedral.
 *
 * Convention: let n be the oriented face normal of the first panel
 * (creaseNormal: for panels drawn counter-clockwise on the sketch plane, the
 * sketch normal). The crease is a VALLEY when the second panel is bent toward
 * the side n points to, so that the two panels form a trough seen from that
 * side, and a MOUNTAIN when it is bent away from n (a ridge seen from there).
 * The signed dihedral θ = creaseDihedralDeg is positive exactly when panel b
 * lies on the side of (a1 − a0) × (offA − a0), so the class is V when θ and
 * ((a1 − a0) × (offA − a0)) · n have the same sign and M otherwise; |θ| is the
 * fold angle (180° flat, 0° closed). Because n follows the panel's winding and
 * not the stored edge direction, neighbouring creases of a sheet drawn on one
 * sketch plane are classified from the same side. Null when the crease is flat
 * within FLAT_TOLERANCE_DEG, is not a crease or cannot be measured. Away from
 * flat this equals creaseMountainValley(m, j, creaseNormal(m, j, pos), pos).
 */
export function creaseMV(m: Model, j: Joint, pos?: Positions): 'M' | 'V' | null {
  const d = creaseDihedralDeg(m, j, pos);
  if (d === null || isFlatDihedral(d)) return null;
  const s = creaseSide(m, j, pos);
  if (s === 0) return null;
  return d > 0 === s > 0 ? 'V' : 'M';
}

/**
 * The signed dihedral (as creaseDihedralDeg measures it and a fold driver
 * prescribes it) at which crease `j` has fold angle |targetDeg| (clamped to
 * 180°) and the class `mv` in the creaseMV convention. Without `mv` the
 * current class is kept, and a flat crease is folded as a valley. Null when
 * the crease cannot be measured.
 */
export function creaseTargetDihedral(m: Model, j: Joint, targetDeg: number, mv?: 'M' | 'V' | null, pos?: Positions): number | null {
  const s = creaseSide(m, j, pos);
  if (s === 0) return null;
  const magnitude = Math.min(180, Math.abs(targetDeg));
  const cls = mv ?? creaseMV(m, j, pos) ?? 'V';
  return (cls === 'V') === (s > 0) ? magnitude : -magnitude;
}

/** The class `mv` (creaseMV convention) expressed as creaseMountainValley seen from `up`: flipped when creaseNormal points against `up`. */
function mvSeenFrom(m: Model, j: Joint, mv: 'M' | 'V', up: Vec3): 'M' | 'V' {
  const n = creaseNormal(m, j);
  return !n || dot(n, up) >= 0 ? mv : mv === 'M' ? 'V' : 'M';
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
 * Crease loops of the model: for every merged vertex (mergedPointGroups, the
 * union-find over the point pairs of all joints) the creases ending there are
 * collected, and they
 * form a loop when there are at least three of them and their panels form one
 * closed cycle around the vertex (every panel carries exactly two of the
 * creases and the cycle is connected). An open fan of panels is not a loop:
 * it folds freely without a pre-fold. The Miura example is one non-flat loop.
 */
export function findCreaseLoops(m: Model): CreaseLoop[] {
  const groups = mergedPointGroups(m);
  const rootOf = groups.root;

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
    const members = (groups.group(root) ?? []).filter((id) => m.points[id]);
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
 * Why a fold did nothing:
 *  noLoop: the model has no crease loop at all;
 *  notFlat: the chosen loop (or every loop, when none was chosen) is already folded;
 *  locked: a panel of the loop other than the ground is locked (its pose may not be edited);
 *  noBranch: no candidate crease and sign produced a converged, fully folded pose with fewer DOF;
 *  unreachable: (foldCreaseToTarget) no pose with the requested fold angle and class was found from the current
 *    pose (a constraint, a driver or a locked panel holds the crease), or the joint is not a crease.
 */
export type FoldReason = 'noLoop' | 'notFlat' | 'noBranch' | 'locked' | 'unreachable';

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
  /**
   * Mountain / valley preference per crease (creaseMountainValley seen from loopNormal). Every entry for a crease of
   * the loop is verified on the folded pose, whichever crease was driven: a pose that gives any named crease the
   * other class is rejected and the other sign tried, so a preference on a collinear crease (which is never driven
   * first) is honoured as well. When the driven crease has an entry its predicted sign is tried first.
   */
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

  const prefs = opts.mountainValley ?? {};
  for (const cid of candidates) {
    const j = m.joints[cid];
    const pref = prefs[cid];
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
      // every named preference on the loop is checked, not only the driven crease's (flipping the sign mirrors the
      // whole vertex, so a preference on a collinear crease selects the sign of whichever crease is driven)
      const mvOk = Object.entries(prefs).every(([id, p]) => !loop.creaseIds.includes(id) || !m.joints[id] || creaseMountainValley(m, m.joints[id], up) === p);
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

// ---------------------------------------------------------------------------
// Fold to target and drive (crease Properties / Driver tool)
// ---------------------------------------------------------------------------

/**
 * Return a crease loop to its exactly flat state: every crease of the loop is
 * prescribed ±180° at once (the sign of its current dihedral; the constraint
 * wraps, so the sign is immaterial), with the drivers acting on the vertex
 * released and every other driver held. Prescribing 180° on one crease alone
 * is not enough: that leaves the vertex anywhere on the straight-hinge branch
 * through the flat state, where a pre-fold cannot start. Temporary drivers are
 * removed again and the id counter rewound; on an accepted solve the pose is
 * applied and the driver values re-measured, otherwise nothing changes.
 */
function unfoldLoop(m: Model, loop: CreaseLoop): boolean {
  const pos0 = positionsFromModel(m);
  const originalDrivers = m.drivers;
  const nextId = m.nextId;
  const onLoop = (d: Driver): boolean => (d.jointId !== undefined && loop.creaseIds.includes(d.jointId)) || (d.linkId !== undefined && loop.linkIds.includes(d.linkId));
  m.drivers = originalDrivers.filter((d) => !onLoop(d));
  const values = m.drivers.map((d) => measureDriver(m, d.id, pos0) ?? d.value);
  for (const id of loop.creaseIds) {
    if (!addFoldDriver(m, id)) {
      m.drivers = originalDrivers;
      m.nextId = nextId;
      return false;
    }
    values.push(180 * Math.sign(creaseDihedralDeg(m, m.joints[id]) ?? 1));
  }
  const res = solveForward(m, values, pos0);
  m.drivers = originalDrivers;
  m.nextId = nextId;
  const ok = isAccepted(res, feasibilityTolerance(m));
  if (ok) {
    applyPositions(m, res.positions);
    syncDriverValues(m);
  }
  return ok;
}

/** Panels of a crease and of the loop it belongs to, other than the ground, that are locked. */
function lockedPanels(m: Model, j: Joint, loop: CreaseLoop | null): ID[] {
  const ids = new Set<ID>([j.a.linkId, (j.b as { linkId: ID }).linkId, ...(loop?.linkIds ?? [])]);
  return [...ids].filter((id) => m.links[id]?.locked && !m.links[id].ground);
}

/**
 * Fold crease `jointId` to the fold angle |targetDeg| (dihedral magnitude:
 * 180° flat, 0° closed) with the class `mv` (creaseMV convention; default: the
 * crease's current class, valley when flat), the "Fold to target" action of a
 * crease's Properties. A crease of a flat loop is pre-folded first
 * (prefoldVertex, preferring this crease and the side `mv` asks for), which
 * selects the generic branch and removes the sketch-plane constraints of the
 * loop's panels; then foldCreaseTo prescribes the signed target
 * (creaseTargetDihedral) by continuation, in steps of at most
 * MAX_PRESCRIBE_STEP_DEG and through the flat state when the target is the
 * same fold angle with the other class (so a plain hinge with no loop can be
 * flipped from mountain to valley), while the drivers acting on the vertex,
 * other than a fold driver on this crease, are released (holding them would
 * fix a 1-DOF vertex), and all driver values are re-measured. When the loop is flat and
 * the pre-fold drove another crease (this one is collinear with a neighbour),
 * both signs of the pre-fold are tried so the requested class is reached
 * without passing through flat again. On an already folded vertex whose crease
 * has the other class, the whole vertex is first returned to its flat state
 * (unfoldLoop) and then pre-folded toward the requested side, because the
 * mirrored branch is only reachable through flat. A target of 180° on a
 * folded degree-4 vertex brings the vertex to (numerically, within the flat
 * tolerance) its flat state, since that is the only generic pose in which one
 * crease is unfolded. A result is accepted only
 * when the solve is accepted, this crease has the requested class (not checked
 * for a flat or a fully closed target, where the panels have no class) and,
 * unless the target itself is flat, no crease of the loop is left flat (the
 * straight-hinge branch); otherwise the model is restored byte for byte and
 * the reason is 'unreachable' (or the pre-fold's reason).
 */
export function foldCreaseToTarget(m: Model, jointId: ID, targetDeg: number, mv?: 'M' | 'V'): FoldResult {
  const j = m.joints[jointId];
  if (!j || !isCrease(m, j)) return { ok: false, reason: 'unreachable' };
  const loop = findCreaseLoops(m).find((l) => l.creaseIds.includes(jointId)) ?? null;
  if (lockedPanels(m, j, loop).length > 0) return { ok: false, reason: 'locked' };
  const snapshot = serializeModel(m);
  const magnitude = Math.min(180, Math.abs(targetDeg));
  const flatTarget = isFlatDihedral(magnitude);
  const closedTarget = magnitude < FLAT_TOLERANCE_DEG; // fully closed: the panels coincide and have no class

  /** Signed angular difference a − b in (−180°, 180°]. */
  const angleDiff = (a: number, b: number): number => ((((a - b) % 360) + 540) % 360) - 180;
  const wrap = (a: number): number => angleDiff(a, 0);

  /** One forward solve prescribing a signed dihedral on this crease from the current pose, with the other drivers of the vertex released. */
  const prescribeStep = (signed: number): boolean => {
    const current = creaseDihedralDeg(m, j);
    if (current !== null && Math.abs(angleDiff(current, signed)) <= 1e-6) return true;
    const originalDrivers = m.drivers;
    const onVertex = (d: Driver): boolean =>
      (d.jointId !== undefined && d.jointId !== jointId && (loop?.creaseIds.includes(d.jointId) ?? false)) || (d.linkId !== undefined && (loop?.linkIds.includes(d.linkId) ?? false));
    m.drivers = originalDrivers.filter((d) => !onVertex(d));
    const f = foldCreaseTo(m, jointId, signed);
    m.drivers = originalDrivers;
    if (!f.ok) return false;
    syncDriverValues(m);
    return true;
  };

  /**
   * Prescribe a signed dihedral by continuation: the dihedral constraint wraps at ±180° from its target, so a single
   * solve cannot cross that discontinuity (M 90° → V 90° on a plain hinge is a half turn: the Gauss–Newton step
   * vanishes there). The way from the current dihedral to the target is split into steps of at most
   * MAX_PRESCRIBE_STEP_DEG along the shorter arc, each warm-started from the previous pose; an antipodal target
   * (the same fold angle with the other class) is routed through the flat state (±180°), the motion a hinge makes
   * when it is unfolded and folded to the other side, never through 0° (the panels passing through each other).
   */
  const prescribe = (signed: number): boolean => {
    const current = creaseDihedralDeg(m, j);
    if (current === null) return false;
    let diff = angleDiff(signed, current);
    const path: number[] = [];
    if (Math.abs(diff) > 180 - FLAT_TOLERANCE_DEG) {
      const viaFlat = wrap(current + 90 * Math.sign(current || 1)); // a quarter turn toward flat, then on to the target
      path.push(viaFlat);
      diff = angleDiff(signed, viaFlat);
      const n = Math.ceil(Math.abs(diff) / MAX_PRESCRIBE_STEP_DEG);
      for (let k = 1; k < n; k++) path.push(wrap(viaFlat + (diff * k) / n));
    } else {
      const n = Math.ceil(Math.abs(diff) / MAX_PRESCRIBE_STEP_DEG);
      for (let k = 1; k < n; k++) path.push(wrap(current + (diff * k) / n));
    }
    path.push(signed);
    return path.every((v) => prescribeStep(v));
  };

  /** Prescribe the signed target on this crease and judge the result. */
  const settle = (): boolean => {
    const target = creaseTargetDihedral(m, j, targetDeg, mv);
    if (target === null || !prescribe(target)) return false;
    if (mv && !flatTarget && !closedTarget && creaseMV(m, j) !== mv) return false;
    if (loop && !flatTarget) {
      for (const id of loop.creaseIds) {
        const d = creaseDihedralDeg(m, m.joints[id]);
        if (d === null || isFlatDihedral(d)) return false;
      }
    }
    return true;
  };

  const done = (removedPlanes: ID[]): FoldResult => {
    const dihedrals: Record<ID, number> = {};
    for (const id of loop?.creaseIds ?? [jointId]) {
      const d = creaseDihedralDeg(m, m.joints[id]);
      if (d !== null) dihedrals[id] = d;
    }
    return { ok: true, creaseId: jointId, dihedrals, dof: computeMobility(m).dof, removedPlanes };
  };

  // the requested side is the mirror image of the current fold: unfold the vertex through flat first, since the
  // mirrored branch can only be reached through the flat state (a collinear crease cannot cross it directly)
  let start = loop; // the loop as it is when the pre-fold starts (flat, or made flat below)
  if (loop && !loop.flat && mv && !flatTarget) {
    const current = creaseMV(m, j);
    if (current !== null && current !== mv && unfoldLoop(m, loop)) {
      const unfolded = findCreaseLoops(m).find((l) => l.creaseIds.includes(jointId));
      if (unfolded?.flat) start = unfolded;
      else restore(m, snapshot);
    }
  }
  if (loop && start?.flat && !flatTarget) {
    const flatSnapshot = serializeModel(m); // every attempt starts from the flat state
    const pref = mv ? mvSeenFrom(m, j, mv, loopNormal(m, start)) : undefined;
    let reason: FoldReason = 'unreachable';
    // the preferred sign of the pre-fold first (prefoldVertex verifies the class when it drives this crease)
    const attempts: (PrefoldOptions['sign'] | undefined)[] = [undefined, 1, -1];
    for (const sign of attempts) {
      const r = prefoldVertex(m, { loop: start, preferCreaseId: jointId, angleDeg: targetDeg, sign, mountainValley: pref && sign === undefined ? { [jointId]: pref } : undefined });
      if (!r.ok) {
        reason = r.reason ?? 'noBranch'; // prefoldVertex restored the flat state itself
        continue;
      }
      if (settle()) return done(r.removedPlanes ?? []);
      restore(m, flatSnapshot);
      reason = 'unreachable';
    }
    restore(m, snapshot);
    return { ok: false, reason };
  }
  if (settle()) return done([]);
  restore(m, snapshot);
  return { ok: false, reason: 'unreachable' };
}

export interface DriveCreaseResult {
  /** The fold driver on the crease (null when the joint is not a revolute). */
  driver: Driver | null;
  /** The pre-fold result (null when the crease is not on a flat loop). */
  prefold: FoldResult | null;
  /** The crease already had a fold driver, which is returned instead of a second one. */
  reused: boolean;
}

/**
 * Make revolute joint `jointId` the driven crease (the Driver tool and the
 * "Drive this crease" action). When the joint is a crease of a flat loop the
 * vertex is pre-folded first (prefoldVertex, preferring this crease and the
 * side its stored fold.mv asks for), because a sweep started in the flat state
 * follows the degenerate straight-hinge branch; the driver the pre-fold keeps
 * for a driverless model is replaced by the one on the picked crease. A crease
 * never carries two fold drivers: an existing fold driver on this crease is
 * returned as is (`reused`), and when the model's only driver is the fold
 * driver that Fold left on another crease of the same vertex, that driver is
 * moved to the picked crease, since a second driver on a one-DOF vertex would
 * freeze it (a deliberate extra driver on a model with several drivers is left
 * alone). Driver values are re-measured. A failed pre-fold still adds the
 * driver so the degenerate motion can be inspected.
 */
export function driveCrease(m: Model, jointId: ID): DriveCreaseResult {
  const j = m.joints[jointId];
  if (!j || j.type !== 'revolute') return { driver: null, prefold: null, reused: false };
  let prefold: FoldResult | null = null;
  const loop = isCrease(m, j) ? findCreaseLoops(m).find((l) => l.creaseIds.includes(jointId)) : undefined;
  if (loop?.flat) {
    const hadDrivers = m.drivers.length > 0;
    const mv = j.fold?.mv;
    prefold = prefoldVertex(m, { loop, preferCreaseId: jointId, mountainValley: mv ? { [jointId]: mvSeenFrom(m, j, mv, loopNormal(m, loop)) } : undefined });
    if (prefold.ok && !hadDrivers) m.drivers = [];
  }
  const existing = m.drivers.find((d) => d.kind === 'fold' && d.jointId === jointId);
  if (!existing && loop && m.drivers.length === 1 && m.drivers[0].kind === 'fold' && m.drivers[0].jointId !== undefined && loop.creaseIds.includes(m.drivers[0].jointId)) {
    m.drivers = []; // the single driver left on another crease of this vertex moves to the picked crease
  }
  const driver = existing ?? addFoldDriver(m, jointId);
  syncDriverValues(m);
  return { driver, prefold, reused: !!existing };
}
