/**
 * Vertex editing: move a vertex of existing geometry to a new position (the
 * "Edit" tool) and join coincident vertices of different links (shared by
 * the Edit and Sketch tools).
 */
import { add, cross, dist, dot, len, normalize, perpendicular, scale, sub } from './geometry';
import { commitSketch, modelSize, solveSketch } from './kinematics';
import {
  addJoint,
  attachHelper,
  bodyPlaneJoint,
  buildRigidity,
  findFrame,
  helperBase,
  jointsAtPoint,
  linkEdges,
  linkShapePointIds,
  parseModel,
  removeJoint,
  rigidityTouches,
  serializeModel,
  sketchNormal,
} from './model';
import type { Construction, ID, Joint, JointType, Link, Model, Vec3 } from './types';
import { isConstructionRef } from './types';

export interface MoveVertexOptions {
  /** Vertex of another link the moved vertex should be joined to (auto-join also runs on coincidence). */
  joinTo?: ID;
  /** Acceptable distance between the vertex and `dest` (default 1e-5 × model size). */
  tol?: number;
}

export interface MoveVertexResult {
  ok: boolean;
  residual: number;
  reason?: 'missing' | 'locked' | 'unreachable';
  joints: Joint[];
}

/**
 * Move `pointId` to `dest`. The vertex is made a dependent point of its link's
 * rigidity (never a frame vertex, so the rest of the link keeps its shape),
 * released together with the joint-helper attachments that reference it, and
 * all other constraints are re-solved. If the destination cannot be reached
 * (e.g. the vertex is pinned to a datum point) the model is restored
 * unchanged. Helper attachments are rebuilt from the new rest geometry and
 * coincident vertices are joined afterwards (creases for shared edges).
 */
export function moveVertex(m: Model, pointId: ID, dest: Vec3, opts: MoveVertexOptions = {}): MoveVertexResult {
  const pt = m.points[pointId];
  if (!pt) return { ok: false, residual: Infinity, reason: 'missing', joints: [] };
  const link = m.links[pt.linkId];
  if (!link || link.locked) return { ok: false, residual: Infinity, reason: 'locked', joints: [] };
  const snapshot = serializeModel(m);
  const tol = opts.tol ?? 1e-5 * Math.max(1, modelSize(m));
  // the sketch-plane constraint must go if the destination leaves that plane
  const bp = bodyPlaneJoint(m, link.id);
  if (bp && isConstructionRef(bp.b)) {
    const c = m.construction[bp.b.constructionId];
    if (c?.dir && Math.abs(dot(sub(dest, c.origin), c.dir)) > tol) removeJoint(m, bp.id);
  }
  // joint-helper attachments (angle relations) that reference the moved vertex are released and rebuilt afterwards
  const affected = releaseHelperAttachments(link, pointId);
  reframeRigidityAway(m, link, pointId);
  const free = new Set([pointId]);
  // Weak targets keep each released helper at its current offset from its base, so a joint axis whose
  // partner helper is only roll-constrained cannot drift (tilt) while the angle relations are released.
  const helperTargets = affected.map(({ helper, base }) => ({
    pointId: helper,
    pos: add(base === pointId ? dest : m.points[base].pos, sub(m.points[helper].pos, m.points[base].pos)),
    weight: 0.02,
  }));
  const res = solveSketch(m, { dragTargets: [{ pointId, pos: dest, weight: 1 }, ...helperTargets], freePointIds: free, allowGroundMove: link.ground, maxIter: 80 });
  const reached = dist(res.positions.get(pointId)!, dest) <= Math.max(tol, 1e-6);
  if (!res.converged || !reached) {
    Object.assign(m, parseModel(snapshot));
    return { ok: false, residual: res.residual, reason: 'unreachable', joints: [] };
  }
  commitSketch(m, res, free);
  rebuildShapeRigidity(m, link.id);
  reattachHelpers(m, m.links[link.id], affected);
  const joints = autoJoinCoincident(m, m.links[link.id], [pointId], { defaultJoint: m.settings.defaultJoint, axis: sketchNormal(m), extraPairs: opts.joinTo ? [[pointId, opts.joinTo]] : [] });
  return { ok: true, residual: res.residual, joints };
}

/** Remove angle relations (cos / parallel / twist) between helpers and the given vertex; returns the affected helpers with their base points. */
function releaseHelperAttachments(link: Link, pointId: ID): { helper: ID; base: ID }[] {
  const affected: { helper: ID; base: ID }[] = [];
  for (const h of link.helperIds) {
    const touches = link.rigidity.some((r) => r.kind !== 'dist' && rigidityTouches(r, h) && rigidityTouches(r, pointId));
    if (!touches) continue;
    const base = helperBase(link, h);
    if (!base) continue;
    affected.push({ helper: h, base });
    link.rigidity = link.rigidity.filter((r) => !(r.kind !== 'dist' && rigidityTouches(r, h)));
  }
  return affected;
}

/** Rebuild the attachment of the given helpers from the current rest geometry. */
function reattachHelpers(m: Model, link: Link, affected: { helper: ID; base: ID }[]): void {
  for (const { helper, base } of affected) {
    if (!m.points[helper] || !m.points[base]) continue;
    link.rigidity = link.rigidity.filter((r) => !rigidityTouches(r, helper));
    attachHelper(m, link, helper, base);
  }
}

/** Rebuild a link's shape rigidity from its current positions, keeping joint-helper attachments. */
export function rebuildShapeRigidity(m: Model, linkId: ID, order?: ID[], frame?: ID[]): void {
  const link = m.links[linkId];
  if (!link || link.kind === 'cylinder') return;
  const helperIds = link.helperIds.filter((h) => m.points[h]?.role === 'helper');
  const helperRigidity = link.rigidity.filter((r) => helperIds.some((h) => rigidityTouches(r, h)));
  link.rigidity = [...buildRigidity(m, order ?? linkShapePointIds(m, link), frame), ...helperRigidity];
}

/**
 * Rebuild shape rigidity so that `pointId` is a dependent point: the frame
 * triangle is chosen among the OTHER vertices (when three non-collinear ones
 * exist), so moving the vertex cannot tilt the rest of the link.
 */
export function reframeRigidityAway(m: Model, link: Link, pointId: ID): void {
  if (link.kind === 'cylinder') return;
  const others = linkShapePointIds(m, link).filter((id) => id !== pointId);
  const frame = findFrame(m, others);
  rebuildShapeRigidity(m, link.id, [...others, pointId], frame.length === 3 ? frame : undefined);
}

export interface AutoJoinOptions {
  defaultJoint: JointType;
  /** Hinge axis for pins between links that are not on a common sketch plane (the common plane's normal is used otherwise). */
  axis: Vec3;
  /** Extra vertex pairs [vertexOfLink, vertexOfOtherLink] to treat as coincident even if not exactly so. */
  extraPairs?: [ID, ID][];
  tol?: number;
}

/**
 * Join vertices of `link` (restricted to `vertexIds`) to coincident vertices of
 * other links:
 *  - two consecutive vertices of `link` that coincide with the two ends of an
 *    edge of one other link become ONE edge–edge revolute joint (a crease);
 *    vertex pins already present on that pair are replaced;
 *  - any other coincident vertex gets the default joint (hinge axis = the
 *    normal of the sketch plane both links share) or, when the links are not
 *    on a common sketch plane, a spherical joint. A link that is already
 *    attached to a crease vertex through one of the crease partners is not
 *    pinned again.
 * Returns the created joints.
 */
export function autoJoinCoincident(m: Model, link: Link, vertexIds: ID[], opts: AutoJoinOptions): Joint[] {
  const tol = opts.tol ?? 1e-6 * Math.max(1, modelSize(m));
  const wanted = new Set(vertexIds);
  // vertex of link -> coincident vertices of other links
  const matches = new Map<ID, ID[]>();
  const consider = (a: ID, b: ID) => {
    if (!m.points[a] || !m.points[b] || m.points[b].linkId === link.id) return;
    const arr = matches.get(a) ?? [];
    if (!arr.includes(b)) arr.push(b);
    matches.set(a, arr);
  };
  for (const [a, b] of opts.extraPairs ?? []) consider(a, b);
  for (const v of link.pointIds) {
    if (!wanted.has(v)) continue;
    const p = m.points[v].pos;
    for (const q of Object.values(m.points)) {
      if (q.linkId === link.id || (q.role !== 'vertex' && q.role !== 'axis')) continue;
      if (dist(p, q.pos) <= tol) consider(v, q.id);
    }
  }
  const created: Joint[] = [];
  /** vertex -> links it was creased with */
  const creasePartners = new Map<ID, Set<ID>>();
  const pairedAlready = (a: ID, b: ID) => jointsAtPoint(m, a).some((j) => j.pairs?.some(([x, y]) => (x === a && y === b) || (x === b && y === a)));
  // 1) creases: consecutive matched vertices forming an edge of the other link
  for (const [a, b] of linkEdges(m, link)) {
    const ma = matches.get(a) ?? [];
    const mb = matches.get(b) ?? [];
    for (const qa of ma) {
      const other = m.links[m.points[qa].linkId];
      if (!other) continue;
      const qb = mb.find((x) => m.points[x].linkId === other.id);
      if (!qb) continue;
      const isEdge = linkEdges(m, other).some(([x, y]) => (x === qa && y === qb) || (x === qb && y === qa));
      if (!isEdge) continue;
      // replace vertex pins on these pairs by one crease
      for (const [v, q] of [[a, qa], [b, qb]] as [ID, ID][]) {
        for (const j of jointsAtPoint(m, v)) {
          if (j.a.kind === 'vertex' && !isConstructionRef(j.b) && j.b.kind === 'vertex' && j.pairs?.some(([x, y]) => (x === v && y === q) || (x === q && y === v))) removeJoint(m, j.id);
        }
      }
      const alreadyCrease = Object.values(m.joints).some((j) => j.type === 'revolute' && j.pairs && j.pairs.length === 2 && j.pairs.every(([x, y]) => ([a, b].includes(x) && [qa, qb].includes(y)) || ([a, b].includes(y) && [qa, qb].includes(x))));
      if (!alreadyCrease) {
        const j = addJoint(m, 'revolute', { linkId: link.id, kind: 'edge', pointIds: [a, b] }, { linkId: other.id, kind: 'edge', pointIds: [qa, qb] });
        if (j) created.push(j);
      }
      for (const v of [a, b]) {
        const set = creasePartners.get(v) ?? new Set<ID>();
        set.add(other.id);
        creasePartners.set(v, set);
      }
    }
  }
  // 2) remaining coincident vertices
  for (const [v, qs] of matches) {
    const partners = creasePartners.get(v) ?? new Set<ID>();
    for (const q of qs) {
      const other = m.links[m.points[q].linkId];
      if (!other || partners.has(other.id) || pairedAlready(v, q)) continue;
      // already attached to this crease vertex through one of the crease partners?
      const viaPartner = partners.size > 0 && jointsAtPoint(m, q).some((j) => {
        const otherSide = j.a.linkId === other.id ? j.b : j.a;
        return !isConstructionRef(otherSide) && partners.has(otherSide.linkId);
      });
      if (viaPartner) continue;
      const shared = sharedSketchPlane(m, link.id, other.id);
      const type: JointType = shared ? opts.defaultJoint : 'spherical';
      const axis = shared?.dir ?? opts.axis;
      const j = addJoint(m, type, { linkId: link.id, kind: 'vertex', pointIds: [v] }, { linkId: other.id, kind: 'vertex', pointIds: [q] }, { axis });
      if (j) created.push(j);
    }
  }
  if (created.length) {
    const res = solveSketch(m, { maxIter: 60 });
    commitSketch(m, res);
  }
  return created;
}

/** The construction plane both links are sketched on, if they share one. */
export function sharedSketchPlane(m: Model, a: ID, b: ID): Construction | null {
  const pa = bodyPlaneJoint(m, a);
  const pb = bodyPlaneJoint(m, b);
  if (!pa || !pb || !isConstructionRef(pa.b) || !isConstructionRef(pb.b) || pa.b.constructionId !== pb.b.constructionId) return null;
  return m.construction[pa.b.constructionId] ?? null;
}

export interface SketchPlaneFit {
  origin: Vec3;
  normal: Vec3;
  /** True when the fitted plane coincides with the given sketch plane (the polygon keeps its 2-D constraint). */
  onSketchPlane: boolean;
}

/**
 * Plane for a sketched polygon. Vertices snapped onto existing geometry are
 * authoritative: three or more non-collinear snapped vertices define the plane
 * (e.g. the last panel of a folded origami vertex); two define the plane that
 * contains their edge and is closest to the sketch plane; one gives the plane
 * parallel to the sketch plane through it; none gives the sketch plane itself.
 */
export function fitSketchPlane(points: Vec3[], snapped: boolean[], sketch: { o: Vec3; n: Vec3 }, tol = 1e-6): SketchPlaneFit {
  const n0 = normalize(sketch.n);
  const sp = points.filter((_, i) => snapped[i]);
  const onPlane = (o: Vec3, n: Vec3) => Math.abs(dot(sub(o, sketch.o), n0)) <= tol && Math.abs(Math.abs(dot(n, n0)) - 1) <= tol;
  if (sp.length >= 3) {
    // Newell normal of the snapped vertices
    let n: Vec3 = [0, 0, 0];
    for (let i = 0; i < sp.length; i++) {
      const a = sp[i];
      const b = sp[(i + 1) % sp.length];
      n = add(n, [(a[1] - b[1]) * (a[2] + b[2]), (a[2] - b[2]) * (a[0] + b[0]), (a[0] - b[0]) * (a[1] + b[1])]);
    }
    if (len(n) > 1e-9) {
      const nn = normalize(n);
      const normal = dot(nn, n0) < 0 ? scale(nn, -1) : nn;
      return { origin: sp[0], normal, onSketchPlane: onPlane(sp[0], normal) };
    }
  }
  if (sp.length >= 2) {
    const e = normalize(sub(sp[1], sp[0]));
    let n = sub(n0, scale(e, dot(n0, e)));
    if (len(n) < 1e-6) n = perpendicular(e); // edge parallel to the sketch normal: any perpendicular plane
    const normal = normalize(n);
    return { origin: sp[0], normal, onSketchPlane: onPlane(sp[0], normal) };
  }
  if (sp.length === 1) return { origin: sp[0], normal: n0, onSketchPlane: onPlane(sp[0], n0) };
  return { origin: sketch.o, normal: n0, onSketchPlane: true };
}

/** Project a point onto a plane. */
export function projectToPlane(p: Vec3, origin: Vec3, normal: Vec3): Vec3 {
  const n = normalize(normal);
  return sub(p, scale(n, dot(sub(p, origin), n)));
}

/** Intersection of a ray with a plane, or null when (nearly) parallel or when the hit lies behind the ray origin. */
export function rayPlane(o: Vec3, d: Vec3, origin: Vec3, normal: Vec3): Vec3 | null {
  const n = normalize(normal);
  const dd = normalize(d);
  const denom = dot(dd, n);
  if (Math.abs(denom) < 1e-6) return null;
  const t = dot(sub(origin, o), n) / denom;
  if (t < 0) return null;
  return add(o, scale(dd, t));
}

/**
 * Place a free sketch vertex on a fitted plane: along its pointer ray when the
 * ray meets the plane at a reasonable angle in front of the camera, otherwise
 * by orthogonal projection (grazing rays would fling the vertex far away).
 */
export function placeOnFittedPlane(p: Vec3, ray: { o: Vec3; d: Vec3 } | undefined, origin: Vec3, normal: Vec3, minCos = 0.15): Vec3 {
  const proj = projectToPlane(p, origin, normal);
  if (!ray) return proj;
  const grazing = Math.abs(dot(normalize(ray.d), normalize(normal))) < minCos;
  if (grazing) return proj;
  return rayPlane(ray.o, ray.d, origin, normal) ?? proj;
}

export { cross };
