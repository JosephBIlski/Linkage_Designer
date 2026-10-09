/**
 * Model construction and editing helpers. All functions mutate the passed
 * Model in place (undo is implemented by snapshotting the whole model, which
 * is cheap for mechanisms of this size).
 */
import type {
  Construction,
  ConstructionRef,
  Feature,
  FeatureKind,
  ID,
  Joint,
  JointType,
  Link,
  LinkKind,
  LinkParams,
  Model,
  Point,
  PointRole,
  RigidityConstraint,
  Vec3,
} from './types';
import { DEFAULT_SETTINGS, isConstructionRef } from './types';
import { add, cross, dist, dot, len, normalize, perpendicular, planeBasis, scale, sub, triple } from './geometry';
import { axisRotation, constRef, cosValue, twistValue, worldAxisRef } from './constraints';

// ---------------------------------------------------------------------------
// Model creation
// ---------------------------------------------------------------------------

export function createModel(): Model {
  const m: Model = {
    version: 1,
    points: {},
    links: {},
    joints: {},
    construction: {},
    drivers: [],
    targets: [],
    settings: { ...DEFAULT_SETTINGS, displayPointIds: [] },
    nextId: 1,
  };
  addBuiltinDatums(m);
  return m;
}

/** Creo-style default datum geometry: origin, X/Y/Z axes, TOP (XY), FRONT (XZ) and RIGHT (YZ) planes. */
export function addBuiltinDatums(m: Model): void {
  const put = (c: Construction) => (m.construction[c.id] = c);
  put({ id: 'point_origin', kind: 'point', name: 'ORIGIN', origin: [0, 0, 0], builtin: true });
  put({ id: 'axis_x', kind: 'axis', name: 'X', origin: [0, 0, 0], dir: [1, 0, 0], size: 20, builtin: true });
  put({ id: 'axis_y', kind: 'axis', name: 'Y', origin: [0, 0, 0], dir: [0, 1, 0], size: 20, builtin: true });
  put({ id: 'axis_z', kind: 'axis', name: 'Z', origin: [0, 0, 0], dir: [0, 0, 1], size: 20, builtin: true });
  put({ id: 'plane_top', kind: 'plane', name: 'TOP', origin: [0, 0, 0], dir: [0, 0, 1], xDir: [1, 0, 0], size: 6, builtin: true });
  put({ id: 'plane_front', kind: 'plane', name: 'FRONT', origin: [0, 0, 0], dir: [0, 1, 0], xDir: [1, 0, 0], size: 6, builtin: true });
  put({ id: 'plane_right', kind: 'plane', name: 'RIGHT', origin: [0, 0, 0], dir: [1, 0, 0], xDir: [0, 1, 0], size: 6, builtin: true });
}

export function newId(m: Model, prefix: string): ID {
  return `${prefix}_${m.nextId++}`;
}

export function cloneModel(m: Model): Model {
  return JSON.parse(JSON.stringify(m)) as Model;
}

// ---------------------------------------------------------------------------
// Points
// ---------------------------------------------------------------------------

export function addPoint(m: Model, linkId: ID, pos: Vec3, role: PointRole, name: string): Point {
  const p: Point = { id: newId(m, 'p'), pos: [pos[0], pos[1], pos[2]], linkId, role, name };
  m.points[p.id] = p;
  return p;
}

export const P = (m: Model, id: ID): Vec3 => m.points[id].pos;

/** Visible + hidden points of a link. */
export function linkAllPointIds(link: Link): ID[] {
  return [...link.pointIds, ...link.helperIds];
}

/** Points that define the link's own shape (visible vertices + cylinder reference points, no joint helpers). */
export function linkShapePointIds(m: Model, link: Link): ID[] {
  return linkAllPointIds(link).filter((id) => m.points[id].role !== 'helper');
}

// ---------------------------------------------------------------------------
// Rigidity rules
// ---------------------------------------------------------------------------

/** First three non-collinear points among ids (or fewer if none exist). */
export function findFrame(m: Model, ids: ID[]): ID[] {
  if (ids.length < 2) return [...ids];
  const a = ids[0];
  let b: ID | null = null;
  for (let i = 1; i < ids.length; i++) {
    if (dist(P(m, ids[i]), P(m, a)) > 1e-9) {
      b = ids[i];
      break;
    }
  }
  if (!b) return [a];
  let best: ID | null = null;
  let bestArea = 0;
  for (const id of ids) {
    if (id === a || id === b) continue;
    const area = len(cross(sub(P(m, b), P(m, a)), sub(P(m, id), P(m, a))));
    if (area > bestArea) {
      bestArea = area;
      best = id;
    }
  }
  const scaleLen = dist(P(m, a), P(m, b));
  if (best && bestArea > 1e-7 * scaleLen * scaleLen) return [a, b, best];
  return [a, b];
}

/**
 * Build rigidity constraints for a set of shape points using the frame-triangle rule:
 *  - frame triangle: 3 mutual distances
 *  - every other point: 3 distances to the frame, or (if coplanar with the frame)
 *    2 distances + a coplanarity constraint, which avoids the first-order flexibility
 *    of distance-only constraints on flat point sets.
 */
export function buildRigidity(m: Model, ids: ID[], frameOverride?: ID[]): RigidityConstraint[] {
  const out: RigidityConstraint[] = [];
  if (ids.length < 2) return out;
  const frame = frameOverride && frameOverride.length === 3 && frameOverride.every((id) => ids.includes(id)) ? frameOverride : findFrame(m, ids);
  const d = (a: ID, b: ID): RigidityConstraint => ({ kind: 'dist', a, b, length: dist(P(m, a), P(m, b)) });
  if (frame.length === 2) {
    out.push(d(frame[0], frame[1]));
    return out;
  }
  const [f0, f1, f2] = frame;
  out.push(d(f0, f1), d(f0, f2), d(f1, f2));
  const n = cross(sub(P(m, f1), P(m, f0)), sub(P(m, f2), P(m, f0)));
  const area = len(n);
  for (const id of ids) {
    if (frame.includes(id)) continue;
    const vol = Math.abs(triple(sub(P(m, f1), P(m, f0)), sub(P(m, f2), P(m, f0)), sub(P(m, id), P(m, f0))));
    const coplanar = vol < 1e-6 * area * Math.max(1, dist(P(m, id), P(m, f0)));
    if (coplanar) {
      // choose the two frame points least collinear with the new point
      const pairs: [ID, ID][] = [
        [f0, f1],
        [f0, f2],
        [f1, f2],
      ];
      let best = pairs[0];
      let bestVal = -1;
      for (const [a, b] of pairs) {
        const v = len(cross(sub(P(m, id), P(m, a)), sub(P(m, b), P(m, a))));
        if (v > bestVal) {
          bestVal = v;
          best = [a, b];
        }
      }
      out.push(d(id, best[0]), d(id, best[1]), { kind: 'coplanar', a: f0, b: f1, c: f2, p: id });
    } else {
      out.push(d(id, f0), d(id, f1), d(id, f2));
    }
  }
  return out;
}

/**
 * Attach a hidden helper point `h` rigidly to a link at base point `base` using
 * length-independent constraints (fixed distance + angle cosines + a relation to
 * an existing helper), so that changing the link's design lengths never fights
 * the helper attachment.
 */
export function attachHelper(m: Model, link: Link, h: ID, base: ID): void {
  const hp = P(m, h);
  const bp = P(m, base);
  const hLen = dist(hp, bp);
  const shape = linkShapePointIds(m, link).filter((id) => id !== base && dist(P(m, id), bp) > 1e-9);
  link.rigidity.push({ kind: 'dist', a: h, b: base, length: hLen, fixed: true });
  if (shape.length === 0) return;
  // farthest shape point from base
  let q = shape[0];
  for (const id of shape) if (dist(P(m, id), bp) > dist(P(m, q), bp)) q = id;
  link.rigidity.push({ kind: 'cos', h, p: base, q, value: cosValue(hp, bp, P(m, q)) });
  // a second, non-collinear shape point fully fixes the helper
  let s: ID | null = null;
  let bestArea = 0;
  for (const id of shape) {
    if (id === q) continue;
    const area = len(cross(sub(P(m, q), bp), sub(P(m, id), bp)));
    if (area > bestArea) {
      bestArea = area;
      s = id;
    }
  }
  if (s && bestArea > 1e-7 * dist(bp, P(m, q)) ** 2) {
    link.rigidity.push({ kind: 'cos', h, p: base, q: s, value: cosValue(hp, bp, P(m, s)) });
    return;
  }
  // bar-like link: relate to an existing helper to fix the roll about the bar
  const other = link.helperIds.find((id) => id !== h && m.points[id].role === 'helper');
  if (!other) return;
  const otherBase = helperBase(link, other);
  if (!otherBase) return;
  const u = sub(hp, bp);
  const w = sub(P(m, other), P(m, otherBase));
  if (len(cross(u, w)) < 1e-6 * len(u) * len(w)) {
    link.rigidity.push({ kind: 'parallel', a: base, b: h, c: otherBase, d: other });
  } else {
    link.rigidity.push({ kind: 'cos', h, p: base, q: other, value: cosValue(hp, bp, P(m, other)) });
    link.rigidity.push({ kind: 'twist', a: base, b: h, c: otherBase, d: other, value: twistValue(bp, hp, P(m, otherBase), P(m, other)) });
  }
}

/** Base point of a helper: the point it has a fixed distance to. */
export function helperBase(link: Link, h: ID): ID | null {
  for (const r of link.rigidity) {
    if (r.kind === 'dist' && r.fixed && r.a === h) return r.b;
  }
  return null;
}

/** Recompute rest lengths / angle values from the current positions (after a shape edit). */
export function refreshRigidity(m: Model, link: Link): void {
  for (const r of link.rigidity) {
    if (r.kind === 'dist') r.length = dist(P(m, r.a), P(m, r.b));
    else if (r.kind === 'cos') r.value = cosValue(P(m, r.h), P(m, r.p), P(m, r.q));
    else if (r.kind === 'twist') r.value = twistValue(P(m, r.a), P(m, r.b), P(m, r.c), P(m, r.d));
  }
  if (link.kind === 'bar') link.params = { ...link.params };
}

/** Bar length (design). */
export function barLength(m: Model, link: Link): number {
  const r = link.rigidity.find((c) => c.kind === 'dist' && !c.fixed && c.a === link.pointIds[0] && c.b === link.pointIds[1]);
  return r && r.kind === 'dist' ? r.length : dist(P(m, link.pointIds[0]), P(m, link.pointIds[1]));
}

// ---------------------------------------------------------------------------
// Link creation
// ---------------------------------------------------------------------------

export interface LinkOptions {
  name?: string;
  /** Construction plane the link is sketched on; adds a planar constraint on all its points. */
  onPlaneId?: ID | null;
  color?: string;
}

function baseLink(m: Model, kind: LinkKind, name: string | undefined): Link {
  const id = newId(m, 'link');
  const link: Link = {
    id,
    kind,
    name: name ?? `${kind} ${id.split('_')[1]}`,
    pointIds: [],
    helperIds: [],
    params: {},
    rigidity: [],
    locked: false,
    ground: false,
    flexible: false,
    stiffness: 0.5,
  };
  m.links[id] = link;
  return link;
}

function finishLink(m: Model, link: Link, opts: LinkOptions): Link {
  link.rigidity = buildRigidity(m, linkShapePointIds(m, link));
  if (opts.color) link.color = opts.color;
  if (opts.onPlaneId) setBodyPlane(m, link.id, opts.onPlaneId);
  return link;
}

/**
 * Keep a link on a construction plane, or release it. The constraint is the planar joint between the link's 'body'
 * feature (all of its visible points) and the plane, the same joint geometry drawn in 2-D mode receives when it is
 * created (finishLink), which compile.ts turns into frozen coordinates for axis-aligned planes. A link is on at
 * most one plane: an existing body planar joint is removed before the new one is added, and `planeId` null only
 * removes it. Only bars and polygons are planar bodies (a prism or cylinder has points off every plane), and an
 * unknown plane leaves the link as it was. Returns the body planar joint the link has after the call (null when it
 * has none). The points are not moved onto the plane here; the caller re-solves (trySolveCommit) so that a pose
 * which cannot satisfy the new constraint is refused instead of committed.
 */
export function setBodyPlane(m: Model, linkId: ID, planeId: ID | null): Joint | null {
  const link = m.links[linkId];
  if (!link) return null;
  const existing = bodyPlaneJoint(m, linkId);
  if (planeId === null) {
    if (existing) removeJoint(m, existing.id);
    return null;
  }
  if (m.construction[planeId]?.kind !== 'plane' || (link.kind !== 'bar' && link.kind !== 'polygon')) return existing;
  if (existing) removeJoint(m, existing.id);
  return addJoint(m, 'planar', { linkId: link.id, kind: 'body', pointIds: [...link.pointIds] }, { constructionId: planeId });
}

export function addBar(m: Model, a: Vec3, b: Vec3, opts: LinkOptions = {}): Link {
  const link = baseLink(m, 'bar', opts.name);
  const pa = addPoint(m, link.id, a, 'vertex', 'A');
  const pb = addPoint(m, link.id, b, 'vertex', 'B');
  link.pointIds = [pa.id, pb.id];
  return finishLink(m, link, opts);
}

/** Regular planar polygon: centre, unit normal, circumradius, number of sides, optional in-plane x direction. */
export function addPolygon(m: Model, center: Vec3, normal: Vec3, radius: number, sides: number, opts: LinkOptions & { xDir?: Vec3; startAngle?: number } = {}): Link {
  const link = baseLink(m, 'polygon', opts.name);
  const n = normalize(normal);
  const [u0, v0] = planeBasis(n);
  const u = opts.xDir ? normalize(sub(opts.xDir, scale(n, dot(opts.xDir, n)))) : u0;
  const v = opts.xDir ? normalize(cross(n, u)) : v0;
  const a0 = opts.startAngle ?? 0;
  for (let i = 0; i < sides; i++) {
    const t = a0 + (2 * Math.PI * i) / sides;
    const p = add(center, add(scale(u, radius * Math.cos(t)), scale(v, radius * Math.sin(t))));
    link.pointIds.push(addPoint(m, link.id, p, 'vertex', `V${i}`).id);
  }
  link.params = { sides, radius };
  return finishLink(m, link, opts);
}

/**
 * Generic link from explicit vertex positions (bars, polygons and prisms).
 * Rigidity is built from the given positions; `params` is stored as given.
 */
export function addLinkFromPoints(m: Model, kind: 'bar' | 'polygon' | 'prism', pts: Vec3[], params: LinkParams, opts: LinkOptions = {}): Link {
  const link = baseLink(m, kind, opts.name);
  pts.forEach((p, i) => link.pointIds.push(addPoint(m, link.id, p, 'vertex', kind === 'bar' ? (i === 0 ? 'A' : 'B') : `V${i}`).id));
  link.params = { ...params };
  return finishLink(m, link, opts);
}

/** Explicit planar polygon from vertex positions (must be coplanar). */
export function addPolygonFromPoints(m: Model, pts: Vec3[], opts: LinkOptions = {}): Link {
  const link = baseLink(m, 'polygon', opts.name);
  pts.forEach((p, i) => link.pointIds.push(addPoint(m, link.id, p, 'vertex', `V${i}`).id));
  link.params = { sides: pts.length };
  return finishLink(m, link, opts);
}

/** Right prism: regular n-gon base at `center` extruded `height` along the unit normal. */
export function addPrism(m: Model, center: Vec3, normal: Vec3, radius: number, sides: number, height: number, opts: LinkOptions & { xDir?: Vec3 } = {}): Link {
  const link = baseLink(m, 'prism', opts.name);
  const n = normalize(normal);
  const [u0, v0] = planeBasis(n);
  const u = opts.xDir ? normalize(sub(opts.xDir, scale(n, dot(opts.xDir, n)))) : u0;
  const v = opts.xDir ? normalize(cross(n, u)) : v0;
  for (let i = 0; i < sides; i++) {
    const t = (2 * Math.PI * i) / sides;
    const p = add(center, add(scale(u, radius * Math.cos(t)), scale(v, radius * Math.sin(t))));
    link.pointIds.push(addPoint(m, link.id, p, 'vertex', `V${i}`).id);
  }
  for (let i = 0; i < sides; i++) {
    const p = add(P(m, link.pointIds[i]), scale(n, height));
    link.pointIds.push(addPoint(m, link.id, p, 'vertex', `V${i + sides}`).id);
  }
  link.params = { sides, radius, height };
  return finishLink(m, link, opts);
}

/** Cylinder between axis end points a and b with the given radius. */
export function addCylinder(m: Model, a: Vec3, b: Vec3, radius: number, opts: LinkOptions = {}): Link {
  const link = baseLink(m, 'cylinder', opts.name);
  const pa = addPoint(m, link.id, a, 'axis', 'A');
  const pb = addPoint(m, link.id, b, 'axis', 'B');
  const axis = normalize(sub(b, a));
  const [u, v] = planeBasis(axis);
  const r1 = addPoint(m, link.id, add(a, scale(u, radius)), 'ref', 'R1');
  const r2 = addPoint(m, link.id, add(a, scale(v, radius)), 'ref', 'R2');
  link.pointIds = [pa.id, pb.id];
  link.helperIds = [r1.id, r2.id];
  link.params = { radius, height: dist(a, b) };
  // rigidity: axis length (design) + reference points with length-independent attachments
  link.rigidity.push({ kind: 'dist', a: pa.id, b: pb.id, length: dist(a, b) });
  link.rigidity.push({ kind: 'dist', a: r1.id, b: pa.id, length: radius, fixed: true });
  link.rigidity.push({ kind: 'cos', h: r1.id, p: pa.id, q: pb.id, value: 0 });
  link.rigidity.push({ kind: 'dist', a: r2.id, b: pa.id, length: radius, fixed: true });
  link.rigidity.push({ kind: 'cos', h: r2.id, p: pa.id, q: pb.id, value: 0 });
  link.rigidity.push({ kind: 'cos', h: r2.id, p: pa.id, q: r1.id, value: 0 });
  if (opts.color) link.color = opts.color;
  return link;
}

// ---------------------------------------------------------------------------
// Construction geometry
// ---------------------------------------------------------------------------

export function addConstructionPoint(m: Model, origin: Vec3, name?: string): Construction {
  const c: Construction = { id: newId(m, 'cpoint'), kind: 'point', name: name ?? `PNT${m.nextId}`, origin: [...origin] as Vec3 };
  m.construction[c.id] = c;
  return c;
}

export function addConstructionAxis(m: Model, origin: Vec3, dir: Vec3, name?: string, size = 10): Construction {
  const c: Construction = { id: newId(m, 'caxis'), kind: 'axis', name: name ?? `AXIS${m.nextId}`, origin: [...origin] as Vec3, dir: normalize(dir), size };
  m.construction[c.id] = c;
  return c;
}

export function addConstructionPlane(m: Model, origin: Vec3, normal: Vec3, name?: string, xDir?: Vec3, size = 6): Construction {
  const n = normalize(normal);
  const x = xDir ? normalize(sub(xDir, scale(n, dot(xDir, n)))) : perpendicular(n);
  const c: Construction = { id: newId(m, 'cplane'), kind: 'plane', name: name ?? `DTM${m.nextId}`, origin: [...origin] as Vec3, dir: n, xDir: x, size };
  m.construction[c.id] = c;
  return c;
}

/** Plane through three points. */
export function addConstructionPlane3(m: Model, a: Vec3, b: Vec3, c: Vec3, name?: string): Construction | null {
  const n = cross(sub(b, a), sub(c, a));
  if (len(n) < 1e-9) return null;
  return addConstructionPlane(m, a, n, name, sub(b, a));
}

/** Plane offset from an existing plane along its normal. */
export function addOffsetPlane(m: Model, baseId: ID, offset: number, name?: string): Construction | null {
  const base = m.construction[baseId];
  if (!base || base.kind !== 'plane' || !base.dir) return null;
  return addConstructionPlane(m, add(base.origin, scale(base.dir, offset)), base.dir, name, base.xDir, base.size);
}

// ---------------------------------------------------------------------------
// Features
// ---------------------------------------------------------------------------

/** Faces of a link as ordered vertex-id lists. */
export function linkFaces(m: Model, link: Link): ID[][] {
  if (link.kind === 'polygon') return [[...link.pointIds]];
  if (link.kind === 'prism') {
    const n = link.params.sides ?? link.pointIds.length / 2;
    const bottom = link.pointIds.slice(0, n);
    const top = link.pointIds.slice(n, 2 * n);
    const faces: ID[][] = [bottom, [...top].reverse()];
    for (let i = 0; i < n; i++) {
      faces.push([bottom[i], bottom[(i + 1) % n], top[(i + 1) % n], top[i]]);
    }
    return faces;
  }
  if (link.kind === 'cylinder') {
    const [a, b] = link.pointIds;
    const [r1, r2] = link.helperIds;
    const off = sub(P(m, b), P(m, a));
    // end caps are represented by their centre and two reference points (top cap uses translated refs, which are not real points)
    void off;
    return [[a, r1, r2]];
  }
  return [];
}

/** Edges of a link as vertex-id pairs. */
export function linkEdges(m: Model, link: Link): [ID, ID][] {
  if (link.kind === 'bar') return [[link.pointIds[0], link.pointIds[1]]];
  if (link.kind === 'polygon') {
    const n = link.pointIds.length;
    return link.pointIds.map((id, i) => [id, link.pointIds[(i + 1) % n]] as [ID, ID]);
  }
  if (link.kind === 'prism') {
    const n = link.params.sides ?? link.pointIds.length / 2;
    const e: [ID, ID][] = [];
    for (let i = 0; i < n; i++) {
      e.push([link.pointIds[i], link.pointIds[(i + 1) % n]]);
      e.push([link.pointIds[n + i], link.pointIds[n + ((i + 1) % n)]]);
      e.push([link.pointIds[i], link.pointIds[n + i]]);
    }
    return e;
  }
  if (link.kind === 'cylinder') return [[link.pointIds[0], link.pointIds[1]]];
  return [];
}

export function featureOf(link: Link, kind: FeatureKind, pointIds: ID[]): Feature {
  return { linkId: link.id, kind, pointIds };
}

/** Edge and cylinder-axis features both carry a line through two end points and are treated alike by axis-type joints. */
const axisLike = (k: FeatureKind): boolean => k === 'edge' || k === 'axis';

/** Which joint types can connect two features (or a feature and construction geometry)? */
export function jointCompatible(m: Model, type: JointType, a: Feature, b: Feature | ConstructionRef): boolean {
  if (a.kind === 'body') return type === 'planar' && isConstructionRef(b) && m.construction[b.constructionId]?.kind === 'plane';
  if (isConstructionRef(b)) {
    const c = m.construction[b.constructionId];
    if (!c) return false;
    switch (type) {
      case 'spherical':
        return a.kind === 'vertex' && c.kind === 'point';
      case 'revolute':
        return (a.kind === 'vertex' && (c.kind === 'point' || c.kind === 'axis')) || ((a.kind === 'edge' || a.kind === 'axis') && c.kind === 'axis');
      case 'cylindrical':
      case 'prismatic':
      case 'screw':
        return (a.kind === 'vertex' || a.kind === 'edge' || a.kind === 'axis') && c.kind === 'axis';
      case 'planar':
        return (a.kind === 'vertex' || a.kind === 'edge' || a.kind === 'face') && c.kind === 'plane';
    }
  }
  const fb = b as Feature;
  if (fb.kind === 'body' || a.linkId === fb.linkId) return false;
  switch (type) {
    case 'spherical':
      return a.kind === 'vertex' && fb.kind === 'vertex';
    case 'revolute':
    case 'cylindrical':
    case 'prismatic':
    case 'screw':
      return (a.kind === 'vertex' || axisLike(a.kind)) && (fb.kind === 'vertex' || axisLike(fb.kind)) && !(type !== 'revolute' && a.kind === 'vertex' && fb.kind === 'vertex');
    case 'planar':
      return (a.kind === 'face' && (fb.kind === 'face' || fb.kind === 'vertex' || fb.kind === 'edge')) || (fb.kind === 'face' && (a.kind === 'vertex' || a.kind === 'edge'));
  }
}

// ---------------------------------------------------------------------------
// Joints
// ---------------------------------------------------------------------------

export interface JointOptions {
  /** Axis direction for axis-type joints created from vertex features (default: sketch plane normal / +Z). */
  axis?: Vec3;
  pitch?: number;
  /**
   * Relative length tolerance (fraction of the longer edge) within which two edges joined by a revolute are treated
   * as the same length and merged into a crease (default EDGE_LENGTH_TOLERANCE).
   */
  lengthTolerance?: number;
}

/**
 * Default relative tolerance for "same length" edges in an edge–edge revolute: hand-placed geometry agrees to about
 * a snap distance (one part in a thousand of the edge), not to floating-point precision.
 */
export const EDGE_LENGTH_TOLERANCE = 1e-3;

/**
 * Relative difference below which two merged edge lengths are identical (floating-point noise of the Polygon tool):
 * no link has to adapt its rest geometry when the crease is created.
 */
export const CREASE_EXACT_TOLERANCE = 1e-9;

/**
 * Largest change of a rest distance (relative to the longer edge) an adapting link may take when a crease merges two
 * edges of slightly different length: twice the merge tolerance, i.e. the snap-sized noise the merge was meant to
 * absorb. A released solve that reshapes a panel by more than this is not absorbing noise but closing a loop that
 * rigid panels cannot close, and is refused (feasibility.ts adaptationLimit, kinematics.ts releaseDrift).
 */
export const CREASE_ADAPTATION_TOLERANCE = 2 * EDGE_LENGTH_TOLERANCE;

/** Current-pose lengths of the two edges of an edge/axis–edge/axis revolute (la for feature a, lb for b), else null. */
export function creaseEdgeLengths(m: Model, joint: Joint): { la: number; lb: number } | null {
  if (joint.type !== 'revolute' || isConstructionRef(joint.b)) return null;
  if (!axisLike(joint.a.kind) || !axisLike(joint.b.kind)) return null;
  const ids = [...joint.a.pointIds, ...joint.b.pointIds];
  if (ids.length < 4 || ids.some((id) => !m.points[id])) return null;
  return { la: dist(P(m, joint.a.pointIds[0]), P(m, joint.a.pointIds[1])), lb: dist(P(m, joint.b.pointIds[0]), P(m, joint.b.pointIds[1])) };
}

/** Point of `link` farthest from the line through the two axis points (null if all collinear). */
export function offAxisPoint(m: Model, link: Link, axis: [ID, ID]): ID | null {
  const a = P(m, axis[0]);
  const d = normalize(sub(P(m, axis[1]), a));
  let best: ID | null = null;
  let bestD = 1e-6 * Math.max(1, dist(a, P(m, axis[1])));
  for (const id of linkShapePointIds(m, link)) {
    if (axis.includes(id)) continue;
    const w = sub(P(m, id), a);
    const perp = len(sub(w, scale(d, dot(w, d))));
    if (perp > bestD) {
      bestD = perp;
      best = id;
    }
  }
  return best;
}

/** Create (and rigidly attach) a hidden helper point for `link` at `base` offset along `dir`. */
export function createHelper(m: Model, link: Link, base: ID, dir: Vec3, length: number): ID {
  const h = addPoint(m, link.id, add(P(m, base), scale(normalize(dir), length)), 'helper', 'h');
  link.helperIds.push(h.id);
  attachHelper(m, link, h.id, base);
  return h.id;
}

/** Axis points [p, q] of a joint side (helpers are created lazily by addJoint). */
export function jointAxisPoints(m: Model, joint: Joint, side: 'a' | 'b'): [ID, ID] | null {
  const f = side === 'a' ? joint.a : joint.b;
  if (isConstructionRef(f)) return null;
  if (f.kind === 'edge' || f.kind === 'axis') return [f.pointIds[0], f.pointIds[1]];
  if (f.kind === 'vertex') {
    const h = joint.helpers?.[side === 'a' ? 0 : 1];
    return h ? [f.pointIds[0], h] : null;
  }
  return null;
}

/** Create a joint between two features. Returns null if incompatible. */
export function addJoint(m: Model, type: JointType, a: Feature, b: Feature | ConstructionRef, opts: JointOptions = {}): Joint | null {
  if (!jointCompatible(m, type, a, b)) return null;
  const joint: Joint = { id: newId(m, 'joint'), type, a, b };
  const linkA = m.links[a.linkId];
  const linkB = isConstructionRef(b) ? null : m.links[b.linkId];
  const h = m.settings.helperOffset;
  const axisType = type === 'revolute' || type === 'cylindrical' || type === 'prismatic' || type === 'screw';

  if (type === 'spherical') {
    if (!isConstructionRef(b)) joint.pairs = [[a.pointIds[0], (b as Feature).pointIds[0]]];
  } else if (axisType) {
    // Determine the axis direction
    let axisDir: Vec3 | null = null;
    const edgeDir = (f: Feature): Vec3 | null => (f.kind === 'edge' || f.kind === 'axis' ? normalize(sub(P(m, f.pointIds[1]), P(m, f.pointIds[0]))) : null);
    if (isConstructionRef(b)) {
      const c = m.construction[b.constructionId];
      axisDir = c.kind === 'axis' && c.dir ? c.dir : (opts.axis ?? sketchNormal(m));
    } else {
      axisDir = edgeDir(a) ?? edgeDir(b as Feature) ?? opts.axis ?? sketchNormal(m);
    }
    joint.axis = axisDir;
    // Helpers for vertex features
    const helpers: ID[] = ['', ''];
    if (a.kind === 'vertex') helpers[0] = createHelper(m, linkA, a.pointIds[0], axisDir, h);
    if (linkB && (b as Feature).kind === 'vertex') helpers[1] = createHelper(m, linkB, (b as Feature).pointIds[0], axisDir, h);
    if (helpers[0] || helpers[1]) joint.helpers = helpers;

    const axA = jointAxisPoints(m, joint, 'a')!;
    let axB = linkB ? jointAxisPoints(m, joint, 'b')! : null;

    if (type === 'revolute' && axB) {
      const fa = a;
      const fb = b as Feature;
      if (fa.kind === 'vertex' && fb.kind === 'vertex') {
        joint.pairs = [
          [axA[0], axB[0]],
          [axA[1], axB[1]],
        ];
      } else if (fa.kind === 'vertex' || fb.kind === 'vertex') {
        // vertex on an edge: the vertex coincides with the nearest edge end point; the helper lies on the edge line
        const vertexSide = fa.kind === 'vertex' ? axA : axB;
        const edgeSide = fa.kind === 'vertex' ? axB : axA;
        const near = dist(P(m, vertexSide[0]), P(m, edgeSide[0])) <= dist(P(m, vertexSide[0]), P(m, edgeSide[1])) ? edgeSide[0] : edgeSide[1];
        joint.pairs = [[vertexSide[0], near]];
      } else {
        // edge/axis against edge/axis (origami crease, or a hinge along two edges of different length).
        // The end points are always paired by proximity: straight (A0–B0, A1–B1) or crossed (A0–B1, A1–B0).
        const la = dist(P(m, axA[0]), P(m, axA[1]));
        const lb = dist(P(m, axB[0]), P(m, axB[1]));
        const straight = dist(P(m, axA[0]), P(m, axB[0])) + dist(P(m, axA[1]), P(m, axB[1]));
        const crossed = dist(P(m, axA[0]), P(m, axB[1])) + dist(P(m, axA[1]), P(m, axB[0]));
        const tol = opts.lengthTolerance ?? EDGE_LENGTH_TOLERANCE;
        if (Math.abs(la - lb) <= tol * Math.max(la, lb)) {
          // same length within the snap tolerance: the end points coincide pairwise (merged solver variables).
          // A small mismatch is absorbed by one link's rest geometry (see creaseReleasePoints), never spread
          // over the mechanism by the least-squares solve.
          joint.pairs = straight <= crossed ? [[axA[0], axB[0]], [axA[1], axB[1]]] : [[axA[0], axB[1]], [axA[1], axB[0]]];
        } else {
          // genuinely different lengths: B's end points lie on A's line and the slide along it is locked.
          // Orient B like A first (a copy of the feature with reversed end points when the crossed pairing is
          // closer; the caller's object is never mutated), then measure the slide offset of B's first end point
          // along A in the units slideLock uses, (B0 − A0) · unit(A1 − A0), so the link keeps its position
          // along the axis instead of being dragged to A's first end point.
          if (crossed < straight) joint.b = { ...fb, pointIds: [fb.pointIds[1], fb.pointIds[0]] };
          axB = jointAxisPoints(m, joint, 'b')!;
          const dir = normalize(sub(P(m, axA[1]), P(m, axA[0])));
          joint.offsets = { slide: dot(sub(P(m, axB[0]), P(m, axA[0])), dir) };
        }
      }
    }
    if ((type === 'revolute' || type === 'cylindrical' || type === 'prismatic' || type === 'screw') && !axB) {
      // against construction geometry: capture the slide offset so the link does not jump along the axis
      const c = m.construction[(b as ConstructionRef).constructionId];
      if (c.kind === 'axis' && c.dir) {
        joint.offsets = { ...(joint.offsets ?? {}), slide: dot(sub(P(m, axA[0]), c.origin), c.dir) };
      }
    }
    if (type === 'prismatic' || type === 'screw') {
      const refA = offAxisPoint(m, linkA, axA) ?? createHelper(m, linkA, axA[0], perpendicular(axisDir), h);
      let refB: ID;
      if (linkB && axB) {
        refB = offAxisPoint(m, linkB, axB) ?? createHelper(m, linkB, axB[0], perpendicular(axisDir), h);
      } else {
        refB = refA; // unused for construction axes (world reference is used instead)
      }
      joint.refs = [refA, refB];
      if (!linkB) {
        const c = m.construction[(b as ConstructionRef).constructionId];
        const w = sub(P(m, refA), c.origin);
        const n0 = normalize(cross(c.dir!, w));
        joint.offsets = { ...(joint.offsets ?? {}), planeNormal: len(n0) > 0.5 ? n0 : perpendicular(c.dir!) };
      }
      if (type === 'screw') {
        joint.pitch = opts.pitch ?? 1;
        // capture the rotation offset so the screw coupling is measured from the creation pose (like the slide offset)
        const none = new Float64Array(0);
        let angle: number;
        if (linkB && axB) {
          angle = axisRotation(constRef(P(m, axA[0])), constRef(P(m, axA[1])), constRef(P(m, refA)), constRef(P(m, refB)), none);
          const dir = normalize(sub(P(m, axA[1]), P(m, axA[0])));
          joint.offsets = { ...(joint.offsets ?? {}), slide: dot(sub(P(m, axB[0]), P(m, axA[0])), dir), angle };
        } else {
          const c = m.construction[(b as ConstructionRef).constructionId];
          const o = c.origin;
          const d = c.dir!;
          angle = axisRotation(constRef(o), constRef([o[0] + d[0], o[1] + d[1], o[2] + d[2]]), constRef(worldAxisRef(o, d)), constRef(P(m, refA)), none);
          joint.offsets = { ...(joint.offsets ?? {}), angle };
        }
      }
    }
  }
  m.joints[joint.id] = joint;
  return joint;
}

/**
 * End points of the edge that must adapt ("rubber-band") after a crease merged two edges whose lengths differ
 * slightly (within the merge tolerance but beyond CREASE_EXACT_TOLERANCE, measured in the current pose). Merging
 * both end-point pairs makes the two links' rest lengths contradict each other by Δ, so a plain least-squares solve
 * would spread Δ over the whole mechanism and never converge; releasing one edge lets that link take the other's
 * length instead (commitSketch bakes it). The link of feature b adapts unless it is ground or locked, then the link
 * of a; `preferLinkId` names the side that should adapt when it may (e.g. the link being sketched). Returns [] when
 * no adaptation is needed (not a merged edge–edge crease, or identical lengths) or when neither side may move.
 * Callers pass the ids as freePointIds to solveSketch / solveSketchWithRelease and to commitSketch.
 */
export function creaseReleasePoints(m: Model, joint: Joint, preferLinkId?: ID): ID[] {
  if (!mergedCreaseMismatch(m, joint)) return [];
  const fa = joint.a;
  const fb = joint.b as Feature;
  const mayMove = (f: Feature): boolean => {
    const link = m.links[f.linkId];
    return !!link && !link.ground && !link.locked;
  };
  const side = (preferLinkId === fa.linkId ? [fa, fb] : [fb, fa]).find(mayMove);
  return side ? [side.pointIds[0], side.pointIds[1]] : [];
}

/**
 * Lengths of the two edges of a merged edge/axis crease (two end-point pairs) that differ by more than
 * CREASE_EXACT_TOLERANCE in the current pose, i.e. a crease that needs one link to adapt its rest geometry; null for
 * identical lengths and for joints that are not merged creases. creaseReleasePoints names the link that adapts; when
 * it names none (both panels ground or locked) the mismatch cannot be absorbed and the crease is refused
 * (feasibility.ts unabsorbedCreaseLengths).
 */
export function mergedCreaseMismatch(m: Model, joint: Joint): { la: number; lb: number } | null {
  if (joint.pairs?.length !== 2) return null;
  const lengths = creaseEdgeLengths(m, joint);
  if (!lengths) return null;
  return Math.abs(lengths.la - lengths.lb) > CREASE_EXACT_TOLERANCE * Math.max(lengths.la, lengths.lb) ? lengths : null;
}

export function sketchNormal(m: Model): Vec3 {
  const pl = m.construction[m.settings.sketchPlaneId];
  return pl?.dir ? pl.dir : [0, 0, 1];
}

/** Change a joint's type in place (recreates helpers as needed). Returns the new joint or null if incompatible. */
export function changeJointType(m: Model, jointId: ID, type: JointType, opts: JointOptions = {}): Joint | null {
  const old = m.joints[jointId];
  if (!old) return null;
  const a = old.a;
  const b = old.b;
  const axis = old.axis;
  const pitch = old.pitch;
  removeJoint(m, jointId);
  const j = addJoint(m, type, a, b, { axis: opts.axis ?? axis, pitch: opts.pitch ?? pitch, lengthTolerance: opts.lengthTolerance });
  return j;
}

export function removeJoint(m: Model, jointId: ID): void {
  const j = m.joints[jointId];
  if (!j) return;
  const removeHelper = (id: ID) => {
    const p = m.points[id];
    if (!p || p.role !== 'helper') return;
    const link = m.links[p.linkId];
    if (link) {
      link.helperIds = link.helperIds.filter((x) => x !== id);
      link.rigidity = link.rigidity.filter((r) => !rigidityTouches(r, id));
    }
    delete m.points[id];
  };
  for (const id of j.helpers ?? []) if (id) removeHelper(id);
  for (const id of j.refs ?? []) if (id && m.points[id]?.role === 'helper') removeHelper(id);
  m.drivers = m.drivers.filter((d) => d.jointId !== jointId);
  delete m.joints[jointId];
}

export function rigidityTouches(r: RigidityConstraint, id: ID): boolean {
  switch (r.kind) {
    case 'dist':
      return r.a === id || r.b === id;
    case 'coplanar':
      return r.a === id || r.b === id || r.c === id || r.p === id;
    case 'cos':
      return r.h === id || r.p === id || r.q === id;
    case 'parallel':
    case 'twist':
      return r.a === id || r.b === id || r.c === id || r.d === id;
  }
}

/** Joints attached to a point (through any feature containing it). */
export function jointsAtPoint(m: Model, pointId: ID): Joint[] {
  const out: Joint[] = [];
  for (const j of Object.values(m.joints)) {
    if (j.a.pointIds.includes(pointId) && j.a.kind !== 'body') out.push(j);
    else if (!isConstructionRef(j.b) && j.b.pointIds.includes(pointId) && j.b.kind !== 'body') out.push(j);
  }
  return out;
}

/** The planar (sketch) constraint joint of a link, if any. */
export function bodyPlaneJoint(m: Model, linkId: ID): Joint | null {
  for (const j of Object.values(m.joints)) if (j.type === 'planar' && j.a.kind === 'body' && j.a.linkId === linkId) return j;
  return null;
}

/**
 * Creases that the sketch-plane constraint makes rigid: edge/axis–edge/axis revolute joints with two merged pairs
 * whose links are both kept on the same construction plane (setBodyPlane) and whose axis lies in that plane
 * (|axis · normal| < 1e-6, measured in the current pose). Folding about an in-plane axis would lift points of both
 * links out of the plane, so such a crease cannot rotate at all and contributes nothing to the DOF count; this is
 * why a flat origami vertex drawn in 2-D mode reports DOF 0 although every crease is a hinge. The Mechanism panel
 * and the DOF chip explain it with SIM.creasesLocked. Hinges between edges of different length (slide-locked, no
 * merged pairs) and links on two different planes are not reported.
 */
export function creasesLockedByPlane(m: Model): ID[] {
  const out: ID[] = [];
  for (const j of Object.values(m.joints)) {
    if (j.type !== 'revolute' || isConstructionRef(j.b) || j.pairs?.length !== 2) continue;
    if (!axisLike(j.a.kind) || !axisLike(j.b.kind)) continue;
    const pa = bodyPlaneJoint(m, j.a.linkId);
    const pb = bodyPlaneJoint(m, j.b.linkId);
    if (!pa || !pb || !isConstructionRef(pa.b) || !isConstructionRef(pb.b) || pa.b.constructionId !== pb.b.constructionId) continue;
    const plane = m.construction[pa.b.constructionId];
    const [p0, p1] = j.a.pointIds;
    if (!plane?.dir || !m.points[p0] || !m.points[p1]) continue;
    const axis = normalize(sub(P(m, p1), P(m, p0)));
    if (len(axis) > 0.5 && Math.abs(dot(axis, normalize(plane.dir))) < 1e-6) out.push(j.id);
  }
  return out;
}

export function removeLink(m: Model, linkId: ID): void {
  const link = m.links[linkId];
  if (!link) return;
  for (const j of Object.values(m.joints)) {
    if (j.a.linkId === linkId || (!isConstructionRef(j.b) && j.b.linkId === linkId)) removeJoint(m, j.id);
  }
  for (const id of linkAllPointIds(link)) delete m.points[id];
  m.drivers = m.drivers.filter((d) => d.linkId !== linkId);
  m.targets = m.targets.filter((t) => m.points[t.pointId] !== undefined);
  m.settings.displayPointIds = m.settings.displayPointIds.filter((id) => m.points[id] !== undefined);
  delete m.links[linkId];
}

export function removeConstruction(m: Model, id: ID): boolean {
  const c = m.construction[id];
  if (!c || c.builtin) return false;
  for (const j of Object.values(m.joints)) if (isConstructionRef(j.b) && j.b.constructionId === id) removeJoint(m, j.id);
  m.targets = m.targets.filter((t) => t.constructionId !== id);
  delete m.construction[id];
  return true;
}

/** Set the ground link (null clears). */
export function setGround(m: Model, linkId: ID | null): void {
  for (const l of Object.values(m.links)) l.ground = l.id === linkId;
}

export function groundLink(m: Model): Link | null {
  return Object.values(m.links).find((l) => l.ground) ?? null;
}

/** Translate all points of a link rigidly. */
export function translateLink(m: Model, link: Link, delta: Vec3): void {
  for (const id of linkAllPointIds(link)) m.points[id].pos = add(m.points[id].pos, delta);
}

// ---------------------------------------------------------------------------
// Drivers
// ---------------------------------------------------------------------------

/** Candidate angle drivers: bars / polygon edges with one end joined (directly) to the ground link or construction geometry. */
export function candidateAngleDrivers(m: Model): { linkId: ID; pivotId: ID; tipId: ID; axis: Vec3 }[] {
  const out: { linkId: ID; pivotId: ID; tipId: ID; axis: Vec3 }[] = [];
  const ground = groundLink(m);
  for (const link of Object.values(m.links)) {
    if (link.ground || link.kind === 'cylinder') continue;
    for (const pid of link.pointIds) {
      const js = jointsAtPoint(m, pid);
      for (const j of js) {
        const other = j.a.linkId === link.id ? j.b : j.a;
        const grounded = isConstructionRef(other) ? m.construction[other.constructionId]?.kind !== 'plane' : ground !== null && other.linkId === ground.id;
        if (!grounded) continue;
        if (j.type !== 'revolute' && j.type !== 'spherical' && j.type !== 'cylindrical') continue;
        const axis = j.axis ?? sketchNormal(m);
        // tip: farthest other visible point of this link
        let tip: ID | null = null;
        for (const q of link.pointIds) if (q !== pid && (!tip || dist(P(m, q), P(m, pid)) > dist(P(m, tip), P(m, pid)))) tip = q;
        if (tip) out.push({ linkId: link.id, pivotId: pid, tipId: tip, axis });
      }
    }
  }
  return out;
}

export function currentAngleDeg(m: Model, pivotId: ID, tipId: ID, axis: Vec3, ref: Vec3): number {
  const w = sub(P(m, tipId), P(m, pivotId));
  const n = normalize(axis);
  const u = normalize(sub(ref, scale(n, dot(ref, n))));
  const v = cross(n, u);
  return (Math.atan2(dot(w, v), dot(w, u)) * 180) / Math.PI;
}

/** Zero-angle reference for an axis: +X for the Z axis, +Y for the X axis, +X for the Y axis (first world axis not parallel to n). */
export function defaultAngleRef(n: Vec3): Vec3 {
  const candidates: Vec3[] = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ];
  for (const c of candidates) {
    if (Math.abs(dot(c, n)) < 0.9) return normalize(sub(c, scale(n, dot(c, n))));
  }
  return perpendicular(n);
}

export function addAngleDriver(m: Model, linkId: ID, pivotId: ID, tipId: ID, axis: Vec3): import('./types').Driver {
  const n = normalize(axis);
  const ref = defaultAngleRef(n);
  const d: import('./types').Driver = {
    id: newId(m, 'drv'),
    kind: 'angle',
    linkId,
    pivotId,
    tipId,
    axis: n,
    ref,
    value: currentAngleDeg(m, pivotId, tipId, n, ref),
  };
  m.drivers.push(d);
  return d;
}

export function addFoldDriver(m: Model, jointId: ID): import('./types').Driver | null {
  const j = m.joints[jointId];
  if (!j || j.type !== 'revolute') return null;
  const d: import('./types').Driver = { id: newId(m, 'drv'), kind: 'fold', jointId, value: 0 };
  m.drivers.push(d);
  return d;
}

export function addSlideDriver(m: Model, jointId: ID): import('./types').Driver | null {
  const j = m.joints[jointId];
  if (!j || (j.type !== 'prismatic' && j.type !== 'cylindrical' && j.type !== 'screw')) return null;
  const d: import('./types').Driver = { id: newId(m, 'drv'), kind: 'slide', jointId, value: 0 };
  m.drivers.push(d);
  return d;
}

// ---------------------------------------------------------------------------
// Serialization
// ---------------------------------------------------------------------------

export function serializeModel(m: Model): string {
  return JSON.stringify(m, null, 2);
}

export function parseModel(json: string): Model {
  const m = JSON.parse(json) as Model;
  if (m.version !== 1) throw new Error('Unsupported file version');
  m.settings = { ...DEFAULT_SETTINGS, ...m.settings };
  m.drivers ??= [];
  m.targets ??= [];
  for (const l of Object.values(m.links)) {
    l.helperIds ??= [];
    l.rigidity ??= [];
  }
  return m;
}
