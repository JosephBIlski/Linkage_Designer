/**
 * Crease patterns: crease-pattern-first construction of rigid-origami
 * mechanisms (docs/CONSTRUCTION_PLAN.md, Phase 3a).
 *
 * A pattern is a planar straight-line graph drawn on a construction plane:
 * vertices (3-D positions on the plane) and creases (two vertices, a class
 * mountain / valley / border / unassigned, an optional target fold angle).
 * The module keeps the graph a proper planar subdivision while it is edited
 * (a new vertex merges with a vertex within tolerance or splits the crease it
 * lies on; a new crease is split at every vertex and crossing it meets, and
 * the crossed creases are split too), finds its faces by half-edge tracing in
 * the plane's 2-D basis, checks the classical single-vertex conditions
 * (Kawasaki, Maekawa) on its interior vertices, and generates the mechanism:
 * one polygon link per bounded face and one edge–edge revolute crease per
 * interior crease, exactly coplanar and exactly joined (equal edge lengths,
 * merged end points), so that the generated model has zero constraint
 * violation and the pattern's vertex positions.
 *
 * Pure core: no DOM or three.js imports, no user-visible text (the UI turns
 * the issue records into messages).
 */
import { SECTOR_SUM_TOLERANCE_DEG } from './feasibility';
import { add, cross, dist, dot, len, normalize, perpendicular, scale, sub } from './geometry';
import { addJoint, addPolygonFromPoints, groundLink, newId, removeJoint, removeLink, setGround } from './model';
import type { CreaseClass, CreasePattern, Feature, ID, Joint, Model, PatternCrease, PatternVertex, Vec3 } from './types';

/** Relative tolerance of the pattern editor: distances below this fraction of the pattern's extent (at least 1) coincide. */
export const PATTERN_TOLERANCE = 1e-6;

/** Fallback base names used when the caller gives none (the UI passes names from its strings). */
export const DEFAULT_PATTERN_NAME = 'Pattern';
export const DEFAULT_PANEL_NAME = 'Panel';

// ---------------------------------------------------------------------------
// Plane basis
// ---------------------------------------------------------------------------

/** Orthonormal frame of the pattern's plane: positions are `origin + u·xDir + v·yDir`, normal = xDir × yDir. */
export interface PatternBasis {
  origin: Vec3;
  xDir: Vec3;
  yDir: Vec3;
  normal: Vec3;
}

export type Vec2 = [number, number];

/**
 * Right-handed 2-D frame of the construction plane a pattern lies on: the
 * plane's origin, its stored in-plane x direction (made exactly perpendicular
 * to the normal) and yDir = normal × xDir, so that counter-clockwise in (u, v)
 * is counter-clockwise seen from the side the plane normal points to. The
 * faces are traced, and the panels generated, counter-clockwise in this frame,
 * which makes the oriented face normal of every panel the plane normal; this
 * is the side from which the crease classes (fold.ts creaseMV) are judged. A
 * missing or non-plane construction id yields the world XY frame.
 */
export function patternBasis(m: Model, p: CreasePattern): PatternBasis {
  const c = m.construction[p.planeId];
  const normal: Vec3 = c?.kind === 'plane' && c.dir && len(c.dir) > 0.5 ? normalize(c.dir) : [0, 0, 1];
  const x0: Vec3 = c?.xDir ?? perpendicular(normal);
  let xDir = normalize(sub(x0, scale(normal, dot(x0, normal))));
  if (len(xDir) < 0.5) xDir = perpendicular(normal);
  const yDir = normalize(cross(normal, xDir));
  return { origin: c?.origin ? [c.origin[0], c.origin[1], c.origin[2]] : [0, 0, 0], xDir, yDir, normal };
}

/** Plane coordinates (u, v) of a world position (its component along the normal is dropped). */
export function to2d(basis: PatternBasis, pos: Vec3): Vec2 {
  const w = sub(pos, basis.origin);
  return [dot(w, basis.xDir), dot(w, basis.yDir)];
}

/** World position of plane coordinates (u, v). */
export function to3d(basis: PatternBasis, uv: Vec2): Vec3 {
  return add(basis.origin, add(scale(basis.xDir, uv[0]), scale(basis.yDir, uv[1])));
}

/** The nearest point of the plane to `pos`. */
export function projectToPlane(basis: PatternBasis, pos: Vec3): Vec3 {
  return to3d(basis, to2d(basis, pos));
}

// ---------------------------------------------------------------------------
// Tolerance
// ---------------------------------------------------------------------------

/** Diagonal of the bounding box of the pattern's vertices (0 for fewer than two vertices). */
export function patternExtent(p: CreasePattern): number {
  const vs = Object.values(p.vertices);
  if (vs.length < 2) return 0;
  const lo: Vec3 = [Infinity, Infinity, Infinity];
  const hi: Vec3 = [-Infinity, -Infinity, -Infinity];
  for (const v of vs) {
    for (let k = 0; k < 3; k++) {
      lo[k] = Math.min(lo[k], v.pos[k]);
      hi[k] = Math.max(hi[k], v.pos[k]);
    }
  }
  return dist(lo, hi);
}

/**
 * Coincidence tolerance of the editing operations: PATTERN_TOLERANCE times the
 * pattern's extent, at least PATTERN_TOLERANCE itself (a pattern of unit size
 * or smaller), so that a pattern drawn in millimetres and one drawn in metres
 * behave alike.
 */
export function patternTolerance(p: CreasePattern): number {
  return PATTERN_TOLERANCE * Math.max(1, patternExtent(p));
}

// ---------------------------------------------------------------------------
// Patterns
// ---------------------------------------------------------------------------

/** "<base> n" with the smallest n ≥ 1 that no pattern uses (so a name freed by a deletion is reused, never a duplicate). */
export function nextPatternName(m: Model, base: string = DEFAULT_PATTERN_NAME): string {
  const used = new Set(Object.values(m.patterns ?? {}).map((p) => p.name));
  let n = 1;
  while (used.has(`${base} ${n}`)) n++;
  return `${base} ${n}`;
}

/** Create an empty pattern on a construction plane. `name` defaults to nextPatternName ("Pattern n", the smallest unused n). */
export function addPattern(m: Model, planeId: ID, name?: string): CreasePattern {
  m.patterns ??= {};
  const id = newId(m, 'pattern');
  const p: CreasePattern = { id, name: name ?? nextPatternName(m), planeId, vertices: {}, creases: {} };
  m.patterns[id] = p;
  return p;
}

/** Remove a pattern together with the links and joints it generated. Returns false for an unknown id. */
export function removePattern(m: Model, patternId: ID): boolean {
  const p = m.patterns?.[patternId];
  if (!p) return false;
  removeGenerated(m, p);
  delete m.patterns![patternId];
  return true;
}

/** Remove the links and joints recorded in `p.generated` (missing ids are ignored) and clear the record. */
function removeGenerated(m: Model, p: CreasePattern): void {
  if (!p.generated) return;
  for (const id of p.generated.linkIds) removeLink(m, id);
  for (const id of p.generated.jointIds) removeJoint(m, id);
  delete p.generated;
}

// ---------------------------------------------------------------------------
// Queries
// ---------------------------------------------------------------------------

/** The pattern vertex nearest to `pos` within `tol`, or null. */
export function findPatternVertex(p: CreasePattern, pos: Vec3, tol: number = patternTolerance(p)): PatternVertex | null {
  let best: PatternVertex | null = null;
  let bestD = tol;
  for (const v of Object.values(p.vertices)) {
    const d = dist(v.pos, pos);
    if (d <= bestD) {
      bestD = d;
      best = v;
    }
  }
  return best;
}

/** A crease whose open segment passes within `tol` of a point (findPatternCrease). */
export interface CreaseHit {
  crease: PatternCrease;
  /** Parameter of the foot point along the crease, 0 at vertex a and 1 at vertex b (strictly inside). */
  t: number;
  /** The foot point on the crease. */
  point: Vec3;
}

/**
 * The crease nearest to `pos` whose segment passes within `tol` of it, the
 * foot point lying strictly inside the segment (more than `tol` from both end
 * points; a point near an end point is the vertex's business). Null when none.
 */
export function findPatternCrease(p: CreasePattern, pos: Vec3, tol: number = patternTolerance(p)): CreaseHit | null {
  let best: CreaseHit | null = null;
  let bestD = tol;
  for (const c of Object.values(p.creases)) {
    const a = p.vertices[c.a]?.pos;
    const b = p.vertices[c.b]?.pos;
    if (!a || !b) continue;
    const ab = sub(b, a);
    const l = len(ab);
    if (l <= tol) continue;
    const s = dot(sub(pos, a), ab) / l; // signed length along the crease
    if (s <= tol || s >= l - tol) continue;
    const foot = add(a, scale(ab, s / l));
    const d = dist(foot, pos);
    if (d <= bestD) {
      bestD = d;
      best = { crease: c, t: s / l, point: foot };
    }
  }
  return best;
}

/** Ids of the creases ending at a vertex. */
export function creasesAtVertex(p: CreasePattern, vertexId: ID): ID[] {
  return Object.values(p.creases)
    .filter((c) => c.a === vertexId || c.b === vertexId)
    .map((c) => c.id);
}

/** The crease joining two vertices (either direction), or null. */
export function creaseBetween(p: CreasePattern, aId: ID, bId: ID): PatternCrease | null {
  return Object.values(p.creases).find((c) => (c.a === aId && c.b === bId) || (c.a === bId && c.b === aId)) ?? null;
}

// ---------------------------------------------------------------------------
// Editing
// ---------------------------------------------------------------------------

export interface AddVertexResult {
  vertex: PatternVertex;
  /** True when `pos` coincided with an existing vertex, which is returned unchanged. */
  existing: boolean;
  /** When the point lay on a crease: the two pieces it was split into (the first keeps the crease's id). */
  splitCreaseIds: ID[];
}

/**
 * Add a vertex at `pos` (projected onto the pattern's plane). A position
 * within `tol` of an existing vertex returns that vertex; one lying on a
 * crease (within `tol` of its segment, strictly inside) splits the crease in
 * two at the new vertex, both pieces keeping its class and target; otherwise
 * the vertex is simply inserted. This keeps the invariant that no vertex lies
 * on the interior of a crease, which the face tracing relies on.
 */
export function addPatternVertex(m: Model, p: CreasePattern, pos: Vec3, tol: number = patternTolerance(p)): AddVertexResult {
  const basis = patternBasis(m, p);
  const q = projectToPlane(basis, pos);
  const hit = findPatternVertex(p, q, tol);
  if (hit) return { vertex: hit, existing: true, splitCreaseIds: [] };
  const on = findPatternCrease(p, q, tol);
  const v: PatternVertex = { id: newId(m, 'pv'), pos: on ? on.point : q };
  p.vertices[v.id] = v;
  return { vertex: v, existing: false, splitCreaseIds: on ? splitCrease(m, p, on.crease.id, v.id) : [] };
}

/** Split crease a–b at vertex v into a–v (keeping the id, class and target) and v–b (a new crease). Returns both ids. */
function splitCrease(m: Model, p: CreasePattern, creaseId: ID, vertexId: ID): ID[] {
  const c = p.creases[creaseId];
  const second: PatternCrease = { id: newId(m, 'pc'), a: vertexId, b: c.b, cls: c.cls };
  if (c.targetDeg !== undefined) second.targetDeg = c.targetDeg;
  c.b = vertexId;
  p.creases[second.id] = second;
  return [c.id, second.id];
}

export type AddCreaseReason = 'sameVertex' | 'missingVertex' | 'zeroLength' | 'duplicate' | 'overlap';

export type AddCreaseResult =
  /** The pieces of the new crease from a to b, and the vertices created at crossings. */
  | { ok: true; creaseIds: ID[]; vertexIds: ID[] }
  /** Nothing was changed; `creaseId` names the duplicate or overlapping crease. */
  | { ok: false; reason: AddCreaseReason; creaseId?: ID };

/**
 * Add a crease between two vertices. Refused, leaving the pattern unchanged,
 * when the vertices are the same ('sameVertex'), unknown ('missingVertex'),
 * closer than `tol` ('zeroLength'), already joined by a crease in either
 * direction ('duplicate'), or when an existing crease is collinear with the
 * new one and overlaps it by more than `tol` ('overlap': exact, partial and
 * contained overlaps alike; a collinear crease that only touches at an end
 * point is fine). Otherwise the new segment is intersected with every crease
 * in the plane's 2-D frame: at a proper crossing (both interiors) a vertex is
 * inserted at the intersection and the crossed crease is split there; an end
 * point of the new segment lying on the interior of a crease splits that
 * crease at the end point. Then every vertex on the open segment (crossing
 * vertices and pre-existing ones alike, such as the far end of a crease that
 * ends on the new segment) splits the new crease, whose pieces are created
 * from a to b with the given class. The result lists the pieces and the
 * vertices created.
 */
export function addPatternCrease(m: Model, p: CreasePattern, aId: ID, bId: ID, cls: CreaseClass = 'U', tol: number = patternTolerance(p)): AddCreaseResult {
  if (aId === bId) return { ok: false, reason: 'sameVertex' };
  const va = p.vertices[aId];
  const vb = p.vertices[bId];
  if (!va || !vb) return { ok: false, reason: 'missingVertex' };
  const dup = creaseBetween(p, aId, bId);
  if (dup) return { ok: false, reason: 'duplicate', creaseId: dup.id };
  const basis = patternBasis(m, p);
  const uv = (id: ID): Vec2 => to2d(basis, p.vertices[id].pos);
  const A = uv(aId);
  const B = uv(bId);
  const r: Vec2 = [B[0] - A[0], B[1] - A[1]];
  const L = Math.hypot(r[0], r[1]);
  if (L <= tol) return { ok: false, reason: 'zeroLength' };
  const dir: Vec2 = [r[0] / L, r[1] / L];
  /** Signed length along the new segment and signed distance from its line. */
  const along = (P: Vec2): number => (P[0] - A[0]) * dir[0] + (P[1] - A[1]) * dir[1];
  const across = (P: Vec2): number => (P[0] - A[0]) * dir[1] - (P[1] - A[1]) * dir[0];

  // pass 1: collinear overlaps are refused before anything is changed
  for (const c of Object.values(p.creases)) {
    if (!p.vertices[c.a] || !p.vertices[c.b]) continue;
    const C = uv(c.a);
    const D = uv(c.b);
    if (Math.abs(across(C)) > tol || Math.abs(across(D)) > tol) continue;
    const s0 = Math.min(along(C), along(D));
    const s1 = Math.max(along(C), along(D));
    if (Math.min(s1, L) - Math.max(s0, 0) > tol) return { ok: false, reason: 'overlap', creaseId: c.id };
  }

  // pass 2: crossings (the crease list is snapshotted; a split piece is collinear with its crease and meets the new
  // segment only at the vertex already inserted)
  const vertexIds: ID[] = [];
  for (const cid of Object.keys(p.creases)) {
    const c = p.creases[cid];
    if (!c || !p.vertices[c.a] || !p.vertices[c.b]) continue;
    const C = uv(c.a);
    const D = uv(c.b);
    const s: Vec2 = [D[0] - C[0], D[1] - C[1]];
    const Ls = Math.hypot(s[0], s[1]);
    if (Ls <= tol) continue;
    const denom = r[0] * s[1] - r[1] * s[0];
    if (Math.abs(denom) <= 1e-12 * L * Ls) continue; // parallel (collinear overlaps were refused above)
    const w: Vec2 = [C[0] - A[0], C[1] - A[1]];
    const t = ((w[0] * s[1] - w[1] * s[0]) / denom) * L; // along the new segment, in length units
    const u = ((w[0] * r[1] - w[1] * r[0]) / denom) * Ls; // along the existing crease
    if (t < -tol || t > L + tol || u < -tol || u > Ls + tol) continue;
    const tAtEnd = t <= tol || t >= L - tol;
    const uAtEnd = u <= tol || u >= Ls - tol;
    if (tAtEnd && uAtEnd) continue; // a shared (or coincident) end point
    if (tAtEnd) {
      // an end point of the new crease lies on the interior of c: split c there (the editing invariant was broken
      // by a vertex placed without addPatternVertex)
      splitCrease(m, p, c.id, t <= tol ? aId : bId);
      continue;
    }
    if (uAtEnd) continue; // c ends on the new segment: its end vertex splits the new crease in pass 3
    // proper crossing: a new vertex on both creases
    const X = to3d(basis, [A[0] + dir[0] * t, A[1] + dir[1] * t]);
    const existing = findPatternVertex(p, X, tol);
    if (existing) continue; // already a vertex there (it splits the new crease in pass 3)
    const v: PatternVertex = { id: newId(m, 'pv'), pos: X };
    p.vertices[v.id] = v;
    vertexIds.push(v.id);
    splitCrease(m, p, c.id, v.id);
  }

  // pass 3: every vertex on the open segment splits the new crease; create its pieces from a to b
  const splits: { id: ID; t: number }[] = [];
  for (const v of Object.values(p.vertices)) {
    if (v.id === aId || v.id === bId) continue;
    const P = uv(v.id);
    if (Math.abs(across(P)) > tol) continue;
    const t = along(P);
    if (t <= tol || t >= L - tol) continue;
    splits.push({ id: v.id, t });
  }
  splits.sort((x, y) => x.t - y.t);
  const chain = [aId, ...splits.map((s) => s.id), bId];
  const creaseIds: ID[] = [];
  for (let i = 0; i + 1 < chain.length; i++) {
    if (chain[i] === chain[i + 1] || creaseBetween(p, chain[i], chain[i + 1])) continue;
    const c: PatternCrease = { id: newId(m, 'pc'), a: chain[i], b: chain[i + 1], cls };
    p.creases[c.id] = c;
    creaseIds.push(c.id);
  }
  return { ok: true, creaseIds, vertexIds };
}

/** Remove a crease (its vertices stay, even when they become isolated). Returns false for an unknown id. */
export function removePatternCrease(m: Model, p: CreasePattern, creaseId: ID): boolean {
  if (!p.creases[creaseId]) return false;
  delete p.creases[creaseId];
  return true;
}

/** Remove a vertex together with every crease ending at it. Returns the ids of the removed creases (empty for an unknown vertex). */
export function removePatternVertex(m: Model, p: CreasePattern, vertexId: ID): ID[] {
  if (!p.vertices[vertexId]) return [];
  const removed = creasesAtVertex(p, vertexId);
  for (const id of removed) delete p.creases[id];
  delete p.vertices[vertexId];
  return removed;
}

/** Set the class of a crease. Returns false for an unknown id. */
export function setCreaseClass(m: Model, p: CreasePattern, creaseId: ID, cls: CreaseClass): boolean {
  const c = p.creases[creaseId];
  if (!c) return false;
  c.cls = cls;
  return true;
}

/** Set (or with undefined clear) the target fold angle of a crease (degrees, clamped to 0–180). Returns false for an unknown id. */
export function setCreaseTarget(m: Model, p: CreasePattern, creaseId: ID, targetDeg: number | undefined): boolean {
  const c = p.creases[creaseId];
  if (!c) return false;
  if (targetDeg === undefined || Number.isNaN(targetDeg)) delete c.targetDeg;
  else c.targetDeg = Math.max(0, Math.min(180, targetDeg));
  return true;
}

// ---------------------------------------------------------------------------
// Face detection
// ---------------------------------------------------------------------------

export interface PatternFaces {
  /** Bounded faces as vertex cycles, counter-clockwise in the plane's frame (patternBasis); each boundary vertex once. */
  faces: ID[][];
  /** The outer boundary (clockwise) of the component enclosing the largest area; empty when there is no bounded face. */
  outer: ID[];
  /** Creases with exactly one generated face (a face with a hole counts as none, except for its own outline). */
  border: ID[];
  /** Creases with two different generated faces. */
  interior: ID[];
  /** Creases with the same face on both sides (tree edges), or with no bounded face at all (slivers). */
  dangling: ID[];
  /** Vertices with no crease. */
  isolated: ID[];
  /** Faces with a hole or a pinched boundary (see PatternHole): counted in `faces`, generated as no panel. */
  holes: PatternHole[];
}

/**
 * A bounded face that cannot become a panel: a closed outline of another
 * component lies inside it without touching its boundary (a hole: the outer
 * polygon would cover the panels generated inside it), or its boundary passes
 * through a vertex twice (an inner outline joined to the boundary by a single
 * crease, so the face is a ring pinched at that crease, not a simple polygon).
 */
export interface PatternHole {
  /** Index into PatternFaces.faces of the face that is not generated. */
  face: number;
  /** The creases to blame: the enclosed outline's, or the crease the boundary runs along twice. */
  creaseIds: ID[];
}

interface HalfEdge {
  index: number;
  creaseId: ID;
  from: ID;
  to: ID;
  angle: number;
  twin: number;
  next: number;
  face: number;
}

/** Full result of the half-edge tracing, shared by patternFaces, validatePattern and generatePanels. */
interface Traced {
  basis: PatternBasis;
  /** Bounded faces: vertex cycles (spurs removed) and the raw half-edge cycle. */
  bounded: { cycle: ID[]; halfEdges: number[] }[];
  /** For every crease: the bounded-face index on the side of each half-edge (a→b, b→a), −1 for the outer or a degenerate face. */
  faceOf: Map<ID, [number, number]>;
  halfEdges: HalfEdge[];
  outer: ID[];
  degree: Map<ID, number>;
  border: ID[];
  interior: ID[];
  dangling: ID[];
  isolated: ID[];
  holes: PatternHole[];
}

/** Even–odd (ray-casting) test: is `q` strictly inside the polygon `poly`? A point on an edge counts as neither. */
function insidePolygon(q: Vec2, poly: Vec2[]): boolean {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const a = poly[i];
    const b = poly[j];
    if (a[1] > q[1] !== b[1] > q[1]) {
      const x = a[0] + ((q[1] - a[1]) * (b[0] - a[0])) / (b[1] - a[1]);
      if (q[0] < x) inside = !inside;
    }
  }
  return inside;
}

/** The tracing behind patternFaces (the half-edge convention is documented there); shared with validatePattern and generatePanels. */
function trace(m: Model, p: CreasePattern, tol: number = patternTolerance(p)): Traced {
  const basis = patternBasis(m, p);
  const uv = new Map<ID, Vec2>();
  for (const v of Object.values(p.vertices)) uv.set(v.id, to2d(basis, v.pos));
  const creases = Object.values(p.creases).filter((c) => uv.has(c.a) && uv.has(c.b) && c.a !== c.b);

  // half-edges
  const halfEdges: HalfEdge[] = [];
  const out = new Map<ID, number[]>();
  const degree = new Map<ID, number>();
  for (const id of uv.keys()) {
    out.set(id, []);
    degree.set(id, 0);
  }
  const angleOf = (from: ID, to: ID): number => {
    const a = uv.get(from)!;
    const b = uv.get(to)!;
    return Math.atan2(b[1] - a[1], b[0] - a[0]);
  };
  for (const c of creases) {
    const i = halfEdges.length;
    halfEdges.push({ index: i, creaseId: c.id, from: c.a, to: c.b, angle: angleOf(c.a, c.b), twin: i + 1, next: -1, face: -1 });
    halfEdges.push({ index: i + 1, creaseId: c.id, from: c.b, to: c.a, angle: angleOf(c.b, c.a), twin: i, next: -1, face: -1 });
    out.get(c.a)!.push(i);
    out.get(c.b)!.push(i + 1);
    degree.set(c.a, degree.get(c.a)! + 1);
    degree.set(c.b, degree.get(c.b)! + 1);
  }
  // next pointers: the half-edge before the twin in the counter-clockwise order at the head vertex
  for (const list of out.values()) {
    list.sort((x, y) => halfEdges[x].angle - halfEdges[y].angle);
  }
  for (const h of halfEdges) {
    const list = out.get(h.to)!;
    const k = list.indexOf(h.twin);
    h.next = list[(k - 1 + list.length) % list.length];
  }
  // trace faces
  const faces: { halfEdges: number[]; area: number; first: ID }[] = [];
  for (const h of halfEdges) {
    if (h.face >= 0) continue;
    const cycle: number[] = [];
    let k = h.index;
    while (halfEdges[k].face < 0) {
      halfEdges[k].face = faces.length;
      cycle.push(k);
      k = halfEdges[k].next;
    }
    let area = 0;
    for (const e of cycle) {
      const a = uv.get(halfEdges[e].from)!;
      const b = uv.get(halfEdges[e].to)!;
      area += a[0] * b[1] - b[0] * a[1];
    }
    faces.push({ halfEdges: cycle, area: area / 2, first: h.from });
  }
  // components (union-find over crease end points) and their outer faces
  const parent = new Map<ID, ID>();
  const find = (x: ID): ID => {
    let r = x;
    while (parent.get(r) !== undefined && parent.get(r) !== r) r = parent.get(r)!;
    return r;
  };
  for (const c of creases) {
    if (!parent.has(c.a)) parent.set(c.a, c.a);
    if (!parent.has(c.b)) parent.set(c.b, c.b);
    const ra = find(c.a);
    const rb = find(c.b);
    if (ra !== rb) parent.set(rb, ra);
  }
  const outerOf = new Map<ID, number>();
  faces.forEach((f, i) => {
    const root = find(f.first);
    const cur = outerOf.get(root);
    if (cur === undefined || f.area < faces[cur].area) outerOf.set(root, i);
  });
  const outerFaces = new Set(outerOf.values());
  const areaTol = 0.5 * tol * Math.max(1, patternExtent(p));
  const boundedIndex = new Map<number, number>();
  const bounded: Traced['bounded'] = [];
  const boundedRoot: ID[] = []; // the component of each bounded face
  faces.forEach((f, i) => {
    if (outerFaces.has(i) || f.area <= areaTol) return;
    const cycle = removeSpurs(f.halfEdges.map((e) => halfEdges[e].from));
    if (cycle.length < 3) return;
    boundedIndex.set(i, bounded.length);
    bounded.push({ cycle, halfEdges: f.halfEdges });
    boundedRoot.push(find(f.first));
  });
  // holes: a component with a bounded face (a closed outline) whose vertex lies strictly inside a bounded face of
  // another component is enclosed by that face (components never cross, since addPatternCrease splits crossings, so
  // one vertex decides for the whole outline); a bounded face whose spur-free cycle repeats a vertex is a ring
  // pinched at the crease its boundary runs along twice. Neither face is generated (PatternHole).
  const holes: PatternHole[] = [];
  const uniq = (ids: ID[]): ID[] => [...new Set(ids)];
  for (const [root, outerIndex] of outerOf) {
    if (!boundedRoot.includes(root)) continue;
    const q = uv.get(faces[outerIndex].first)!;
    const outline = uniq(faces[outerIndex].halfEdges.map((e) => halfEdges[e].creaseId));
    bounded.forEach((b, k) => {
      if (boundedRoot[k] !== root && insidePolygon(q, b.cycle.map((id) => uv.get(id)!))) holes.push({ face: k, creaseIds: outline });
    });
  }
  bounded.forEach((b, k) => {
    if (new Set(b.cycle).size === b.cycle.length || holes.some((h) => h.face === k)) return;
    const seen = new Set<ID>();
    const twice = uniq(b.halfEdges.map((e) => halfEdges[e].creaseId).filter((id) => (seen.has(id) ? true : (seen.add(id), false))));
    holes.push({ face: k, creaseIds: twice.length ? twice : [halfEdges[b.halfEdges[0]].creaseId] });
  });
  // the outer boundary: the outer face enclosing the largest area
  let outer: ID[] = [];
  let outerArea = 0;
  for (const i of outerFaces) {
    if (faces[i].area < outerArea) {
      outerArea = faces[i].area;
      outer = removeSpurs(faces[i].halfEdges.map((e) => halfEdges[e].from));
    }
  }
  if (outer.length < 3) outer = [];
  // crease classification: a holed face generates no panel, so a crease is interior only between two generated faces;
  // one with a bounded face on at least one side (generated or not) is a border, one with none (or the same face on
  // both sides) is dangling
  const holed = new Set(holes.map((h) => h.face));
  const generated = (k: number): boolean => k >= 0 && !holed.has(k);
  const faceOf = new Map<ID, [number, number]>();
  const border: ID[] = [];
  const interior: ID[] = [];
  const dangling: ID[] = [];
  for (let i = 0; i < halfEdges.length; i += 2) {
    const h0 = halfEdges[i];
    const h1 = halfEdges[i + 1];
    const f0 = boundedIndex.get(h0.face) ?? -1;
    const f1 = boundedIndex.get(h1.face) ?? -1;
    faceOf.set(h0.creaseId, [f0, f1]);
    if (h0.face === h1.face || (f0 < 0 && f1 < 0)) dangling.push(h0.creaseId);
    else if (generated(f0) && generated(f1)) interior.push(h0.creaseId);
    else border.push(h0.creaseId);
  }
  const isolated = [...degree.entries()].filter(([, d]) => d === 0).map(([id]) => id);
  return { basis, bounded, faceOf, halfEdges, outer, degree, border, interior, dangling, isolated, holes };
}

/** Collapse every x, y, x excursion of a vertex cycle to x until none is left (the spurs of dangling creases). */
function removeSpurs(cycle: ID[]): ID[] {
  const out = [...cycle];
  let changed = true;
  while (changed) {
    changed = false;
    const n = out.length;
    if (n === 2) {
      out.pop();
      break;
    }
    if (n < 3) break;
    for (let i = 0; i < n; i++) {
      const j = (i + 2) % n;
      if (out[i] === out[j]) {
        const k = (i + 1) % n;
        for (const r of [k, j].sort((x, y) => y - x)) out.splice(r, 1);
        changed = true;
        break;
      }
    }
  }
  return out;
}

/**
 * Faces and crease classification of a pattern by half-edge tracing in its
 * 2-D frame. Every crease a–b gives two half-edges a→b and b→a; at each
 * vertex the outgoing half-edges are sorted counter-clockwise by angle. The
 * walk from a half-edge h = x→y continues at y with the half-edge that comes
 * just before the reverse y→x in that counter-clockwise order (equivalently:
 * just after it clockwise), which keeps the face on the left, so a bounded
 * face is traversed counter-clockwise (positive shoelace area) and the outer
 * boundary of each connected component clockwise (negative area):
 *
 *     D ─────────── C     Outgoing half-edges at B sorted by angle: B→C (90°),
 *     │ ╲    f2     │     B→A (180°). From A→B the walk continues with the one
 *     │   ╲         │     just before the reverse B→A, i.e. B→C; at C the one
 *     │     ╲       │     before C→B (−90°) is C→A (−135°). A→B→C→A is f1,
 *     │  f1   ╲     │     counter-clockwise; B→C→D→A→B is f2; the outer walk
 *     │         ╲   │     A→D→C→B→A is clockwise. A vertex of degree one sends
 *     A ─────────── B     the walk straight back (x→y, y→x): a dangling crease
 *                         has the same face on both sides and adds no area.
 *
 * A bounded face is a traced cycle of positive area beyond the tolerance;
 * the face of least signed area in each component is its outer face. The
 * vertex cycles reported drop the spurs of dangling creases (… x, y, x … →
 * … x …), so a face's cycle lists its boundary vertices only. A face that
 * encloses another component's outline, or whose cycle repeats a vertex, is
 * listed in `holes` (PatternHole) and generates no panel.
 */
export function patternFaces(m: Model, p: CreasePattern, tol: number = patternTolerance(p)): PatternFaces {
  const t = trace(m, p, tol);
  return { faces: t.bounded.map((f) => f.cycle), outer: t.outer, border: t.border, interior: t.interior, dangling: t.dangling, isolated: t.isolated, holes: t.holes };
}

// ---------------------------------------------------------------------------
// Validation
// ---------------------------------------------------------------------------

/** The creases around a vertex in counter-clockwise order and the sector angles between consecutive ones. */
export interface VertexSectors {
  /** Crease ids sorted counter-clockwise by direction from the vertex. */
  creaseIds: ID[];
  /** sectorDeg[i] is the angle (degrees) from creaseIds[i] to creaseIds[i + 1] (cyclic); they sum to 360. */
  sectorDeg: number[];
}

/**
 * Creases at a vertex in counter-clockwise order (in the plane's frame) with
 * the sector angles between consecutive ones; a vertex of degree one has one
 * sector of 360°, an isolated vertex none.
 */
export function patternVertexSectors(m: Model, p: CreasePattern, vertexId: ID): VertexSectors {
  const basis = patternBasis(m, p);
  const v = p.vertices[vertexId];
  if (!v) return { creaseIds: [], sectorDeg: [] };
  const o = to2d(basis, v.pos);
  const dirs: { id: ID; angle: number }[] = [];
  for (const c of Object.values(p.creases)) {
    const otherId = c.a === vertexId ? c.b : c.b === vertexId ? c.a : null;
    if (otherId === null || !p.vertices[otherId]) continue;
    const q = to2d(basis, p.vertices[otherId].pos);
    dirs.push({ id: c.id, angle: Math.atan2(q[1] - o[1], q[0] - o[0]) });
  }
  dirs.sort((x, y) => x.angle - y.angle);
  const sectorDeg = dirs.map((d, i) => {
    const next = dirs[(i + 1) % dirs.length];
    let a = next.angle - d.angle;
    if (dirs.length === 1) a = 2 * Math.PI;
    else if (a <= 0) a += 2 * Math.PI;
    return (a * 180) / Math.PI;
  });
  return { creaseIds: dirs.map((d) => d.id), sectorDeg };
}

export type PatternIssueKind = 'oddDegree' | 'kawasaki' | 'maekawa' | 'dangling' | 'unassigned' | 'borderInside' | 'hole' | 'isolated' | 'noFaces';

/**
 * One problem of a pattern (validatePattern). The UI turns these into
 * messages; the fields present depend on the kind:
 *  - oddDegree: vertexId, creaseIds (the creases at the vertex);
 *  - kawasaki: vertexId, creaseIds, valueDeg (the alternating sector sum);
 *  - maekawa: vertexId, creaseIds, mountains, valleys;
 *  - dangling: creaseIds (one crease);
 *  - unassigned: creaseIds (one interior crease of class 'U');
 *  - borderInside: creaseIds (one interior crease of class 'B');
 *  - hole: creaseIds (the enclosed outline's creases, or the crease a pinched boundary runs along twice; PatternHole);
 *  - isolated: vertexId;
 *  - noFaces: nothing else.
 */
export interface PatternIssue {
  kind: PatternIssueKind;
  vertexId?: ID;
  creaseIds?: ID[];
  valueDeg?: number;
  mountains?: number;
  valleys?: number;
}

/**
 * Interior vertices of a pattern: vertices with at least one crease whose
 * creases are all interior (two bounded faces each), so that the panels
 * around them close a ring. A vertex on the outer boundary has border
 * creases, one with a dangling crease is skipped as well.
 */
export function patternInteriorVertices(m: Model, p: CreasePattern, tol: number = patternTolerance(p)): ID[] {
  return interiorVertices(p, trace(m, p, tol));
}

function interiorVertices(p: CreasePattern, t: Traced): ID[] {
  const interior = new Set(t.interior);
  const out: ID[] = [];
  for (const id of Object.keys(p.vertices)) {
    const at = creasesAtVertex(p, id);
    if (at.length > 0 && at.every((c) => interior.has(c))) out.push(id);
  }
  return out;
}

/**
 * The classical single-vertex conditions on every interior vertex and the
 * structural problems of the pattern, as records (no text):
 *  - oddDegree: an interior vertex with an odd number of creases can never
 *    fold flat (the paper reverses at every crease and must come back to
 *    itself after one turn);
 *  - kawasaki: for an even degree, the alternating sum α₁ − α₂ + α₃ − … of
 *    the sector angles in angular order must vanish within `tolDeg`;
 *  - maekawa: when every crease at the vertex is assigned M or V, the counts
 *    must differ by exactly two;
 *  - dangling: a crease with the same face on both sides (or no panel at
 *    all) would generate no hinge;
 *  - unassigned: an interior crease of class 'U' generates a crease without
 *    a mountain / valley preference;
 *  - borderInside: an interior crease of class 'B' (a border drawn between
 *    two faces) generates such a crease as well, since a border carries no
 *    fold direction; the vertex conditions treat it as unassigned;
 *  - hole: a face enclosing another outline, or pinched at one crease, is
 *    not generated (PatternHole);
 *  - isolated: a vertex without creases;
 *  - noFaces: the pattern encloses no bounded face, so nothing is generated.
 * Developability needs no check: a drawn pattern is flat by construction.
 */
export function validatePattern(m: Model, p: CreasePattern, tolDeg: number = SECTOR_SUM_TOLERANCE_DEG, tol: number = patternTolerance(p)): PatternIssue[] {
  const t = trace(m, p, tol);
  const issues: PatternIssue[] = [];
  for (const vid of interiorVertices(p, t)) {
    const { creaseIds, sectorDeg } = patternVertexSectors(m, p, vid);
    if (creaseIds.length % 2 !== 0) {
      issues.push({ kind: 'oddDegree', vertexId: vid, creaseIds });
    } else {
      const alt = sectorDeg.reduce((acc, a, i) => acc + (i % 2 === 0 ? a : -a), 0);
      if (Math.abs(alt) > tolDeg) issues.push({ kind: 'kawasaki', vertexId: vid, creaseIds, valueDeg: alt });
    }
    const classes = creaseIds.map((id) => p.creases[id].cls);
    if (classes.every((c) => c === 'M' || c === 'V')) {
      const mountains = classes.filter((c) => c === 'M').length;
      const valleys = classes.length - mountains;
      if (Math.abs(mountains - valleys) !== 2) issues.push({ kind: 'maekawa', vertexId: vid, creaseIds, mountains, valleys });
    }
  }
  for (const id of t.dangling) issues.push({ kind: 'dangling', creaseIds: [id] });
  for (const id of t.interior) {
    const cls = p.creases[id].cls;
    if (cls === 'U') issues.push({ kind: 'unassigned', creaseIds: [id] });
    else if (cls === 'B') issues.push({ kind: 'borderInside', creaseIds: [id] });
  }
  for (const h of t.holes) issues.push({ kind: 'hole', creaseIds: h.creaseIds });
  for (const id of t.isolated) issues.push({ kind: 'isolated', vertexId: id });
  if (t.bounded.length === 0) issues.push({ kind: 'noFaces' });
  return issues;
}

// ---------------------------------------------------------------------------
// Generation
// ---------------------------------------------------------------------------

export interface GenerateOptions {
  /** Ground the first generated panel when the model has no ground link (default true). */
  ground?: boolean;
  /** Base of the panel names, "<base> n" with the smallest n not in use (default DEFAULT_PANEL_NAME). */
  panelName?: string;
}

export interface GenerateResult {
  linkIds: ID[];
  jointIds: ID[];
  /** The pattern's validation issues (validatePattern) at the time of generation. */
  warnings: PatternIssue[];
}

/** "<base> n" with the smallest n ≥ 1 that no link uses. */
function nextLinkName(m: Model, base: string): string {
  const used = new Set(Object.values(m.links).map((l) => l.name));
  let n = 1;
  while (used.has(`${base} ${n}`)) n++;
  return `${base} ${n}`;
}

/**
 * Generate the mechanism of a pattern: the links and joints of a previous
 * generation are removed first (removeLink takes their joints with it), then
 * one polygon link is created per bounded face from the face's vertex
 * positions (counter-clockwise in the plane's frame, no sketch-plane
 * constraint, so the panels can fold), and one edge–edge revolute per
 * interior crease between the edges of its two panels that correspond to the
 * crease's vertices. Both edges have exactly the same end points, so addJoint
 * merges them pairwise (a crease in the sense of fold.ts: two merged pairs)
 * and no panel has to adapt; the generated pose is the pattern's and its
 * constraint violation is zero. The crease's class (M or V only) and target
 * are stored in joint.fold for the Fold command and the crease display. When
 * the model has no ground link and `opts.ground` is not false, the first
 * panel becomes ground. The pattern records what was generated
 * (`p.generated`, with each link's face vertices in point order). Returns
 * the ids and the pattern's validation issues as warnings; a pattern without
 * bounded faces generates nothing. A face with a hole or a pinched boundary
 * (PatternHole, reported among the warnings) is skipped — its panel would
 * cover the panels inside it, the overlap the Panel tool refuses as well — and
 * so is a crease between it and a neighbour.
 */
export function generatePanels(m: Model, p: CreasePattern, opts: GenerateOptions = {}): GenerateResult {
  removeGenerated(m, p);
  const t = trace(m, p);
  const base = opts.panelName ?? DEFAULT_PANEL_NAME;
  const linkIds: ID[] = [];
  const faces: Record<ID, ID[]> = {};
  const holed = new Set(t.holes.map((h) => h.face));
  const linkOfFace: (ID | null)[] = t.bounded.map((f, k) => {
    if (holed.has(k)) return null;
    const pts = f.cycle.map((id) => [...p.vertices[id].pos] as Vec3);
    const link = addPolygonFromPoints(m, pts, { name: nextLinkName(m, base), onPlaneId: null });
    linkIds.push(link.id);
    faces[link.id] = [...f.cycle];
    return link.id;
  });
  const jointIds: ID[] = [];
  const edgeOf = (linkId: ID | null, c: PatternCrease): Feature | null => {
    const link = linkId ? m.links[linkId] : undefined;
    if (!link || !linkId) return null;
    const ia = faces[linkId].indexOf(c.a);
    const ib = faces[linkId].indexOf(c.b);
    if (ia < 0 || ib < 0) return null;
    return { linkId, kind: 'edge', pointIds: [link.pointIds[ia], link.pointIds[ib]] };
  };
  for (const cid of t.interior) {
    const c = p.creases[cid];
    const [fa, fb] = t.faceOf.get(cid) ?? [-1, -1];
    if (fa < 0 || fb < 0 || fa === fb) continue;
    const ea = edgeOf(linkOfFace[fa], c);
    const eb = edgeOf(linkOfFace[fb], c);
    if (!ea || !eb) continue;
    const joint: Joint | null = addJoint(m, 'revolute', ea, eb);
    if (!joint) continue;
    const fold: NonNullable<Joint['fold']> = {};
    if (c.cls === 'M' || c.cls === 'V') fold.mv = c.cls;
    if (c.targetDeg !== undefined) fold.target = c.targetDeg;
    joint.fold = fold;
    jointIds.push(joint.id);
  }
  if (opts.ground !== false && linkIds.length > 0 && !groundLink(m)) setGround(m, linkIds[0]);
  p.generated = { linkIds, jointIds, faces };
  return { linkIds, jointIds, warnings: validatePattern(m, p) };
}

/** The pattern whose generation produced joint `jointId` (recorded in CreasePattern.generated), if any. */
export function patternOfJoint(m: Model, jointId: ID): CreasePattern | null {
  return Object.values(m.patterns ?? {}).find((p) => p.generated?.jointIds.includes(jointId)) ?? null;
}

// ---------------------------------------------------------------------------
// Miura-ori
// ---------------------------------------------------------------------------

/**
 * The classic Miura-ori tessellation with `rows` × `cols` interior vertices,
 * i.e. (rows + 1) × (cols + 1) parallelogram cells of width `cellW` (along
 * the plane's x direction) and height `cellH`. The horizontal creases are
 * straight lines; the "vertical" creases zigzag: every other row of vertices
 * is shifted by d = cellH / tan α along x, so each tilted crease makes the
 * angle α with the horizontal and every interior vertex has the sector
 * angles (α, π − α, π − α, α) counter-clockwise from its right-hand crease:
 * the horizontal pair is collinear, the tilted pair is bent (the degree-4
 * vertex of examples.ts origamiMiuraVertex), Kawasaki holds since
 * α − (π − α) + (π − α) − α = 0.
 *
 * Mountain / valley assignment (the rigid folding of the degree-4 vertex,
 * docs/SPEC.md §17.2: the two bent creases share a class, the two collinear
 * creases differ, and the odd one is the collinear crease between the two
 * equal sectors on the side the bent creases tilt toward): each zigzag line
 * has one class, mountains and valleys alternating from one zigzag to the
 * next, and each horizontal line changes class at every vertex, the segment
 * on the tilt side of a vertex taking the class opposite to its zigzag. So
 * every interior vertex has three creases of one class and one of the other
 * (Maekawa), and the whole sheet folds on one branch. The outer edges are
 * borders.
 *
 *        ╲      ╱      ╲          row 2 (shifted by d)
 *     ────┼────┼────┼────┼────    horizontal line: classes alternate along it
 *        ╱      ╲      ╱          row 1
 *     ────┼────┼────┼────┼────
 *        ╲      ╱      ╲          row 0: zigzag i has one class, M V M V …
 *
 * `rows` and `cols` count the interior vertices (1 × 1 is the Miura unit of
 * four cells, 2 × 2 the 9-cell sheet). The pattern is added to the model
 * (addPattern) and returned.
 */
export function miuraPattern(m: Model, planeId: ID, rows: number, cols: number, alphaDeg = 60, cellW = 2, cellH = 2, name?: string): CreasePattern {
  const p = addPattern(m, planeId, name);
  const basis = patternBasis(m, p);
  const nx = Math.max(1, Math.floor(cols)) + 1; // cells along x
  const ny = Math.max(1, Math.floor(rows)) + 1; // cells along y
  const d = cellH / Math.tan((alphaDeg * Math.PI) / 180);
  const shift = (j: number): number => (j % 2 === 1 ? d : 0);
  const ids: ID[][] = [];
  for (let j = 0; j <= ny; j++) {
    ids.push([]);
    for (let i = 0; i <= nx; i++) {
      const v: PatternVertex = { id: newId(m, 'pv'), pos: to3d(basis, [i * cellW + shift(j), j * cellH]) };
      p.vertices[v.id] = v;
      ids[j].push(v.id);
    }
  }
  const put = (a: ID, b: ID, cls: CreaseClass): void => {
    const c: PatternCrease = { id: newId(m, 'pc'), a, b, cls };
    p.creases[c.id] = c;
  };
  const zig = (i: number): CreaseClass => (i % 2 === 0 ? 'M' : 'V');
  const other = (c: CreaseClass): CreaseClass => (c === 'M' ? 'V' : 'M');
  // horizontal creases of row j, segment i → i + 1
  for (let j = 0; j <= ny; j++) {
    const boundary = j === 0 || j === ny;
    const tiltRight = shift(j + 1) - shift(j) > 0; // the bent creases of row j lean toward +x (sign of d included)
    for (let i = 0; i < nx; i++) {
      // the segment is on the tilt side of vertex i (tiltRight) or of vertex i + 1 (else): the odd crease of that vertex
      const cls: CreaseClass = boundary ? 'B' : tiltRight ? other(zig(i)) : other(zig(i + 1));
      put(ids[j][i], ids[j][i + 1], cls);
    }
  }
  // zigzag creases of column i, segment j → j + 1
  for (let i = 0; i <= nx; i++) {
    const boundary = i === 0 || i === nx;
    for (let j = 0; j < ny; j++) put(ids[j][i], ids[j + 1][i], boundary ? 'B' : zig(i));
  }
  return p;
}
