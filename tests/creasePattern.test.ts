import { describe, expect, it } from 'vitest';
import { addConstructionPlane, addOffsetPlane, bodyPlaneJoint, cloneModel, createModel, groundLink, parseModel, planeCarriesPattern, removeConstruction, serializeModel } from '../src/core/model';
import {
  addPattern,
  addPatternCrease,
  addPatternVertex,
  creasesAtVertex,
  findPatternCrease,
  findPatternVertex,
  generatePanels,
  miuraPattern,
  nextPatternName,
  patternBasis,
  patternFaces,
  patternInteriorVertices,
  patternTolerance,
  patternVertexSectors,
  removePattern,
  removePatternCrease,
  removePatternVertex,
  setCreaseClass,
  setCreaseTarget,
  to2d,
  to3d,
  validatePattern,
  type PatternIssue,
} from '../src/core/creasePattern';
import { currentViolation, computeMobility } from '../src/core/kinematics';
import { validateCreasePattern } from '../src/core/validate';
import { findCreaseLoops, isCrease } from '../src/core/fold';
import { cross, dist, dot, newellNormal, normalize } from '../src/core/geometry';
import { isConstructionRef, type CreaseClass, type CreasePattern, type ID, type Model, type Vec3 } from '../src/core/types';

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

function fresh(planeId = 'plane_top'): { m: Model; p: CreasePattern } {
  const m = createModel();
  const p = addPattern(m, planeId, 'Test pattern');
  return { m, p };
}

/** Add (or find) a vertex and return its id. */
const V = (m: Model, p: CreasePattern, x: number, y: number, z = 0): ID => addPatternVertex(m, p, [x, y, z]).vertex.id;

/** Add a crease that must succeed; returns the ids of its pieces. */
function C(m: Model, p: CreasePattern, a: ID, b: ID, cls: CreaseClass = 'U'): ID[] {
  const r = addPatternCrease(m, p, a, b, cls);
  expect(r.ok).toBe(true);
  return r.ok ? r.creaseIds : [];
}

/** Square (0,0)–(s,0)–(s,s)–(0,s) of border creases; returns its corner ids [a, b, c, d]. */
function square(m: Model, p: CreasePattern, s = 2, origin: [number, number] = [0, 0]): ID[] {
  const [ox, oy] = origin;
  const a = V(m, p, ox, oy);
  const b = V(m, p, ox + s, oy);
  const c = V(m, p, ox + s, oy + s);
  const d = V(m, p, ox, oy + s);
  C(m, p, a, b, 'B');
  C(m, p, b, c, 'B');
  C(m, p, c, d, 'B');
  C(m, p, d, a, 'B');
  return [a, b, c, d];
}

/** Shoelace area of a vertex cycle in the pattern's 2-D frame (positive = counter-clockwise). */
function area2d(m: Model, p: CreasePattern, cycle: ID[]): number {
  const basis = patternBasis(m, p);
  const pts = cycle.map((id) => to2d(basis, p.vertices[id].pos));
  let s = 0;
  for (let i = 0; i < pts.length; i++) {
    const a = pts[i];
    const b = pts[(i + 1) % pts.length];
    s += a[0] * b[1] - b[0] * a[1];
  }
  return s / 2;
}

const kinds = (issues: PatternIssue[], kind: PatternIssue['kind']): PatternIssue[] => issues.filter((i) => i.kind === kind);
const count = (p: CreasePattern) => ({ vertices: Object.keys(p.vertices).length, creases: Object.keys(p.creases).length });
const sorted = (xs: number[]): number[] => [...xs].sort((a, b) => a - b);

// ---------------------------------------------------------------------------
// Plane basis
// ---------------------------------------------------------------------------

describe('patternBasis', () => {
  it('is the right-handed frame of the plane and maps 2-D ↔ 3-D exactly', () => {
    const { m, p } = fresh('plane_top');
    const b = patternBasis(m, p);
    expect(b.origin).toEqual([0, 0, 0]);
    expect(b.xDir).toEqual([1, 0, 0]);
    expect(b.yDir).toEqual([0, 1, 0]);
    expect(b.normal).toEqual([0, 0, 1]);
    expect(dist(cross(b.xDir, b.yDir), b.normal)).toBeLessThan(1e-12);
    expect(to2d(b, [3, -1, 7])).toEqual([3, -1]);
    expect(to3d(b, [3, -1])).toEqual([3, -1, 0]);
    // FRONT (normal −Y, toward the Front view's camera; x = +X): y direction of the frame is normal × xDir = +Z, up on screen
    const f = fresh('plane_front');
    const bf = patternBasis(f.m, f.p);
    expect(bf.normal).toEqual([0, -1, 0]);
    expect(bf.xDir).toEqual([1, 0, 0]);
    expect(dist(bf.yDir, [0, 0, 1])).toBeLessThan(1e-12);
    expect(dist(cross(bf.xDir, bf.yDir), bf.normal)).toBeLessThan(1e-12);
    // a tilted user plane with an offset origin
    const m2 = createModel();
    const plane = addConstructionPlane(m2, [1, 2, 3], normalize([1, 1, 1]), 'TILT');
    const p2 = addPattern(m2, plane.id);
    const bt = patternBasis(m2, p2);
    expect(Math.abs(dot(bt.xDir, bt.normal))).toBeLessThan(1e-12);
    expect(Math.abs(dot(bt.yDir, bt.normal))).toBeLessThan(1e-12);
    expect(dist(cross(bt.xDir, bt.yDir), bt.normal)).toBeLessThan(1e-12);
    const q = to3d(bt, [0.7, -2.5]);
    expect(dist(to3d(bt, to2d(bt, q)), q)).toBeLessThan(1e-12);
    expect(Math.abs(dot(cross([1, 0, 0], [0, 1, 0]), [0, 0, 1]) - 1)).toBeLessThan(1e-12);
  });

  it('addPatternVertex projects the position onto the plane', () => {
    const { m, p } = fresh('plane_front');
    const r = addPatternVertex(m, p, [1, 5, 2]);
    expect(r.existing).toBe(false);
    expect(r.vertex.pos).toEqual([1, 0, 2]);
    expect(addPattern(m, 'plane_top').name).toBe('Pattern 1'); // the smallest unused number ("Test pattern" does not count)
    expect(m.patterns && Object.keys(m.patterns).length).toBe(2);
  });

  it('addPattern names: the smallest unused number, so a deleted pattern\'s name is reused and never duplicated', () => {
    const m = createModel();
    const p1 = addPattern(m, 'plane_top');
    const p2 = addPattern(m, 'plane_top');
    expect([p1.name, p2.name]).toEqual(['Pattern 1', 'Pattern 2']);
    expect(removePattern(m, p1.id)).toBe(true);
    const p3 = addPattern(m, 'plane_top');
    expect(p3.name).toBe('Pattern 1');
    const names = Object.values(m.patterns!).map((p) => p.name);
    expect(new Set(names).size).toBe(names.length);
    expect(nextPatternName(m)).toBe('Pattern 3');
    expect(nextPatternName(m, 'Sheet')).toBe('Sheet 1');
  });

  it('a construction plane that carries a pattern cannot be deleted (the pattern\'s frame is the plane\'s)', () => {
    const m = createModel();
    const plane = addConstructionPlane(m, [0, 0, 5], [0, 0, -1], 'Under', [1, 0, 0]);
    const p = miuraPattern(m, plane.id, 1, 1, 60);
    const before = serializeModel(m);
    expect(planeCarriesPattern(m, plane.id)).toBe(true);
    expect(removeConstruction(m, plane.id)).toBe(false);
    expect(serializeModel(m)).toBe(before);
    expect(patternBasis(m, p).normal).toEqual([0, 0, -1]);
    expect(patternFaces(m, p).faces.length).toBe(4);
    // the builtin planes stay refused for their own reason; a plane without a pattern goes
    expect(removeConstruction(m, 'plane_top')).toBe(false);
    const free = addOffsetPlane(m, 'plane_front', 1)!;
    expect(planeCarriesPattern(m, free.id)).toBe(false);
    expect(removeConstruction(m, free.id)).toBe(true);
    // once the pattern is gone the plane can be deleted
    expect(removePattern(m, p.id)).toBe(true);
    expect(removeConstruction(m, plane.id)).toBe(true);
    expect(m.construction[plane.id]).toBeUndefined();
  });
});

// ---------------------------------------------------------------------------
// Editing
// ---------------------------------------------------------------------------

describe('editing', () => {
  it('merges a vertex within tolerance and keeps the existing one', () => {
    const { m, p } = fresh();
    const a = V(m, p, 0, 0);
    V(m, p, 2, 0);
    const tol = patternTolerance(p);
    expect(tol).toBeCloseTo(2e-6, 12);
    const r = addPatternVertex(m, p, [tol / 2, 0, 0]);
    expect(r.existing).toBe(true);
    expect(r.vertex.id).toBe(a);
    expect(count(p).vertices).toBe(2);
    expect(findPatternVertex(p, [0, tol / 2, 0])?.id).toBe(a);
    expect(findPatternVertex(p, [0, 10 * tol, 0])).toBeNull();
    // a vertex just outside the tolerance is a new one
    expect(addPatternVertex(m, p, [0, 10 * tol, 0]).existing).toBe(false);
    expect(count(p).vertices).toBe(3);
  });

  it('a vertex on a crease splits it into two pieces of the same class and target', () => {
    const { m, p } = fresh();
    const a = V(m, p, 0, 0);
    const b = V(m, p, 2, 0);
    const [ab] = C(m, p, a, b, 'M');
    setCreaseTarget(m, p, ab, 120);
    expect(findPatternCrease(p, [1, 0, 0])?.crease.id).toBe(ab);
    expect(findPatternCrease(p, [1, 0, 0])?.t).toBeCloseTo(0.5, 12);
    expect(findPatternCrease(p, [1, 0.01, 0])).toBeNull(); // too far from the segment
    expect(findPatternCrease(p, [2, 0, 0])).toBeNull(); // the end point belongs to the vertex
    const r = addPatternVertex(m, p, [1, 1e-9, 0]);
    expect(r.existing).toBe(false);
    expect(r.vertex.pos).toEqual([1, 0, 0]); // the foot point on the crease
    expect(r.splitCreaseIds.length).toBe(2);
    expect(r.splitCreaseIds[0]).toBe(ab);
    const [first, second] = r.splitCreaseIds.map((id) => p.creases[id]);
    expect([first.a, first.b]).toEqual([a, r.vertex.id]);
    expect([second.a, second.b]).toEqual([r.vertex.id, b]);
    expect(first.cls).toBe('M');
    expect(second.cls).toBe('M');
    expect(second.targetDeg).toBe(120);
    expect(count(p)).toEqual({ vertices: 3, creases: 2 });
  });

  it('refuses a crease to the same vertex, a duplicate in either direction, a zero length and unknown vertices', () => {
    const { m, p } = fresh();
    const a = V(m, p, 0, 0);
    const b = V(m, p, 2, 0);
    expect(addPatternCrease(m, p, a, a)).toEqual({ ok: false, reason: 'sameVertex' });
    expect(addPatternCrease(m, p, a, 'nope')).toEqual({ ok: false, reason: 'missingVertex' });
    const [ab] = C(m, p, a, b, 'V');
    expect(addPatternCrease(m, p, a, b)).toEqual({ ok: false, reason: 'duplicate', creaseId: ab });
    expect(addPatternCrease(m, p, b, a)).toEqual({ ok: false, reason: 'duplicate', creaseId: ab });
    // two vertices closer than the tolerance (inserted directly, which addPatternVertex never does)
    p.vertices['pv_x'] = { id: 'pv_x', pos: [1e-9, 0, 0] };
    expect(addPatternCrease(m, p, a, 'pv_x')).toEqual({ ok: false, reason: 'zeroLength' });
    expect(count(p).creases).toBe(1);
  });

  it('refuses collinear creases that overlap (contained or partial) but accepts one that only touches at an end', () => {
    const { m, p } = fresh();
    const a = V(m, p, 0, 0);
    const b = V(m, p, 2, 0);
    const c = V(m, p, 4, 0);
    const d = V(m, p, 3, 0);
    const [ab] = C(m, p, a, b, 'M');
    expect(addPatternCrease(m, p, a, c)).toEqual({ ok: false, reason: 'overlap', creaseId: ab }); // contains a–b
    expect(addPatternCrease(m, p, d, a)).toEqual({ ok: false, reason: 'overlap', creaseId: ab }); // partial overlap
    expect(addPatternCrease(m, p, b, a)).toEqual({ ok: false, reason: 'duplicate', creaseId: ab });
    const bd = C(m, p, b, d, 'V'); // touches a–b at b only
    expect(bd.length).toBe(1);
    expect(count(p)).toEqual({ vertices: 4, creases: 2 });
    // the pattern is unchanged by a refusal
    const before = JSON.stringify(p);
    expect(addPatternCrease(m, p, a, d).ok).toBe(false);
    expect(JSON.stringify(p)).toBe(before);
  });

  it('two crossing creases give 4 creases and 1 new vertex at the crossing', () => {
    const { m, p } = fresh();
    const a = V(m, p, 0, 0);
    const b = V(m, p, 2, 2);
    const c = V(m, p, 0, 2);
    const d = V(m, p, 2, 0);
    const [ab] = C(m, p, a, b, 'M');
    const r = addPatternCrease(m, p, c, d, 'V');
    expect(r.ok).toBe(true);
    if (!r.ok) return;
    expect(r.creaseIds.length).toBe(2);
    expect(r.vertexIds.length).toBe(1);
    const x = p.vertices[r.vertexIds[0]];
    expect(dist(x.pos, [1, 1, 0])).toBeLessThan(1e-12);
    expect(count(p)).toEqual({ vertices: 5, creases: 4 });
    expect(creasesAtVertex(p, x.id).length).toBe(4);
    // the crossed crease kept its id for the piece at its first vertex, the new piece continues to b
    expect([p.creases[ab].a, p.creases[ab].b]).toEqual([a, x.id]);
    const abTail = Object.values(p.creases).find((cr) => cr.a === x.id && cr.b === b)!;
    expect(abTail.cls).toBe('M');
    // the new crease runs c → x → d
    expect([p.creases[r.creaseIds[0]].a, p.creases[r.creaseIds[0]].b]).toEqual([c, x.id]);
    expect([p.creases[r.creaseIds[1]].a, p.creases[r.creaseIds[1]].b]).toEqual([x.id, d]);
    expect(r.creaseIds.every((id) => p.creases[id].cls === 'V')).toBe(true);
    // no face: everything is dangling
    const f = patternFaces(m, p);
    expect(f.faces).toEqual([]);
    expect(f.outer).toEqual([]);
    expect(f.dangling.length).toBe(4);
    expect(f.interior).toEqual([]);
    expect(f.border).toEqual([]);
    const issues = validatePattern(m, p);
    expect(kinds(issues, 'noFaces').length).toBe(1);
    expect(kinds(issues, 'dangling').length).toBe(4);
  });

  it('splits a new crease at vertices lying on it and a crease at an end point that lies on its interior', () => {
    const { m, p } = fresh();
    const a = V(m, p, 0, 0);
    const b = V(m, p, 4, 0);
    const x = V(m, p, 2, 0); // isolated vertex on the future crease
    const r = addPatternCrease(m, p, a, b, 'M');
    expect(r.ok && r.creaseIds.length).toBe(2);
    expect(r.ok && r.vertexIds).toEqual([]);
    expect(creasesAtVertex(p, x).length).toBe(2);
    // a T-junction: a crease ending on an existing crease's interior (vertex placed through addPatternVertex)
    const y = V(m, p, 1, 0);
    expect(count(p).creases).toBe(3);
    const top = V(m, p, 1, 2);
    expect(C(m, p, top, y).length).toBe(1);
    expect(count(p)).toEqual({ vertices: 5, creases: 4 });
    // a vertex inserted directly onto a crease's interior (bypassing addPatternVertex): the crease is split when
    // a crease is drawn from it
    p.vertices['pv_raw'] = { id: 'pv_raw', pos: [3, 0, 0] };
    const low = V(m, p, 3, -2);
    expect(C(m, p, 'pv_raw', low).length).toBe(1);
    expect(creasesAtVertex(p, 'pv_raw').length).toBe(3);
    expect(count(p)).toEqual({ vertices: 7, creases: 6 });
  });

  it('removes creases, vertices with their creases, and sets classes and targets', () => {
    const { m, p } = fresh();
    const [a, b, c] = square(m, p);
    const [diag] = C(m, p, a, c, 'U');
    expect(setCreaseClass(m, p, diag, 'V')).toBe(true);
    expect(p.creases[diag].cls).toBe('V');
    expect(setCreaseClass(m, p, 'nope', 'V')).toBe(false);
    expect(setCreaseTarget(m, p, diag, 250)).toBe(true);
    expect(p.creases[diag].targetDeg).toBe(180); // clamped
    expect(setCreaseTarget(m, p, diag, -5)).toBe(true);
    expect(p.creases[diag].targetDeg).toBe(0);
    expect(setCreaseTarget(m, p, diag, undefined)).toBe(true);
    expect('targetDeg' in p.creases[diag]).toBe(false);
    expect(removePatternCrease(m, p, diag)).toBe(true);
    expect(removePatternCrease(m, p, diag)).toBe(false);
    expect(count(p)).toEqual({ vertices: 4, creases: 4 });
    const removed = removePatternVertex(m, p, b);
    expect(removed.length).toBe(2);
    expect(count(p)).toEqual({ vertices: 3, creases: 2 });
    expect(removePatternVertex(m, p, b)).toEqual([]);
    // a crease removal leaves its vertices, which the validator reports when they become isolated
    const { m: m2, p: p2 } = fresh();
    const x = V(m2, p2, 0, 0);
    const y = V(m2, p2, 1, 0);
    const [xy] = C(m2, p2, x, y);
    removePatternCrease(m2, p2, xy);
    expect(kinds(validatePattern(m2, p2), 'isolated').map((i) => i.vertexId).sort()).toEqual([x, y].sort());
  });
});

// ---------------------------------------------------------------------------
// Face detection
// ---------------------------------------------------------------------------

describe('patternFaces', () => {
  it('a square with one diagonal: 2 counter-clockwise faces, 1 interior and 4 border creases, a clockwise outer boundary', () => {
    const { m, p } = fresh();
    const [a, b, c, d] = square(m, p);
    const [diag] = C(m, p, a, c, 'V');
    const f = patternFaces(m, p);
    expect(f.faces.length).toBe(2);
    for (const face of f.faces) {
      expect(face.length).toBe(3);
      expect(area2d(m, p, face)).toBeCloseTo(2, 12);
    }
    expect(f.faces.map((face) => sorted(face.map((id) => [a, b, c, d].indexOf(id))))).toEqual([
      [0, 1, 2],
      [0, 2, 3],
    ]);
    expect(f.interior).toEqual([diag]);
    expect(f.border.length).toBe(4);
    expect(f.dangling).toEqual([]);
    expect(f.isolated).toEqual([]);
    expect(f.outer.length).toBe(4);
    expect(area2d(m, p, f.outer)).toBeCloseTo(-4, 12);
    expect(validatePattern(m, p)).toEqual([]);
  });

  it('a square with both diagonals: 4 faces around the crossing vertex', () => {
    const { m, p } = fresh();
    const [a, b, c, d] = square(m, p);
    C(m, p, a, c, 'M');
    const r = addPatternCrease(m, p, b, d, 'V');
    expect(r.ok && r.vertexIds.length).toBe(1);
    expect(count(p)).toEqual({ vertices: 5, creases: 8 });
    const f = patternFaces(m, p);
    expect(f.faces.length).toBe(4);
    expect(f.interior.length).toBe(4);
    expect(f.border.length).toBe(4);
    const x = (r.ok && r.vertexIds[0]) as ID;
    expect(patternInteriorVertices(m, p)).toEqual([x]);
    const s = patternVertexSectors(m, p, x);
    expect(s.creaseIds.length).toBe(4);
    expect(s.sectorDeg.map((v) => Math.round(v))).toEqual([90, 90, 90, 90]);
    // the X vertex passes Kawasaki but its 2 : 2 assignment fails Maekawa
    const issues = validatePattern(m, p);
    expect(kinds(issues, 'kawasaki')).toEqual([]);
    expect(kinds(issues, 'maekawa')).toEqual([{ kind: 'maekawa', vertexId: x, creaseIds: s.creaseIds, mountains: 2, valleys: 2 }]);
  });

  it('a dangling crease is reported and leaves the face cycle without a spur', () => {
    const { m, p } = fresh();
    const [a] = square(m, p);
    const x = V(m, p, 1, 1);
    const [spur] = C(m, p, a, x, 'M');
    const f = patternFaces(m, p);
    expect(f.faces.length).toBe(1);
    expect(f.faces[0].length).toBe(4);
    expect(f.faces[0]).not.toContain(x);
    expect(f.dangling).toEqual([spur]);
    expect(f.border.length).toBe(4);
    expect(f.interior).toEqual([]);
    expect(patternInteriorVertices(m, p)).toEqual([]);
    const issues = validatePattern(m, p);
    expect(kinds(issues, 'dangling')).toEqual([{ kind: 'dangling', creaseIds: [spur] }]);
    expect(kinds(issues, 'noFaces')).toEqual([]);
    // a branching tree inside the face collapses too
    const y = V(m, p, 1.5, 1.2);
    const z = V(m, p, 0.5, 1.4);
    C(m, p, x, y);
    C(m, p, x, z);
    const f2 = patternFaces(m, p);
    expect(f2.faces.length).toBe(1);
    expect(f2.faces[0].length).toBe(4);
    expect(f2.dangling.length).toBe(3);
  });

  it('two separate squares are two faces of two components, all creases border', () => {
    const { m, p } = fresh();
    square(m, p, 2, [0, 0]);
    square(m, p, 1, [5, 5]);
    const f = patternFaces(m, p);
    expect(f.faces.length).toBe(2);
    expect(f.border.length).toBe(8);
    expect(f.interior).toEqual([]);
    expect(f.dangling).toEqual([]);
    expect(area2d(m, p, f.outer)).toBeCloseTo(-4, 12); // the larger component's boundary
  });

  it('faces are counter-clockwise about the plane normal on any plane', () => {
    const { m, p } = fresh('plane_front');
    const a = V(m, p, 0, 0, 0);
    const b = V(m, p, 2, 0, 0);
    const c = V(m, p, 2, 0, 2);
    const d = V(m, p, 0, 0, 2);
    for (const [x, y] of [[a, b], [b, c], [c, d], [d, a]] as [ID, ID][]) C(m, p, x, y, 'B');
    C(m, p, a, c, 'V');
    const f = patternFaces(m, p);
    expect(f.faces.length).toBe(2);
    for (const face of f.faces) {
      const n = normalize(newellNormal(face.map((id) => p.vertices[id].pos)));
      expect(dist(n, [0, -1, 0])).toBeLessThan(1e-12);
    }
  });

  it('a closed outline inside a face (a hole) is reported with the enclosing face, which generates no panel', () => {
    const { m, p } = fresh();
    const outer = square(m, p, 6, [0, 0]);
    const inner = square(m, p, 2, [2, 2]);
    const f = patternFaces(m, p);
    expect(f.faces.length).toBe(2);
    expect(f.border.length).toBe(8);
    expect(f.holes.length).toBe(1);
    expect(f.faces[f.holes[0].face]).toEqual(expect.arrayContaining(outer));
    expect(sorted(f.holes[0].creaseIds.map((id) => Number(id.split('_')[1])))).toEqual(sorted(Object.values(p.creases).filter((c) => inner.includes(c.a) && inner.includes(c.b)).map((c) => Number(c.id.split('_')[1]))));
    const issues = validatePattern(m, p);
    expect(issues.map((i) => i.kind)).toEqual(['hole']);
    expect(issues[0].creaseIds).toEqual(f.holes[0].creaseIds);
    const r = generatePanels(m, p);
    expect(r.linkIds.length).toBe(1); // the inner panel only: the outer one would cover it
    expect(r.jointIds.length).toBe(0);
    expect(r.warnings.map((w) => w.kind)).toEqual(['hole']);
    const pts = m.links[r.linkIds[0]].pointIds.map((id) => m.points[id].pos);
    expect(pts.every((q) => q[0] >= 2 - 1e-9 && q[0] <= 4 + 1e-9 && q[1] >= 2 - 1e-9 && q[1] <= 4 + 1e-9)).toBe(true);
    // a hinged pair inside the hole generates its crease; the outer face is still skipped
    C(m, p, inner[0], inner[2], 'V');
    const r2 = generatePanels(m, p);
    expect(patternFaces(m, p).faces.length).toBe(3);
    expect(r2.linkIds.length).toBe(2);
    expect(r2.jointIds.length).toBe(1);
    expect(r2.warnings.map((w) => w.kind)).toEqual(['hole']);
    // two squares side by side (the existing case) and a square with a diagonal have no hole
    const { m: m2, p: p2 } = fresh();
    square(m2, p2, 2, [0, 0]);
    square(m2, p2, 2, [5, 0]);
    expect(patternFaces(m2, p2).holes).toEqual([]);
    expect(validatePattern(m2, p2)).toEqual([]);
    const { m: m3, p: p3 } = fresh();
    const [a3, , c3] = square(m3, p3);
    C(m3, p3, a3, c3, 'V');
    expect(patternFaces(m3, p3).holes).toEqual([]);
    // an outline joined to the boundary by a single crease pinches the ring face at that crease: reported, not generated
    const { m: m4, p: p4 } = fresh();
    const outer4 = square(m4, p4, 6, [0, 0]);
    const inner4 = square(m4, p4, 2, [2, 2]);
    const [link] = C(m4, p4, outer4[0], inner4[0], 'B');
    const f4 = patternFaces(m4, p4);
    expect(f4.faces.length).toBe(2);
    expect(f4.holes.length).toBe(1);
    expect(f4.holes[0].creaseIds).toEqual([link]);
    expect(f4.dangling).toEqual([link]); // the pinch crease has the ring face on both sides: no hinge either way
    expect(validatePattern(m4, p4).map((i) => i.kind)).toEqual(['dangling', 'hole']);
    const r4 = generatePanels(m4, p4);
    expect(r4.linkIds.length).toBe(1);
    expect(r4.warnings.map((w) => w.kind)).toEqual(['dangling', 'hole']);
    // a second connector makes two proper ring halves: no hole, three panels and the hinges between them (the inner
    // sides and the connectors are now interior border creases, and the connector vertices have degree 3: reported)
    C(m4, p4, outer4[2], inner4[2], 'B');
    expect(patternFaces(m4, p4).holes).toEqual([]);
    const kinds4 = validatePattern(m4, p4).map((i) => i.kind);
    expect(kinds4).not.toContain('hole');
    expect(kinds4).not.toContain('dangling');
    expect(kinds4.filter((k) => k === 'borderInside').length).toBe(6);
    const r5 = generatePanels(m4, p4);
    expect(r5.linkIds.length).toBe(3);
    expect(r5.jointIds.length).toBe(6);
  });
});

// ---------------------------------------------------------------------------
// Validation
// ---------------------------------------------------------------------------

describe('validatePattern', () => {
  it('the Miura unit: 4 faces, 4 interior creases, Kawasaki 0 and Maekawa 3 : 1 once M, V, V, V sit in the right places', () => {
    const { m } = fresh();
    const p = miuraPattern(m, 'plane_top', 1, 1, 60, 2, 2);
    expect(count(p)).toEqual({ vertices: 9, creases: 12 });
    const f = patternFaces(m, p);
    expect(f.faces.length).toBe(4);
    expect(f.interior.length).toBe(4);
    expect(f.border.length).toBe(8);
    expect(f.dangling).toEqual([]);
    const [x] = patternInteriorVertices(m, p);
    expect(x).toBeDefined();
    const s = patternVertexSectors(m, p, x);
    expect(sorted(s.sectorDeg)).toEqual([60, 60, 120, 120].map((v) => expect.closeTo(v, 9)));
    expect(validatePattern(m, p)).toEqual([]);
    const classes = s.creaseIds.map((id) => p.creases[id].cls);
    expect(sorted([classes.filter((c) => c === 'M').length, classes.filter((c) => c === 'V').length])).toEqual([1, 3]);
    // unassigned: every interior crease is reported, nothing else
    const assigned = s.creaseIds.map((id) => p.creases[id].cls);
    for (const id of s.creaseIds) setCreaseClass(m, p, id, 'U');
    const issues = validatePattern(m, p);
    expect(kinds(issues, 'unassigned').map((i) => i.creaseIds![0]).sort()).toEqual([...s.creaseIds].sort());
    expect(issues.length).toBe(4);
    // M, V, V, V in the right places: the odd class on the collinear crease between the two 60° sectors
    s.creaseIds.forEach((id, i) => setCreaseClass(m, p, id, assigned[i]));
    expect(validatePattern(m, p)).toEqual([]);
    const odd = classes.findIndex((c) => classes.filter((d) => d === c).length === 1);
    const n = s.sectorDeg.length;
    expect(s.sectorDeg[(odd - 1 + n) % n]).toBeCloseTo(60, 9);
    expect(s.sectorDeg[odd]).toBeCloseTo(60, 9);
    // the odd class in a wrong place (2 : 2) fails Maekawa
    const wrong = (odd + 1) % n;
    setCreaseClass(m, p, s.creaseIds[wrong], classes[odd]);
    expect(kinds(validatePattern(m, p), 'maekawa')).toEqual([{ kind: 'maekawa', vertexId: x, creaseIds: s.creaseIds, mountains: 2, valleys: 2 }]);
  });

  it('an interior vertex of odd degree', () => {
    const { m, p } = fresh();
    const [a, b, c] = square(m, p);
    const x = V(m, p, 1, 1);
    const at = [C(m, p, x, a)[0], C(m, p, x, b)[0], C(m, p, x, c)[0]];
    expect(patternFaces(m, p).faces.length).toBe(3);
    const issues = validatePattern(m, p);
    const odd = kinds(issues, 'oddDegree');
    expect(odd.length).toBe(1);
    expect(odd[0].vertexId).toBe(x);
    expect([...odd[0].creaseIds!].sort()).toEqual([...at].sort());
    expect(kinds(issues, 'kawasaki')).toEqual([]);
    expect(kinds(issues, 'unassigned').length).toBe(3);
  });

  it('Kawasaki: the alternating sector sum of a degree-4 vertex that is not zero', () => {
    const { m, p } = fresh();
    const [a, b, c] = square(m, p);
    const x = V(m, p, 1, 1);
    const e = V(m, p, 0, 1.5); // splits the left side
    for (const v of [a, b, c, e]) C(m, p, x, v, 'M');
    expect(count(p).creases).toBe(9);
    const s = patternVertexSectors(m, p, x);
    // directions from x, sorted counter-clockwise from −180°: a −135°, b −45°, c 45°, e 153.4°
    expect(s.creaseIds.map((id) => p.creases[id].b)).toEqual([a, b, c, e]);
    expect(s.sectorDeg.map((v) => Math.round(v * 10) / 10)).toEqual([90, 90, 108.4, 71.6]);
    const issues = validatePattern(m, p);
    const k = kinds(issues, 'kawasaki');
    expect(k.length).toBe(1);
    expect(k[0].vertexId).toBe(x);
    expect(k[0].valueDeg).toBeCloseTo(108.43 - 71.57, 1);
    expect(kinds(issues, 'maekawa')).toEqual([{ kind: 'maekawa', vertexId: x, creaseIds: s.creaseIds, mountains: 4, valleys: 0 }]);
    // a vertex where the sum is within the tolerance passes
    expect(kinds(validatePattern(m, p, 40), 'kawasaki')).toEqual([]);
  });

  it('an empty pattern has no faces; a straight crease through a degree-2 vertex passes', () => {
    const { m, p } = fresh();
    expect(validatePattern(m, p)).toEqual([{ kind: 'noFaces' }]);
    const [a, b, c, d] = square(m, p);
    const top = V(m, p, 1, 2); // splits c–d
    const bottom = V(m, p, 1, 0); // splits a–b
    const mid = V(m, p, 1, 1);
    C(m, p, top, mid, 'M');
    C(m, p, mid, bottom, 'M');
    expect(patternInteriorVertices(m, p)).toEqual([mid]);
    expect(validatePattern(m, p)).toEqual([]);
    setCreaseClass(m, p, creasesAtVertex(p, mid)[0], 'V');
    expect(kinds(validatePattern(m, p), 'maekawa').length).toBe(1);
    void [a, b, c, d];
  });
});

// ---------------------------------------------------------------------------
// Generation
// ---------------------------------------------------------------------------

describe('generatePanels', () => {
  it('a square with a valley diagonal: 2 panels, 1 crease with 2 merged pairs, the class and target, zero violation', () => {
    const { m, p } = fresh();
    const [a, , c] = square(m, p);
    const [diag] = C(m, p, a, c, 'V');
    setCreaseTarget(m, p, diag, 120);
    const r = generatePanels(m, p);
    expect(r.linkIds.length).toBe(2);
    expect(r.jointIds.length).toBe(1);
    expect(r.warnings).toEqual([]);
    expect(m.links[r.linkIds[0]].name).toBe('Panel 1');
    expect(m.links[r.linkIds[1]].name).toBe('Panel 2');
    expect(groundLink(m)?.id).toBe(r.linkIds[0]);
    const j = m.joints[r.jointIds[0]];
    expect(j.type).toBe('revolute');
    expect(j.pairs?.length).toBe(2);
    expect(isCrease(m, j)).toBe(true);
    expect(j.fold).toEqual({ mv: 'V', target: 120 });
    expect(currentViolation(m)).toBeLessThan(1e-12);
    // the panels are the faces, at the pattern's positions, without a sketch-plane constraint
    for (const id of r.linkIds) {
      const link = m.links[id];
      expect(link.kind).toBe('polygon');
      expect(link.pointIds.length).toBe(3);
      expect(bodyPlaneJoint(m, id)).toBeNull();
      const face = p.generated!.faces[id];
      link.pointIds.forEach((pid, i) => expect(m.points[pid].pos).toEqual(p.vertices[face[i]].pos));
    }
    expect(p.generated).toEqual({ linkIds: r.linkIds, jointIds: r.jointIds, faces: p.generated!.faces });
    // the merged pairs join the points that lie at the crease's vertices
    for (const [x, y] of j.pairs!) {
      expect(m.points[x].pos).toEqual(m.points[y].pos);
      expect([a, c].some((vid) => dist(p.vertices[vid].pos, m.points[x].pos) < 1e-12)).toBe(true);
    }
    expect(computeMobility(m).dof).toBe(1);
  });

  it('the 2 × 2 Miura sheet: 9 panels, 12 interior creases with the drawn classes, zero violation, developable with Kawasaki 0', () => {
    const { m } = fresh();
    const p = miuraPattern(m, 'plane_top', 2, 2);
    expect(count(p)).toEqual({ vertices: 16, creases: 24 });
    const f = patternFaces(m, p);
    expect(f.faces.length).toBe(9);
    expect(f.interior.length).toBe(12);
    expect(f.border.length).toBe(12);
    expect(f.interior.filter((id) => p.creases[id].cls === 'M').length).toBe(6);
    expect(f.interior.filter((id) => p.creases[id].cls === 'V').length).toBe(6);
    expect(f.border.every((id) => p.creases[id].cls === 'B')).toBe(true);
    expect(validatePattern(m, p)).toEqual([]);
    // every interior vertex: sectors (60, 120, 120, 60) up to rotation, 3 : 1 with the odd class between the two 60° sectors
    const interior = patternInteriorVertices(m, p);
    expect(interior.length).toBe(4);
    for (const v of interior) {
      const s = patternVertexSectors(m, p, v);
      expect(sorted(s.sectorDeg)).toEqual([60, 60, 120, 120].map((x) => expect.closeTo(x, 9)));
      const classes = s.creaseIds.map((id) => p.creases[id].cls);
      const odd = classes.findIndex((c) => classes.filter((d) => d === c).length === 1);
      expect(odd).toBeGreaterThanOrEqual(0);
      expect(s.sectorDeg[(odd + 3) % 4]).toBeCloseTo(60, 9);
      expect(s.sectorDeg[odd]).toBeCloseTo(60, 9);
    }
    // each zigzag line has one class, alternating between neighbouring zigzags
    const basis = patternBasis(m, p);
    const tilted = f.interior.filter((id) => Math.abs(to2d(basis, p.vertices[p.creases[id].a].pos)[1] - to2d(basis, p.vertices[p.creases[id].b].pos)[1]) > 1e-9);
    expect(tilted.length).toBe(6);
    const column = (id: ID): number => Math.round(Math.min(...[p.creases[id].a, p.creases[id].b].map((vid) => to2d(basis, p.vertices[vid].pos)[0])) / 2);
    const byColumn = new Map<number, Set<CreaseClass>>();
    for (const id of tilted) byColumn.set(column(id), new Set([...(byColumn.get(column(id)) ?? []), p.creases[id].cls]));
    expect([...byColumn.values()].every((set) => set.size === 1)).toBe(true);
    expect(new Set([...byColumn.values()].map((set) => [...set][0])).size).toBe(2);

    const r = generatePanels(m, p);
    expect(r.linkIds.length).toBe(9);
    expect(r.jointIds.length).toBe(12);
    expect(r.warnings).toEqual([]);
    expect(Object.keys(m.links).length).toBe(9);
    expect(Object.keys(m.joints).length).toBe(12);
    expect(currentViolation(m)).toBeLessThan(1e-12);
    for (const id of r.linkIds) {
      expect(m.links[id].pointIds.length).toBe(4);
      expect(m.links[id].pointIds.every((pid) => Math.abs(m.points[pid].pos[2]) < 1e-15)).toBe(true);
    }
    // every joint is a crease with two merged pairs and the class of its pattern crease
    const byCrease = new Map<ID, ID>();
    for (const cid of f.interior) {
      const [va, vb] = [p.vertices[p.creases[cid].a].pos, p.vertices[p.creases[cid].b].pos];
      const j = r.jointIds.map((id) => m.joints[id]).find((jt) => {
        const [x, y] = jt.a.pointIds.map((pid) => m.points[pid].pos);
        return (dist(x, va) < 1e-12 && dist(y, vb) < 1e-12) || (dist(x, vb) < 1e-12 && dist(y, va) < 1e-12);
      });
      expect(j).toBeDefined();
      byCrease.set(cid, j!.id);
      expect(j!.pairs?.length).toBe(2);
      expect(isCrease(m, j!)).toBe(true);
      expect(j!.fold?.mv).toBe(p.creases[cid].cls);
      expect(j!.fold?.target).toBeUndefined();
    }
    expect(new Set(byCrease.values()).size).toBe(12);
    // the Phase 2 validator sees four flat, developable, Kawasaki-0 vertices
    const reports = validateCreasePattern(m);
    expect(reports.length).toBe(4);
    for (const rep of reports) {
      expect(rep.panelCount).toBe(4);
      expect(rep.developable).toBe(true);
      expect(rep.sumDeg).toBeCloseTo(360, 9);
      expect(Math.abs(rep.kawasakiDeg)).toBeLessThan(1e-9);
      expect(rep.flatFoldable).toBe(true);
      expect(rep.maekawa).toBeNull(); // flat: the classes are not readable from the pose
    }
    expect(findCreaseLoops(m).every((loop) => loop.flat)).toBe(true);
  });

  it('regeneration leaves the same counts and no stale link or joint ids', () => {
    const { m } = fresh();
    const p = miuraPattern(m, 'plane_top', 2, 2);
    const first = generatePanels(m, p);
    const ground = groundLink(m)!.id;
    expect(first.linkIds).toContain(ground);
    setCreaseTarget(m, p, patternFaces(m, p).interior[0], 100);
    const second = generatePanels(m, p);
    expect(second.linkIds.length).toBe(9);
    expect(second.jointIds.length).toBe(12);
    expect(Object.keys(m.links).length).toBe(9);
    expect(Object.keys(m.joints).length).toBe(12);
    for (const id of first.linkIds) expect(m.links[id]).toBeUndefined();
    for (const id of first.jointIds) expect(m.joints[id]).toBeUndefined();
    expect(second.linkIds.every((id) => !!m.links[id])).toBe(true);
    expect(second.jointIds.every((id) => !!m.joints[id])).toBe(true);
    expect(p.generated!.linkIds).toEqual(second.linkIds);
    expect(p.generated!.jointIds).toEqual(second.jointIds);
    // no point of a removed link survives, every joint references live points
    expect(Object.values(m.points).every((pt) => !!m.links[pt.linkId])).toBe(true);
    for (const j of Object.values(m.joints)) {
      const ids = [...j.a.pointIds, ...(isConstructionRef(j.b) ? [] : j.b.pointIds)];
      expect(ids.every((id) => !!m.points[id])).toBe(true);
    }
    expect(groundLink(m)?.id).toBe(second.linkIds[0]);
    expect(m.links[second.linkIds[0]].name).toBe('Panel 1');
    expect(currentViolation(m)).toBeLessThan(1e-12);
    expect(second.jointIds.map((id) => m.joints[id].fold?.target).filter((t) => t !== undefined)).toEqual([100]);
  });

  it('keeps an existing ground, can skip grounding, generates unassigned creases with a warning and nothing without faces', () => {
    const { m, p } = fresh();
    const [a, , c] = square(m, p);
    C(m, p, a, c, 'U');
    const r = generatePanels(m, p, { ground: false, panelName: 'Sheet' });
    expect(groundLink(m)).toBeNull();
    expect(m.links[r.linkIds[0]].name).toBe('Sheet 1');
    expect(r.warnings.map((w) => w.kind)).toEqual(['unassigned']);
    expect(m.joints[r.jointIds[0]].fold).toEqual({});
    // a second pattern leaves the first's ground alone
    const q = addPattern(m, 'plane_top');
    square(m, q, 1, [5, 5]);
    const r2 = generatePanels(m, q);
    expect(r2.linkIds.length).toBe(1);
    expect(r2.jointIds).toEqual([]);
    expect(groundLink(m)?.id).toBe(r2.linkIds[0]);
    const r3 = generatePanels(m, p);
    expect(groundLink(m)?.id).toBe(r2.linkIds[0]);
    expect(r3.linkIds.length).toBe(2);
    // an empty pattern generates nothing
    const empty = addPattern(m, 'plane_top');
    expect(generatePanels(m, empty)).toEqual({ linkIds: [], jointIds: [], warnings: [{ kind: 'noFaces' }] });
    expect(empty.generated).toEqual({ linkIds: [], jointIds: [], faces: {} });
  });

  it('panels generated on another plane lie in it with their normal along the plane normal', () => {
    const { m } = fresh();
    const p = miuraPattern(m, 'plane_front', 1, 1, 60, 1.5, 1);
    const r = generatePanels(m, p);
    expect(r.linkIds.length).toBe(4);
    expect(r.jointIds.length).toBe(4);
    expect(currentViolation(m)).toBeLessThan(1e-12);
    for (const id of r.linkIds) {
      const pts = m.links[id].pointIds.map((pid) => m.points[pid].pos);
      expect(pts.every((pt) => Math.abs(pt[1]) < 1e-15)).toBe(true);
      expect(dist(normalize(newellNormal(pts)), [0, -1, 0])).toBeLessThan(1e-12);
    }
    const reports = validateCreasePattern(m);
    expect(reports.length).toBe(1);
    expect(Math.abs(reports[0].kawasakiDeg)).toBeLessThan(1e-9);
  });

  it('on every builtin plane the generated panels face the camera of the plane\'s view (top +Z, front −Y, right +X)', () => {
    // scene.ts setView places the camera along these directions, so the side the crease classes are judged from
    // (the plane normal, the panels\' oriented normal) is the side the user draws the pattern from
    const views: [string, Vec3][] = [['plane_top', [0, 0, 1]], ['plane_front', [0, -1, 0]], ['plane_right', [1, 0, 0]]];
    for (const [planeId, toCamera] of views) {
      const { m } = fresh();
      expect(dist(normalize(m.construction[planeId].dir!), toCamera)).toBeLessThan(1e-12);
      const p = miuraPattern(m, planeId, 1, 1, 60, 1.5, 1);
      const r = generatePanels(m, p);
      expect(r.linkIds.length).toBe(4);
      for (const id of r.linkIds) {
        const pts = m.links[id].pointIds.map((pid) => m.points[pid].pos);
        expect(dist(normalize(newellNormal(pts)), toCamera)).toBeLessThan(1e-12);
      }
    }
  });

  it('an interior crease of class Border is reported (borderInside) and generates a hinge without a class, counted as a warning', () => {
    const { m, p } = fresh();
    const [a, , c] = square(m, p);
    const [diag] = C(m, p, a, c, 'B');
    const f = patternFaces(m, p);
    expect(f.faces.length).toBe(2);
    expect(f.interior).toEqual([diag]);
    expect(f.border.length).toBe(4);
    const issues = validatePattern(m, p);
    expect(issues).toEqual([{ kind: 'borderInside', creaseIds: [diag] }]);
    const r = generatePanels(m, p);
    expect(r.linkIds.length).toBe(2);
    expect(r.jointIds.length).toBe(1);
    expect(m.joints[r.jointIds[0]].fold).toEqual({});
    expect(r.warnings.length).toBe(1);
    // the vertex conditions treat it as unassigned: Maekawa is skipped on the Miura unit with one border inside
    const { m: m2 } = fresh();
    const p2 = miuraPattern(m2, 'plane_top', 1, 1, 60, 2, 2);
    const inner = patternFaces(m2, p2).interior[0];
    setCreaseClass(m2, p2, inner, 'B');
    expect(validatePattern(m2, p2)).toEqual([{ kind: 'borderInside', creaseIds: [inner] }]);
    setCreaseClass(m2, p2, inner, 'U');
    expect(validatePattern(m2, p2)).toEqual([{ kind: 'unassigned', creaseIds: [inner] }]);
  });

  it('removePattern removes the generated links and joints with the pattern', () => {
    const { m } = fresh();
    const p = miuraPattern(m, 'plane_top', 1, 1);
    generatePanels(m, p);
    expect(Object.keys(m.links).length).toBe(4);
    expect(removePattern(m, p.id)).toBe(true);
    expect(Object.keys(m.links).length).toBe(0);
    expect(Object.keys(m.joints).length).toBe(0);
    expect(Object.keys(m.points).length).toBe(0);
    expect(m.patterns![p.id]).toBeUndefined();
    expect(Object.keys(m.patterns!).length).toBe(1); // the empty pattern of fresh()
    expect(removePattern(m, p.id)).toBe(false);
  });
});

// ---------------------------------------------------------------------------
// Files
// ---------------------------------------------------------------------------

describe('files', () => {
  it('serializeModel / parseModel round-trip a model with patterns, and cloneModel copies them', () => {
    const { m } = fresh();
    const p = miuraPattern(m, 'plane_top', 1, 1);
    generatePanels(m, p);
    const json = serializeModel(m);
    expect(json).toContain('"patterns"');
    const back = parseModel(json);
    expect(back.patterns).toEqual(m.patterns);
    expect(back.patterns![p.id].generated!.linkIds).toEqual(p.generated!.linkIds);
    expect(serializeModel(back)).toBe(json);
    const clone = cloneModel(m);
    expect(clone.patterns).toEqual(m.patterns);
    clone.patterns![p.id].name = 'changed';
    expect(m.patterns![p.id].name).not.toBe('changed');
  });

  it('a model without patterns loads, and a pattern without its records is normalised', () => {
    const m = createModel();
    const json = serializeModel(m);
    expect(json).not.toContain('patterns');
    const back = parseModel(json);
    expect(back.patterns).toBeUndefined();
    expect(serializeModel(back)).toBe(json);
    const raw = JSON.parse(json) as Record<string, unknown>;
    raw.patterns = { pattern_1: { id: 'pattern_1', name: 'P', planeId: 'plane_top' } };
    const loaded = parseModel(JSON.stringify(raw));
    expect(loaded.patterns!.pattern_1.vertices).toEqual({});
    expect(loaded.patterns!.pattern_1.creases).toEqual({});
    expect(patternFaces(loaded, loaded.patterns!.pattern_1).faces).toEqual([]);
  });
});
