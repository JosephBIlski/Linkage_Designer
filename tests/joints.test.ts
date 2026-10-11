import { describe, expect, it } from 'vitest';
import {
  CREASE_EXACT_TOLERANCE,
  EDGE_LENGTH_TOLERANCE,
  addConstructionPoint,
  addCylinder,
  addJoint,
  addPolygon,
  addPolygonFromPoints,
  creaseReleasePoints,
  createModel,
  setGround,
} from '../src/core/model';
import { commitSketch, computeMobility, currentViolation, solveSketch, solveSketchWithRelease } from '../src/core/kinematics';
import { autoJoinCoincident } from '../src/core/edit';
import { tryAddJoint } from '../src/core/feasibility';
import { creaseDihedralDeg } from '../src/core/fold';
import { add, dist } from '../src/core/geometry';
import type { Feature, ID, Link, Model, Vec3 } from '../src/core/types';

const L = 2;
const H = 1.5;

interface Fixture {
  m: Model;
  A: Link;
  B: Link;
  edgeA: Feature;
  edgeB: Feature;
  before: Map<ID, Vec3>;
}

const edge = (link: Link): Feature => ({ linkId: link.id, kind: 'edge', pointIds: [link.pointIds[0], link.pointIds[1]] });
const snapshot = (m: Model): Map<ID, Vec3> => new Map(Object.values(m.points).map((p) => [p.id, [...p.pos] as Vec3]));
const maxMove = (m: Model, before: Map<ID, Vec3>, link: Link): number => Math.max(...link.pointIds.map((id) => dist(m.points[id].pos, before.get(id)!)));
const restLength = (link: Link, a: ID, b: ID): number => {
  const r = link.rigidity.find((c) => c.kind === 'dist' && ((c.a === a && c.b === b) || (c.a === b && c.b === a)));
  return r && r.kind === 'dist' ? r.length : NaN;
};

/**
 * Two triangles sharing the edge from (0,0,0) to (L,0,0): A above the x-axis (ground by default), B below it.
 * B's shared edge is `ratio` times as long, optionally listed in reversed order and translated by `shift`.
 */
function twoTriangles(opts: { ratio?: number; reversed?: boolean; shift?: Vec3; ground?: boolean } = {}): Fixture {
  const m = createModel();
  const A = addPolygonFromPoints(m, [[0, 0, 0], [L, 0, 0], [L / 2, H, 0]], { name: 'A' });
  const lb = L * (opts.ratio ?? 1);
  const s = opts.shift ?? [0, 0, 0];
  const b0 = add(s, [0, 0, 0]);
  const b1 = add(s, [lb, 0, 0]);
  const apex = add(s, [lb / 2, -H, 0]);
  const B = addPolygonFromPoints(m, opts.reversed ? [b1, b0, apex] : [b0, b1, apex], { name: 'B' });
  if (opts.ground ?? true) setGround(m, A.id);
  return { m, A, B, edgeA: edge(A), edgeB: edge(B), before: snapshot(m) };
}

describe('edge–edge revolute (crease) matching', () => {
  it('a) exactly equal edges in the same order merge into two straight pairs and nothing moves', () => {
    const { m, A, B, edgeA, edgeB, before } = twoTriangles();
    const j = addJoint(m, 'revolute', edgeA, edgeB)!;
    expect(j.pairs).toEqual([
      [A.pointIds[0], B.pointIds[0]],
      [A.pointIds[1], B.pointIds[1]],
    ]);
    expect(j.offsets).toBeUndefined();
    expect(creaseReleasePoints(m, j)).toEqual([]);
    expect(currentViolation(m)).toBeLessThan(1e-9);
    const res = solveSketch(m, {});
    expect(res.converged).toBe(true);
    commitSketch(m, res);
    expect(maxMove(m, before, A)).toBeLessThan(1e-12);
    expect(maxMove(m, before, B)).toBeLessThan(1e-12);
    expect(computeMobility(m).dof).toBe(1); // B hinges about the shared edge
  });

  it('b) exactly equal edges listed in reverse order merge as crossed pairs with zero residual', () => {
    const { m, A, B, edgeA, edgeB, before } = twoTriangles({ reversed: true });
    const j = addJoint(m, 'revolute', edgeA, edgeB)!;
    expect(j.pairs).toEqual([
      [A.pointIds[0], B.pointIds[1]],
      [A.pointIds[1], B.pointIds[0]],
    ]);
    expect(currentViolation(m)).toBeLessThan(1e-9);
    const res = solveSketch(m, {});
    commitSketch(m, res);
    expect(maxMove(m, before, A)).toBeLessThan(1e-12);
    expect(maxMove(m, before, B)).toBeLessThan(1e-12);
    expect(computeMobility(m).dof).toBe(1);
  });

  it('c) a 1e-5 relative mismatch with reversed order is merged and B rubber-bands instead of flipping', () => {
    const { m, A, B, edgeA, edgeB, before } = twoTriangles({ ratio: 1 + 1e-5, reversed: true });
    const rest0 = B.rigidity.map((r) => (r.kind === 'dist' ? { a: r.a, b: r.b, length: r.length } : null));
    const restBefore = (a: ID, b: ID): number => rest0.find((r) => r && ((r.a === a && r.b === b) || (r.a === b && r.b === a)))!.length;
    const j = addJoint(m, 'revolute', edgeA, edgeB)!;
    expect(j.pairs?.length).toBe(2);
    const free = new Set(creaseReleasePoints(m, j));
    expect([...free].sort()).toEqual([B.pointIds[0], B.pointIds[1]].sort()); // B adapts (A is ground)
    const res = solveSketch(m, { freePointIds: free });
    expect(res.converged).toBe(true);
    commitSketch(m, res, free);
    expect(currentViolation(m)).toBeLessThan(1e-9);
    expect(maxMove(m, before, A)).toBeLessThan(1e-12);
    // the merged pairs force B's edge to A's length, so B's released end point moves by exactly Δ = |la − lb| and
    // its apex by less (previously every vertex of B moved by ≈ 2: flipped across the axis)
    const delta = L * 1e-5;
    expect(maxMove(m, before, B)).toBeLessThanOrEqual(delta + 1e-9);
    expect(maxMove(m, before, B)).toBeGreaterThan(delta / 2);
    expect(dist(m.points[B.pointIds[2]].pos, before.get(B.pointIds[2])!)).toBeLessThan(delta);
    // B's rest geometry took A's edge length and no other rest distance changed by more than Δ
    expect(restLength(B, B.pointIds[0], B.pointIds[1])).toBeCloseTo(L, 9);
    for (const r of B.rigidity) {
      if (r.kind !== 'dist') continue;
      expect(Math.abs(r.length - restBefore(r.a, r.b))).toBeLessThanOrEqual(delta * (1 + 1e-6));
    }
    expect(computeMobility(m).dof).toBe(1);
  });

  it('a 1e-5 jitter on every vertex (hand-placed panels) merges, B adapts by about the jitter and keeps its shape', () => {
    let seed = 7;
    const rnd = () => ((seed = (seed * 1664525 + 1013904223) >>> 0) / 4294967296 - 0.5);
    const jit = (p: Vec3, e: number): Vec3 => [p[0] + e * rnd(), p[1] + e * rnd(), 0];
    const m = createModel();
    const A = addPolygonFromPoints(m, [jit([0, 0, 0], 1e-5), jit([L, 0, 0], 1e-5), jit([L / 2, H, 0], 1e-5)], { name: 'A' });
    const B = addPolygonFromPoints(m, [jit([0, 0, 0], 1e-5), jit([L, 0, 0], 1e-5), jit([L / 2, -H, 0], 1e-5)], { name: 'B' });
    setGround(m, A.id);
    const before = snapshot(m);
    const rest0 = B.rigidity.map((r) => (r.kind === 'dist' ? r.length : 0));
    const j = addJoint(m, 'revolute', edge(A), edge(B))!;
    expect(j.pairs?.length).toBe(2);
    const free = new Set(creaseReleasePoints(m, j));
    expect(free.size).toBe(2);
    const res = solveSketchWithRelease(m, free);
    expect(res.converged).toBe(true);
    expect(res.drift).toBeLessThan(3e-5);
    commitSketch(m, res, free);
    expect(currentViolation(m)).toBeLessThan(1e-9);
    expect(maxMove(m, before, A)).toBeLessThan(1e-12);
    expect(maxMove(m, before, B)).toBeLessThan(3e-5);
    B.rigidity.forEach((r, i) => r.kind === 'dist' && expect(Math.abs(r.length - rest0[i])).toBeLessThan(3e-5));
    expect(computeMobility(m).dof).toBe(1);
  });

  it('solveSketchWithRelease carries a displaced near-equal link rigidly before its edge adapts', () => {
    // B is drawn beside A (not on it) with an edge 0.05 % longer: a single released solve would leave its apex behind
    const { m, A, B, edgeA, edgeB } = twoTriangles({ ratio: 1 + 5e-4, shift: [0.4, -3, 0] });
    const apex0 = [...m.points[B.pointIds[2]].pos] as Vec3;
    const d0 = dist(apex0, m.points[B.pointIds[0]].pos);
    const d1 = dist(apex0, m.points[B.pointIds[1]].pos);
    const j = addJoint(m, 'revolute', edgeA, edgeB)!;
    expect(j.pairs?.length).toBe(2);
    const free = new Set(creaseReleasePoints(m, j));
    const res = solveSketchWithRelease(m, free);
    expect(res.converged).toBe(true);
    commitSketch(m, res, free);
    expect(currentViolation(m)).toBeLessThan(1e-9);
    expect(dist(m.points[B.pointIds[0]].pos, m.points[A.pointIds[0]].pos)).toBeLessThan(1e-9);
    expect(dist(m.points[B.pointIds[1]].pos, m.points[A.pointIds[1]].pos)).toBeLessThan(1e-9);
    // the apex came along: B keeps its shape up to the 0.05 % edge adjustment, on the near side of the axis
    const apex = m.points[B.pointIds[2]].pos;
    expect(apex[1]).toBeLessThan(0);
    expect(Math.abs(dist(apex, m.points[B.pointIds[0]].pos) - d0)).toBeLessThan(2e-3);
    expect(Math.abs(dist(apex, m.points[B.pointIds[1]].pos) - d1)).toBeLessThan(2e-3);
    expect(dist(apex, [L / 2, -H, 0])).toBeLessThan(2e-3);
  });

  it('d) a 5 % mismatch with reversed order: no pairs, B oriented like A, slide measured, nothing moves', () => {
    const { m, A, B, edgeA, edgeB, before } = twoTriangles({ ratio: 1.05, reversed: true });
    const j = addJoint(m, 'revolute', edgeA, edgeB)!;
    expect(j.pairs).toBeUndefined();
    // the stored feature is a reversed copy; the caller's object is untouched
    expect((j.b as Feature).pointIds).toEqual([B.pointIds[1], B.pointIds[0]]);
    expect(edgeB.pointIds).toEqual([B.pointIds[0], B.pointIds[1]]);
    expect(j.offsets?.slide).toBeCloseTo(0, 9);
    expect(creaseReleasePoints(m, j)).toEqual([]);
    expect(currentViolation(m)).toBeLessThan(1e-9);
    const res = solveSketch(m, {});
    expect(res.converged).toBe(true);
    commitSketch(m, res);
    expect(currentViolation(m)).toBeLessThan(1e-9);
    expect(maxMove(m, before, A)).toBeLessThan(1e-12);
    expect(dist(m.points[B.pointIds[2]].pos, before.get(B.pointIds[2])!)).toBeLessThan(1e-6);
    expect(computeMobility(m).dof).toBe(1); // collinear + slide lock = a hinge
  });

  it('e) a 5 % mismatch in the same order with B shifted 0.3 along the line keeps B where it is', () => {
    const { m, B, edgeA, edgeB, before } = twoTriangles({ ratio: 1.05, shift: [0.3, 0, 0] });
    const j = addJoint(m, 'revolute', edgeA, edgeB)!;
    expect(j.pairs).toBeUndefined();
    expect((j.b as Feature).pointIds).toEqual([B.pointIds[0], B.pointIds[1]]);
    expect(j.offsets?.slide).toBeCloseTo(0.3, 9);
    const res = solveSketch(m, {});
    expect(res.converged).toBe(true);
    commitSketch(m, res);
    expect(currentViolation(m)).toBeLessThan(1e-9);
    expect(dist(m.points[B.pointIds[2]].pos, before.get(B.pointIds[2])!)).toBeLessThan(1e-6);
    for (const id of [B.pointIds[0], B.pointIds[1]]) {
      expect(Math.abs(m.points[id].pos[1])).toBeLessThan(1e-9); // still on the shared line
      expect(Math.abs(m.points[id].pos[2])).toBeLessThan(1e-9);
    }
  });

  it('a mismatched edge placed off the line is pulled onto it and keeps its position along the axis', () => {
    const { m, B, edgeA, edgeB } = twoTriangles({ ratio: 1.05, shift: [0.3, 0.2, 0] });
    const j = addJoint(m, 'revolute', edgeA, edgeB)!;
    expect(j.offsets?.slide).toBeCloseTo(0.3, 9);
    const res = solveSketch(m, {});
    expect(res.converged).toBe(true);
    commitSketch(m, res);
    expect(currentViolation(m)).toBeLessThan(1e-9);
    expect(dist(m.points[B.pointIds[0]].pos, [0.3, 0, 0])).toBeLessThan(1e-6);
    expect(dist(m.points[B.pointIds[1]].pos, [0.3 + 1.05 * L, 0, 0])).toBeLessThan(1e-6);
    expect(dist(m.points[B.pointIds[2]].pos, [0.3 + (1.05 * L) / 2, -H, 0])).toBeLessThan(1e-6);
  });

  it('uses a relative tolerance of one part in a thousand that a joint option can override', () => {
    expect(EDGE_LENGTH_TOLERANCE).toBe(1e-3);
    expect(CREASE_EXACT_TOLERANCE).toBe(1e-9);
    const within = twoTriangles({ ratio: 1 + 0.9e-3 });
    expect(addJoint(within.m, 'revolute', within.edgeA, within.edgeB)!.pairs?.length).toBe(2);
    const beyond = twoTriangles({ ratio: 1 + 1.1e-3 });
    const jb = addJoint(beyond.m, 'revolute', beyond.edgeA, beyond.edgeB)!;
    expect(jb.pairs).toBeUndefined();
    expect(jb.offsets?.slide).toBeCloseTo(0, 9);
    const wide = twoTriangles({ ratio: 1.05 });
    expect(addJoint(wide.m, 'revolute', wide.edgeA, wide.edgeB, { lengthTolerance: 0.1 })!.pairs?.length).toBe(2);
    const narrow = twoTriangles({ ratio: 1 + 5e-4 });
    expect(addJoint(narrow.m, 'revolute', narrow.edgeA, narrow.edgeB, { lengthTolerance: 1e-4 })!.pairs).toBeUndefined();
  });

  it('cylinder axes behave like edges', () => {
    const m = createModel();
    const A = addPolygonFromPoints(m, [[0, 0, 0], [L, 0, 0], [L / 2, H, 0]]);
    setGround(m, A.id);
    // axis listed from the far end back to the origin, 5 % longer than A's edge
    const cyl = addCylinder(m, [1.05 * L, 0, 0], [0, 0, 0], 0.2);
    const before = snapshot(m);
    const axisFeature: Feature = { linkId: cyl.id, kind: 'axis', pointIds: [cyl.pointIds[0], cyl.pointIds[1]] };
    const j = addJoint(m, 'revolute', edge(A), axisFeature)!;
    expect(j.pairs).toBeUndefined();
    expect((j.b as Feature).kind).toBe('axis');
    expect((j.b as Feature).pointIds).toEqual([cyl.pointIds[1], cyl.pointIds[0]]);
    expect(j.offsets?.slide).toBeCloseTo(0, 9);
    const res = solveSketch(m, {});
    expect(res.converged).toBe(true);
    commitSketch(m, res);
    expect(maxMove(m, before, cyl)).toBeLessThan(1e-9);
    // an axis of exactly the same length merges like an edge
    const m2 = createModel();
    const A2 = addPolygonFromPoints(m2, [[0, 0, 0], [L, 0, 0], [L / 2, H, 0]]);
    const cyl2 = addCylinder(m2, [0, 0, 0], [L, 0, 0], 0.2);
    const j2 = addJoint(m2, 'revolute', edge(A2), { linkId: cyl2.id, kind: 'axis', pointIds: [cyl2.pointIds[0], cyl2.pointIds[1]] })!;
    expect(j2.pairs?.length).toBe(2);
  });

  it('cylindrical, prismatic and screw edge–edge joints keep sliding and are not re-oriented', () => {
    for (const type of ['cylindrical', 'prismatic', 'screw'] as const) {
      const { m, B, edgeA, edgeB } = twoTriangles({ ratio: 1.05, reversed: true });
      const j = addJoint(m, type, edgeA, edgeB)!;
      expect(j.pairs).toBeUndefined();
      expect((j.b as Feature).pointIds).toEqual([B.pointIds[0], B.pointIds[1]]);
      expect(creaseReleasePoints(m, j)).toEqual([]);
      if (type === 'cylindrical') expect(j.offsets).toBeUndefined();
      const res = solveSketch(m, {});
      expect(res.converged).toBe(true);
      commitSketch(m, res);
      expect(currentViolation(m)).toBeLessThan(1e-9);
      expect(computeMobility(m).dof).toBe(type === 'cylindrical' ? 2 : 1);
    }
  });
});

describe('a panel drawn elsewhere is placed beside its neighbour, not on top of it', () => {
  it('two Polygon-tool triangles drawn 4.5 units apart: the crease is created with the panels side by side (dihedral 180°), in 2-D and in 3-D', () => {
    for (const planar of [true, false]) {
      const m = createModel();
      const s = Math.sqrt(3);
      const P = (k: number): Vec3 => [s * Math.cos((Math.PI / 3) * k), s * Math.sin((Math.PI / 3) * k), 0];
      const mk = (k: number, off: Vec3, name: string): Link => {
        const c: Vec3 = [(P(k)[0] + P(k + 1)[0]) / 3 + off[0], (P(k)[1] + P(k + 1)[1]) / 3 + off[1], 0];
        return addPolygon(m, c, [0, 0, 1], 1, 3, { startAngle: Math.atan2(off[1] - c[1], off[0] - c[0]), onPlaneId: planar ? 'plane_top' : null, name });
      };
      const T1 = mk(0, [0, 0, 0], 'T1');
      const off: Vec3 = [4.5, 1.8, 0];
      const T2 = mk(1, off, 'T2');
      setGround(m, T1.id);
      const nearest = (link: Link, p: Vec3): ID => link.pointIds.reduce((b, id) => (dist(m.points[id].pos, p) < dist(m.points[b].pos, p) ? id : b), link.pointIds[0]);
      const r = tryAddJoint(m, 'revolute', { linkId: T1.id, kind: 'edge', pointIds: [nearest(T1, [0, 0, 0]), nearest(T1, P(1))] }, { linkId: T2.id, kind: 'edge', pointIds: [nearest(T2, off), nearest(T2, add(P(1), off))] });
      expect(r.ok).toBe(true);
      expect(r.joint?.pairs?.length).toBe(2);
      expect(Math.abs(creaseDihedralDeg(m, r.joint!)!)).toBeCloseTo(180, 6);
      expect(currentViolation(m)).toBeLessThan(1e-9);
      // T2 occupies the sector beside T1 (60°–120°), with its far corner at P(2), not T1's corners
      const far = T2.pointIds.map((id) => m.points[id].pos).find((p) => dist(p, [0, 0, 0]) > 1 && dist(p, P(1)) > 1)!;
      expect(dist(far, P(2))).toBeLessThan(1e-6);
      expect(computeMobility(m).dof).toBe(planar ? 0 : 1);
    }
  });
});

describe('creaseReleasePoints', () => {
  it('lets the link of feature b adapt unless it is ground or locked, then the link of a, else nobody', () => {
    const free = twoTriangles({ ratio: 1 + 1e-5, ground: false });
    const j = addJoint(free.m, 'revolute', free.edgeA, free.edgeB)!;
    expect(creaseReleasePoints(free.m, j)).toEqual([free.B.pointIds[0], free.B.pointIds[1]]);
    setGround(free.m, free.B.id);
    expect(creaseReleasePoints(free.m, j)).toEqual([free.A.pointIds[0], free.A.pointIds[1]]);
    free.A.locked = true;
    expect(creaseReleasePoints(free.m, j)).toEqual([]);
  });

  it('prefers the named link when it may move', () => {
    const { m, A, B, edgeA, edgeB } = twoTriangles({ ratio: 1 + 1e-5, ground: false });
    const j = addJoint(m, 'revolute', edgeA, edgeB)!;
    expect(creaseReleasePoints(m, j, A.id)).toEqual([A.pointIds[0], A.pointIds[1]]);
    expect(creaseReleasePoints(m, j, B.id)).toEqual([B.pointIds[0], B.pointIds[1]]);
    A.locked = true;
    expect(creaseReleasePoints(m, j, A.id)).toEqual([B.pointIds[0], B.pointIds[1]]);
  });

  it('returns nothing for identical lengths, unmerged edges and vertex pins', () => {
    const same = twoTriangles({ ground: false });
    expect(creaseReleasePoints(same.m, addJoint(same.m, 'revolute', same.edgeA, same.edgeB)!)).toEqual([]);
    const far = twoTriangles({ ratio: 1.05, ground: false });
    expect(creaseReleasePoints(far.m, addJoint(far.m, 'revolute', far.edgeA, far.edgeB)!)).toEqual([]);
    const pin = twoTriangles({ ground: false });
    const jp = addJoint(pin.m, 'revolute', { linkId: pin.A.id, kind: 'vertex', pointIds: [pin.A.pointIds[0]] }, { linkId: pin.B.id, kind: 'vertex', pointIds: [pin.B.pointIds[0]] })!;
    expect(jp.pairs?.length).toBe(2);
    expect(creaseReleasePoints(pin.m, jp)).toEqual([]);
  });
});

describe('commitSketch never bakes a violated pose', () => {
  it('leaves the rest geometry alone when the released solve did not converge', () => {
    const m = createModel();
    const tri = addPolygonFromPoints(m, [[0, 0, 0], [2, 0, 0], [1, 1, 0]]);
    // V0 pinned 3 units from the RIGHT plane that V1 must stay on: the 2-unit edge cannot span it
    const pnt = addConstructionPoint(m, [3, 0, 0]);
    addJoint(m, 'spherical', { linkId: tri.id, kind: 'vertex', pointIds: [tri.pointIds[0]] }, { constructionId: pnt.id });
    addJoint(m, 'planar', { linkId: tri.id, kind: 'vertex', pointIds: [tri.pointIds[1]] }, { constructionId: 'plane_right' });
    const rigidity = JSON.stringify(tri.rigidity);
    const free = new Set([tri.pointIds[2]]);
    const res = solveSketch(m, { freePointIds: free });
    expect(res.converged).toBe(false);
    commitSketch(m, res, free);
    expect(JSON.stringify(tri.rigidity)).toBe(rigidity);
    expect(currentViolation(m)).toBeGreaterThan(0.1); // the violation stays visible instead of becoming the design
  });
});

describe('autoJoinCoincident with edges that coincide only within the snap tolerance', () => {
  it('creates one crease and lets the edited link adapt its own edge', () => {
    const m = createModel();
    const A = addPolygonFromPoints(m, [[0, 0, 0], [L, 0, 0], [L / 2, H, 0]], { name: 'A' });
    const B = addPolygonFromPoints(m, [[0, 0, 0], [L + 2e-7, 0, 0], [L / 2, -H, 0]], { name: 'B' });
    const restA = JSON.stringify(A.rigidity);
    const created = autoJoinCoincident(m, B, B.pointIds, { defaultJoint: 'revolute', axis: [0, 0, 1] }).joints;
    expect(created.length).toBe(1);
    expect(created[0].type).toBe('revolute');
    expect(created[0].pairs?.length).toBe(2);
    expect(currentViolation(m)).toBeLessThan(1e-9);
    expect(JSON.stringify(A.rigidity)).toBe(restA); // the existing panel keeps its design
    expect(restLength(B, B.pointIds[0], B.pointIds[1])).toBeCloseTo(L, 9); // the sketched panel took A's edge length
    expect(dist(m.points[B.pointIds[1]].pos, m.points[A.pointIds[1]].pos)).toBeLessThan(1e-9);
  });
});

describe('four regular triangles joined edge to edge (Polygon-tool geometry)', () => {
  it('the first three joints are exact merges that move nothing', () => {
    const m = createModel();
    const s = Math.sqrt(3); // side of a regular triangle with circumradius 1
    const P = (k: number): Vec3 => [s * Math.cos((Math.PI / 3) * k), s * Math.sin((Math.PI / 3) * k), 0];
    const O: Vec3 = [0, 0, 0];
    const tris: Link[] = [];
    for (let k = 0; k < 4; k++) {
      const c: Vec3 = [(P(k)[0] + P(k + 1)[0]) / 3, (P(k)[1] + P(k + 1)[1]) / 3, 0];
      tris.push(addPolygon(m, c, [0, 0, 1], 1, 3, { startAngle: Math.atan2(O[1] - c[1], O[0] - c[0]) }));
    }
    const nearest = (link: Link, p: Vec3): ID => link.pointIds.reduce((b, id) => (dist(m.points[id].pos, p) < dist(m.points[b].pos, p) ? id : b), link.pointIds[0]);
    setGround(m, tris[0].id);
    for (let k = 0; k < 3; k++) {
      const a = tris[k];
      const b = tris[k + 1];
      const before = snapshot(m);
      const j = addJoint(m, 'revolute', { linkId: a.id, kind: 'edge', pointIds: [nearest(a, O), nearest(a, P(k + 1))] }, { linkId: b.id, kind: 'edge', pointIds: [nearest(b, O), nearest(b, P(k + 1))] })!;
      expect(j.pairs?.length).toBe(2);
      const free = new Set(creaseReleasePoints(m, j));
      expect(free.size).toBe(0); // Polygon-tool edges agree to floating-point precision
      const res = solveSketchWithRelease(m, free);
      expect(res.converged).toBe(true);
      commitSketch(m, res, free);
      expect(currentViolation(m)).toBeLessThan(1e-9);
      expect(maxMove(m, before, b)).toBeLessThan(1e-9);
    }
    expect(computeMobility(m).dof).toBe(3); // an open chain of three creases
  });
});
