import { describe, expect, it } from 'vitest';
import {
  addBar,
  addConstructionAxis,
  addJoint,
  addPolygon,
  addPolygonFromPoints,
  addPrism,
  bodyPlaneJoint,
  createModel,
  serializeModel,
  setGround,
} from '../src/core/model';
import {
  SECTOR_SUM_TOLERANCE_DEG,
  adaptationLimit,
  closesWithout2d,
  diagnoseJoint,
  feasibilityTolerance,
  interiorAngleDeg,
  isAccepted,
  isReleaseAccepted,
  linkPath,
  sectorSumAt,
  tryAddJoint,
  tryChangeJointType,
  trySolveCommit,
  unabsorbedCreaseLengths,
  unequalEdgeLengths,
} from '../src/core/feasibility';
import { CREASE_ADAPTATION_TOLERANCE, creaseReleasePoints } from '../src/core/model';
import { autoJoinCoincident } from '../src/core/edit';
import { computeMobility, currentViolation, modelSize, releaseDrift, solveSketch, solveSketchWithRelease } from '../src/core/kinematics';
import { dist } from '../src/core/geometry';
import type { Feature, ID, Link, Model, Vec3 } from '../src/core/types';

/** Edge feature of `link` whose end points are nearest to p and q. */
function edgeNear(m: Model, link: Link, p: Vec3, q: Vec3): Feature {
  const nearest = (x: Vec3): ID => link.pointIds.reduce((b, id) => (dist(m.points[id].pos, x) < dist(m.points[b].pos, x) ? id : b), link.pointIds[0]);
  return { linkId: link.id, kind: 'edge', pointIds: [nearest(p), nearest(q)] };
}

const vertex = (link: Link, i: number): Feature => ({ linkId: link.id, kind: 'vertex', pointIds: [link.pointIds[i]] });
const positions = (m: Model): Map<ID, Vec3> => new Map(Object.values(m.points).map((p) => [p.id, [...p.pos] as Vec3]));
const maxMove = (m: Model, before: Map<ID, Vec3>): number => Math.max(...Object.values(m.points).map((p) => dist(p.pos, before.get(p.id)!)));

const SIDE = Math.sqrt(3); // side of a regular triangle with circumradius 1
const O: Vec3 = [0, 0, 0];
const corner = (k: number): Vec3 => [SIDE * Math.cos((Math.PI / 3) * k), SIDE * Math.sin((Math.PI / 3) * k), 0];

/** Deterministic pseudo-random offsets in [−0.5, 0.5) (a linear congruential generator) for hand-placed geometry. */
function lcg(seed = 1): () => number {
  let s = seed;
  return () => ((s = (s * 1664525 + 1013904223) >>> 0) / 4294967296 - 0.5);
}

/** Point offset by ±jitter/2 per in-plane coordinate (0 leaves it alone). */
const jitterer = (jitter: number | undefined, r: () => number) => (p: Vec3): Vec3 => (jitter ? [p[0] + jitter * r(), p[1] + jitter * r(), p[2]] : p);

/**
 * Four regular triangles around the origin, consecutive ones sharing the edge O–corner(k+1); T1 is ground. By
 * default as the Polygon tool draws them in 2-D mode (addPolygon, circumradius 1, flat in XY, body planar joint to
 * TOP), whose shared edges agree to floating-point precision. `radii` gives each triangle its own circumradius (the
 * mouse-placed Polygon tool: the clicked circumcircle point is never exactly on the circle); `jitter` builds them
 * from points offset by ±jitter/2 per coordinate (typed or snapped vertices that agree to a snap distance only).
 */
function fourRegularTriangles(opts: { planar?: boolean; jitter?: number; radii?: number[] } = {}): { m: Model; tris: Link[] } {
  const m = createModel();
  const tris: Link[] = [];
  const onPlaneId = opts.planar === false ? null : 'plane_top';
  const jit = jitterer(opts.jitter, lcg());
  for (let k = 0; k < 4; k++) {
    if (opts.jitter) {
      tris.push(addPolygonFromPoints(m, [jit(O), jit(corner(k)), jit(corner(k + 1))], { onPlaneId, name: `T${k + 1}` }));
      continue;
    }
    const c: Vec3 = [(corner(k)[0] + corner(k + 1)[0]) / 3, (corner(k)[1] + corner(k + 1)[1]) / 3, 0];
    tris.push(addPolygon(m, c, [0, 0, 1], opts.radii?.[k] ?? 1, 3, { startAngle: Math.atan2(O[1] - c[1], O[0] - c[0]), onPlaneId, name: `T${k + 1}` }));
  }
  setGround(m, tris[0].id);
  return { m, tris };
}

/** Four developable triangles O–p_i–p_{i+1} with sectors 60°, 60°, 120°, 120° on a circle of radius 2, flat, 2-D (`jitter` as above). */
function fourDevelopableTriangles(opts: { jitter?: number } = {}): { m: Model; tris: Link[]; p: (i: number) => Vec3 } {
  const m = createModel();
  const ang = [0, 60, 120, 240, 360].map((d) => (d * Math.PI) / 180);
  const p = (i: number): Vec3 => [2 * Math.cos(ang[i]), 2 * Math.sin(ang[i]), 0];
  const jit = jitterer(opts.jitter, lcg());
  const tris = [0, 1, 2, 3].map((i) => addPolygonFromPoints(m, [jit(O), jit(p(i)), jit(p(i + 1))], { onPlaneId: 'plane_top', name: `D${i + 1}` }));
  setGround(m, tris[0].id);
  return { m, tris, p };
}

/** Three unit squares in a row on TOP (S0 ground), which can only close into a triangular tube out of the plane (`jitter` as above). */
function threeSquares(opts: { jitter?: number } = {}): { m: Model; sq: Link[] } {
  const m = createModel();
  const jit = jitterer(opts.jitter, lcg());
  const sq: Link[] = [];
  for (let i = 0; i < 3; i++) sq.push(addPolygonFromPoints(m, [jit([i, 0, 0]), jit([i + 1, 0, 0]), jit([i + 1, 1, 0]), jit([i, 1, 0])], { onPlaneId: 'plane_top', name: `S${i}` }));
  setGround(m, sq[0].id);
  return { m, sq };
}

/** Largest change of an interior angle (degrees) and of a rest distance of `tris` since `before` was taken. */
function shapeChange(m: Model, tris: Link[], before: { angles: number[][]; rest: number[][] }): { angle: number; rest: number } {
  let angle = 0;
  let rest = 0;
  tris.forEach((t, k) => {
    const link = m.links[t.id];
    link.pointIds.forEach((id, i) => (angle = Math.max(angle, Math.abs(interiorAngleDeg(m, link, id)! - before.angles[k][i]))));
    link.rigidity.filter((c) => c.kind === 'dist').forEach((c, i) => (rest = Math.max(rest, Math.abs((c.kind === 'dist' ? c.length : 0) - before.rest[k][i]))));
  });
  return { angle, rest };
}
const shapeOf = (m: Model, tris: Link[]): { angles: number[][]; rest: number[][] } => ({
  angles: tris.map((t) => t.pointIds.map((id) => interiorAngleDeg(m, m.links[t.id], id)!)),
  rest: tris.map((t) => m.links[t.id].rigidity.filter((c) => c.kind === 'dist').map((c) => (c.kind === 'dist' ? c.length : 0))),
});

/** Planar four-bar on TOP with revolute pins; returns the coupler so a test can change its length. */
function fourBar(): { m: Model; coupler: Link } {
  const m = createModel();
  const g = addBar(m, [0, 0, 0], [4, 0, 0], { onPlaneId: 'plane_top', name: 'ground' });
  setGround(m, g.id);
  const crank = addBar(m, [0, 0, 0], [0, 2, 0], { onPlaneId: 'plane_top', name: 'crank' });
  const coupler = addBar(m, [0, 2, 0], [4, 3, 0], { onPlaneId: 'plane_top', name: 'coupler' });
  const rocker = addBar(m, [4, 3, 0], [4, 0, 0], { onPlaneId: 'plane_top', name: 'rocker' });
  const pin = (x: Link, xi: number, y: Link, yi: number) => addJoint(m, 'revolute', vertex(x, xi), vertex(y, yi), { axis: [0, 0, 1] });
  pin(g, 0, crank, 0);
  pin(crank, 1, coupler, 0);
  pin(coupler, 1, rocker, 0);
  pin(rocker, 1, g, 1);
  return { m, coupler };
}

describe('feasibility tolerance and acceptance', () => {
  it('scales with the model size and never drops below 1e-9', () => {
    const m = createModel();
    expect(feasibilityTolerance(m)).toBe(1e-7); // empty model counts as size 1
    addBar(m, [0, 0, 0], [1000, 0, 0]);
    expect(feasibilityTolerance(m)).toBeCloseTo(1e-7 * modelSize(m), 12);
    const tiny = createModel();
    addBar(tiny, [0, 0, 0], [1e-4, 0, 0]);
    expect(feasibilityTolerance(tiny)).toBe(1e-9);
  });

  it('accepts a converged result or a residual within the tolerance', () => {
    const m = createModel();
    addBar(m, [0, 0, 0], [1, 0, 0]);
    const res = solveSketch(m, {});
    expect(isAccepted(res, feasibilityTolerance(m))).toBe(true);
    expect(isAccepted({ ...res, converged: false, residual: 5e-8 }, 1e-7)).toBe(true);
    expect(isAccepted({ ...res, converged: false, residual: 2e-7 }, 1e-7)).toBe(false);
    expect(isAccepted({ ...res, converged: true, residual: 2e-7 }, 1e-7)).toBe(true);
  });
});

describe('tryAddJoint: four regular triangles (the reported workflow)', () => {
  it('a) creates the first three creases and refuses the fourth with the sector-angle diagnosis, restoring the model', () => {
    const { m, tris } = fourRegularTriangles();
    for (let k = 0; k < 3; k++) {
      const before = positions(m);
      const r = tryAddJoint(m, 'revolute', edgeNear(m, tris[k], O, corner(k + 1)), edgeNear(m, tris[k + 1], O, corner(k + 1)));
      expect(r.ok).toBe(true);
      expect(r.joint?.pairs?.length).toBe(2);
      expect(r.residual).toBeLessThan(1e-12);
      expect(maxMove(m, before)).toBeLessThan(1e-9); // Polygon-tool edges agree exactly: nothing moves
    }
    expect(Object.values(m.joints).filter((j) => j.a.kind !== 'body').length).toBe(3);
    const snapshot = serializeModel(m);
    const before = positions(m);
    const r = tryAddJoint(m, 'revolute', edgeNear(m, tris[3], O, corner(4)), edgeNear(m, tris[0], O, corner(0)));
    expect(r.ok).toBe(false);
    expect(r.joint).toBeUndefined();
    expect(r.residual).toBeGreaterThan(0.1);
    expect(r.diagnosis?.kind).toBe('sectorSum');
    if (r.diagnosis?.kind === 'sectorSum') {
      expect(Math.abs(r.diagnosis.sumDeg - 240)).toBeLessThanOrEqual(0.5);
      expect(r.diagnosis.constrained2d).toBe(true);
      expect(r.diagnosis.linkIds.sort()).toEqual(tris.map((t) => t.id).sort());
      expect(m.points[r.diagnosis.vertexPointId]?.name).toBe('V0'); // the vertex on the first-picked link (T4)
      expect(m.points[r.diagnosis.vertexPointId]?.linkId).toBe(tris[3].id);
      expect(r.diagnosis.vertexName).toBe('T4 V0');
    }
    expect(serializeModel(m)).toBe(snapshot); // byte-identical: no joint, no helper, no moved vertex
    expect(maxMove(m, before)).toBe(0);
    expect(Object.values(m.joints).filter((j) => j.a.kind !== 'body').length).toBe(3);
    expect(currentViolation(m)).toBeLessThan(1e-9);
  });

  it('reports constrained2d = false when the panels are free in 3-D', () => {
    const { m, tris } = fourRegularTriangles({ planar: false });
    for (let k = 0; k < 3; k++) expect(tryAddJoint(m, 'revolute', edgeNear(m, tris[k], O, corner(k + 1)), edgeNear(m, tris[k + 1], O, corner(k + 1))).ok).toBe(true);
    const r = tryAddJoint(m, 'revolute', edgeNear(m, tris[3], O, corner(4)), edgeNear(m, tris[0], O, corner(0)));
    expect(r.ok).toBe(false);
    expect(r.diagnosis).toMatchObject({ kind: 'sectorSum', sumDeg: 240, constrained2d: false });
  });

  // Hand-placed geometry: the shared edges agree only to a snap distance, so joints 1–3 are merged creases whose
  // adapting panel takes its neighbour's edge length (a rubber-band of the order of the jitter), and the fourth joint
  // must still be refused: a released solve could "close" the 240° loop by reshaping T4 into a line, which the
  // adaptation limit forbids, and the diagnosis is made from the rigid solve.
  for (const jitter of [1e-7, 1e-5, 5e-4]) {
    for (const planar of [true, false]) {
      it(`hand-placed (jitter ${jitter}, ${planar ? '2-D' : '3-D'}): joints 1–3 merge with a snap-sized adaptation, the fourth is refused with the sector diagnosis and the model restored`, () => {
        const { m, tris } = fourRegularTriangles({ planar, jitter });
        const shape0 = shapeOf(m, tris);
        for (let k = 0; k < 3; k++) {
          const r = tryAddJoint(m, 'revolute', edgeNear(m, tris[k], O, corner(k + 1)), edgeNear(m, tris[k + 1], O, corner(k + 1)));
          expect(r.ok).toBe(true);
          expect(r.joint?.pairs?.length).toBe(2);
        }
        const change = shapeChange(m, tris, shape0);
        expect(change.angle).toBeLessThan(0.05);
        expect(change.rest).toBeLessThanOrEqual(2 * jitter + 1e-12);
        expect(currentViolation(m)).toBeLessThan(1e-8);
        const snapshot = serializeModel(m);
        const r = tryAddJoint(m, 'revolute', edgeNear(m, tris[3], O, corner(4)), edgeNear(m, tris[0], O, corner(0)));
        expect(r.ok).toBe(false);
        expect(r.diagnosis).toMatchObject({ kind: 'sectorSum', sumDeg: 240, constrained2d: planar });
        expect(r.residual).toBeGreaterThan(0.1); // the rigid solve's gap, not the collapsed panel's zero
        expect(serializeModel(m)).toBe(snapshot);
        expect(shapeChange(m, tris, shape0).angle).toBeLessThan(0.05); // T4 was not collapsed into a line (180°, 0°, 0°)
      });
    }
  }

  it('mouse-placed Polygon-tool triangles (circumradii off by 1e-7 … 4e-4) are refused like the exact ones, with T4 intact', () => {
    for (const radii of [[1.000000047, 0.99999967, 0.99999951, 0.99999979], [1, 1.0003, 0.9997, 1.0004]]) {
      const { m, tris } = fourRegularTriangles({ radii });
      const shape0 = shapeOf(m, tris);
      for (let k = 0; k < 3; k++) expect(tryAddJoint(m, 'revolute', edgeNear(m, tris[k], O, corner(k + 1)), edgeNear(m, tris[k + 1], O, corner(k + 1))).ok).toBe(true);
      const snapshot = serializeModel(m);
      const r = tryAddJoint(m, 'revolute', edgeNear(m, tris[3], O, corner(4)), edgeNear(m, tris[0], O, corner(0)));
      expect(r.ok).toBe(false);
      expect(r.diagnosis).toMatchObject({ kind: 'sectorSum', sumDeg: 240, vertexName: 'T4 V0', constrained2d: true });
      expect(serializeModel(m)).toBe(snapshot);
      expect(shapeChange(m, tris, shape0).angle).toBeLessThan(0.05); // T4 kept its 60° corners
      expect(computeMobility(m).dof).toBe(0);
    }
  });
});

describe('crease adaptation limit', () => {
  it('bounds the adapting link\'s shape change by twice the merge tolerance of the longer edge, plus the solve tolerance', () => {
    const m = createModel();
    const A = addPolygonFromPoints(m, [[0, 0, 0], [2, 0, 0], [1, 1.5, 0]], { name: 'A' });
    setGround(m, A.id);
    const B = addPolygonFromPoints(m, [[0, 0, 0], [2.0005, 0, 0], [1.00025, -1.5, 0]], { name: 'B' });
    const j = addJoint(m, 'revolute', edgeNear(m, A, [0, 0, 0], [2, 0, 0]), edgeNear(m, B, [0, 0, 0], [2.0005, 0, 0]))!;
    expect(j.pairs?.length).toBe(2);
    const limit = adaptationLimit(m, [j]);
    expect(limit).toBeCloseTo(CREASE_ADAPTATION_TOLERANCE * 2.0005 + feasibilityTolerance(m), 12);
    expect(CREASE_ADAPTATION_TOLERANCE).toBe(2e-3);
    const free = new Set(creaseReleasePoints(m, j));
    const res = solveSketchWithRelease(m, free);
    expect(res.rigid.converged).toBe(false); // B's edge rest length disagrees with the merged end points by 5e-4
    expect(res.drift).toBeCloseTo(0.0005, 6); // B's edge takes A's length, nothing else changes
    expect(releaseDrift(m, free, res.positions)).toBe(res.drift);
    expect(releaseDrift(m, new Set(), res.positions)).toBe(0);
    expect(isReleaseAccepted(m, res, limit)).toBe(true);
    expect(isReleaseAccepted(m, { ...res, drift: limit * 1.01 }, limit)).toBe(false);
    expect(isReleaseAccepted(m, { ...res, converged: false, residual: 1 }, limit)).toBe(false);
    // without released points the result is the rigid solve itself
    const plain = solveSketchWithRelease(m, new Set());
    expect(plain.drift).toBe(0);
    expect(plain.rigid.residual).toBe(plain.residual);
  });
});

describe('tryAddJoint: hand-placed loops that do close', () => {
  it('developable triangles drawn with a 1e-5 jitter: all four joints accepted, every panel keeps its shape to within the jitter', () => {
    const { m, tris, p } = fourDevelopableTriangles({ jitter: 1e-5 });
    const shape0 = shapeOf(m, tris);
    for (let i = 0; i < 4; i++) {
      const r = tryAddJoint(m, 'revolute', edgeNear(m, tris[i], O, p(i + 1)), edgeNear(m, tris[(i + 1) % 4], O, p(i + 1)));
      expect(r.ok).toBe(true);
      expect(r.joint?.pairs?.length).toBe(2);
    }
    const change = shapeChange(m, tris, shape0);
    expect(change.angle).toBeLessThan(0.05);
    expect(change.rest).toBeLessThan(1e-4);
    expect(currentViolation(m)).toBeLessThan(1e-8);
    expect(computeMobility(m).dof).toBe(0);
  });

  for (const jitter of [1e-5, 1e-4, 5e-4]) {
    it(`three squares drawn with a ${jitter} jitter still report needs3d for the closing edge (the tube closes with rigid panels once the 2-D constraint is gone)`, () => {
      const { m, sq } = threeSquares({ jitter });
      for (let i = 0; i < 2; i++) expect(tryAddJoint(m, 'revolute', edgeNear(m, sq[i], [i + 1, 0, 0], [i + 1, 1, 0]), edgeNear(m, sq[i + 1], [i + 1, 0, 0], [i + 1, 1, 0])).ok).toBe(true);
      const snapshot = serializeModel(m);
      const r = tryAddJoint(m, 'revolute', edgeNear(m, sq[2], [3, 0, 0], [3, 1, 0]), edgeNear(m, sq[0], [0, 0, 0], [0, 1, 0]));
      expect(r.ok).toBe(false);
      expect(r.diagnosis?.kind).toBe('needs3d');
      expect(serializeModel(m)).toBe(snapshot);
    });
  }
});

describe('tryAddJoint: four developable triangles (60°, 60°, 120°, 120°)', () => {
  it('b) closes the flat loop with zero residual at every joint', () => {
    const { m, tris, p } = fourDevelopableTriangles();
    for (let i = 0; i < 4; i++) {
      const r = tryAddJoint(m, 'revolute', edgeNear(m, tris[i], O, p(i + 1)), edgeNear(m, tris[(i + 1) % 4], O, p(i + 1)));
      expect(r.ok).toBe(true);
      expect(r.residual).toBeLessThan(1e-12);
    }
    expect(Object.values(m.joints).filter((j) => j.type === 'revolute').length).toBe(4);
    expect(currentViolation(m)).toBeLessThan(1e-12);
    // the sector sum at the shared vertex is 360°, so no diagnosis would fire
    const j = Object.values(m.joints).find((x) => x.type === 'revolute')!;
    const s = sectorSumAt(m, j)!;
    expect(s.sumDeg).toBe(360);
    expect(s.constrained2d).toBe(true);
    expect(computeMobility(m).dof).toBe(0); // the 2-D constraint freezes the in-plane creases (plan §2, item 4)
  });
});

describe('tryAddJoint: other diagnoses', () => {
  it('c) an incompatible pair (face ↔ vertex revolute) leaves the model unchanged', () => {
    const { m, tris } = fourRegularTriangles();
    const snapshot = serializeModel(m);
    const face: Feature = { linkId: tris[0].id, kind: 'face', pointIds: [...tris[0].pointIds] };
    const r = tryAddJoint(m, 'revolute', face, vertex(tris[1], 0));
    expect(r.ok).toBe(false);
    expect(r.diagnosis).toEqual({ kind: 'incompatible' });
    expect(serializeModel(m)).toBe(snapshot);
  });

  it('needs3d: an edge that must lie on a tilted datum axis while its panel is held on the sketch plane', () => {
    const m = createModel();
    const poly = addPolygonFromPoints(m, [[0, 0, 0], [2, 0, 0], [1, 1, 0]], { onPlaneId: 'plane_top' });
    const axis = addConstructionAxis(m, [0, 0, 0], [1, 0, 1]);
    const snapshot = serializeModel(m);
    const r = tryAddJoint(m, 'revolute', edgeNear(m, poly, [0, 0, 0], [2, 0, 0]), { constructionId: axis.id });
    expect(r.ok).toBe(false);
    expect(r.diagnosis).toEqual({ kind: 'needs3d', linkIds: [poly.id] });
    expect(serializeModel(m)).toBe(snapshot);
    expect(bodyPlaneJoint(m, poly.id)).not.toBeNull();
    // without the 2-D constraint the same joint is created
    const m2 = createModel();
    const poly2 = addPolygonFromPoints(m2, [[0, 0, 0], [2, 0, 0], [1, 1, 0]]);
    const axis2 = addConstructionAxis(m2, [0, 0, 0], [1, 0, 1]);
    expect(tryAddJoint(m2, 'revolute', edgeNear(m2, poly2, [0, 0, 0], [2, 0, 0]), { constructionId: axis2.id }).ok).toBe(true);
    expect(currentViolation(m2)).toBeLessThan(1e-9);
  });

  it('needs3d: three squares in a row can only close into a tube out of the sketch plane', () => {
    const { m, sq } = threeSquares();
    for (let i = 0; i < 2; i++) expect(tryAddJoint(m, 'revolute', edgeNear(m, sq[i], [i + 1, 0, 0], [i + 1, 1, 0]), edgeNear(m, sq[i + 1], [i + 1, 0, 0], [i + 1, 1, 0])).ok).toBe(true);
    const snapshot = serializeModel(m);
    const r = tryAddJoint(m, 'revolute', edgeNear(m, sq[2], [3, 0, 0], [3, 1, 0]), edgeNear(m, sq[0], [0, 0, 0], [0, 1, 0]));
    expect(r.ok).toBe(false);
    expect(r.diagnosis?.kind).toBe('needs3d'); // no central vertex (each edge touches two squares), so not a sector sum
    if (r.diagnosis?.kind === 'needs3d') expect(r.diagnosis.linkIds.sort()).toEqual(sq.map((s) => s.id).sort());
    expect(serializeModel(m)).toBe(snapshot);
  });

  it('edgeLengths: a hinge between edges of different length that cannot be satisfied names the lengths', () => {
    // A (ground) and B share the x-axis edge; B's shared edge is 10 % longer and B's other end is pinned where it
    // stands, so B can neither slide onto A's line nor adapt
    const m = createModel();
    const A = addPolygonFromPoints(m, [[0, 0, 0], [2, 0, 0], [1, 1.5, 0]], { name: 'A' });
    setGround(m, A.id);
    const B = addPolygonFromPoints(m, [[0, 0.3, 0], [2.2, 0.3, 0], [1.1, -1.5, 0]], { name: 'B' });
    addJoint(m, 'spherical', vertex(B, 2), { constructionId: 'point_origin' });
    const snapshot = serializeModel(m);
    const r = tryAddJoint(m, 'revolute', edgeNear(m, A, [0, 0, 0], [2, 0, 0]), edgeNear(m, B, [0, 0.3, 0], [2.2, 0.3, 0]));
    expect(r.ok).toBe(false);
    expect(r.diagnosis?.kind).toBe('edgeLengths');
    if (r.diagnosis?.kind === 'edgeLengths') {
      expect(r.diagnosis.la).toBeCloseTo(2, 9);
      expect(r.diagnosis.lb).toBeCloseTo(2.2, 9);
    }
    expect(serializeModel(m)).toBe(snapshot);
  });

  it('edgeLengths: a merged crease whose small mismatch neither panel may absorb (ground + locked) is refused instead of hiding the conflict', () => {
    for (const groundA of [true, false]) {
      const m = createModel();
      const A = addPolygonFromPoints(m, [[0, 0, 0], [2, 0, 0], [1, 1.5, 0]], { name: 'A' });
      const B = addPolygonFromPoints(m, [[0, 0, 0], [2.0005, 0, 0], [1.00025, -1.5, 0]], { name: 'B' });
      // ground A + locked B, or locked A + ground B: the 2.5e-4 relative mismatch merges, but no link may adapt
      setGround(m, groundA ? A.id : B.id);
      (groundA ? B : A).locked = true;
      const snapshot = serializeModel(m);
      const r = tryAddJoint(m, 'revolute', edgeNear(m, A, [0, 0, 0], [2, 0, 0]), edgeNear(m, B, [0, 0, 0], [2.0005, 0, 0]));
      expect(r.ok).toBe(false);
      expect(r.diagnosis).toMatchObject({ kind: 'edgeLengths' });
      if (r.diagnosis?.kind === 'edgeLengths') {
        expect(r.diagnosis.la).toBeCloseTo(2, 9);
        expect(r.diagnosis.lb).toBeCloseTo(2.0005, 9);
      }
      expect(r.residual).toBeCloseTo(5e-4, 6); // the rest length of the locked edge against the merged (frozen) end points
      expect(serializeModel(m)).toBe(snapshot);
      const j = addJoint(m, 'revolute', edgeNear(m, A, [0, 0, 0], [2, 0, 0]), edgeNear(m, B, [0, 0, 0], [2.0005, 0, 0]))!;
      expect(creaseReleasePoints(m, j)).toEqual([]);
      expect(unabsorbedCreaseLengths(m, j)).toEqual({ la: 2, lb: 2.0005 });
      expect(unequalEdgeLengths(m, j)).toBeNull(); // within the merge tolerance: it is the locking that forbids it
    }
    // control: with B free to adapt the same crease is accepted and B's edge takes A's length
    const m = createModel();
    const A = addPolygonFromPoints(m, [[0, 0, 0], [2, 0, 0], [1, 1.5, 0]], { name: 'A' });
    const B = addPolygonFromPoints(m, [[0, 0, 0], [2.0005, 0, 0], [1.00025, -1.5, 0]], { name: 'B' });
    setGround(m, A.id);
    expect(tryAddJoint(m, 'revolute', edgeNear(m, A, [0, 0, 0], [2, 0, 0]), edgeNear(m, B, [0, 0, 0], [2.0005, 0, 0])).ok).toBe(true);
    expect(currentViolation(m)).toBeLessThan(1e-9);
    setGround(m, null);
    expect(currentViolation(m)).toBeLessThan(1e-9); // no hidden inconsistency once the ground is released
  });

  it('infeasible: a triangle pinned corner by corner to a ground triangle with a different edge length is refused (a distance between two frozen points counts)', () => {
    const m = createModel();
    const t1 = addPolygonFromPoints(m, [[0, 0, 0], [2, 0, 0], [1, -1.5, 0]]);
    setGround(m, t1.id);
    const t2 = addPolygonFromPoints(m, [[0, 0, 0], [2.2, 0, 0], [1.1, 1.5, 0]]);
    expect(tryAddJoint(m, 'revolute', vertex(t2, 0), vertex(t1, 0), { axis: [0, 0, 1] }).ok).toBe(true);
    const snapshot = serializeModel(m);
    const r = tryAddJoint(m, 'revolute', vertex(t2, 1), vertex(t1, 1), { axis: [0, 0, 1] });
    expect(r.ok).toBe(false);
    expect(r.diagnosis).toMatchObject({ kind: 'infeasible' });
    if (r.diagnosis?.kind === 'infeasible') expect(r.diagnosis.residual).toBeCloseTo(0.2, 6);
    expect(serializeModel(m)).toBe(snapshot);
  });

  it('infeasible: a generic conflict reports the remaining gap and moves nothing', () => {
    // a bar pinned to the origin at one end cannot reach a datum axis 3 units away with its other end on the axis' far side
    const m = createModel();
    const bar = addBar(m, [0, 0, 0], [1, 0, 0]);
    addJoint(m, 'spherical', vertex(bar, 0), { constructionId: 'point_origin' });
    const axis = addConstructionAxis(m, [3, 0, 0], [0, 1, 0]);
    const snapshot = serializeModel(m);
    const r = tryAddJoint(m, 'spherical', vertex(bar, 1), { constructionId: axis.id });
    expect(r.diagnosis).toEqual({ kind: 'incompatible' }); // spherical ↔ axis is not a joint type: use revolute
    const r2 = tryAddJoint(m, 'revolute', vertex(bar, 1), { constructionId: axis.id });
    expect(r2.ok).toBe(false);
    expect(r2.diagnosis?.kind).toBe('infeasible');
    if (r2.diagnosis?.kind === 'infeasible') expect(r2.diagnosis.residual).toBeGreaterThan(0.5);
    expect(serializeModel(m)).toBe(snapshot);
  });

  it('a joint between unconnected links closes no loop and never reports a sector sum', () => {
    const { m, tris } = fourRegularTriangles();
    expect(linkPath(m, tris[0].id, tris[1].id)).toBeNull();
    const j = addJoint(m, 'revolute', edgeNear(m, tris[0], O, corner(1)), edgeNear(m, tris[1], O, corner(1)))!;
    expect(linkPath(m, tris[0].id, tris[1].id)).toEqual([tris[0].id, tris[1].id]);
    expect(sectorSumAt(m, j)).toBeNull(); // only two panels meet at the vertex
    expect(diagnoseJoint(m, j, 0.5, null).kind).not.toBe('sectorSum'); // (a diagnosis is only meaningful after a refused solve)
    expect(unequalEdgeLengths(m, j)).toBeNull();
    expect(closesWithout2d(m, j, [tris[0].id, tris[1].id])).toBe(true); // a consistent loop also closes without the 2-D constraint (only consulted after a failure)
  });
});

describe('sector sums', () => {
  it('measures interior angles of polygons and prism faces', () => {
    const m = createModel();
    const sq = addPolygonFromPoints(m, [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]]);
    for (const id of sq.pointIds) expect(interiorAngleDeg(m, sq, id)).toBeCloseTo(90, 9);
    const tri = addPolygon(m, [0, 0, 0], [0, 0, 1], 1, 3);
    for (const id of tri.pointIds) expect(interiorAngleDeg(m, tri, id)).toBeCloseTo(60, 9);
    const prism = addPrism(m, [0, 0, 0], [0, 0, 1], 1, 6, 1);
    for (const id of prism.pointIds) expect(interiorAngleDeg(m, prism, id)).toBeCloseTo(120, 9); // both rings
    const bar = addBar(m, [0, 0, 0], [1, 0, 0]);
    expect(interiorAngleDeg(m, bar, bar.pointIds[0])).toBeNull();
  });

  it('measures reflex corners of non-convex panels as interior angles (270°), independently of the winding', () => {
    const m = createModel();
    const notched = addPolygonFromPoints(m, [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0.5, 0.5, 0], [0, 1, 0]]);
    expect(interiorAngleDeg(m, notched, notched.pointIds[3])).toBeCloseTo(270, 9);
    expect(notched.pointIds.reduce((sum, id) => sum + interiorAngleDeg(m, notched, id)!, 0)).toBeCloseTo(540, 9); // (n − 2)·180°
    const reversed = addPolygonFromPoints(m, [[0, 1, 0], [0.5, 0.5, 0], [1, 1, 0], [1, 0, 0], [0, 0, 0]]);
    expect(interiorAngleDeg(m, reversed, reversed.pointIds[1])).toBeCloseTo(270, 9);
    const arrow = addPolygonFromPoints(m, [[0, 0, 0], [2, -2, 0], [0, 3, 0], [-2, -2, 0]]);
    expect(interiorAngleDeg(m, arrow, arrow.pointIds[0])).toBeCloseTo(270, 9);
    const L = addPolygonFromPoints(m, [[0, 0, 0], [0, 1, 0], [-1, 1, 0], [-1, -1, 0], [1, -1, 0], [1, 0, 0]]);
    expect(interiorAngleDeg(m, L, L.pointIds[0])).toBeCloseTo(270, 9);
    expect(L.pointIds.reduce((sum, id) => sum + interiorAngleDeg(m, L, id)!, 0)).toBeCloseTo(720, 9);
  });

  it('an L-shaped panel (270° at the vertex) with two 45° triangles is developable: sector sum 360°, the closing crease is accepted', () => {
    const m = createModel();
    const L = addPolygonFromPoints(m, [O, [0, 1, 0], [-1, 1, 0], [-1, -1, 0], [1, -1, 0], [1, 0, 0]], { onPlaneId: 'plane_top', name: 'L' });
    setGround(m, L.id);
    const T1 = addPolygonFromPoints(m, [O, [1, 0, 0], [1, 1, 0]], { onPlaneId: 'plane_top' });
    const T2 = addPolygonFromPoints(m, [O, [1, 1, 0], [0, 1, 0]], { onPlaneId: 'plane_top' });
    expect(tryAddJoint(m, 'revolute', edgeNear(m, L, O, [1, 0, 0]), edgeNear(m, T1, O, [1, 0, 0])).ok).toBe(true);
    expect(tryAddJoint(m, 'revolute', edgeNear(m, T1, O, [1, 1, 0]), edgeNear(m, T2, O, [1, 1, 0])).ok).toBe(true);
    const probe = serializeModel(m);
    const j = addJoint(m, 'revolute', edgeNear(m, T2, O, [0, 1, 0]), edgeNear(m, L, O, [0, 1, 0]))!;
    expect(sectorSumAt(m, j)?.sumDeg).toBe(360);
    expect(diagnoseJoint(m, j, 0.1, linkPath(m, T2.id, L.id, j.id)).kind).not.toBe('sectorSum');
    const fresh = createModel();
    Object.assign(fresh, JSON.parse(probe));
    const r = tryAddJoint(fresh, 'revolute', edgeNear(fresh, fresh.links[T2.id], O, [0, 1, 0]), edgeNear(fresh, fresh.links[L.id], O, [0, 1, 0]));
    expect(r.ok).toBe(true);
    expect(currentViolation(fresh)).toBeLessThan(1e-9);
  });

  it(`uses a ${SECTOR_SUM_TOLERANCE_DEG}° tolerance on the 360° sum`, () => {
    const { m, tris, p } = fourDevelopableTriangles();
    for (let i = 0; i < 3; i++) expect(tryAddJoint(m, 'revolute', edgeNear(m, tris[i], O, p(i + 1)), edgeNear(m, tris[i + 1], O, p(i + 1))).ok).toBe(true);
    const j = addJoint(m, 'revolute', edgeNear(m, tris[3], O, p(4)), edgeNear(m, tris[0], O, p(0)))!;
    expect(sectorSumAt(m, j)?.sumDeg).toBe(360);
    expect(diagnoseJoint(m, j, 0.1, linkPath(m, tris[3].id, tris[0].id, j.id)).kind).not.toBe('sectorSum');
  });
});

describe('diagnosis precedence (plan 0b: edge lengths first, then the sector sum, then the 2-D closure)', () => {
  it('an unequal closing edge on a developable vertex drawn in 2-D reports the edge lengths, not needs3d', () => {
    const m = createModel();
    const pp = (deg: number, rad = 2): Vec3 => [rad * Math.cos((deg * Math.PI) / 180), rad * Math.sin((deg * Math.PI) / 180), 0];
    const D1 = addPolygonFromPoints(m, [O, pp(0), pp(60)], { onPlaneId: 'plane_top' });
    const D2 = addPolygonFromPoints(m, [O, pp(60), pp(120)], { onPlaneId: 'plane_top' });
    const D3 = addPolygonFromPoints(m, [O, pp(120), pp(240)], { onPlaneId: 'plane_top' });
    const q = pp(359.7, 2.04); // D4's closing edge is 2 % longer than D1's and 0.3° off: sectors still sum to 360° within tolerance
    const D4 = addPolygonFromPoints(m, [O, pp(240), q], { onPlaneId: 'plane_top' });
    setGround(m, D1.id);
    expect(tryAddJoint(m, 'revolute', edgeNear(m, D1, O, pp(60)), edgeNear(m, D2, O, pp(60))).ok).toBe(true);
    expect(tryAddJoint(m, 'revolute', edgeNear(m, D2, O, pp(120)), edgeNear(m, D3, O, pp(120))).ok).toBe(true);
    expect(tryAddJoint(m, 'revolute', edgeNear(m, D3, O, pp(240)), edgeNear(m, D4, O, pp(240))).ok).toBe(true);
    const snapshot = serializeModel(m);
    const r = tryAddJoint(m, 'revolute', edgeNear(m, D4, O, q), edgeNear(m, D1, O, pp(0)));
    expect(r.ok).toBe(false);
    expect(r.diagnosis?.kind).toBe('edgeLengths');
    if (r.diagnosis?.kind === 'edgeLengths') {
      expect(r.diagnosis.la).toBeCloseTo(2.04, 9);
      expect(r.diagnosis.lb).toBeCloseTo(2, 9);
    }
    expect(serializeModel(m)).toBe(snapshot);
  });

  it('a 240° loop closed with a 5 % longer edge reports the edge lengths before the sector sum', () => {
    const { m, tris } = fourRegularTriangles({ radii: [1, 1, 1, 1.05] });
    for (let k = 0; k < 3; k++) expect(tryAddJoint(m, 'revolute', edgeNear(m, tris[k], O, corner(k + 1)), edgeNear(m, tris[k + 1], O, corner(k + 1))).ok).toBe(true);
    const snapshot = serializeModel(m);
    const r = tryAddJoint(m, 'revolute', edgeNear(m, tris[3], O, corner(4)), edgeNear(m, tris[0], O, corner(0)));
    expect(r.ok).toBe(false);
    expect(r.diagnosis?.kind).toBe('edgeLengths');
    expect(serializeModel(m)).toBe(snapshot);
    // with equal edges the same loop reports the sector sum (the committed four-regular-triangle case)
    const exact = fourRegularTriangles();
    for (let k = 0; k < 3; k++) expect(tryAddJoint(exact.m, 'revolute', edgeNear(exact.m, exact.tris[k], O, corner(k + 1)), edgeNear(exact.m, exact.tris[k + 1], O, corner(k + 1))).ok).toBe(true);
    expect(tryAddJoint(exact.m, 'revolute', edgeNear(exact.m, exact.tris[3], O, corner(4)), edgeNear(exact.m, exact.tris[0], O, corner(0))).diagnosis?.kind).toBe('sectorSum');
  });
});

describe('tryChangeJointType', () => {
  it('keeps the joint when the new type is incompatible instead of deleting it', () => {
    const { m } = fourBar();
    const pin = Object.values(m.joints).find((j) => j.type === 'revolute')!;
    const snapshot = serializeModel(m);
    const r = tryChangeJointType(m, pin.id, 'planar'); // vertex ↔ vertex planar does not exist
    expect(r.ok).toBe(false);
    expect(r.diagnosis).toEqual({ kind: 'incompatible' });
    expect(serializeModel(m)).toBe(snapshot);
    expect(m.joints[pin.id]).toBeDefined();
  });

  it('changes a compatible type and re-solves', () => {
    const { m } = fourBar();
    const pin = Object.values(m.joints).find((j) => j.type === 'revolute')!;
    const r = tryChangeJointType(m, pin.id, 'spherical');
    expect(r.ok).toBe(true);
    expect(r.joint?.type).toBe('spherical');
    expect(m.joints[pin.id]).toBeUndefined();
    expect(currentViolation(m)).toBeLessThan(1e-9);
  });
});

describe('trySolveCommit', () => {
  it('d) restores the model when a bar in a closed loop is given an impossible rest length', () => {
    const { m, coupler } = fourBar();
    expect(currentViolation(m)).toBeLessThan(1e-9);
    const snapshot = serializeModel(m);
    const before = positions(m);
    const r = coupler.rigidity.find((c) => c.kind === 'dist' && !c.fixed)!;
    if (r.kind === 'dist') r.length = 20; // longer than the other three bars together
    const res = trySolveCommit(m, {}, snapshot);
    expect(res.ok).toBe(false);
    expect(res.residual).toBeGreaterThan(1);
    expect(serializeModel(m)).toBe(snapshot); // including the rest length
    expect(maxMove(m, before)).toBe(0);
    expect(currentViolation(m)).toBeLessThan(1e-9);
  });

  it("without a caller snapshot it restores the positions but not the caller's own edit", () => {
    const { m, coupler } = fourBar();
    const before = positions(m);
    const r = coupler.rigidity.find((c) => c.kind === 'dist' && !c.fixed)!;
    if (r.kind === 'dist') r.length = 20;
    const res = trySolveCommit(m, {});
    expect(res.ok).toBe(false);
    expect(maxMove(m, before)).toBe(0);
    const after = m.links[coupler.id].rigidity.find((c) => c.kind === 'dist' && !c.fixed)!;
    expect(after.kind === 'dist' && after.length).toBe(20); // the UI undoes this through its own snapshot (abortChange)
  });

  it('commits a feasible length change', () => {
    const { m, coupler } = fourBar();
    const r = coupler.rigidity.find((c) => c.kind === 'dist' && !c.fixed)!;
    if (r.kind === 'dist') r.length = 4.5;
    const res = trySolveCommit(m, {});
    expect(res.ok).toBe(true);
    expect(dist(m.points[coupler.pointIds[0]].pos, m.points[coupler.pointIds[1]].pos)).toBeCloseTo(4.5, 7);
    expect(currentViolation(m)).toBeLessThan(1e-8);
  });

  it('refreshes released rest geometry only on acceptance', () => {
    const m = createModel();
    const tri = addPolygonFromPoints(m, [[0, 0, 0], [2, 0, 0], [1, 1, 0]]);
    const rigidity = JSON.stringify(tri.rigidity);
    // a reachable rubber-band move: the apex is released and dragged
    const ok = trySolveCommit(m, { dragTargets: [{ pointId: tri.pointIds[2], pos: [1, 1.5, 0], weight: 1 }], freePointIds: new Set([tri.pointIds[2]]) });
    expect(ok.ok).toBe(true);
    expect(JSON.stringify(m.links[tri.id].rigidity)).not.toBe(rigidity);
    expect(currentViolation(m)).toBeLessThan(1e-8);
  });
});

describe('autoJoinCoincident never commits a refused solve', () => {
  it('still joins coincident panels (the normal case) with zero residual', () => {
    const m = createModel();
    const A = addPolygonFromPoints(m, [[0, 0, 0], [2, 0, 0], [1, 1.5, 0]], { name: 'A' });
    const B = addPolygonFromPoints(m, [[0, 0, 0], [2, 0, 0], [1, -1.5, 0]], { name: 'B' });
    const created = autoJoinCoincident(m, B, B.pointIds, { defaultJoint: 'revolute', axis: [0, 0, 1] }).joints;
    expect(created.length).toBe(1);
    expect(currentViolation(m)).toBeLessThan(1e-9);
    void A;
  });
});
