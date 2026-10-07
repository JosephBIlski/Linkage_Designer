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
  closesWithout2d,
  diagnoseJoint,
  feasibilityTolerance,
  interiorAngleDeg,
  isAccepted,
  linkPath,
  sectorSumAt,
  tryAddJoint,
  tryChangeJointType,
  trySolveCommit,
  unequalEdgeLengths,
} from '../src/core/feasibility';
import { autoJoinCoincident } from '../src/core/edit';
import { computeMobility, currentViolation, modelSize, solveSketch } from '../src/core/kinematics';
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

/**
 * Four regular triangles as the Polygon tool draws them in 2-D mode (circumradius 1, flat in XY, body planar joint to
 * TOP), consecutive triangles sharing the edge O–corner(k+1); T1 is ground.
 */
function fourRegularTriangles(opts: { planar?: boolean } = {}): { m: Model; tris: Link[] } {
  const m = createModel();
  const tris: Link[] = [];
  for (let k = 0; k < 4; k++) {
    const c: Vec3 = [(corner(k)[0] + corner(k + 1)[0]) / 3, (corner(k)[1] + corner(k + 1)[1]) / 3, 0];
    tris.push(addPolygon(m, c, [0, 0, 1], 1, 3, { startAngle: Math.atan2(O[1] - c[1], O[0] - c[0]), onPlaneId: opts.planar === false ? null : 'plane_top', name: `T${k + 1}` }));
  }
  setGround(m, tris[0].id);
  return { m, tris };
}

/** Four developable triangles O–p_i–p_{i+1} with sectors 60°, 60°, 120°, 120° on a circle of radius 2, flat, 2-D. */
function fourDevelopableTriangles(): { m: Model; tris: Link[]; p: (i: number) => Vec3 } {
  const m = createModel();
  const ang = [0, 60, 120, 240, 360].map((d) => (d * Math.PI) / 180);
  const p = (i: number): Vec3 => [2 * Math.cos(ang[i]), 2 * Math.sin(ang[i]), 0];
  const tris = [0, 1, 2, 3].map((i) => addPolygonFromPoints(m, [O, p(i), p(i + 1)], { onPlaneId: 'plane_top', name: `D${i + 1}` }));
  setGround(m, tris[0].id);
  return { m, tris, p };
}

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
    const m = createModel();
    const sq: Link[] = [];
    for (let i = 0; i < 3; i++) sq.push(addPolygonFromPoints(m, [[i, 0, 0], [i + 1, 0, 0], [i + 1, 1, 0], [i, 1, 0]], { onPlaneId: 'plane_top', name: `S${i}` }));
    setGround(m, sq[0].id);
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

  it(`uses a ${SECTOR_SUM_TOLERANCE_DEG}° tolerance on the 360° sum`, () => {
    const { m, tris, p } = fourDevelopableTriangles();
    for (let i = 0; i < 3; i++) expect(tryAddJoint(m, 'revolute', edgeNear(m, tris[i], O, p(i + 1)), edgeNear(m, tris[i + 1], O, p(i + 1))).ok).toBe(true);
    const j = addJoint(m, 'revolute', edgeNear(m, tris[3], O, p(4)), edgeNear(m, tris[0], O, p(0)))!;
    expect(sectorSumAt(m, j)?.sumDeg).toBe(360);
    expect(diagnoseJoint(m, j, 0.1, linkPath(m, tris[3].id, tris[0].id, j.id)).kind).not.toBe('sectorSum');
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
    const created = autoJoinCoincident(m, B, B.pointIds, { defaultJoint: 'revolute', axis: [0, 0, 1] });
    expect(created.length).toBe(1);
    expect(currentViolation(m)).toBeLessThan(1e-9);
    void A;
  });
});
