import { describe, expect, it } from 'vitest';
import { addBar, addConstructionPoint, addJoint, addPolygonFromPoints, createModel, serializeModel, setGround } from '../src/core/model';
import {
  CONVERGED_RESIDUAL,
  consistencyTolerance,
  feasibilityTolerance,
  isAccepted,
  poseIsConsistent,
  solidifyRestGeometry,
  trySolveCommit,
} from '../src/core/feasibility';
import { commitSketch, currentViolation, solveSketch } from '../src/core/kinematics';
import { dist } from '../src/core/geometry';
import type { Feature, Link, Model, RigidityConstraint } from '../src/core/types';

const vertex = (link: Link, i: number): Feature => ({ linkId: link.id, kind: 'vertex', pointIds: [link.pointIds[i]] });

/** Planar four-bar on TOP with revolute pins; returns the coupler so a test can change its rest length. */
function fourBar(): { m: Model; coupler: Link } {
  const m = createModel();
  const g = addBar(m, [0, 0, 0], [4, 0, 0], { onPlaneId: 'plane_top', name: 'ground' });
  setGround(m, g.id);
  const crank = addBar(m, [0, 0, 0], [0, 2, 0], { onPlaneId: 'plane_top', name: 'crank' });
  const coupler = addBar(m, [0, 2, 0], [4, 3, 0], { onPlaneId: 'plane_top', name: 'coupler' });
  const rocker = addBar(m, [4, 3, 0], [4, 0, 0], { onPlaneId: 'plane_top', name: 'rocker' });
  const pin = (x: Link, xi: number, y: Link, yi: number) => addJoint(m, 'revolute', vertex(x, xi), vertex(y, yi), { axis: [0, 0, 1] });
  pin(crank, 0, g, 0);
  pin(crank, 1, coupler, 0);
  pin(coupler, 1, rocker, 0);
  pin(rocker, 1, g, 1);
  return { m, coupler };
}

/** The design length of a bar (its one non-fixed distance). */
const barRest = (bar: Link): RigidityConstraint & { kind: 'dist' } => {
  const r = bar.rigidity.find((c) => c.kind === 'dist' && !c.fixed);
  if (!r || r.kind !== 'dist') throw new Error('bar has no design length');
  return r;
};

/** Every rigidity value of every link, for change detection. */
const restGeometry = (m: Model): string => JSON.stringify(Object.values(m.links).map((l) => l.rigidity));
const positions = (m: Model): string => JSON.stringify(Object.values(m.points).map((p) => p.pos));

/**
 * A triangle that cannot satisfy its joints: V0 is pinned 3 units from the
 * RIGHT plane that V1 must stay on, but the edge V0–V1 is only 2 units long.
 * Releasing the apex (V2) does not help, so a released solve is refused.
 */
function impossibleTriangle(): { m: Model; tri: Link; free: Set<string> } {
  const m = createModel();
  const tri = addPolygonFromPoints(m, [[0, 0, 0], [2, 0, 0], [1, 1, 0]]);
  const pnt = addConstructionPoint(m, [3, 0, 0]);
  addJoint(m, 'spherical', vertex(tri, 0), { constructionId: pnt.id });
  addJoint(m, 'planar', vertex(tri, 1), { constructionId: 'plane_right' });
  return { m, tri, free: new Set([tri.pointIds[2]]) };
}

describe('consistency tolerance', () => {
  it('never drops below the residual a converged solve may leave and scales with large models', () => {
    const m = createModel();
    expect(consistencyTolerance(m)).toBe(CONVERGED_RESIDUAL);
    addBar(m, [0, 0, 0], [1, 0, 0]);
    expect(feasibilityTolerance(m)).toBeLessThan(CONVERGED_RESIDUAL);
    expect(consistencyTolerance(m)).toBe(CONVERGED_RESIDUAL);
    const big = createModel();
    addBar(big, [0, 0, 0], [1e5, 0, 0]);
    expect(consistencyTolerance(big)).toBe(feasibilityTolerance(big));
    expect(consistencyTolerance(big)).toBeGreaterThan(CONVERGED_RESIDUAL);
  });
});

describe('poseIsConsistent', () => {
  it('is true for an empty model and for a freshly built four-bar', () => {
    expect(poseIsConsistent(createModel())).toBe(true);
    expect(poseIsConsistent(fourBar().m)).toBe(true);
  });

  it('tolerates the slop of a converged solve, honours an explicit tolerance, and rejects a real violation', () => {
    const { m, coupler } = fourBar();
    const r = barRest(coupler);
    r.length += 1e-8; // the kind of difference a converged solve leaves behind
    expect(currentViolation(m)).toBeCloseTo(1e-8, 12);
    expect(poseIsConsistent(m)).toBe(true);
    expect(poseIsConsistent(m, 1e-9)).toBe(false);
    r.length = 20; // longer than the other three bars together
    expect(poseIsConsistent(m)).toBe(false);
  });
});

describe('solidifyRestGeometry', () => {
  it('a) refuses a four-bar whose coupler rest length is impossible, leaving every rigidity value and position unchanged', () => {
    const { m, coupler } = fourBar();
    barRest(coupler).length = 20;
    expect(currentViolation(m)).toBeGreaterThan(consistencyTolerance(m));
    const rest = restGeometry(m);
    const pos = positions(m);
    expect(solidifyRestGeometry(m)).toBe(false);
    expect(restGeometry(m)).toBe(rest);
    expect(positions(m)).toBe(pos);
    expect(barRest(coupler).length).toBe(20); // the impossible design is still there for the UI's undo to remove
    expect(currentViolation(m)).toBeGreaterThan(1); // and the violation stays visible instead of becoming the design
  });

  it('b) refreshes the rest geometry of a consistent model and returns true', () => {
    const { m, coupler } = fourBar();
    const r = barRest(coupler);
    const measured = dist(m.points[coupler.pointIds[0]].pos, m.points[coupler.pointIds[1]].pos);
    r.length = measured + 1e-8; // consistent to within the tolerance, but not exactly the pose
    expect(poseIsConsistent(m)).toBe(true);
    expect(solidifyRestGeometry(m)).toBe(true);
    expect(r.length).toBe(measured);
    expect(currentViolation(m)).toBeLessThan(1e-12);
  });

  it('refreshes the angle values of a polygon as well, once its pose is consistent', () => {
    const m = createModel();
    const tri = addPolygonFromPoints(m, [[0, 0, 0], [2, 0, 0], [1, 1, 0]]);
    const rest = restGeometry(m);
    m.points[tri.pointIds[2]].pos = [1 + 1e-8, 1, 0]; // a hair off the design, as after a converged solve
    expect(currentViolation(m)).toBeGreaterThan(0);
    expect(solidifyRestGeometry(m)).toBe(true);
    expect(restGeometry(m)).not.toBe(rest);
    expect(currentViolation(m)).toBeLessThan(1e-12);
  });

  it('does not bake a polygon whose shape has been distorted beyond the tolerance', () => {
    const m = createModel();
    const tri = addPolygonFromPoints(m, [[0, 0, 0], [2, 0, 0], [1, 1, 0]]);
    const rest = restGeometry(m);
    m.points[tri.pointIds[2]].pos = [1, 1.5, 0];
    expect(solidifyRestGeometry(m)).toBe(false);
    expect(restGeometry(m)).toBe(rest);
    expect(currentViolation(m)).toBeGreaterThan(0.1);
  });
});

describe('commitSketch', () => {
  it('c) returns false and leaves the rest geometry alone when the result is not accepted, while still applying the positions', () => {
    const { m, tri, free } = impossibleTriangle();
    const rest = restGeometry(m);
    const res = solveSketch(m, { freePointIds: free });
    expect(isAccepted(res, feasibilityTolerance(m))).toBe(false);
    expect(commitSketch(m, res, free)).toBe(false);
    expect(restGeometry(m)).toBe(rest);
    for (const id of tri.pointIds) expect(dist(m.points[id].pos, res.positions.get(id)!)).toBe(0); // the compromise pose is shown
    expect(currentViolation(m)).toBeGreaterThan(0.1); // and reported, instead of becoming the design
  });

  it('returns true and refreshes the released link when the result is accepted', () => {
    const m = createModel();
    const tri = addPolygonFromPoints(m, [[0, 0, 0], [2, 0, 0], [1, 1, 0]]);
    const rest = restGeometry(m);
    const apex = tri.pointIds[2];
    const free = new Set([apex]);
    const res = solveSketch(m, { dragTargets: [{ pointId: apex, pos: [1, 1.5, 0], weight: 1 }], freePointIds: free });
    expect(isAccepted(res, feasibilityTolerance(m))).toBe(true);
    expect(commitSketch(m, res, free)).toBe(true);
    expect(restGeometry(m)).not.toBe(rest);
    expect(dist(m.points[apex].pos, [1, 1.5, 0])).toBeLessThan(1e-6);
    expect(currentViolation(m)).toBeLessThan(1e-8);
  });

  it('never touches rest geometry when no points were released', () => {
    const { m } = impossibleTriangle();
    const rest = restGeometry(m);
    expect(commitSketch(m, solveSketch(m, {}))).toBe(false);
    expect(restGeometry(m)).toBe(rest);
    const ok = createModel();
    addBar(ok, [0, 0, 0], [2, 0, 0]);
    const restBar = restGeometry(ok);
    expect(commitSketch(ok, solveSketch(ok, {}))).toBe(true);
    expect(restGeometry(ok)).toBe(restBar);
  });
});

describe('trySolveCommit (the Select tool release path) with released points', () => {
  it('restores the whole model when the released solve is refused', () => {
    const { m, free } = impossibleTriangle();
    const snapshot = serializeModel(m);
    const r = trySolveCommit(m, { freePointIds: free, maxIter: 40 });
    expect(r.ok).toBe(false);
    expect(serializeModel(m)).toBe(snapshot);
  });
});
