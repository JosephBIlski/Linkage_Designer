import { describe, expect, it } from 'vitest';
import { fourBarCrankRocker } from '../src/core/examples';
import { samplePoseValues, solvePosesAt, sweepDriver, solveForward } from '../src/core/kinematics';
import { analyseDesign, applyDesign, solveDesign } from '../src/core/synthesis';
import { dist } from '../src/core/geometry';
import type { Target } from '../src/core/types';
import { addConstructionPlane } from '../src/core/model';

function setup() {
  const m = fourBarCrankRocker();
  const sweep = sweepDriver(m, 0, { step: 5 });
  const P = 12;
  const values = samplePoseValues(sweep, P);
  const poses = solvePosesAt(m, sweep, values);
  const coupler = Object.values(m.links).find((l) => l.name === 'Coupler')!;
  const C = coupler.pointIds[2];
  return { m, sweep, P, values, poses, C };
}

describe('inverse design of a four-bar coupler curve', () => {
  it('reports 9 design degrees of freedom for a planar four-bar with a triangular coupler and free pivots', () => {
    const { m, values, poses, C } = setup();
    const res = solveDesign({ model: m, poseValues: values.map((v) => [v]), initPositions: poses.map((p) => p.positions), soft: true });
    expect(res.converged).toBe(true);
    const an = analyseDesign(m, res, [C]);
    expect(an.motionDOF).toBe(1);
    expect(an.designDOF).toBe(9);
    expect(an.spaces[0].dim).toBe(2); // planar: the coupler point can be moved anywhere in the sketch plane
  });

  it('moves the coupler point to a dragged target while keeping all link lengths constant across poses', () => {
    const { m, values, poses, C } = setup();
    const k = 4;
    const original = poses[k].positions.get(C)!;
    const target: Target = { id: 't1', pointId: C, pose: k, kind: 'position', position: [original[0] + 0.4, original[1] + 0.3, 0], locked: false };
    m.targets.push(target);
    const res = solveDesign({ model: m, poseValues: values.map((v) => [v]), initPositions: poses.map((p) => p.positions), soft: true });
    expect(res.converged).toBe(true);
    const reached = res.poses[k].positions.get(C)!;
    expect(dist(reached, target.position!)).toBeLessThan(1e-5);
    // lengths equal across poses
    const crank = Object.values(m.links).find((l) => l.name === 'Crank')!;
    const L0 = dist(res.poses[0].positions.get(crank.pointIds[0])!, res.poses[0].positions.get(crank.pointIds[1])!);
    for (const p of res.poses) {
      const L = dist(p.positions.get(crank.pointIds[0])!, p.positions.get(crank.pointIds[1])!);
      expect(L).toBeCloseTo(L0, 6);
    }
    // design DOF decreased by 2 (planar position target)
    const an = analyseDesign(m, res, [C]);
    expect(an.designDOF).toBe(7);
    expect(an.targetCount).toBe(1);
    // apply the design and check the forward simulation reproduces the target
    applyDesign(m, res);
    for (const [id, p] of res.poses[0].positions) m.points[id].pos = p;
    m.drivers[0].value = values[0];
    const fwd = solveForward(m, [values[k]], res.poses[k].positions);
    expect(fwd.converged).toBe(true);
    expect(dist(fwd.positions.get(C)!, target.position!)).toBeLessThan(1e-5);
  });

  it('locked links keep their length during inverse design', () => {
    const { m, values, poses, C } = setup();
    const crank = Object.values(m.links).find((l) => l.name === 'Crank')!;
    crank.locked = true;
    const k = 6;
    const original = poses[k].positions.get(C)!;
    m.targets.push({ id: 't1', pointId: C, pose: k, kind: 'position', position: [original[0] - 0.3, original[1] + 0.2, 0], locked: false });
    const res = solveDesign({ model: m, poseValues: values.map((v) => [v]), initPositions: poses.map((p) => p.positions), soft: true });
    expect(res.converged).toBe(true);
    const L = dist(res.poses[3].positions.get(crank.pointIds[0])!, res.poses[3].positions.get(crank.pointIds[1])!);
    expect(L).toBeCloseTo(1, 6);
    const an = analyseDesign(m, res, [C]);
    expect(an.designDOF).toBe(6); // 9 - 1 (locked crank length) - 2 (target)
  });

  it('a fully specified point has a zero-dimensional design space', () => {
    const { m, values, poses, C } = setup();
    // Lock everything except the coupler triangle (3 shape parameters), then
    // impose 3 equations: a position target (2) and a construction-plane target (1).
    for (const l of Object.values(m.links)) if (l.name !== 'Coupler') l.locked = true;
    const base = (k: number) => poses[k].positions.get(C)!;
    const plane = addConstructionPlane(m, [0, base(7)[1] + 0.05, 0], [0, 1, 0], 'DTM1');
    m.targets.push({ id: 't1', pointId: C, pose: 2, kind: 'position', position: [base(2)[0] + 0.1, base(2)[1] + 0.05, 0], locked: true });
    m.targets.push({ id: 't2', pointId: C, pose: 7, kind: 'onPlane', constructionId: plane.id, locked: true });
    const res = solveDesign({ model: m, poseValues: values.map((v) => [v]), initPositions: poses.map((p) => p.positions), soft: true, maxIter: 200 });
    expect(res.converged).toBe(true);
    expect(res.poses[7].positions.get(C)![1]).toBeCloseTo(base(7)[1] + 0.05, 5);
    const an = analyseDesign(m, res, [C]);
    expect(an.designDOF).toBe(0);
    expect(an.spaces[0].dim).toBe(0);
    expect(an.overConstrained).toBe(false);
  });

  it('reports over-constrained designs when targets exceed the design freedom', () => {
    const { m, values, poses, C } = setup();
    for (const l of Object.values(m.links)) if (l.name !== 'Coupler') l.locked = true;
    const base = (k: number) => poses[k].positions.get(C)!;
    // two incompatible position targets = 4 equations for 3 shape parameters
    m.targets.push({ id: 't1', pointId: C, pose: 2, kind: 'position', position: [base(2)[0] + 0.3, base(2)[1] + 0.3, 0], locked: true });
    m.targets.push({ id: 't2', pointId: C, pose: 8, kind: 'position', position: [base(8)[0] - 0.3, base(8)[1] - 0.3, 0], locked: true });
    const res = solveDesign({ model: m, poseValues: values.map((v) => [v]), initPositions: poses.map((p) => p.positions), soft: true, maxIter: 100 });
    const an = analyseDesign(m, res, [C]);
    expect(an.designDOF).toBe(0);
    expect(an.overConstrained).toBe(true);
  });
});
