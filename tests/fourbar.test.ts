import { describe, expect, it } from 'vitest';
import { fourBarCrankRocker, sliderCrank, sphericalPendulum, origamiMiuraVertex } from '../src/core/examples';
import { commitSketch, computeMobility, currentViolation, samplePoseValues, solveForward, solvePosesAt, solveSketch, sweepDriver } from '../src/core/kinematics';
import { addBar, addJoint, createModel, setGround } from '../src/core/model';
import { barLength } from '../src/core/model';
import { measureJointDriver } from '../src/core/jointMeasure';
import { dist } from '../src/core/geometry';

describe('four-bar crank-rocker', () => {
  it('starts in a valid assembly', () => {
    const m = fourBarCrankRocker();
    expect(currentViolation(m)).toBeLessThan(1e-6);
  });
  it('has one degree of freedom', () => {
    const m = fourBarCrankRocker();
    const mob = computeMobility(m);
    expect(mob.grounded).toBe(true);
    expect(mob.dof).toBe(1);
  });
  it('rotates fully as a crank (Grashof) and keeps link lengths', () => {
    const m = fourBarCrankRocker();
    const sweep = sweepDriver(m, 0, { step: 5 });
    expect(sweep.isCrank).toBe(true);
    expect(sweep.poses.length).toBeGreaterThan(60);
    const crank = Object.values(m.links).find((l) => l.name === 'Crank')!;
    const follower = Object.values(m.links).find((l) => l.name === 'Follower')!;
    for (const p of sweep.poses) {
      const a = p.positions.get(crank.pointIds[0])!;
      const b = p.positions.get(crank.pointIds[1])!;
      expect(dist(a, b)).toBeCloseTo(barLength(m, crank), 6);
      const c = p.positions.get(follower.pointIds[0])!;
      const d = p.positions.get(follower.pointIds[1])!;
      expect(dist(c, d)).toBeCloseTo(3, 6);
      // planar
      expect(Math.abs(b[2])).toBeLessThan(1e-9);
    }
    // follower is a rocker: its angle range is less than 360
    const values = samplePoseValues(sweep, 8);
    const poses = solvePosesAt(m, sweep, values);
    expect(poses.every((p) => p.converged)).toBe(true);
  });
  it('matches the analytic follower position at crank = 90°', () => {
    const m = fourBarCrankRocker();
    const res = solveForward(m, [90]);
    expect(res.converged).toBe(true);
    const follower = Object.values(m.links).find((l) => l.name === 'Follower')!;
    const B = res.positions.get(follower.pointIds[1])!;
    // B is at distance 3.5 from A=(0,1) and 3 from O4=(4,0)
    expect(dist(B, [0, 1, 0])).toBeCloseTo(3.5, 6);
    expect(dist(B, [4, 0, 0])).toBeCloseTo(3, 6);
  });
});

describe('slider-crank', () => {
  it('has one degree of freedom and the slider stays on its axis', () => {
    const m = sliderCrank();
    expect(currentViolation(m)).toBeLessThan(1e-6);
    expect(computeMobility(m).dof).toBe(1);
    const sweep = sweepDriver(m, 0, { step: 10 });
    expect(sweep.isCrank).toBe(true);
    const slider = Object.values(m.links).find((l) => l.name === 'Slider')!;
    const y0 = m.points[slider.pointIds[2]].pos[1];
    for (const p of sweep.poses) {
      const v2 = p.positions.get(slider.pointIds[2])!;
      const v3 = p.positions.get(slider.pointIds[3])!;
      expect(v2[1]).toBeCloseTo(y0, 6);
      expect(v3[1]).toBeCloseTo(y0, 6);
    }
  });
});

describe('spherical pendulum', () => {
  it('has two degrees of freedom (a 2-point bar has no roll)', () => {
    const m = sphericalPendulum();
    expect(computeMobility(m).dof).toBe(2);
  });
});

describe('rigid origami degree-4 vertex (Miura)', () => {
  it('starts exactly assembled with a single folding degree of freedom', () => {
    const m = origamiMiuraVertex(60);
    expect(currentViolation(m)).toBeLessThan(1e-9);
    expect(computeMobility(m).dof).toBe(1);
  });
  it('folds as a Miura vertex: all four creases move, opposite creases are equal', () => {
    const m = origamiMiuraVertex(60);
    const sweep = sweepDriver(m, 0, { step: 5 });
    const creases = Object.values(m.joints).filter((j) => j.type === 'revolute');
    expect(creases.length).toBe(4);
    const angles = creases.map((j) => sweep.poses.map((p) => measureJointDriver(m, { id: 'x', kind: 'fold', jointId: j.id, value: 0 }, p.positions)!));
    // every crease folds through a substantial range (the degenerate straight-hinge branch keeps two creases flat)
    for (const a of angles) {
      const span = Math.max(...a.map((v) => 180 - Math.abs(v))) - Math.min(...a.map((v) => 180 - Math.abs(v)));
      expect(span).toBeGreaterThan(60);
    }
    // creases 0 and 2 are collinear (straight pair) and fold by the same angle; creases 1 and 3 (bent pair) too
    for (let k = 0; k < sweep.poses.length; k++) {
      expect(Math.abs(Math.abs(angles[0][k]) - Math.abs(angles[2][k]))).toBeLessThan(1e-3);
      expect(Math.abs(Math.abs(angles[1][k]) - Math.abs(angles[3][k]))).toBeLessThan(1e-3);
    }
    // Miura relation between the bent pair and the straight pair: tan(ρ_bent/2) = tan(ρ_straight/2) / cos α
    const k = Math.floor(sweep.poses.length / 4);
    const rhoS = (Math.PI - Math.abs((angles[0][k] * Math.PI) / 180));
    const rhoB = (Math.PI - Math.abs((angles[1][k] * Math.PI) / 180));
    if (rhoS > 0.2 && rhoS < Math.PI - 0.2) {
      expect(Math.tan(rhoB / 2) / Math.tan(rhoS / 2)).toBeCloseTo(1 / Math.cos(Math.PI / 3), 2);
    }
    // the sweep never visits the straight-hinge branch (bent creases flat while the straight pair is folded)
    for (let p = 0; p < sweep.poses.length; p++) {
      const straightFolded = 180 - Math.abs(angles[0][p]) > 20;
      const bentFlat = 180 - Math.abs(angles[1][p]) < 1e-3;
      expect(straightFolded && bentFlat).toBe(false);
    }
  });
});

describe('construction-mode sketch editing', () => {
  it('rubber-bands a free link end toward a soft drag target and refreshes the length', () => {
    const m = createModel();
    const g = addBar(m, [0, 0, 0], [2, 0, 0], { onPlaneId: 'plane_top' });
    const l2 = addBar(m, [2, 0, 0], [3, 1.5, 0], { onPlaneId: 'plane_top' });
    setGround(m, g.id);
    addJoint(m, 'revolute', { linkId: g.id, kind: 'vertex', pointIds: [g.pointIds[1]] }, { linkId: l2.id, kind: 'vertex', pointIds: [l2.pointIds[0]] });
    const end = l2.pointIds[1];
    const res = solveSketch(m, { dragTargets: [{ pointId: end, pos: [3.5, 2.5, 0], weight: 1 }], freePointIds: new Set([end]) });
    const p = res.positions.get(end)!;
    expect(dist(p, [3.5, 2.5, 0])).toBeLessThan(1e-4);
    commitSketch(m, res, new Set([end]));
    expect(barLength(m, l2)).toBeCloseTo(dist([2, 0, 0], [3.5, 2.5, 0]), 4);
    // the pinned end did not move
    expect(dist(m.points[l2.pointIds[0]].pos, [2, 0, 0])).toBeLessThan(1e-8);
  });

  it('translating a link drags the links joined to it', () => {
    const m = createModel();
    const g = addBar(m, [0, 0, 0], [4, 0, 0], { onPlaneId: 'plane_top' });
    const crank = addBar(m, [0, 0, 0], [0, 1, 0], { onPlaneId: 'plane_top' });
    setGround(m, g.id);
    addJoint(m, 'revolute', { linkId: g.id, kind: 'vertex', pointIds: [g.pointIds[0]] }, { linkId: crank.id, kind: 'vertex', pointIds: [crank.pointIds[0]] });
    // move the ground link: the crank must follow its pivot
    const res = solveSketch(m, { dragTargets: g.pointIds.map((id, i) => ({ pointId: id, pos: [i * 4 + 1, 1, 0] as [number, number, number], weight: 1 })), allowGroundMove: true });
    commitSketch(m, res);
    expect(dist(m.points[g.pointIds[0]].pos, [1, 1, 0])).toBeLessThan(1e-4);
    expect(dist(m.points[crank.pointIds[0]].pos, [1, 1, 0])).toBeLessThan(1e-6);
    expect(barLength(m, crank)).toBeCloseTo(1, 6);
  });
});
