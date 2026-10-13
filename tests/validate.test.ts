import { describe, expect, it } from 'vitest';
import { addBar, addJoint, addPolygonFromPoints, createModel, setGround } from '../src/core/model';
import { tryAddJoint } from '../src/core/feasibility';
import { currentViolation } from '../src/core/kinematics';
import { origamiMiuraVertex } from '../src/core/examples';
import { creaseMV, findCreaseLoops, prefoldVertex } from '../src/core/fold';
import { kawasakiSumDeg, validateCreasePattern, type VertexReport } from '../src/core/validate';
import { SIM } from '../src/ui/strings';
import { dist } from '../src/core/geometry';
import type { Feature, ID, Link, Model, Vec3 } from '../src/core/types';

const O: Vec3 = [0, 0, 0];
/** Point on the circle of radius 2 in the TOP plane at the given angle (degrees). */
const R = (deg: number): Vec3 => [2 * Math.cos((deg * Math.PI) / 180), 2 * Math.sin((deg * Math.PI) / 180), 0];

/** Edge feature of `link` whose end points are nearest to p and q. */
function edgeNear(m: Model, link: Link, p: Vec3, q: Vec3): Feature {
  const nearest = (x: Vec3): ID => link.pointIds.reduce((b, id) => (dist(m.points[id].pos, x) < dist(m.points[b].pos, x) ? id : b), link.pointIds[0]);
  return { linkId: link.id, kind: 'edge', pointIds: [nearest(p), nearest(q)] };
}

/**
 * Triangles apex–c_i–c_{i+1} around a common apex, consecutive ones joined into creases with tryAddJoint (exact merges:
 * nothing moves); the first triangle is ground. `creases` limits the number of joints (n − 1 leaves an open fan).
 */
function ring(apex: Vec3, corners: Vec3[], opts: { planar?: boolean; creases?: number } = {}): { m: Model; tris: Link[] } {
  const m = createModel();
  const n = corners.length;
  const c = (i: number): Vec3 => corners[i % n];
  const tris = corners.map((_, i) => addPolygonFromPoints(m, [apex, c(i), c(i + 1)], { onPlaneId: opts.planar ? 'plane_top' : null, name: `D${i + 1}` }));
  setGround(m, tris[0].id);
  for (let i = 0; i < (opts.creases ?? n); i++) {
    const r = tryAddJoint(m, 'revolute', edgeNear(m, tris[i], apex, c(i + 1)), edgeNear(m, tris[(i + 1) % n], apex, c(i + 1)));
    expect(r.ok).toBe(true);
    expect(r.joint?.pairs?.length).toBe(2);
  }
  return { m, tris };
}

/** Flat triangles around the origin with creases in the given directions (degrees): the sectors are the gaps between them. */
const flatVertex = (anglesDeg: number[], opts: { planar?: boolean; creases?: number } = {}) => ring(O, anglesDeg.map(R), opts);
/** The developable (60°, 60°, 120°, 120°) vertex. */
const developable = (opts: { planar?: boolean; creases?: number } = {}) => flatVertex([0, 60, 120, 240], opts);
/** The "X" vertex (four 90° sectors). */
const xVertex = () => flatVertex([0, 90, 180, 270]);
/** Square pyramid with apex height h over the diamond of radius 2: four lateral triangles closing a 3-D vertex. */
const pyramid = (h: number) => ring([0, 0, h], [0, 90, 180, 270].map(R));

const sorted = (xs: number[]): number[] => [...xs].sort((a, b) => a - b);
const expectPass = (r: VertexReport, tol = 1e-9): void => {
  expect(r.developable).toBe(true);
  expect(r.sumDeg).toBeCloseTo(360, 9);
  expect(Math.abs(r.kawasakiDeg)).toBeLessThan(tol);
  expect(r.flatFoldable).toBe(true);
};

describe('kawasakiSumDeg', () => {
  it('is the alternating sum in cyclic order and NaN for an odd number of sectors', () => {
    expect(kawasakiSumDeg([60, 120, 120, 60])).toBe(0);
    expect(kawasakiSumDeg([90, 90, 60, 120])).toBe(-60);
    expect(kawasakiSumDeg([120, 120, 120])).toBeNaN();
    expect(kawasakiSumDeg([])).toBe(0);
  });
});

describe('validateCreasePattern: interior vertices', () => {
  it('the developable (60°, 60°, 120°, 120°) vertex: developable, Kawasaki 0, M/V not assigned while flat', () => {
    const { m, tris } = developable();
    const reports = validateCreasePattern(m);
    expect(reports.length).toBe(1);
    const [r] = reports;
    expect(r.panelCount).toBe(4);
    expect(sorted(r.sectorDeg)).toEqual([60, 60, 120, 120].map((x) => expect.closeTo(x, 9)));
    expectPass(r);
    expect(r.mountains).toBeNull();
    expect(r.valleys).toBeNull();
    expect(r.maekawa).toBeNull();
    // the sectors and creases follow the loop order: sector i lies between crease i and crease i + 1
    const [loop] = findCreaseLoops(m);
    expect(r.creaseIds).toEqual(loop.creaseIds);
    expect(r.vertexPointId).toBe(loop.vertexPointId);
    expect(m.links[m.points[r.vertexPointId].linkId].ground).toBe(true);
    expect(r.vertexName).toBe(`${tris[0].name} V0`);
    for (let i = 0; i < 4; i++) {
      const panel = m.links[loop.linkIds[i]];
      const expected = loop.linkIds[i] === tris[0].id || loop.linkIds[i] === tris[1].id ? 60 : 120;
      expect(panel.kind).toBe('polygon');
      expect(r.sectorDeg[i]).toBeCloseTo(expected, 9);
    }
  });

  it('the developable vertex after Fold: four classes assigned, Maekawa 3:1 holds', () => {
    const { m } = developable();
    expect(prefoldVertex(m).ok).toBe(true);
    const [r] = validateCreasePattern(m);
    expectPass(r);
    expect(r.mountains! + r.valleys!).toBe(4);
    expect(sorted([r.mountains!, r.valleys!])).toEqual([1, 3]);
    expect(r.maekawa).toBe(true);
    // the counts agree with the classes the crease display shows
    const shown = r.creaseIds.map((id) => creaseMV(m, m.joints[id]));
    expect(shown.filter((c) => c === 'M').length).toBe(r.mountains);
    expect(shown.filter((c) => c === 'V').length).toBe(r.valleys);
  });

  it('the "X" vertex (four 90° sectors): developable and Kawasaki 0; the failed pre-fold leaves M/V unassigned', () => {
    const { m } = xVertex();
    const [r] = validateCreasePattern(m);
    expect(r.panelCount).toBe(4);
    expect(r.sectorDeg.every((a) => Math.abs(a - 90) < 1e-9)).toBe(true);
    expectPass(r);
    expect(r.maekawa).toBeNull();
    const fold = prefoldVertex(m);
    expect(fold.ok).toBe(false);
    expect(validateCreasePattern(m)[0].maekawa).toBeNull();
  });

  it('an open fan of three creases is not an interior vertex: no report; the closing crease adds one', () => {
    const { m, tris } = developable({ creases: 3 });
    expect(validateCreasePattern(m)).toEqual([]);
    const r = tryAddJoint(m, 'revolute', edgeNear(m, tris[3], O, R(0)), edgeNear(m, tris[0], O, R(0)));
    expect(r.ok).toBe(true);
    expect(validateCreasePattern(m).length).toBe(1);
  });

  it('a (90°, 90°, 60°, 120°) vertex is developable but fails Kawasaki by 60°; (90°, 90°, 100°, 80°) by 20°', () => {
    const a = validateCreasePattern(flatVertex([0, 90, 180, 240]).m);
    expect(a.length).toBe(1);
    expect(sorted(a[0].sectorDeg)).toEqual([60, 90, 90, 120].map((x) => expect.closeTo(x, 9)));
    expect(a[0].developable).toBe(true);
    expect(Math.abs(a[0].kawasakiDeg)).toBeCloseTo(60, 9);
    expect(a[0].flatFoldable).toBe(false);
    const b = validateCreasePattern(flatVertex([0, 90, 180, 280]).m);
    expect(b[0].developable).toBe(true);
    expect(Math.abs(b[0].kawasakiDeg)).toBeCloseTo(20, 9);
    expect(b[0].flatFoldable).toBe(false);
    // a tolerance wider than the defect accepts it
    expect(validateCreasePattern(flatVertex([0, 90, 180, 280]).m, 25)[0].flatFoldable).toBe(true);
  });

  it('a vertex of odd degree (three 120° sectors): developable, Kawasaki NaN and not flat-foldable', () => {
    const { m } = flatVertex([0, 120, 240]);
    const [r] = validateCreasePattern(m);
    expect(r.panelCount).toBe(3);
    expect(r.sumDeg).toBeCloseTo(360, 9);
    expect(r.developable).toBe(true);
    expect(r.kawasakiDeg).toBeNaN();
    expect(r.flatFoldable).toBe(false);
    expect(r.maekawa).toBeNull();
  });

  it('a square pyramid apex (four 60° sectors) is a closed, non-developable vertex: sum 240°, Kawasaki 0', () => {
    const { m } = pyramid(2); // lateral edge √8 = base side: equilateral faces
    expect(currentViolation(m)).toBeLessThan(1e-9);
    const [r] = validateCreasePattern(m);
    expect(r.panelCount).toBe(4);
    expect(r.sectorDeg.every((a) => Math.abs(a - 60) < 1e-9)).toBe(true);
    expect(r.sumDeg).toBeCloseTo(240, 9);
    expect(r.developable).toBe(false);
    expect(Math.abs(r.kawasakiDeg)).toBeLessThan(1e-9);
    expect(r.flatFoldable).toBe(true); // the Kawasaki check alone; developability is reported separately
    // every face is bent the same way about the apex: four creases of one class, Maekawa fails
    expect(r.mountains! + r.valleys!).toBe(4);
    expect(r.maekawa).toBe(false);
  });

  it('the tolerance: a shallow pyramid (sum 359.63°) is developable within 0.5° but not within 0.1°', () => {
    const { m } = pyramid(0.08);
    const [r] = validateCreasePattern(m);
    expect(r.sumDeg).toBeGreaterThan(359.5);
    expect(r.sumDeg).toBeLessThan(359.7);
    expect(r.developable).toBe(true);
    expect(validateCreasePattern(m, 0.1)[0].developable).toBe(false);
  });

  it('the Miura example: developable, Kawasaki 0 and Maekawa 3:1 in its folded pose', () => {
    const m = origamiMiuraVertex(60);
    const reports = validateCreasePattern(m);
    expect(reports.length).toBe(1);
    const [r] = reports;
    expect(r.panelCount).toBe(4);
    expect(sorted(r.sectorDeg)).toEqual([60, 60, 120, 120].map((x) => expect.closeTo(x, 9)));
    expectPass(r);
    expect(sorted([r.mountains!, r.valleys!])).toEqual([1, 3]);
    expect(r.maekawa).toBe(true);
    expect(r.vertexName).toBe('Panel 1 V0');
  });

  it('models without a closed ring of panels report nothing', () => {
    expect(validateCreasePattern(createModel())).toEqual([]);
    const m = createModel();
    const bar = addBar(m, [0, 0, 0], [2, 0, 0]);
    const other = addBar(m, [2, 0, 0], [2, 2, 0]);
    addJoint(m, 'revolute', { linkId: bar.id, kind: 'vertex', pointIds: [bar.pointIds[1]] }, { linkId: other.id, kind: 'vertex', pointIds: [other.pointIds[0]] }, { axis: [0, 0, 1] });
    expect(validateCreasePattern(m)).toEqual([]);
  });
});

describe('Mechanism-panel lines (SIM.vertexReport / SIM.vertexFails)', () => {
  it('a passing vertex reads "… 360.0° developable ✓ · Kawasaki 0.0° ✓ · M/V 3:1 ✓"', () => {
    const m = origamiMiuraVertex(60);
    const [r] = validateCreasePattern(m);
    const line = SIM.vertexReport(r);
    expect(line).toBe(`Panel 1 V0 · 4 panels · 360.0° developable ✓ · Kawasaki 0.0° ✓ · M/V ${r.mountains}:${r.valleys} ✓`);
  });

  it('a flat vertex has no M/V assignment; failures show ✗ with the measured numbers and a hint names them', () => {
    const flat = validateCreasePattern(developable().m)[0];
    expect(SIM.vertexReport(flat)).toContain('M/V not assigned');
    expect(SIM.vertexReport(flat)).not.toContain('✗');
    const kawasaki = validateCreasePattern(flatVertex([0, 90, 180, 280]).m)[0];
    expect(SIM.vertexReport(kawasaki)).toContain('Kawasaki 20.0° ✗');
    expect(SIM.vertexFails(kawasaki)).toMatch(/alternating sum .* 20\.0°/);
    const odd = validateCreasePattern(flatVertex([0, 120, 240]).m)[0];
    expect(SIM.vertexReport(odd)).toContain('3 panels');
    expect(SIM.vertexReport(odd)).toContain('Kawasaki ✗ odd degree');
    expect(SIM.vertexFails(odd)).toMatch(/odd number/);
    const apex = validateCreasePattern(pyramid(2).m)[0];
    expect(SIM.vertexReport(apex)).toContain('240.0° developable ✗');
    expect(SIM.vertexFails(apex)).toMatch(/240\.0°, not 360°/);
    expect(SIM.vertexFails(apex)).toMatch(/Maekawa/);
  });
});
