import { describe, expect, it } from 'vitest';
import { addBar, addJoint, addPolygon, addPolygonFromPoints, bodyPlaneJoint, createModel, serializeModel, setGround } from '../src/core/model';
import { PLACEMENT_MIN_COS, autoJoinCoincident, fitSketchPlane, moveVertex, placeOnFittedPlane, placementOnPlane, projectToPlane, rayPlane } from '../src/core/edit';
import { addConstructionAxis, addConstructionPoint } from '../src/core/model';
import { commitSketch, solveSketch } from '../src/core/kinematics';
import { computeMobility, currentViolation } from '../src/core/kinematics';
import { foldedVertexPositions } from '../src/core/examples';
import { dist } from '../src/core/geometry';
import type { Vec3 } from '../src/core/types';

describe('edit tool: moveVertex', () => {
  it('moves a frame vertex of a quad off its plane without disturbing the other vertices', () => {
    const m = createModel();
    const poly = addPolygon(m, [0, 0, 0], [0, 0, 1], 1, 4, { onPlaneId: 'plane_top' });
    const before = poly.pointIds.map((id) => [...m.points[id].pos] as Vec3);
    const v0 = poly.pointIds[0];
    const dest: Vec3 = [before[0][0], before[0][1], 1];
    const res = moveVertex(m, v0, dest);
    expect(res.ok).toBe(true);
    expect(dist(m.points[v0].pos, dest)).toBeLessThan(1e-6);
    for (let i = 1; i < 4; i++) expect(dist(m.points[poly.pointIds[i]].pos, before[i])).toBeLessThan(1e-9);
    expect(bodyPlaneJoint(m, poly.id)).toBeNull(); // left the sketch plane
    expect(currentViolation(m)).toBeLessThan(1e-9); // rest geometry rebuilt consistently
  });

  it('keeps an in-plane edit planar and lets a joined link follow', () => {
    const m = createModel();
    const g = addBar(m, [-2, 0, 0], [-1, 0, 0], { onPlaneId: 'plane_top' });
    setGround(m, g.id);
    const bar = addBar(m, [-1, 0, 0], [1, 0, 0], { onPlaneId: 'plane_top' });
    addJoint(m, 'revolute', { linkId: g.id, kind: 'vertex', pointIds: [g.pointIds[1]] }, { linkId: bar.id, kind: 'vertex', pointIds: [bar.pointIds[0]] });
    const res = moveVertex(m, bar.pointIds[1], [1, 1.5, 0]);
    expect(res.ok).toBe(true);
    expect(dist(m.points[bar.pointIds[1]].pos, [1, 1.5, 0])).toBeLessThan(1e-6);
    expect(dist(m.points[bar.pointIds[0]].pos, [-1, 0, 0])).toBeLessThan(1e-9);
    expect(bodyPlaneJoint(m, bar.id)).not.toBeNull();
    expect(currentViolation(m)).toBeLessThan(1e-9);
  });

  it('restores the model unchanged when the destination is unreachable', () => {
    const m = createModel();
    const bar = addBar(m, [0, 0, 0], [1, 0, 0], { onPlaneId: 'plane_top' });
    addJoint(m, 'spherical', { linkId: bar.id, kind: 'vertex', pointIds: [bar.pointIds[0]] }, { constructionId: 'point_origin' });
    const snapshot = serializeModel(m);
    const res = moveVertex(m, bar.pointIds[0], [1, 1, 0]); // pinned to the origin: cannot move
    expect(res.ok).toBe(false);
    expect(res.reason).toBe('unreachable');
    expect(serializeModel(m)).toBe(snapshot);
  });

  it('refuses to edit a locked link', () => {
    const m = createModel();
    const bar = addBar(m, [0, 0, 0], [1, 0, 0]);
    bar.locked = true;
    expect(moveVertex(m, bar.pointIds[1], [2, 0, 0]).reason).toBe('locked');
  });
});

describe('auto-joining coincident vertices (sketching the fourth origami panel)', () => {
  function threePanels() {
    const a = Math.PI / 3;
    const sectors = [a, a, Math.PI - a, Math.PI - a];
    const dirs: Vec3[] = [];
    let ang = 0;
    for (let i = 0; i < 4; i++) {
      dirs.push([Math.cos(ang), Math.sin(ang), 0]);
      ang += sectors[i];
    }
    const folded = foldedVertexPositions(dirs, sectors, 2, 0.6);
    const m = createModel();
    const panels = [0, 1, 2].map((i) => addPolygonFromPoints(m, folded[i], { name: `Panel ${i + 1}` }));
    setGround(m, panels[0].id);
    for (let i = 0; i < 2; i++) {
      addJoint(m, 'revolute', { linkId: panels[i].id, kind: 'edge', pointIds: [panels[i].pointIds[0], panels[i].pointIds[3]] }, { linkId: panels[i + 1].id, kind: 'edge', pointIds: [panels[i + 1].pointIds[0], panels[i + 1].pointIds[1]] });
    }
    return { m, panels, folded };
  }

  it('turns shared edges into creases and yields a 1-DOF rigid vertex', () => {
    const { m, panels, folded } = threePanels();
    const fourth = addPolygonFromPoints(m, folded[3], { name: 'Panel 4' });
    const created = autoJoinCoincident(m, fourth, fourth.pointIds, { defaultJoint: 'revolute', axis: [0, 0, 1] });
    // shares edge O–p0 with panel 3 and edge O–p1 with panel 1: two creases, no vertex pins
    expect(created.length).toBe(2);
    expect(created.every((j) => j.type === 'revolute' && j.a.kind === 'edge' && j.pairs?.length === 2)).toBe(true);
    expect(Object.values(m.joints).filter((j) => j.type === 'revolute').length).toBe(4);
    expect(currentViolation(m)).toBeLessThan(1e-9);
    expect(computeMobility(m).dof).toBe(1);
    void panels;
  });

  it('replaces existing vertex pins by a crease and does not duplicate joints', () => {
    const { m, folded } = threePanels();
    const fourth = addPolygonFromPoints(m, folded[3], { name: 'Panel 4' });
    const first = autoJoinCoincident(m, fourth, [fourth.pointIds[1]], { defaultJoint: 'revolute', axis: [0, 0, 1] });
    expect(first.length).toBe(1); // a single vertex pin (different sketch planes → spherical)
    expect(first[0].type).toBe('spherical');
    const second = autoJoinCoincident(m, fourth, fourth.pointIds, { defaultJoint: 'revolute', axis: [0, 0, 1] });
    expect(second.length).toBe(2);
    expect(Object.values(m.joints).filter((j) => j.a.kind === 'vertex').length).toBe(0); // the pin was upgraded to a crease
    expect(computeMobility(m).dof).toBe(1);
  });
});

describe('sketch plane fitting from snapped vertices', () => {
  const sketch = { o: [0, 0, 0] as Vec3, n: [0, 0, 1] as Vec3 };
  it('uses the sketch plane when nothing is snapped', () => {
    const f = fitSketchPlane([[0, 0, 0], [1, 0, 0], [0, 1, 0]], [false, false, false], sketch);
    expect(f.onSketchPlane).toBe(true);
  });
  it('three snapped vertices define a tilted plane and free vertices are projected onto it', () => {
    const pts: Vec3[] = [[0, 0, 0], [2, 0, 0], [2, 1, 1], [0, 1, 0]];
    const f = fitSketchPlane(pts, [true, true, true, false], sketch);
    expect(f.onSketchPlane).toBe(false);
    const q = projectToPlane(pts[3], f.origin, f.normal);
    expect(dist(q, [0, 0.5, 0.5])).toBeLessThan(1e-9); // plane through the x-axis and (0,1,1)
    expect(f.normal[2]).toBeGreaterThan(0); // oriented like the sketch normal
  });
  it('two snapped vertices give the plane through their edge closest to the sketch plane', () => {
    const f = fitSketchPlane([[0, 0, 0], [1, 0, 1], [0, 1, 0]], [true, true, false], sketch);
    expect(Math.abs(dot3(f.normal, [1, 0, 1]))).toBeLessThan(1e-9);
    expect(f.onSketchPlane).toBe(false);
  });
  it('one snapped vertex gives a parallel plane through it', () => {
    const f = fitSketchPlane([[0, 0, 0.5], [1, 0, 0], [0, 1, 0]], [true, false, false], sketch);
    expect(f.onSketchPlane).toBe(false);
    expect(f.normal).toEqual([0, 0, 1]);
    expect(f.origin[2]).toBe(0.5);
  });
});

function dot3(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

describe('edit tool: joint helpers, hinge axes and re-framing (verification findings)', () => {
  it('lifts a hinged bar out of its plane: helper attachments are released and rebuilt', () => {
    const m = createModel();
    const g = addBar(m, [-2, 0, 0], [-1, 0, 0], { onPlaneId: 'plane_top' });
    setGround(m, g.id);
    const bar = addBar(m, [-1, 0, 0], [1, 0, 0], { onPlaneId: 'plane_top' });
    addJoint(m, 'revolute', { linkId: g.id, kind: 'vertex', pointIds: [g.pointIds[1]] }, { linkId: bar.id, kind: 'vertex', pointIds: [bar.pointIds[0]] });
    const res = moveVertex(m, bar.pointIds[1], [1, 0, 1]);
    expect(res.ok).toBe(true);
    expect(dist(m.points[bar.pointIds[1]].pos, [1, 0, 1])).toBeLessThan(1e-6);
    expect(dist(m.points[bar.pointIds[0]].pos, [-1, 0, 0])).toBeLessThan(1e-9); // hinge unchanged
    expect(currentViolation(m)).toBeLessThan(1e-9);
    // the helper cos attachment was recomputed (no longer the stale perpendicular value)
    const cos = bar.rigidity.find((r) => r.kind === 'cos');
    expect(cos && cos.kind === 'cos' ? Math.abs(cos.value) : 1).toBeGreaterThan(0.1);
  });

  it('lifting a panel hinged to a floating bar does not displace the hinge or the bar', () => {
    const m = createModel();
    const g = addBar(m, [-4, 0, 0], [-3, 0, 0], { onPlaneId: 'plane_top' });
    setGround(m, g.id);
    const bar = addBar(m, [-3, 0, 0], [-1, 0, 0], { onPlaneId: 'plane_top' });
    addJoint(m, 'spherical', { linkId: g.id, kind: 'vertex', pointIds: [g.pointIds[1]] }, { linkId: bar.id, kind: 'vertex', pointIds: [bar.pointIds[0]] });
    const quad = addPolygonFromPoints(m, [[-1, 0, 0], [1, 0, 0], [1, 2, 0], [-1, 2, 0]], { onPlaneId: 'plane_top' });
    addJoint(m, 'revolute', { linkId: bar.id, kind: 'vertex', pointIds: [bar.pointIds[1]] }, { linkId: quad.id, kind: 'vertex', pointIds: [quad.pointIds[0]] });
    const before = quad.pointIds.map((id) => [...m.points[id].pos] as Vec3);
    const barEnd = [...m.points[bar.pointIds[1]].pos] as Vec3;
    const res = moveVertex(m, quad.pointIds[2], [1, 2, 1]);
    expect(res.ok).toBe(true);
    expect(dist(m.points[quad.pointIds[0]].pos, before[0])).toBeLessThan(1e-9);
    expect(dist(m.points[quad.pointIds[1]].pos, before[1])).toBeLessThan(1e-9);
    expect(dist(m.points[quad.pointIds[3]].pos, before[3])).toBeLessThan(1e-9);
    expect(dist(m.points[bar.pointIds[1]].pos, barEnd)).toBeLessThan(1e-9);
    expect(currentViolation(m)).toBeLessThan(1e-9);
  });

  it('pins between links on a shared non-active sketch plane hinge about that plane\'s normal', () => {
    const m = createModel();
    m.settings.sketchPlaneId = 'plane_top';
    const g = addBar(m, [0, 0, 0], [1, 0, 0], { onPlaneId: 'plane_front' });
    setGround(m, g.id);
    const bar = addBar(m, [2, 0, 0], [3, 0, 1], { onPlaneId: 'plane_front' });
    const res = moveVertex(m, bar.pointIds[0], [1, 0, 0], { joinTo: g.pointIds[1] });
    expect(res.ok).toBe(true);
    expect(res.joints.length).toBe(1);
    expect(Math.abs(res.joints[0].axis![1])).toBeCloseTo(1); // FRONT normal, not the active TOP normal
    expect(computeMobility(m).dof).toBe(1);
  });

  it('joins a third link touching a crease-end vertex', () => {
    const m = createModel();
    const A = addPolygonFromPoints(m, [[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0]], { onPlaneId: 'plane_top' });
    setGround(m, A.id);
    const Y = addBar(m, [0, 0, 0], [-1, -1, 0], { onPlaneId: 'plane_top' });
    const B = addPolygonFromPoints(m, [[0, 0, 0], [2, 0, 0], [2, -2, 0], [0, -2, 0]], { onPlaneId: 'plane_top' });
    const created = autoJoinCoincident(m, B, B.pointIds, { defaultJoint: 'revolute', axis: [0, 0, 1] });
    const types = created.map((j) => `${j.type}:${j.a.kind}`).sort();
    expect(types).toEqual(['revolute:edge', 'revolute:vertex']); // crease with A plus a pin to Y
    const toY = created.find((j) => !('constructionId' in j.b) && (j.b as { linkId: string }).linkId === Y.id);
    expect(toY).toBeDefined();
  });

  it('re-framing keeps the other vertices rigid even when the moved vertex spans the largest triangle', () => {
    const m = createModel();
    const quad = addPolygonFromPoints(m, [[-2, 3, 0], [0, 0, 0], [1, 0, 0], [1, 1, 0]], { onPlaneId: 'plane_top' });
    const ids = quad.pointIds;
    const d = (a: number, b: number) => dist(m.points[ids[a]].pos, m.points[ids[b]].pos);
    const before = [d(1, 2), d(2, 3), d(1, 3)];
    const res = moveVertex(m, ids[0], [-2, 3, 1.5]);
    expect(res.ok).toBe(true);
    expect([d(1, 2), d(2, 3), d(1, 3)].map((v, i) => Math.abs(v - before[i]))).toEqual([expect.closeTo(0, 9), expect.closeTo(0, 9), expect.closeTo(0, 9)]);
    expect(m.points[ids[1]].pos[2]).toBeCloseTo(0, 9);
  });

  it('fitSketchPlane never returns a zero normal for two snapped vertices', () => {
    const f = fitSketchPlane([[0, 0, 0], [1, 0, 0], [0, 1, 0]], [true, true, false], { o: [0, 0, 0], n: [1, 0, 0] });
    expect(Math.hypot(...f.normal)).toBeCloseTo(1);
    expect(Math.abs(f.normal[0])).toBeLessThan(1e-9); // perpendicular to the snapped edge along X
  });
});

describe('re-verification follow-ups', () => {
  it('moving the hinge end of a crank keeps the joint axis upright (dyad with a floating bar)', () => {
    const m = createModel();
    const g = addBar(m, [-2, 0, 0], [0, 0, 0], { onPlaneId: 'plane_top' });
    setGround(m, g.id);
    const crank = addBar(m, [0, 0, 0], [1, 1, 0], { onPlaneId: 'plane_top' });
    const floating = addBar(m, [1, 1, 0], [3, 1, 0], { onPlaneId: 'plane_top' });
    addJoint(m, 'revolute', { linkId: g.id, kind: 'vertex', pointIds: [g.pointIds[1]] }, { linkId: crank.id, kind: 'vertex', pointIds: [crank.pointIds[0]] });
    const hinge = addJoint(m, 'revolute', { linkId: crank.id, kind: 'vertex', pointIds: [crank.pointIds[1]] }, { linkId: floating.id, kind: 'vertex', pointIds: [floating.pointIds[0]] })!;
    expect(computeMobility(m).dof).toBe(2);
    const res = moveVertex(m, crank.pointIds[1], [1.2, 1.7, 0]);
    expect(res.ok).toBe(true);
    for (const h of hinge.helpers ?? []) {
      if (!h) continue;
      const base = m.points[h].linkId === crank.id ? crank.pointIds[1] : floating.pointIds[0];
      const off = [0, 1, 2].map((i) => m.points[h].pos[i] - m.points[base].pos[i]);
      expect(Math.abs(off[0])).toBeLessThan(1e-6);
      expect(Math.abs(off[1])).toBeLessThan(1e-6);
      expect(off[2]).toBeCloseTo(1, 6);
    }
    expect(computeMobility(m).dof).toBe(2);
    expect(currentViolation(m)).toBeLessThan(1e-8);
  });

  it('screw joints keep their creation rotation offset so sketch drags stay consistent', () => {
    const m = createModel();
    const bar = addBar(m, [0, 0, 0], [2, 0, 0]);
    const j = addJoint(m, 'screw', { linkId: bar.id, kind: 'vertex', pointIds: [bar.pointIds[0]] }, { constructionId: 'axis_z' }, { pitch: 0.5 })!;
    expect(j.offsets?.angle).toBeDefined();
    expect(currentViolation(m)).toBeLessThan(1e-9);
    // drag the free end ~30° about Z (as the Select tool does: a weak target, then an exact projection on release)
    const a = Math.PI / 6;
    const drag = solveSketch(m, { dragTargets: [{ pointId: bar.pointIds[1], pos: [2 * Math.cos(a), 2 * Math.sin(a), 0], weight: 0.05 }] });
    commitSketch(m, drag);
    const release = solveSketch(m, {});
    commitSketch(m, release);
    expect(currentViolation(m)).toBeLessThan(1e-8);
    const A = m.points[bar.pointIds[0]].pos;
    const B = m.points[bar.pointIds[1]].pos;
    const phi = Math.atan2(B[1] - A[1], B[0] - A[0]);
    expect(phi).toBeGreaterThan(0.3); // the bar did rotate
    // the screw slid the bar along Z by pitch·Δφ/2π
    expect(A[2]).toBeCloseTo((0.5 * phi) / (2 * Math.PI), 6);
    // link–link screw also stores the offset
    const m2 = createModel();
    const base = addBar(m2, [0, 0, 0], [0, 0, 3]);
    setGround(m2, base.id);
    const nut = addBar(m2, [0, 0, 1], [1, 0, 1]);
    const j2 = addJoint(m2, 'screw', { linkId: base.id, kind: 'edge', pointIds: [base.pointIds[0], base.pointIds[1]] }, { linkId: nut.id, kind: 'vertex', pointIds: [nut.pointIds[0]] }, { pitch: 1 })!;
    expect(j2.offsets?.angle).toBeDefined();
    expect(j2.offsets?.slide).toBeCloseTo(1, 9);
  });

  it('free sketch vertices are projected, not flung away, when the pointer ray grazes the fitted plane', () => {
    const origin: Vec3 = [0, 0, 0];
    const normal: Vec3 = [0, 0, 1];
    // ray nearly parallel to the plane, 1 unit above it
    const grazing = { o: [0, 0, 1] as Vec3, d: [1, 0, -0.01] as Vec3 };
    const placed = placeOnFittedPlane([0.3, 0.2, 1], grazing, origin, normal);
    expect(dist(placed, [0.3, 0.2, 0])).toBeLessThan(1e-9);
    // a well-posed ray hits where it should
    const good = { o: [0, 0, 1] as Vec3, d: [1, 0, -1] as Vec3 };
    expect(dist(placeOnFittedPlane([9, 9, 9], good, origin, normal), [1, 0, 0])).toBeLessThan(1e-9);
    // hits behind the ray origin are rejected
    expect(rayPlane([0, 0, 1], [0, 0, 1], origin, normal)).toBeNull();
    expect(rayPlane([0, 0, 1], [0, 0, -1], origin, normal)).toEqual([0, 0, 0]);
  });
});

describe('3-D placement on the sketch plane (plan 2c: placementOnPlane)', () => {
  const origin: Vec3 = [0, 0, 0];
  const normal: Vec3 = [0, 0, 1];
  const fallback: Vec3 = [5, 5, 5];

  it('places a ray that meets the plane squarely at the hit, not at the fallback', () => {
    // the default isometric-ish view: direction (1, -1, 0.9) reversed, 15 units from the origin
    const o: Vec3 = [7, -7, 6.3];
    const d: Vec3 = [-0.4, 0.6, -0.9]; // not normalised: the helper normalises
    const hit = placementOnPlane({ o, d }, origin, normal, fallback);
    expect(hit[2]).toBeCloseTo(0, 12);
    expect(dist(hit, [4.2, -2.8, 0])).toBeLessThan(1e-9); // o + 7·d
    // an off-origin plane with an unnormalised normal
    const hit2 = placementOnPlane({ o: [0, 0, 5], d: [1, 0, -1] }, [9, 9, 2], [0, 0, 4], fallback);
    expect(dist(hit2, [3, 0, 2])).toBeLessThan(1e-9);
  });

  it('returns the fallback for a grazing ray (|cos| below 0.15) and places at exactly the threshold', () => {
    const grazing = { o: [0, 0, 1] as Vec3, d: [1, 0, -0.1] as Vec3 }; // cos ≈ 0.0995
    expect(placementOnPlane(grazing, origin, normal, fallback)).toEqual(fallback);
    const c = PLACEMENT_MIN_COS;
    const atThreshold = { o: [0, 0, 1] as Vec3, d: [Math.sqrt(1 - c * c), 0, -c] as Vec3 }; // |cos| = minCos: not grazing
    expect(placementOnPlane(atThreshold, origin, normal, fallback)[2]).toBeCloseTo(0, 12);
    // the threshold is a parameter
    expect(placementOnPlane(atThreshold, origin, normal, fallback, 0.2)).toEqual(fallback);
  });

  it('returns the fallback for a ray parallel to the plane', () => {
    expect(placementOnPlane({ o: [0, 0, 1], d: [1, 0, 0] }, origin, normal, fallback)).toEqual(fallback);
    // the TOP plane seen exactly edge-on from the Front view (orthographic rays), its origin on the plane
    expect(placementOnPlane({ o: [0, -15, 0], d: [0, 1, 0] }, origin, normal, fallback)).toEqual(fallback);
  });

  it('returns the fallback when the plane lies behind the ray origin', () => {
    expect(placementOnPlane({ o: [0, 0, 1], d: [0, 0, 1] }, origin, normal, fallback)).toEqual(fallback); // straight up: cos = 1
    expect(placementOnPlane({ o: [0, 0, 1], d: [1, 0, 0.5] }, origin, normal, fallback)).toEqual(fallback);
  });

  it('treats a hit at the camera as a miss (Front view: the camera sits on the TOP plane, off-centre rays are not grazing)', () => {
    const view: Vec3 = [0, 0, 0]; // the view plane through the origin, 15 units ahead
    const up = { o: [0, -15, 0] as Vec3, d: [0, 1, 0.3] as Vec3 }; // cos ≈ 0.29 with the TOP normal: not grazing, meets the plane at t = 0
    expect(rayPlane(up.o, up.d, origin, normal)).toEqual(up.o);
    expect(placementOnPlane(up, origin, normal, view)).toEqual(view);
    // a camera a hair below the plane: the hit is a few millimetres in front of it, still "at the camera"
    expect(placementOnPlane({ o: [0, -15, -0.001], d: [0, 1, 0.3] }, origin, normal, view)).toEqual(view);
    // a low view from height 1 clicking below the horizon hits the plane 3.5 units ahead: a real placement
    const low = placementOnPlane({ o: [0, -15, 1], d: [0, 1, -0.3] }, origin, normal, view);
    expect(low[2]).toBeCloseTo(0, 12);
    expect(low[1]).toBeCloseTo(-15 + 1 / 0.3, 9);
  });

  it('keeps placeOnFittedPlane as the same rule with the orthogonal projection as the fallback', () => {
    const p: Vec3 = [0.3, 0.2, 1];
    const grazing = { o: [0, 0, 1] as Vec3, d: [1, 0, -0.01] as Vec3 };
    expect(placeOnFittedPlane(p, grazing, origin, normal)).toEqual(placementOnPlane(grazing, origin, normal, projectToPlane(p, origin, normal)));
    const good = { o: [0, 0, 1] as Vec3, d: [1, 0, -1] as Vec3 };
    expect(placeOnFittedPlane([9, 9, 9], good, origin, normal)).toEqual(placementOnPlane(good, origin, normal, [9, 9, 0]));
    expect(placeOnFittedPlane(p, undefined, origin, normal)).toEqual([0.3, 0.2, 0]);
  });
});

describe('edits that require the hinge axis to tilt', () => {
  it('succeeds when the joint axis must rotate with a spatially constrained partner link', () => {
    const m = createModel();
    const g = addBar(m, [-2, 0, 0], [-1, 0, 0]);
    setGround(m, g.id);
    const bar1 = addBar(m, [-1, 0, 0], [1, 0, 0]);
    const bar2 = addBar(m, [1, 0, 0], [3, 0, 0]);
    addJoint(m, 'revolute', { linkId: g.id, kind: 'vertex', pointIds: [g.pointIds[1]] }, { linkId: bar1.id, kind: 'vertex', pointIds: [bar1.pointIds[0]] }, { axis: [0, 0, 1] });
    const hinge = addJoint(m, 'revolute', { linkId: bar1.id, kind: 'vertex', pointIds: [bar1.pointIds[1]] }, { linkId: bar2.id, kind: 'vertex', pointIds: [bar2.pointIds[0]] }, { axis: [0, 0, 1] })!;
    const anchor = addConstructionPoint(m, [3, 0, 0], 'P');
    addJoint(m, 'spherical', { linkId: bar2.id, kind: 'vertex', pointIds: [bar2.pointIds[1]] }, { constructionId: anchor.id });
    expect(currentViolation(m)).toBeLessThan(1e-9);
    const th = 0.3;
    const dest: Vec3 = [3 - 2 * Math.cos(th), 0, 2 * Math.sin(th)]; // on bar2's sphere about the anchor: bar2 must tilt
    const res = moveVertex(m, bar1.pointIds[1], dest);
    expect(res.ok).toBe(true);
    expect(dist(m.points[bar1.pointIds[1]].pos, dest)).toBeLessThan(1e-6);
    expect(currentViolation(m)).toBeLessThan(1e-8);
    // the hinge helpers followed the tilted partner (they stay perpendicular to bar2)
    const b2 = [0, 1, 2].map((i) => m.points[bar2.pointIds[1]].pos[i] - m.points[bar2.pointIds[0]].pos[i]);
    for (const h of hinge.helpers ?? []) {
      if (!h) continue;
      const base = m.points[h].linkId === bar1.id ? bar1.pointIds[1] : bar2.pointIds[0];
      const off = [0, 1, 2].map((i) => m.points[h].pos[i] - m.points[base].pos[i]);
      expect(Math.abs(off[0] * b2[0] + off[1] * b2[1] + off[2] * b2[2])).toBeLessThan(1e-6);
    }
  });
});
