import { describe, expect, it } from 'vitest';
import { addBar, addJoint, addPolygon, addPolygonFromPoints, bodyPlaneJoint, createModel, serializeModel, setGround } from '../src/core/model';
import { autoJoinCoincident, fitSketchPlane, moveVertex, projectToPlane } from '../src/core/edit';
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
