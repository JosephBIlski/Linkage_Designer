import { describe, expect, it } from 'vitest';
import { addBar, addPolygon, addPolygonFromPoints, bodyPlaneJoint, createModel, linkFaces, setGround, addJoint } from '../src/core/model';
import { duplicateLink, extrudePolygon, linearArray, mirrorAcrossPlane, polarArray, polygonNormal, translation } from '../src/core/patterns';
import { computeMobility, currentViolation } from '../src/core/kinematics';
import { dist } from '../src/core/geometry';

describe('patterning', () => {
  it('copies a bar with a translation and keeps the sketch-plane constraint', () => {
    const m = createModel();
    const bar = addBar(m, [0, 0, 0], [2, 0, 0], { onPlaneId: 'plane_top' });
    const copy = duplicateLink(m, bar, translation([0, 1, 0]), 'Copy')!;
    expect(dist(m.points[copy.pointIds[0]].pos, [0, 1, 0])).toBeLessThan(1e-12);
    expect(dist(m.points[copy.pointIds[1]].pos, [2, 1, 0])).toBeLessThan(1e-12);
    expect(bodyPlaneJoint(m, copy.id)).not.toBeNull();
    expect(Object.keys(m.links).length).toBe(2);
  });

  it('mirrors a polygon across the RIGHT plane and preserves edge lengths', () => {
    const m = createModel();
    const poly = addPolygonFromPoints(m, [[1, 0, 0], [3, 0, 0], [2, 1.5, 0]], { onPlaneId: 'plane_top' });
    const mirrored = duplicateLink(m, poly, mirrorAcrossPlane([0, 0, 0], [1, 0, 0]), 'Mirror')!;
    const p = mirrored.pointIds.map((id) => m.points[id].pos);
    expect(p[0][0]).toBeCloseTo(-1);
    expect(p[1][0]).toBeCloseTo(-3);
    expect(dist(p[0], p[1])).toBeCloseTo(2);
    expect(bodyPlaneJoint(m, mirrored.id)).not.toBeNull(); // still on TOP
    expect(currentViolation(m)).toBeLessThan(1e-9);
  });

  it('drops the sketch-plane constraint when the copy leaves the plane', () => {
    const m = createModel();
    const bar = addBar(m, [0, 0, 0], [2, 0, 0], { onPlaneId: 'plane_top' });
    const copy = duplicateLink(m, bar, translation([0, 0, 1]), 'Lifted')!;
    expect(bodyPlaneJoint(m, copy.id)).toBeNull();
  });

  it('builds linear and polar arrays', () => {
    const m = createModel();
    const bar = addBar(m, [1, 0, 0], [2, 0, 0]);
    const lin = linearArray(m, bar, [0, 1, 0], 3, (i) => `L${i}`);
    expect(lin.length).toBe(3);
    expect(m.points[lin[2].pointIds[0]].pos[1]).toBeCloseTo(3);
    const pol = polarArray(m, bar, [0, 0, 0], [0, 0, 1], 3, (i) => `P${i}`);
    expect(pol.length).toBe(3);
    // quarter turns: first copy of (1,0,0) goes to (0,1,0)
    expect(dist(m.points[pol[0].pointIds[0]].pos, [0, 1, 0])).toBeLessThan(1e-9);
    expect(Object.keys(m.links).length).toBe(7);
  });

  it('extrudes a polygon into a rigid prism and keeps attached joints valid', () => {
    const m = createModel();
    const ground = addBar(m, [-2, 0, 0], [-1, 0, 0], { onPlaneId: 'plane_top' });
    setGround(m, ground.id);
    const poly = addPolygon(m, [0, 0, 0], [0, 0, 1], 1, 4, { onPlaneId: 'plane_top' });
    addJoint(m, 'revolute', { linkId: ground.id, kind: 'vertex', pointIds: [ground.pointIds[1]] }, { linkId: poly.id, kind: 'vertex', pointIds: [poly.pointIds[2]] });
    expect(polygonNormal(m, poly)![2]).toBeCloseTo(1);
    expect(extrudePolygon(m, poly, 0.5)).toBe(true);
    expect(poly.kind).toBe('prism');
    expect(poly.pointIds.length).toBe(8);
    expect(linkFaces(m, poly).length).toBe(6);
    expect(bodyPlaneJoint(m, poly.id)).toBeNull();
    expect(currentViolation(m)).toBeLessThan(1e-9);
    // a prism pinned by one revolute joint rotates about that pin: 1 DOF
    expect(computeMobility(m).dof).toBe(1);
  });
});
