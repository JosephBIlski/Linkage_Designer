import { describe, expect, it } from 'vitest';
import {
  addBar,
  addConstructionAxis,
  addCylinder,
  addJoint,
  addPolygonFromPoints,
  addPrism,
  bodyPlaneJoint,
  createModel,
  creasesLockedByPlane,
  serializeModel,
  setBodyPlane,
  setGround,
} from '../src/core/model';
import { compile } from '../src/core/compile';
import { tryAddJoint, trySolveCommit } from '../src/core/feasibility';
import { computeMobility, currentViolation } from '../src/core/kinematics';
import { foldedVertexPositions } from '../src/core/examples';
import { findCreaseLoops } from '../src/core/fold';
import { SIM } from '../src/ui/strings';
import { dist } from '../src/core/geometry';
import type { Feature, ID, Link, Model, Vec3 } from '../src/core/types';
import { isConstructionRef } from '../src/core/types';

/** Edge feature of `link` whose end points are nearest to p and q. */
function edgeNear(m: Model, link: Link, p: Vec3, q: Vec3): Feature {
  const nearest = (x: Vec3): ID => link.pointIds.reduce((b, id) => (dist(m.points[id].pos, x) < dist(m.points[b].pos, x) ? id : b), link.pointIds[0]);
  return { linkId: link.id, kind: 'edge', pointIds: [nearest(p), nearest(q)] };
}

const vertex = (link: Link, i: number): Feature => ({ linkId: link.id, kind: 'vertex', pointIds: [link.pointIds[i]] });
const bodyJoints = (m: Model) => Object.values(m.joints).filter((j) => j.type === 'planar' && j.a.kind === 'body');
const revolutes = (m: Model): ID[] => Object.values(m.joints).filter((j) => j.type === 'revolute').map((j) => j.id).sort();
const planeOf = (m: Model, linkId: ID): ID | null => {
  const bp = bodyPlaneJoint(m, linkId);
  return bp && isConstructionRef(bp.b) ? bp.b.constructionId : null;
};
/** Which coordinates compile freezes for every visible point of the link ([x, y, z] flags, all points agree). */
function frozenAxes(m: Model, link: Link): [boolean, boolean, boolean] {
  const sys = compile(m, { mode: 'forward', poses: 1, includeDrivers: false, flexibility: false });
  const flags = link.pointIds.map((id) => sys.groupOf(id).frozen);
  for (const f of flags) expect(f).toEqual(flags[0]);
  return flags[0];
}

const O: Vec3 = [0, 0, 0];

/** Four developable triangles O–p_i–p_{i+1} with sectors 60°, 60°, 120°, 120° on a circle of radius 2, flat, 2-D. */
function fourDevelopableTriangles(): { m: Model; tris: Link[]; p: (i: number) => Vec3 } {
  const m = createModel();
  const ang = [0, 60, 120, 240, 360].map((d) => (d * Math.PI) / 180);
  const p = (i: number): Vec3 => [2 * Math.cos(ang[i]), 2 * Math.sin(ang[i]), 0];
  const tris = [0, 1, 2, 3].map((i) => addPolygonFromPoints(m, [O, p(i), p(i + 1)], { onPlaneId: 'plane_top', name: `D${i + 1}` }));
  setGround(m, tris[0].id);
  return { m, tris, p };
}

/** The developable vertex with all four creases closed (zero residual, DOF 0 in 2-D). */
function closedDevelopableVertex(): { m: Model; tris: Link[] } {
  const { m, tris, p } = fourDevelopableTriangles();
  for (let i = 0; i < 4; i++) {
    const r = tryAddJoint(m, 'revolute', edgeNear(m, tris[i], O, p(i + 1)), edgeNear(m, tris[(i + 1) % 4], O, p(i + 1)));
    expect(r.ok).toBe(true);
  }
  return { m, tris };
}

/**
 * Miura vertex (α, α, π−α, π−α) built in its exactly folded configuration (foldedVertexPositions, as the Sketch-tool
 * workflow leaves it): only the ground panel lies in TOP and carries the 2-D constraint.
 */
function foldedVertex(): { m: Model; panels: Link[] } {
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
  const panels = [0, 1, 2, 3].map((i) => addPolygonFromPoints(m, folded[i], { name: `Panel ${i + 1}`, onPlaneId: i === 0 ? 'plane_top' : null }));
  setGround(m, panels[0].id);
  for (let i = 0; i < 4; i++) {
    const pa = panels[i];
    const pb = panels[(i + 1) % 4];
    expect(addJoint(m, 'revolute', { linkId: pa.id, kind: 'edge', pointIds: [pa.pointIds[0], pa.pointIds[3]] }, { linkId: pb.id, kind: 'edge', pointIds: [pb.pointIds[0], pb.pointIds[1]] })).not.toBeNull();
  }
  return { m, panels };
}

describe('setBodyPlane', () => {
  it('adds the body planar joint of a 2-D link and compile freezes the plane coordinate', () => {
    const m = createModel();
    const tri = addPolygonFromPoints(m, [[0, 0, 0], [2, 0, 0], [1, 1, 0]]);
    expect(bodyPlaneJoint(m, tri.id)).toBeNull();
    expect(frozenAxes(m, tri)).toEqual([false, false, false]);
    const j = setBodyPlane(m, tri.id, 'plane_top');
    expect(j).not.toBeNull();
    expect(j!.type).toBe('planar');
    expect(j!.a).toEqual({ linkId: tri.id, kind: 'body', pointIds: tri.pointIds });
    expect(j!.b).toEqual({ constructionId: 'plane_top' });
    expect(bodyPlaneJoint(m, tri.id)?.id).toBe(j!.id);
    expect(bodyJoints(m).length).toBe(1);
    expect(frozenAxes(m, tri)).toEqual([false, false, true]); // TOP is z = 0
    expect(currentViolation(m)).toBeLessThan(1e-12);
  });

  it('removes it with null and the round trip leaves no joint and no frozen coordinate', () => {
    const m = createModel();
    const tri = addPolygonFromPoints(m, [[0, 0, 0], [2, 0, 0], [1, 1, 0]]);
    const before = serializeModel(m);
    const added = setBodyPlane(m, tri.id, 'plane_top')!;
    expect(setBodyPlane(m, tri.id, null)).toBeNull();
    expect(bodyPlaneJoint(m, tri.id)).toBeNull();
    expect(Object.keys(m.joints).length).toBe(0);
    expect(m.joints[added.id]).toBeUndefined();
    expect(frozenAxes(m, tri)).toEqual([false, false, false]);
    // identical apart from the id counter the removed joint consumed
    expect(serializeModel(m).replace(/"nextId": \d+/, '')).toBe(before.replace(/"nextId": \d+/, ''));
    expect(setBodyPlane(m, tri.id, null)).toBeNull(); // removing twice is harmless
  });

  it('is exactly what drawing the link in 2-D mode does (finishLink)', () => {
    const drawn = createModel();
    addBar(drawn, [0, 0, 0], [1, 0, 0], { onPlaneId: 'plane_top', name: 'bar' });
    const toggled = createModel();
    const bar = addBar(toggled, [0, 0, 0], [1, 0, 0], { name: 'bar' });
    setBodyPlane(toggled, bar.id, 'plane_top');
    expect(serializeModel(toggled)).toBe(serializeModel(drawn));
  });

  it('replaces an existing constraint so a link is on at most one plane', () => {
    const m = createModel();
    const bar = addBar(m, [0, 0, 0], [1, 0, 0], { onPlaneId: 'plane_top' });
    const first = bodyPlaneJoint(m, bar.id)!;
    const j = setBodyPlane(m, bar.id, 'plane_front');
    expect(j).not.toBeNull();
    expect(j!.id).not.toBe(first.id);
    expect(m.joints[first.id]).toBeUndefined();
    expect(bodyJoints(m).length).toBe(1);
    expect(planeOf(m, bar.id)).toBe('plane_front');
    expect(frozenAxes(m, bar)).toEqual([false, true, false]); // FRONT is y = 0
  });

  it('refuses 3-D links and unknown planes without touching an existing constraint', () => {
    const m = createModel();
    const prism = addPrism(m, [0, 0, 0], [0, 0, 1], 1, 4, 1);
    const cyl = addCylinder(m, [0, 0, 0], [0, 0, 2], 0.3);
    expect(setBodyPlane(m, prism.id, 'plane_top')).toBeNull();
    expect(setBodyPlane(m, cyl.id, 'plane_top')).toBeNull();
    expect(bodyJoints(m).length).toBe(0);
    const bar = addBar(m, [0, 0, 0], [1, 0, 0], { onPlaneId: 'plane_top' });
    const existing = bodyPlaneJoint(m, bar.id)!;
    expect(setBodyPlane(m, bar.id, 'no_such_plane')?.id).toBe(existing.id);
    expect(setBodyPlane(m, bar.id, 'axis_x')?.id).toBe(existing.id); // an axis is not a plane
    expect(setBodyPlane(m, 'no_such_link', 'plane_top')).toBeNull();
    expect(bodyJoints(m).length).toBe(1);
    expect(m.joints[existing.id]).toBe(existing);
  });

  it('ticking the box on a link parallel to the plane re-solves it onto the plane (the Properties path)', () => {
    const m = createModel();
    const tri = addPolygonFromPoints(m, [[0, 0, 1], [2, 0, 1], [1, 1, 1]]);
    setBodyPlane(m, tri.id, 'plane_top');
    const r = trySolveCommit(m, {});
    expect(r.ok).toBe(true);
    for (const id of tri.pointIds) expect(Math.abs(m.points[id].pos[2])).toBeLessThan(1e-9);
    expect(dist(m.points[tri.pointIds[0]].pos, m.points[tri.pointIds[1]].pos)).toBeCloseTo(2, 9); // shape kept
    expect(currentViolation(m)).toBeLessThan(1e-9);
  });

  it('a tilted free panel is simply rotated onto the plane', () => {
    const m = createModel();
    const tri = addPolygonFromPoints(m, [[0, 0, 0], [2, 0, 0], [1, 1, 1]]);
    setBodyPlane(m, tri.id, 'plane_top');
    expect(trySolveCommit(m, {}).ok).toBe(true);
    for (const id of tri.pointIds) expect(Math.abs(m.points[id].pos[2])).toBeLessThan(1e-9);
    expect(dist(m.points[tri.pointIds[2]].pos, m.points[tri.pointIds[0]].pos)).toBeCloseTo(Math.sqrt(3), 7); // rigid
    expect(currentViolation(m)).toBeLessThan(1e-9);
  });

  it('a link held off the plane by another constraint cannot be flattened: refused and rolled back', () => {
    const m = createModel();
    const bar = addBar(m, [0, 0, 0], [1, 0, 1]);
    addJoint(m, 'spherical', vertex(bar, 0), { constructionId: 'point_origin' });
    const rail = addConstructionAxis(m, [0, 0, 1], [1, 0, 0]); // B has to stay on the line z = 1
    expect(addJoint(m, 'cylindrical', vertex(bar, 1), { constructionId: rail.id })).not.toBeNull();
    expect(currentViolation(m)).toBeLessThan(1e-9);
    const snapshot = serializeModel(m);
    setBodyPlane(m, bar.id, 'plane_top');
    const r = trySolveCommit(m, {}, snapshot);
    expect(r.ok).toBe(false);
    expect(r.residual).toBeGreaterThan(0.1);
    expect(serializeModel(m)).toBe(snapshot);
    expect(bodyPlaneJoint(m, bar.id)).toBeNull();
  });
});

describe('creasesLockedByPlane', () => {
  it('reports the four creases of a flat developable vertex whose panels are all kept on TOP', () => {
    const { m } = closedDevelopableVertex();
    expect(currentViolation(m)).toBeLessThan(1e-12);
    expect(creasesLockedByPlane(m).sort()).toEqual(revolutes(m));
    expect(revolutes(m).length).toBe(4);
    expect(computeMobility(m).dof).toBe(0);
  });

  it('a crease is free as soon as one of its panels leaves the plane', () => {
    const { m, tris } = closedDevelopableVertex();
    setBodyPlane(m, tris[1].id, null);
    const locked = creasesLockedByPlane(m);
    expect(locked.length).toBe(2);
    for (const id of locked) {
      const j = m.joints[id];
      expect(j.a.linkId).not.toBe(tris[1].id);
      expect(!isConstructionRef(j.b) && j.b.linkId).not.toBe(tris[1].id);
    }
    for (const t of tris) setBodyPlane(m, t.id, null);
    expect(creasesLockedByPlane(m)).toEqual([]);
    expect(computeMobility(m).dof).toBeGreaterThan(0); // the flat vertex can fold once the 2-D constraint is gone
  });

  it('two triangles sharing an edge: locked on a common plane, free once one is released', () => {
    const m = createModel();
    const A = addPolygonFromPoints(m, [[0, 0, 0], [2, 0, 0], [1, 1.5, 0]], { onPlaneId: 'plane_top', name: 'A' });
    const B = addPolygonFromPoints(m, [[0, 0, 0], [2, 0, 0], [1, -1.5, 0]], { onPlaneId: 'plane_top', name: 'B' });
    setGround(m, A.id);
    const r = tryAddJoint(m, 'revolute', edgeNear(m, A, [0, 0, 0], [2, 0, 0]), edgeNear(m, B, [0, 0, 0], [2, 0, 0]));
    expect(r.ok).toBe(true);
    expect(creasesLockedByPlane(m)).toEqual([r.joint!.id]);
    expect(computeMobility(m).dof).toBe(0);
    // the hint for this model (no flat crease loop, so no Fold button) must not point at Fold
    expect(findCreaseLoops(m).some((l) => l.flat)).toBe(false);
    expect(SIM.creasesLocked(1, false)).not.toMatch(/Fold/);
    expect(SIM.creasesLocked(1, false)).toMatch(/^1 crease cannot fold/);
    expect(SIM.creasesLocked(4, true)).toMatch(/4 creases cannot fold.*or use Fold\.$/);
    setBodyPlane(m, B.id, null);
    expect(creasesLockedByPlane(m)).toEqual([]);
    expect(computeMobility(m).dof).toBe(1);
  });

  it('returns [] for a pre-folded vertex (only the ground panel lies on the sketch plane)', () => {
    const { m } = foldedVertex();
    expect(currentViolation(m)).toBeLessThan(1e-9);
    expect(revolutes(m).length).toBe(4);
    expect(creasesLockedByPlane(m)).toEqual([]);
    expect(computeMobility(m).dof).toBe(1);
  });

  it('ignores vertex pins, hinges between edges of different length and unmerged joints', () => {
    const m = createModel();
    const g = addBar(m, [0, 0, 0], [4, 0, 0], { onPlaneId: 'plane_top' });
    const crank = addBar(m, [0, 0, 0], [0, 2, 0], { onPlaneId: 'plane_top' });
    setGround(m, g.id);
    addJoint(m, 'revolute', vertex(g, 0), vertex(crank, 0), { axis: [0, 0, 1] }); // pin about the plane normal
    expect(creasesLockedByPlane(m)).toEqual([]);
    expect(computeMobility(m).dof).toBe(1);
    // a slide-locked hinge (edges of clearly different length, no merged pairs) is not a crease
    const A = addPolygonFromPoints(m, [[10, 0, 0], [12, 0, 0], [11, 1.5, 0]], { onPlaneId: 'plane_top' });
    const B = addPolygonFromPoints(m, [[10, 0, 0], [13, 0, 0], [11, -1.5, 0]], { onPlaneId: 'plane_top' });
    const hinge = addJoint(m, 'revolute', edgeNear(m, A, [10, 0, 0], [12, 0, 0]), edgeNear(m, B, [10, 0, 0], [13, 0, 0]))!;
    expect(hinge.pairs).toBeUndefined();
    expect(creasesLockedByPlane(m)).toEqual([]);
    // the same edges as a cylindrical joint are not a revolute either
    const C = addPolygonFromPoints(m, [[20, 0, 0], [22, 0, 0], [21, 1.5, 0]], { onPlaneId: 'plane_top' });
    const D = addPolygonFromPoints(m, [[20, 0, 0], [22, 0, 0], [21, -1.5, 0]], { onPlaneId: 'plane_top' });
    expect(addJoint(m, 'cylindrical', edgeNear(m, C, [20, 0, 0], [22, 0, 0]), edgeNear(m, D, [20, 0, 0], [22, 0, 0]))).not.toBeNull();
    expect(creasesLockedByPlane(m)).toEqual([]);
  });
});
