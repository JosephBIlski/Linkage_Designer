import { describe, expect, it } from 'vitest';
import { addBar, addJoint, addPolygon, addPolygonFromPoints, createModel, setGround } from '../src/core/model';
import { autoJoinCoincident, coincidenceTolerance, coincidentVertex } from '../src/core/edit';
import { mergedPointGroups } from '../src/core/feasibility';
import { sectorPreview, sectorStatus, type SectorPreview } from '../src/core/sector';
import { computeMobility, currentViolation, modelSize } from '../src/core/kinematics';
import { dist } from '../src/core/geometry';
import type { ID, Link, Model, Vec3 } from '../src/core/types';

const O: Vec3 = [0, 0, 0];
/** Point on the circle of radius 2 in the TOP plane at the given angle (degrees). */
const R = (deg: number): Vec3 => [2 * Math.cos((deg * Math.PI) / 180), 2 * Math.sin((deg * Math.PI) / 180), 0];
const SIDE = Math.sqrt(3); // side of a regular triangle with circumradius 1
/** Corner of a regular-triangle fan about the origin: k-th corner at 60°·k, distance √3. */
const C = (k: number): Vec3 => [SIDE * Math.cos((Math.PI / 3) * k), SIDE * Math.sin((Math.PI / 3) * k), 0];
const vertexAt = (m: Model, link: Link, p: Vec3): ID => link.pointIds.reduce((b, id) => (dist(m.points[id].pos, p) < dist(m.points[b].pos, p) ? id : b), link.pointIds[0]);
const creases = (m: Model) => Object.values(m.joints).filter((j) => j.type === 'revolute' && j.a.kind === 'edge' && j.pairs?.length === 2);
const join = (m: Model, link: Link) => autoJoinCoincident(m, link, link.pointIds, { defaultJoint: 'revolute', axis: [0, 0, 1] });

/**
 * Three developable triangles O–p_i–p_{i+1} with sectors 60°, 60°, 120° (the fourth, 120°, is missing), flat on TOP,
 * D1 ground. With `join` each panel is joined to the ones before it as it is added (one crease each), as the Panel
 * tool does.
 */
function threeDevelopable(opts: { join?: boolean } = {}): { m: Model; tris: Link[] } {
  const m = createModel();
  const ang = [0, 60, 120, 240];
  const tris: Link[] = [];
  for (const i of [0, 1, 2]) {
    tris.push(addPolygonFromPoints(m, [O, R(ang[i]), R(ang[i + 1])], { onPlaneId: 'plane_top', name: `D${i + 1}` }));
    if (i === 0) setGround(m, tris[0].id);
    else if (opts.join) expect(join(m, tris[i]).length).toBe(1);
  }
  return { m, tris };
}

/** Three regular triangles O–C(k)–C(k+1), k = 0..2 (sectors 60° each, edges at 0°, 60°, 120°, 180°), flat on TOP, T1 ground. */
function threeRegularFlat(): { m: Model; tris: Link[] } {
  const m = createModel();
  const tris = [0, 1, 2].map((k) => addPolygonFromPoints(m, [O, C(k), C(k + 1)], { onPlaneId: 'plane_top', name: `T${k + 1}` }));
  setGround(m, tris[0].id);
  return { m, tris };
}

/**
 * Three regular triangles around one vertex in 3-D: lateral faces of a square pyramid whose faces are equilateral
 * (base side √3, apex height √3/√2). The fourth face would close the ring with a 60° corner: four 60° sectors, 240°.
 * In a plane a closed ring always sums to 360°, so this is the configuration in which the closing panel's label
 * must turn red.
 */
function threeRegularPyramid(opts: { join?: boolean } = {}): { m: Model; faces: Link[]; apex: Vec3; base: Vec3[] } {
  const m = createModel();
  const half = SIDE / 2;
  const apex: Vec3 = [0, 0, SIDE / Math.SQRT2];
  const base: Vec3[] = [[half, half, 0], [-half, half, 0], [-half, -half, 0], [half, -half, 0]];
  const faces: Link[] = [];
  for (const k of [0, 1, 2]) {
    faces.push(addPolygonFromPoints(m, [apex, base[k], base[k + 1]], { name: `F${k + 1}` }));
    if (k === 0) setGround(m, faces[0].id);
    else if (opts.join) expect(join(m, faces[k]).length).toBe(1);
  }
  return { m, faces, apex, base };
}

describe('sectorPreview: live sector-angle feedback while sketching a panel', () => {
  it('three developable triangles plus the closing panel: 360° and the ring closes (coincident, unjoined panels)', () => {
    const { m, tris } = threeDevelopable();
    const s = sectorPreview(m, O, R(240), R(0), vertexAt(m, tris[0], O));
    expect(s.existingDeg).toBeCloseTo(240, 9);
    expect(s.addedDeg).toBeCloseTo(120, 9);
    expect(s.sumDeg).toBeCloseTo(360, 9);
    expect(s.closesRing).toBe(true);
    expect(sectorStatus(s)).toBe('ok');
  });

  it('counts panels merged through creases once each, whichever member of the group is named', () => {
    const { m, tris } = threeDevelopable({ join: true });
    expect(creases(m).length).toBe(2);
    for (const t of tris) {
      const s = sectorPreview(m, O, R(240), R(0), vertexAt(m, t, O));
      expect(s.existingDeg).toBeCloseTo(240, 9);
      expect(s.sumDeg).toBeCloseTo(360, 9);
      expect(s.closesRing).toBe(true);
    }
    // the open edge follows the cursor: with the cursor off the ring the sum is still running and the ring is open
    const open = sectorPreview(m, O, R(240), R(330), vertexAt(m, tris[2], O));
    expect(open.addedDeg).toBeCloseTo(90, 9);
    expect(open.sumDeg).toBeCloseTo(330, 9);
    expect(open.closesRing).toBe(false);
    expect(sectorStatus(open)).toBe('neutral');
  });

  it('three regular triangles plus a fourth that closes the ring: 240° (a square pyramid, which cannot lie flat)', () => {
    const { m, faces, apex, base } = threeRegularPyramid();
    for (const f of faces) expect(dist(m.points[f.pointIds[1]].pos, m.points[f.pointIds[2]].pos)).toBeCloseTo(SIDE, 9); // equilateral faces
    const s = sectorPreview(m, apex, base[3], base[0], vertexAt(m, faces[0], apex));
    expect(s.existingDeg).toBeCloseTo(180, 9);
    expect(s.addedDeg).toBeCloseTo(60, 9);
    expect(s.sumDeg).toBeCloseTo(240, 9);
    expect(s.closesRing).toBe(true);
    expect(sectorStatus(s)).toBe('bad');
  });

  it('three regular triangles in a flat fan plus a fourth: 240° with the ring still open (two more would close it)', () => {
    const { m, tris } = threeRegularFlat();
    const s = sectorPreview(m, O, C(3), C(4), vertexAt(m, tris[2], O));
    expect(s.existingDeg).toBeCloseTo(180, 9);
    expect(s.addedDeg).toBeCloseTo(60, 9);
    expect(s.sumDeg).toBeCloseTo(240, 9);
    expect(s.closesRing).toBe(false);
    expect(sectorStatus(s)).toBe('neutral');
  });

  it('a panel touching the vertex with only one shared edge leaves the ring open', () => {
    const { m, tris } = threeDevelopable();
    // one edge along the existing edge O–R(240), the other to a free point
    const s = sectorPreview(m, O, R(240), [1, -1.5, 0], vertexAt(m, tris[2], O));
    expect(s.closesRing).toBe(false);
    expect(s.existingDeg).toBeCloseTo(240, 9);
    expect(sectorStatus(s)).toBe('neutral');
    // at a far corner shared by two panels only those two count, and nothing closes there
    const corner = sectorPreview(m, R(60), O, [3, 3, 0], vertexAt(m, tris[0], R(60)));
    expect(corner.existingDeg).toBeCloseTo(60 + 60, 9); // D1 and D2 both have a 60° corner at R(60)
    expect(corner.closesRing).toBe(false);
  });

  it('a sum beyond 360° is bad even while the ring is open; a zero-length edge adds 0°', () => {
    const m = createModel();
    // three unit squares around the origin (270°) on TOP
    addPolygonFromPoints(m, [O, [1, 0, 0], [1, 1, 0], [0, 1, 0]], { onPlaneId: 'plane_top' });
    addPolygonFromPoints(m, [O, [0, 1, 0], [-1, 1, 0], [-1, 0, 0]], { onPlaneId: 'plane_top' });
    const third = addPolygonFromPoints(m, [O, [-1, 0, 0], [-1, -1, 0], [0, -1, 0]], { onPlaneId: 'plane_top' });
    const fits = sectorPreview(m, O, [0, -1, 0], [1, -1, 0], vertexAt(m, third, O)); // 270° + 45°
    expect(fits.sumDeg).toBeCloseTo(315, 9);
    expect(fits.closesRing).toBe(false);
    expect(sectorStatus(fits)).toBe('neutral');
    const tooMuch = sectorPreview(m, O, [-1, -1, 0], [1, -0.2, 0], vertexAt(m, third, O)); // 270° + 146.3°: overlaps
    expect(tooMuch.sumDeg).toBeGreaterThan(360.5);
    expect(tooMuch.closesRing).toBe(false);
    expect(sectorStatus(tooMuch)).toBe('bad');
    const degenerate = sectorPreview(m, O, O, [1, -1, 0], vertexAt(m, third, O));
    expect(degenerate.addedDeg).toBe(0);
    expect(degenerate.sumDeg).toBeCloseTo(270, 9);
  });

  it('ignores bars at the vertex (only polygons and prisms have sector angles) and panels elsewhere', () => {
    const { m, tris } = threeDevelopable();
    addBar(m, O, [0, 0, 3]);
    addPolygonFromPoints(m, [[5, 5, 0], [6, 5, 0], [6, 6, 0]], { onPlaneId: 'plane_top' });
    const s = sectorPreview(m, O, R(240), R(0), vertexAt(m, tris[0], O));
    expect(s.existingDeg).toBeCloseTo(240, 9);
    expect(s.closesRing).toBe(true);
  });

  it('sectorStatus thresholds follow the 0.5° sector tolerance', () => {
    const p = (sumDeg: number, closesRing: boolean): SectorPreview => ({ existingDeg: 0, addedDeg: sumDeg, sumDeg, closesRing });
    expect(sectorStatus(p(360, true))).toBe('ok');
    expect(sectorStatus(p(360.4, true))).toBe('ok');
    expect(sectorStatus(p(359.4, true))).toBe('bad');
    expect(sectorStatus(p(240, true))).toBe('bad');
    expect(sectorStatus(p(240, false))).toBe('neutral');
    expect(sectorStatus(p(360.4, false))).toBe('neutral');
    expect(sectorStatus(p(360.6, false))).toBe('bad');
    expect(sectorStatus(p(360, false))).toBe('neutral'); // 360° with an open ring: nothing closes it yet
  });
});

describe('coincident vertices (Panel and Polygon tools join typed and snapped vertices alike)', () => {
  it('coincidenceTolerance is one part in a million of the model size, never below 1e-6', () => {
    const m = createModel();
    expect(coincidenceTolerance(m)).toBe(1e-6);
    addBar(m, [0, 0, 0], [30, 40, 0]);
    expect(modelSize(m)).toBeCloseTo(50, 9);
    expect(coincidenceTolerance(m)).toBeCloseTo(50e-6, 15);
  });

  it('coincidentVertex finds the nearest vertex within the tolerance, never helpers or the excluded link', () => {
    const { m } = threeDevelopable({ join: true });
    const q = coincidentVertex(m, [1e-7, -1e-7, 0]);
    expect(q).not.toBeNull();
    expect(dist(q!.pos, O)).toBeLessThan(1e-6);
    expect(coincidentVertex(m, [1e-7, 0, 0], { excludeLinkId: q!.linkId })?.linkId).not.toBe(q!.linkId);
    expect(coincidentVertex(m, [0.01, 0, 0])).toBeNull();
    // the helper points of a pin sit next to the hinge (creases merge their end points and need none) but are never a snap target
    const bar = addBar(m, R(0), [4, 0, 0], { onPlaneId: 'plane_top' });
    expect(addJoint(m, 'revolute', { linkId: bar.id, kind: 'vertex', pointIds: [bar.pointIds[0]] }, { linkId: q!.linkId, kind: 'vertex', pointIds: [q!.id] }, { axis: [0, 0, 1] })).not.toBeNull();
    const helpers = Object.values(m.points).filter((p) => p.role === 'helper');
    expect(helpers.length).toBeGreaterThan(0);
    for (const h of helpers) expect(coincidentVertex(m, h.pos)).toBeNull();
  });

  it('mergedPointGroups lists the points merged by creases and pins; a lone point has no group', () => {
    const { m, tris } = threeDevelopable({ join: true });
    const groups = mergedPointGroups(m);
    const o1 = vertexAt(m, tris[0], O);
    const g = groups.group(o1)!;
    expect(g).toContain(o1);
    expect(g).toContain(vertexAt(m, tris[1], O));
    expect(g).toContain(vertexAt(m, tris[2], O));
    expect(new Set(g.map((id) => m.points[id].linkId)).size).toBe(3);
    expect(groups.group(vertexAt(m, tris[0], R(0)))).toBeNull();
  });

  it('finishSketch core path: a fourth panel built from typed (unsnapped) coordinates gets its creases', () => {
    const { m } = threeDevelopable({ join: true });
    const fourth = addPolygonFromPoints(m, [O, R(240), R(0)], { onPlaneId: 'plane_top', name: 'D4' });
    const created = join(m, fourth);
    expect(created.length).toBe(2);
    expect(created.every((j) => j.type === 'revolute' && j.a.kind === 'edge' && j.pairs?.length === 2)).toBe(true);
    expect(creases(m).length).toBe(4);
    expect(Object.values(m.joints).filter((j) => j.a.kind === 'vertex').length).toBe(0); // no stray pins at the shared vertices
    expect(currentViolation(m)).toBeLessThan(1e-9);
    expect(computeMobility(m).dof).toBe(0); // flat and kept on the sketch plane (the Fold command lifts it)
  });

  it('typed coordinates rounded to seven decimals still count as coincident and the sketched panel adapts', () => {
    const { m } = threeDevelopable({ join: true });
    const round = (p: Vec3): Vec3 => p.map((x) => Math.round(x * 1e7) / 1e7) as Vec3;
    const fourth = addPolygonFromPoints(m, [round(O), round(R(240)), round(R(0))], { onPlaneId: 'plane_top', name: 'D4' });
    expect(join(m, fourth).length).toBe(2);
    expect(creases(m).length).toBe(4);
    expect(currentViolation(m)).toBeLessThan(1e-9);
    // the existing panels did not move: the sketched panel took their edge lengths
    for (const id of Object.values(m.links).filter((l) => l.name !== 'D4').flatMap((l) => l.pointIds)) {
      const p = m.points[id].pos;
      expect(Math.min(dist(p, O), dist(p, R(0)), dist(p, R(60)), dist(p, R(120)), dist(p, R(240)))).toBeLessThan(1e-9);
    }
  });

  it('the closing face of the pyramid joins with two creases and a consistent 3-D vertex', () => {
    const { m, apex, base } = threeRegularPyramid({ join: true });
    const fourth = addPolygonFromPoints(m, [apex, base[3], base[0]], { name: 'F4' });
    expect(join(m, fourth).length).toBe(2);
    expect(creases(m).length).toBe(4);
    expect(currentViolation(m)).toBeLessThan(1e-9);
  });

  it('Polygon tool core path: a regular triangle whose first and second vertices land on an existing triangle becomes a crease', () => {
    // T1 as the Polygon tool draws it: centre at the centroid of O, C(0), C(1), first vertex typed at C(1)
    const m = createModel();
    const centroid = (a: Vec3, b: Vec3): Vec3 => [(a[0] + b[0]) / 3, (a[1] + b[1]) / 3, 0];
    const c0 = centroid(C(0), C(1));
    const t1 = addPolygon(m, c0, [0, 0, 1], dist(c0, C(1)), 3, { xDir: [C(1)[0] - c0[0], C(1)[1] - c0[1], 0], onPlaneId: 'plane_top', name: 'T1' });
    expect(dist(m.points[t1.pointIds[1]].pos, O)).toBeLessThan(1e-12);
    expect(dist(m.points[t1.pointIds[2]].pos, C(0))).toBeLessThan(1e-12);
    setGround(m, t1.id);
    // T2: centre at the centroid of O, C(1), C(2), first vertex typed at O (on T1's V1), second lands on C(1) (T1's V0)
    const c1 = centroid(C(1), C(2));
    const t2 = addPolygon(m, c1, [0, 0, 1], dist(c1, O), 3, { xDir: [O[0] - c1[0], O[1] - c1[1], 0], onPlaneId: 'plane_top', name: 'T2' });
    expect(dist(m.points[t2.pointIds[0]].pos, O)).toBeLessThan(1e-12);
    expect(dist(m.points[t2.pointIds[1]].pos, C(1))).toBeLessThan(1e-12);
    const joints = autoJoinCoincident(m, t2, t2.pointIds, { defaultJoint: 'revolute', axis: [0, 0, 1], extraPairs: [[t2.pointIds[0], t1.pointIds[1]]] });
    expect(joints.length).toBe(1);
    expect(joints[0].type).toBe('revolute');
    expect(joints[0].a.kind).toBe('edge');
    expect(joints[0].pairs?.length).toBe(2);
    expect(currentViolation(m)).toBeLessThan(1e-9);
    // a snapped first vertex that nothing else touches is still pinned through the extra pair (the old behaviour)
    const t3 = addPolygon(m, [5, 5, 0], [0, 0, 1], 1, 3, { xDir: [1, 0, 0], onPlaneId: 'plane_top', name: 'T3' });
    const pin = autoJoinCoincident(m, t3, t3.pointIds, { defaultJoint: 'revolute', axis: [0, 0, 1], extraPairs: [[t3.pointIds[0], t1.pointIds[2]]] });
    expect(pin.length).toBe(1);
    expect(pin[0].a.kind).toBe('vertex');
    expect(dist(m.points[t3.pointIds[0]].pos, C(0))).toBeLessThan(1e-7); // the solve pulled T3 onto the pinned vertex
    expect(currentViolation(m)).toBeLessThan(1e-8);
  });
});
