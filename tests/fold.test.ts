import { describe, expect, it } from 'vitest';
import { addBar, addFoldDriver, addJoint, addPolygonFromPoints, bodyPlaneJoint, createModel, serializeModel, setGround } from '../src/core/model';
import { tryAddJoint } from '../src/core/feasibility';
import { computeMobility, currentViolation, sweepDriver } from '../src/core/kinematics';
import { origamiMiuraVertex } from '../src/core/examples';
import {
  COLLINEAR_TOLERANCE_DEG,
  DEFAULT_PREFOLD_DEG,
  FLAT_TOLERANCE_DEG,
  creaseDihedralDeg,
  creaseMountainValley,
  findCreaseLoops,
  foldCreaseTo,
  isCrease,
  isFlatDihedral,
  loopNormal,
  prefoldVertex,
} from '../src/core/fold';
import { jointDihedralDeg } from '../src/core/jointMeasure';
import { dist } from '../src/core/geometry';
import type { Feature, ID, Link, Model, Vec3 } from '../src/core/types';

/** Edge feature of `link` whose end points are nearest to p and q. */
function edgeNear(m: Model, link: Link, p: Vec3, q: Vec3): Feature {
  const nearest = (x: Vec3): ID => link.pointIds.reduce((b, id) => (dist(m.points[id].pos, x) < dist(m.points[b].pos, x) ? id : b), link.pointIds[0]);
  return { linkId: link.id, kind: 'edge', pointIds: [nearest(p), nearest(q)] };
}

const O: Vec3 = [0, 0, 0];
const vertex = (link: Link, i: number): Feature => ({ linkId: link.id, kind: 'vertex', pointIds: [link.pointIds[i]] });

/**
 * Four flat triangles O–p_i–p_{i+1} around the origin on a circle of radius 2, consecutive ones joined into creases
 * with tryAddJoint (exact merges, nothing moves); the first triangle is ground. `anglesDeg` are the directions of
 * the four creases, so [0, 60, 120, 240] gives the developable (60°, 60°, 120°, 120°) vertex and [0, 90, 180, 270]
 * the "X" vertex. With `planar` every triangle carries the sketch-plane (2-D) constraint, as the Polygon tool adds.
 */
function flatVertex(anglesDeg: number[], opts: { planar?: boolean; creases?: number } = {}): { m: Model; tris: Link[]; p: (i: number) => Vec3 } {
  const m = createModel();
  const ang = anglesDeg.map((d) => (d * Math.PI) / 180);
  const p = (i: number): Vec3 => [2 * Math.cos(ang[i % 4]), 2 * Math.sin(ang[i % 4]), 0];
  const tris = [0, 1, 2, 3].map((i) => addPolygonFromPoints(m, [O, p(i), p(i + 1)], { onPlaneId: opts.planar ? 'plane_top' : null, name: `D${i + 1}` }));
  setGround(m, tris[0].id);
  for (let i = 0; i < (opts.creases ?? 4); i++) {
    const r = tryAddJoint(m, 'revolute', edgeNear(m, tris[i], O, p(i + 1)), edgeNear(m, tris[(i + 1) % 4], O, p(i + 1)));
    expect(r.ok).toBe(true);
    expect(r.joint?.pairs?.length).toBe(2);
  }
  return { m, tris, p };
}

const developable = (opts: { planar?: boolean; creases?: number } = {}) => flatVertex([0, 60, 120, 240], opts);
const xVertex = (opts: { planar?: boolean } = {}) => flatVertex([0, 90, 180, 270], opts);

const foldAngles = (m: Model, ids: ID[]): number[] => ids.map((id) => creaseDihedralDeg(m, m.joints[id])!);

describe('creases and their dihedral', () => {
  it('isCrease: edge–edge revolute with two merged pairs only', () => {
    const { m } = developable({ creases: 1 });
    const crease = Object.values(m.joints).find((j) => j.type === 'revolute')!;
    expect(isCrease(m, crease)).toBe(true);
    // a vertex pin, a planar body joint and a hinge between edges of different length are not creases
    const bar = addBar(m, [5, 0, 0], [7, 0, 0]);
    const other = addBar(m, [5, 0, 0], [5, 2, 0]);
    const pin = addJoint(m, 'revolute', vertex(bar, 0), vertex(other, 0), { axis: [0, 0, 1] })!;
    expect(isCrease(m, pin)).toBe(false);
    const bar3 = addBar(m, [5, 0, 0], [8, 0, 0]);
    const hinge = addJoint(m, 'revolute', { linkId: bar.id, kind: 'edge', pointIds: bar.pointIds }, { linkId: bar3.id, kind: 'edge', pointIds: bar3.pointIds })!;
    expect(hinge.pairs).toBeUndefined();
    expect(isCrease(m, hinge)).toBe(false);
    const planar = developable({ planar: true, creases: 0 }).m;
    const body = Object.values(planar.joints).find((j) => j.type === 'planar')!;
    expect(isCrease(planar, body)).toBe(false);
  });

  it('creaseDihedralDeg is ±180° for an unfolded crease, equals the fold-driver measurement and is null for non-creases', () => {
    const { m } = developable({ creases: 1 });
    const crease = Object.values(m.joints).find((j) => j.type === 'revolute')!;
    const d = creaseDihedralDeg(m, crease)!;
    expect(Math.abs(d)).toBeCloseTo(180, 9);
    expect(isFlatDihedral(d)).toBe(true);
    expect(isFlatDihedral(180 - FLAT_TOLERANCE_DEG - 0.01)).toBe(false);
    expect(d).toBe(jointDihedralDeg(m, crease));
    const miura = origamiMiuraVertex(60);
    const driven = miura.joints[miura.drivers[0].jointId!];
    expect(creaseDihedralDeg(miura, driven)).toBeCloseTo(miura.drivers[0].value, 9);
    const bar = addBar(m, [5, 0, 0], [7, 0, 0]);
    const pin = addJoint(m, 'revolute', vertex(bar, 0), vertex(Object.values(m.links)[0], 0), { axis: [0, 0, 1] })!;
    expect(creaseDihedralDeg(m, pin)).toBeNull();
  });
});

describe('findCreaseLoops', () => {
  it('a) reports the developable vertex as one flat loop of four creases with exactly one collinear pair', () => {
    const { m, tris } = developable();
    const loops = findCreaseLoops(m);
    expect(loops.length).toBe(1);
    const [loop] = loops;
    expect(loop.flat).toBe(true);
    expect(loop.creaseIds.length).toBe(4);
    expect(loop.linkIds.length).toBe(4);
    expect(new Set(loop.linkIds)).toEqual(new Set(tris.map((t) => t.id)));
    expect(loop.collinearPairs.length).toBe(1);
    // the collinear creases are the ones along 60° and 240° (D1–D2 and D3–D4)
    const joints = Object.values(m.joints).filter((j) => j.type === 'revolute');
    expect(loop.collinearPairs[0].sort()).toEqual([joints[0].id, joints[2].id].sort());
    // the vertex point belongs to the ground panel; the loop order is cyclic: linkIds[i] joins creaseIds[i] and [i+1]
    expect(m.links[m.points[loop.vertexPointId].linkId].ground).toBe(true);
    for (let i = 0; i < 4; i++) {
      const a = m.joints[loop.creaseIds[i]];
      const b = m.joints[loop.creaseIds[(i + 1) % 4]];
      const linksOf = (j: typeof a) => [j.a.linkId, (j.b as Feature).linkId];
      expect(linksOf(a)).toContain(loop.linkIds[i]);
      expect(linksOf(b)).toContain(loop.linkIds[i]);
    }
    expect(loopNormal(m, loop)).toEqual([0, 0, 1]);
  });

  it('an open fan of three creases is not a loop; the fourth crease closes it', () => {
    const { m, tris, p } = developable({ creases: 3 });
    expect(findCreaseLoops(m)).toEqual([]);
    expect(tryAddJoint(m, 'revolute', edgeNear(m, tris[3], O, p(4)), edgeNear(m, tris[0], O, p(4))).ok).toBe(true);
    expect(findCreaseLoops(m).length).toBe(1);
  });

  it('the X vertex has two collinear pairs; the Miura example is one loop that is not flat', () => {
    const x = xVertex();
    const [xl] = findCreaseLoops(x.m);
    expect(xl.flat).toBe(true);
    expect(xl.collinearPairs.length).toBe(2);
    const miura = origamiMiuraVertex(60);
    const loops = findCreaseLoops(miura);
    expect(loops.length).toBe(1);
    expect(loops[0].flat).toBe(false);
    expect(loops[0].creaseIds.length).toBe(4);
    expect(COLLINEAR_TOLERANCE_DEG).toBe(1);
  });
});

describe('prefoldVertex', () => {
  it('a) pre-folds the developable vertex onto the generic branch: dihedrals (170, 160, 170, 160), DOF 1, every panel sweeps', () => {
    const { m, tris } = developable();
    expect(computeMobility(m).dof).toBe(2); // the flat state is a bifurcation
    const [loop] = findCreaseLoops(m);
    const r = prefoldVertex(m);
    expect(r.ok).toBe(true);
    expect(r.reason).toBeUndefined();
    expect(r.removedPlanes).toEqual([]);
    expect(r.dof).toBe(1);
    expect(computeMobility(m).dof).toBe(1);
    expect(currentViolation(m)).toBeLessThan(1e-9);
    // the driven crease is not one of the collinear pair and sits at ±160°
    expect(loop.collinearPairs.flat()).not.toContain(r.creaseId);
    expect(Math.abs(r.dihedrals![r.creaseId!])).toBeCloseTo(DEFAULT_PREFOLD_DEG, 6);
    const expected = [170, 160, 170, 160];
    const got = loop.creaseIds.map((id) => Math.abs(r.dihedrals![id]));
    got.forEach((g, i) => expect(Math.abs(g - expected[i])).toBeLessThan(2));
    expect(foldAngles(m, loop.creaseIds).map(Math.abs)).toEqual(got);
    // Maekawa: three mountains and one valley (or the reverse) seen from the loop normal
    const mv = loop.creaseIds.map((id) => creaseMountainValley(m, m.joints[id], loopNormal(m, loop)));
    expect(mv.filter((x) => x === 'M').length === 3 || mv.filter((x) => x === 'V').length === 3).toBe(true);
    // the model had no driver: the fold driver on the chosen crease is kept, and a sweep moves every panel
    expect(m.drivers.length).toBe(1);
    expect(m.drivers[0].kind).toBe('fold');
    expect(m.drivers[0].jointId).toBe(r.creaseId);
    expect(m.drivers[0].value).toBeCloseTo(r.dihedrals![r.creaseId!], 9);
    const sweep = sweepDriver(m, 0);
    expect(sweep.poses.length).toBeGreaterThan(10);
    for (const t of tris.slice(1)) {
      const far = t.pointIds.filter((id) => dist(m.points[id].pos, O) > 1);
      const move = Math.max(...far.map((pid) => Math.max(...sweep.poses.map((pose) => dist(pose.positions.get(pid)!, m.points[pid].pos)))));
      expect(move).toBeGreaterThan(1);
    }
    // on the generic branch every crease folds through a substantial range
    for (const id of loop.creaseIds) {
      const angles = sweep.poses.map((pose) => creaseDihedralDeg(m, m.joints[id], pose.positions)!);
      expect(Math.max(...angles.map((v) => 180 - Math.abs(v))) - Math.min(...angles.map((v) => 180 - Math.abs(v)))).toBeGreaterThan(60);
    }
  });

  it('b) removes the four sketch-plane constraints of a vertex drawn in 2-D and succeeds', () => {
    const { m, tris } = developable({ planar: true });
    expect(computeMobility(m).dof).toBe(0); // held flat by the 2-D constraints
    expect(tris.every((t) => bodyPlaneJoint(m, t.id) !== null)).toBe(true);
    const r = prefoldVertex(m);
    expect(r.ok).toBe(true);
    expect(r.removedPlanes!.length).toBe(4);
    expect(tris.every((t) => bodyPlaneJoint(m, t.id) === null)).toBe(true);
    expect(r.removedPlanes!.every((id) => m.joints[id] === undefined)).toBe(true);
    expect(r.dof).toBe(1);
    expect(computeMobility(m).dof).toBe(1);
    expect(currentViolation(m)).toBeLessThan(1e-9);
    // the ground panel did not move; the others left the plane
    for (const pid of tris[0].pointIds) expect(Math.abs(m.points[pid].pos[2])).toBeLessThan(1e-9);
    expect(Math.max(...tris.slice(1).flatMap((t) => t.pointIds.map((pid) => Math.abs(m.points[pid].pos[2]))))).toBeGreaterThan(0.1);
  });

  it('c) refuses the X vertex (two collinear pairs: only straight-hinge branches) and leaves the model byte-identical', () => {
    const { m } = xVertex({ planar: true });
    const before = serializeModel(m);
    const r = prefoldVertex(m);
    expect(r.ok).toBe(false);
    expect(r.reason).toBe('noBranch');
    expect(r.creaseId).toBeUndefined();
    expect(serializeModel(m)).toBe(before); // planar joints, positions, drivers and the id counter restored
  });

  it('d) leaves the Miura example alone: its loop is not flat', () => {
    const m = origamiMiuraVertex(60);
    const before = serializeModel(m);
    const r = prefoldVertex(m);
    expect(r.ok).toBe(false);
    expect(r.reason).toBe('notFlat');
    expect(serializeModel(m)).toBe(before);
  });

  it('reports noLoop without creases, notFlat for a given folded loop and locked for a locked panel', () => {
    const open = developable({ creases: 3 });
    expect(prefoldVertex(open.m)).toEqual({ ok: false, reason: 'noLoop' });
    const miura = origamiMiuraVertex(60);
    const [loop] = findCreaseLoops(miura);
    expect(prefoldVertex(miura, { loop }).reason).toBe('notFlat');
    expect(prefoldVertex(miura, { preferCreaseId: loop.creaseIds[0] }).reason).toBe('notFlat');
    const { m, tris } = developable();
    tris[2].locked = true;
    const before = serializeModel(m);
    expect(prefoldVertex(m)).toEqual({ ok: false, reason: 'locked' });
    expect(serializeModel(m)).toBe(before);
    tris[2].locked = false;
    tris[0].locked = true; // a locked ground panel does not move anyway
    expect(prefoldVertex(m).ok).toBe(true);
  });

  it('honours the preferred crease when it is not collinear and skips it in favour of a non-collinear one when it is', () => {
    const a = developable();
    const [loopA] = findCreaseLoops(a.m);
    const nonCollinear = loopA.creaseIds.filter((id) => !loopA.collinearPairs.flat().includes(id));
    const rA = prefoldVertex(a.m, { preferCreaseId: nonCollinear[1] });
    expect(rA.ok).toBe(true);
    expect(rA.creaseId).toBe(nonCollinear[1]);
    const b = developable();
    const [loopB] = findCreaseLoops(b.m);
    const rB = prefoldVertex(b.m, { preferCreaseId: loopB.collinearPairs[0][0] });
    expect(rB.ok).toBe(true);
    expect(loopB.collinearPairs.flat()).not.toContain(rB.creaseId);
  });

  it('uses the requested sign, angle and mountain / valley preference', () => {
    const base = developable();
    const [loop] = findCreaseLoops(base.m);
    const cid = loop.creaseIds.filter((id) => !loop.collinearPairs.flat().includes(id))[0];
    const snapshot = serializeModel(base.m);
    const fresh = (): Model => {
      const m = createModel();
      Object.assign(m, JSON.parse(snapshot));
      return m;
    };
    const plus = fresh();
    expect(prefoldVertex(plus, { preferCreaseId: cid, sign: 1, angleDeg: 150 }).dihedrals![cid]).toBeCloseTo(150, 6);
    const minus = fresh();
    expect(prefoldVertex(minus, { preferCreaseId: cid, sign: -1, angleDeg: 150 }).dihedrals![cid]).toBeCloseTo(-150, 6);
    for (const pref of ['M', 'V'] as const) {
      const m = fresh();
      const r = prefoldVertex(m, { preferCreaseId: cid, mountainValley: { [cid]: pref } });
      expect(r.ok).toBe(true);
      expect(r.creaseId).toBe(cid);
      expect(creaseMountainValley(m, m.joints[cid], loopNormal(m, loop))).toBe(pref);
    }
  });

  it('releases a fold driver that already sits on a crease of the loop instead of holding the vertex flat', () => {
    const { m } = developable();
    const [loop] = findCreaseLoops(m);
    const collinear = loop.collinearPairs[0][0];
    const d = addFoldDriver(m, collinear)!;
    d.value = creaseDihedralDeg(m, m.joints[collinear])!;
    const r = prefoldVertex(m);
    expect(r.ok).toBe(true);
    expect(m.drivers.length).toBe(1); // the model had a driver: no second one is kept
    expect(m.drivers[0].id).toBe(d.id);
    expect(Math.abs(m.drivers[0].value)).toBeCloseTo(Math.abs(r.dihedrals![collinear]), 9); // re-measured
  });
});

describe('foldCreaseTo', () => {
  it('e) folds a crease of the pre-folded developable vertex to 120° (within 0.1°)', () => {
    const { m } = developable();
    const r = prefoldVertex(m);
    expect(r.ok).toBe(true);
    const cid = r.creaseId!;
    const sign = Math.sign(creaseDihedralDeg(m, m.joints[cid])!);
    const f = foldCreaseTo(m, cid, sign * 120);
    expect(f.ok).toBe(true);
    expect(Math.abs(Math.abs(creaseDihedralDeg(m, m.joints[cid])!) - 120)).toBeLessThan(0.1);
    expect(currentViolation(m)).toBeLessThan(1e-9);
    // the model's own fold driver on that crease follows (its value is re-measured, no extra driver is left behind)
    expect(m.drivers.length).toBe(1);
    expect(Math.abs(m.drivers[0].value)).toBeCloseTo(120, 6);
    // another crease of the loop can be folded while the first stays where its driver holds it
    const [loop] = findCreaseLoops(m);
    const other = loop.creaseIds.find((id) => id !== cid && !loop.collinearPairs.flat().includes(id))!;
    const before = creaseDihedralDeg(m, m.joints[other])!;
    const g = foldCreaseTo(m, other, Math.sign(before) * 120);
    expect(g.ok).toBe(false); // the driven crease is held at 120°, which fixes the whole 1-DOF vertex
    expect(creaseDihedralDeg(m, m.joints[other])).toBeCloseTo(before, 9);
  });

  it('restores the model byte for byte when the target cannot be reached', () => {
    const { m } = developable();
    expect(prefoldVertex(m).ok).toBe(true);
    m.drivers = []; // no driver: a temporary one is used and removed again
    const [loop] = findCreaseLoops(m);
    const cid = loop.creaseIds[1];
    const before = serializeModel(m);
    // a bar pinned between two far vertices of different panels forbids any further folding
    const pa = m.points[m.links[loop.linkIds[1]].pointIds[1]].pos;
    const pb = m.points[m.links[loop.linkIds[2]].pointIds[2]].pos;
    const bar = addBar(m, pa, pb);
    addJoint(m, 'spherical', vertex(bar, 0), vertex(m.links[loop.linkIds[1]], 1));
    addJoint(m, 'spherical', vertex(bar, 1), vertex(m.links[loop.linkIds[2]], 2));
    const locked = serializeModel(m);
    const cur = creaseDihedralDeg(m, m.joints[cid])!;
    const f = foldCreaseTo(m, cid, Math.sign(cur) * 100);
    expect(f.ok).toBe(false);
    expect(serializeModel(m)).toBe(locked);
    expect(before).not.toBe(locked);
    expect(foldCreaseTo(m, 'joint_nonexistent', 100)).toEqual({ ok: false, residual: Infinity });
  });

  it('on the flat X vertex a collinear crease folds as a straight hinge: the opposite pair stays at 180°', () => {
    const { m } = xVertex();
    const [loop] = findCreaseLoops(m);
    const f = foldCreaseTo(m, loop.creaseIds[0], 160);
    expect(f.ok).toBe(true);
    const angles = foldAngles(m, loop.creaseIds).map(Math.abs);
    expect(angles[0]).toBeCloseTo(160, 6);
    expect(angles[2]).toBeCloseTo(160, 6);
    expect(angles[1]).toBeCloseTo(180, 6);
    expect(angles[3]).toBeCloseTo(180, 6);
  });
});
