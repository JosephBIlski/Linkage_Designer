import { describe, expect, it } from 'vitest';
import { addBar, addFoldDriver, addJoint, addPolygonFromPoints, bodyPlaneJoint, createModel, jointAxisPoints, offAxisPoint, parseModel, serializeModel, setGround } from '../src/core/model';
import { tryAddJoint } from '../src/core/feasibility';
import { computeMobility, currentViolation, sweepDriver, syncDriverValues } from '../src/core/kinematics';
import { origamiMiuraVertex } from '../src/core/examples';
import {
  creaseDihedralDeg,
  creaseMV,
  creaseMountainValley,
  creaseNormal,
  creaseTargetDihedral,
  driveCrease,
  findCreaseLoops,
  foldCreaseToTarget,
  isCrease,
  isFlatDihedral,
  prefoldVertex,
} from '../src/core/fold';
import { cross, dist, dot, sub } from '../src/core/geometry';
import type { Feature, ID, Joint, Link, Model, Vec3 } from '../src/core/types';
import { PANEL, creaseLabel } from '../src/ui/strings';
import { DEFAULT_SETTINGS, loadSettings } from '../src/ui/settings';

/** Edge feature of `link` whose end points are nearest to p and q. */
function edgeNear(m: Model, link: Link, p: Vec3, q: Vec3): Feature {
  const nearest = (x: Vec3): ID => link.pointIds.reduce((b, id) => (dist(m.points[id].pos, x) < dist(m.points[b].pos, x) ? id : b), link.pointIds[0]);
  return { linkId: link.id, kind: 'edge', pointIds: [nearest(p), nearest(q)] };
}

const O: Vec3 = [0, 0, 0];
const vertex = (link: Link, i: number): Feature => ({ linkId: link.id, kind: 'vertex', pointIds: [link.pointIds[i]] });

/**
 * Four flat triangles O–p_i–p_{i+1} (counter-clockwise seen from +z) around the origin, consecutive ones joined into
 * creases with tryAddJoint; the first triangle is ground. Crease i (creation order) lies along direction
 * anglesDeg[(i + 1) % 4], so for the developable vertex [0, 60, 120, 240] the creases are along 60°, 120°, 240° and
 * 0°: the first lies between the two 60° sectors, the third between the two 120° sectors, and those two are the
 * collinear pair. With `planar` every triangle carries the sketch-plane (2-D) constraint, as the Polygon tool adds.
 */
function flatVertex(anglesDeg: number[], opts: { planar?: boolean } = {}): { m: Model; tris: Link[]; creases: Joint[] } {
  const m = createModel();
  const ang = anglesDeg.map((d) => (d * Math.PI) / 180);
  const p = (i: number): Vec3 => [2 * Math.cos(ang[i % 4]), 2 * Math.sin(ang[i % 4]), 0];
  const tris = [0, 1, 2, 3].map((i) => addPolygonFromPoints(m, [O, p(i), p(i + 1)], { onPlaneId: opts.planar ? 'plane_top' : null, name: `D${i + 1}` }));
  setGround(m, tris[0].id);
  const creases: Joint[] = [];
  for (let i = 0; i < 4; i++) {
    const r = tryAddJoint(m, 'revolute', edgeNear(m, tris[i], O, p(i + 1)), edgeNear(m, tris[(i + 1) % 4], O, p(i + 1)));
    expect(r.ok).toBe(true);
    creases.push(r.joint!);
  }
  return { m, tris, creases };
}

const developable = (opts: { planar?: boolean } = {}) => flatVertex([0, 60, 120, 240], opts);

const classes = (m: Model, ids: ID[]): ('M' | 'V' | null)[] => ids.map((id) => creaseMV(m, m.joints[id]));
const counts = (cls: ('M' | 'V' | null)[]): number[] => [cls.filter((c) => c === 'M').length, cls.filter((c) => c === 'V').length].sort();
const noneFlat = (m: Model, ids: ID[]): boolean => ids.every((id) => !isFlatDihedral(creaseDihedralDeg(m, m.joints[id])!));

describe('mountain / valley convention (creaseMV)', () => {
  it('is null while the vertex is flat and, once folded, is the sign of the dihedral seen from the first panel\'s face normal', () => {
    const { m, creases } = developable();
    for (const j of creases) expect(creaseMV(m, j)).toBeNull();
    expect(prefoldVertex(m).ok).toBe(true);
    for (const j of creases) {
      // counter-clockwise panels on TOP: the face normal is the sketch normal although the stored edges point outward
      const n = creaseNormal(m, j)!;
      expect(n[2]).toBeGreaterThan(0.9);
      const mv = creaseMV(m, j)!;
      expect(mv).toBe(creaseMountainValley(m, j, n));
      // θ > 0 puts panel b on the side of (a1 − a0) × (offA − a0); valley when that side is the normal's side
      const d = creaseDihedralDeg(m, j)!;
      const [a0, a1] = jointAxisPoints(m, j, 'a')!;
      const offA = offAxisPoint(m, m.links[j.a.linkId], [a0, a1])!;
      const side = dot(cross(sub(m.points[a1].pos, m.points[a0].pos), sub(m.points[offA].pos, m.points[a0].pos)), n);
      expect(mv).toBe(d > 0 === side > 0 ? 'V' : 'M');
      // for the ground panel's creases the class is unambiguous: panel b rises (valley) or dips (mountain) from z = 0
      const b = j.b as Feature;
      if (m.links[j.a.linkId].ground || m.links[b.linkId].ground) {
        const other = m.links[j.a.linkId].ground ? m.links[b.linkId] : m.links[j.a.linkId];
        const far = other.pointIds.find((id) => dist(m.points[id].pos, O) > 1 && !j.a.pointIds.includes(id) && !b.pointIds.includes(id))!;
        expect(mv).toBe(m.points[far].pos[2] > 0 ? 'V' : 'M');
      }
    }
  });

  it('developable vertex: the crease between the two equal 60° sectors is the odd one (Maekawa 3:1): the bent pair share a class, the collinear pair differ', () => {
    const { m, creases } = developable();
    const [loop] = findCreaseLoops(m); // collinearity is measured in the flat pose
    expect(loop.collinearPairs[0].slice().sort()).toEqual([creases[0].id, creases[2].id].sort());
    expect(prefoldVertex(m).ok).toBe(true);
    const [c60, c120, c240, c0] = classes(m, creases.map((j) => j.id));
    expect(c0).toBe(c120); // the bent (zigzag) pair fold to the same side
    expect(c60).not.toBe(c240); // the collinear pair have opposite classes
    expect(c240).toBe(c120); // so the crease between the two small sectors is the single odd one
    expect(counts([c60, c120, c240, c0])).toEqual([1, 3]);
  });

  it('Miura example: the same structure in its closed-form pose; reversing a stored edge flips the dihedral sign but not the class', () => {
    const m = origamiMiuraVertex(60);
    const creases = Object.values(m.joints).filter((j) => j.type === 'revolute');
    expect(creases.length).toBe(4);
    const [loop] = findCreaseLoops(m);
    expect(loop.flat).toBe(false);
    // crease i joins panel i and i + 1 along direction i + 1 (0°, 60°, 120°, 240°): creases 0 and 2 are the collinear pair
    const cls = classes(m, creases.map((j) => j.id));
    expect(cls.every((c) => c !== null)).toBe(true);
    expect(cls[3]).toBe(cls[1]);
    expect(cls[0]).not.toBe(cls[2]);
    expect(counts(cls)).toEqual([1, 3]);
    for (const j of creases) {
      const d = creaseDihedralDeg(m, j)!;
      const mv = creaseMV(m, j);
      j.a = { ...j.a, pointIds: [j.a.pointIds[1], j.a.pointIds[0]] };
      expect(creaseDihedralDeg(m, j)).toBeCloseTo(-d, 9);
      expect(creaseMV(m, j)).toBe(mv);
    }
  });

  it('is null for joints that are not creases', () => {
    const { m, tris } = developable();
    const bar = addBar(m, [5, 0, 0], [7, 0, 0]);
    const pin = addJoint(m, 'revolute', vertex(bar, 0), vertex(tris[1], 1), { axis: [0, 0, 1] })!;
    expect(creaseMV(m, pin)).toBeNull();
    expect(creaseTargetDihedral(m, pin, 120)).not.toBeNull(); // a pin has an axis and a side; only the class needs a crease
    const planar = developable({ planar: true }).m;
    const body = Object.values(planar.joints).find((j) => j.type === 'planar')!;
    expect(creaseMV(planar, body)).toBeNull();
  });
});

describe('Joint.fold persistence', () => {
  it('round-trips through serializeModel / parseModel and is simply absent in files written without it', () => {
    const { m, creases } = developable();
    creases[1].fold = { target: 120, mv: 'M' };
    creases[2].fold = { mv: 'V' };
    const text = serializeModel(m);
    const back = parseModel(text);
    expect(back.joints[creases[1].id].fold).toEqual({ target: 120, mv: 'M' });
    expect(back.joints[creases[2].id].fold).toEqual({ mv: 'V' });
    expect(back.joints[creases[0].id].fold).toBeUndefined();
    expect(serializeModel(back)).toBe(text);
    // a model that never had the field (as every file before this version) loads unchanged and its creases work
    const legacy = JSON.parse(serializeModel(origamiMiuraVertex(60))) as { joints: Record<ID, Record<string, unknown>> };
    expect(Object.values(legacy.joints).every((j) => !('fold' in j))).toBe(true);
    const loaded = parseModel(JSON.stringify(legacy));
    expect(Object.values(loaded.joints).every((j) => j.fold === undefined)).toBe(true);
    expect(isCrease(loaded, loaded.joints[loaded.drivers[0].jointId!])).toBe(true);
    expect(creaseMV(loaded, loaded.joints[loaded.drivers[0].jointId!])).not.toBeNull();
  });
});

describe('creaseTargetDihedral and foldCreaseToTarget', () => {
  it('creaseTargetDihedral gives ±|target| with the sign of the requested class (current class, or valley when flat, by default)', () => {
    const { m, creases } = developable();
    const j = creases[1];
    expect(creaseTargetDihedral(m, j, 150)).toBe(creaseTargetDihedral(m, j, 150, 'V'));
    expect(creaseTargetDihedral(m, j, -150, 'M')).toBe(-creaseTargetDihedral(m, j, 150, 'V')!);
    expect(creaseTargetDihedral(m, j, 500, 'V')).toBe(creaseTargetDihedral(m, j, 180, 'V'));
    expect(prefoldVertex(m).ok).toBe(true);
    const mv = creaseMV(m, j)!;
    const d = creaseDihedralDeg(m, j)!;
    expect(creaseTargetDihedral(m, j, 120)).toBe(Math.sign(d) * 120);
    expect(creaseTargetDihedral(m, j, 120, mv)).toBe(Math.sign(d) * 120);
    expect(creaseTargetDihedral(m, j, 120, mv === 'M' ? 'V' : 'M')).toBe(-Math.sign(d) * 120);
  });

  it('a) folds a crease of the pre-folded vertex to 120° in its current class, then to 100°; the model\'s fold driver follows', () => {
    const { m } = developable();
    const r = prefoldVertex(m);
    const cid = r.creaseId!;
    const mv = creaseMV(m, m.joints[cid])!;
    const f = foldCreaseToTarget(m, cid, 120, mv);
    expect(f.ok).toBe(true);
    expect(f.creaseId).toBe(cid);
    expect(f.dof).toBe(1);
    expect(f.removedPlanes).toEqual([]);
    expect(Math.abs(creaseDihedralDeg(m, m.joints[cid])!)).toBeCloseTo(120, 6);
    expect(creaseMV(m, m.joints[cid])).toBe(mv);
    expect(currentViolation(m)).toBeLessThan(1e-9);
    expect(Object.keys(f.dihedrals!).length).toBe(4);
    expect(m.drivers.length).toBe(1);
    expect(Math.abs(m.drivers[0].value)).toBeCloseTo(120, 6);
    expect(foldCreaseToTarget(m, cid, 100).ok).toBe(true); // no class given: the current one is kept
    expect(Math.abs(creaseDihedralDeg(m, m.joints[cid])!)).toBeCloseTo(100, 6);
    expect(creaseMV(m, m.joints[cid])).toBe(mv);
  });

  it('b) on a flat vertex drawn in 2-D it pre-folds first: every crease, both classes, planar joints removed, DOF 1, no crease left flat', () => {
    for (const k of [0, 1, 2, 3]) {
      for (const mv of ['M', 'V'] as const) {
        const { m, tris } = developable({ planar: true });
        const [loop] = findCreaseLoops(m);
        const id = loop.creaseIds[k];
        const f = foldCreaseToTarget(m, id, 150, mv);
        expect(f.ok).toBe(true);
        expect(f.creaseId).toBe(id);
        expect(f.removedPlanes!.length).toBe(4);
        expect(tris.every((t) => bodyPlaneJoint(m, t.id) === null)).toBe(true);
        expect(Math.abs(creaseDihedralDeg(m, m.joints[id])!)).toBeCloseTo(150, 6);
        expect(creaseMV(m, m.joints[id])).toBe(mv);
        expect(noneFlat(m, loop.creaseIds)).toBe(true);
        expect(f.dof).toBe(1);
        expect(computeMobility(m).dof).toBe(1);
        expect(currentViolation(m)).toBeLessThan(1e-9);
        // a driverless model keeps the pre-fold's driver (as the Fold command does), re-measured
        expect(m.drivers.length).toBe(1);
        expect(Math.abs(m.drivers[0].value)).toBeCloseTo(Math.abs(creaseDihedralDeg(m, m.joints[m.drivers[0].jointId!])!), 9);
      }
    }
  });

  it('mirrors a folded vertex when the other class is requested: unfolds through flat, then pre-folds to that side (every crease)', () => {
    for (const k of [0, 1, 2, 3]) {
      const { m } = developable();
      const [loop] = findCreaseLoops(m);
      expect(prefoldVertex(m).ok).toBe(true);
      const id = loop.creaseIds[k];
      const before = classes(m, loop.creaseIds);
      const flip = before[k] === 'M' ? 'V' : 'M';
      const f = foldCreaseToTarget(m, id, 150, flip);
      expect(f.ok).toBe(true);
      expect(Math.abs(creaseDihedralDeg(m, m.joints[id])!)).toBeCloseTo(150, 6);
      expect(creaseMV(m, m.joints[id])).toBe(flip);
      // the whole vertex is the mirror image: every class swapped, nothing left flat, still one DOF
      expect(classes(m, loop.creaseIds)).toEqual(before.map((c) => (c === 'M' ? 'V' : 'M')));
      expect(noneFlat(m, loop.creaseIds)).toBe(true);
      expect(f.dof).toBe(1);
      expect(currentViolation(m)).toBeLessThan(1e-9);
      expect(m.drivers.length).toBe(1);
    }
  });

  it('unfolds a vertex when the target is 180° and accepts a flat target on a flat vertex without changing anything', () => {
    const { m } = developable();
    const [loop] = findCreaseLoops(m);
    expect(prefoldVertex(m).ok).toBe(true);
    const u = foldCreaseToTarget(m, loop.creaseIds[1], 180);
    expect(u.ok).toBe(true);
    // one unfolded crease of a generic degree-4 vertex means the flat state, reached up to the flat tolerance (the
    // solver may stop a fraction of a degree along the straight-hinge branch that passes through it)
    expect(Math.abs(creaseDihedralDeg(m, m.joints[loop.creaseIds[1]])!)).toBeCloseTo(180, 6);
    for (const id of loop.creaseIds) expect(isFlatDihedral(creaseDihedralDeg(m, m.joints[id])!)).toBe(true);
    expect(classes(m, loop.creaseIds)).toEqual([null, null, null, null]);
    expect(findCreaseLoops(m)[0].flat).toBe(true);
    const flat = developable({ planar: true });
    const before = serializeModel(flat.m);
    expect(foldCreaseToTarget(flat.m, flat.creases[0].id, 180, 'M').ok).toBe(true);
    expect(serializeModel(flat.m)).toBe(before);
  });

  it('flips the class of a plain hinge (open fan, no loop) at the same fold angle by passing through flat: M 90° → V 90° and back', () => {
    const m = createModel();
    const A = addPolygonFromPoints(m, [[0, 0, 0], [2, 0, 0], [1, 2, 0]], { onPlaneId: null });
    const B = addPolygonFromPoints(m, [[0, 0, 0], [1, -2, 0], [2, 0, 0]], { onPlaneId: null });
    setGround(m, A.id);
    const j = tryAddJoint(m, 'revolute', edgeNear(m, A, O, [2, 0, 0]), edgeNear(m, B, O, [2, 0, 0])).joint!;
    expect(findCreaseLoops(m).length).toBe(0);
    const steps: [number, 'M' | 'V'][] = [[90, 'M'], [90, 'V'], [90, 'M'], [45, 'V'], [170, 'M'], [10, 'V'], [10, 'M'], [120, 'V']];
    for (const [angle, mv] of steps) {
      const f = foldCreaseToTarget(m, j.id, angle, mv);
      expect(f).toMatchObject({ ok: true, creaseId: j.id, removedPlanes: [] });
      expect(Math.abs(creaseDihedralDeg(m, j)!)).toBeCloseTo(angle, 5);
      expect(creaseMV(m, j)).toBe(mv);
      expect(currentViolation(m)).toBeLessThan(1e-9);
      expect(m.drivers.length).toBe(0); // no driver is left behind on a driverless model without a loop
    }
    // fully closed (0°, the panels coincide, no class) and back open to flat (180°): the half turn is taken in steps
    expect(foldCreaseToTarget(m, j.id, 0, 'V').ok).toBe(true);
    expect(Math.abs(creaseDihedralDeg(m, j)!)).toBeLessThan(1e-5);
    expect(foldCreaseToTarget(m, j.id, 180).ok).toBe(true);
    expect(Math.abs(creaseDihedralDeg(m, j)!)).toBeCloseTo(180, 5);
    expect(foldCreaseToTarget(m, j.id, 60, 'M').ok).toBe(true);
    expect(creaseMV(m, j)).toBe('M');
  });

  it('flips the middle crease of a three-panel fan at the same angle while the other crease is held by its driver', () => {
    const { m, tris } = developable({ planar: false });
    // keep only two creases: D1–D2 and D2–D3 (an open fan), D1 ground
    const [loop] = findCreaseLoops(m);
    const keep = loop.creaseIds.slice(0, 2);
    for (const id of loop.creaseIds) if (!keep.includes(id)) delete m.joints[id];
    expect(findCreaseLoops(m)).toEqual([]);
    const [first, middle] = keep.map((id) => m.joints[id]);
    expect(foldCreaseToTarget(m, first.id, 150, 'M').ok).toBe(true);
    expect(foldCreaseToTarget(m, middle.id, 100, 'V').ok).toBe(true);
    addFoldDriver(m, first.id); // holds the first crease where it is (a driverless hinge may move with the solve)
    syncDriverValues(m);
    const held = creaseDihedralDeg(m, first)!;
    for (const mv of ['M', 'V', 'M'] as const) {
      const f = foldCreaseToTarget(m, middle.id, 100, mv);
      expect(f.ok).toBe(true);
      expect(Math.abs(creaseDihedralDeg(m, middle)!)).toBeCloseTo(100, 5);
      expect(creaseMV(m, middle)).toBe(mv);
      expect(creaseDihedralDeg(m, first)).toBeCloseTo(held, 5); // not on the loop (there is none): the driver holds it
      expect(currentViolation(m)).toBeLessThan(1e-9);
      expect(m.drivers.length).toBe(1);
    }
    void tris;
  });

  it('c) releases a fold driver that sits on another crease of the vertex and re-measures it', () => {
    const { m } = developable();
    const r = prefoldVertex(m);
    const [loop] = findCreaseLoops(m);
    const other = loop.creaseIds.find((id) => id !== r.creaseId && !loop.collinearPairs.flat().includes(id))!;
    expect(m.drivers[0].jointId).toBe(r.creaseId);
    const f = foldCreaseToTarget(m, other, 110);
    expect(f.ok).toBe(true);
    expect(Math.abs(creaseDihedralDeg(m, m.joints[other])!)).toBeCloseTo(110, 6);
    expect(m.drivers.length).toBe(1);
    expect(m.drivers[0].jointId).toBe(r.creaseId);
    expect(Math.abs(m.drivers[0].value)).toBeCloseTo(Math.abs(creaseDihedralDeg(m, m.joints[r.creaseId!])!), 9);
    expect(currentViolation(m)).toBeLessThan(1e-9);
  });

  it('d) is refused with the model byte-identical when the target is unreachable, the joint is not a crease or a panel is locked', () => {
    const { m, tris } = developable();
    expect(prefoldVertex(m).ok).toBe(true);
    const [loop] = findCreaseLoops(m);
    const cid = loop.creaseIds[1];
    // a bar pinned between two far vertices of different panels forbids any further folding
    const pa = m.points[m.links[loop.linkIds[1]].pointIds[1]].pos;
    const pb = m.points[m.links[loop.linkIds[2]].pointIds[2]].pos;
    const bar = addBar(m, pa, pb);
    addJoint(m, 'spherical', vertex(bar, 0), vertex(m.links[loop.linkIds[1]], 1));
    addJoint(m, 'spherical', vertex(bar, 1), vertex(m.links[loop.linkIds[2]], 2));
    const before = serializeModel(m);
    const cur = creaseDihedralDeg(m, m.joints[cid])!;
    expect(foldCreaseToTarget(m, cid, Math.abs(cur) - 40)).toEqual({ ok: false, reason: 'unreachable' });
    expect(serializeModel(m)).toBe(before);
    expect(foldCreaseToTarget(m, bar.id, 100)).toEqual({ ok: false, reason: 'unreachable' });
    m.links[tris[2].id].locked = true; // (restore() replaced the link objects, so go through the model)
    expect(foldCreaseToTarget(m, cid, 100).reason).toBe('locked');
    m.links[tris[2].id].locked = false;
    expect(serializeModel(m)).toBe(before);
    // a flat vertex whose panels cannot leave the plane (X vertex: hinge branches only) reports the pre-fold's reason
    const x = flatVertex([0, 90, 180, 270], { planar: true });
    const snapshot = serializeModel(x.m);
    expect(foldCreaseToTarget(x.m, x.creases[0].id, 160, 'M').reason).toBe('noBranch');
    expect(serializeModel(x.m)).toBe(snapshot);
  });
});

describe('driveCrease', () => {
  it('pre-folds a flat vertex to the side of the stored assignment, adds exactly one driver on the picked crease and measures it', () => {
    for (const mv of ['M', 'V'] as const) {
      const { m, creases, tris } = developable({ planar: true });
      const j = creases[0]; // the crease between the two 60° sectors (collinear with the third)
      j.fold = { mv };
      const r = driveCrease(m, j.id);
      expect(r.prefold?.ok).toBe(true);
      expect(r.driver).not.toBeNull();
      expect(m.drivers.length).toBe(1);
      expect(m.drivers[0]).toBe(r.driver);
      expect(m.drivers[0].kind).toBe('fold');
      expect(m.drivers[0].jointId).toBe(j.id);
      expect(m.drivers[0].value).toBeCloseTo(creaseDihedralDeg(m, j)!, 9);
      expect(tris.every((t) => bodyPlaneJoint(m, t.id) === null)).toBe(true);
      expect(noneFlat(m, creases.map((c) => c.id))).toBe(true);
      expect(computeMobility(m).dof).toBe(1);
      // the pre-fold drove a non-collinear crease, with the sign that gives the picked (collinear) crease its stored class
      expect(r.reused).toBe(false);
      expect(creaseMV(m, j)).toBe(mv);
    }
  });

  it('honours fold.mv on the crease it drives, needs no pre-fold on a folded vertex and moves the single driver of a one-DOF vertex to the picked crease', () => {
    const { m, creases } = developable();
    const j = creases[1]; // non-collinear: the pre-fold drives this crease itself
    for (const mv of ['M', 'V'] as const) {
      const fresh = parseModel(serializeModel(m));
      fresh.joints[j.id].fold = { mv };
      const r = driveCrease(fresh, j.id);
      expect(r.prefold?.ok).toBe(true);
      expect(r.prefold?.creaseId).toBe(j.id);
      expect(r.reused).toBe(false);
      expect(creaseMV(fresh, fresh.joints[j.id])).toBe(mv);
      expect(fresh.drivers.length).toBe(1);
    }
    // the Miura example has one fold driver on a one-DOF vertex: driving another crease moves it (a second driver
    // would freeze the vertex)
    const miura = origamiMiuraVertex(60);
    const previous = miura.drivers[0].jointId!;
    const other = Object.values(miura.joints).find((x) => x.type === 'revolute' && x.id !== previous)!;
    const r = driveCrease(miura, other.id);
    expect(r.prefold).toBeNull();
    expect(r.reused).toBe(false);
    expect(miura.drivers.length).toBe(1);
    expect(miura.drivers[0]).toBe(r.driver);
    expect(miura.drivers[0].jointId).toBe(other.id);
    expect(miura.drivers[0].value).toBeCloseTo(creaseDihedralDeg(miura, other)!, 9);
    expect(sweepDriver(miura, 0).poses.length).toBeGreaterThan(10);
    // a model with several drivers is left alone: a deliberate extra driver stays
    const extra = origamiMiuraVertex(60);
    extra.drivers.push({ ...extra.drivers[0], id: 'driver_extra', jointId: undefined, kind: 'angle', linkId: Object.keys(extra.links)[0] });
    const r2 = driveCrease(extra, other.id);
    expect(extra.drivers.length).toBe(3);
    expect(extra.drivers[2]).toBe(r2.driver);
    const sph = addJoint(miura, 'spherical', vertex(addBar(miura, [5, 0, 0], [6, 0, 0]), 0), vertex(addBar(miura, [5, 0, 0], [5, 1, 0]), 0))!;
    expect(driveCrease(miura, sph.id)).toEqual({ driver: null, prefold: null, reused: false });
  });

  it('after Fold, driving the crease that already has the driver reuses it, and driving another crease moves it: never two fold drivers on the vertex', () => {
    const { m, tris } = developable({ planar: true });
    const r = prefoldVertex(m);
    expect(r.ok).toBe(true);
    expect(m.drivers.length).toBe(1);
    const kept = m.drivers[0];
    // "Drive this crease" on the crease Fold selected
    const same = driveCrease(m, r.creaseId!);
    expect(same.reused).toBe(true);
    expect(same.prefold).toBeNull();
    expect(same.driver).toBe(kept);
    expect(m.drivers.length).toBe(1);
    const sweep = sweepDriver(m, 0);
    expect(sweep.poses.length).toBeGreaterThan(10);
    expect(sweep.range[1] - sweep.range[0]).toBeGreaterThan(30);
    // the Driver tool on another crease of the vertex
    const [loop] = findCreaseLoops(m);
    const other = loop.creaseIds.find((id) => id !== r.creaseId)!;
    const moved = driveCrease(m, other);
    expect(moved.reused).toBe(false);
    expect(moved.prefold).toBeNull();
    expect(m.drivers.length).toBe(1);
    expect(m.drivers[0]).toBe(moved.driver);
    expect(m.drivers[0].jointId).toBe(other);
    expect(m.drivers[0].value).toBeCloseTo(creaseDihedralDeg(m, m.joints[other])!, 9);
    const sweep2 = sweepDriver(m, 0);
    expect(sweep2.poses.length).toBeGreaterThan(10);
    for (const t of tris.slice(1)) {
      const far = t.pointIds.filter((id) => dist(m.points[id].pos, O) > 1);
      const move = Math.max(...far.map((pid) => Math.max(...sweep2.poses.map((pose) => dist(pose.positions.get(pid)!, m.points[pid].pos)))));
      expect(move).toBeGreaterThan(1);
    }
    // a flat vertex whose crease already carries a driver: the pre-fold runs and the driver is reused, still one
    const flat = developable({ planar: true });
    const cid = findCreaseLoops(flat.m)[0].creaseIds[1];
    flat.m.drivers.push({ id: 'driver_x', kind: 'fold', jointId: cid, value: 180 });
    const again = driveCrease(flat.m, cid);
    expect(again.prefold?.ok).toBe(true);
    expect(again.reused).toBe(true);
    expect(flat.m.drivers.length).toBe(1);
    expect(flat.m.drivers[0].id).toBe('driver_x');
    expect(Math.abs(flat.m.drivers[0].value)).toBeLessThan(179); // re-measured on the folded pose
  });
});

describe('crease presentation: labels and colours', () => {
  it('creaseLabel reads "Crease M 160°" (fold angle rounded, class omitted while flat, angle omitted when unknown)', () => {
    expect(creaseLabel('M', -160.4)).toBe('Crease M 160°');
    expect(creaseLabel('V', 119.6)).toBe('Crease V 120°');
    expect(creaseLabel(null, 180)).toBe('Crease 180°');
    expect(creaseLabel(null, null)).toBe('Crease');
  });

  it('the Properties readout shows the fold angle magnitude with the class, never a signed dihedral', () => {
    expect(PANEL.creaseAngleValue(-169.9, 'V')).toBe('169.9° · Valley (V)');
    expect(PANEL.creaseAngleValue(160, 'M')).toBe('160.0° · Mountain (M)');
    expect(PANEL.creaseAngleValue(-180, null)).toBe('180.0° · flat');
    const { m, creases } = developable();
    expect(prefoldVertex(m).ok).toBe(true);
    for (const j of creases) {
      const deg = creaseDihedralDeg(m, j)!;
      const text = PANEL.creaseAngleValue(deg, creaseMV(m, j));
      expect(text).toMatch(/^\d/);
      expect(text.startsWith(`${Math.abs(deg).toFixed(1)}°`)).toBe(true);
      expect(creaseLabel(creaseMV(m, j), deg)).toContain(`${Math.round(Math.abs(deg))}°`);
    }
  });

  it('mountain / valley colours default to red / blue and are filled in for stored settings that predate them', () => {
    expect(DEFAULT_SETTINGS.colors.mountain).toBe('#d9342b');
    expect(DEFAULT_SETTINGS.colors.valley).toBe('#2f6fd6');
    const g = globalThis as unknown as { localStorage?: { getItem(k: string): string | null; setItem(k: string, v: string): void } };
    const saved = g.localStorage;
    g.localStorage = { getItem: () => JSON.stringify({ colors: { geometry: '#000000' } }), setItem: () => undefined };
    try {
      const s = loadSettings();
      expect(s.colors.geometry).toBe('#000000');
      expect(s.colors.mountain).toBe('#d9342b');
      expect(s.colors.valley).toBe('#2f6fd6');
    } finally {
      if (saved) g.localStorage = saved;
      else delete g.localStorage;
    }
  });
});
