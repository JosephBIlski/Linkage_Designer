import { describe, expect, it } from 'vitest';
import miuraSheetFile from '../examples/miuraSheet.linkage.json';
import { addAngleDriver, addJoint, candidateAngleDrivers, createModel, groundLink, parseModel, removeJoint, serializeModel, setGround } from '../src/core/model';
import { addPattern, addPatternCrease, addPatternVertex, findPatternVertex, generatePanels, miuraPattern, patternBasis, patternFaces, patternOfJoint, setCreaseClass, to3d } from '../src/core/creasePattern';
import { applyPositions, computeMobility, currentViolation, positionsFromModel, solveForward, sweepDriver, syncDriverValues, type SweepResult } from '../src/core/kinematics';
import { DEFAULT_PREFOLD_DEG, creaseDihedralDeg, creaseMV, driveCrease, findCreaseLoops, foldCreaseToTarget, isCrease, isFlatDihedral, patternLoopCreases, prefoldFlat, prefoldPattern, prefoldVertex } from '../src/core/fold';
import { EXAMPLE_BUILDERS, miuraSheet } from '../src/core/examples';
import { EXAMPLES } from '../src/ui/strings';
import { dist, dot, scale, sub } from '../src/core/geometry';
import type { CreasePattern, ID, Model, Vec3 } from '../src/core/types';

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/** A Miura sheet with rows × cols interior vertices generated from its pattern; the first panel is ground. */
function sheet(rows = 2, cols = 2): { m: Model; p: CreasePattern; creaseIds: ID[]; linkIds: ID[] } {
  const m = createModel();
  const p = miuraPattern(m, 'plane_top', rows, cols);
  const r = generatePanels(m, p);
  return { m, p, creaseIds: r.jointIds, linkIds: r.linkIds };
}

/** The creases among `ids` that are still within the flat tolerance. */
const flatOnes = (m: Model, ids: ID[]): ID[] =>
  ids.filter((id) => {
    const d = creaseDihedralDeg(m, m.joints[id]);
    return d === null || isFlatDihedral(d);
  });

/** The creases among `ids` whose class in the pose is not the one stored on them (the drawn class). */
const offDrawn = (m: Model, ids: ID[]): ID[] => ids.filter((id) => creaseMV(m, m.joints[id]) !== m.joints[id].fold?.mv);

/** The generated joint of pattern crease `cid` (matched by the end points of its a feature). */
function jointOfCrease(m: Model, p: CreasePattern, jointIds: ID[], cid: ID): ID {
  const [va, vb] = [p.vertices[p.creases[cid].a].pos, p.vertices[p.creases[cid].b].pos];
  const j = jointIds.map((id) => m.joints[id]).find((jt) => {
    const [x, y] = jt.a.pointIds.map((pid) => m.points[pid].pos);
    return (dist(x, va) < 1e-9 && dist(y, vb) < 1e-9) || (dist(x, vb) < 1e-9 && dist(y, va) < 1e-9);
  });
  expect(j).toBeDefined();
  return j!.id;
}

/** For every non-ground link, how far its vertex farthest from the ground's first point travels over the sweep. */
function travels(m: Model, sweep: SweepResult): number[] {
  const g = groundLink(m)!;
  const origin = m.points[g.pointIds[0]].pos;
  return Object.values(m.links)
    .filter((l) => !l.ground)
    .map((l) => {
      const far = l.pointIds.reduce((b, id) => (dist(m.points[id].pos, origin) > dist(m.points[b].pos, origin) ? id : b), l.pointIds[0]);
      return Math.max(...sweep.poses.map((pose) => dist(pose.positions.get(far)!, m.points[far].pos)));
    });
}

/** Distance from point `id` to the nearest of the points `ids`. */
const farFrom = (m: Model, ids: ID[], id: ID): number => Math.min(...ids.map((g) => dist(m.points[g].pos, m.points[id].pos)));

const centroid = (pts: Vec3[]): Vec3 => pts.reduce((s, q) => [s[0] + q[0] / pts.length, s[1] + q[1] / pts.length, s[2] + q[2] / pts.length] as Vec3, [0, 0, 0]);

/** The flat 3 × 3 sheet grounded at the panel nearest its middle (the example's configuration). */
function sheetMiddleGround(): { m: Model; p: CreasePattern; creaseIds: ID[]; linkIds: ID[] } {
  const m = createModel();
  const p = miuraPattern(m, 'plane_top', 3, 3);
  const r = generatePanels(m, p, { ground: false });
  const middle = centroid(Object.values(p.vertices).map((v) => v.pos));
  const centre = (id: ID): Vec3 => centroid(m.links[id].pointIds.map((pid) => m.points[pid].pos));
  setGround(m, r.linkIds.reduce((best, id) => (dist(centre(id), middle) < dist(centre(best), middle) ? id : best), r.linkIds[0]));
  return { m, p, creaseIds: r.jointIds, linkIds: r.linkIds };
}

// ---------------------------------------------------------------------------
// prefoldPattern
// ---------------------------------------------------------------------------

describe('prefoldPattern on the 2 × 2 Miura sheet', () => {
  it('folds every generated crease on the drawn branch: DOF 1, a zigzag crease driven at 160°, a fold driver kept, every panel sweeps', () => {
    const { m, p, creaseIds, linkIds } = sheet();
    expect(linkIds.length).toBe(9);
    expect(creaseIds.length).toBe(12);
    expect(computeMobility(m).dof).toBeGreaterThan(1); // four flat vertices: a singular state
    const flatLoops = findCreaseLoops(m); // the collinear pairs are measured here, while the horizontal lines are straight
    expect(flatLoops.every((l) => l.flat)).toBe(true);
    const r = prefoldPattern(m, p.id);
    expect(r.ok).toBe(true);
    expect(r.reason).toBeUndefined();
    expect(r.conflicts).toBeUndefined();
    expect(r.removedPlanes).toEqual([]);
    expect(r.dof).toBe(1);
    expect(computeMobility(m).dof).toBe(1);
    expect(currentViolation(m)).toBeLessThan(1e-9);
    // every generated crease has left the flat state with the class stored on it from the pattern
    expect(Object.keys(r.dihedrals!).sort()).toEqual([...creaseIds].sort());
    expect(flatOnes(m, creaseIds)).toEqual([]);
    expect(offDrawn(m, creaseIds)).toEqual([]);
    expect(creaseIds.filter((id) => creaseMV(m, m.joints[id]) === 'M').length).toBe(6);
    // the driven crease is a zigzag (not collinear with another crease of its vertex) at exactly ±160°
    const loop = flatLoops.find((l) => l.creaseIds.includes(r.creaseId!))!;
    expect(loop.collinearPairs.flat()).not.toContain(r.creaseId);
    expect(Math.abs(r.dihedrals![r.creaseId!])).toBeCloseTo(DEFAULT_PREFOLD_DEG, 6);
    // the horizontal creases (collinear while flat) fold equally and less than the zigzags, as at the single Miura
    // vertex; no loop is flat any more
    expect(findCreaseLoops(m).every((l) => !l.flat)).toBe(true);
    for (const l of flatLoops) {
      expect(l.collinearPairs.length).toBe(1);
      for (const [a, b] of l.collinearPairs) {
        expect(Math.abs(creaseDihedralDeg(m, m.joints[a])!)).toBeCloseTo(Math.abs(creaseDihedralDeg(m, m.joints[b])!), 6);
        expect(Math.abs(creaseDihedralDeg(m, m.joints[a])!)).toBeGreaterThan(165);
      }
    }
    // the model had no driver: the fold driver on the driven crease is kept, and a sweep moves every panel
    expect(m.drivers.length).toBe(1);
    expect(m.drivers[0].kind).toBe('fold');
    expect(m.drivers[0].jointId).toBe(r.creaseId);
    expect(m.drivers[0].value).toBeCloseTo(r.dihedrals![r.creaseId!], 9);
    const sweep = sweepDriver(m, 0);
    expect(sweep.poses.length).toBeGreaterThan(10);
    const moved = travels(m, sweep);
    expect(moved.length).toBe(8);
    for (const d of moved) expect(d).toBeGreaterThan(0.5);
  });

  it('takes the other branch for the mirrored drawing: the stored classes choose the sign', () => {
    const { m, p, creaseIds } = sheet();
    const first = prefoldPattern(m, p.id);
    expect(first.ok).toBe(true);
    const corner = Object.values(m.points).filter((pt) => pt.role === 'vertex').reduce((b, pt) => (pt.pos[0] + pt.pos[1] > b.pos[0] + b.pos[1] ? pt : b));
    const lift = corner.pos[2];
    expect(Math.abs(lift)).toBeGreaterThan(0.1);
    // flip every class of the pattern and regenerate: the sheet must fold to the mirror image
    const mirrored = createModel();
    const q = miuraPattern(mirrored, 'plane_top', 2, 2);
    for (const c of Object.values(q.creases)) if (c.cls === 'M' || c.cls === 'V') setCreaseClass(mirrored, q, c.id, c.cls === 'M' ? 'V' : 'M');
    const gen = generatePanels(mirrored, q);
    expect(gen.warnings).toEqual([]);
    const r = prefoldPattern(mirrored, q.id);
    expect(r.ok).toBe(true);
    expect(flatOnes(mirrored, gen.jointIds)).toEqual([]);
    expect(offDrawn(mirrored, gen.jointIds)).toEqual([]);
    expect(gen.jointIds.filter((id) => creaseMV(mirrored, mirrored.joints[id]) === 'V').length).toBe(6);
    const corner2 = Object.values(mirrored.points).filter((pt) => pt.role === 'vertex').reduce((b, pt) => (pt.pos[0] + pt.pos[1] > b.pos[0] + b.pos[1] ? pt : b));
    expect(corner2.pos[2]).toBeCloseTo(-lift, 6);
    expect(creaseIds.length).toBe(gen.jointIds.length);
  });

  it('does not require a generated crease that closes no loop to fold: a panel hung on the border', () => {
    const m = createModel();
    const p = miuraPattern(m, 'plane_top', 1, 1, 60, 2, 2);
    // a triangle below the bottom-left cell: its base, a border crease of the unit, becomes an interior hinge
    const a = addPatternVertex(m, p, [0, 0, 0]);
    const b = addPatternVertex(m, p, [2, 0, 0]);
    expect(a.existing && b.existing).toBe(true);
    const tip = addPatternVertex(m, p, [1, -1, 0]).vertex.id;
    expect(addPatternCrease(m, p, a.vertex.id, tip, 'B').ok).toBe(true);
    expect(addPatternCrease(m, p, tip, b.vertex.id, 'B').ok).toBe(true);
    const faces = patternFaces(m, p);
    expect(faces.faces.length).toBe(5);
    expect(faces.interior.length).toBe(5);
    const r = generatePanels(m, p);
    expect(r.linkIds.length).toBe(5);
    expect(r.jointIds.length).toBe(5);
    const hinge = r.jointIds.find((id) => m.joints[id].fold?.mv === undefined)!;
    expect(hinge).toBeDefined();
    expect(isCrease(m, m.joints[hinge])).toBe(true);
    expect(findCreaseLoops(m).length).toBe(1);
    const f = prefoldPattern(m, p.id);
    expect(f.ok).toBe(true);
    expect(f.dof).toBe(2); // the vertex's DOF plus the free hinge
    expect(Object.keys(f.dihedrals!).length).toBe(4);
    expect(f.dihedrals![hinge]).toBeUndefined();
    expect(isFlatDihedral(creaseDihedralDeg(m, m.joints[hinge])!)).toBe(true);
    expect(flatOnes(m, r.jointIds.filter((id) => id !== hinge))).toEqual([]);
    expect(offDrawn(m, r.jointIds.filter((id) => id !== hinge))).toEqual([]);
    // the same set asked of prefoldVertex directly is refused: the hinge never leaves the flat state
    const again = createModel();
    const q = miuraPattern(again, 'plane_top', 1, 1, 60, 2, 2);
    const a2 = addPatternVertex(again, q, [0, 0, 0]).vertex.id;
    const b2 = addPatternVertex(again, q, [2, 0, 0]).vertex.id;
    const tip2 = addPatternVertex(again, q, [1, -1, 0]).vertex.id;
    addPatternCrease(again, q, a2, tip2, 'B');
    addPatternCrease(again, q, tip2, b2, 'B');
    const r2 = generatePanels(again, q);
    const before = serializeModel(again);
    expect(prefoldVertex(again, { creaseIds: r2.jointIds })).toEqual({ ok: false, reason: 'noBranch' });
    expect(serializeModel(again)).toBe(before);
  });

  it('reports noLoop, notFlat and locked, and patternOfJoint finds the pattern of a generated crease', () => {
    const { m, p, creaseIds, linkIds } = sheet();
    expect(prefoldPattern(m, 'pattern_none')).toEqual({ ok: false, reason: 'noLoop' });
    const empty = addPattern(m, 'plane_top');
    expect(prefoldPattern(m, empty.id)).toEqual({ ok: false, reason: 'noLoop' });
    // a square with a diagonal (beside the sheet) generates one hinge and no loop
    const sq = addPattern(m, 'plane_top');
    const ids = [[10, 0, 0], [12, 0, 0], [12, 2, 0], [10, 2, 0]].map((pos) => addPatternVertex(m, sq, pos as Vec3).vertex.id);
    for (let i = 0; i < 4; i++) expect(addPatternCrease(m, sq, ids[i], ids[(i + 1) % 4], 'B').ok).toBe(true);
    expect(addPatternCrease(m, sq, ids[0], ids[2], 'V').ok).toBe(true);
    const gen = generatePanels(m, sq);
    expect(gen.jointIds.length).toBe(1);
    expect(prefoldPattern(m, sq.id)).toEqual({ ok: false, reason: 'noLoop' });
    expect(patternOfJoint(m, gen.jointIds[0])).toBe(sq);
    expect(patternOfJoint(m, creaseIds[0])).toBe(p);
    expect(patternOfJoint(m, 'joint_none')).toBeNull();
    // a locked panel of the sheet
    m.links[linkIds[4]].locked = true;
    const before = serializeModel(m);
    expect(prefoldPattern(m, p.id)).toEqual({ ok: false, reason: 'locked' });
    expect(serializeModel(m)).toBe(before);
    m.links[linkIds[4]].locked = false;
    expect(prefoldPattern(m, p.id).ok).toBe(true);
    expect(prefoldPattern(m, p.id)).toEqual({ ok: false, reason: 'notFlat' });
  });
});

describe('mountain / valley conflicts', () => {
  it('a Miura unit with one crease flipped (2 : 2) is refused with mvConflict naming that crease, the model byte-identical', () => {
    for (const k of [0, 1, 2, 3]) {
      const m = createModel();
      const p = miuraPattern(m, 'plane_top', 1, 1);
      const cid = patternFaces(m, p).interior[k];
      const drawn = p.creases[cid].cls as 'M' | 'V';
      setCreaseClass(m, p, cid, drawn === 'M' ? 'V' : 'M');
      const r = generatePanels(m, p);
      expect(r.warnings.map((w) => w.kind)).toEqual(['maekawa']);
      const flipped = jointOfCrease(m, p, r.jointIds, cid);
      const before = serializeModel(m);
      const f = prefoldPattern(m, p.id);
      expect(f.ok).toBe(false);
      expect(f.reason).toBe('mvConflict');
      expect(f.conflicts).toEqual([flipped]);
      expect(f.creaseId).toBeUndefined();
      expect(serializeModel(m)).toBe(before);
      // without the stored classes the vertex folds (on whichever branch), and the flipped crease then has its old class
      const free = prefoldPattern(m, p.id, { honourAssigned: false });
      expect(free.ok).toBe(true);
      expect(flatOnes(m, r.jointIds)).toEqual([]);
      expect(creaseMV(m, m.joints[flipped])).toBe(drawn);
      expect(offDrawn(m, r.jointIds)).toEqual([flipped]);
      // an explicit preference wins over the stored class of the crease it names (plane_top: seen from +Z, the creaseMV side)
      const again = parseModel(before);
      const pref = prefoldPattern(again, p.id, { mountainValley: { [flipped]: drawn } });
      expect(pref.ok).toBe(true);
      expect(creaseMV(again, again.joints[flipped])).toBe(drawn);
    }
  });

  it('Fold to target with a class changed on one generated crease names that crease, flat or already folded', () => {
    // what the crease Properties do: the stored class is changed on the joint, then "Fold to target" with that class
    const flipOne = (m: Model, p: CreasePattern, jointIds: ID[], k: number): { id: ID; mv: 'M' | 'V' } => {
      const id = jointOfCrease(m, p, jointIds, patternFaces(m, p).interior[k]);
      const j = m.joints[id];
      const mv = j.fold!.mv === 'M' ? 'V' : 'M';
      j.fold = { ...j.fold, mv };
      return { id, mv };
    };
    for (const k of [0, 1, 2, 3]) {
      const m = createModel();
      const p = miuraPattern(m, 'plane_top', 1, 1);
      const r = generatePanels(m, p);
      const { id, mv } = flipOne(m, p, r.jointIds, k);
      const before = serializeModel(m);
      for (const target of [120, 160]) {
        const f = foldCreaseToTarget(m, id, target, mv);
        expect(f.ok).toBe(false);
        expect(f.reason).toBe('mvConflict');
        expect(f.conflicts).toEqual([id]);
        expect(serializeModel(m)).toBe(before);
      }
    }
    // the unit folded first (prefoldPattern), then one class flipped: the mirrored branch is asked for through flat
    const m = createModel();
    const p = miuraPattern(m, 'plane_top', 1, 1);
    const r = generatePanels(m, p);
    const id = jointOfCrease(m, p, r.jointIds, patternFaces(m, p).interior[0]);
    expect(prefoldPattern(m, p.id).ok).toBe(true);
    const j = m.joints[id];
    const mv = j.fold!.mv === 'M' ? 'V' : 'M';
    j.fold = { ...j.fold, mv };
    const before = serializeModel(m);
    const f = foldCreaseToTarget(m, id, 120, mv);
    expect(f.reason).toBe('mvConflict');
    expect(f.conflicts).toEqual([id]);
    expect(serializeModel(m)).toBe(before);
  });

  it('a rigid degree-3 vertex in the sheet is a geometric failure (noBranch), never a conflict blamed on honoured classes', () => {
    const m = createModel();
    const p = miuraPattern(m, 'plane_top', 2, 2);
    const basis = patternBasis(m, p);
    const d = 2 / Math.tan(Math.PI / 3);
    const at = (u: number, v: number): ID => findPatternVertex(p, to3d(basis, [u, v]), 1e-6)!.id;
    // a degree-3 vertex inside cell (0, 0), joined to three of its corners: three creases around a vertex cannot fold
    const c = addPatternVertex(m, p, to3d(basis, [0.9, 0.9])).vertex.id;
    for (const [corner, cls] of [[at(0, 0), 'M'], [at(2, 0), 'V'], [at(d, 2), 'M']] as [ID, 'M' | 'V'][]) expect(addPatternCrease(m, p, c, corner, cls).ok).toBe(true);
    const r = generatePanels(m, p);
    expect(r.linkIds.length).toBe(11);
    expect(r.jointIds.length).toBe(15);
    expect(findCreaseLoops(m).map((l) => l.creaseIds.length).sort()).toEqual([3, 4, 4, 4, 4]);
    const snapshot = serializeModel(m);
    // every Miura loop, each of its zigzag (non-collinear) creases left unassigned: the first stage folds that loop
    // on either branch, the degree-3 stage fails geometrically
    let runs = 0;
    for (const loop of findCreaseLoops(m).filter((l) => l.creaseIds.length === 4)) {
      const collinear = new Set(loop.collinearPairs.flat());
      for (const pref of loop.creaseIds.filter((id) => !collinear.has(id))) {
        const mm = parseModel(snapshot);
        delete mm.joints[pref].fold!.mv;
        const res = prefoldPattern(mm, p.id, { preferCreaseId: pref, honourAssigned: true });
        expect(res).toEqual({ ok: false, reason: 'noBranch' });
        expect(prefoldPattern(parseModel(snapshot), p.id, { preferCreaseId: pref, honourAssigned: false })).toEqual({ ok: false, reason: 'noBranch' });
        runs++;
      }
    }
    expect(runs).toBe(8);
  });

  it('on the 2 × 2 sheet a flipped crease away from the first loop is reported too', () => {
    const { m, p } = sheet();
    const loops = findCreaseLoops(m);
    const firstLoop = loops.find((l) => l.flat)!;
    // a crease of the last loop that the first loop does not share
    const last = loops[loops.length - 1];
    const jointId = last.creaseIds.find((id) => !firstLoop.creaseIds.includes(id))!;
    const cid = Object.values(p.creases).find((c) => jointOfCreaseSafe(m, p, [jointId], c.id) === jointId)!.id;
    setCreaseClass(m, p, cid, p.creases[cid].cls === 'M' ? 'V' : 'M');
    const r = generatePanels(m, p);
    const flipped = jointOfCrease(m, p, r.jointIds, cid);
    const before = serializeModel(m);
    const f = prefoldPattern(m, p.id);
    expect(f.ok).toBe(false);
    expect(f.reason).toBe('mvConflict');
    expect(f.conflicts).toEqual([flipped]);
    expect(serializeModel(m)).toBe(before);
  });
});

/** jointOfCrease without the assertion: null when `jointIds` holds no joint on crease `cid`. */
function jointOfCreaseSafe(m: Model, p: CreasePattern, jointIds: ID[], cid: ID): ID | null {
  const [va, vb] = [p.vertices[p.creases[cid].a].pos, p.vertices[p.creases[cid].b].pos];
  const j = jointIds.map((id) => m.joints[id]).find((jt) => {
    const [x, y] = jt.a.pointIds.map((pid) => m.points[pid].pos);
    return (dist(x, va) < 1e-9 && dist(y, vb) < 1e-9) || (dist(x, vb) < 1e-9 && dist(y, va) < 1e-9);
  });
  return j?.id ?? null;
}

// ---------------------------------------------------------------------------
// Sheets that need the stages, and the example
// ---------------------------------------------------------------------------

describe('the 3 × 3 sheet grounded in the middle', () => {
  it('the pre-fold of one loop alone lands on a mixed branch; the pattern-wide pre-fold folds every crease on the drawn one', () => {
    const one = sheetMiddleGround();
    expect(one.creaseIds.length).toBe(24);
    const r1 = prefoldVertex(one.m); // the first flat loop, its creases only
    expect(r1.ok).toBe(true);
    // the documented reason for the stages: straight lines of creases folded as hinges leave other creases flat
    expect(flatOnes(one.m, one.creaseIds).length + offDrawn(one.m, one.creaseIds).length).toBeGreaterThan(0);
    const all = sheetMiddleGround();
    const r2 = prefoldPattern(all.m, all.p.id);
    expect(r2.ok).toBe(true);
    expect(r2.dof).toBe(1);
    expect(flatOnes(all.m, all.creaseIds)).toEqual([]);
    expect(offDrawn(all.m, all.creaseIds)).toEqual([]);
    expect(Math.abs(r2.dihedrals![r2.creaseId!])).toBeCloseTo(DEFAULT_PREFOLD_DEG, 6);
    expect(currentViolation(all.m)).toBeLessThan(1e-9);
  });
});

describe('the Miura-ori sheet example', () => {
  it('miuraSheet: 16 panels, 24 creases folded with their drawn classes, DOF 1, a fold driver at 160° on a zigzag, the middle grounded, a corner shown', () => {
    expect(EXAMPLE_BUILDERS.miuraSheet).toBeDefined();
    expect(EXAMPLES.miuraSheet.label).toContain('3×3');
    const m = EXAMPLE_BUILDERS.miuraSheet();
    expect(Object.keys(m.links).length).toBe(16);
    const creases = Object.values(m.joints).filter((j) => isCrease(m, j)).map((j) => j.id);
    expect(creases.length).toBe(24);
    expect(computeMobility(m).dof).toBe(1);
    expect(currentViolation(m)).toBeLessThan(1e-9);
    expect(flatOnes(m, creases)).toEqual([]);
    expect(offDrawn(m, creases)).toEqual([]);
    // 10 mountains and 14 valleys: the 3 × 3 pattern's zigzag columns run V, M, V (4 segments each) and its horizontals split 6 : 6
    expect(creases.filter((id) => creaseMV(m, m.joints[id]) === 'M').length).toBe(10);
    expect(creases.filter((id) => creaseMV(m, m.joints[id]) === 'V').length).toBe(14);
    // the driver: one fold driver at ±160° on a crease that is not collinear with a neighbour
    expect(m.drivers.length).toBe(1);
    expect(m.drivers[0].kind).toBe('fold');
    expect(Math.abs(m.drivers[0].value)).toBeCloseTo(DEFAULT_PREFOLD_DEG, 6);
    const driven = m.drivers[0].jointId!;
    const loop = findCreaseLoops(m).find((l) => l.creaseIds.includes(driven))!;
    expect(loop.collinearPairs.flat()).not.toContain(driven);
    // the ground panel is the one nearest the middle of the sheet; the displayed point is a corner, far from it
    const p = Object.values(m.patterns!)[0];
    expect(p.hidden).toBe(true);
    expect(p.generated!.linkIds.length).toBe(16);
    expect(p.generated!.jointIds.length).toBe(24);
    const middle = centroid(Object.values(p.vertices).map((v) => v.pos));
    const centre = (l: { pointIds: ID[] }): Vec3 => centroid(l.pointIds.map((pid) => m.points[pid].pos));
    const ground = groundLink(m)!;
    for (const l of Object.values(m.links)) expect(dist(centre(l), middle)).toBeGreaterThanOrEqual(dist(centre(ground), middle) - 1e-9);
    expect(m.settings.displayPointIds.length).toBe(1);
    const shown = m.points[m.settings.displayPointIds[0]];
    expect(shown.role).toBe('vertex');
    expect(dist(shown.pos, middle)).toBeGreaterThan(5);
    // what loadExample does (a forward solve at the driver values) keeps the pose
    const pos0 = positionsFromModel(m);
    const res = solveForward(m, m.drivers.map((d) => d.value));
    expect(res.converged).toBe(true);
    expect(Math.max(...[...res.positions].map(([id, q]) => dist(q, pos0.get(id)!)))).toBeLessThan(1e-7);
    applyPositions(m, res.positions);
    // a sweep moves every panel
    const sweep = sweepDriver(m, 0);
    expect(sweep.poses.length).toBeGreaterThan(10);
    const moved = travels(m, sweep);
    expect(moved.length).toBe(15);
    for (const d of moved) expect(d).toBeGreaterThan(0.5);
    // files: the example round-trips, and examples/miuraSheet.linkage.json is the builder's output
    const back = parseModel(serializeModel(m));
    expect(serializeModel(back)).toBe(serializeModel(m));
    const file = parseModel(JSON.stringify(miuraSheetFile));
    expect(Object.keys(file.links).length).toBe(16);
    expect(Object.values(file.joints).filter((j) => isCrease(file, j)).length).toBe(24);
    expect(file.drivers.length).toBe(1);
    expect(Object.values(file.patterns!)[0].hidden).toBe(true);
    expect(computeMobility(file).dof).toBe(1);
    expect(flatOnes(file, Object.values(file.joints).filter((j) => isCrease(file, j)).map((j) => j.id))).toEqual([]);
    expect(miuraSheet(2, 2).drivers.length).toBe(1);
  });
});

// ---------------------------------------------------------------------------
// The single-crease commands on a generated sheet (Driver tool, Drive this crease, Fold to target)
// ---------------------------------------------------------------------------

describe('driveCrease and foldCreaseToTarget on a generated sheet', () => {
  const sheets: [string, () => { m: Model; p: CreasePattern; creaseIds: ID[] }][] = [
    ['2 × 2, first panel ground', () => sheet()],
    ['3 × 3, first panel ground', () => sheet(3, 3)],
    ['3 × 3, middle panel ground', sheetMiddleGround],
  ];
  for (const [name, build] of sheets) {
    it(`${name}: every crease picked pre-folds the whole sheet on its drawn branch, DOF 1, the driver on the picked crease`, () => {
      const { m: base, p, creaseIds } = build();
      expect(patternLoopCreases(base, p).sort()).toEqual([...creaseIds].sort());
      const snapshot = serializeModel(base);
      for (const id of creaseIds) {
        const m = parseModel(snapshot);
        const r = driveCrease(m, id);
        expect(r.prefold?.ok).toBe(true);
        expect(r.patternId).toBe(p.id);
        expect(Object.keys(r.prefold!.dihedrals!).length).toBe(creaseIds.length);
        expect(flatOnes(m, creaseIds)).toEqual([]);
        expect(offDrawn(m, creaseIds)).toEqual([]);
        expect(findCreaseLoops(m).some((l) => l.flat)).toBe(false);
        expect(computeMobility(m).dof).toBe(1);
        expect(currentViolation(m)).toBeLessThan(1e-6);
        expect(m.drivers.map((d) => [d.kind, d.jointId])).toEqual([['fold', id]]);
        expect(r.driver?.value).toBeCloseTo(creaseDihedralDeg(m, m.joints[id])!, 9);
      }
    });

    it(`${name}: Fold to target 150° with the stored class on every crease leaves no crease of the sheet flat or off its class`, () => {
      const { m: base, creaseIds } = build();
      const snapshot = serializeModel(base);
      for (const id of creaseIds) {
        const m = parseModel(snapshot);
        const mv = m.joints[id].fold!.mv!;
        const f = foldCreaseToTarget(m, id, 150, mv);
        expect(f.ok).toBe(true);
        expect(Math.abs(creaseDihedralDeg(m, m.joints[id])!)).toBeCloseTo(150, 5);
        expect(creaseMV(m, m.joints[id])).toBe(mv);
        expect(Object.keys(f.dihedrals!).sort()).toEqual([...creaseIds].sort());
        expect(flatOnes(m, creaseIds)).toEqual([]);
        expect(offDrawn(m, creaseIds)).toEqual([]);
        expect(computeMobility(m).dof).toBe(1);
        expect(currentViolation(m)).toBeLessThan(1e-6);
      }
    });
  }

  it('the sweep after driveCrease on a 2 × 2 sheet moves every panel', () => {
    const { m, creaseIds } = sheet();
    driveCrease(m, creaseIds[5]);
    const sweep = sweepDriver(m, 0);
    expect(sweep.poses.length).toBeGreaterThan(10);
    for (const d of travels(m, sweep)) expect(d).toBeGreaterThan(0.5);
  });
});

describe('prefoldFlat (the Fold button)', () => {
  it('folds a generated sheet as a whole, a hand-built vertex on its own, and a loop mixing both on its own', () => {
    // a generated unit: the sheet is folded (patternId set), every crease on its drawn class
    const { m, p, creaseIds } = sheet();
    const r = prefoldFlat(m, { honourAssigned: true });
    expect(r.ok).toBe(true);
    expect(r.patternId).toBe(p.id);
    expect(flatOnes(m, creaseIds)).toEqual([]);
    expect(offDrawn(m, creaseIds)).toEqual([]);
    expect(prefoldFlat(m)).toEqual({ ok: false, reason: 'notFlat' });
    expect(prefoldFlat(createModel())).toEqual({ ok: false, reason: 'noLoop' });
    // a generated unit whose fourth crease was deleted and re-made by hand with the same features and class: the loop is
    // flat and mixes a hand-made crease with generated ones, so the sheet pre-fold does not apply and the loop is folded
    // on its own instead of being refused as "nothing to fold"
    const mixed = createModel();
    const q = miuraPattern(mixed, 'plane_top', 1, 1);
    const gen = generatePanels(mixed, q);
    const victim = mixed.joints[gen.jointIds[3]];
    const [a, b, fold] = [victim.a, victim.b as typeof victim.a, victim.fold];
    removeJoint(mixed, victim.id);
    const hand = addJoint(mixed, 'revolute', a, b)!;
    hand.fold = fold;
    const loops = findCreaseLoops(mixed);
    expect(loops.length).toBe(1);
    expect(loops[0].flat).toBe(true);
    expect(loops[0].creaseIds).toContain(hand.id);
    expect(prefoldPattern(mixed, q.id, { honourAssigned: true })).toEqual({ ok: false, reason: 'noLoop' });
    const f = prefoldFlat(mixed, { honourAssigned: true });
    expect(f.ok).toBe(true);
    expect(f.patternId).toBeUndefined();
    expect(f.dof).toBe(1);
    expect(flatOnes(mixed, loops[0].creaseIds)).toEqual([]);
    expect(offDrawn(mixed, loops[0].creaseIds)).toEqual([]);
  });
});

describe('the default driver after Fold to target on a loop-free generated hinge', () => {
  it('the candidate angle driver takes a tip off the hinge axis, so its sweep moves the free panel', () => {
    const m = createModel();
    const p = addPattern(m, 'plane_top');
    const ids = [[0, 0, 0], [3, 0, 0], [3, 3, 0], [0, 3, 0]].map((pos) => addPatternVertex(m, p, pos as Vec3).vertex.id);
    for (let i = 0; i < 4; i++) expect(addPatternCrease(m, p, ids[i], ids[(i + 1) % 4], 'B').ok).toBe(true);
    expect(addPatternCrease(m, p, ids[0], ids[2], 'V').ok).toBe(true);
    const gen = generatePanels(m, p);
    expect(gen.jointIds.length).toBe(1);
    const jointId = gen.jointIds[0];
    expect(foldCreaseToTarget(m, jointId, 120, 'V').ok).toBe(true);
    expect(Math.abs(creaseDihedralDeg(m, m.joints[jointId])!)).toBeCloseTo(120, 5);
    expect(m.drivers.length).toBe(0); // no loop: nothing kept
    const cands = candidateAngleDrivers(m);
    expect(cands.length).toBeGreaterThan(0);
    for (const c of cands) {
      // the tip is the free corner, off the hinge line through the pivot
      const pivot = m.points[c.pivotId].pos;
      const tip = m.points[c.tipId].pos;
      const along = sub(tip, pivot);
      expect(dist(along, scale(c.axis, dot(along, c.axis)))).toBeGreaterThan(1);
    }
    const c = cands[0];
    addAngleDriver(m, c.linkId, c.pivotId, c.tipId, c.axis);
    syncDriverValues(m);
    const sweep = sweepDriver(m, 0);
    expect(sweep.poses.length).toBeGreaterThan(10);
    expect(Math.abs(sweep.range[1] - sweep.range[0])).toBeGreaterThan(90);
    // the free panel's corner off the hinge (the vertex farthest from every ground point) travels
    const ground = groundLink(m)!;
    const free = Object.values(m.links).find((l) => !l.ground)!;
    const offHinge = free.pointIds.reduce((b, id) => (farFrom(m, ground.pointIds, id) > farFrom(m, ground.pointIds, b) ? id : b), free.pointIds[0]);
    expect(Math.max(...sweep.poses.map((pose) => dist(pose.positions.get(offHinge)!, m.points[offHinge].pos)))).toBeGreaterThan(0.5);
  });
});
