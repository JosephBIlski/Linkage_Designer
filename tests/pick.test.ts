import { describe, expect, it } from 'vitest';
import { PICK_PRIORITY, pickKey, rankPickCandidates, samePick, type PickResult, type PickType } from '../src/viewport/pickRank';
import { PickCycle, QUERY_CLICK_PX } from '../src/viewport/pickCycle';

const hit = (type: PickType, id: string, extra: Partial<PickResult> = {}): PickResult => ({ type, id, point: [0, 0, 0], distance: 1, ...extra });
const names = (rs: PickResult[]): string[] => rs.map((r) => `${r.type}:${r.id}${r.pointIds ? '[' + r.pointIds.join(',') + ']' : ''}${r.faceIndex !== undefined ? '#' + r.faceIndex : ''}`);

describe('rankPickCandidates', () => {
  it('returns nothing for no hits', () => {
    expect(rankPickCandidates([], 0.1)).toEqual([]);
  });

  it('orders model features by pick priority at equal depth, whatever the input order', () => {
    const rs = [hit('face', 'P1'), hit('axis', 'C1'), hit('edge', 'P1', { pointIds: ['a', 'b'] }), hit('joint', 'J1'), hit('vertex', 'P1', { pointId: 'a', pointIds: ['a'] }), hit('editPoint', 'a', { pointId: 'a', pose: 0 })];
    const ranked = rankPickCandidates(rs, 0.1);
    expect(ranked.map((r) => r.type)).toEqual(['editPoint', 'vertex', 'joint', 'edge', 'axis', 'face']);
    expect(ranked.map((r) => PICK_PRIORITY[r.type])).toEqual([0, 1, 2, 3, 4, 5]);
  });

  it('orders the closest first within a priority', () => {
    const rs = [hit('vertex', 'L1', { pointId: 'far', pointIds: ['far'], distance: 1.05 }), hit('vertex', 'L2', { pointId: 'near', pointIds: ['near'], distance: 1.0 }), hit('vertex', 'L3', { pointId: 'mid', pointIds: ['mid'], distance: 1.02 })];
    expect(rankPickCandidates(rs, 0.1).map((r) => r.pointId)).toEqual(['near', 'mid', 'far']);
  });

  it('puts exact-ray hits before ring hits at equal priority, even when the ring hit is nearer', () => {
    const rs = [hit('edge', 'P1', { pointIds: ['a', 'b'], distance: 1.0, exact: false }), hit('edge', 'P2', { pointIds: ['c', 'd'], distance: 1.01, exact: true })];
    expect(rankPickCandidates(rs, 0.1).map((r) => r.id)).toEqual(['P2', 'P1']);
  });

  it('de-duplicates by feature, keeping the closest instance and the exact flag', () => {
    const rs = [
      hit('edge', 'P1', { pointIds: ['a', 'b'], distance: 1.3, exact: true, point: [1, 0, 0] }),
      hit('edge', 'P1', { pointIds: ['a', 'b'], distance: 1.1, exact: false, point: [2, 0, 0] }),
      hit('edge', 'P1', { pointIds: ['a', 'b'], distance: 1.2, exact: false, point: [3, 0, 0] }),
      hit('vertex', 'P1', { pointId: 'a', pointIds: ['a'], distance: 1.4, exact: false }),
      hit('vertex', 'P1', { pointId: 'a', pointIds: ['a'], distance: 1.5, exact: true }),
    ];
    const ranked = rankPickCandidates(rs, 1);
    expect(ranked).toHaveLength(2);
    const edge = ranked.find((r) => r.type === 'edge')!;
    expect(edge.distance).toBe(1.1);
    expect(edge.point).toEqual([2, 0, 0]);
    expect(edge.exact).toBe(true); // one of the duplicates was an exact hit
    const vertex = ranked.find((r) => r.type === 'vertex')!;
    expect(vertex.distance).toBe(1.4);
    expect(vertex.exact).toBe(true);
    // the input is not mutated
    expect(rs[3].exact).toBe(false);
  });

  it('keeps the edges and faces of one link apart: point ids and face index are part of the identity', () => {
    const rs = [hit('edge', 'P1', { pointIds: ['a', 'b'] }), hit('edge', 'P1', { pointIds: ['b', 'c'] }), hit('face', 'P1', { pointIds: ['a', 'b', 'c'], faceIndex: 0 }), hit('face', 'P1', { pointIds: ['d', 'e', 'f'], faceIndex: 1 })];
    expect(names(rankPickCandidates(rs, 0.1))).toEqual(['edge:P1[a,b]', 'edge:P1[b,c]', 'face:P1[a,b,c]#0', 'face:P1[d,e,f]#1']);
    expect(pickKey(rs[0])).not.toBe(pickKey(rs[1]));
    expect(samePick(rs[2], rs[3])).toBe(false);
    expect(samePick(rs[0], { ...rs[0], distance: 5 })).toBe(true);
    expect(samePick(null, null)).toBe(true);
    expect(samePick(rs[0], null)).toBe(false);
  });

  it('lists two coincident edges of different links one after the other, in input order', () => {
    const rs = [hit('face', 'P2', { pointIds: ['d', 'e', 'f'], faceIndex: 0, exact: false }), hit('edge', 'P1', { pointIds: ['a', 'b'], exact: true }), hit('edge', 'P2', { pointIds: ['d', 'e'], exact: true })];
    expect(names(rankPickCandidates(rs, 0.1))).toEqual(['edge:P1[a,b]', 'edge:P2[d,e]', 'face:P2[d,e,f]#0']);
  });

  it('puts datum geometry last: points before axes before planes, then by distance', () => {
    const rs = [hit('construction', 'TOP', { sub: 2, distance: 0.5 }), hit('construction', 'AXIS2', { sub: 1, distance: 0.9 }), hit('face', 'P1', { pointIds: ['a', 'b', 'c'], faceIndex: 0, distance: 3 }), hit('construction', 'ORIGIN', { sub: 0, distance: 4 }), hit('construction', 'AXIS1', { sub: 1, distance: 0.8 })];
    expect(rankPickCandidates(rs, 0.1).map((r) => r.id)).toEqual(['P1', 'ORIGIN', 'AXIS1', 'AXIS2', 'TOP']);
  });

  it('ranks a feature deeper than the tolerance after everything in the front depth band', () => {
    // a vertex 2 units behind the panel under the pointer must not be offered before the panel's own features
    const front = [hit('face', 'P1', { pointIds: ['a', 'b', 'c'], faceIndex: 0, distance: 10 }), hit('edge', 'P1', { pointIds: ['a', 'b'], distance: 10.01 })];
    const behind = hit('vertex', 'P9', { pointId: 'z', pointIds: ['z'], distance: 12 });
    expect(names(rankPickCandidates([behind, ...front], 0.1))).toEqual(['edge:P1[a,b]', 'face:P1[a,b,c]#0', 'vertex:P9[z]']);
    // with a tolerance that spans the depth difference the priority order applies to all of them
    expect(names(rankPickCandidates([behind, ...front], 5))).toEqual(['vertex:P9[z]', 'edge:P1[a,b]', 'face:P1[a,b,c]#0']);
  });

  it('reports the depth band of every model feature; datum geometry carries none', () => {
    const front = [hit('face', 'P1', { pointIds: ['a', 'b', 'c'], faceIndex: 0, distance: 10 }), hit('edge', 'P1', { pointIds: ['a', 'b'], distance: 10.01 })];
    const behind = hit('vertex', 'P9', { pointId: 'z', pointIds: ['z'], distance: 12 });
    const farther = hit('edge', 'P7', { pointIds: ['q', 'r'], distance: 14 });
    const plane = hit('construction', 'TOP', { sub: 2, distance: 9 });
    const ranked = rankPickCandidates([plane, farther, behind, ...front], 0.1);
    expect(ranked.map((r) => `${r.id}:${r.band ?? '-'}`)).toEqual(['P1:0', 'P1:0', 'P9:1', 'P7:2', 'TOP:-']);
    // the input objects are not annotated
    expect(behind.band).toBeUndefined();
    // the hover identity ignores the band and the queried tag
    expect(samePick(ranked[2], { ...behind, band: 5, queried: true })).toBe(true);
  });

  it('opens a new depth band from the first feature outside the previous band', () => {
    const rs = [hit('face', 'A', { pointIds: ['a'], faceIndex: 0, distance: 1 }), hit('face', 'B', { pointIds: ['b'], faceIndex: 0, distance: 1.08 }), hit('vertex', 'C', { pointId: 'c', pointIds: ['c'], distance: 1.16 }), hit('vertex', 'D', { pointId: 'd', pointIds: ['d'], distance: 1.24 })];
    // bands of width 0.1 anchored at 1: [A, B] then [C, D]
    expect(rankPickCandidates(rs, 0.1).map((r) => r.id)).toEqual(['A', 'B', 'C', 'D']);
    // bands of width 0.2: [A, B, C] (C's vertex priority wins the band) then [D]
    expect(rankPickCandidates(rs, 0.2).map((r) => r.id)).toEqual(['C', 'A', 'B', 'D']);
  });
});

describe('PickCycle', () => {
  const edge1 = hit('edge', 'P1', { pointIds: ['a', 'b'] });
  const edge2 = hit('edge', 'P2', { pointIds: ['c', 'd'] });
  const face2 = hit('face', 'P2', { pointIds: ['c', 'd', 'e'], faceIndex: 0 });

  it('starts on the first candidate that is not the current hover', () => {
    const c = new PickCycle();
    expect(c.active).toBe(false);
    expect(c.index).toBe(-1);
    const first = c.start([edge1, edge2, face2], { x: 10, y: 10 }, { ...edge1, distance: 7 });
    expect(first).toBe(edge2);
    expect(c.active).toBe(true);
    expect(c.index).toBe(1);
    expect(c.count).toBe(3);
    expect(c.current).toBe(edge2);
  });

  it('starts at the top when nothing is hovered or the hover is not among the candidates', () => {
    const c = new PickCycle();
    expect(c.start([edge1, edge2], { x: 0, y: 0 }, null)).toBe(edge1);
    expect(c.start([edge1, edge2], { x: 0, y: 0 }, face2)).toBe(edge1);
    expect(c.index).toBe(0);
  });

  it('starts at the top when the only candidate is the hover', () => {
    const c = new PickCycle();
    expect(c.start([edge1], { x: 0, y: 0 }, edge1)).toBe(edge1);
    expect(c.index).toBe(0);
    expect(c.count).toBe(1);
  });

  it('next() advances and wraps around', () => {
    const c = new PickCycle();
    c.start([edge1, edge2, face2], { x: 0, y: 0 }, null);
    expect(c.next()).toBe(edge2);
    expect(c.next()).toBe(face2);
    expect(c.next()).toBe(edge1);
    expect(c.index).toBe(0);
  });

  it('isSamePlace uses a 4 px radius by default and the given radius otherwise', () => {
    const c = new PickCycle();
    expect(c.isSamePlace(100, 100)).toBe(false); // inactive: no place
    c.start([edge1], { x: 100, y: 100 }, null);
    expect(QUERY_CLICK_PX).toBe(4);
    expect(c.isSamePlace(100, 100)).toBe(true);
    expect(c.isSamePlace(103, 102)).toBe(true); // 3.6 px
    expect(c.isSamePlace(104, 0 + 100)).toBe(true); // exactly 4 px
    expect(c.isSamePlace(103, 103)).toBe(false); // 4.24 px
    expect(c.isSamePlace(110, 100, 12)).toBe(true);
  });

  it('cancel() deactivates; next() on an inactive cycle returns null', () => {
    const c = new PickCycle();
    c.start([edge1, edge2], { x: 0, y: 0 }, null);
    c.cancel();
    expect(c.active).toBe(false);
    expect(c.count).toBe(0);
    expect(c.index).toBe(-1);
    expect(c.current).toBeNull();
    expect(c.next()).toBeNull();
    expect(c.isSamePlace(0, 0)).toBe(false);
  });

  it('start() with no candidates stays inactive and returns null', () => {
    const c = new PickCycle();
    c.start([edge1], { x: 0, y: 0 }, null);
    expect(c.start([], { x: 0, y: 0 }, null)).toBeNull();
    expect(c.active).toBe(false);
  });

  it('keeps its own copy of the candidate list', () => {
    const c = new PickCycle();
    const list = [edge1, edge2];
    c.start(list, { x: 0, y: 0 }, null);
    list.pop();
    expect(c.count).toBe(2);
    expect(c.next()).toBe(edge2);
  });
});
