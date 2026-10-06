/**
 * Built-in example mechanisms. Used by the UI "Examples" menu and by the tests.
 * Names/labels shown in the UI come from src/ui/strings.ts (EXAMPLES).
 */
import { addAngleDriver, addBar, addConstructionAxis, addFoldDriver, addJoint, addPolygon, addPolygonFromPoints, createModel, setGround } from './model';
import type { Feature, Model, Vec3 } from './types';
import { syncDriverValues } from './kinematics';

const vtx = (linkId: string, pid: string): Feature => ({ linkId, kind: 'vertex', pointIds: [pid] });
const edge = (linkId: string, a: string, b: string): Feature => ({ linkId, kind: 'edge', pointIds: [a, b] });

/** Intersection of two circles in the XY plane (upper solution). */
export function circleIntersect(c1: Vec3, r1: number, c2: Vec3, r2: number, upper = true): Vec3 {
  const dx = c2[0] - c1[0];
  const dy = c2[1] - c1[1];
  const d = Math.hypot(dx, dy);
  const a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
  const h = Math.sqrt(Math.max(0, r1 * r1 - a * a));
  const mx = c1[0] + (a * dx) / d;
  const my = c1[1] + (a * dy) / d;
  const s = upper ? 1 : -1;
  return [mx - (s * h * dy) / d, my + (s * h * dx) / d, 0];
}

/**
 * Grashof crank-rocker four-bar with a triangular coupler.
 * Ground 4, crank 1, coupler 3.5, follower 3 (s + l = 5 < 6.5).
 */
export function fourBarCrankRocker(): Model {
  const m = createModel();
  const plane = m.settings.sketchPlaneId;
  const O2: Vec3 = [0, 0, 0];
  const O4: Vec3 = [4, 0, 0];
  const A: Vec3 = [0, 1, 0];
  const B = circleIntersect(A, 3.5, O4, 3, true);
  const C: Vec3 = [(A[0] + B[0]) / 2 - (B[1] - A[1]) * 0.5, (A[1] + B[1]) / 2 + (B[0] - A[0]) * 0.5, 0];
  const ground = addBar(m, O2, O4, { name: 'Ground', onPlaneId: plane });
  const crank = addBar(m, O2, A, { name: 'Crank', onPlaneId: plane });
  const coupler = addPolygonFromPoints(m, [A, B, C], { name: 'Coupler', onPlaneId: plane });
  const follower = addBar(m, O4, B, { name: 'Follower', onPlaneId: plane });
  setGround(m, ground.id);
  addJoint(m, 'revolute', vtx(ground.id, ground.pointIds[0]), vtx(crank.id, crank.pointIds[0]));
  addJoint(m, 'revolute', vtx(crank.id, crank.pointIds[1]), vtx(coupler.id, coupler.pointIds[0]));
  addJoint(m, 'revolute', vtx(coupler.id, coupler.pointIds[1]), vtx(follower.id, follower.pointIds[1]));
  addJoint(m, 'revolute', vtx(follower.id, follower.pointIds[0]), vtx(ground.id, ground.pointIds[1]));
  addAngleDriver(m, crank.id, crank.pointIds[0], crank.pointIds[1], [0, 0, 1]);
  m.settings.displayPointIds = [coupler.pointIds[2]];
  syncDriverValues(m);
  return m;
}

/** Slider-crank: crank 1, connecting rod 3, slider on the X axis (prismatic to construction axis). */
export function sliderCrank(): Model {
  const m = createModel();
  const plane = m.settings.sketchPlaneId;
  const O: Vec3 = [0, 0, 0];
  const A: Vec3 = [Math.cos(Math.PI / 3), Math.sin(Math.PI / 3), 0];
  const bx = A[0] + Math.sqrt(9 - A[1] * A[1]);
  const B: Vec3 = [bx, 0, 0];
  const ground = addBar(m, [-1, 0, 0], O, { name: 'Ground', onPlaneId: plane });
  const crank = addBar(m, O, A, { name: 'Crank', onPlaneId: plane });
  const rod = addBar(m, A, B, { name: 'Connecting rod', onPlaneId: plane });
  // slider square: vertex 0 sits on the rod pin B, its bottom edge (vertices 2-3) slides on a construction axis
  const r = 0.4;
  const h = r * Math.SQRT1_2;
  const slider = addPolygon(m, [B[0] - h, B[1] - h, 0], [0, 0, 1], r, 4, { name: 'Slider', onPlaneId: plane, startAngle: Math.PI / 4 });
  setGround(m, ground.id);
  addJoint(m, 'revolute', vtx(ground.id, ground.pointIds[1]), vtx(crank.id, crank.pointIds[0]));
  addJoint(m, 'revolute', vtx(crank.id, crank.pointIds[1]), vtx(rod.id, rod.pointIds[0]));
  const sv = slider.pointIds;
  addJoint(m, 'revolute', vtx(rod.id, rod.pointIds[1]), vtx(slider.id, sv[0]));
  const axis = addConstructionAxis(m, [0, B[1] - 2 * h, 0], [1, 0, 0], 'SLIDE');
  addJoint(m, 'prismatic', edge(slider.id, sv[2], sv[3]), { constructionId: axis.id });
  addAngleDriver(m, crank.id, crank.pointIds[0], crank.pointIds[1], [0, 0, 1]);
  m.settings.displayPointIds = [rod.pointIds[1]];
  syncDriverValues(m);
  return m;
}

/** Spherical pendulum: a bar on a spherical joint at the origin (output path = sphere, design space = ball). */
export function sphericalPendulum(): Model {
  const m = createModel();
  const bar = addBar(m, [0, 0, 0], [1.5, 1.0, 1.2], { name: 'Pendulum' });
  addJoint(m, 'spherical', vtx(bar.id, bar.pointIds[0]), { constructionId: 'point_origin' });
  addAngleDriver(m, bar.id, bar.pointIds[0], bar.pointIds[1], [0, 0, 1]);
  addAngleDriver(m, bar.id, bar.pointIds[0], bar.pointIds[1], [0, 1, 0]);
  m.settings.displayPointIds = [bar.pointIds[1]];
  syncDriverValues(m);
  return m;
}

/**
 * Rigid-origami degree-4 vertex (Miura-ori unit): four quad panels around a
 * central vertex with sector angles (α, π−α, α, π−α). One degree of freedom.
 */
export function origamiMiuraVertex(alphaDeg = 60): Model {
  const m = createModel();
  const a = (alphaDeg * Math.PI) / 180;
  const sectors = [a, Math.PI - a, a, Math.PI - a];
  const R = 2;
  const dirs: Vec3[] = [];
  let ang = 0;
  for (let i = 0; i < 4; i++) {
    dirs.push([Math.cos(ang), Math.sin(ang), 0]);
    ang += sectors[i];
  }
  const O: Vec3 = [0, 0, 0];
  const panels = [];
  for (let i = 0; i < 4; i++) {
    const d0 = dirs[i];
    const d1 = dirs[(i + 1) % 4];
    const p0: Vec3 = [d0[0] * R, d0[1] * R, 0];
    const p1: Vec3 = [d1[0] * R, d1[1] * R, 0];
    const far: Vec3 = [p0[0] + p1[0], p0[1] + p1[1], 0];
    panels.push(addPolygonFromPoints(m, [O, p0, far, p1], { name: `Panel ${i + 1}` }));
  }
  setGround(m, panels[0].id);
  // crease i between panel i (edge O→p1 = vertices 0,3) and panel i+1 (edge O→p0 = vertices 0,1)
  const joints = [];
  for (let i = 0; i < 4; i++) {
    const pa = panels[i];
    const pb = panels[(i + 1) % 4];
    joints.push(addJoint(m, 'revolute', edge(pa.id, pa.pointIds[0], pa.pointIds[3]), edge(pb.id, pb.pointIds[0], pb.pointIds[1]))!);
  }
  // pre-fold slightly so the solver starts off the flat (singular) state
  prefoldMiura(m, panels.map((p) => p.pointIds), 0.35);
  addFoldDriver(m, joints[0].id);
  m.settings.displayPointIds = [panels[2].pointIds[2]];
  syncDriverValues(m);
  return m;
}

/** Give the Miura vertex a small initial fold (z offsets on the far corners) so it is not flat. */
function prefoldMiura(m: Model, panelPts: string[][], amount: number): void {
  // lift alternate far corners up/down; the solver will project onto the rigid-folding manifold
  panelPts.forEach((ids, i) => {
    const far = m.points[ids[2]];
    far.pos = [far.pos[0], far.pos[1], (i % 2 === 0 ? 1 : -1) * amount];
  });
  // ground panel stays flat
  const g = panelPts[0];
  m.points[g[2]].pos[2] = 0;
}

export const EXAMPLE_BUILDERS: Record<string, () => Model> = {
  fourBar: fourBarCrankRocker,
  sliderCrank,
  sphericalPendulum,
  miuraVertex: () => origamiMiuraVertex(60),
};
