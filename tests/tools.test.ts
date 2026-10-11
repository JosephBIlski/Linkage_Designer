/**
 * ToolManager driven through a fake App and a fake Viewport (a pin-hole camera
 * over the TOP or FRONT view): the query cycle's life cycle, the Joint tool's
 * same-spot substitution, the Panel tool's overlap refusal and 2-D placement
 * from a camera that sits on the sketch plane.
 */
import { describe, expect, it } from 'vitest';
import type { App, ToolName } from '../src/app';
import { rayPlane } from '../src/core/edit';
import { dist, dot, normalize, scale, sub } from '../src/core/geometry';
import { addBar, addPolygonFromPoints, addPrism, bodyPlaneJoint, createModel, parseModel, serializeModel, setGround } from '../src/core/model';
import { currentViolation } from '../src/core/kinematics';
import { autoJoinCoincident } from '../src/core/edit';
import type { ID, Link, Model, Vec3 } from '../src/core/types';
import { LINK_NAMES, STATUS } from '../src/ui/strings';
import type { OverlayView, SelectionView } from '../src/viewport/render';
import type { PickResult, ViewportPointerEvent } from '../src/viewport/scene';
import { ToolManager } from '../src/viewport/tools';

const CX = 500;
const CY = 500;
/** Pixels per unit at unit depth; 50 px per unit at the sketch plane of the TOP camera. */
const F = 1000;

/** Pin-hole camera 20 units above the origin (top) or 10 units in front of it (front), screen up = +y / +z. */
class FakeViewport {
  onPointer: ((ev: ViewportPointerEvent) => void) | null = null;
  /** What pick() returns at any position. */
  pickAt: PickResult | null = null;
  /** What pickCandidates() returns at any position. */
  candidates: PickResult[] = [];
  constructor(private view: 'top' | 'front') {}
  private get cam(): Vec3 {
    return this.view === 'top' ? [0, 0, 20] : [0, -10, 0];
  }
  viewDirection(): Vec3 {
    return this.view === 'top' ? [0, 0, -1] : [0, 1, 0];
  }
  pointerRay(x: number, y: number): { o: Vec3; d: Vec3 } {
    const u = (x - CX) / F;
    const w = -(y - CY) / F;
    return { o: this.cam, d: normalize(this.view === 'top' ? [u, w, -1] : [u, 1, w]) };
  }
  worldToScreen(p: Vec3): { x: number; y: number } {
    const depth = this.view === 'top' ? 20 - p[2] : p[1] + 10;
    return this.view === 'top' ? { x: CX + (F * p[0]) / depth, y: CY - (F * p[1]) / depth } : { x: CX + (F * p[0]) / depth, y: CY - (F * p[2]) / depth };
  }
  projectToPlane(x: number, y: number, origin: Vec3, normal: Vec3): Vec3 | null {
    const r = this.pointerRay(x, y);
    return rayPlane(r.o, r.d, origin, normal);
  }
  projectToViewPlane(x: number, y: number, origin: Vec3): Vec3 | null {
    return this.projectToPlane(x, y, origin, this.viewDirection());
  }
  pick(): PickResult | null {
    return this.pickAt;
  }
  pickCandidates(): PickResult[] {
    return this.candidates;
  }
}

interface FakeApp {
  model: Model;
  viewport: FakeViewport;
  tool: ToolName;
  mode: string;
  hover: PickResult | null;
  status: string;
  hint: string;
  notice: string;
  overlay: OverlayView;
  selection: SelectionView | null;
  toolOptions: App['toolOptions'];
  settings: { gridSnap: boolean; gridStep: number };
  snapshot: string | null;
}

function fakeApp(model: Model, view: 'top' | 'front' = 'top'): { app: FakeApp; vp: FakeViewport; tools: ToolManager } {
  const vp = new FakeViewport(view);
  const app: FakeApp & Record<string, unknown> = {
    model,
    viewport: vp,
    tool: 'select',
    mode: 'construction',
    hover: null,
    status: STATUS.ready,
    hint: '',
    notice: '',
    overlay: {},
    selection: null,
    toolOptions: { sides: 4, radius: 1, height: 1, mode2d: true, planeMode: 'three', offset: 1, jointType: 'revolute', pitch: 1, patternKind: 'linear', patternCount: 3, patternAngle: 360 },
    settings: { gridSnap: false, gridStep: 0.5 },
    snapshot: null,
    setHover(h: PickResult | null) {
      app.hover = h;
    },
    setStatus(t: string) {
      app.status = t;
    },
    setHint(t: string) {
      app.hint = t;
    },
    setOverlay(o: OverlayView) {
      app.overlay = o;
    },
    setNotice(t: string) {
      app.notice = t;
    },
    report(t: string) {
      app.status = t;
      app.notice = t;
    },
    setTool(t: ToolName) {
      app.tool = t;
      app.overlay = {};
    },
    select(s: SelectionView | null) {
      app.selection = s;
    },
    requestRender() {},
    notify() {},
    beginChange() {
      if (app.snapshot === null) app.snapshot = serializeModel(app.model);
    },
    endChange() {
      app.snapshot = null;
    },
    abortChange() {
      if (app.snapshot === null) return;
      app.model = parseModel(app.snapshot); // as App.abortChange does: a fresh model object
      app.snapshot = null;
      app.selection = null;
    },
    nextLinkName(kind: string) {
      return `${LINK_NAMES[kind] ?? kind} ${Object.values(app.model.links).filter((l) => l.kind === kind).length + 1}`;
    },
  };
  const tools = new ToolManager(app as unknown as App);
  return { app, vp, tools };
}

const ev = (kind: ViewportPointerEvent['kind'], x: number, y: number, pick: PickResult | null, button = 0): ViewportPointerEvent => ({ kind, button, clientX: x, clientY: y, shiftKey: false, ctrlKey: false, altKey: false, pick, original: null as unknown as MouseEvent });
const edgeOf = (m: Model, link: Link, i: number, j: number, extra: Partial<PickResult> = {}): PickResult => {
  const a = m.points[link.pointIds[i]].pos;
  const b = m.points[link.pointIds[j]].pos;
  return { type: 'edge', id: link.id, linkId: link.id, pointIds: [link.pointIds[i], link.pointIds[j]], point: [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2], distance: 20, ...extra };
};
const vertexOf = (m: Model, link: Link, i: number): PickResult => ({ type: 'vertex', id: link.id, linkId: link.id, pointId: link.pointIds[i], pointIds: [link.pointIds[i]], point: [...m.points[link.pointIds[i]].pos] as Vec3, distance: 20 });
const creases = (m: Model) => Object.values(m.joints).filter((j) => j.type === 'revolute' && j.a.kind === 'edge' && j.pairs?.length === 2);

/** Two triangles sharing the base (0.5,0.5)–(2.5,0.5) exactly, as smoke step 20 builds them. */
function twoTriangles(): { m: Model; p1: Link; p2: Link } {
  const m = createModel();
  const p1 = addPolygonFromPoints(m, [[0.5, 0.5, 0], [2.5, 0.5, 0], [1.5, 2, 0]], { onPlaneId: 'plane_top', name: 'Polygon 1' });
  const p2 = addPolygonFromPoints(m, [[0.5, 0.5, 0], [2.5, 0.5, 0], [1.5, -1, 0]], { onPlaneId: 'plane_top', name: 'Polygon 2' });
  return { m, p1, p2 };
}

describe('Joint tool: substitution of a feature on another link at the same spot', () => {
  it('takes a candidate from the front depth band only; a link hidden behind the clicked one gives the same-link message', () => {
    const { m, p1, p2 } = twoTriangles();
    const { app, vp, tools } = fakeApp(m);
    tools.setTool('joint');
    const e1 = edgeOf(m, p1, 0, 1);
    const e2 = edgeOf(m, p2, 0, 1);
    const at = vp.worldToScreen(e1.point);
    tools.handle(ev('down', at.x, at.y, e1));
    expect(app.status).toContain(STATUS.pickSecondFeature);
    expect(app.status).toContain('Polygon 1');
    // the other link's edge is only in the band behind the clicked panel: no substitution
    vp.candidates = [{ ...e1, band: 0 }, { ...e2, band: 1, distance: 22 }];
    tools.handle(ev('down', at.x, at.y, e1));
    expect(app.status).toBe(STATUS.jointSameLink);
    expect(creases(app.model).length).toBe(0);
    // datum geometry after the model features is never substituted either
    vp.candidates = [{ ...e1, band: 0 }, { type: 'construction', id: 'axis_x', sub: 1, point: [1.5, 0, 0], distance: 20 }];
    tools.handle(ev('down', at.x, at.y, e1));
    expect(app.status).toBe(STATUS.jointSameLink);
    // in the front band the coincident edge of Polygon 2 is used and the status says so
    vp.candidates = [{ ...e1, band: 0 }, { ...e2, band: 0 }];
    tools.handle(ev('down', at.x, at.y, e1));
    expect(app.status).toContain(STATUS.pickedOtherLink('edge V0-V1 of Polygon 2'));
    expect(app.status).toContain(STATUS.jointCreated);
    expect(creases(app.model).length).toBe(1);
  });
});

describe('query cycle life cycle', () => {
  function cycleSetup() {
    const { m, p1, p2 } = twoTriangles();
    const { app, vp, tools } = fakeApp(m);
    const e1 = edgeOf(m, p1, 0, 1);
    const e2 = edgeOf(m, p2, 0, 1);
    vp.pickAt = e1;
    vp.candidates = [e1, e2];
    const at = vp.worldToScreen(e1.point);
    const startCycle = () => {
      tools.handle(ev('move', at.x, at.y, e1));
      tools.handle(ev('query', at.x, at.y, e1, 2));
    };
    return { m, p1, p2, app, vp, tools, e1, e2, at, startCycle };
  }

  it('a right-click highlights the next candidate, tagged as queried, with the counting status', () => {
    const { app, tools, e2, startCycle } = cycleSetup();
    startCycle();
    expect(tools.queryActive).toBe(true);
    expect(app.hover?.type).toBe('edge');
    expect(app.hover?.id).toBe(e2.id);
    expect(app.hover?.queried).toBe(true);
    expect(app.status).toMatch(/^2 of 2 · edge V0-V1 of Polygon 2 · /);
    expect(app.status).toContain(STATUS.queryHint);
  });

  it('a joint candidate is described as the tree describes it', () => {
    const { m, p2, app, vp, tools, e1, at, startCycle } = cycleSetup();
    const crease = autoJoinCoincident(m, p2, p2.pointIds, { defaultJoint: 'revolute', axis: [0, 0, 1] }).joints[0];
    const jp: PickResult = { type: 'joint', id: crease.id, point: e1.point, distance: 20 };
    vp.candidates = [e1, jp];
    startCycle();
    expect(app.status).toBe(`2 of 2 · Crease 180° · Polygon 2 ↔ Polygon 1 · ${STATUS.queryHint}`);
    tools.cancel();
    tools.handle(ev('move', at.x, at.y, jp));
    expect(app.status).toBe('Crease 180° · Polygon 2 ↔ Polygon 1');
  });

  it('leaving the canvas ends the cycle and clears its instructions', () => {
    const { app, tools, startCycle } = cycleSetup();
    startCycle();
    tools.handle(ev('leave', 0, 0, null));
    expect(tools.queryActive).toBe(false);
    expect(app.hover).toBeNull();
    expect(app.status).toBe('');
  });

  it('switching tools during a cycle restores the normal hover and status; the next click uses the real pick', () => {
    const { app, tools, e1, at, startCycle } = cycleSetup();
    startCycle();
    tools.setTool('joint');
    expect(tools.queryActive).toBe(false);
    expect(app.hover?.id).toBe(e1.id);
    expect(app.hover?.queried).toBeUndefined();
    expect(app.status).toBe('edge V0-V1 of Polygon 1');
    tools.handle(ev('down', at.x, at.y, e1));
    expect(app.status).toContain('edge V0-V1 of Polygon 1');
    expect(app.status).toContain(STATUS.pickSecondFeature);
  });

  it('Esc ends the cycle first (normal hover back) and only a second Esc cancels the tool', () => {
    const { app, tools, e1, startCycle } = cycleSetup();
    tools.setTool('joint');
    startCycle();
    tools.cancel();
    expect(tools.queryActive).toBe(false);
    expect(app.tool).toBe('joint');
    expect(app.hover?.id).toBe(e1.id);
    expect(app.status).not.toContain(STATUS.queryHint);
    tools.cancel();
    expect(app.tool).toBe('select');
  });

  it('a left-click with a tool that writes no status leaves the plain description of the feature it used', () => {
    const { app, tools, e1, at, startCycle } = cycleSetup();
    tools.setTool('delete');
    startCycle();
    tools.handle(ev('down', at.x, at.y, e1));
    expect(Object.values(app.model.links).map((l) => l.name)).toEqual(['Polygon 1']); // the highlighted Polygon 2 was deleted
    expect(tools.queryActive).toBe(false);
    expect(app.status).toBe('edge V0-V1 of Polygon 2');
    expect(app.status).not.toContain(STATUS.queryHint);
  });

  it('a cycle started on a model that undo replaced is inert; endQuery() before the undo restores the hover', () => {
    const { app, tools, e1, startCycle } = cycleSetup();
    startCycle();
    app.model = parseModel(serializeModel(app.model)); // what App.undo / redo / loadModel do
    expect(tools.queryActive).toBe(false);
    startCycle();
    expect(tools.queryActive).toBe(true);
    tools.endQuery();
    expect(tools.queryActive).toBe(false);
    expect(app.hover?.id).toBe(e1.id);
    expect(app.status).toBe('edge V0-V1 of Polygon 1');
  });

  it('Esc during a sketch restores the tool preview for the hover under the pointer (snap ring, marker)', () => {
    const m = createModel();
    const bar = addBar(m, [2, 0, 0], [4, 0, 0], { onPlaneId: 'plane_top', name: 'Link 1' });
    const { app, vp, tools } = fakeApp(m);
    tools.setTool('sketch');
    tools.handle(ev('down', CX, CY, null)); // a free vertex at the origin
    const vtx = vertexOf(m, bar, 0);
    const edge = edgeOf(m, bar, 0, 1);
    vp.pickAt = vtx;
    vp.candidates = [vtx, edge];
    const near = vp.worldToScreen([2.1, 0.05, 0]);
    tools.handle(ev('move', near.x, near.y, vtx));
    expect(app.overlay.snapPoint).toEqual([2, 0, 0]);
    tools.handle(ev('query', near.x, near.y, vtx, 2));
    expect(app.hover?.type).toBe('edge');
    expect(app.overlay.snapPoint).toBeNull();
    expect(dist(app.overlay.marker!, [2.1, 0.05, 0])).toBeLessThan(1e-6);
    tools.cancel();
    expect(app.hover?.type).toBe('vertex');
    expect(app.status).toBe('vertex A of Link 1');
    expect(app.overlay.snapPoint).toEqual([2, 0, 0]); // the preview agrees with what a click would do
    expect(app.overlay.marker).toEqual([2, 0, 0]);
    // and the click does what the preview showed: the second sketch vertex is the snapped vertex
    tools.handle(ev('down', near.x, near.y, vtx));
    tools.handle(ev('move', near.x + 60, near.y, null));
    expect(app.overlay.polyline![1]).toEqual([2, 0, 0]);
  });

  it('Backspace while sketching drops the removed vertex from the preview at once', () => {
    const { m } = twoTriangles();
    const { app, vp, tools } = fakeApp(m);
    tools.setTool('sketch');
    for (const p of [[-3, -3, 0], [-1, -3, 0], [-1, -1, 0]] as Vec3[]) {
      const s = vp.worldToScreen(p);
      tools.handle(ev('down', s.x, s.y, null));
    }
    const cursor = vp.worldToScreen([-3, -1, 0]);
    tools.handle(ev('move', cursor.x, cursor.y, null));
    expect(app.overlay.polyline?.length).toBe(4);
    expect(tools.popSketchPoint()).toBe(true);
    expect(app.overlay.polyline?.length).toBe(3);
  });
});

describe('Panel tool: in-plane overlap refusal', () => {
  const O: Vec3 = [0, 0, 0];
  const SIDE = Math.sqrt(3);
  const C = (k: number): Vec3 => [SIDE * Math.cos((Math.PI / 3) * k), SIDE * Math.sin((Math.PI / 3) * k), 0];
  const vertexAt = (m: Model, link: Link, p: Vec3): number => link.pointIds.findIndex((id) => dist(m.points[id].pos, p) < 1e-9);

  it('a fourth regular triangle over the flat fan is not created and the sector message names the vertex; a fan continuation is', () => {
    const m = createModel();
    const tris = [0, 1, 2].map((k) => addPolygonFromPoints(m, [O, C(k), C(k + 1)], { onPlaneId: 'plane_top', name: `T${k + 1}` }));
    setGround(m, tris[0].id);
    for (const t of tris.slice(1)) autoJoinCoincident(m, t, t.pointIds, { defaultJoint: 'revolute', axis: [0, 0, 1] });
    const { app, vp, tools } = fakeApp(m);
    tools.setTool('sketch');
    const click = (link: Link, p: Vec3) => {
      const pick = vertexOf(m, link, vertexAt(m, link, p));
      const s = vp.worldToScreen(p);
      tools.handle(ev('down', s.x, s.y, pick));
    };
    const before = serializeModel(m);
    click(tris[0], O);
    click(tris[2], C(3));
    click(tris[0], C(1));
    tools.finishSketch();
    expect(app.status).toBe(STATUS.sketchRefusedSector('T1 V0', 300));
    expect(app.notice).toBe(app.status);
    expect(serializeModel(app.model)).toBe(before);
    expect(Object.keys(app.model.links).length).toBe(3);
    // the sketch is kept for correction: two vertices removed and the fan continued instead
    expect(tools.popSketchPoint()).toBe(true);
    expect(tools.popSketchPoint()).toBe(true);
    click(tris[2], C(3));
    const s4 = vp.worldToScreen(C(4));
    tools.handle(ev('down', s4.x, s4.y, null));
    tools.finishSketch();
    expect(app.status).toBe(`${STATUS.jointCreated} (1)`);
    expect(Object.keys(app.model.links).length).toBe(4);
    expect(creases(app.model).length).toBe(3);
  });
});

describe('placement in 2-D mode from a camera on the sketch plane (Front view)', () => {
  it('off-centre clicks land on the view plane through the previous point, never at the camera', () => {
    const m = createModel();
    const { app, tools } = fakeApp(m, 'front');
    tools.setTool('bar');
    expect(app.toolOptions.mode2d).toBe(true);
    tools.handle(ev('down', CX + 120, CY + 60, null));
    tools.handle(ev('down', CX + 200, CY + 100, null));
    const bars = Object.values(app.model.links).filter((l) => l.kind === 'bar');
    expect(bars.length).toBe(1);
    const ends = bars[0].pointIds.map((id: ID) => m.points[id].pos);
    for (const p of ends) {
      expect(dist(p, [0, -10, 0])).toBeGreaterThan(1); // not at the camera
      expect(Math.abs(p[1])).toBeLessThan(1e-9); // on the view plane through the origin / the first point
    }
    expect(ends[0][0]).toBeCloseTo(1.2, 6);
    expect(ends[0][2]).toBeCloseTo(-0.6, 6);
    expect(ends[1][0]).toBeCloseTo(2, 6);
    expect(ends[1][2]).toBeCloseTo(-1, 6);
  });
});

describe('Polygon tool in 2-D mode with its circumcircle point snapped to a vertex off the sketch plane', () => {
  /** A grounded or free unit prism on TOP; the polygon's centre is clicked on the plane and its first vertex snapped to a top vertex (z = 1). */
  function draw(ground: boolean) {
    const m = createModel();
    const prism = addPrism(m, [0, 0, 0], [0, 0, 1], 1, 4, 1);
    if (ground) setGround(m, prism.id);
    const topId = prism.pointIds[4];
    const top = [...m.points[topId].pos] as Vec3;
    expect(top[2]).toBeCloseTo(1, 9);
    const { app, vp, tools } = fakeApp(m);
    tools.setTool('polygon');
    expect(app.toolOptions.mode2d).toBe(true);
    const c: Vec3 = [3, 0, 0];
    const sc = vp.worldToScreen(c);
    tools.handle(ev('down', sc.x, sc.y, null));
    const st = vp.worldToScreen(top);
    tools.handle(ev('down', st.x, st.y, vertexOf(m, prism, 4)));
    const poly = Object.values(app.model.links).find((l) => l.kind === 'polygon')!;
    return { m: app.model, app, prism, poly, c, top, topId };
  }

  it('tilts the polygon to meet the snapped vertex exactly, joins it there and drops the 2-D constraint, saying so', () => {
    const { m, app, poly, c, top } = draw(true);
    expect(poly).toBeDefined();
    expect(bodyPlaneJoint(m, poly.id)).toBeNull(); // no "Keep on sketch plane" joint on a tilted polygon
    expect(dist(m.points[poly.pointIds[0]].pos, top)).toBeLessThan(1e-9);
    const pins = Object.values(m.joints).filter((j) => j.a.kind === 'vertex');
    expect(pins.length).toBe(1);
    expect(pins[0].type).toBe('spherical');
    expect(app.status).toBe(`${STATUS.jointCreated} (1) · ${STATUS.sketchOffPlane}`);
    expect(currentViolation(m)).toBeLessThan(1e-9);
    // the polygon's plane passes through the centre and the snapped vertex, and is the one closest to the sketch plane
    const pts = poly.pointIds.map((id) => m.points[id].pos);
    const e = normalize(sub(top, c));
    const n = normalize(sub([0, 0, 1], scale(e, e[2])));
    for (const p of pts) expect(Math.abs(dot(sub(p, c), n))).toBeLessThan(1e-9);
    expect(pts.some((p) => Math.abs(p[2]) > 0.1)).toBe(true);
    // the ground stayed where it was: nothing was pulled towards the polygon
    for (const p of pts) expect(dist(p, c)).toBeCloseTo(dist(top, c), 9);
    expect(m.points[poly.pointIds[0]].pos[2]).toBeCloseTo(1, 9);
  });

  it('does the same on a free prism, which is not dragged out of shape by the join', () => {
    const { m, prism, poly, top } = draw(false);
    expect(bodyPlaneJoint(m, poly.id)).toBeNull();
    expect(dist(m.points[poly.pointIds[0]].pos, top)).toBeLessThan(1e-9);
    for (const id of prism.pointIds.slice(4)) expect(m.points[id].pos[2]).toBeCloseTo(1, 9);
    for (const id of prism.pointIds.slice(0, 4)) expect(m.points[id].pos[2]).toBeCloseTo(0, 9);
    expect(currentViolation(m)).toBeLessThan(1e-9);
  });

  it('a snapped vertex on the sketch plane keeps the 2-D constraint as before', () => {
    const m = createModel();
    const bar = addBar(m, [3, 0, 0], [5, 0, 0], { onPlaneId: 'plane_top' });
    const { app, vp, tools } = fakeApp(m);
    tools.setTool('polygon');
    const sc = vp.worldToScreen([2, 0, 0]);
    tools.handle(ev('down', sc.x, sc.y, null));
    const sv = vp.worldToScreen([3, 0, 0]);
    tools.handle(ev('down', sv.x, sv.y, vertexOf(m, bar, 0)));
    const poly = Object.values(app.model.links).find((l) => l.kind === 'polygon')!;
    expect(bodyPlaneJoint(app.model, poly.id)).not.toBeNull();
    expect(app.status).toBe(`${STATUS.jointCreated} (1)`);
  });
});
