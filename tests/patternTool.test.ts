/**
 * The Crease pattern tool (docs/CONSTRUCTION_PLAN.md, 3b): the DOM-free
 * helpers of src/viewport/patternPolyline.ts (pattern choice, the snap onto
 * a crease, the polyline state machine) and the ToolManager's crease-pattern
 * flow driven through a fake App and a pin-hole Viewport over the TOP view:
 * clicks, snaps to pattern vertices and creases, Enter / Esc / double-click,
 * Backspace, typed coordinates, selection and deletion.
 */
import { describe, expect, it } from 'vitest';
import type { App, ToolName } from '../src/app';
import { addPattern, addPatternCrease, addPatternVertex, generatePanels, patternFaces } from '../src/core/creasePattern';
import { rayPlane } from '../src/core/edit';
import { dist } from '../src/core/geometry';
import { createModel, parseModel, serializeModel } from '../src/core/model';
import type { CreasePattern, ID, Model, PatternCrease, Vec3 } from '../src/core/types';
import { LINK_NAMES, PATTERN, STATUS } from '../src/ui/strings';
import type { OverlayView, SelectionView } from '../src/viewport/render';
import type { PickResult, ViewportPointerEvent } from '../src/viewport/scene';
import { PatternPolyline, choosePattern, nextPatternName, selectedPatternId, snapToCrease } from '../src/viewport/patternPolyline';
import { ToolManager } from '../src/viewport/tools';

// ---------------------------------------------------------------------------
// Fake app and viewport (the harness of tests/tools.test.ts)
// ---------------------------------------------------------------------------

const CX = 500;
const CY = 500;
/** Pixels per unit at unit depth: 50 px per unit on the sketch plane of the TOP camera, 20 units above it. */
const F = 1000;

class FakeViewport {
  onPointer: ((ev: ViewportPointerEvent) => void) | null = null;
  pickAt: PickResult | null = null;
  candidates: PickResult[] = [];
  viewDirection(): Vec3 {
    return [0, 0, -1];
  }
  pointerRay(x: number, y: number): { o: Vec3; d: Vec3 } {
    const u = (x - CX) / F;
    const w = -(y - CY) / F;
    const L = Math.hypot(u, w, 1);
    return { o: [0, 0, 20], d: [u / L, w / L, -1 / L] };
  }
  worldToScreen(p: Vec3): { x: number; y: number } {
    const depth = 20 - p[2];
    return { x: CX + (F * p[0]) / depth, y: CY - (F * p[1]) / depth };
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
  /** Undo entries recorded by endChange, as App records them (one per change that altered the model). */
  undo: string[];
}

function fakeApp(model: Model): { app: FakeApp; vp: FakeViewport; tools: ToolManager } {
  const vp = new FakeViewport();
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
    toolOptions: { sides: 4, radius: 1, height: 1, mode2d: true, planeMode: 'three', offset: 1, jointType: 'revolute', pitch: 1, patternKind: 'linear', patternCount: 3, patternAngle: 360, creaseClass: 'U' },
    settings: { gridSnap: false, gridStep: 0.5 },
    snapshot: null,
    undo: [],
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
      if (app.snapshot !== null && app.snapshot !== serializeModel(app.model)) app.undo.push(app.snapshot);
      app.snapshot = null;
    },
    abortChange() {
      if (app.snapshot === null) return;
      app.model = parseModel(app.snapshot);
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

/** Click at a world position on the sketch plane, with an optional pick under the pointer. */
function clickAt(tools: ToolManager, vp: FakeViewport, p: Vec3, pick: PickResult | null = null): void {
  const s = vp.worldToScreen(p);
  tools.handle(ev('move', s.x, s.y, pick));
  tools.handle(ev('down', s.x, s.y, pick));
  tools.handle(ev('up', s.x, s.y, pick));
}

const onlyPattern = (m: Model): CreasePattern => {
  const ps = Object.values(m.patterns ?? {});
  expect(ps.length).toBe(1);
  return ps[0];
};
const vertexAt = (p: CreasePattern, pos: Vec3): string | undefined => Object.values(p.vertices).find((v) => dist(v.pos, pos) < 1e-9)?.id;
const creaseCount = (p: CreasePattern): number => Object.keys(p.creases).length;
const vertexCount = (p: CreasePattern): number => Object.keys(p.vertices).length;
const pvertexPick = (p: CreasePattern, id: ID): PickResult => ({ type: 'pvertex', id: p.id, pointId: id, point: [...p.vertices[id].pos] as Vec3, distance: 20 });
const pcreasePick = (p: CreasePattern, c: PatternCrease, at: Vec3): PickResult => ({ type: 'pcrease', id: p.id, creaseId: c.id, point: at, distance: 20 });

// ---------------------------------------------------------------------------
// Pure helpers
// ---------------------------------------------------------------------------

describe('choosePattern / selectedPatternId / nextPatternName', () => {
  it('a selected pattern (or its vertex / crease) on the sketch plane is used, hidden or not; otherwise the single visible one; otherwise none', () => {
    const m = createModel();
    expect(choosePattern(m, null, 'plane_top')).toBeNull();
    const a = addPattern(m, 'plane_top', 'A');
    expect(choosePattern(m, null, 'plane_top')?.id).toBe(a.id);
    expect(choosePattern(m, null, 'plane_front')).toBeNull();
    const b = addPattern(m, 'plane_top', 'B');
    // two visible patterns on the plane: none is chosen implicitly
    expect(choosePattern(m, null, 'plane_top')).toBeNull();
    expect(choosePattern(m, { type: 'pattern', id: b.id }, 'plane_top')?.id).toBe(b.id);
    expect(choosePattern(m, { type: 'pvertex', id: a.id, pointId: 'x' }, 'plane_top')?.id).toBe(a.id);
    expect(choosePattern(m, { type: 'pcrease', id: a.id, creaseId: 'x' }, 'plane_top')?.id).toBe(a.id);
    // the selection names a pattern on another plane: not used
    const f = addPattern(m, 'plane_front', 'F');
    expect(choosePattern(m, { type: 'pattern', id: f.id }, 'plane_top')).toBeNull();
    // a hidden pattern is used only through the selection
    b.hidden = true;
    expect(choosePattern(m, null, 'plane_top')?.id).toBe(a.id);
    expect(choosePattern(m, { type: 'pattern', id: b.id }, 'plane_top')?.id).toBe(b.id);
    expect(selectedPatternId({ type: 'link', id: 'l1' })).toBeNull();
    expect(selectedPatternId({ type: 'pattern', id: a.id })).toBe(a.id);
  });

  it('nextPatternName takes the smallest unused number', () => {
    const m = createModel();
    expect(nextPatternName(m, 'Pattern')).toBe('Pattern 1');
    addPattern(m, 'plane_top', 'Pattern 1');
    addPattern(m, 'plane_top', 'Pattern 3');
    expect(nextPatternName(m, 'Pattern')).toBe('Pattern 2');
  });
});

describe('snapToCrease', () => {
  it('projects onto the crease line and clamps to the segment', () => {
    const m = createModel();
    const p = addPattern(m, 'plane_top');
    const a = addPatternVertex(m, p, [0, 0, 0]).vertex.id;
    const b = addPatternVertex(m, p, [4, 0, 0]).vertex.id;
    const r = addPatternCrease(m, p, a, b, 'M');
    const c = p.creases[r.ok ? r.creaseIds[0] : ''];
    expect(snapToCrease(p, c, [1, 0.3, 0.1])).toEqual([1, 0, 0]);
    expect(snapToCrease(p, c, [-2, 1, 0])).toEqual([0, 0, 0]);
    expect(snapToCrease(p, c, [9, -1, 0])).toEqual([4, 0, 0]);
    expect(snapToCrease(p, { id: 'x', a: 'missing', b, cls: 'U' }, [1, 0, 0])).toBeNull();
  });
});

describe('PatternPolyline', () => {
  it('adds a vertex per point and a crease from the previous one; pop restores the snapshot of the click taken back', () => {
    const m = createModel();
    const p = addPattern(m, 'plane_top');
    const pl = new PatternPolyline();
    expect(pl.active).toBe(false);
    pl.begin(p.id);
    const r0 = pl.addPoint(m, p, [0, 0, 0], 'M');
    expect(r0.crease).toBeNull();
    expect(r0.vertex.existing).toBe(false);
    expect(pl.active).toBe(true);
    expect(pl.length).toBe(1);
    const r1 = pl.addPoint(m, p, [2, 0, 0], 'M');
    expect(r1.crease?.ok).toBe(true);
    expect(creaseCount(p)).toBe(1);
    expect(p.creases[r1.crease && r1.crease.ok ? r1.crease.creaseIds[0] : ''].cls).toBe('M');
    // the same point again: nothing happens, no segment recorded
    const same = pl.addPoint(m, p, [2, 0, 0], 'M');
    expect(same.same).toBe(true);
    expect(pl.length).toBe(2);
    // a third point crossing nothing, class V
    pl.addPoint(m, p, [2, 2, 0], 'V');
    expect(creaseCount(p)).toBe(2);
    // pop: the segment's snapshot is the model before the third click, the anchor goes back to the second vertex
    const seg = pl.pop();
    expect(seg?.from).toBe(r1.vertex.vertex.id);
    expect(pl.last).toBe(r1.vertex.vertex.id);
    const restored = parseModel(seg!.snapshot);
    expect(Object.keys(restored.patterns![p.id].creases).length).toBe(1);
    expect(Object.keys(restored.patterns![p.id].vertices).length).toBe(2);
    // popping the first vertex ends the polyline
    pl.pop();
    expect(pl.active).toBe(true);
    pl.pop();
    expect(pl.active).toBe(false);
    expect(pl.patternId).toBeNull();
    expect(pl.pop()).toBeNull();
  });

  it('a refused crease (duplicate) still moves the anchor and records a segment', () => {
    const m = createModel();
    const p = addPattern(m, 'plane_top');
    const pl = new PatternPolyline();
    pl.begin(p.id);
    pl.addPoint(m, p, [0, 0, 0], 'U');
    const b = pl.addPoint(m, p, [2, 0, 0], 'U').vertex.vertex.id;
    pl.addPoint(m, p, [0, 0, 0], 'U');
    const r = pl.addPoint(m, p, [2, 0, 0], 'U');
    expect(r.crease && !r.crease.ok && r.crease.reason).toBe('duplicate');
    expect(pl.last).toBe(b);
    expect(creaseCount(p)).toBe(1);
  });
});

// ---------------------------------------------------------------------------
// The tool through the ToolManager
// ---------------------------------------------------------------------------

describe('Crease pattern tool', () => {
  it('creates a pattern on the sketch plane on the first click, one undoable change per click, and selects it', () => {
    const { app, vp, tools } = fakeApp(createModel());
    tools.setTool('cpattern');
    app.toolOptions.creaseClass = 'B';
    clickAt(tools, vp, [0, 0, 0]);
    const p = onlyPattern(app.model);
    expect(p.planeId).toBe('plane_top');
    expect(p.name).toBe(`${PATTERN.name} 1`);
    expect(vertexCount(p)).toBe(1);
    expect(app.selection).toEqual({ type: 'pattern', id: p.id });
    expect(app.status).toBe(STATUS.patternVertexPlaced(p.name));
    expect(app.undo.length).toBe(1);
    clickAt(tools, vp, [2, 0, 0]);
    expect(creaseCount(p)).toBe(1);
    expect(Object.values(p.creases)[0].cls).toBe('B');
    expect(app.status).toBe(STATUS.patternCreaseAdded('B', 1));
    expect(app.undo.length).toBe(2);
    // the rubber band follows the pointer from the last vertex
    const s = vp.worldToScreen([2, 1.5, 0]);
    tools.handle(ev('move', s.x, s.y, null));
    expect(app.overlay.rubberBand?.a).toEqual([2, 0, 0]);
    expect(app.overlay.rubberBand && dist(app.overlay.rubberBand.b, [2, 1.5, 0])).toBeLessThan(1e-6);
    expect(app.overlay.snapPoint).toBeNull();
  });

  it('a square closed by snapping onto its first vertex, then a valley diagonal: 2 faces, the classes as drawn', () => {
    const { app, vp, tools } = fakeApp(createModel());
    tools.setTool('cpattern');
    app.toolOptions.creaseClass = 'B';
    for (const q of [[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0]] as Vec3[]) clickAt(tools, vp, q);
    const p = onlyPattern(app.model);
    const a = vertexAt(p, [0, 0, 0])!;
    clickAt(tools, vp, [0.02, 0.01, 0], pvertexPick(p, a)); // the pointer rests a little off: the pick snaps
    expect(vertexCount(p)).toBe(4);
    expect(creaseCount(p)).toBe(4);
    tools.finish(); // Enter
    expect(app.status).toBe(STATUS.patternPolylineEnded);
    expect(app.overlay).toEqual({});
    app.toolOptions.creaseClass = 'V';
    clickAt(tools, vp, [0, 0, 0], pvertexPick(p, a));
    clickAt(tools, vp, [2, 2, 0], pvertexPick(p, vertexAt(p, [2, 2, 0])!));
    tools.finish();
    expect(vertexCount(p)).toBe(4);
    expect(creaseCount(p)).toBe(5);
    const faces = patternFaces(app.model, p);
    expect(faces.faces.length).toBe(2);
    expect(faces.interior.length).toBe(1);
    expect(p.creases[faces.interior[0]].cls).toBe('V');
    expect(faces.border.every((id) => p.creases[id].cls === 'B')).toBe(true);
    // the same pattern was used for the second polyline because it was selected
    expect(Object.keys(app.model.patterns ?? {}).length).toBe(1);
  });

  it('a click snapped onto a crease splits it there; a crossing crease is split at the intersection', () => {
    const { app, vp, tools } = fakeApp(createModel());
    tools.setTool('cpattern');
    clickAt(tools, vp, [0, 0, 0]);
    clickAt(tools, vp, [4, 0, 0]);
    tools.finish();
    const p = onlyPattern(app.model);
    const c = Object.values(p.creases)[0];
    // snapped onto the crease a little off its line: the vertex lands on the foot point and splits the crease
    clickAt(tools, vp, [1, 0.3, 0], pcreasePick(p, c, [1, 0.3, 0]));
    expect(vertexCount(p)).toBe(3);
    expect(creaseCount(p)).toBe(2);
    expect(vertexAt(p, [1, 0, 0])).toBeDefined();
    expect(app.overlay.rubberBand?.a).toEqual([1, 0, 0]);
    // a crease from there across the other piece: the crossing inserts a vertex and splits both
    clickAt(tools, vp, [3, -1, 0]);
    clickAt(tools, vp, [3, 1, 0]);
    expect(vertexAt(p, [3, 0, 0])).toBeDefined();
    expect(vertexCount(p)).toBe(6);
    expect(creaseCount(p)).toBe(6); // the crossed piece and the new crease are both split at (3, 0)
    expect(app.status).toBe(STATUS.patternCreaseAdded('U', 2));
  });

  it('Backspace takes back the last click exactly (splits included) as one undoable change; taking back the first vertex removes the pattern it created', () => {
    const { app, vp, tools } = fakeApp(createModel());
    tools.setTool('cpattern');
    clickAt(tools, vp, [0, 0, 0]);
    clickAt(tools, vp, [4, 0, 0]);
    tools.finish();
    const before = serializeModel(app.model);
    const p = onlyPattern(app.model);
    const c = Object.values(p.creases)[0];
    expect(tools.popSketchSupported()).toBe(false);
    clickAt(tools, vp, [1, 0, 0], pcreasePick(p, c, [1, 0, 0]));
    clickAt(tools, vp, [1, 2, 0]);
    expect(Object.keys(onlyPattern(app.model).creases).length).toBe(3);
    const undoBefore = app.undo.length;
    expect(tools.popSketchSupported()).toBe(true);
    expect(tools.popSketchPoint()).toBe(true);
    expect(app.status).toBe(STATUS.patternCreaseRemoved);
    expect(app.undo.length).toBe(undoBefore + 1);
    expect(Object.keys(onlyPattern(app.model).creases).length).toBe(2);
    expect(app.selection).toEqual({ type: 'pattern', id: p.id });
    // the polyline continues from the split vertex
    const s = vp.worldToScreen([1, 1, 0]);
    tools.handle(ev('move', s.x, s.y, null));
    expect(app.overlay.rubberBand?.a).toEqual([1, 0, 0]);
    expect(tools.popSketchPoint()).toBe(true);
    expect(app.status).toBe(STATUS.patternVertexRemoved);
    expect(serializeModel(app.model)).toBe(before);
    expect(tools.popSketchSupported()).toBe(false);
    expect(app.overlay).toEqual({});
    // a fresh polyline whose first click created the pattern: Backspace removes the pattern with the vertex
    const { app: app2, vp: vp2, tools: tools2 } = fakeApp(createModel());
    tools2.setTool('cpattern');
    clickAt(tools2, vp2, [0, 0, 0]);
    expect(Object.keys(app2.model.patterns ?? {}).length).toBe(1);
    expect(tools2.popSketchPoint()).toBe(true);
    expect(Object.keys(app2.model.patterns ?? {}).length).toBe(0);
    expect(app2.selection).toBeNull();
  });

  it('Esc ends the polyline first and only a second Esc cancels the tool; a double-click ends it too', () => {
    const { app, vp, tools } = fakeApp(createModel());
    tools.setTool('cpattern');
    clickAt(tools, vp, [0, 0, 0]);
    clickAt(tools, vp, [2, 0, 0]);
    tools.cancel();
    expect(app.tool).toBe('cpattern');
    expect(app.status).toBe(STATUS.patternPolylineEnded);
    tools.cancel();
    expect(app.tool).toBe('select');
    const p = onlyPattern(app.model);
    expect(creaseCount(p)).toBe(1);
    tools.setTool('cpattern');
    clickAt(tools, vp, [0, 2, 0]);
    clickAt(tools, vp, [2, 2, 0]);
    // the second click of the double-click lands on the last vertex and is ignored; dblclick ends the polyline
    const s = vp.worldToScreen([2, 2, 0]);
    tools.handle(ev('down', s.x + 1, s.y + 1, null));
    tools.handle(ev('dblclick', s.x + 1, s.y + 1, null));
    expect(vertexCount(p)).toBe(4);
    expect(creaseCount(p)).toBe(2);
    expect(app.status).toBe(STATUS.patternPolylineEnded);
  });

  it('typed coordinates place vertices (relative ones from the last vertex); a duplicate crease is refused with its reason', () => {
    const { app, tools } = fakeApp(createModel());
    tools.setTool('cpattern');
    app.toolOptions.creaseClass = 'M';
    tools.applyTyped({ kind: 'abs', p: [1, 1, 0] });
    tools.applyTyped({ kind: 'rel', d: [2, 0, 0] });
    tools.applyTyped({ kind: 'polar', L: 2, angleDeg: 90 });
    const p = onlyPattern(app.model);
    expect(vertexAt(p, [3, 1, 0])).toBeDefined();
    expect(vertexAt(p, [3, 3, 0])).toBeDefined();
    expect(creaseCount(p)).toBe(2);
    expect(Object.values(p.creases).every((c) => c.cls === 'M')).toBe(true);
    // back to the second vertex and over the existing crease again: refused, the anchor moves there
    tools.applyTyped({ kind: 'abs', p: [3, 1, 0] });
    tools.applyTyped({ kind: 'abs', p: [3, 3, 0] });
    expect(app.status).toBe(STATUS.patternCreaseRefused('duplicate'));
    expect(creaseCount(p)).toBe(2);
    expect(vertexCount(p)).toBe(3);
  });

  it('the selected pattern on the sketch plane is drawn into (shown again when hidden); with none selected a lone visible pattern is used', () => {
    const m = createModel();
    const a = addPattern(m, 'plane_top', 'A');
    const { app, vp, tools } = fakeApp(m);
    tools.setTool('cpattern');
    clickAt(tools, vp, [0, 0, 0]);
    expect(Object.keys(app.model.patterns!).length).toBe(1);
    expect(vertexCount(app.model.patterns![a.id])).toBe(1);
    tools.finish();
    app.model.patterns![a.id].hidden = true;
    app.selection = null;
    clickAt(tools, vp, [5, 5, 0]);
    // the hidden pattern is not chosen implicitly: a second pattern is created
    expect(Object.keys(app.model.patterns!).length).toBe(2);
    tools.finish();
    app.selection = { type: 'pattern', id: a.id };
    clickAt(tools, vp, [1, 0, 0]);
    expect(Object.keys(app.model.patterns!).length).toBe(2);
    expect(vertexCount(app.model.patterns![a.id])).toBe(2);
    expect(app.model.patterns![a.id].hidden).toBe(false);
  });

  it('Select tool: a click on a pattern vertex or crease selects it; the status bar describes it; Delete removes it', () => {
    const { app, vp, tools } = fakeApp(createModel());
    tools.setTool('cpattern');
    app.toolOptions.creaseClass = 'V';
    clickAt(tools, vp, [0, 0, 0]);
    clickAt(tools, vp, [2, 0, 0]);
    clickAt(tools, vp, [2, 2, 0]);
    tools.finish();
    const p = onlyPattern(app.model);
    const c = Object.values(p.creases)[0];
    const vid = vertexAt(p, [2, 2, 0])!;
    tools.setTool('select');
    const cp = pcreasePick(p, c, [1, 0, 0]);
    const s = vp.worldToScreen([1, 0, 0]);
    tools.handle(ev('move', s.x, s.y, cp));
    expect(app.status).toBe(PATTERN.describeCrease('V', p.name));
    clickAt(tools, vp, [1, 0, 0], cp);
    expect(app.selection).toEqual({ type: 'pcrease', id: p.id, creaseId: c.id });
    const vpick = pvertexPick(p, vid);
    const sv = vp.worldToScreen([2, 2, 0]);
    tools.handle(ev('move', sv.x, sv.y, vpick));
    expect(app.status).toBe(PATTERN.describeVertex(p.name));
    clickAt(tools, vp, [2, 2, 0], vpick);
    expect(app.selection).toEqual({ type: 'pvertex', id: p.id, pointId: vid });
    // dragging a pattern vertex (or crease) moves nothing and says so; the model and the undo history are untouched
    const beforeDrag = serializeModel(app.model);
    const undoBefore = app.undo.length;
    for (const pick of [vpick, cp]) {
      tools.handle(ev('move', sv.x, sv.y, pick));
      tools.handle(ev('down', sv.x, sv.y, pick));
      tools.handle(ev('move', sv.x + 40, sv.y, null));
      tools.handle(ev('up', sv.x + 40, sv.y, null));
      expect(app.status).toBe(STATUS.patternDragNothing);
      expect(serializeModel(app.model)).toBe(beforeDrag);
      expect(app.undo.length).toBe(undoBefore);
    }
    // the Edit tool refuses a pattern vertex with the same message
    tools.setTool('edit');
    clickAt(tools, vp, [2, 2, 0], vpick);
    expect(app.status).toBe(STATUS.patternDragNothing);
    expect(serializeModel(app.model)).toBe(beforeDrag);
    tools.setTool('select');
    // delete the vertex: its crease goes with it; then the crease; then the pattern
    tools.deletePick({ type: 'pvertex', id: p.id, pointId: vid });
    expect(vertexCount(p)).toBe(2);
    expect(creaseCount(p)).toBe(1);
    expect(app.selection).toBeNull();
    tools.deletePick({ type: 'pcrease', id: p.id, creaseId: c.id });
    expect(creaseCount(p)).toBe(0);
    tools.deletePick({ type: 'pattern', id: p.id });
    expect(Object.keys(app.model.patterns ?? {}).length).toBe(0);
  });

  it('deleting the pattern being drawn ends the polyline; the next click starts a new pattern', () => {
    const { app, vp, tools } = fakeApp(createModel());
    tools.setTool('cpattern');
    clickAt(tools, vp, [0, 0, 0]);
    clickAt(tools, vp, [2, 0, 0]);
    const p = onlyPattern(app.model);
    tools.deletePick({ type: 'pattern', id: p.id });
    expect(tools.popSketchSupported()).toBe(false);
    clickAt(tools, vp, [1, 1, 0]);
    const q = onlyPattern(app.model);
    expect(q.id).not.toBe(p.id);
    expect(vertexCount(q)).toBe(1);
    expect(app.status).toBe(STATUS.patternVertexPlaced(q.name));
  });
});

// ---------------------------------------------------------------------------
// The polyline and the model it was drawn on: undo, New, hiding, double-clicks, deletion feedback
// ---------------------------------------------------------------------------

describe('Crease pattern tool: undo, replaced models, hiding and double-clicks', () => {
  /** What App.undo does: the previous snapshot becomes a fresh model object, the selection is cleared. */
  const undoLikeApp = (app: FakeApp): void => {
    const s = app.undo.pop()!;
    app.model = parseModel(s);
    app.selection = null;
  };

  it('Ctrl+Z mid-polyline ends it: Backspace restores nothing stale, and the next click starts a new polyline with its status', () => {
    const { app, vp, tools } = fakeApp(createModel());
    tools.setTool('cpattern');
    clickAt(tools, vp, [0, 0, 0]);
    clickAt(tools, vp, [2, 0, 0]);
    clickAt(tools, vp, [2, 2, 0]);
    const p = onlyPattern(app.model);
    expect(vertexCount(p)).toBe(3);
    expect(creaseCount(p)).toBe(2);
    undoLikeApp(app);
    undoLikeApp(app);
    const after = onlyPattern(app.model);
    expect(vertexCount(after)).toBe(1);
    expect(creaseCount(after)).toBe(0);
    expect(app.undo.length).toBe(1);
    const stateAfterUndo = serializeModel(app.model);
    // Backspace: the polyline died with its model; nothing comes back
    expect(tools.popSketchSupported()).toBe(false);
    expect(tools.popSketchPoint()).toBe(false);
    expect(serializeModel(app.model)).toBe(stateAfterUndo);
    expect(app.undo.length).toBe(1);
    expect(app.overlay).toEqual({});
    // the next click starts a new polyline (a vertex placed, no crease to the undone vertex whose id is now reused)
    clickAt(tools, vp, [4, 4, 0]);
    const q = onlyPattern(app.model);
    expect(vertexCount(q)).toBe(2);
    expect(creaseCount(q)).toBe(0);
    expect(app.status).toBe(STATUS.patternVertexPlaced(q.name));
    expect(app.undo.length).toBe(2);
    clickAt(tools, vp, [4, 0, 0]);
    expect(creaseCount(q)).toBe(1);
    expect(Object.values(q.creases).every((c) => [q.vertices[c.a].pos, q.vertices[c.b].pos].every((pos) => pos[0] === 4))).toBe(true);
    expect(tools.popSketchSupported()).toBe(true);
    // a single undo after two clicks: the undone vertex's id is reused by the next vertex, which is still a fresh start
    const { app: app2, vp: vp2, tools: tools2 } = fakeApp(createModel());
    tools2.setTool('cpattern');
    clickAt(tools2, vp2, [0, 0, 0]);
    clickAt(tools2, vp2, [3, 0, 0]);
    undoLikeApp(app2);
    clickAt(tools2, vp2, [3, 3, 0]);
    const p2 = onlyPattern(app2.model);
    expect(vertexCount(p2)).toBe(2);
    expect(creaseCount(p2)).toBe(0);
    expect(app2.status).toBe(STATUS.patternVertexPlaced(p2.name));
    clickAt(tools2, vp2, [0, 3, 0]);
    expect(creaseCount(p2)).toBe(1);
    expect(app2.overlay.rubberBand && dist(app2.overlay.rubberBand.a, [0, 3, 0])).toBeLessThan(1e-9);
  });

  it('File > New (the model replaced) while drawing: Backspace brings nothing back; the polyline helper itself forgets a vanished anchor', () => {
    const { app, vp, tools } = fakeApp(createModel());
    tools.setTool('cpattern');
    clickAt(tools, vp, [0, 0, 0]);
    clickAt(tools, vp, [2, 0, 0]);
    app.model = createModel();
    app.undo = [];
    app.selection = null;
    const fresh = serializeModel(app.model);
    expect(tools.popSketchSupported()).toBe(false);
    expect(tools.popSketchPoint()).toBe(false);
    expect(serializeModel(app.model)).toBe(fresh);
    expect(Object.keys(app.model.patterns ?? {}).length).toBe(0);
    clickAt(tools, vp, [1, 1, 0]);
    const q = onlyPattern(app.model);
    expect(vertexCount(q)).toBe(1);
    expect(app.status).toBe(STATUS.patternVertexPlaced(q.name));
    // PatternPolyline.addPoint on its own: a last vertex that no longer exists starts a new polyline
    const m = createModel();
    const p = addPattern(m, 'plane_top');
    const pl = new PatternPolyline();
    pl.begin(p.id);
    pl.addPoint(m, p, [0, 0, 0], 'U');
    const b = pl.addPoint(m, p, [2, 0, 0], 'U').vertex.vertex.id;
    delete p.vertices[b];
    delete p.creases[Object.keys(p.creases)[0]];
    m.nextId -= 2; // as an undo would: the ids are handed out again
    const r = pl.addPoint(m, p, [2, 2, 0], 'U');
    expect(r.same).toBe(false);
    expect(r.crease).toBeNull();
    expect(r.vertex.vertex.id).toBe(b);
    expect(Object.keys(p.creases).length).toBe(0);
  });

  it('hiding the pattern mid-polyline (tree eye, Properties): the next click shows it again; a quiet end leaves the status alone', () => {
    const { app, vp, tools } = fakeApp(createModel());
    tools.setTool('cpattern');
    clickAt(tools, vp, [0, 0, 0]);
    clickAt(tools, vp, [2, 0, 0]);
    const p = onlyPattern(app.model);
    p.hidden = true;
    clickAt(tools, vp, [2, 2, 0]);
    expect(p.hidden).toBe(false);
    expect(creaseCount(p)).toBe(2);
    expect(Object.keys(app.model.patterns!).length).toBe(1);
    const status = app.status;
    tools.endPolyline(true);
    expect(app.status).toBe(status);
    expect(app.overlay).toEqual({});
    expect(tools.popSketchSupported()).toBe(false);
  });

  it('a double-click as the first click leaves the polyline open at that vertex instead of an isolated vertex', () => {
    const { app, vp, tools } = fakeApp(createModel());
    tools.setTool('cpattern');
    const s = vp.worldToScreen([1, 1, 0]);
    tools.handle(ev('move', s.x, s.y, null));
    tools.handle(ev('down', s.x, s.y, null));
    tools.handle(ev('up', s.x, s.y, null));
    tools.handle(ev('down', s.x + 1, s.y, null));
    tools.handle(ev('up', s.x + 1, s.y, null));
    tools.handle(ev('dblclick', s.x + 1, s.y, null));
    const p = onlyPattern(app.model);
    expect(vertexCount(p)).toBe(1);
    expect(creaseCount(p)).toBe(0);
    expect(app.status).toBe(STATUS.patternVertexPlaced(p.name));
    expect(tools.popSketchSupported()).toBe(true);
    const t = vp.worldToScreen([3, 1, 0]);
    tools.handle(ev('move', t.x, t.y, null));
    expect(app.overlay.rubberBand && dist(app.overlay.rubberBand.a, [1, 1, 0])).toBeLessThan(1e-9);
    clickAt(tools, vp, [3, 1, 0]);
    expect(creaseCount(p)).toBe(1);
    // with two vertices the double-click ends the polyline as before
    tools.handle(ev('down', t.x + 1, t.y, null));
    tools.handle(ev('dblclick', t.x + 1, t.y, null));
    expect(app.status).toBe(STATUS.patternPolylineEnded);
    expect(tools.popSketchSupported()).toBe(false);
  });

  it('deleting pattern elements reports what went: a pattern with the panels it generated', () => {
    const { app, vp, tools } = fakeApp(createModel());
    tools.setTool('cpattern');
    app.toolOptions.creaseClass = 'B';
    for (const q of [[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0], [0, 0, 0]] as Vec3[]) clickAt(tools, vp, q);
    tools.finish();
    const p = onlyPattern(app.model);
    const vid = vertexAt(p, [2, 2, 0])!;
    const c = Object.values(p.creases)[0];
    tools.deletePick({ type: 'pcrease', id: p.id, creaseId: c.id });
    expect(app.status).toBe(STATUS.patternCreaseDeleted);
    tools.deletePick({ type: 'pvertex', id: p.id, pointId: vid });
    expect(app.status).toBe(STATUS.patternVertexDeleted);
    expect(creaseCount(p)).toBe(1);
    tools.deletePick({ type: 'pattern', id: p.id });
    expect(app.status).toBe(STATUS.patternDeleted(p.name, 0, 0));
    expect(Object.keys(app.model.patterns ?? {}).length).toBe(0);
    // a generated pattern: the counts of the panels and creases removed with it
    const m = createModel();
    const q = addPattern(m, 'plane_top');
    const ids = [[0, 0, 0], [2, 0, 0], [2, 2, 0], [0, 2, 0]].map((pos) => addPatternVertex(m, q, pos as Vec3).vertex.id);
    for (let i = 0; i < 4; i++) addPatternCrease(m, q, ids[i], ids[(i + 1) % 4], 'B');
    addPatternCrease(m, q, ids[0], ids[2], 'V');
    generatePanels(m, q);
    const { app: app2, tools: tools2 } = fakeApp(m);
    tools2.deletePick({ type: 'pattern', id: q.id });
    expect(app2.status).toBe(STATUS.patternDeleted(q.name, 2, 1));
    expect(Object.keys(app2.model.links).length).toBe(0);
    expect(Object.keys(app2.model.joints).length).toBe(0);
  });
});
