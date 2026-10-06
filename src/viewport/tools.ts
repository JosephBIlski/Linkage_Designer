/**
 * Interactive tools: selection & dragging, link / polygon / prism / cylinder
 * creation, construction geometry, joints, ground, drivers, deletion and
 * editing-point dragging for inverse design.
 */
import type { App, ToolName } from '../app';
import { add, dist, dot, len, normalize, scale, sub, cross } from '../core/geometry';
import { commitSketch, solveSketch, syncDriverValues } from '../core/kinematics';
import {
  addAngleDriver,
  addBar,
  addConstructionAxis,
  addConstructionPlane3,
  addConstructionPoint,
  addCylinder,
  addFoldDriver,
  addJoint,
  addOffsetPlane,
  addPolygon,
  addPrism,
  addSlideDriver,
  candidateAngleDrivers,
  jointCompatible,
  jointsAtPoint,
  linkAllPointIds,
  removeConstruction,
  removeJoint,
  removeLink,
  setGround,
  sketchNormal,
} from '../core/model';
import type { ConstructionRef, Feature, ID, Target, Vec3 } from '../core/types';
import { moveVertex } from '../core/edit';
import { duplicateLink, linearArray, mirrorAcrossPlane, polarArray } from '../core/patterns';
import { addPolygonFromPoints } from '../core/model';
import { isConstructionRef } from '../core/types';
import { FEATURES, STATUS, TOOLS, CONSTRUCTION_NAMES } from '../ui/strings';
import type { PickResult, ViewportPointerEvent } from './scene';

export function toolHint(tool: ToolName): string {
  const t = (TOOLS as Record<string, unknown>)[tool];
  return typeof t === 'object' && t !== null && 'hint' in t ? String((t as { hint: string }).hint) : '';
}

export type TypedPoint = { kind: 'abs'; p: Vec3 } | { kind: 'rel'; d: Vec3 } | { kind: 'polar'; L: number; angleDeg: number } | { kind: 'length'; L: number };

/** Parse coordinate input: "x,y[,z]" | "@dx,dy[,dz]" | "L<angle" | "L". */
export function parseTypedPoint(text: string): TypedPoint | null {
  const t = text.trim().replace(/\s+/g, '');
  if (!t) return null;
  const num = '[-+]?(?:\\d+\\.?\\d*|\\.\\d+)(?:e[-+]?\\d+)?';
  let m = t.match(new RegExp(`^@(${num}),(${num})(?:,(${num}))?$`, 'i'));
  if (m) return { kind: 'rel', d: [Number(m[1]), Number(m[2]), Number(m[3] ?? 0)] };
  m = t.match(new RegExp(`^(${num})<(${num})$`, 'i'));
  if (m) return { kind: 'polar', L: Number(m[1]), angleDeg: Number(m[2]) };
  m = t.match(new RegExp(`^(${num}),(${num})(?:,(${num}))?$`, 'i'));
  if (m) return { kind: 'abs', p: [Number(m[1]), Number(m[2]), Number(m[3] ?? 0)] };
  m = t.match(new RegExp(`^(${num})$`, 'i'));
  if (m) return { kind: 'length', L: Number(m[1]) };
  return null;
}

interface PlacedPoint {
  pos: Vec3;
  snappedPointId?: ID;
  snappedConstructionId?: ID;
}

type DragState =
  | { kind: 'none' }
  | { kind: 'pending'; pick: PickResult; startX: number; startY: number }
  | { kind: 'point'; pointId: ID; linkId: ID; plane: { o: Vec3; n: Vec3 }; moved: boolean }
  | { kind: 'link'; linkId: ID; grab: Vec3; start: Map<ID, Vec3>; plane: { o: Vec3; n: Vec3 }; moved: boolean }
  | { kind: 'editPoint'; pointId: ID; pose: number; target: Target; created: boolean; mode: 'plane' | 'line' | 'free' | 'none'; dirs: Vec3[]; origin: Vec3; moved: boolean };

export class ToolManager {
  private points: PlacedPoint[] = [];
  private featureA: Feature | null = null;
  /** Edit / mirror / pattern tools: the vertex or link picked first. */
  private editSource: { pointId: ID; linkId: ID } | null = null;
  private patternLinkId: ID | null = null;
  private pendingLength: number | null = null;
  private drag: DragState = { kind: 'none' };
  private lastPointer: { x: number; y: number } | null = null;
  onPopup: ((pointId: ID | null) => void) | null = null;

  constructor(private app: App) {
    app.viewport.onPointer = (ev) => this.handle(ev);
  }

  reset(): void {
    this.points = [];
    this.featureA = null;
    this.editSource = null;
    this.patternLinkId = null;
    this.pendingLength = null;
    this.drag = { kind: 'none' };
    this.app.setOverlay({});
    this.app.setHint(toolHint(this.app.tool));
  }

  setTool(tool: ToolName): void {
    this.app.setTool(tool);
    this.reset();
  }

  cancel(): void {
    if (this.drag.kind !== 'none' && this.drag.kind !== 'pending') {
      // abort drag: restore from undo snapshot
      this.app.undo();
    }
    if (this.points.length === 0 && !this.featureA && !this.editSource && !this.patternLinkId && this.drag.kind === 'none') this.app.setTool('select');
    this.reset();
  }

  // ---------------------------------------------------------------------------
  // Point placement helpers
  // ---------------------------------------------------------------------------

  private sketchPlane(): { o: Vec3; n: Vec3 } {
    const c = this.app.model.construction[this.app.model.settings.sketchPlaneId];
    return c?.dir ? { o: c.origin, n: c.dir } : { o: [0, 0, 0], n: [0, 0, 1] };
  }

  private gridSnap(p: Vec3): Vec3 {
    const s = this.app.settings;
    if (!s.gridSnap) return p;
    const step = s.gridStep || 0.5;
    return [Math.round(p[0] / step) * step, Math.round(p[1] / step) * step, Math.round(p[2] / step) * step];
  }

  /** Where would a click at this pointer position place a point? */
  private place(ev: { clientX: number; clientY: number; pick: PickResult | null }, allowSnap = true): PlacedPoint | null {
    const m = this.app.model;
    if (allowSnap && ev.pick) {
      if (ev.pick.type === 'vertex' && ev.pick.pointId && m.points[ev.pick.pointId]) return { pos: [...m.points[ev.pick.pointId].pos] as Vec3, snappedPointId: ev.pick.pointId };
      if (ev.pick.type === 'construction') {
        const c = m.construction[ev.pick.id];
        if (c?.kind === 'point') return { pos: [...c.origin] as Vec3, snappedConstructionId: c.id };
      }
    }
    const vp = this.app.viewport;
    let pos: Vec3 | null = null;
    const last = this.points[this.points.length - 1]?.pos;
    if (this.app.toolOptions.mode2d || this.app.tool === 'cpoint' || this.app.tool === 'caxis' || this.app.tool === 'cplane' || this.app.tool === 'sketch') {
      const pl = this.sketchPlane();
      pos = vp.projectToPlane(ev.clientX, ev.clientY, pl.o, pl.n);
    }
    if (!pos) pos = vp.projectToViewPlane(ev.clientX, ev.clientY, last ?? [0, 0, 0]);
    if (!pos) return null;
    pos = this.gridSnap(pos);
    if (this.pendingLength !== null && last) {
      const d = sub(pos, last);
      const L = len(d);
      if (L > 1e-9) pos = add(last, scale(d, this.pendingLength / L));
    }
    return { pos };
  }

  private featureFromPick(pick: PickResult): Feature | ConstructionRef | null {
    const m = this.app.model;
    switch (pick.type) {
      case 'vertex':
        return pick.pointId && pick.linkId ? { linkId: pick.linkId, kind: 'vertex', pointIds: [pick.pointId] } : null;
      case 'edge':
        return pick.linkId && pick.pointIds ? { linkId: pick.linkId, kind: 'edge', pointIds: pick.pointIds } : null;
      case 'axis':
        return pick.linkId && pick.pointIds ? { linkId: pick.linkId, kind: 'axis', pointIds: pick.pointIds } : null;
      case 'face':
        return pick.linkId && pick.pointIds ? { linkId: pick.linkId, kind: 'face', pointIds: pick.pointIds } : null;
      case 'construction':
        return m.construction[pick.id] ? { constructionId: pick.id } : null;
      default:
        return null;
    }
  }

  describeFeature(f: Feature | ConstructionRef): string {
    const m = this.app.model;
    if (isConstructionRef(f)) return `${m.construction[f.constructionId]?.name ?? f.constructionId} (${FEATURES.construction})`;
    const link = m.links[f.linkId];
    const names = f.pointIds.map((id) => m.points[id]?.name ?? id).join('-');
    return `${FEATURES[f.kind]} ${names} ${FEATURES.of} ${link?.name ?? f.linkId}`;
  }

  // ---------------------------------------------------------------------------
  // Event dispatch
  // ---------------------------------------------------------------------------

  handle(ev: ViewportPointerEvent): void {
    const app = this.app;
    this.lastPointer = { x: ev.clientX, y: ev.clientY };
    if (ev.kind === 'leave') {
      app.setHover(null);
      return;
    }
    if (ev.kind === 'move') {
      if (this.drag.kind !== 'none') return this.dragMove(ev);
      app.setHover(ev.pick);
      this.updatePreview(ev);
      if (ev.pick) app.setStatus(this.describePick(ev.pick));
      return;
    }
    if (ev.kind === 'up') {
      if (this.drag.kind !== 'none') this.dragEnd(ev);
      return;
    }
    if (ev.kind === 'dblclick') {
      if (app.tool === 'sketch') this.finishSketch();
      return;
    }
    if (ev.kind !== 'down' || ev.button !== 0) return;
    this.onPopup?.(null);
    switch (app.tool) {
      case 'select':
        return this.selectDown(ev);
      case 'sketch':
        return this.sketchTool(ev);
      case 'edit':
        return this.editTool(ev);
      case 'mirror':
        return this.mirrorTool(ev);
      case 'pattern':
        return this.patternTool(ev);
      case 'bar':
      case 'caxis':
        return this.twoPointTool(ev);
      case 'polygon':
      case 'prism':
        return this.polygonTool(ev);
      case 'cylinder':
        return this.cylinderTool(ev);
      case 'cpoint':
        return this.pointTool(ev);
      case 'cplane':
        return this.planeTool(ev);
      case 'joint':
        return this.jointTool(ev);
      case 'ground':
        return this.groundTool(ev);
      case 'driver':
        return this.driverTool(ev);
      case 'delete':
        return this.deleteTool(ev);
    }
  }

  private describePick(p: PickResult): string {
    const m = this.app.model;
    if (p.type === 'editPoint') return `${STATUS.pose} ${p.pose} · ${m.points[p.pointId!]?.name ?? ''}`;
    if (p.type === 'joint') return m.joints[p.id]?.type ?? '';
    if (p.type === 'construction') return m.construction[p.id]?.name ?? '';
    const f = this.featureFromPick(p);
    return f ? this.describeFeature(f) : '';
  }

  /** Called when the coordinate box is submitted. */
  applyTyped(tp: TypedPoint): void {
    const app = this.app;
    const tool = app.tool;
    if (tp.kind === 'length') {
      this.pendingLength = tp.L;
      app.setStatus(`${STATUS.coordHelp} (L = ${tp.L})`);
      return;
    }
    const last = this.points[this.points.length - 1]?.pos ?? [0, 0, 0];
    let pos: Vec3;
    if (tp.kind === 'abs') pos = tp.p;
    else if (tp.kind === 'rel') pos = add(last, tp.d);
    else {
      const pl = this.sketchPlane();
      const n = normalize(pl.n);
      const u = normalize(sub([1, 0, 0], scale(n, dot([1, 0, 0], n)))) ;
      const uu = len(u) > 0.5 ? u : normalize(cross(n, [0, 1, 0]));
      const v = cross(n, uu);
      const a = (tp.angleDeg * Math.PI) / 180;
      pos = add(last, add(scale(uu, tp.L * Math.cos(a)), scale(v, tp.L * Math.sin(a))));
    }
    const fake = { kind: 'down' as const, button: 0, clientX: 0, clientY: 0, shiftKey: false, ctrlKey: false, altKey: false, pick: null, original: new MouseEvent('click') };
    switch (tool) {
      case 'bar':
      case 'caxis':
        return this.twoPointTool(fake, { pos });
      case 'polygon':
      case 'prism':
        return this.polygonTool(fake, { pos });
      case 'cylinder':
        return this.cylinderTool(fake, { pos });
      case 'cpoint':
        return this.pointTool(fake, { pos });
      case 'cplane':
        return this.planeTool(fake, { pos });
      case 'sketch':
        return this.sketchTool(fake, { pos });
      case 'edit':
        return this.editTool(fake, { pos });
      case 'pattern':
        return this.patternTool(fake, { pos });
      default:
        app.setStatus(STATUS.coordHelp);
    }
  }

  // ---------------------------------------------------------------------------
  // Previews
  // ---------------------------------------------------------------------------

  private updatePreview(ev: ViewportPointerEvent): void {
    const app = this.app;
    const tool = app.tool;
    if (tool === 'select' || tool === 'ground' || tool === 'driver' || tool === 'delete' || tool === 'joint' || tool === 'mirror' || (tool === 'edit' && !this.editSource) || (tool === 'pattern' && (!this.patternLinkId || this.app.toolOptions.patternKind === 'polar'))) {
      app.setOverlay({});
      return;
    }
    const placed = this.place(ev);
    if (!placed) return;
    const overlay: import('./render').OverlayView = { snapPoint: placed.snappedPointId || placed.snappedConstructionId ? placed.pos : null, marker: placed.pos };
    const first = this.points[0]?.pos;
    if (first && (tool === 'bar' || tool === 'caxis' || tool === 'cylinder')) overlay.rubberBand = { a: first, b: placed.pos };
    if (first && (tool === 'polygon' || tool === 'prism')) {
      const r = dist(first, placed.pos);
      overlay.circle = { center: first, normal: this.sketchPlane().n, radius: r };
      overlay.rubberBand = { a: first, b: placed.pos };
    }
    if (tool === 'cplane' && this.points.length > 0) overlay.polyline = [...this.points.map((p) => p.pos), placed.pos];
    if (tool === 'sketch' && this.points.length > 0) {
      overlay.polyline = [...this.points.map((p) => p.pos), placed.pos];
      overlay.rubberBand = { a: this.points[this.points.length - 1].pos, b: placed.pos };
    }
    if (tool === 'pattern' && this.patternLinkId && this.points.length === 1 && app.toolOptions.patternKind === 'linear') overlay.rubberBand = { a: this.points[0].pos, b: placed.pos };
    if (tool === 'edit' && this.editSource) overlay.rubberBand = { a: app.model.points[this.editSource.pointId]?.pos ?? placed.pos, b: placed.pos };
    app.setOverlay(overlay);
  }

  // ---------------------------------------------------------------------------
  // Sketch (free polygon) tool
  // ---------------------------------------------------------------------------

  private sketchTool(ev: ViewportPointerEvent, forced?: PlacedPoint): void {
    const app = this.app;
    const placed = forced ?? this.place(ev, false);
    if (!placed) return;
    // clicking near the first vertex closes the polygon
    if (this.points.length >= 3 && !forced) {
      const s0 = app.viewport.worldToScreen(this.points[0].pos);
      if (Math.hypot(s0.x - ev.clientX, s0.y - ev.clientY) < 12) {
        this.finishSketch();
        return;
      }
    }
    // project onto the sketch plane so the polygon is planar
    const pl = this.sketchPlane();
    const pos = sub(placed.pos, scale(pl.n, dot(sub(placed.pos, pl.o), pl.n)));
    if (dist(pos, placed.pos) > 1e-6) app.setStatus(STATUS.sketchNotPlanar);
    const last = this.points[this.points.length - 1];
    if (last && dist(last.pos, pos) < 1e-9) return;
    this.points.push({ pos });
    app.setHint(TOOLS.sketch.hint);
  }

  /** Close the sketched polygon (Enter / double-click / click on the first vertex). */
  finishSketch(): void {
    const app = this.app;
    if (app.tool !== 'sketch') return;
    if (this.points.length < 3) {
      app.setStatus(STATUS.sketchNeedsThree);
      return;
    }
    const m = app.model;
    app.beginChange();
    const link = addPolygonFromPoints(m, this.points.map((p) => p.pos), { name: app.nextLinkName('polygon'), onPlaneId: m.settings.sketchPlaneId });
    app.select({ type: 'link', id: link.id });
    app.endChange();
    this.points = [];
    app.setOverlay({});
  }

  // ---------------------------------------------------------------------------
  // Edit (move / snap vertex) tool
  // ---------------------------------------------------------------------------

  private editTool(ev: ViewportPointerEvent, forced?: PlacedPoint): void {
    const app = this.app;
    const m = app.model;
    if (!this.editSource) {
      const pick = ev.pick;
      if (pick?.type === 'vertex' && pick.pointId && pick.linkId) {
        if (m.links[pick.linkId].locked) {
          app.setStatus(STATUS.linkLocked);
          return;
        }
        this.editSource = { pointId: pick.pointId, linkId: pick.linkId };
        app.select({ type: 'vertex', id: pick.linkId, pointId: pick.pointId });
        app.setStatus(STATUS.editPickTarget);
      } else app.setStatus(TOOLS.edit.hint);
      return;
    }
    const placed = forced ?? this.place(ev);
    if (!placed) return;
    const joinTo = placed.snappedPointId && placed.snappedPointId !== this.editSource.pointId && m.points[placed.snappedPointId]?.linkId !== this.editSource.linkId ? placed.snappedPointId : undefined;
    app.beginChange();
    const res = moveVertex(m, this.editSource.pointId, placed.pos, joinTo);
    app.endChange();
    app.setStatus(res.ok ? STATUS.editDone + (res.joinedTo ? ` · ${STATUS.jointCreated}` : '') : STATUS.violated);
    app.select({ type: 'vertex', id: this.editSource.linkId, pointId: this.editSource.pointId });
    this.editSource = null;
    app.setOverlay({});
  }

  // ---------------------------------------------------------------------------
  // Mirror & pattern tools
  // ---------------------------------------------------------------------------

  private mirrorTool(ev: ViewportPointerEvent): void {
    const app = this.app;
    const m = app.model;
    const pick = ev.pick;
    if (!this.patternLinkId) {
      if (pick?.linkId && m.links[pick.linkId]) {
        this.patternLinkId = pick.linkId;
        app.select({ type: 'link', id: pick.linkId });
        app.setStatus(STATUS.mirrorPickPlane);
      } else app.setStatus(TOOLS.mirror.hint);
      return;
    }
    if (pick?.type === 'construction') {
      const c = m.construction[pick.id];
      if (c?.kind === 'plane' && c.dir) {
        const src = m.links[this.patternLinkId];
        app.beginChange();
        const copy = duplicateLink(m, src, mirrorAcrossPlane(c.origin, c.dir), `${src.name} mirror`);
        app.endChange();
        if (copy) app.select({ type: 'link', id: copy.id });
        this.patternLinkId = null;
        return;
      }
    }
    app.setStatus(STATUS.mirrorPickPlane);
  }

  private patternTool(ev: ViewportPointerEvent, forced?: PlacedPoint): void {
    const app = this.app;
    const m = app.model;
    const o = app.toolOptions;
    const pick = ev.pick;
    if (!this.patternLinkId) {
      if (pick?.linkId && m.links[pick.linkId]) {
        this.patternLinkId = pick.linkId;
        app.select({ type: 'link', id: pick.linkId });
        app.setStatus(STATUS.patternPickFirst);
      } else app.setStatus(TOOLS.pattern.hint);
      return;
    }
    const src = m.links[this.patternLinkId];
    const count = Math.max(1, Math.round(o.patternCount));
    if (o.patternKind === 'polar') {
      let origin: Vec3 | null = null;
      let axis: Vec3 = sketchNormal(m);
      if (pick?.type === 'construction') {
        const c = m.construction[pick.id];
        if (c?.kind === 'axis' && c.dir) {
          origin = c.origin;
          axis = c.dir;
        } else if (c?.kind === 'point') origin = c.origin;
      }
      if (!origin) {
        const placed = forced ?? this.place(ev);
        if (!placed) return;
        origin = placed.pos;
      }
      app.beginChange();
      const copies = polarArray(m, src, origin, axis, count, (i) => `${src.name} ${i + 1}`, (o.patternAngle * Math.PI) / 180);
      app.endChange();
      if (copies.length) app.select({ type: 'link', id: copies[copies.length - 1].id });
      app.setStatus(STATUS.patternDone);
      this.patternLinkId = null;
      app.setOverlay({});
      return;
    }
    // linear: two points define the spacing vector
    const placed = forced ?? this.place(ev);
    if (!placed) return;
    if (this.points.length === 0) {
      this.points.push(placed);
      app.setStatus(STATUS.patternPickSecond);
      return;
    }
    const step = sub(placed.pos, this.points[0].pos);
    if (len(step) < 1e-9) return;
    app.beginChange();
    const copies = linearArray(m, src, step, count, (i) => `${src.name} ${i + 1}`);
    app.endChange();
    if (copies.length) app.select({ type: 'link', id: copies[copies.length - 1].id });
    app.setStatus(STATUS.patternDone);
    this.patternLinkId = null;
    this.points = [];
    app.setOverlay({});
  }

  // ---------------------------------------------------------------------------
  // Creation tools
  // ---------------------------------------------------------------------------

  private twoPointTool(ev: ViewportPointerEvent, forced?: PlacedPoint): void {
    const app = this.app;
    const placed = forced ?? this.place(ev);
    if (!placed) return;
    if (this.points.length === 0) {
      this.points.push(placed);
      app.setHint(TOOLS[app.tool as 'bar' | 'caxis'].hint);
      return;
    }
    const a = this.points[0];
    const b = placed;
    if (dist(a.pos, b.pos) < 1e-6) return;
    app.beginChange();
    if (app.tool === 'caxis') {
      addConstructionAxis(app.model, a.pos, sub(b.pos, a.pos), `${CONSTRUCTION_NAMES.axis}${app.model.nextId}`, Math.max(dist(a.pos, b.pos), 5));
    } else {
      const m = app.model;
      const link = addBar(m, a.pos, b.pos, { name: app.nextLinkName('bar'), onPlaneId: app.toolOptions.mode2d ? m.settings.sketchPlaneId : null });
      const joinTo = (newPid: ID, snap: PlacedPoint) => {
        if (snap.snappedPointId && m.points[snap.snappedPointId]) {
          const other = m.points[snap.snappedPointId];
          if (other.linkId !== link.id) addJoint(m, m.settings.defaultJoint, { linkId: link.id, kind: 'vertex', pointIds: [newPid] }, { linkId: other.linkId, kind: 'vertex', pointIds: [other.id] }, { axis: sketchNormal(m) });
        } else if (snap.snappedConstructionId) {
          addJoint(m, m.settings.defaultJoint === 'spherical' ? 'spherical' : 'revolute', { linkId: link.id, kind: 'vertex', pointIds: [newPid] }, { constructionId: snap.snappedConstructionId }, { axis: sketchNormal(m) });
        }
      };
      joinTo(link.pointIds[0], a);
      joinTo(link.pointIds[1], b);
      app.select({ type: 'link', id: link.id });
    }
    app.endChange();
    this.points = [];
    this.pendingLength = null;
    app.setOverlay({});
  }

  private polygonTool(ev: ViewportPointerEvent, forced?: PlacedPoint): void {
    const app = this.app;
    const placed = forced ?? this.place(ev);
    if (!placed) return;
    if (this.points.length === 0) {
      this.points.push(placed);
      return;
    }
    const c = this.points[0].pos;
    const r = dist(c, placed.pos);
    if (r < 1e-6) return;
    const m = app.model;
    const n = this.sketchPlane().n;
    const xDir = sub(placed.pos, c);
    app.beginChange();
    const opts = app.toolOptions;
    const link =
      app.tool === 'prism'
        ? addPrism(m, c, n, r, Math.max(3, opts.sides), opts.height, { name: app.nextLinkName('prism'), xDir })
        : addPolygon(m, c, n, r, Math.max(3, opts.sides), { name: app.nextLinkName('polygon'), xDir, onPlaneId: opts.mode2d ? m.settings.sketchPlaneId : null });
    // join first vertex to a snapped vertex if any
    if (placed.snappedPointId && m.points[placed.snappedPointId] && m.points[placed.snappedPointId].linkId !== link.id) {
      const other = m.points[placed.snappedPointId];
      addJoint(m, m.settings.defaultJoint, { linkId: link.id, kind: 'vertex', pointIds: [link.pointIds[0]] }, { linkId: other.linkId, kind: 'vertex', pointIds: [other.id] }, { axis: n });
    }
    app.select({ type: 'link', id: link.id });
    app.endChange();
    this.points = [];
    app.setOverlay({});
  }

  private cylinderTool(ev: ViewportPointerEvent, forced?: PlacedPoint): void {
    const app = this.app;
    const placed = forced ?? this.place(ev);
    if (!placed) return;
    if (this.points.length === 0) {
      this.points.push(placed);
      return;
    }
    const a = this.points[0].pos;
    if (dist(a, placed.pos) < 1e-6) return;
    app.beginChange();
    const link = addCylinder(app.model, a, placed.pos, app.toolOptions.radius, { name: app.nextLinkName('cylinder') });
    app.select({ type: 'link', id: link.id });
    app.endChange();
    this.points = [];
    app.setOverlay({});
  }

  private pointTool(ev: ViewportPointerEvent, forced?: PlacedPoint): void {
    const app = this.app;
    const placed = forced ?? this.place(ev, false);
    if (!placed) return;
    app.beginChange();
    const c = addConstructionPoint(app.model, placed.pos, `${CONSTRUCTION_NAMES.point}${app.model.nextId}`);
    app.select({ type: 'construction', id: c.id });
    app.endChange();
  }

  private planeTool(ev: ViewportPointerEvent, forced?: PlacedPoint): void {
    const app = this.app;
    const m = app.model;
    if (app.toolOptions.planeMode === 'offset') {
      const pick = ev.pick;
      if (pick?.type === 'construction' && m.construction[pick.id]?.kind === 'plane') {
        app.beginChange();
        const c = addOffsetPlane(m, pick.id, app.toolOptions.offset, `${CONSTRUCTION_NAMES.plane}${m.nextId}`);
        if (c) app.select({ type: 'construction', id: c.id });
        app.endChange();
      } else app.setStatus(TOOLS.cplane.hint);
      return;
    }
    const placed = forced ?? this.place(ev);
    if (!placed) return;
    this.points.push(placed);
    if (this.points.length < 3) return;
    app.beginChange();
    const c = addConstructionPlane3(m, this.points[0].pos, this.points[1].pos, this.points[2].pos, `${CONSTRUCTION_NAMES.plane}${m.nextId}`);
    if (c) app.select({ type: 'construction', id: c.id });
    app.endChange();
    this.points = [];
    app.setOverlay({});
  }

  // ---------------------------------------------------------------------------
  // Joint / ground / driver / delete
  // ---------------------------------------------------------------------------

  private jointTool(ev: ViewportPointerEvent): void {
    const app = this.app;
    const m = app.model;
    if (!ev.pick) return;
    const f = this.featureFromPick(ev.pick);
    if (!f) return;
    if (!this.featureA) {
      if (isConstructionRef(f)) {
        app.setStatus(TOOLS.joint.hint);
        return;
      }
      this.featureA = f;
      app.setStatus(`${this.describeFeature(f)} — ${STATUS.pickSecondFeature}`);
      return;
    }
    if (!isConstructionRef(f) && f.linkId === this.featureA.linkId) {
      app.setStatus(STATUS.jointSameLink);
      return;
    }
    const type = app.toolOptions.jointType;
    if (!jointCompatible(m, type, this.featureA, f)) {
      app.setStatus(STATUS.jointIncompatible);
      return;
    }
    app.beginChange();
    const j = addJoint(m, type, this.featureA, f, { axis: sketchNormal(m), pitch: app.toolOptions.pitch });
    if (j) {
      // snap geometry so the new joint is satisfied
      const res = solveSketch(m, {});
      commitSketch(m, res);
      app.select({ type: 'joint', id: j.id });
      app.setStatus(STATUS.jointCreated);
    }
    app.endChange();
    this.featureA = null;
  }

  private groundTool(ev: ViewportPointerEvent): void {
    const app = this.app;
    const linkId = ev.pick?.linkId;
    if (!linkId) return;
    app.beginChange();
    setGround(app.model, app.model.links[linkId].ground ? null : linkId);
    app.endChange();
    app.setStatus(STATUS.groundSet);
    app.select({ type: 'link', id: linkId });
  }

  private driverTool(ev: ViewportPointerEvent): void {
    const app = this.app;
    const m = app.model;
    const pick = ev.pick;
    if (!pick) return;
    app.beginChange();
    if (pick.type === 'joint') {
      const j = m.joints[pick.id];
      if (j?.type === 'revolute') addFoldDriver(m, j.id);
      else if (j && (j.type === 'prismatic' || j.type === 'cylindrical' || j.type === 'screw')) addSlideDriver(m, j.id);
    } else if (pick.linkId) {
      const cands = candidateAngleDrivers(m).filter((c) => c.linkId === pick.linkId);
      const c = cands.find((x) => x.pivotId === pick.pointId) ?? cands[0];
      if (c) addAngleDriver(m, c.linkId, c.pivotId, c.tipId, c.axis);
      else app.setStatus(TOOLS.driver.hint);
    }
    syncDriverValues(m);
    app.sim.activeDriver = Math.max(0, m.drivers.length - 1);
    app.endChange();
    app.setStatus(STATUS.driverSet);
  }

  private deleteTool(ev: ViewportPointerEvent): void {
    if (!ev.pick) return;
    this.deletePick(ev.pick);
  }

  /** Delete the entity behind a pick / selection. */
  deletePick(pick: { type: string; id: ID; pointId?: ID; pose?: number }): void {
    const app = this.app;
    const m = app.model;
    app.beginChange();
    switch (pick.type) {
      case 'joint':
        removeJoint(m, pick.id);
        break;
      case 'construction':
        removeConstruction(m, pick.id);
        break;
      case 'editPoint': {
        if (pick.pointId !== undefined && pick.pose !== undefined) m.targets = m.targets.filter((t) => !(t.pointId === pick.pointId && t.pose === pick.pose));
        break;
      }
      case 'vertex':
      case 'edge':
      case 'face':
      case 'axis':
      case 'link':
        if (m.links[pick.id]) removeLink(m, pick.id);
        break;
    }
    app.select(null);
    app.endChange();
  }

  // ---------------------------------------------------------------------------
  // Select tool & dragging
  // ---------------------------------------------------------------------------

  private selectDown(ev: ViewportPointerEvent): void {
    const app = this.app;
    const pick = ev.pick;
    if (!pick) {
      app.select(null);
      return;
    }
    this.drag = { kind: 'pending', pick, startX: ev.clientX, startY: ev.clientY };
  }

  private startDrag(ev: ViewportPointerEvent, pick: PickResult): void {
    const app = this.app;
    const m = app.model;
    const vp = app.viewport;
    if (pick.type === 'editPoint' && app.mode === 'simulation' && pick.pointId && pick.pose !== undefined) {
      const existing = app.targetFor(pick.pointId, pick.pose);
      if (existing?.locked) {
        this.drag = { kind: 'none' };
        return;
      }
      const sp = app.sim.analysis?.spaces.find((s) => s.pointId === pick.pointId);
      const local = sp?.perPose[pick.pose];
      let mode: 'plane' | 'line' | 'free' | 'none' = 'free';
      let dirs: Vec3[] = [];
      if (!ev.shiftKey && local) {
        if (local.dim === 0) mode = 'none';
        else if (local.dim === 1) {
          mode = 'line';
          dirs = local.dirs;
        } else if (local.dim === 2) {
          mode = 'plane';
          dirs = local.dirs;
        }
      }
      if (mode === 'none') {
        app.setStatus(STATUS.dragHint);
        this.drag = { kind: 'none' };
        app.select({ type: 'editPoint', id: pick.pointId, pointId: pick.pointId, pose: pick.pose });
        return;
      }
      const origin = app.posePositions(pick.pose)?.get(pick.pointId) ?? pick.point;
      app.beginChange();
      let target = existing;
      let created = false;
      if (!target || target.kind !== 'position') {
        m.targets = m.targets.filter((t) => t !== existing);
        target = { id: `t_${m.nextId++}`, pointId: pick.pointId, pose: pick.pose, kind: 'position', position: [...origin] as Vec3, locked: false };
        m.targets.push(target);
        created = true;
      }
      this.drag = { kind: 'editPoint', pointId: pick.pointId, pose: pick.pose, target, created, mode, dirs, origin, moved: false };
      app.select({ type: 'editPoint', id: pick.pointId, pointId: pick.pointId, pose: pick.pose });
      app.setStatus(STATUS.dragHint);
      return;
    }
    if (app.mode !== 'construction') {
      this.drag = { kind: 'none' };
      return;
    }
    if (pick.type === 'vertex' && pick.pointId && pick.linkId) {
      const link = m.links[pick.linkId];
      if (link.locked) {
        app.setStatus(STATUS.linkLocked);
        this.drag = { kind: 'none' };
        return;
      }
      const p = m.points[pick.pointId].pos;
      app.beginChange();
      this.drag = { kind: 'point', pointId: pick.pointId, linkId: pick.linkId, plane: this.dragPlane(pick.linkId, p), moved: false };
      return;
    }
    if ((pick.type === 'edge' || pick.type === 'face' || pick.type === 'axis') && pick.linkId) {
      const link = m.links[pick.linkId];
      if (link.locked) {
        app.setStatus(STATUS.linkLocked);
        this.drag = { kind: 'none' };
        return;
      }
      const start = new Map<ID, Vec3>();
      for (const id of linkAllPointIds(link)) start.set(id, [...m.points[id].pos] as Vec3);
      app.beginChange();
      this.drag = { kind: 'link', linkId: pick.linkId, grab: pick.point, start, plane: this.dragPlane(pick.linkId, pick.point), moved: false };
      return;
    }
    this.drag = { kind: 'none' };
    void vp;
  }

  /** Plane in which a link / point is dragged: the sketch plane for planar links, else the view plane. */
  private dragPlane(linkId: ID, at: Vec3): { o: Vec3; n: Vec3 } {
    const m = this.app.model;
    for (const j of Object.values(m.joints)) {
      if (j.type === 'planar' && j.a.kind === 'body' && j.a.linkId === linkId && isConstructionRef(j.b)) {
        const c = m.construction[j.b.constructionId];
        if (c?.dir) return { o: at, n: c.dir };
      }
    }
    return { o: at, n: this.app.viewport.viewDirection() };
  }

  private dragMove(ev: ViewportPointerEvent): void {
    const app = this.app;
    const m = app.model;
    const vp = app.viewport;
    if (this.drag.kind === 'pending') {
      const dx = ev.clientX - this.drag.startX;
      const dy = ev.clientY - this.drag.startY;
      if (Math.hypot(dx, dy) < 4) return;
      const pick = this.drag.pick;
      this.startDrag(ev, pick);
      if ((this.drag as DragState).kind === 'none') return;
    }
    const d = this.drag as DragState;
    if (d.kind === 'point') {
      const pos = vp.projectToPlane(ev.clientX, ev.clientY, d.plane.o, d.plane.n) ?? vp.projectToViewPlane(ev.clientX, ev.clientY, d.plane.o);
      if (!pos) return;
      const target = this.gridSnap(ev.pick?.type === 'vertex' && ev.pick.pointId !== d.pointId && ev.pick.pointId ? m.points[ev.pick.pointId].pos : pos);
      const link = m.links[d.linkId];
      const res = solveSketch(m, { dragTargets: [{ pointId: d.pointId, pos: target, weight: 1 }], freePointIds: new Set([d.pointId]), allowGroundMove: link.ground, maxIter: 25 });
      for (const [id, p] of res.positions) m.points[id].pos = p;
      d.moved = true;
      app.setOverlay({ snapPoint: ev.pick?.type === 'vertex' && ev.pick.pointId !== d.pointId ? target : null });
      app.requestRender();
      return;
    }
    if (d.kind === 'link') {
      const pos = vp.projectToPlane(ev.clientX, ev.clientY, d.plane.o, d.plane.n) ?? vp.projectToViewPlane(ev.clientX, ev.clientY, d.plane.o);
      if (!pos) return;
      const delta = this.gridSnap(sub(pos, d.grab));
      const link = m.links[d.linkId];
      const targets = link.pointIds.map((id) => ({ pointId: id, pos: add(d.start.get(id)!, delta), weight: 1 }));
      const res = solveSketch(m, { dragTargets: targets, allowGroundMove: link.ground, maxIter: 25 });
      for (const [id, p] of res.positions) m.points[id].pos = p;
      d.moved = true;
      app.requestRender();
      return;
    }
    if (d.kind === 'editPoint') {
      let pos: Vec3 | null = null;
      const mode = ev.shiftKey ? 'free' : d.mode;
      if (mode === 'line') pos = vp.projectToLine(ev.clientX, ev.clientY, d.origin, d.dirs[0]);
      else if (mode === 'plane') pos = vp.projectToPlane(ev.clientX, ev.clientY, d.origin, normalize(cross(d.dirs[0], d.dirs[1])));
      if (!pos) pos = vp.projectToViewPlane(ev.clientX, ev.clientY, d.origin);
      if (!pos) return;
      d.target.position = pos;
      d.moved = true;
      app.solveDesignInteractive();
      return;
    }
  }

  private dragEnd(ev: ViewportPointerEvent): void {
    const app = this.app;
    const m = app.model;
    const d = this.drag;
    this.drag = { kind: 'none' };
    app.setOverlay({});
    if (d.kind === 'pending') {
      // click without drag: select
      const pick = d.pick;
      if (pick.type === 'vertex' && pick.pointId) {
        app.select({ type: 'vertex', id: pick.linkId ?? pick.id, pointId: pick.pointId });
        this.onPopup?.(pick.pointId);
      } else if (pick.type === 'editPoint' && pick.pointId) {
        app.select({ type: 'editPoint', id: pick.pointId, pointId: pick.pointId, pose: pick.pose });
      } else if (pick.type === 'edge' || pick.type === 'face' || pick.type === 'axis') {
        app.select({ type: 'link', id: pick.linkId ?? pick.id });
      } else app.select({ type: pick.type, id: pick.id });
      return;
    }
    if (d.kind === 'point') {
      // snap-join when released on another vertex that is not already joined
      if (ev.pick?.type === 'vertex' && ev.pick.pointId && ev.pick.pointId !== d.pointId) {
        const other = m.points[ev.pick.pointId];
        const already = jointsAtPoint(m, d.pointId).some((j) => j.pairs?.some(([a, b]) => a === other.id || b === other.id));
        if (other.linkId !== d.linkId && !already) {
          addJoint(m, m.settings.defaultJoint, { linkId: d.linkId, kind: 'vertex', pointIds: [d.pointId] }, { linkId: other.linkId, kind: 'vertex', pointIds: [other.id] }, { axis: sketchNormal(m) });
        }
      }
      const res = solveSketch(m, { freePointIds: new Set([d.pointId]), allowGroundMove: m.links[d.linkId].ground, maxIter: 40 });
      commitSketch(m, res, new Set([d.pointId]));
      app.select({ type: 'vertex', id: d.linkId, pointId: d.pointId });
      app.endChange();
      return;
    }
    if (d.kind === 'link') {
      const res = solveSketch(m, { allowGroundMove: m.links[d.linkId].ground, maxIter: 40 });
      commitSketch(m, res);
      app.select({ type: 'link', id: d.linkId });
      app.endChange();
      return;
    }
    if (d.kind === 'editPoint') {
      if (!d.moved && d.created) {
        // click without drag: remove the provisional target
        m.targets = m.targets.filter((t) => t !== d.target);
      }
      app.endChange();
      app.select({ type: 'editPoint', id: d.pointId, pointId: d.pointId, pose: d.pose });
      return;
    }
  }

  get pointer(): { x: number; y: number } | null {
    return this.lastPointer;
  }
}

