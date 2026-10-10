/**
 * Interactive tools: selection & dragging, link / polygon / prism / cylinder
 * creation, construction geometry, joints, ground, drivers, deletion and
 * editing-point dragging for inverse design.
 */
import type { App, ToolName } from '../app';
import { add, dist, dot, len, normalize, scale, sub, cross } from '../core/geometry';
import { solveSketch, syncDriverValues } from '../core/kinematics';
import { tryAddJoint, trySolveCommit } from '../core/feasibility';
import { driveCrease, findCreaseLoops, isCrease } from '../core/fold';
import {
  addAngleDriver,
  addBar,
  addConstructionAxis,
  addConstructionPlane3,
  addConstructionPoint,
  addCylinder,
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
import type { ConstructionRef, Feature, ID, JointType, Target, Vec3 } from '../core/types';
import { autoJoinCoincident, coincidentVertex, fitSketchPlane, moveVertex, placeOnFittedPlane, placementOnPlane, projectToPlane } from '../core/edit';
import { sectorPreview, sectorStatus, type SectorStatus } from '../core/sector';
import { duplicateLink, hasCollinearTriple, linearArray, mirrorAcrossPlane, polarArray } from '../core/patterns';
import { addPolygonFromPoints } from '../core/model';
import { isConstructionRef } from '../core/types';
import { FEATURES, OVERLAY, STATUS, TOOLS, CONSTRUCTION_NAMES, jointRefusedMessage } from '../ui/strings';
import type { OverlayLabel, OverlayView } from './render';
import type { PickResult, ViewportPointerEvent } from './scene';
import { PickCycle } from './pickCycle';

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

/** Weight of soft drag targets relative to hard constraints: joints stay satisfied while the body follows the pointer. */
const DRAG_WEIGHT = 0.05;

/** CSS classes of the Panel tool's sector-angle labels (src/style.css). */
const SECTOR_LABEL_CLASS: Record<SectorStatus, string> = { neutral: 'label label--sector', ok: 'label label--sector-ok', bad: 'label label--sector-bad' };

interface PlacedPoint {
  pos: Vec3;
  snappedPointId?: ID;
  snappedConstructionId?: ID;
  /** Pointer ray at placement (free sketch vertices are re-placed on the fitted plane along it). */
  ray?: { o: Vec3; d: Vec3 };
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
  /** Query cycle: a right-click steps through everything under the pointer; the current candidate is shown as the hover. */
  private cycle = new PickCycle();
  /** True while the pick being dispatched was named through the query cycle (the user chose it explicitly). */
  private pickFromCycle = false;
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
    this.cycle.cancel();
    this.app.setOverlay({});
    this.app.setHint(toolHint(this.app.tool));
  }

  /** True while a query cycle (right-click) is stepping through the features under the pointer. */
  get queryActive(): boolean {
    return this.cycle.active;
  }

  setTool(tool: ToolName): void {
    this.app.setTool(tool);
    this.reset();
    // Mirror / Pattern act on the current selection (e.g. picked in the model tree) when there is one
    if (tool === 'mirror' || tool === 'pattern') {
      const sel = this.app.selection;
      const id = sel && (sel.type === 'link' || sel.type === 'vertex' || sel.type === 'edge' || sel.type === 'face' || sel.type === 'axis') ? sel.id : null;
      if (id && this.app.model.links[id]) {
        this.patternLinkId = id;
        this.app.setStatus(tool === 'mirror' ? STATUS.mirrorPickPlane : STATUS.patternPickFirst);
      }
    }
  }

  cancel(): void {
    if (this.cycle.active) {
      // Esc first leaves the query cycle (normal hover resumes); a second Esc cancels the tool as before
      this.endCycle();
      return;
    }
    if (this.drag.kind !== 'none' && this.drag.kind !== 'pending') {
      // abort the drag: restore the pre-drag model without touching the undo history
      this.app.abortChange();
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

  /**
   * Where would a click at this pointer position place a point? A hovered vertex or datum point snaps; otherwise the
   * pointer ray is cast onto the active sketch plane (always for the datum and Panel tools and in 2-D mode; in 3-D mode
   * unless the ray grazes or misses the plane, plan 2c) and the view plane through the previous point is the fallback.
   */
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
    const pl = this.sketchPlane();
    const ray = vp.pointerRay(ev.clientX, ev.clientY);
    const onSketchPlane = this.app.toolOptions.mode2d || this.app.tool === 'cpoint' || this.app.tool === 'caxis' || this.app.tool === 'cplane' || this.app.tool === 'sketch';
    if (onSketchPlane) pos = vp.projectToPlane(ev.clientX, ev.clientY, pl.o, pl.n);
    if (!pos) {
      const view = vp.projectToViewPlane(ev.clientX, ev.clientY, last ?? [0, 0, 0]);
      // 3-D mode (plan 2c): a free click still lands where the pointer ray meets the sketch plane, so geometry is built
      // on a predictable plane; the view plane through the previous point serves only when the ray grazes or misses it
      pos = onSketchPlane || !view ? view : placementOnPlane(ray, pl.o, pl.n, view);
    }
    if (!pos) return null;
    pos = this.gridSnap(pos);
    if (this.pendingLength !== null && last) {
      const d = sub(pos, last);
      const L = len(d);
      if (L > 1e-9) pos = add(last, scale(d, this.pendingLength / L));
    }
    return { pos, ray };
  }

  /**
   * Final positions of the sketched vertices: snapped vertices stay exact; free
   * vertices lie on the fitted plane, placed along their pointer ray when the
   * plane is not the sketch plane (so they end up under the cursor).
   */
  private sketchPositions(points: PlacedPoint[]): { pts: Vec3[]; onSketchPlane: boolean } {
    const pl = this.sketchPlane();
    const snapped = points.map((p) => !!(p.snappedPointId || p.snappedConstructionId));
    const fit = fitSketchPlane(points.map((p) => p.pos), snapped, pl);
    const pts = points.map((p, i) => {
      if (snapped[i]) return p.pos;
      if (fit.onSketchPlane) return projectToPlane(p.pos, fit.origin, fit.normal);
      return placeOnFittedPlane(p.pos, p.ray, fit.origin, fit.normal);
    });
    return { pts, onSketchPlane: fit.onSketchPlane };
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
      this.cycle.cancel();
      app.setHover(null);
      return;
    }
    if (ev.kind === 'query') return this.queryClick(ev);
    if (ev.kind === 'move') {
      if (this.drag.kind !== 'none') return this.dragMove(ev);
      if (this.cycle.active) {
        // within a few pixels of the query position the highlighted candidate stays (normal hover would replace it);
        // farther away the cycle ends and normal hover resumes
        if (this.cycle.isSamePlace(ev.clientX, ev.clientY)) {
          this.updatePreview({ ...ev, pick: this.cycle.current });
          return;
        }
        this.cycle.cancel();
        if (!ev.pick) app.setStatus('');
      }
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
    // a left-click while a query cycle is active uses the highlighted candidate as the pick of the active tool (every
    // tool, and the snap of place(), reads ev.pick); the candidate may be stale when the model changed meanwhile
    const chosen = this.cycle.active && this.cycle.isSamePlace(ev.clientX, ev.clientY) ? this.cycle.current : null;
    this.cycle.cancel();
    const queried = chosen !== null && this.pickExists(chosen);
    this.pickFromCycle = queried;
    try {
      this.dispatchDown(queried ? { ...ev, pick: chosen } : ev);
    } finally {
      this.pickFromCycle = false;
    }
  }

  /**
   * Right-click that was not an orbit drag: start the query cycle at this spot
   * (first candidate that is not the current hover) or step it when it is
   * already running here. The candidate is shown as the hover (its highlight)
   * and the status bar reads "2 of 3 · edge V0-V1 of Polygon 2 · right-click:
   * next …"; a left-click then uses it (handle()).
   */
  private queryClick(ev: ViewportPointerEvent): void {
    const app = this.app;
    if (this.drag.kind !== 'none') return;
    const here = { x: ev.clientX, y: ev.clientY };
    const cand = this.cycle.active && this.cycle.isSamePlace(here.x, here.y) ? this.cycle.next() : this.cycle.start(app.viewport.pickCandidates(here.x, here.y), here, app.hover);
    if (!cand) {
      app.setStatus(STATUS.queryNothing);
      return;
    }
    app.setHover(cand);
    this.updatePreview({ ...ev, pick: cand });
    app.setStatus(STATUS.queryPick(this.cycle.index + 1, this.cycle.count, this.describePick(cand), this.cycle.count > 1 ? STATUS.queryHint : STATUS.queryHintSingle));
  }

  /** Leave the query cycle: the normal hover at the pointer position resumes. */
  private endCycle(): void {
    this.cycle.cancel();
    const p = this.lastPointer ? this.app.viewport.pick(this.lastPointer.x, this.lastPointer.y) : null;
    this.app.setHover(p);
    this.app.setStatus(p ? this.describePick(p) : '');
  }

  /** Does the feature a pick names still exist? A cycle candidate may outlive an undo or a deletion. */
  private pickExists(p: PickResult): boolean {
    const m = this.app.model;
    switch (p.type) {
      case 'joint':
        return !!m.joints[p.id];
      case 'construction':
        return !!m.construction[p.id];
      case 'editPoint':
        return !!m.points[p.pointId ?? p.id];
      default:
        return !!m.links[p.id] && (p.pointIds ?? []).every((id) => !!m.points[id]);
    }
  }

  private dispatchDown(ev: ViewportPointerEvent): void {
    const app = this.app;
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
    const overlay: OverlayView = { snapPoint: placed.snappedPointId || placed.snappedConstructionId ? placed.pos : null, marker: placed.pos };
    const first = this.points[0]?.pos;
    if (first && (tool === 'bar' || tool === 'caxis' || tool === 'cylinder')) overlay.rubberBand = { a: first, b: placed.pos };
    if (first && (tool === 'polygon' || tool === 'prism')) {
      const r = dist(first, placed.pos);
      overlay.circle = { center: first, normal: this.sketchPlane().n, radius: r };
      overlay.rubberBand = { a: first, b: placed.pos };
    }
    if (tool === 'cplane' && this.points.length > 0) overlay.polyline = [...this.points.map((p) => p.pos), placed.pos];
    if (tool === 'sketch' && this.points.length > 0) {
      // the cursor vertex counts as placed on an existing vertex when it is snapped or lands exactly on one
      const cursor = this.withCoincidence([{ pos: placed.pos, snappedPointId: placed.snappedPointId, snappedConstructionId: placed.snappedConstructionId, ray: placed.ray }])[0];
      const all = [...this.withCoincidence(this.points), cursor];
      const { pts } = this.sketchPositions(all);
      overlay.polyline = pts;
      overlay.rubberBand = { a: pts[pts.length - 2], b: pts[pts.length - 1] };
      overlay.marker = pts[pts.length - 1];
      overlay.labels = this.sectorLabels(all, pts);
    }
    if (tool === 'pattern' && this.patternLinkId && this.points.length === 1 && app.toolOptions.patternKind === 'linear') overlay.rubberBand = { a: this.points[0].pos, b: placed.pos };
    if (tool === 'edit' && this.editSource) overlay.rubberBand = { a: app.model.points[this.editSource.pointId]?.pos ?? placed.pos, b: placed.pos };
    app.setOverlay(overlay);
  }

  /**
   * Sector-angle labels of the Panel tool: at every sketch vertex that sits on
   * an existing vertex, the running sum of the corner angles of the panels
   * around it plus the angle the new panel adds between its two adjacent
   * sketch edges (sectorPreview). `pts` is the closed polyline including the
   * cursor, so the cursor closes the open edge; a cursor resting on the first
   * vertex (about to close the polygon) is left out, since it duplicates it.
   * Green when the panel closes the ring at 360°, red when the angles cannot
   * lie flat, neutral otherwise (sectorStatus).
   */
  private sectorLabels(points: PlacedPoint[], pts: Vec3[]): OverlayLabel[] {
    const m = this.app.model;
    const n = pts.length >= 3 && dist(pts[pts.length - 1], pts[0]) < 1e-9 ? pts.length - 1 : pts.length;
    if (n < 3) return [];
    const labels: OverlayLabel[] = [];
    for (let i = 0; i < n; i++) {
      const pid = points[i].snappedPointId;
      if (!pid || !m.points[pid]) continue;
      const prev = pts[(i + n - 1) % n];
      const next = pts[(i + 1) % n];
      if (dist(prev, pts[i]) < 1e-9 || dist(next, pts[i]) < 1e-9) continue;
      const s = sectorPreview(m, pts[i], prev, next, pid);
      labels.push({ pos: pts[i], text: OVERLAY.sectorSum(s.sumDeg, s.addedDeg), cls: SECTOR_LABEL_CLASS[sectorStatus(s)] });
    }
    return labels;
  }

  /**
   * Sketch vertices that coincide with an existing vertex (coincidentVertex:
   * typed coordinates, grid-snapped clicks) are treated exactly like snapped
   * ones: they keep the existing vertex's position, take part in the plane
   * fit and are joined when the panel is closed. Points already snapped to a
   * vertex or a datum point are returned unchanged.
   */
  private withCoincidence(points: PlacedPoint[]): PlacedPoint[] {
    const m = this.app.model;
    return points.map((p) => {
      if (p.snappedPointId || p.snappedConstructionId) return p;
      const q = coincidentVertex(m, p.pos);
      return q ? { ...p, pos: [...q.pos] as Vec3, snappedPointId: q.id } : p;
    });
  }

  // ---------------------------------------------------------------------------
  // Panel (sketch polygon) tool
  // ---------------------------------------------------------------------------

  private sketchTool(ev: ViewportPointerEvent, forced?: PlacedPoint): void {
    const app = this.app;
    const placed = forced ? this.withCoincidence([forced])[0] : this.place(ev, true);
    if (!placed) return;
    // clicking near the first vertex closes the polygon
    if (this.points.length >= 3 && !forced) {
      const s0 = app.viewport.worldToScreen(this.points[0].pos);
      if (Math.hypot(s0.x - ev.clientX, s0.y - ev.clientY) < 12) {
        this.finishSketch();
        return;
      }
    }
    // snapped vertices keep their exact (possibly 3-D) position: they define the polygon's plane when
    // the sketch is closed; free vertices are placed on the sketch plane and projected onto that plane later
    const pl = this.sketchPlane();
    const snapped = !!(placed.snappedPointId || placed.snappedConstructionId);
    const pos = snapped ? placed.pos : projectToPlane(placed.pos, pl.o, pl.n);
    const last = this.points[this.points.length - 1];
    if (last) {
      if (dist(last.pos, pos) < 1e-9) return;
      // the second click of a double-click lands within a few pixels of the last vertex: not a new vertex
      if (!forced) {
        const sl = app.viewport.worldToScreen(last.pos);
        if (Math.hypot(sl.x - ev.clientX, sl.y - ev.clientY) < 12) return;
      }
    }
    this.points.push({ pos, snappedPointId: placed.snappedPointId, snappedConstructionId: placed.snappedConstructionId, ray: placed.ray });
    app.setHint(TOOLS.sketch.hint);
  }

  /** True while the sketch tool has vertices that Backspace can remove. */
  popSketchSupported(): boolean {
    return this.app.tool === 'sketch' && this.points.length > 0;
  }

  /** Backspace while sketching removes the last vertex. Returns false when nothing was removed. */
  popSketchPoint(): boolean {
    if (this.app.tool !== 'sketch' || this.points.length === 0) return false;
    this.points.pop();
    if (this.points.length === 0) this.app.setOverlay({});
    return true;
  }

  /** Close the sketched polygon (Enter / double-click / click on the first vertex). */
  finishSketch(): void {
    const app = this.app;
    if (app.tool !== 'sketch') return;
    if (this.points.length < 3) {
      app.setStatus(STATUS.sketchNeedsThree);
      return;
    }
    // every vertex that sits on an existing vertex, snapped or typed, is joined (plan 2b): shared edges become creases,
    // single vertices get pins; a refused re-solve leaves the panel unjoined (autoJoinCoincident restores the model)
    const points = this.withCoincidence(this.points);
    const { pts, onSketchPlane } = this.sketchPositions(points);
    if (hasCollinearTriple(pts)) {
      app.setStatus(STATUS.sketchDegenerate);
      return;
    }
    const m = app.model;
    app.beginChange();
    const link = addPolygonFromPoints(m, pts, { name: app.nextLinkName('polygon'), onPlaneId: onSketchPlane ? m.settings.sketchPlaneId : null });
    const snapped = link.pointIds.filter((_, i) => points[i].snappedPointId);
    const joints = snapped.length ? autoJoinCoincident(m, link, snapped, { defaultJoint: m.settings.defaultJoint, axis: sketchNormal(m) }) : [];
    for (let i = 0; i < points.length; i++) {
      const cid = points[i].snappedConstructionId;
      if (cid && m.construction[cid]?.kind === 'point') addJoint(m, 'spherical', { linkId: link.id, kind: 'vertex', pointIds: [link.pointIds[i]] }, { constructionId: cid });
    }
    app.select({ type: 'link', id: link.id });
    app.endChange();
    const notes: string[] = [];
    if (joints.length) notes.push(`${STATUS.jointCreated} (${joints.length})`);
    if (!onSketchPlane) notes.push(STATUS.sketchOffPlane);
    if (notes.length) app.setStatus(notes.join(' · '));
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
    const res = moveVertex(m, this.editSource.pointId, placed.pos, { joinTo });
    let joints = res.joints.length;
    if (res.ok && placed.snappedConstructionId && m.construction[placed.snappedConstructionId]?.kind === 'point') {
      // snapped onto a datum point: pin the vertex there (like the Link and Sketch tools do)
      const alreadyPinned = jointsAtPoint(m, this.editSource.pointId).some((j) => isConstructionRef(j.b) && j.b.constructionId === placed.snappedConstructionId);
      if (!alreadyPinned) {
        const j = addJoint(m, m.settings.defaultJoint === 'spherical' ? 'spherical' : 'revolute', { linkId: this.editSource.linkId, kind: 'vertex', pointIds: [this.editSource.pointId] }, { constructionId: placed.snappedConstructionId }, { axis: sketchNormal(m) });
        if (j) joints++;
      }
    }
    app.endChange({ skipUndo: !res.ok });
    app.setStatus(res.ok ? STATUS.editDone + (joints ? ` · ${STATUS.jointCreated} (${joints})` : '') : res.reason === 'locked' ? STATUS.linkLocked : STATUS.editUnreachable);
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
    // every vertex of the new polygon / prism that lands on an existing vertex is joined (plan 2b): shared edges become
    // creases, single vertices get pins; the first vertex is the clicked circumcircle point, so a snap there is kept
    // as a pair even when the polygon's plane leaves it slightly off the snapped vertex (the pin was always made)
    const snappedFirst = placed.snappedPointId && m.points[placed.snappedPointId] ? placed.snappedPointId : null;
    const joints = autoJoinCoincident(m, link, link.pointIds, { defaultJoint: m.settings.defaultJoint, axis: n, extraPairs: snappedFirst ? [[link.pointIds[0], snappedFirst]] : [] });
    app.select({ type: 'link', id: link.id });
    app.endChange();
    if (joints.length) app.setStatus(`${STATUS.jointCreated} (${joints.length})`);
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
    let f = this.featureFromPick(ev.pick);
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
    const type = app.toolOptions.jointType;
    let prefix = '';
    if (!isConstructionRef(f) && f.linkId === this.featureA.linkId) {
      // the hover found the first link again (coincident edges, a shared vertex): use the best-ranked compatible feature
      // of another link under the pointer instead, unless the user named this very feature through the query cycle
      const alt = this.pickFromCycle ? null : this.otherLinkCandidate(ev, type, this.featureA);
      if (!alt) {
        app.setStatus(STATUS.jointSameLink);
        return;
      }
      f = alt;
      prefix = STATUS.pickedOtherLink(this.describeFeature(alt));
    }
    const withPrefix = (text: string): string => (prefix ? `${prefix} · ${text}` : text);
    if (!jointCompatible(m, type, this.featureA, f)) {
      app.setStatus(STATUS.jointIncompatible);
      return;
    }
    // pre-flight: add the joint, re-solve so it is satisfied (a crease between edges of slightly different length
    // lets one link's edge adapt) and keep it only when the solve is accepted; otherwise the model is restored,
    // no undo entry is recorded and the status bar explains why (docs/CONSTRUCTION_PLAN.md, 0b)
    app.beginChange();
    const r = tryAddJoint(m, type, this.featureA, f, { axis: sketchNormal(m), pitch: app.toolOptions.pitch });
    if (!r.ok || !r.joint) {
      app.abortChange();
      // kept in the Mechanism panel as well: the diagnosis is long and the status bar is replaced on the next hover
      const message = jointRefusedMessage(r.diagnosis ?? { kind: 'infeasible', residual: r.residual });
      app.report(message);
      if (prefix) app.setStatus(withPrefix(message));
    } else {
      app.select({ type: 'joint', id: r.joint.id });
      // a crease that closes a flat loop of panels leaves the vertex in its singular flat state: point at Fold
      const flat = this.inFlatLoop(r.joint.id);
      app.setStatus(withPrefix(flat ? `${STATUS.jointCreated} · ${STATUS.vertexFlatHint}` : STATUS.jointCreated));
      app.endChange();
    }
    this.featureA = null;
  }

  /**
   * The best-ranked feature under the pointer that lies on a link other than
   * `a`'s and that the joint type accepts with `a` (datum geometry is never
   * substituted: joining to the X axis because two panel edges coincide would
   * surprise). The query cycle still lets the user override the choice.
   */
  private otherLinkCandidate(ev: { clientX: number; clientY: number }, type: JointType, a: Feature): Feature | null {
    const m = this.app.model;
    for (const c of this.app.viewport.pickCandidates(ev.clientX, ev.clientY)) {
      const f = this.featureFromPick(c);
      if (!f || isConstructionRef(f) || f.linkId === a.linkId) continue;
      if (jointCompatible(m, type, a, f)) return f;
    }
    return null;
  }

  /** Is the joint a crease of a flat crease loop (a vertex whose creases are all unfolded)? */
  private inFlatLoop(jointId: ID): boolean {
    const m = this.app.model;
    const j = m.joints[jointId];
    return !!j && isCrease(m, j) && findCreaseLoops(m).some((l) => l.flat && l.creaseIds.includes(jointId));
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
    let note = '';
    let active = -1; // index of the driver to activate (default: the last one)
    let message = STATUS.driverSet;
    if (pick.type === 'joint') {
      const j = m.joints[pick.id];
      if (j?.type === 'revolute') {
        // a fold driver on a flat vertex would sweep the degenerate straight-hinge branch: driveCrease pre-folds
        // first, in the same change (one undo entry), and leaves exactly one driver on the picked crease; a crease
        // that already has one keeps it (no second driver, which would freeze the vertex) and it becomes active
        const r = driveCrease(m, j.id);
        if (r.prefold) note = r.prefold.ok ? STATUS.prefolded : STATUS.flatVertexWarning;
        if (r.reused) message = STATUS.driverReused;
        if (r.driver) active = m.drivers.indexOf(r.driver);
      } else if (j && (j.type === 'prismatic' || j.type === 'cylindrical' || j.type === 'screw')) addSlideDriver(m, j.id);
    } else if (pick.linkId) {
      const cands = candidateAngleDrivers(m).filter((c) => c.linkId === pick.linkId);
      const c = cands.find((x) => x.pivotId === pick.pointId) ?? cands[0];
      if (c) addAngleDriver(m, c.linkId, c.pivotId, c.tipId, c.axis);
      else app.setStatus(TOOLS.driver.hint);
    }
    syncDriverValues(m);
    app.sim.activeDriver = active >= 0 ? active : Math.max(0, m.drivers.length - 1);
    app.endChange(); // records an undo entry only when the model changed (not for a reused driver on a folded vertex)
    app.setStatus(note ? `${message} · ${note}` : message);
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
    if (pick.type === 'construction') app.setStatus(STATUS.dragNothing);
    this.drag = { kind: 'none' };
    void vp;
  }

  /** Pointer position in the drag plane; near edge-on views fall back to the view plane, keeping the body in its sketch plane. */
  private dragPointer(ev: ViewportPointerEvent, plane: { o: Vec3; n: Vec3 }): { pos: Vec3; edgeOn: boolean } | null {
    const vp = this.app.viewport;
    const n = normalize(plane.n);
    const edgeOn = Math.abs(dot(n, vp.viewDirection())) < 0.15;
    const pos = edgeOn ? vp.projectToViewPlane(ev.clientX, ev.clientY, plane.o) : (vp.projectToPlane(ev.clientX, ev.clientY, plane.o, plane.n) ?? vp.projectToViewPlane(ev.clientX, ev.clientY, plane.o));
    return pos ? { pos, edgeOn } : null;
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
      const dp = this.dragPointer(ev, d.plane);
      if (!dp) return;
      let pos = dp.pos;
      if (dp.edgeOn) {
        const n = normalize(d.plane.n);
        const p0 = m.points[d.pointId].pos;
        pos = sub(pos, scale(n, dot(sub(pos, p0), n)));
      }
      const target = this.gridSnap(ev.pick?.type === 'vertex' && ev.pick.pointId !== d.pointId && ev.pick.pointId ? m.points[ev.pick.pointId].pos : pos);
      const link = m.links[d.linkId];
      const res = solveSketch(m, { dragTargets: [{ pointId: d.pointId, pos: target, weight: DRAG_WEIGHT }], freePointIds: new Set([d.pointId]), allowGroundMove: link.ground, maxIter: 30 });
      for (const [id, p] of res.positions) m.points[id].pos = p;
      d.moved = true;
      app.setOverlay({ snapPoint: ev.pick?.type === 'vertex' && ev.pick.pointId !== d.pointId ? target : null });
      app.requestRender();
      return;
    }
    if (d.kind === 'link') {
      const dp = this.dragPointer(ev, d.plane);
      if (!dp) return;
      let delta = sub(dp.pos, d.grab);
      if (dp.edgeOn) {
        const n = normalize(d.plane.n);
        delta = sub(delta, scale(n, dot(delta, n)));
      }
      delta = this.gridSnap(delta);
      const link = m.links[d.linkId];
      const targets = link.pointIds.map((id) => ({ pointId: id, pos: add(d.start.get(id)!, delta), weight: DRAG_WEIGHT }));
      const res = solveSketch(m, { dragTargets: targets, allowGroundMove: link.ground, maxIter: 30 });
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
      // exact solve on release (the drag preview used soft targets); a pose that cannot satisfy the constraints is
      // never committed: the pre-drag model comes back and the status bar says so
      const r = trySolveCommit(m, { freePointIds: new Set([d.pointId]), allowGroundMove: m.links[d.linkId].ground, maxIter: 40 });
      if (!r.ok) {
        app.abortChange();
        app.setStatus(STATUS.editRefused);
        return;
      }
      app.select({ type: 'vertex', id: d.linkId, pointId: d.pointId });
      app.endChange();
      return;
    }
    if (d.kind === 'link') {
      const r = trySolveCommit(m, { allowGroundMove: m.links[d.linkId].ground, maxIter: 40 });
      if (!r.ok) {
        app.abortChange();
        app.setStatus(STATUS.editRefused);
        return;
      }
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

