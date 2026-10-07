/**
 * Application state and orchestration: model, undo/redo, modes, kinematic
 * refresh (mobility, sweep, poses, inverse design) and the render-state
 * assembly for the viewport. UI panels and tools talk to this class.
 */
import { OBJExporter } from 'three/addons/exporters/OBJExporter.js';
import {
  applyPositions,
  computeMobility,
  currentViolation,
  pathOf,
  positionsFromModel,
  samplePoseValues,
  solveForward,
  solvePosesAt,
  sweepDriver,
  sweepGrid,
  syncDriverValues,
  type MobilityResult,
  type Pose,
  type Positions,
  type SweepResult,
} from './core/kinematics';
import { EXAMPLE_BUILDERS } from './core/examples';
import { addAngleDriver, candidateAngleDrivers, cloneModel, createModel, parseModel, refreshRigidity, serializeModel, groundLink } from './core/model';
import { analyseDesign, applyDesign, solveDesign, symmetricEigen3, type DesignAnalysis, type DesignResult, type PointDesignSpace } from './core/synthesis';
import { normalize, sub } from './core/geometry';
import type { ID, JointType, Model, Target, Vec3 } from './core/types';
import { loadSettings, saveSettings, type AppSettings } from './ui/settings';
import { LINK_NAMES, SIM, STATUS } from './ui/strings';
import { ModelRenderer, type DesignSpaceView, type EditPointView, type OverlayView, type PathView, type RenderState, type SelectionView, type SurfaceView } from './viewport/render';
import { Viewport, type PickResult } from './viewport/scene';

export type Mode = 'construction' | 'simulation' | 'preview';
export type ToolName = 'select' | 'bar' | 'sketch' | 'polygon' | 'prism' | 'cylinder' | 'edit' | 'cpoint' | 'caxis' | 'cplane' | 'joint' | 'ground' | 'driver' | 'mirror' | 'pattern' | 'delete';

export interface ToolOptions {
  sides: number;
  radius: number;
  height: number;
  mode2d: boolean;
  planeMode: 'three' | 'offset';
  offset: number;
  jointType: JointType;
  pitch: number;
  patternKind: 'linear' | 'polar';
  patternCount: number;
  patternAngle: number;
}

export interface SimState {
  activeDriver: number;
  sweep: SweepResult | null;
  /** Output surface grid for 2-DOF mechanisms (poses[iB][iA]). */
  surface: Pose[][] | null;
  poseValues: number[];
  poses: Pose[];
  design: DesignResult | null;
  analysis: DesignAnalysis | null;
  mobility: MobilityResult | null;
  violation: number;
  assembled: boolean;
  showDesignSpace: boolean;
  showEditPoints: boolean;
  showPaths: boolean;
  message: string;
}

export interface PreviewState {
  value: number;
  playing: boolean;
  speed: number; // units per second (deg/s)
  direction: 1 | -1;
  positions: Positions | null;
  trace: boolean;
}

export class App {
  model: Model;
  settings: AppSettings;
  mode: Mode = 'construction';
  tool: ToolName = 'select';
  toolOptions: ToolOptions = { sides: 4, radius: 1, height: 1, mode2d: true, planeMode: 'three', offset: 1, jointType: 'revolute', pitch: 1, patternKind: 'linear', patternCount: 3, patternAngle: 360 };
  selection: SelectionView | null = null;
  hover: PickResult | null = null;
  overlay: OverlayView = {};
  ghost: Positions | null = null;
  status = STATUS.ready;
  hint = '';
  viewport: Viewport;
  renderer: ModelRenderer;
  sim: SimState = { activeDriver: 0, sweep: null, surface: null, poseValues: [], poses: [], design: null, analysis: null, mobility: null, violation: 0, assembled: true, showDesignSpace: true, showEditPoints: true, showPaths: true, message: '' };
  preview: PreviewState = { value: 0, playing: false, speed: 60, direction: 1, positions: null, trace: true };
  fileName = '';

  private undoStack: string[] = [];
  private redoStack: string[] = [];
  private pendingSnapshot: string | null = null;
  private listeners = new Set<() => void>();
  private renderDirty = true;
  private kinematicsDirty = true;
  private lastFrame = performance.now();

  constructor(container: HTMLElement) {
    this.settings = loadSettings();
    this.model = createModel();
    this.viewport = new Viewport(container, this.settings);
    this.renderer = new ModelRenderer(this.viewport);
    this.viewport.onBeforeRender = () => this.frame();
  }

  // ---------------------------------------------------------------------------
  // Listeners / render scheduling
  // ---------------------------------------------------------------------------

  subscribe(fn: () => void): () => void {
    this.listeners.add(fn);
    return () => this.listeners.delete(fn);
  }

  /** Notify UI panels (state changed). */
  notify(): void {
    for (const l of this.listeners) l();
  }

  requestRender(): void {
    this.renderDirty = true;
  }

  private frame(): void {
    const now = performance.now();
    const dt = Math.min((now - this.lastFrame) / 1000, 0.1);
    this.lastFrame = now;
    if (this.kinematicsDirty) {
      this.kinematicsDirty = false;
      this.recomputeKinematics();
      this.notify();
      this.renderDirty = true;
    }
    if (this.mode === 'preview' && this.preview.playing && this.sim.sweep) {
      this.advancePreview(dt);
      this.renderDirty = true;
    }
    if (this.renderDirty) {
      this.renderDirty = false;
      this.renderer.update(this.renderState());
    }
  }

  // ---------------------------------------------------------------------------
  // Undo / redo & change tracking
  // ---------------------------------------------------------------------------

  /** Call before mutating the model (snapshot for undo). Safe to call repeatedly during a drag. */
  beginChange(): void {
    if (this.pendingSnapshot === null) this.pendingSnapshot = serializeModel(this.model);
  }

  /** Call after mutating the model. */
  endChange(opts: { skipUndo?: boolean; keepSelection?: boolean } = {}): void {
    if (this.pendingSnapshot !== null) {
      if (!opts.skipUndo && this.pendingSnapshot !== serializeModel(this.model)) {
        this.undoStack.push(this.pendingSnapshot);
        if (this.undoStack.length > 100) this.undoStack.shift();
        this.redoStack = [];
      }
      this.pendingSnapshot = null;
    }
    this.markDirty();
  }

  /** Discard an in-progress change (e.g. an aborted drag): restore the pending snapshot without touching undo/redo. */
  abortChange(): void {
    if (this.pendingSnapshot === null) return;
    this.model = parseModel(this.pendingSnapshot);
    this.pendingSnapshot = null;
    this.selection = null;
    this.ghost = null;
    this.markDirty();
  }

  /** Model changed: recompute kinematics on the next frame. */
  markDirty(): void {
    this.kinematicsDirty = true;
    this.renderDirty = true;
    this.notify();
  }

  undo(): void {
    const s = this.undoStack.pop();
    if (!s) {
      this.setStatus(STATUS.nothingToUndo);
      return;
    }
    this.redoStack.push(serializeModel(this.model));
    this.model = parseModel(s);
    this.selection = null;
    this.markDirty();
  }

  redo(): void {
    const s = this.redoStack.pop();
    if (!s) return;
    this.undoStack.push(serializeModel(this.model));
    this.model = parseModel(s);
    this.selection = null;
    this.markDirty();
  }

  get canUndo(): boolean {
    return this.undoStack.length > 0;
  }
  get canRedo(): boolean {
    return this.redoStack.length > 0;
  }

  // ---------------------------------------------------------------------------
  // Mode / tool / selection
  // ---------------------------------------------------------------------------

  setMode(mode: Mode): void {
    if (this.mode === mode) return;
    this.mode = mode;
    this.preview.playing = false;
    this.ghost = null;
    if (mode !== 'construction') this.setTool('select');
    if (mode === 'preview' && this.sim.sweep) this.preview.value = this.model.drivers[this.sim.activeDriver]?.value ?? 0;
    this.selection = null;
    this.markDirty();
  }

  setTool(tool: ToolName): void {
    this.tool = tool;
    this.overlay = {};
    this.notify();
    this.requestRender();
  }

  select(sel: SelectionView | null): void {
    this.selection = sel;
    this.ghost = null;
    if (sel?.type === 'editPoint' && sel.pose !== undefined) this.ghost = this.posePositions(sel.pose);
    this.notify();
    this.requestRender();
  }

  setHover(h: PickResult | null): void {
    const changed = JSON.stringify(h && { t: h.type, id: h.id, p: h.pointId, k: h.pose }) !== JSON.stringify(this.hover && { t: this.hover.type, id: this.hover.id, p: this.hover.pointId, k: this.hover.pose });
    this.hover = h;
    if (changed) {
      if (this.mode === 'simulation' && !(this.selection?.type === 'editPoint')) {
        this.ghost = h?.type === 'editPoint' && h.pose !== undefined ? this.posePositions(h.pose) : null;
      }
      this.requestRender();
    }
  }

  /** Lightweight listener for status / hint text (avoids rebuilding panels on every pointer move). */
  onStatus: (() => void) | null = null;

  setStatus(text: string): void {
    this.status = text;
    this.onStatus?.();
  }

  setHint(text: string): void {
    this.hint = text;
    this.onStatus?.();
  }

  setOverlay(o: OverlayView): void {
    this.overlay = o;
    this.requestRender();
  }

  updateSettings(patch: Omit<Partial<AppSettings>, 'colors'> & { colors?: Partial<AppSettings['colors']> }): void {
    this.settings = { ...this.settings, ...patch, colors: { ...this.settings.colors, ...(patch.colors ?? {}) } };
    saveSettings(this.settings);
    this.viewport.applySettings(this.settings);
    this.requestRender();
    this.notify();
  }

  // ---------------------------------------------------------------------------
  // Model lifecycle
  // ---------------------------------------------------------------------------

  loadModel(model: Model, fileName = ''): void {
    this.model = model;
    this.fileName = fileName;
    this.undoStack = [];
    this.redoStack = [];
    this.pendingSnapshot = null;
    this.selection = null;
    this.sim.sweep = null;
    this.sim.design = null;
    this.sim.analysis = null;
    this.preview.positions = null;
    this.markDirty();
    // frame after first kinematics pass
    requestAnimationFrame(() => this.zoomToFit());
  }

  newModel(): void {
    this.loadModel(createModel());
    this.setMode('construction');
  }

  loadExample(key: string): void {
    const builder = EXAMPLE_BUILDERS[key];
    if (!builder) return;
    const m = builder();
    // make sure the example is projected onto its constraints (origami pre-fold etc.)
    const res = solveForward(m, m.drivers.map((d) => d.value));
    if (res.converged) applyPositions(m, res.positions);
    syncDriverValues(m);
    this.loadModel(m, key);
  }

  zoomToFit(): void {
    const pts = Object.values(this.model.points)
      .filter((p) => p.role !== 'helper')
      .map((p) => p.pos);
    for (const path of this.currentPaths()) pts.push(...path.points);
    this.viewport.fit(pts);
  }

  /** Default display name for a new link of a given kind. */
  nextLinkName(kind: string): string {
    const base = LINK_NAMES[kind] ?? kind;
    const count = Object.values(this.model.links).filter((l) => l.kind === kind).length + 1;
    return `${base} ${count}`;
  }

  // ---------------------------------------------------------------------------
  // Kinematics refresh
  // ---------------------------------------------------------------------------

  private ensureDriver(): void {
    if (this.model.drivers.length > 0) return;
    const cands = candidateAngleDrivers(this.model);
    if (cands.length === 0) return;
    const c = cands[0];
    addAngleDriver(this.model, c.linkId, c.pivotId, c.tipId, c.axis);
  }

  private recomputeKinematics(): void {
    const m = this.model;
    const sim = this.sim;
    sim.message = '';
    try {
      sim.mobility = Object.keys(m.links).length ? computeMobility(m) : null;
      sim.violation = Object.keys(m.links).length ? currentViolation(m) : 0;
    } catch (e) {
      console.error(e);
      sim.mobility = null;
    }
    const needMotion = this.mode !== 'construction' || m.settings.displayPointIds.length > 0;
    if (!needMotion) {
      sim.sweep = null;
      sim.poses = [];
      sim.design = null;
      sim.analysis = null;
      return;
    }
    if (this.mode !== 'construction') this.ensureDriver();
    const grounded = sim.mobility?.grounded ?? groundLink(m) !== null;
    if (m.drivers.length === 0 || !grounded) {
      sim.sweep = null;
      sim.surface = null;
      sim.poses = [];
      sim.design = null;
      sim.analysis = null;
      sim.message = grounded ? SIM.noDriver : SIM.notGrounded;
      return;
    }
    if (sim.activeDriver >= m.drivers.length) sim.activeDriver = 0;
    try {
      this.computeSweepAndPoses();
      if (this.mode === 'simulation') {
        this.computeDesign(true);
      } else {
        sim.design = null;
        sim.analysis = null;
      }
      if (this.mode === 'preview') this.updatePreviewPositions();
    } catch (e) {
      console.error(e);
      sim.message = String(e);
    }
  }

  private computeSweepAndPoses(): void {
    const m = this.model;
    const sim = this.sim;
    const sweep = sweepDriver(m, sim.activeDriver);
    sim.sweep = sweep;
    const d = m.drivers[sim.activeDriver];
    d.range = sweep.range;
    d.isCrank = sweep.isCrank;
    sim.poseValues = samplePoseValues(sweep, m.settings.poseCount);
    sim.poses = solvePosesAt(m, sweep, sim.poseValues);
    sim.assembled = sim.poses.some((p) => p.converged);
    // 2-DOF mechanisms: the output path of a point is a surface; sample it on a grid
    sim.surface = null;
    if (m.drivers.length >= 2 && (sim.mobility?.dof ?? 0) >= 2 && m.settings.displayPointIds.length > 0) {
      const other = sim.activeDriver === 0 ? 1 : 0;
      try {
        sim.surface = sweepGrid(m, sim.activeDriver, other, 9);
      } catch (e) {
        console.warn(e);
      }
    }
  }

  /** Solve the stacked design system with the current targets; applies the design to the model when it changed. */
  private computeDesign(allowReSweep: boolean): void {
    const m = this.model;
    const sim = this.sim;
    if (!sim.sweep) return;
    const poseValues = sim.poseValues.map((v) => {
      const vals = [...sim.sweep!.heldValues];
      vals[sim.activeDriver] = v;
      return vals;
    });
    const init = sim.design && sim.design.poses.length === sim.poses.length ? sim.design.poses.map((p) => p.positions) : sim.poses.map((p) => p.positions);
    const res = solveDesign({ model: m, poseValues, initPositions: init, soft: m.settings.softAssumptions, maxIter: 80 });
    sim.design = res;
    if (res.converged && m.targets.length > 0) {
      const changed = applyDesign(m, res);
      if (changed.length > 0) {
        this.refreshConstructionPose();
        if (allowReSweep) {
          this.computeSweepAndPoses();
          sim.design = solveDesign({ model: m, poseValues, initPositions: sim.poses.map((p) => p.positions), soft: m.settings.softAssumptions, maxIter: 40 });
        }
      }
    }
    sim.analysis = analyseDesign(m, sim.design, m.settings.displayPointIds);
    this.augmentDesignSpaces();
    if (sim.analysis.overConstrained) sim.message = SIM.overConstrained;
  }

  /**
   * For multi-DOF mechanisms the held drivers also move the point: fold the
   * tangent directions of the sampled output surface into each design space so
   * e.g. a spherical pendulum reports a 3-D ball rather than a disc.
   */
  private augmentDesignSpaces(): void {
    const grid = this.sim.surface;
    const an = this.sim.analysis;
    if (!grid || !an) return;
    for (const sp of an.spaces) {
      const tangents: Vec3[] = [];
      for (let b = 0; b < grid.length; b += Math.max(1, Math.floor(grid.length / 4))) {
        const row = grid[b];
        for (let a = 0; a + 1 < row.length; a += Math.max(1, Math.floor(row.length / 6))) {
          const p0 = row[a].positions.get(sp.pointId);
          const p1 = row[a + 1].positions.get(sp.pointId);
          const q = grid[Math.min(b + 1, grid.length - 1)][Math.min(a, grid[Math.min(b + 1, grid.length - 1)].length - 1)]?.positions.get(sp.pointId);
          if (p0 && p1) tangents.push(normalize(sub(p1, p0)));
          if (p0 && q && b + 1 < grid.length) tangents.push(normalize(sub(q, p0)));
        }
      }
      if (tangents.length === 0) continue;
      const G = new Float64Array(9);
      for (const d of [...sp.dirs, ...tangents]) for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) G[i * 3 + j] += d[i] * d[j];
      const eig = symmetricEigen3(G);
      const dirs: Vec3[] = [];
      for (let i = 0; i < 3; i++) if (eig.values[i] > 1e-6) dirs.push(eig.vectors[i]);
      (sp as PointDesignSpace).dirs = dirs;
      (sp as PointDesignSpace).dim = dirs.length;
    }
  }

  /** After the design changed, re-solve the construction pose at the current driver values. */
  private refreshConstructionPose(): void {
    const m = this.model;
    const res = solveForward(m, m.drivers.map((d) => d.value), positionsFromModel(m));
    if (res.converged) applyPositions(m, res.positions);
    else if (this.sim.design && this.sim.design.poses.length) {
      applyPositions(m, this.sim.design.poses[0].positions);
      syncDriverValues(m);
    }
  }

  /** Interactive (during drag) design solve: fewer iterations, no re-sweep. */
  solveDesignInteractive(): void {
    const m = this.model;
    const sim = this.sim;
    if (!sim.sweep) return;
    const poseValues = sim.poseValues.map((v) => {
      const vals = [...sim.sweep!.heldValues];
      vals[sim.activeDriver] = v;
      return vals;
    });
    const init = sim.design ? sim.design.poses.map((p) => p.positions) : sim.poses.map((p) => p.positions);
    const res = solveDesign({ model: m, poseValues, initPositions: init, soft: m.settings.softAssumptions, maxIter: 12 });
    sim.design = res;
    if (res.converged) {
      applyDesign(m, res);
      this.refreshConstructionPose();
    }
    sim.analysis = analyseDesign(m, res, m.settings.displayPointIds);
    this.augmentDesignSpaces();
    sim.message = res.converged ? '' : SIM.overConstrained;
    if (this.selection?.type === 'editPoint' && this.selection.pose !== undefined) this.ghost = this.posePositions(this.selection.pose);
    this.requestRender();
    this.notify();
  }

  /** Positions of a sampled pose (design result preferred). */
  posePositions(pose: number): Positions | null {
    const src = this.sim.design?.poses ?? this.sim.poses;
    return src[pose]?.positions ?? null;
  }

  /** Find or create the editing-point target for (pointId, pose). */
  targetFor(pointId: ID, pose: number): Target | undefined {
    return this.model.targets.find((t) => t.pointId === pointId && t.pose === pose);
  }

  setActiveDriver(index: number): void {
    this.sim.activeDriver = index;
    this.sim.design = null;
    this.markDirty();
  }

  /** Commit the current geometry as the design (rest lengths from current positions). */
  solidifyAssumptions(): void {
    this.beginChange();
    for (const l of Object.values(this.model.links)) refreshRigidity(this.model, l);
    this.endChange();
  }

  // ---------------------------------------------------------------------------
  // Preview animation
  // ---------------------------------------------------------------------------

  setPreviewValue(v: number): void {
    this.preview.value = v;
    this.updatePreviewPositions();
    this.requestRender();
    this.notify();
  }

  private advancePreview(dt: number): void {
    const sweep = this.sim.sweep;
    if (!sweep) return;
    const [lo, hi] = sweep.range;
    let v = this.preview.value + this.preview.speed * dt * this.preview.direction;
    if (sweep.isCrank) {
      if (v >= hi) v = lo + (v - hi);
      if (v < lo) v = hi - (lo - v);
    } else {
      if (v > hi) {
        v = hi;
        this.preview.direction = -1;
      } else if (v < lo) {
        v = lo;
        this.preview.direction = 1;
      }
    }
    this.preview.value = v;
    this.updatePreviewPositions();
    this.notify();
  }

  private updatePreviewPositions(): void {
    const sweep = this.sim.sweep;
    if (!sweep) {
      this.preview.positions = null;
      return;
    }
    // warm start from the nearest sweep pose and solve exactly
    let best = sweep.poses[0];
    for (const p of sweep.poses) if (Math.abs(p.value - this.preview.value) < Math.abs(best.value - this.preview.value)) best = p;
    const vals = [...sweep.heldValues];
    vals[sweep.driverIndex] = this.preview.value;
    const res = solveForward(this.model, vals, best.positions, { maxIter: 30 });
    this.preview.positions = res.converged ? res.positions : best.positions;
  }

  // ---------------------------------------------------------------------------
  // Render state assembly
  // ---------------------------------------------------------------------------

  private currentPaths(): PathView[] {
    const m = this.model;
    const ids = m.settings.displayPointIds.filter((id) => m.points[id]);
    if (ids.length === 0) return [];
    const sim = this.sim;
    if (this.mode === 'simulation' && sim.design && sim.design.poses.length > 1) {
      if (!m.settings.softAssumptions && sim.analysis && sim.analysis.designDOF > 0) return [];
      return ids.map((id) => ({ pointId: id, points: pathOf(sim.design!.poses, id), closed: !!sim.sweep?.isCrank }));
    }
    if (sim.sweep) return ids.map((id) => ({ pointId: id, points: pathOf(sim.sweep!.poses, id), closed: sim.sweep!.isCrank }));
    return [];
  }

  private currentSurfaces(): SurfaceView[] {
    const grid = this.sim.surface;
    if (!grid) return [];
    const out: SurfaceView[] = [];
    for (const id of this.model.settings.displayPointIds) {
      if (!this.model.points[id]) continue;
      const rows: Vec3[][] = [];
      for (const row of grid) {
        const pts = pathOf(row, id);
        if (pts.length > 1) rows.push(pts);
      }
      if (rows.length > 1) out.push({ pointId: id, rows, closed: !!this.sim.sweep?.isCrank });
    }
    return out;
  }

  private currentEditPoints(): EditPointView[] {
    if (this.mode !== 'simulation' || !this.sim.showEditPoints) return [];
    const m = this.model;
    const src = this.sim.design?.poses ?? this.sim.poses;
    const out: EditPointView[] = [];
    for (const id of m.settings.displayPointIds) {
      if (!m.points[id]) continue;
      src.forEach((pose, k) => {
        const p = pose.positions.get(id);
        if (!p) return;
        const t = this.targetFor(id, k);
        out.push({ pointId: id, pose: k, pos: p, constrained: !!t, locked: !!t?.locked });
      });
    }
    return out;
  }

  private currentDesignSpaces(): DesignSpaceView[] {
    if (this.mode !== 'simulation' || !this.sim.showDesignSpace || !this.sim.analysis) return [];
    return this.sim.analysis.spaces.map((s) => ({ pointId: s.pointId, dim: s.dim, dirs: s.dirs, center: s.center, radius: s.radius }));
  }

  renderState(): RenderState {
    let positions = positionsFromModel(this.model);
    if (this.mode === 'preview' && this.preview.positions) positions = this.preview.positions;
    return {
      model: this.model,
      positions,
      settings: this.settings,
      selection: this.selection,
      hover: this.hover,
      paths: this.sim.showPaths || this.mode === 'preview' ? this.currentPaths() : [],
      surfaces: this.sim.showPaths && this.sim.surface ? this.currentSurfaces() : [],
      designSpaces: this.currentDesignSpaces(),
      editPoints: this.currentEditPoints(),
      ghost: this.mode === 'simulation' ? this.ghost : null,
      overlay: this.overlay,
      displayPointIds: this.model.settings.displayPointIds,
    };
  }

  // ---------------------------------------------------------------------------
  // File IO
  // ---------------------------------------------------------------------------

  /** Serialise the model, solidifying the current design first. */
  saveToJson(): string {
    for (const l of Object.values(this.model.links)) refreshRigidity(this.model, l);
    return serializeModel(this.model);
  }

  download(name: string, content: string | Blob, type = 'application/json'): void {
    const blob = content instanceof Blob ? content : new Blob([content], { type });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = name;
    document.body.appendChild(a);
    a.click();
    a.remove();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }

  exportCsv(): string {
    const lines = ['point_id,point_name,link,pose_index,driver_value,x,y,z'];
    const src = this.sim.sweep?.poses ?? [];
    for (const id of this.model.settings.displayPointIds) {
      const pt = this.model.points[id];
      if (!pt) continue;
      src.forEach((pose, k) => {
        const p = pose.positions.get(id);
        if (p) lines.push([id, pt.name, this.model.links[pt.linkId]?.name ?? '', k, pose.value.toFixed(4), p[0].toFixed(6), p[1].toFixed(6), p[2].toFixed(6)].join(','));
      });
    }
    return lines.join('\n');
  }

  exportObj(): string {
    const exporter = new OBJExporter();
    return exporter.parse(this.viewport.groups.model);
  }

  loadFromJson(text: string, fileName: string): void {
    const m = parseModel(text);
    this.loadModel(m, fileName);
  }

  cloneCurrentModel(): Model {
    return cloneModel(this.model);
  }
}

export type { Vec3 };
